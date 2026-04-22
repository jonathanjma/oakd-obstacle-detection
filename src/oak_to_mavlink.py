# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"

import argparse
import math as m
import signal
import sys
import threading
import time

from apscheduler.schedulers.background import BackgroundScheduler
from pymavlink import mavutil
import cv2
import depthai as dai
import numpy as np

from stream import rtsp_init, rtsp_exit
# export DYLD_LIBRARY_PATH="/opt/homebrew/lib:${DYLD_LIBRARY_PATH}"

######################################################
##  Depth parameters - reconfigurable               ##
######################################################

DEPTH_WIDTH = 640
DEPTH_HEIGHT = 360
FPS = 30
DEPTH_RANGE_M = [0.1, 10.0]

obstacle_line_height_ratio = 0.5  # [0-1]: 0-Top, 1-Bottom. The height of the horizontal line to find distance to obstacle.
obstacle_line_thickness_pixel = 15 # [1-DEPTH_HEIGHT]: Number of pixel rows to use to generate the obstacle distance message. For each column, the scan will return the minimum value for those pixels centered vertically in the image.

# Sanity check for depth configuration
assert obstacle_line_height_ratio >= 0 and obstacle_line_height_ratio <= 1
assert obstacle_line_thickness_pixel >= 1 and obstacle_line_thickness_pixel <= DEPTH_HEIGHT

RTSP_STREAMING_ENABLE = True

######################################################
##  ArduPilot-related parameters - reconfigurable   ##
######################################################

# Default configurations for connection to the FCU
connection_string_default = "udpout:192.168.2.2:14569" # boat
# connection_string_default = "tcp:127.0.0.1:5762" # sitl

# Use this to rotate all processed data
camera_facing_angle_degree = 0

# Enable/disable each message/function individually
enable_msg_obstacle_distance = True
enable_msg_distance_sensor = False
obstacle_distance_msg_hz_default = 15.0

mavlink_thread_should_exit = False

debug_enable_default = 1

######################################################
##  Global variables                                ##
######################################################

pipeline: dai.Pipeline = None
oak_device: dai.Device = None
calibration_handler: dai.CalibrationHandler = None

# The name of the display window
display_name = "Input/output depth"
rtsp_server = None
hover_depth_m = 0.0

# Mouse callback function
def on_mouse(event, x, y, flags, param):
    global hover_depth_m
    if event == cv2.EVENT_MOUSEMOVE:
        depth_m_frame = param
        if y < depth_m_frame.shape[0] and x < depth_m_frame.shape[1]:
            hover_depth_m = depth_m_frame[y, x]

# Data variables
vehicle_pitch_rad = None
current_time_us = 0
last_obstacle_distance_sent_ms = 0  # value of current_time_us when obstacle_distance last sent

# Obstacle distances in front of the sensor, starting from the left in increment degrees to the right
# See here: https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
min_depth_cm = int(DEPTH_RANGE_M[0] * 100)  # In cm
max_depth_cm = int(DEPTH_RANGE_M[1] * 100)  # In cm, should be a little conservative
distances_array_length = 72
angle_offset = None
increment_f = None
depth_hfov_deg = None
depth_vfov_deg = None
distances = np.ones((distances_array_length,), dtype=np.uint16) * (max_depth_cm + 1)

######################################################
##  Parsing user inputs                             ##
######################################################

parser = argparse.ArgumentParser(description="Send OAK depth to MAVLink")
parser.add_argument("--connect",
    help="Vehicle connection target string. If not specified, a default string will be used.",
)
parser.add_argument("--obstacle_distance_msg_hz", type=float,
    help="Update frequency for OBSTACLE_DISTANCE message. If not specified, a default value will be used.",
)
parser.add_argument("--debug_enable", type=float, 
    help="Enable debugging information")

args = parser.parse_args()

connection_string = args.connect
obstacle_distance_msg_hz = args.obstacle_distance_msg_hz
debug_enable = args.debug_enable


def progress(string):
    print(string, file=sys.stdout)
    sys.stdout.flush()

# Using default values if no specified inputs
if not connection_string:
    connection_string = connection_string_default
    progress("INFO: Using default connection_string %s" % connection_string)
else:
    progress("INFO: Using connection_string %s" % connection_string)

if not obstacle_distance_msg_hz:
    obstacle_distance_msg_hz = obstacle_distance_msg_hz_default
    progress("INFO: Using default obstacle_distance_msg_hz %s" % obstacle_distance_msg_hz)
else:
    progress("INFO: Using obstacle_distance_msg_hz %s" % obstacle_distance_msg_hz)

if not debug_enable:
    debug_enable = debug_enable_default

if debug_enable == 1:
    progress("INFO: Debugging option enabled")
    cv2.namedWindow(display_name, cv2.WINDOW_AUTOSIZE)
else:
    progress("INFO: Debugging option DISABLED")

######################################################
##  Functions - MAVLink                             ##
######################################################


def mavlink_loop(conn, callbacks):
    """Main routine for MAVLink thread."""
    while not mavlink_thread_should_exit:
        # Send a heartbeat message
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            0,
            0,
            0,
        )
        msg = conn.recv_match(type=list(callbacks.keys()), timeout=1, blocking=True)
        if msg is None:
            continue
        callbacks[msg.get_type()](msg)


# https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
def send_obstacle_distance_message():
    global current_time_us, distances
    global last_obstacle_distance_sent_ms

    if current_time_us == last_obstacle_distance_sent_ms:
        # no new frame
        return

    last_obstacle_distance_sent_ms = current_time_us
    if angle_offset is None or increment_f is None:
        progress("Please call set_obstacle_distance_params before continue")
    else:
        conn.mav.obstacle_distance_send(
            current_time_us,    # us Timestamp (UNIX time or time since system boot)
            0,                  # sensor_type, defined here: https://mavlink.io/en/messages/common.html#MAV_DISTANCE_SENSOR
            distances,          # distances,    uint16_t[72],   cm
            0,                  # increment,    uint8_t,        deg
            min_depth_cm,	    # min_distance, uint16_t,       cm
            max_depth_cm,       # max_distance, uint16_t,       cm
            increment_f,	    # increment_f,  float,          deg
            angle_offset,       # angle_offset, float,          deg
            12,                 # MAV_FRAME, vehicle-front aligned: https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD    
        )

# https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR
def send_distance_sensor_message():
    # Average out a portion of the centermost part
    curr_dist = int(np.mean(distances[33:38]))
    conn.mav.distance_sensor_send(
        0,              # ms Timestamp (UNIX time or time since system boot) (ignored)
        min_depth_cm,   # min_distance, uint16_t, cm
        max_depth_cm,   # min_distance, uint16_t, cm
        curr_dist,      # current_distance,	uint16_t, cm	
        0,	            # type : 0 (ignored)
        0,              # id : 0 (ignored)
        int(camera_facing_angle_degree / 45),              # orientation
        0,              # covariance : 0 (ignored)
    )


def send_msg_to_gcs(text_to_be_sent):
    # MAV_SEVERITY: 0=EMERGENCY 1=ALERT 2=CRITICAL 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
    text_msg = 'OAK: ' + text_to_be_sent
    conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())
    progress("INFO: %s" % text_to_be_sent)


# Listen to ATTITUDE data: https://mavlink.io/en/messages/common.html#ATTITUDE
def att_msg_callback(value):
    global vehicle_pitch_rad
    vehicle_pitch_rad = value.pitch
    # if debug_enable == 1:
    #     progress(
    #         "INFO: Received ATTITUDE msg, current pitch is %.2f degrees"
    #         % (m.degrees(vehicle_pitch_rad),)
    #     )


######################################################
##  Functions - OAK camera                          ##
######################################################


def oak_build_pipeline():
    p = dai.Pipeline()

    # Depth Stream (DepthAI v2 API)
    mono_left = p.createMonoCamera()
    mono_right = p.createMonoCamera()
    stereo = p.createStereoDepth()
    xout_depth = p.createXLinkOut()

    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    mono_left.setFps(FPS)
    mono_right.setFps(FPS)

    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)
    stereo.depth.link(xout_depth.input)
    xout_depth.setStreamName("depth")

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
    stereo.initialConfig.PostProcessing.ThresholdFilter.minRange = int(DEPTH_RANGE_M[0] * 1000)
    stereo.initialConfig.PostProcessing.ThresholdFilter.maxRange = int(DEPTH_RANGE_M[1] * 1000)

    stereo.initialConfig.setConfidenceThreshold(255-50) # threshold is reversed, so 0 is highest confidence
    stereo.initialConfig.PostProcessing.TemporalFilter.enable = True
    stereo.initialConfig.PostProcessing.TemporalFilter.alpha = 0.65
    stereo.initialConfig.PostProcessing.TemporalFilter.PersistencyMode = dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_3

    stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
    stereo.initialConfig.PostProcessing.SpeckleFilter.enable = True
    stereo.initialConfig.PostProcessing.SpeckleFilter.speckleRange = 200
    stereo.initialConfig.PostProcessing.SpatialFilter.enable = False

    # RGB Stream Encoder (DepthAI v2 API)
    rgb_cam = p.createColorCamera()
    rgb_cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    rgb_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    rgb_cam.setFps(FPS)

    rgb_enc = p.createVideoEncoder()
    rgb_enc.setDefaultProfilePreset(FPS, dai.VideoEncoderProperties.Profile.H264_HIGH)

    xout_rgb = p.createXLinkOut()
    xout_rgb.setStreamName("rgb_encoded")

    rgb_cam.video.link(rgb_enc.input)
    rgb_enc.bitstream.link(xout_rgb.input)

    return p


def set_obstacle_distance_params():
    global angle_offset, increment_f, depth_hfov_deg, depth_vfov_deg

    if calibration_handler is None:
        raise RuntimeError("Calibration handler not initialized")

    intrinsics = calibration_handler.getCameraIntrinsics(
        dai.CameraBoardSocket.CAM_B, DEPTH_WIDTH*2, DEPTH_HEIGHT*2
    )
    print(intrinsics)
    fx = intrinsics[0][0]
    fy = intrinsics[1][1]

    # For forward facing camera with a horizontal wide view:
    #   HFOV=2*atan[w/(2*fx)],
    #   VFOV=2*atan[h/(2*fy)],
    #   DFOV=2*atan(Diag/2*f),
    #   Diag=sqrt(w^2 + h^2)
    depth_hfov_deg = m.degrees(2 * m.atan((DEPTH_WIDTH*2) / (2 * fx)))
    depth_vfov_deg = m.degrees(2 * m.atan((DEPTH_HEIGHT*2) / (2 * fy)))

    progress("INFO: Depth camera HFOV: %0.2f degrees" % depth_hfov_deg)
    progress("INFO: Depth camera VFOV: %0.2f degrees" % depth_vfov_deg)

    angle_offset = camera_facing_angle_degree - (depth_hfov_deg / 2)
    increment_f = depth_hfov_deg / distances_array_length

    progress("INFO: OBSTACLE_DISTANCE angle_offset: %0.3f" % angle_offset)
    progress("INFO: OBSTACLE_DISTANCE increment_f: %0.3f" % increment_f)
    progress("INFO: OBSTACLE_DISTANCE coverage: from %0.3f to %0.3f degrees"
        % (angle_offset, angle_offset + increment_f * distances_array_length)
    )


# Find the height of the horizontal line to calculate the obstacle distances
#   - Basis: depth camera's vertical FOV, user's input
#   - Compensation: vehicle's current pitch angle
def find_obstacle_line_height(frame_height):
    # Basic position
    obstacle_line_height = frame_height * obstacle_line_height_ratio

    # Compensate for the vehicle's pitch angle if data is available
    if vehicle_pitch_rad is not None and depth_vfov_deg is not None:
        delta_height = (
            m.sin(vehicle_pitch_rad / 2) / m.sin(m.radians(depth_vfov_deg) / 2) * frame_height
        )
        obstacle_line_height += delta_height

    # Sanity check
    if obstacle_line_height < 0:
        obstacle_line_height = 0
    elif obstacle_line_height > frame_height:
        obstacle_line_height = frame_height
    return obstacle_line_height


# Calculate the distances array by dividing the FOV (horizontal) into $distances_array_length rays,
# then pick out the depth value at the pixel corresponding to each ray. Based on the definition of
# the MAVLink messages, the invalid distance value (below MIN/above MAX) will be replaced with MAX+1.
#    
# [0]    [35]   [71]    <- Output: distances[72]
#  |      |      |      <- step = width / 72
#  ---------------      <- horizontal line, or height/2
#  \      |      /
#   \     |     /
#    \    |    /
#     \   |   /
#      \  |  /
#       \ | /           
#       Camera          <- Input: depth_mat, obtained from depth image
#
# Note that we assume the input depth_mat is already processed by at least hole-filling filter.
# Otherwise, the output array might not be stable from frame to frame.
def distances_from_depth_image(
    obstacle_line_height,
    depth_mat_m,
    distances,
    min_depth_m,
    max_depth_m,
    obstacle_line_thickness_pixel,
):
    depth_img_width = depth_mat_m.shape[1]
    depth_img_height = depth_mat_m.shape[0]
    step = depth_img_width / distances_array_length

    for i in range(distances_array_length):
        # Each range (left to right) is found from a set of rows within a column
        #  [ ] -> ignored
        #  [x] -> center + obstacle_line_thickness_pixel / 2
        #  [x] -> center = obstacle_line_height (moving up and down according to the vehicle's pitch angle)
        #  [x] -> center - obstacle_line_thickness_pixel / 2
        #  [ ] -> ignored
        #   ^ One of [distances_array_length] number of columns, from left to right in the image
        center_pixel = obstacle_line_height
        upper_pixel = center_pixel + obstacle_line_thickness_pixel / 2
        lower_pixel = center_pixel - obstacle_line_thickness_pixel / 2

        # Sanity checks
        if upper_pixel > depth_img_height:
            upper_pixel = depth_img_height
        elif upper_pixel < 1:
            upper_pixel = 1
        if lower_pixel > depth_img_height:
            lower_pixel = depth_img_height - 1
        elif lower_pixel < 0:
            lower_pixel = 0

        # Find min distance in the vertical line for each column
        min_point_in_scan = np.min(
            depth_mat_m[int(lower_pixel): int(upper_pixel), int(i * step)]
        )
        dist_m = float(min_point_in_scan)

        # Distances array values: 
        #   A value of max_distance + 1 (cm) means no obstacle is present. 
        #   A value of UINT16_MAX (65535) for unknown/not used.

        # Note that dist_m is in meter, while distances[] is in cm.
        if dist_m > min_depth_m and dist_m < max_depth_m:
            distances[i] = int(dist_m * 100)
        else:
            distances[i] = 65535

######################################################
##  Main code starts here                           ##
######################################################

progress("INFO: Starting Vehicle communications")
conn = mavutil.mavlink_connection(
    connection_string,
    autoreconnect=True,
    source_system=1,   # needed for cockpit to see the messages
    source_component=1 # needed for cockpit to see the messages
)
mavlink_callbacks = {
    "ATTITUDE": att_msg_callback,
}
mavlink_thread = threading.Thread(target=mavlink_loop, args=(conn, mavlink_callbacks))
mavlink_thread.start()

send_msg_to_gcs("Connecting to OAK camera...")

# Build pipeline and start device using DepthAI v2 API.
pipeline = oak_build_pipeline()
oak_device = dai.Device()
calibration_handler = oak_device.readCalibration()
oak_device.startPipeline(pipeline)

raw_depth_queue = oak_device.getOutputQueue(name="depth")
rgb_queue = oak_device.getOutputQueue(name="rgb_encoded")

send_msg_to_gcs("OAK camera connected.")

set_obstacle_distance_params()

# Send MAVlink messages in the background at pre-determined frequencies
sched = BackgroundScheduler()

if enable_msg_obstacle_distance:
    sched.add_job(send_obstacle_distance_message, "interval", seconds=1 / obstacle_distance_msg_hz)
    send_msg_to_gcs("Sending obstacle distance messages to FCU")
elif enable_msg_distance_sensor:
    sched.add_job(send_distance_sensor_message, "interval", seconds=1 / obstacle_distance_msg_hz)
    send_msg_to_gcs("Sending distance sensor messages to FCU")
else:
    send_msg_to_gcs("Nothing to do. Check params to enable something")
    conn.mav.close()
    progress("INFO: Vehicle object closed.")
    sys.exit()

if RTSP_STREAMING_ENABLE is True:
    rtsp_server, msg = rtsp_init()
    send_msg_to_gcs(msg)
else:
    send_msg_to_gcs("RTSP not streaming")

sched.start()

# Gracefully terminate the script if an interrupt signal (e.g. ctrl-c) is received.
def sigint_handler(sig, frame):
    global main_loop_should_quit
    main_loop_should_quit = True

signal.signal(signal.SIGINT, sigint_handler)
signal.signal(signal.SIGTERM, sigint_handler)

main_loop_should_quit = False

# Begin of the main loop
last_time = time.time()
try:
    while not main_loop_should_quit:
        depth_raw_frame = raw_depth_queue.get()
        if depth_raw_frame is None:
            continue

        # Store the timestamp for MAVLink messages
        current_time_us = int(round(time.time() * 1000000))

        # OAK depth output is millimeters
        depth_m = depth_raw_frame.getFrame().astype(np.float32) / 1000.0

        obstacle_line_height = find_obstacle_line_height(depth_m.shape[0])
        distances_from_depth_image(
            obstacle_line_height,
            depth_m,
            distances,
            DEPTH_RANGE_M[0],
            DEPTH_RANGE_M[1],
            obstacle_line_thickness_pixel,
        )

        # Use depth colormap as debug image source
        depth_u8 = np.clip(depth_m / DEPTH_RANGE_M[1] * 255, 0, 255).astype(np.uint8)
        depth_image = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
        depth_image[depth_m == 0] = 0 # set invalid depth to black

        # Handle Encoded Data for RTSP
        if RTSP_STREAMING_ENABLE and rtsp_server:
            rtsp_server.send_data('depth', depth_image)
            
            # while rgb_queue.has():
            r_pkt = rgb_queue.get().getData()
            rtsp_server.send_data('rgb', r_pkt)

        if debug_enable == 1:
            # Show depth at current mouse position
            cv2.setMouseCallback(display_name, on_mouse, param=depth_m)
            cv2.putText(
                depth_image,
                f"Depth at mouse: {hover_depth_m:.2f}m",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2,
            )

            # Draw a horizontal line to visualize the obstacles' line
            x1, y1 = int(0), int(obstacle_line_height)
            x2, y2 = int(DEPTH_WIDTH), int(obstacle_line_height)
            cv2.line(depth_image, (x1, y1), (x2, y2), (0, 255, 0), thickness=obstacle_line_thickness_pixel)

            cv2.imshow(display_name, depth_image)
            cv2.waitKey(1)

            # Print all the distances in a line
            # progress("%s" % (str(distances)))
            last_time = time.time()

except Exception as e:
    progress(e)
    send_msg_to_gcs('ERROR: Depth camera disconnected')  

finally:
    progress("Closing the script...")
    if RTSP_STREAMING_ENABLE is True:
        rtsp_exit()
    if oak_device is not None:
        oak_device.close()
    mavlink_thread_should_exit = True
    mavlink_thread.join()
    conn.close()
    sched.shutdown()
    progress("INFO: OAK device and vehicle object closed.")
