import os
os.environ["MAVLINK20"] = "1"

import argparse
from collections import deque
import math
import struct
import sys
import threading
import time

from pymavlink import mavutil
import serial

from crc_utils import crc_table


class LIDAR:
    def __init__(self, serial_port, baudrate):
        self.PACKET_LENGTH = 49
        self.POINT_PER_PACK = 12
        self.serial_conn = serial.Serial(serial_port, baudrate=baudrate, timeout=1)

    def calculate_crc8(self, data):
        crc = 0x00
        for byte in data:
            crc = crc_table[(crc ^ byte) & 0xFF]
        return crc

    def parse_packet(self, packet):
        if len(packet) != self.PACKET_LENGTH:
            return None

        if packet[0] != 0x54 or packet[1] != 0x2C:
            return None

        received_crc = packet[self.PACKET_LENGTH - 3]
        calculated_crc = self.calculate_crc8(packet[: self.PACKET_LENGTH - 3])
        if received_crc != calculated_crc:
            return None

        _, _, speed, start_angle = struct.unpack("<BBHH", packet[:6])

        points = []
        offset = 6
        for _ in range(self.POINT_PER_PACK):
            distance, intensity = struct.unpack("<HB", packet[offset:offset + 3])
            points.append({"distance": distance, "intensity": intensity})
            offset += 3

        end_angle, timestamp = struct.unpack("<HH", packet[42:46])

        start_angle = (start_angle % 36000) / 100.0
        end_angle = (end_angle % 36000) / 100.0

        angle_diff = (end_angle - start_angle + 360.0) % 360.0
        angle_increment = angle_diff / 11
        angles = [
            (start_angle + i * angle_increment) % 360.0
            for i in range(self.POINT_PER_PACK)
        ]

        scan_data = []
        for i, point in enumerate(points):
            scan_data.append({
                "angle": angles[i],
                "distance": point["distance"],
                "intensity": point["intensity"],
            })

        return {
            "speed": speed,
            "start_angle": start_angle,
            "end_angle": end_angle,
            "timestamp": timestamp,
            "scan_data": scan_data,
        }

    def read_lidar_data(self):
        while True:
            first = self.serial_conn.read(1)
            if not first:
                return None
            if first[0] != 0x54:
                continue
            second = self.serial_conn.read(1)
            if not second:
                return None
            if second[0] != 0x2C:
                continue

            payload = self.serial_conn.read(self.PACKET_LENGTH - 2)
            if len(payload) != (self.PACKET_LENGTH - 2):
                return None
            packet = bytes([0x54, 0x2C]) + payload
            data = self.parse_packet(packet)
            if data:
                return data

    def close_serial_connection(self):
        self.serial_conn.close()


def progress(message):
    print(message, file=sys.stdout)
    sys.stdout.flush()


def angle_to_index(angle_deg, start_deg, increment_deg, bucket_count):
    # deal with wrap-around since North is 0 degrees, so start is always > end
    delta = (angle_deg - start_deg + 360.0) % 360.0
    idx = int(delta / increment_deg)
    if idx < 0:
        return 0
    if idx >= bucket_count:
        return bucket_count - 1
    return idx


def mavlink_loop(conn, callbacks, should_exit_flag):
    while not should_exit_flag[0]:
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


def send_obstacle_distance_message(conn, distances_cm, min_cm, max_cm, angle_offset, increment_f):
    timestamp_us = int(round(time.time() * 1000000))
    conn.mav.obstacle_distance_send(
        timestamp_us,
        0,
        distances_cm,
        0,
        min_cm,
        max_cm,
        increment_f,
        angle_offset,
        12,
    )


def start_debug_plotter(debug_state, max_range_mm):
    import matplotlib.animation as animation
    import matplotlib.pyplot as plt

    angles = debug_state["angles"]
    distances = debug_state["distances"]
    intensities = debug_state["intensities"]
    packet_count = debug_state["packet_count"]
    zoom_scale = debug_state["zoom_scale"]
    lock = debug_state["lock"]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_title("Lidar Distance Colormap")

    scatter = ax.scatter([], [], c=[], s=18, cmap="viridis", vmin=0, vmax=255)
    colorbar = fig.colorbar(scatter, ax=ax, pad=0.1)
    colorbar.set_label("Intensity")
    status = ax.text(
        0.02,
        0.02,
        "Packets: 0 | Points: 0",
        transform=ax.transAxes,
        ha="left",
        va="bottom",
    )

    def update(_):
        with lock:
            local_angles = list(angles)
            local_distances = list(distances)
            local_intensities = list(intensities)
            local_packet_count = packet_count[0]
            local_zoom = zoom_scale[0]

        if not local_angles:
            return scatter,

        scatter.set_offsets(list(zip(local_angles, local_distances)))
        scatter.set_array(local_intensities)
        ax.set_ylim(0, max_range_mm * local_zoom)
        status.set_text(
            f"Packets: {local_packet_count} | Points: {len(local_angles)} | Zoom: {local_zoom:.2f}x"
        )
        return scatter, status

    def handle_close(_):
        debug_state["plot_should_exit"][0] = True

    def handle_key(event):
        if event.key in {"+", "="}:
            zoom_scale[0] = max(0.1, zoom_scale[0] * 0.8)
        elif event.key == "-":
            zoom_scale[0] = min(5.0, zoom_scale[0] / 0.8)
        elif event.key in {"0", "r"}:
            zoom_scale[0] = 1.0

    fig.canvas.mpl_connect("close_event", handle_close)
    fig.canvas.mpl_connect("key_press_event", handle_key)
    anim = animation.FuncAnimation(
        fig, update, interval=50, blit=False, cache_frame_data=False
    )
    plt.show()
    return anim


def lidar_loop(
    args,
    conn,
    lidar,
    should_exit_flag,
    debug_state,
    min_depth_cm,
    max_depth_cm,
    sector_start,
    sector_end,
    bucket_count,
    increment_f,
    angle_offset,
):
    last_send = 0.0
    distances = [65535] * bucket_count
    last_end_angle = 0
    rotations_since_reset = 0
    while not should_exit_flag[0]:
        data = lidar.read_lidar_data()
        # reset distances after a full rotation to clear stale data
        if last_end_angle > 340 and data["start_angle"] < 20:
            rotations_since_reset += 1
            if rotations_since_reset == 5:
                rotations_since_reset = 0
                distances = [65535] * bucket_count
        last_end_angle = data["end_angle"]
        if not data:
            continue

        if debug_state is not None:
            with debug_state["lock"]:
                debug_state["packet_count"][0] += 1
                for point in data["scan_data"]:
                    debug_state["angles"].append(math.radians(point["angle"]))
                    debug_state["distances"].append(point["distance"])
                    debug_state["intensities"].append(point["intensity"])

        for point in data["scan_data"]:
            angle = point["angle"]
            if not (angle >= sector_start or angle <= sector_end):
                continue

            distance_cm = int(point["distance"] / 10)
            if distance_cm < min_depth_cm or distance_cm > max_depth_cm:
                continue

            idx = angle_to_index(angle, sector_start, increment_f, bucket_count)
            distances[idx] = distance_cm

        now = time.time()
        if now - last_send >= (1.0 / args.hz):
            send_obstacle_distance_message(
                conn,
                distances,
                min_depth_cm,
                max_depth_cm,
                angle_offset,
                increment_f,
            )
            last_send = now


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Send LD Lidar data to MAVLink")
    parser.add_argument("--connect", default="udpout:192.168.2.2:14569")
    parser.add_argument("--port", default="/dev/tty.usbserial-0001")
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument("--hz", type=float, default=15.0)
    parser.add_argument("--min_range_m", type=float, default=0.1)
    parser.add_argument("--max_range_m", type=float, default=10.0)
    parser.add_argument("--debug_plot", action="store_true", help="Show live colormap plot")
    args = parser.parse_args()

    min_depth_cm = int(args.min_range_m * 100)
    max_depth_cm = int(args.max_range_m * 100)

    # Sector from 315 deg through 360 to 45 deg (front arc)
    sector_start = 270.0
    sector_end = 90.0
    bucket_count = 72
    sector_span = 360.0 - sector_start + sector_end
    increment_f = sector_span / bucket_count
    angle_offset = sector_start

    distances = [65535] * bucket_count

    progress("INFO: Starting Vehicle communications")
    conn = mavutil.mavlink_connection(
        args.connect,
        autoreconnect=True,
        source_system=1,
        source_component=1,
    )

    should_exit_flag = [False]
    mavlink_thread = threading.Thread(
        target=mavlink_loop,
        args=(conn, {}, should_exit_flag),
        daemon=True,
    )
    mavlink_thread.start()

    lidar = LIDAR(serial_port=args.port, baudrate=args.baud)

    debug_state = None
    if args.debug_plot:
        debug_state = {
            "angles": deque(maxlen=360),
            "distances": deque(maxlen=360),
            "intensities": deque(maxlen=360),
            "packet_count": [0],
            "zoom_scale": [1.0],
            "lock": threading.Lock(),
            "plot_should_exit": [False],
        }

    lidar_thread = None
    try:
        if args.debug_plot:
            lidar_thread = threading.Thread(
                target=lidar_loop,
                args=(
                    args,
                    conn,
                    lidar,
                    should_exit_flag,
                    debug_state,
                    min_depth_cm,
                    max_depth_cm,
                    sector_start,
                    sector_end,
                    bucket_count,
                    increment_f,
                    angle_offset,
                ),
                daemon=True,
            )
            lidar_thread.start()
            start_debug_plotter(debug_state, args.max_range_m * 1000)
        else:
            lidar_loop(
                args,
                conn,
                lidar,
                should_exit_flag,
                debug_state,
                min_depth_cm,
                max_depth_cm,
                sector_start,
                sector_end,
                bucket_count,
                increment_f,
                angle_offset,
            )

    except KeyboardInterrupt:
        progress("INFO: Shutting down")

    finally:
        should_exit_flag[0] = True
        if lidar_thread is not None:
            lidar_thread.join(timeout=2.0)
        lidar.close_serial_connection()
        conn.close()
