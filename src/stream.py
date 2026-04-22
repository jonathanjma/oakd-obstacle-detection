import requests
import pprint
import socket
import numpy as np
import threading

import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst, GstRtspServer, GLib

streams =  {
    "depth": "Oak-D Stereo Depth",
    "rgb": "Oak-D RGB"
}
mcm_endpoint = "http://127.0.0.1:6020/streams"

RTSP_PORT = "8554"
glib_loop = None
glib_thread = None

######################################################
##  Functions - RTSP Streaming                      ##
######################################################

class SensorFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, kind, **properties):
        super(SensorFactory, self).__init__(**properties)
        self.kind = kind
        if self.kind == "rgb":
            self.launch_string = (
                "appsrc name=source is-live=true block=true do-timestamp=true format=GST_FORMAT_TIME "
                "caps=video/x-h264,stream-format=byte-stream,alignment=au,profile=main "
                "! h264parse ! rtph264pay config-interval=1 name=pay0 pt=96"
            )
        else:
            from oak_to_mavlink import DEPTH_WIDTH, DEPTH_HEIGHT, FPS
            self.launch_string = (
                "appsrc name=source is-live=true block=true do-timestamp=true format=GST_FORMAT_TIME "
                f"caps=video/x-raw,format=BGR,width={DEPTH_WIDTH},height={DEPTH_HEIGHT},framerate={FPS}/1 "
                "! videoconvert "
                "! video/x-raw,format=I420 "
                "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 key-int-max=30 "
                "! rtph264pay config-interval=1 name=pay0 pt=96"
            )
        self.set_launch(self.launch_string)

    def do_configure(self, rtsp_media):
        self.appsrc = rtsp_media.get_element().get_child_by_name("source")


class GstServer(GstRtspServer.RTSPServer):
    def __init__(self, **properties):
        super(GstServer, self).__init__(**properties)
        self.set_service(RTSP_PORT)
        self.rgb_factory = SensorFactory("rgb")
        self.depth_factory = SensorFactory("depth")
        self.rgb_factory.set_shared(True)
        self.depth_factory.set_shared(True)
        self.get_mount_points().add_factory("/rgb", self.rgb_factory)
        self.get_mount_points().add_factory("/depth", self.depth_factory)
        self.attach(None)

    def send_data(self, kind, data):
        factory = self.rgb_factory if kind == "rgb" else self.depth_factory
        if hasattr(factory, "appsrc"):
            if kind == "depth":
                frame = np.ascontiguousarray(data)
                payload = frame.tobytes()
            else:
                payload = bytes(data)
            buf = Gst.Buffer.new_wrapped(payload)
            factory.appsrc.emit("push-buffer", buf)


def get_local_ip():
    local_ip_address = "127.0.0.1"
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 1)) # connect() for UDP doesn't send packets
        local_ip_address = s.getsockname()[0]
    except Exception:
        local_ip_address = socket.gethostbyname(socket.gethostname())
    return local_ip_address


def has_oak_stream(current_streams, name):
    for stream in current_streams:
        if stream["video_and_stream"]["name"] == name:
            return True
    return False

def add_mcm_stream(endpoint):
    name = streams[endpoint]
    new_stream = {
        "name": name,
        "source": "Redirect",
        "stream_information": {
            "endpoints": [
                f"rtsp://127.0.0.1:{RTSP_PORT}/{endpoint}"
            ],
            "configuration": {
                "type": "redirect"
            },
            "extended_configuration": {
                "thermal": False,
                "disable_mavlink": True
            }
        }
    }
    print(f"adding stream {endpoint}")
    response = requests.post(mcm_endpoint, json=new_stream)
    pprint.pprint(response.text)
    response.raise_for_status()

def register_streams():
    try:
        current_streams = requests.get(mcm_endpoint).json()
    except Exception as error:
        print(f"Failed to fetch current MCM streams: {error}")
        return False

    ok = True
    for endpoint, name in streams.items():
        try:
            if has_oak_stream(current_streams, name):
                print(f"{endpoint} stream already exists in MCM")
                continue

            add_mcm_stream(endpoint)
            print(f"added {endpoint} stream to MCM")
        except Exception as error:
            print(f"Failed to register {endpoint}: {error}")
            ok = False

    return ok

def rtsp_init():
    # if not register_streams():
    #     send_msg_to_gcs("ERROR: Failed to register RTSP streams in MCM")
    #     progress("ERROR: Failed to register RTSP streams in MCM")

    msg = "RTSP at rtsp://" + get_local_ip() + ":" + RTSP_PORT + "/rgb and /depth"
    Gst.init(None)
    rtsp_server = GstServer()
    glib_loop = GLib.MainLoop()
    glib_thread = threading.Thread(target=glib_loop.run, args=())
    glib_thread.start()
    return rtsp_server, msg

def rtsp_exit():
    if glib_loop is not None:
        glib_loop.quit()
    if glib_thread is not None:
        glib_thread.join()