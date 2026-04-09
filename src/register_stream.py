import requests
import pprint

streams =  {
    "oak/depth": "Oak-D Stereo Disparity",
    "oak/rgb": "Oak-D RGB"
}
mcm_endpoint = "http://192.168.2.2:6020/streams"

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
            f"rtsp://192.168.2.1:8554/{endpoint}"
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