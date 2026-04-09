# docker buildx build --platform linux/arm/v7 . -t jonathan735/blueboat-oakd:latest --output type=registry

FROM python:3.10-slim

# install opencv and gstreamer dependencies
RUN apt update && apt install -y build-essential cmake git libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev \
libgirepository-2.0-dev libcairo2-dev gir1.2-gtk-4.0 gstreamer1.0-plugins-base gir1.2-gst-rtsp-server-1.0

COPY src /src

RUN pip install -r /src/requirements.txt

ENTRYPOINT ["python3", "/src/oak_to_mavlink.py"]

LABEL version="1.0.0"
LABEL permissions='\
{\
   "NetworkMode":"host",\
   "HostConfig":{\
      "Privileged":true,\
      "NetworkMode":"host",\
      "Binds":[\
         "/dev/bus/usb:/dev/bus/usb"\
      ],\
      "DeviceCgroupRules":[\
         "c 189:* rmw"\
      ]\
   }\
}'

LABEL authors='[\
    {\
        "name": "Jonathan Ma",\
        "email": ""\
    }\
]'
LABEL company='{\
        "about": "",\
        "name": "Cornell CEI Lab",\
        "email": ""\
    }'
LABEL type="device-integration"
LABEL readme=''
LABEL links='{\
        "website": "",\
        "support": ""\
    }'
LABEL requirements="core >= 1.1"