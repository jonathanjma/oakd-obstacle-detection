# docker buildx build --platform linux/arm/v7 . -t jonathan735/blueboat-oakd:latest --output type=registry
# docker run -it --entrypoint /bin/bash jonathan735/blueboat-oakd:latest
# docker run --privileged --network host -v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw' jonathan735/blueboat-oakd:latest

FROM luxonis/depthai-library:v2.32.0.0-armv7

# Base image uses /usr/local Python; include distro dist-packages so python3-gi is importable.
ENV PYTHONPATH=/usr/lib/python3/dist-packages

# install opencv and gstreamer dependencies
RUN apt update && apt install -y libopenblas0 \
    python3-gi \
    python3-gst-1.0 \
    gir1.2-gstreamer-1.0 \
    gir1.2-gst-rtsp-server-1.0 \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    libcairo2-dev \
    libgirepository1.0-dev

COPY src /src

RUN pip install --extra-index-url https://www.piwheels.org/simple/ --prefer-binary -r /src/requirements.txt

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