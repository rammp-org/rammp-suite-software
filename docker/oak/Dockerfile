# Luxonis OAK-D Pro (PoE) driver — an alternative wrist camera to the D405.
FROM ghcr.io/rammp-org/rammp-base:1.0.0-jp6

RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-humble-depthai-ros-v3 \
 && rm -rf /var/lib/apt/lists/*

COPY wrist.yaml /etc/rammp/oak/wrist.yaml
