# Intel RealSense D405 wrist camera driver.
FROM ghcr.io/rammp-org/rammp-base:1.0.0-jp6

RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-humble-realsense2-camera \
 && rm -rf /var/lib/apt/lists/*
