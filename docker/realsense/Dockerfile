# Intel RealSense D405 wrist camera driver.
FROM ros:humble-ros-base

RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-humble-realsense2-camera \
 && rm -rf /var/lib/apt/lists/*
