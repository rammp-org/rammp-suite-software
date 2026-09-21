# Foxglove bridge — a WebSocket server (port 8765) that lets Foxglove see the
# ROS graph. It is on rammp-base for the same reason the cameras are: only a
# Cyclone participant with the fleet config can see the fleet's topics.
FROM ghcr.io/rammp-org/rammp-base:1.0.0-jp6

RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-humble-foxglove-bridge \
 && rm -rf /var/lib/apt/lists/*
