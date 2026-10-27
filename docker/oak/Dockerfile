# syntax=docker/dockerfile:1
# Luxonis OAK driver (DepthAI ROS v3): OAK-D Pro PoE, OAK-D Pro W PoE, and the
# rest of the family. Which one, and how it is tuned, comes from the deployment.
#
# The driver itself is built from our depthai-ros fork: DEPTHAI_ROS_REF is the
# 3.3.0 Humble release plus the fixes we are upstreaming (rammp-suite-software#8).
# Everything else, depthai-core included, is the apt release that fork is
# based on. The pin fails the build once apt moves past 3.3.0, rather than
# building the fork against a newer bridge and core than it was tested with.
ARG DEPTHAI_ROS_VERSION=3.3.0

FROM ghcr.io/rammp-org/rammp-base:1.0.0-jp6 AS base
ARG DEPTHAI_ROS_VERSION
# The stock driver is installed only for its dependencies, then removed so
# the fork's build is the only depthai_ros_driver_v3 on the ament index.
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
    ros-humble-depthai-ros-driver-v3=${DEPTHAI_ROS_VERSION}-* \
    ros-humble-ffmpeg-image-transport \
 && dpkg --remove ros-humble-depthai-ros-driver-v3 \
 && rm -rf /var/lib/apt/lists/*

FROM base AS build
ARG DEPTHAI_ROS_REPO=https://github.com/SwapnilPande/depthai-ros.git
ARG DEPTHAI_ROS_REF=humble-develop
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
    build-essential python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*
# A git ADD resolves the ref on every build, so a moved branch is never
# served from cache.
ADD ${DEPTHAI_ROS_REPO}#${DEPTHAI_ROS_REF}:depthai_ros_driver /ws/src/depthai_ros_driver
RUN . /opt/ros/humble/setup.sh \
 && cd /ws \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM base
COPY --from=build /ws/install/depthai_ros_driver_v3/ /opt/ros/humble/
COPY oak.yaml /etc/rammp/oak/oak.yaml
COPY ready_timeout.py /usr/local/bin/ready_timeout.py

# The whole default invocation is the entrypoint, so a bare `docker run` brings
# the camera up at /driver/... (the node's own name) and anything a deployment
# appends is an override:
# sheppy adds `--ros-args --params-file` for its `params:` block, and a
# `command:` of `--ros-args -r __node:=wrist_camera` moves the topics. The node
# binary rather than driver.launch.py: with rs_compat that launch file forces
# a /camera namespace, and only a node's argv can take the appended arguments.
# ready_timeout.py wraps it: the driver searches forever for a camera that is
# not there, so the container exits 1 if it is not ready within
# RAMMP_OAK_READY_TIMEOUT seconds (default 60, 0 = never).
ENTRYPOINT ["/ros_entrypoint.sh", "/usr/local/bin/ready_timeout.py", \
            "/opt/ros/humble/lib/depthai_ros_driver_v3/driver_node", \
            "--ros-args", "--params-file", "/etc/rammp/oak/oak.yaml"]
