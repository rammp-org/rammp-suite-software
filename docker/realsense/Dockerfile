# Intel RealSense driver, defaults for the D405 wrist camera. Same apt package
# as the D435i image; the two differ only in their defaults file.
FROM ghcr.io/rammp-org/rammp-base:1.0.0-jp6

RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-humble-realsense2-camera \
 && rm -rf /var/lib/apt/lists/*

COPY realsense.yaml /etc/rammp/realsense/realsense.yaml

# The whole default invocation is the entrypoint, so a bare `docker run` brings
# the camera up at /camera/... and anything a deployment appends is an override:
# sheppy adds `--ros-args --params-file` for its `params:` block, and a
# `command:` of `--ros-args -r __node:=wrist_camera` moves the topics. The node
# binary rather than rs_launch.py, because only a node's argv can take those.
ENTRYPOINT ["/ros_entrypoint.sh", \
            "/opt/ros/humble/lib/realsense2_camera/realsense2_camera_node", \
            "--ros-args", "-r", "__ns:=/", \
            "--params-file", "/etc/rammp/realsense/realsense.yaml"]
