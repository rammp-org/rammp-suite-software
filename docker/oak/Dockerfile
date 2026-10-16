# Luxonis OAK driver (DepthAI ROS v3): OAK-D Pro PoE, OAK-D Pro W PoE, and the
# rest of the family. Which one, and how it is tuned, comes from the deployment.
FROM ghcr.io/rammp-org/rammp-base:1.0.0-jp6

RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-humble-depthai-ros-v3 \
 && rm -rf /var/lib/apt/lists/*

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
