# Luxonis OAK driver (DepthAI ROS v3): OAK-D Pro PoE, OAK-D Pro W PoE, and the
# rest of the family. Which one, and how it is tuned, comes from the deployment.
FROM ghcr.io/rammp-org/rammp-base:1.0.0-jp6

RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-humble-depthai-ros-v3 \
 && rm -rf /var/lib/apt/lists/*

COPY oak.yaml /etc/rammp/oak/oak.yaml

# The whole default invocation is the entrypoint, so a bare `docker run` brings
# the camera up at /driver/... (the node's own name) and anything a deployment
# appends is an override:
# sheppy adds `--ros-args --params-file` for its `params:` block, and a
# `command:` of `--ros-args -r __node:=wrist_camera` moves the topics. The node
# binary rather than driver.launch.py: with rs_compat that launch file forces
# a /camera namespace, and only a node's argv can take the appended arguments.
ENTRYPOINT ["/ros_entrypoint.sh", \
            "/opt/ros/humble/lib/depthai_ros_driver_v3/driver_node", \
            "--ros-args", "--params-file", "/etc/rammp/oak/oak.yaml"]
