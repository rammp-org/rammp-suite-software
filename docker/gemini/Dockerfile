# Orbbec Gemini 330-series driver (the 336L is one of them).
FROM ghcr.io/rammp-org/rammp-base:1.0.0-jp6

RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-humble-orbbec-camera \
 && rm -rf /var/lib/apt/lists/*

COPY gemini.yaml /etc/rammp/orbbec/gemini.yaml

# The whole default invocation is the entrypoint, so a bare `docker run` brings
# the camera up at /color/image_raw, /depth/... and anything a deployment
# appends is an override: sheppy adds `--ros-args --params-file` for its
# `params:` block, and a `command:` of `--ros-args -r __ns:=/scene_camera`
# moves the topics. This driver names by namespace, and the first __ns remap
# on a command line wins, so none is baked here.
ENTRYPOINT ["/ros_entrypoint.sh", \
            "/opt/ros/humble/lib/orbbec_camera/orbbec_camera_node", \
            "--ros-args", "--params-file", "/etc/rammp/orbbec/gemini.yaml"]
