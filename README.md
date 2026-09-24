# rammp-suite-software

Code & Drivers for the sensors on RAMMP.

## Camera drivers

| Camera | Role | Image | ROS package |
|---|---|---|---|
| Orbbec Gemini 336L | scene | `ghcr.io/rammp-org/rammp-suite-software/gemini` | `ros-humble-orbbec-camera` |
| Intel RealSense D405 | wrist | `ghcr.io/rammp-org/rammp-suite-software/realsense` | `ros-humble-realsense2-camera` |
| Intel RealSense D435i | any, with IMU | `ghcr.io/rammp-org/rammp-suite-software/d435i` | `ros-humble-realsense2-camera` |
| Luxonis OAK-D Pro (PoE) | wrist, alternative | `ghcr.io/rammp-org/rammp-suite-software/oak` | `ros-humble-depthai-ros-v3` |

Every driver ships as an arm64 apt package from `packages.ros.org`, so each
Dockerfile is `rammp-base` plus one `apt install` — no source builds. The base
is what puts the cameras on Cyclone DDS with the fleet's config, the same graph
as the arm; its tag is arm64-only, so these images are too.

The OAK image also carries `/etc/rammp/oak/wrist.yaml`, the near-field stereo
tuning (ROBOTICS preset, extended disparity, dot projector). The camera's
address is not in it: the file reads `OAK_IP` from the environment at launch.

The D435i image likewise carries `/etc/rammp/realsense/d435i.yaml`: stream
profiles (`WxHxFPS`), IMU rates, aligned depth. Its entrypoint runs the node
with that file, so a bare `docker run` publishes at `/camera/...`, and a
deployment only appends: sheppy's `params:` block overrides any value, and a
`command:` of `--ros-args -r __node:=wrist_camera` renames it. The file's
header shows the manifest entry. The IMU needs the host kernel's HID-sensor
IIO drivers, which abra's Tegra kernel lacks: there the node logs that the
IMU is disabled and streams color and depth regardless.

```bash
make build              # every image, tagged :dev
make gemini             # just one
make push TAG=v0.1.0    # after `docker login ghcr.io`
```

CI builds them all on every push and publishes branch, semver, and sha tags to
GHCR. The containers are *run* by sheppy, not by this repo: the flags they need
(privileged, host net/IPC, `/dev`, uid) live with the deployment manifest in
`rammp-deployments`, and the dojo cell's `compose.yaml` runs the same images.
