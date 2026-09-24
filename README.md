# rammp-suite-software

Code & Drivers for the sensors on RAMMP.

## Camera drivers

| Camera | Image | ROS package | Which camera | Rename |
|---|---|---|---|---|
| Orbbec Gemini 336L | `ghcr.io/rammp-org/rammp-suite-software/gemini` | `ros-humble-orbbec-camera` | `serial_number`, `usb_port` | `-r __ns:=/scene_camera`, plus `camera_name` for the frames |
| Intel RealSense D405 | `ghcr.io/rammp-org/rammp-suite-software/realsense` | `ros-humble-realsense2-camera` | `serial_no`, `usb_port_id`, `device_type` | `-r __node:=wrist_camera` |
| Intel RealSense D435i | `ghcr.io/rammp-org/rammp-suite-software/d435i` | `ros-humble-realsense2-camera` | `serial_no`, `usb_port_id`, `device_type` | `-r __node:=wrist_camera` |
| Luxonis OAK-D Pro PoE, OAK-D Pro W PoE | `ghcr.io/rammp-org/rammp-suite-software/oak` | `ros-humble-depthai-ros-v3` | `driver.i_ip`, `driver.i_device_id`, `driver.i_usb_port_id` | `-r __node:=wrist_camera` |

Every driver ships as an arm64 apt package from `packages.ros.org`, so each
Dockerfile is `rammp-base`, one `apt install`, and one defaults file — no
source builds. The base is what puts the cameras on Cyclone DDS with the
fleet's config, the same graph as the arm; its tag is arm64-only, so these
images are too.

Every image runs its driver's node binary as the entrypoint with the defaults
file at `/etc/rammp/<vendor>/<image>.yaml`, so a bare `docker run` brings the
camera up and a deployment only appends. Sheppy's `params:` block, delivered
as a later `--params-file`, does two jobs: it picks the physical camera by
serial, USB port, or IP (the "which camera" column), and it sets the
configuration — resolution, framerate, alignment, point cloud, IMU, presets.
A `command:` of the rename column moves the topics to the camera's role. The
defaults are the driver's own; camera-specific tuning, such as the wrist OAK's
near-field preset, belongs to the deployment. Each yaml's header shows the
manifest entry, and the D435i's IMU needs the host kernel's HID-sensor IIO
drivers, which abra's Tegra kernel lacks: there the node logs that the IMU is
disabled and streams color and depth regardless.

No image bakes a rename, because the first `__node` or `__ns` remap on a
command line wins and would block the deployment's. The D405 and D435i images
are the same package with different defaults; the OAK image serves the wide
OAK-D Pro W (the OV9782 "97" variant included) with its own IP and resolution
in `params:`, its default color undistortion taking care of the wide lens.

```bash
make build              # every image, tagged :dev
make gemini             # just one
make push TAG=v0.1.0    # after `docker login ghcr.io`
```

CI builds them all on every push and publishes branch, semver, and sha tags to
GHCR. The containers are *run* by sheppy, not by this repo: the flags they need
(privileged, host net/IPC, `/dev`, uid) live with the deployment manifest in
`rammp-deployments`, and the dojo cell's `compose.yaml` runs the same images.
