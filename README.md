# rammp-suite-software

Code & Drivers for the sensors on RAMMP.

## Camera drivers

| Camera | Role | Image | ROS package |
|---|---|---|---|
| Orbbec Gemini 336L | scene | `ghcr.io/rammp-org/rammp-suite-software/gemini` | `ros-humble-orbbec-camera` |
| Intel RealSense D405 | wrist | `ghcr.io/rammp-org/rammp-suite-software/realsense` | `ros-humble-realsense2-camera` |

Both drivers ship as arm64 apt packages from `packages.ros.org`, so each
Dockerfile is `ros:humble-ros-base` plus one `apt install` — no source builds.

```bash
make build              # both images, tagged :dev
make gemini             # just one
make push TAG=v0.1.0    # after `docker login ghcr.io`
```

CI builds both on every push and publishes branch, semver, and sha tags to
GHCR. The containers are *run* by sheppy, not by this repo: the flags they need
(privileged, host net/IPC, `/dev`, uid) live with the deployment manifest in
`rammp-deployments`, and the dojo cell's `compose.yaml` runs the same images.
