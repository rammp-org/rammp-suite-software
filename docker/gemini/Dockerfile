# Orbbec Gemini 336L driver — the 336L is served by the 330-series launch file.
FROM ros:humble-ros-base

RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-humble-orbbec-camera \
 && rm -rf /var/lib/apt/lists/*
