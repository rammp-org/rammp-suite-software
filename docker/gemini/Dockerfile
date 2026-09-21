# Orbbec Gemini 336L driver — the 336L is served by the 330-series launch file.
FROM ghcr.io/rammp-org/rammp-base:1.0.0-jp6

RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-humble-orbbec-camera \
 && rm -rf /var/lib/apt/lists/*
