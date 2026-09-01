# Camera driver images. Built natively on the arm64 machine that runs them
# (abra, the demo Jetson); CI publishes the same images to GHCR.
#   make build          both images, tagged :dev
#   make gemini         just one
#   make push TAG=v0.1.0
REGISTRY ?= ghcr.io/rammp-org/rammp-suite-software
TAG ?= dev
IMAGES := gemini realsense

.PHONY: build push $(IMAGES)

build: $(IMAGES)

$(IMAGES):
	docker build -t $(REGISTRY)/$@:$(TAG) docker/$@

push: build
	for i in $(IMAGES); do docker push $(REGISTRY)/$$i:$(TAG); done
