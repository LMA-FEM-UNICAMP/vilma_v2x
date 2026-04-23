#!/bin/bash

docker run -it --rm \
  --net=host \
  --ipc=host \
  --cap-add=NET_ADMIN \
  --cap-add=NET_RAW \
  --privileged \
  --device=/dev:/dev \
  --name=obu_localization_container \
  obu_localization_container:dev_V0