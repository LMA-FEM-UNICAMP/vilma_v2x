#!/bin/bash

docker run -it --rm \
  --net=host \
  --ipc=host \
  --cap-add=NET_ADMIN \
  --cap-add=NET_RAW \
  --privileged \
  --device=/dev:/dev \
  --name=obu_localization_container \
  --mount type=bind,src=/home/toffanetto/bag_docker/,dst=/workspace/bag \
  obu_localization_container:dev_V0