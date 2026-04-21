#!/bin/bash

docker run -it --rm \
  --net=host \
  --ipc=host \
  --cap-add=NET_ADMIN \
  --cap-add=NET_RAW \
  --device=/dev/ttyUSB0:/dev/ttyUSB0 \
  xsens_obu_container:ARM_V1
