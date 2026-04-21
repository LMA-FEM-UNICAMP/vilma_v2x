#!/bin/bash

# Let all /dev to container once xsens package can find the IMU in any port and the OBU sometimes mount it in random ports.

docker run -it --rm --net=host   --ipc=host   --cap-add=NET_ADMIN   --cap-add=NET_RAW  --privileged --device=/dev:/dev  --name xsens_obu_container xsens_obu_container:ARM_V2
   
  