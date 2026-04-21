#!/bin/bash

docker buildx build --platform linux/arm64 -t xsens_obu_container:ARM_V1 --load .

docker save xsens_obu_container:ARM_V1 -o xsens_obu_container-ARM_V1.tar