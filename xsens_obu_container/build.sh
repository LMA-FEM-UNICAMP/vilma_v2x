#!/bin/bash

docker buildx build --platform linux/arm64 -t xsens_obu_container:ARM_VX --load .

