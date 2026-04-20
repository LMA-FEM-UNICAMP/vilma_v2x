docker run -it --rm \
  --net=host \
  --cap-add=NET_ADMIN \
  --cap-add=NET_RAW \
  --device=/dev/ttyUSB0:/dev/ttyUSB0 \
  xsens_obu_conteiner:latest