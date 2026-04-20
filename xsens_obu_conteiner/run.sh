docker run -it --rm \
  --net=host \
  --cap-add=NET_ADMIN \
  --cap-add=NET_RAW \
  xsens_obu_conteiner:latest