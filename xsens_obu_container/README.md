# xsens_obu_container: IMU to CAN container bridge for Cohda MK6 OBU

## Build

```
docker buildx build --platform linux/arm64 -t xsens_obu_container:ARM_VX --load .
```

## Deploying

Export the built image:

```shell
docker save xsens_obu_container:ARM_VX -o xsens_obu_container-ARM_VX.tar
```

Sent the image to the OBU:

```shell
scp xsens_obu_container-ARM_VX.tar user@<OBU_IP>:.
```

From the OBU, load the image:

```shell
docker load -i xsens_obu_container-ARM_VX.tar
```

## Running

Manually:

```shell
docker run -it --rm \
  --net=host \
  --ipc=host \
  --cap-add=NET_ADMIN \
  --cap-add=NET_RAW \
  --device=/dev/ttyUSBX:/dev/ttyUSBX \ # IMU USB port
  xsens_obu_container:ARM_VX
```

`systemd`:

Copy `xsens_obu_container.service` to `/etc/systemd/system/` and run:

```shell
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable xsens_obu_container
sudo systemctl start xsens_obu_container
```

To stop:

```shell
sudo systemctl disable xsens_obu_container
sudo systemctl stop xsens_obu_container
```
