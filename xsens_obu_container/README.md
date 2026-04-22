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

### Configure OBU to run the container

```shell
apt update

# Install Docker keyring
sudo apt install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF

apt update

# Install Docker Engine
apt install -y docker-ce docker-ce-cli containerd.io

# Init Docker daemon
systemctl start docker

# Post-installation
groupadd docker
usermod -aG docker $USER
newgrp docker
```

Change Docker database folder:

> OBU does not have much storage in `/`, you need to move the database to `/mnt/rw`, or better `/mnt/ubi` when using a SD Card.

```shell
# Stop Docker
sudo systemctl stop docker
sudo systemctl stop docker.socket

# Create new directory
sudo mkdir -p /mnt/rw/docker

# Copy files
sudo rsync -aHAX --numeric-ids /var/lib/docker/ /mnt/rw/docker/

# Delete old files (check if they were copied first)
rm -rf /var/lib/docker

# Create symbolic link
ln -s /mnt/rw/docker /var/lib/docker

# Check symbolic link
ll /var/lib

# Start Docker again
sudo systemctl start docker

# Check new database
docker info | grep "Docker Root Dir"
```

From the OBU, move the image to a good place (as `/mnt/rw/docker`) then:

```shell
docker load -i xsens_obu_container-ARM_VX.tar
```

## Running

### Manually:

```shell
docker run -it --rm \
  --net=host \
  --ipc=host \
  --cap-add=NET_ADMIN \
  --cap-add=NET_RAW \
  --device=/dev/ttyUSB3:/dev/ttyUSB3 \ 
  xsens_obu_container:ARM_VX
```

### Daemon:

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
