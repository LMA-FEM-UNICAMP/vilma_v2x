#!/bin/bash

# Exit on error
set -e

IFACE=${1:-vcan0}

echo "Setting up virtual CAN interface: $IFACE"

# Load CAN kernel modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan

# Create the vcan interface (ignore error if already exists)
sudo ip link add dev $IFACE type vcan 2>/dev/null || true

# Bring the interface up
sudo ip link set up $IFACE

echo "Interface $IFACE is up"

# Show interface details
ip link show $IFACE
