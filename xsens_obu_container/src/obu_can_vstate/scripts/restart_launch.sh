#!/bin/bash

RESTART_DELAY=2

export ROS_DOMAIN_ID=0
source /workspace/install/setup.bash

STOP=false

trap "STOP=true" SIGINT

while true; do
  ros2 launch obu_can_vstate obu_can_vstate.launch.py
  
  EXIT_CODE=$?

  if $STOP; then
    echo "[EXIT] Received Ctrl+C, exiting restart loop."
    break
  fi

  echo "Launch exited with code $EXIT_CODE"
  echo "Restarting in $RESTART_DELAY seconds..."

  sleep $RESTART_DELAY
done