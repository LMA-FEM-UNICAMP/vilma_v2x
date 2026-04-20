#!/bin/bash

RESTART_DELAY=2

source /workspace/install/setup.bash

while true; do
  ros2 launch obu_can_vstate obu_can_vstate.launch.py
  EXIT_CODE=$?

  echo "Launch exited with code $EXIT_CODE"

  echo "Restarting in $RESTART_DELAY seconds..."
  sleep $RESTART_DELAY
done