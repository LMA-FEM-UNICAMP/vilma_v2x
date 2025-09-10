/opt/carla/0.9.15/CarlaUE4.sh -carla-rpc-port=1403

sleep 5

ros2 launch vilma_platooning carla_aw_bridge.launch.py town:=Town06 timeout:=100 port:=1403 view:=true
