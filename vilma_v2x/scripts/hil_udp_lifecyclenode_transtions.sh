#!/bin/bash

ros2 lifecycle set /v2x_obu_01/udp_receiver_obu_01 1
ros2 lifecycle set /v2x_obu_01/udp_sender_obu_01 1
ros2 lifecycle set /v2x_obu_02/udp_receiver_obu_02 1
ros2 lifecycle set /v2x_obu_02/udp_sender_obu_02 1
ros2 lifecycle set /v2x_obu_01/udp_receiver_obu_01 3
ros2 lifecycle set /v2x_obu_01/udp_sender_obu_01 3
ros2 lifecycle set /v2x_obu_02/udp_receiver_obu_02 3
ros2 lifecycle set /v2x_obu_02/udp_sender_obu_02 3