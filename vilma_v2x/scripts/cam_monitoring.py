#!/usr/bin/env python

# ==============================================================================
# MIT License
#
# Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ==============================================================================

import rclpy
from rclpy.node import Node
from etsi_its_cam_msgs.msg import *
import utils

class cam_monitoring(Node):

    def __init__(self):

        super().__init__("cam_publisher")
        
        self.cam_monitoring = self.create_publisher(CAM, "/v2x/etsi_parser/cam/in", 1)
        
        self.cam_sub = self.create_subscription(
           CAM, '/v2x/etsi_parser/cam/out', self.camCallback, 1)
        
        self.last_cam = self.get_clock().now().nanoseconds * 1e-9
        
    def camCallback(self, msg):
        period = (self.get_clock().now().nanoseconds * 1e-9) - self.last_cam
        self.last_cam = self.get_clock().now().nanoseconds * 1e-9
        if (period > 2.0):
            
            self.get_logger().warn('CAM period: '+f"{period:.6f}"+' s')
        else:
            self.get_logger().info('CAM period: '+f"{period:.6f}"+' s')


if __name__ == "__main__":

    rclpy.init()
    cam_monitoring = cam_monitoring()
    rclpy.spin(cam_monitoring)
    rclpy.shutdown()
