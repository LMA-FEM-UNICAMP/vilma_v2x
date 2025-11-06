#!/usr/bin/env python3
# GPT made
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu

class TfPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')

        # Create a TransformBroadcaster
        self.br = TransformBroadcaster(self)

        # Create a timer to send transforms periodically (10 Hz)
        # self.timer = self.create_timer(0.1, self.broadcast_tf)
        
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.broadcast_tf, 1)
        
        # self.imu_sub = self.create_subscription(
        #     PointCloud2, 'velodyne_points', self.broadcast_tf, 1)

    def broadcast_tf(self, msg):
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # No rotation (identity quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Send the transform
        self.br.sendTransform(t)
        self.get_logger().debug(f'Published TF from {t.header.frame_id} to {t.child_frame_id}')
        
        
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'vlp16_front'
        
        self.br.sendTransform(t)
        self.get_logger().debug(f'Published TF from {t.header.frame_id} to {t.child_frame_id}')

def main(args=None):
    rclpy.init(args=args)
    node = TfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
