
import rclpy
from rclpy.node import Node

from etsi_its_cam_msgs.msg import *
import utils

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive 

class carla_leader_controller(Node):

    def __init__(self):

        super().__init__("carla_leader_controller")
        
        self.timer = self.create_timer(
            1, self.publishCAM)
        
        self.timer = self.create_timer(
            10, self.publishAckermannLeader)

        self.leader_control = self.create_publisher(
            AckermannDrive, '/carla/leader_vehicle/ackermann_cmd', 1)
        
        self.gnss_sub = self.create_subscription(
            NavSatFix, '/carla/leader_vehicle/leader/sensor/gnss', self.gnssCallback, 1)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/carla/leader_vehicle/odometry', self.odomCallback, 1)
        
        self.publisher_cam = self.create_publisher(
            CAM, "/cam/in", 1)
        
        self.gnss = NavSatFix()
        self.odom = Odometry()
        self.ackermann = AckermannDrive()
        self.ackermann.speed = 10
        self.ackermann.acceleration = 1.5
        
        
    def odomCallback(self, msg):
        
        self.odom = msg


    def gnssCallback(self, msg):
        
        self.gnss = msg
    
    def publishCAM(self):
        
        msg = CAM()

        msg.header.protocol_version = 2
        msg.header.message_id = msg.header.MESSAGE_ID_CAM
        msg.header.station_id.value = 32

        msg.cam.generation_delta_time.value = int(utils.get_t_its(self.get_clock().now().nanoseconds) % 65536)

        msg.cam.cam_parameters.basic_container.station_type.value = msg.cam.cam_parameters.basic_container.station_type.PASSENGER_CAR
        msg.cam.cam_parameters.basic_container.reference_position.latitude.value = int(msg.cam.cam_parameters.basic_container.reference_position.latitude.ONE_MICRODEGREE_NORTH * 1e6 * self.gnss.latitude)
        msg.cam.cam_parameters.basic_container.reference_position.longitude.value = int(msg.cam.cam_parameters.basic_container.reference_position.longitude.ONE_MICRODEGREE_EAST * 1e6 * self.gnss.longitude)
        msg.cam.cam_parameters.basic_container.reference_position.altitude.altitude_value = int(1e2 * self.gnss.altitude) # ? Is altitude in cm?

        basic_vehicle_container_high_frequency = BasicVehicleContainerHighFrequency()
        basic_vehicle_container_high_frequency.heading.heading_value.value = basic_vehicle_container_high_frequency.heading.heading_value.WGS84_NORTH
        basic_vehicle_container_high_frequency.heading.heading_confidence.value = basic_vehicle_container_high_frequency.heading.heading_confidence.EQUAL_OR_WITHIN_ONE_DEGREE
        basic_vehicle_container_high_frequency.speed.speed_value.value = int(self.odom.twist.twist.linear.x)
        basic_vehicle_container_high_frequency.speed.speed_confidence.value = basic_vehicle_container_high_frequency.speed.speed_confidence.EQUAL_OR_WITHIN_ONE_CENTIMETER_PER_SEC
        basic_vehicle_container_high_frequency.vehicle_length.vehicle_length_value.value = basic_vehicle_container_high_frequency.vehicle_length.vehicle_length_value.TEN_CENTIMETERS * 42
        basic_vehicle_container_high_frequency.vehicle_width.value = basic_vehicle_container_high_frequency.vehicle_width.TEN_CENTIMETERS * 18
        msg.cam.cam_parameters.high_frequency_container.choice = msg.cam.cam_parameters.high_frequency_container.CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY
        msg.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency = basic_vehicle_container_high_frequency

        self.publisher_cam.publish(msg)
        
    def publishAckermannLeader(self):
        
        self.leader_control.publish(self.ackermann)
        
        if(self.ackermann.speed >= 20.0):
            self.ackermann.speed = self.ackermann.speed - 5
            
        elif(self.ackermann.speed <= 5.0):
            self.ackermann.speed = self.ackermann.speed + 5
        
        
        


if __name__ == "__main__":

    rclpy.init()
    carla_leader_controller = carla_leader_controller()
    rclpy.spin(carla_leader_controller)
    rclpy.shutdown()