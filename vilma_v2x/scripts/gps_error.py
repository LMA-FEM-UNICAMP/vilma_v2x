#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from etsi_its_cam_msgs.msg import *
import utils

from sensor_msgs.msg import NavSatFix

import csv

from geopy.distance import geodesic

from geometry_msgs.msg import Vector3Stamped

from math import sqrt

from xsens_mti_ros2_driver.msg import XsStatusWord


header = ['timestamp_gnss', 'timestamp_cam', 'distance', 'velocity_imu', 'velocity_cam', 'rtk', 'lat_gnss', 'long_gnss', 'lat_cam', 'long_cam']


class GpsError(Node):

    def __init__(self):
        super().__init__('gps_error')
        
        
        self.cam_sub = self.create_subscription(
           CAM, '/v2x/etsi_parser/cam/out', self.camCallback, 1)
        
        self.cam_sub = self.create_subscription(
           Vector3Stamped, '/filter/velocity', self.velocityCallback, 1)
        
        self.gnss_sub = self.create_subscription(
            NavSatFix, '/gnss', self.gnssCallback, 1)
        
        self.status_sub = self.create_subscription(
            XsStatusWord, '/status', self.statusCallback, 1)
        
        self.new_data_cam = False
        self.new_data_gnss = False
        
        self.velocity_cam = 0.0
        
        self.cam_latlong = NavSatFix()
        
        self.gnss_latlong = NavSatFix()
        
        self.velocity = 0.0
        
        self.rows = []
        
        self.rtk = False
        
    def save(self):
        with open('low_speed_data_v2x_rtk3.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(header)
            writer.writerows(self.rows)
            
        print('CSV saved.')
        
    def statusCallback(self, msg):
        self.rtk = bool(msg.rtk_status)
    
    def velocityCallback(self, msg):
        
        self.velocity = sqrt(msg.vector.x**2 + msg.vector.y**2 + msg.vector.z**2)*3.6
        
    def camCallback(self, msg):
        
        self.cam_latlong.header.stamp = self.get_clock().now().to_msg()
        
        self.cam_latlong.latitude = float(msg.cam.cam_parameters.basic_container.reference_position.latitude.value) * 1e-7
        self.cam_latlong.longitude = float(msg.cam.cam_parameters.basic_container.reference_position.longitude.value) * 1e-7
        
        self.velocity_cam = float(msg.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.speed.speed_value.value) / 100.0 * 3.6
        
        if(self.new_data_gnss):
            self.getDistance()
        else:
            self.new_data_cam = True
        
    def gnssCallback(self, msg):
        
        self.gnss_latlong = msg
        
        if(self.new_data_cam):
            self.getDistance()
        else:
            self.new_data_gnss = True
        
    def getDistance(self):
        coord_gnss = (self.gnss_latlong.latitude, self.gnss_latlong.longitude)
        coord_cam = (self.cam_latlong.latitude, self.cam_latlong.longitude)
        distance = geodesic(coord_gnss, coord_cam).meters
        
        row = [ float(self.gnss_latlong.header.stamp.sec) + float(self.gnss_latlong.header.stamp.nanosec *1e-9), 
                float(self.cam_latlong.header.stamp.sec) + float(self.cam_latlong.header.stamp.nanosec *1e-9), 
                distance, self.velocity, self.velocity_cam, self.rtk,
                self.gnss_latlong.latitude, self.gnss_latlong.longitude, 
                self.cam_latlong.latitude, self.cam_latlong.longitude
        ]
        
        self.rows.append(row)
            
        print(row)

        self.new_data_cam = False
        self.new_data_gnss = False
        
        
if __name__ == "__main__":

    rclpy.init()
    gps_error = GpsError()
    
    try:
        rclpy.spin(gps_error)
    except KeyboardInterrupt:
        gps_error.save()
    finally:
        rclpy.shutdown()
