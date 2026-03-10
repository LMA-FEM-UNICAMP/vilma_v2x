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

initial_latlong = [-22.819264393369803, -47.063799710238186]


class GpsError(Node):

    def __init__(self):
        super().__init__('cv2x_range')
        
        
        self.cam_sub = self.create_subscription(
           CAM, '/v2x/etsi_parser/cam/out', self.camCallback, 1)
        
        self.cam_latlong = NavSatFix()
        
        self.gnss_latlong = NavSatFix()
        self.gnss_latlong.latitude = initial_latlong[0]
        self.gnss_latlong.longitude = initial_latlong[1]
        
        self.max_distance = 0
        
    def camCallback(self, msg):
        
        self.cam_latlong.header.stamp = self.get_clock().now().to_msg()
        
        self.cam_latlong.latitude = float(msg.cam.cam_parameters.basic_container.reference_position.latitude.value) * 1e-7
        self.cam_latlong.longitude = float(msg.cam.cam_parameters.basic_container.reference_position.longitude.value) * 1e-7
        
        self.velocity_cam = float(msg.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.speed.speed_value.value) / 100.0 * 3.6
        
        self.getDistance()
        
    
        
    def getDistance(self):
        coord_gnss = (self.gnss_latlong.latitude, self.gnss_latlong.longitude)
        coord_cam = (self.cam_latlong.latitude, self.cam_latlong.longitude)
        distance = geodesic(coord_gnss, coord_cam).meters
        print(str(distance)+" metros")
        
        if(self.max_distance < distance):
            print("*** New max distance: "+str(distance)+" metros")
            self.max_distance = distance
        
        
if __name__ == "__main__":

    rclpy.init()
    gps_error = GpsError()
    
    try:
        rclpy.spin(gps_error)
    except KeyboardInterrupt:
        gps_error.save()
    finally:
        rclpy.shutdown()
