import rclpy
# import time
# import datetime
# import numpy as np
# import cv2
# import threading
# import socket
# import sys
# from math import pi
# import random

from rclpy.node import Node
from sensor_msgs.msg import Imu, BatteryState
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
# from geometry_msgs.msg import Twist
# import json


"""
Note that the following code was tested in a simulator and might not necessarily work on a real, physical turtlebot.
If the process requires anything GPU-related, I am 99% certain that it will not work at all.

Test system specs:
Processor: AMD Ryzen 5 5600H
RAM: 16GB LPDDR4 3200MHz
GPU: Nvidia RTX 3050ti
"""

class SensorsSubscriber(Node):

    def __init__(self):

        super().__init__('sensors_subscriber')
        #self.create_subscription(LaserScan, 'scan', self.scan_listener, qos_profile_sensor_data)
        #self.create_subscription(Imu,'imu', self.imu_listener, qos_profile_sensor_data)
        #self.scan_timer = self.create_timer(10, self.scan_listener) # input args = (delay in seconds, function to be called)
        #self.imu_timer = self.create_timer(5, self.imu_listener)
        print(help(self.create_timer))
    def scan_listener(self, msg = None):

        if msg != None:
            print('msg received')

    def imu_listener(self, msg = None):
        
        print("Angular: ", msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z) # rad/s
        print("Linear Acceleration: ", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z) # Measured in m/s^2
        print("Orientation: ", msg.orientation.x, msg.orientation.y, msg.orientation.z) # meters?
        

def main(args=None):

    rclpy.init(args=args)
    sensors_subscriber = SensorsSubscriber()
    rclpy.spin(sensors_subscriber)
    sensors_subscriber.destroy_node()

if __name__ == '__main__':

    main()