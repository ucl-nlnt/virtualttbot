import rclpy
import time
import threading
import socket
import sys

from rclpy.node import Node
from sensor_msgs.msg import Imu, BatteryState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist

"""
Note that the following code was tested in a simulator and might not necessarily work on a real, physical turtlebot.
If the process requires anything GPU-related, I am 99% certain that it will not work at all.

Test system specs:
Processor: AMD Ryzen 5 5600H
RAM: 16GB LPDDR4 3200MHz (Dual-channel)
GPU: Nvidia RTX 3050ti

Edit (Nov. 24, 2023)
File has been updated to work on Turtlebot3 (MIGHT BE SLOW)
"""

class SensorsSubscriber(Node):

    def __init__(self, server_ip_address, transmit_to_server = True):

        super().__init__('sensors_subscriber')

        # Subscriptions
        # self.create_subscription(LaserScan, 'scan', self.scan_listener, qos_profile_sensor_data)
        # self.create_subscription(Twist,'cmd_vel', self.cmd_vel_listener, qos_profile_sensor_data)
        # ^ does not seem to contribute much -Gab
        # self.create_subscription(Imu,'imu', self.imu_listener, qos_profile_sensor_data) # IMU data doesn't seem to be useful currently

        self.create_subscription(Odometry, 'odom', self.odometry_listener, qos_profile_sensor_data)
        self.create_publisher(Twist, '/cmd_vel',10)
        self.order_clear_timer = self.create_timer(0.1, self.order_clear)
        # IMU variables:
        # self.linear_vel_imu = (0,0,0)
        # self.angular_vel_imu = (0,0,0)
        # self.orientation_imu = (0,0,0)

        # Odometry variables:
        self.position_odom = [None,None,None]
        self.rotation_odom = None

        # Instruction variables
        self.last_order = '@0000'
        self.time_last_order_issued = 0.0
        self.velocity_change = False
        
        self.twist_msg = [None, None] # linear x, angular z

        # Scanner variables:
        # empty for now

        self.transmit_to_server = transmit_to_server

        if self.transmit_to_server:
            self.sock = socket.socket()
            server_ip = server_ip_address

            server_port = 50000;
            flag_connected = False
            self.send_lock = True # signals that new data is allowed to be transmitted

            try:
                self.sock.connect((server_ip, server_port))
                print("[SensorsSubscriber] Connected to server.")
                flag_connected = True

            except Exception as e:
                print(e)
                print("[SensorsSubscriber] Error has occured when trying to connect to server.")

            if not flag_connected:
                print("[SensorsSubscriber] Exiting program...")
                sys.exit() # Close program if unable to connect to server

    def order_clear(self):
        if time.time() - self.time_last_order_issued >= 0.1:
            pass

    def twist_listener(self, msg = None):
        
        if msg == None: return
        
        self.twist_msg = [msg.linear.x, msg.angular.z]

    def odometry_listener(self, msg = None):

        if msg == None: return
        # Odometer listener

        self.position_odom = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z] # x, y, phi
       
    def data_bridge_tx_TwistOdometry(self):

        if not self.transmit_to_server: return # error handling

        
        while True:

            # STATE 3

            print('connected')
            transmission_size, encoded_data = self.parse_for_transmit_str(str(self.position_odom))
            self.sock.sendall(transmission_size)
            self.sock.recv(2) # wait for OK
            self.sock.sendall(encoded_data)
            
            # STATE 4
            
            # loop back to STATE 3, wait for next prompt

    def parse_for_transmit_str(self,msg_str: str):
        
        data_encoded = msg_str.encode()
        data_size = str(len(data_encoded)) # For this system, the other end expects a data identifier length of 5, ex. @1234. Maximum STR length of 9999 chars.
        dat_size_transmit = '@' + ((4 - len(data_size)) * 'X') + data_size

        return (dat_size_transmit.encode(), data_encoded)

    def movement(self,linear_x,angular_z): # helper function that instantiates turtlebot movement
    
        data = Twist()
        data.linear.x = linear_x
        data.angular.z = angular_z
        self.movement_publisher.publish(data)
        
        return

def main(args=None):


    server_ip_address = "192.168.68.60" # Gab's boarding house IP address
    rclpy.init(args=args)
    sensors_subscriber = SensorsSubscriber(server_ip_address = server_ip_address)
    rclpy.spin(sensors_subscriber)
    sensors_subscriber.destroy_node()

if __name__ == '__main__':

    main()
