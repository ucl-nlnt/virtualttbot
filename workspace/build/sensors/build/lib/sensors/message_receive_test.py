import rclpy
import time
import threading
import socket
import sys

from rclpy.node import Node
from sensor_msgs.msg import Imu, BatteryState
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
File has been updated to work on Turtlebot3
"""

class SensorsSubscriber(Node):

    def __init__(self):

        super().__init__('sensors_subscriber')
        
        # self.create_subscription(LaserScan, 'scan', self.scan_listener, qos_profile_sensor_data)
        self.create_subscription(Imu,'imu', self.imu_listener, qos_profile_sensor_data)
        self.create_subscription(Twist,'cmd_vel', self.cmd_vel_listener, qos_profile_sensor_data)
        self.time_last_frame = 0
        
        self.linear_x, self.linear_y, self.linear_z = 0, 0, 0
        self.angular_x, self.angular_y, self.angular_z = 0, 0, 0

        
        self.sock = socket.socket()
        server_ip = "192.168.68.60" # Gab's PC at boarding house
        # server_ip = "10.158.38.95" # Gab's RoboticsClass network address
        server_port = 50000; 
        flag_connected = False
        
        try:
            self.sock.connect((server_ip, server_port))
            print("Connected to server.")
            flag_connected = True
        
        except Exception as e:
            print(e)
            print("Error has occured when trying to connect to server.")

        if not flag_connected:
            print("Exiting program...")
            sys.exit() # Close program if unable to connect to server


    # LIDAR is disabled for now, not needed for basic proof-of-concept

    def imu_listener(self, msg = None):
        
        if msg == None: return
        
        t = time.time()
        time.sleep(0.1)

        dt = t - self.time_last_frame
        self.time_last_frame = time.time()
        
        # print("Angular: ", msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z) # rad/s
        # print("Linear Acceleration: ", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z) # Measured in m/s^2
        # print("Orientation: ", msg.orientation.x, msg.orientation.y, msg.orientation.z) # meters?

        data = f"[PLACEHOLDER] {msg.orientation.x} {msg.orientation.y} {msg.orientation.z} gx gy gz {self.linear_x} {self.linear_y} {self.linear_z} {self.angular_x} {self.angular_y} {self.angular_z} {dt} phase".encode("utf-8")
        data_len = str(len(data))
        
        data_len = ('@' + data_len + 'X' * (8 - len(data_len))).encode() # padded on the right, 1234XXXX as an example

        self.sock.send(data_len)
        print("sending data length:",data_len)
        self.sock.recv(5) # wait for reply from server
        print("sending data:",data)
        self.sock.sendall(data)
    
    def cmd_vel_listener(self, msg = None):

        if msg == None: return

        self.linear_x, self.linear_y, self.linear_z = msg.linear.x, msg.linear.y, msg.linear.z
        self.angular_x, self.angular_y, self.angular_z = msg.angular.x, msg.angular.y, msg.angular.z
        
def main(args=None):

    rclpy.init(args=args)
    sensors_subscriber = SensorsSubscriber()
    rclpy.spin(sensors_subscriber)
    sensors_subscriber.destroy_node()

if __name__ == '__main__':

    main()