import rclpy
import time
import threading
import math
import cv2

from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
import json
import argparse

from KNetworking import DataBridgeServer_TCP, DataBridgeClient_TCP

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

import socket
import threading
import time
import sys
from collections import deque
import struct
import random
import base64

###
#   NOTE on USAGE: DataBridgeClient assumes that the Server is already running. Please ensure that it's setting up first!
###

# NOTE: added DataBridge objects here cause colcon build is messing up for some reason



def quaternion_to_yaw(x, y, z, w):

    """
    Convert a quaternion into yaw (rotation around z axis in radians)
    yaw is counterclockwise
    """

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return yaw_z # in radians


class SensorsSubscriber(Node):

    def __init__(self,):

        super().__init__('sensors_subscriber')

        # Subscriptions
        self.create_subscription(LaserScan, 'scan', self.laserscan_callback, qos_profile_sensor_data)
        self.create_subscription(Twist,'cmd_vel', self.twist_callback, qos_profile_sensor_data)
        self.create_subscription(Imu,'imu', self.imu_callback, qos_profile_sensor_data) # IMU data doesn't seem to be useful currently // Gab
        self.create_subscription(Odometry, 'odom', self.odometer_callback, qos_profile_sensor_data)
        self.create_subscription(BatteryState, 'battery_state', self.battery_state_callback, qos_profile_sensor_data)
        self.movement_publisher = self.create_publisher(Twist, '/cmd_vel',10)
    
        # Instruction variables
        self.killswitch = False
        self.sampling_delay = 0.1
        self.server_ip_address = None

        # DEBUG LOCKS
        self.debug_send_data = False
        self.debug_odometer = False
        self.debug_randomizer = False
        self.debug_movement = False

        self.enable_camera = False
        self.linear_x_speed = 0.0
        self.angular_z_speed = 0.0
        self.destination_ip = None
        self.camera_device = 0

        # Sensor messages:
        self.laserscan_msg = None
        self.twist_msg = None
        self.imu_msg = None
        self.odometry_msg = None
        self.battery_state_msg = None

        
        self.set_params()
        self.get_params()

        
        lock = True
        while lock:
            if self.destination_ip != None: self.destination_ip = input("Please enter the destination server's IP << ")
            try:
                self.data_transfer_client = DataBridgeClient_TCP(destination_ip_address=self.destination_ip,
                                                                destination_port=50000)
                
                time.sleep(0.5) # wait for the server to open the 2nd port.
                self.movement_instruction_client = DataBridgeClient_TCP(destination_ip_address=self.destination_ip,
                                                                destination_port=50001)
                lock = False
            except Exception as e:
                print(e)

        # threads

        self.super_json = None
        self.is_collecting_data = False

        self.transfer_thread = threading.Thread(target=self.send_to_server)
        self.movement_subscriber_thread = threading.Thread(target=self.receive_movement_from_server)
        # Compile Data and Transmit to Server
        
        self.compilation_thread = threading.Thread(target = self.compile_to_json)
        self.compilation_thread.start()
        self.transfer_thread.start()
        self.movement_subscriber_thread.start()

        print('INIT COMPLETE')

    def set_params(self):

        self.declare_parameter('linear_x',0.2)              # Default value
        self.declare_parameter('angular_z',0.75)            # Default value
        self.declare_parameter('photos',1)                  # Set to 0 to disable
        self.declare_parameter('sampling_t',0.1)            # DO NOT GO LOWER THAN 0.1!
        self.declare_parameter('server_ip_address',"0.0.0.0")    # If None, will prompt the user for the IP Address
        self.declare_parameter('cam_device',0)              # default is zero

    def get_params(self):

        self.linear_x_speed = self.get_parameter('linear_x').get_parameter_value().double_value
        self.angular_z_speed = self.get_parameter('angular_z').get_parameter_value().double_value
        self.enable_camera = self.get_parameter('photos').get_parameter_value().integer_value
        self.sampling_delay = self.get_parameter('sampling_t').get_parameter_value().double_value
        self.destination_ip = self.get_parameter('server_ip_address').get_parameter_value().string_value
        self.camera_device = self.get_parameter('cam_device').get_parameter_value().integer_value

        print('params:', self.destination_ip, [self.linear_x_speed, self.angular_z_speed], self.enable_camera, self.sampling_delay)

    def receive_movement_from_server(self):

        while True:

            print('waiting for inst')
            inst = self.movement_instruction_client.receive_data()
            print(f'inst: {inst}')

            if isinstance(inst, str): # see KNetworking.py for error codes.
                # TODO: implement error-handling

                print("An error has occured in the movement instruction listener client. Closing the thread. Please restart the program.")
                self.killswitch = True
                break

            elif inst == b'@0000': self.movement(0.0,0.0)
            elif inst == b'@FRWD': self.movement(self.linear_x_speed,0.0)
            elif inst == b'@LEFT': self.movement(0.0,self.angular_z_speed)
            elif inst == b'@RGHT': self.movement(0.0,-self.angular_z_speed)
            elif inst == b'@STRT': self.is_collecting_data = True; print("is_collecting_data = True"); time.sleep(0.2);
            elif inst == b'@STOP': self.is_collecting_data = False; print("is_collecting_data = False"); time.sleep(0.2);

            elif inst == b'@KILL': # terminate program
                self.killswitch = True
                break

            self.movement_instruction_client.send_data(b'@CONT') # give an ACK

    def send_to_server(self):

        while True:

            if self.super_json == None or not self.is_collecting_data: time.sleep(0.1); continue; # prevent excessive CPU usage when nothing is going on
            self.data_transfer_client.send_data(str(self.super_json).encode())
            time.sleep(self.sampling_delay)

    def laserscan_callback(self,msg):
        # print('lsr_scn')
        self.laserscan_msg = msg

    def twist_callback(self,msg):
        if msg == None: return
        self.twist_msg = msg

    def imu_callback(self,msg):
        #print('imu')
        if msg == None: return
        self.imu_msg = msg
    
    def odometer_callback(self,msg):
        #print('odom')
        if msg == None: return
        self.odometry_msg = msg

    def battery_state_callback(self,msg):
        #print('batst')
        if msg == None: return
        self.battery_state_msg = msg

    def compile_to_json(self):

        # Runs asynchronously.
        # Prevent variable was not declared errors.
        laserscan_msg_jsonized = None
        twist_msg_jsonized = {
                    "linear":(0.0, 0.0, 0.0),
                    "angular":(0.0, 0.0, 0.0)
                }
        imu_msg_jsonized = None
        odometry_msg_jsonized = None
        battery_state_msg_jsonized = None
        camera_frame = None

        try:
            cap = cv2.VideoCapture(self.camera_device)
            print('taking a test photo...')
            ret, frame = cap.read()
            if not ret:
                self.enable_camera = False
                print('Camera is good. Proceeding.')

        except Exception as e:
            
            self.enable_camera = False
            print("Error encountered when attempting to turn on camera.")

        while True:

            if not self.is_collecting_data: time.sleep(0.1); continue;

            if self.enable_camera:
                ret, frame = cap.read()
                if not ret:
                    print('WARNING: Camera failed to return an image.')
                    continue

                success, encoded_image = cv2.imencode('.jpg',frame)
                if not success:
                    print('WARNING: Failed to encode image.')
                    continue
            
                camera_frame = base64.b64encode(encoded_image.tobytes()).decode('utf-8')
    
            if self.laserscan_msg != None:

                laserscan_msg_jsonized = {
                    # "seq":self.laserscan_msg.header.seq,       # Broken. Cannot find any documentation
                    # "frame_id":self.laserscan_msg.frame_id,
                    "time_sec":self.laserscan_msg.header.stamp.sec,
                    "time_nano":self.laserscan_msg.header.stamp.nanosec,
                    "angle_min":self.laserscan_msg.angle_min,
                    "angle_max":self.laserscan_msg.angle_max,
                    "angle_increment":self.laserscan_msg.time_increment,
                    "scan_time":self.laserscan_msg.scan_time,
                    "range_min":self.laserscan_msg.range_min,
                    "range_max":self.laserscan_msg.range_max,
                    "ranges":self.laserscan_msg.ranges,
                    "intensities":self.laserscan_msg.intensities
                }

            if self.twist_msg != None:

                twist_msg_jsonized = {
                    "linear":(self.twist_msg.linear.x, self.twist_msg.linear.y, self.twist_msg.linear.z),
                    "angular":(self.twist_msg.angular.x, self.twist_msg.angular.y, self.twist_msg.angular.z)
                }

            if self.imu_msg != None:
                
                imu_msg_jsonized = {
                    "quarternion_orientation": (self.imu_msg.orientation.x, self.imu_msg.orientation.y, self.imu_msg.orientation.z, self.imu_msg.orientation.w),
                    "orientation_covariance": [i for i in self.imu_msg.orientation_covariance],
                    "angular_velocity": (self.imu_msg.angular_velocity.x, self.imu_msg.angular_velocity.y, self.imu_msg.angular_velocity.z),
                    "angular_velocity_covariance": [i for i in self.imu_msg.angular_velocity_covariance],
                    "linear_acceleration": (self.imu_msg.linear_acceleration.x, self.imu_msg.linear_acceleration.y, self.imu_msg.linear_acceleration.z),
                    "linear_acceleration_covariance": [i for i in self.imu_msg.linear_acceleration_covariance]
                }
            
            if self.odometry_msg != None:
                
                msg_pose = self.odometry_msg.pose.pose
                position, orientation = msg_pose.position, msg_pose.orientation
                msg_pose_covariance = self.odometry_msg.pose.covariance

                odometry_msg_jsonized = {
                    "time_sec":self.odometry_msg.header.stamp.sec,
                    "time_nano":self.odometry_msg.header.stamp.nanosec,
                    "pose_position":(position.x, position.y, position.z),
                    "pose_orientation_quarternion":(orientation.x, orientation.y, orientation.z, orientation.w),
                    "object_covariance":[i for i in msg_pose_covariance]
                }

            if self.battery_state_msg != None:

                battery_state_msg_jsonized = {
                    "percentage":self.battery_state_msg.percentage,
                    "voltage":self.battery_state_msg.voltage,
                    "temperature":self.battery_state_msg.temperature,
                    "current":self.battery_state_msg.current,
                }
            
            
            self.super_json = json.dumps({"laser_scan":laserscan_msg_jsonized, "twist":twist_msg_jsonized, "imu":imu_msg_jsonized, "odometry":odometry_msg_jsonized, "battery":battery_state_msg_jsonized, "frame_data":camera_frame})
            time.sleep(self.sampling_delay)
    
    def movement(self,linear_x,angular_z): # helper function that instantiates turtlebot movement
    
        data = Twist()
        data.linear.x = linear_x
        data.angular.z = angular_z
        print(linear_x, angular_z)
        print(data.linear.x, data.angular.z)
        self.movement_publisher.publish(data)
        
        return

    # May or may not be useful?
    def movement_rotate_until(self, target_radians, tolerance=0.1, rotation_s=0.05):

        """
        dirrection_right = True
        current_radians = self.position_odom[2]
        t1 = target_radians - current_radians
        t2 = current_radians - target_radians
        
        if t1 < 0: t1 += 2 * math.pi
        if t2 < 0: t2 += 2 * math.pi

        if t1 < t2: dirrection_right = True
        else: dirrection_right = False
        """
        while True:

            current_radians = self.position_odom[2]

            # Calculate the difference in radians
            diff_radians = target_radians - current_radians

            # Normalize the angle difference to the range [-pi, pi]
            diff_radians = (diff_radians + math.pi) % (2 * math.pi) - math.pi

            # Check if the TurtleBot is within the tolerance of the target orientation
            if math.isclose(diff_radians, 0, abs_tol=tolerance):
            
                self.movement(0.0, 0.0)
                break
            
            # Determine rotation direction for the shortest path
            if (diff_radians >= 0):
            
                if abs(diff_radians) >= 0.25: rotation_speed = rotation_s * 4
                elif abs(diff_radians) <= 0.25 and abs(diff_radians) >= 0.1: rotation_speed = rotation_s
                else: rotation_speed = rotation_s / 4  # Positive value for clockwise rotation
            
            else:
            
                if abs(diff_radians) >= 0.25: rotation_speed = -rotation_s * 4
                elif abs(diff_radians) <= 0.25 and abs(diff_radians) >= 0.1: rotation_speed = -rotation_s
                else: rotation_speed = -rotation_s / 4  # Negative value for counterclockwise rotation

            # Execute the rotation
            if self.debug_randomizer: print(f'rotating to goal... current: {current_radians}, goal: {target_radians}')
            self.movement(0.0, rotation_speed)
            time.sleep(0.1)

    def movement_forward_until_xy(self, target_x, target_y, tolerance = 0.2, linear_s = 0.22):

        distance = math.sqrt((target_x - self.position_odom[0])**2 + (target_y - self.position_odom[1])**2)
        print(f'distance: {distance}')
        while True:

            current_x, current_y = self.position_odom[0], self.position_odom[1]

            # Calculate the difference in radians
            if self.debug_movement: print(f'current: {current_x}, {current_y} | goal: {target_x}, {target_y}')
            diff_x, diff_y = target_x - current_x, target_y - current_y
            current_distance = math.sqrt(diff_x**2 + diff_y**2)
            # Check if the TurtleBot is within the tolerance of the target orientation
         
            if (math.isclose(diff_x, 0, abs_tol=tolerance) and math.isclose(diff_y, 0, abs_tol=tolerance)): print('b')
            if (math.isclose(target_x,self.position_odom[0], abs_tol=tolerance) and math.isclose(target_y,self.position_odom[1], abs_tol=tolerance)):
                self.movement(0.0, 0.0)
                if self.debug_movement: print('movement complete')
                break

            # Determine rotation direction for the shortest path
            if diff_x > 0:
                if self.debug_movement: print('continuing advance')
                self.movement(linear_s, 0.0)
                time.sleep(0.25)

    def movement_forward_until_distance(self, target_distance, tolerance=0.1, linear_s=0.22):
        
        starting_x, starting_y = self.position_odom[0], self.position_odom[1] 
        while True:
            
            distance_traveled = math.sqrt((self.position_odom[0] - starting_x)**2 + (self.position_odom[1] - starting_y)**2)
            if self.debug_movement: print(f'distance traveled: {distance_traveled}, goal: {target_distance}')
            
            if (distance_traveled / target_distance) < 0.9: # accelerate while still far away
                linear_s = 0.22
            else: linear_s = 0.11

            if distance_traveled > target_distance or math.isclose(distance_traveled, target_distance, abs_tol=tolerance):
                if self.debug_movement: print('movement complete')
                self.movement(0.0,0.0)
                break

            else:
                if self.debug_randomizer: print('continuing advance')
                self.movement(linear_s,0.0)
                time.sleep(0.25)

def main(args=None):

    rclpy.init(args=args)

    sensors_subscriber = SensorsSubscriber()
    rclpy.spin(sensors_subscriber)
    sensors_subscriber.destroy_node()

if __name__ == '__main__':

    main()
