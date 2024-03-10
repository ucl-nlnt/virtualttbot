import rclpy
import time
import threading
import socket
import sys
import struct
import random
import math

from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from copy import deepcopy

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

    def __init__(self, server_ip_address, transmit_to_server = False):

        super().__init__('sensors_subscriber')

        # Subscriptions
        self.create_subscription(LaserScan, 'scan', self.laserscan_callback, qos_profile_sensor_data)
        self.create_subscription(Twist,'cmd_vel', self.twist_callback, qos_profile_sensor_data)
        self.create_subscription(Imu,'imu', self.imu_callback, qos_profile_sensor_data) # IMU data doesn't seem to be useful currently // Gab
        self.create_subscription(Odometry, 'odom', self.odometer_callback, qos_profile_sensor_data)
        self.create_subscription(BatteryState, 'battery_state', self.battery_state_callback, qos_profile_sensor_data)
        self.movement_publisher = self.create_publisher(Twist, '/cmd_vel',10)
    
        # Instruction variables
        self.last_order = '@0000'
        self.time_last_order_issued = 0.0
        self.velocity_change = False
        self.killswitch = False

        # DEBUG LOCKS
        self.debug_send_data = False
        self.debug_odometer = False
        self.debug_randomizer = False
        self.debug_movement = False

        self.transmit_to_server = transmit_to_server

        # Sensor messages:
        self.laserscan_msg = None
        self.twist_msg = None
        self.imu_msg = None
        self.odometry_msg = None
        self.battery_state_msg = None

        self.data_transfer_client = DataBridgeClient_TCP(destination_ip_address="localhost",
                                                         destination_port=50000)

        # threads
                
        self.transfer_thread = threading.Thread(target=self.send_to_server)

        # Compile Data and Transmit to Server
        self.super_json = None
        self.compilation_thread = threading.Thread(target = self.compile_to_json)
        self.compilation_thread.start()
        self.transfer_thread.start()

        print('INIT COMPLETE')


    def laserscan_callback(self,msg):
        print('lsr_scn')
        self.laserscan_msg = deepcopy(msg)

    def twist_callback(self,msg):
        #print('twst')
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
        twist_msg_jsonized = None
        imu_msg_jsonized = None
        odometry_msg_jsonized = None
        battery_state_msg_jsonized = None

        while True:

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
            
            self.super_json = str({"laser_scan":laserscan_msg_jsonized, "twist":twist_msg_jsonized, "imu":imu_msg_jsonized, "odometry":odometry_msg_jsonized, "battery":battery_state_msg_jsonized}).encode()

            time.sleep(1) # TODO: implement sampling frequency parameter
    
    # TODO: fix this.
    def data_bridge_tx_TwistOdometry(self):

        if not self.transmit_to_server: return # error handling
        data_is_collecting = True

        while self.position_odom == [None,None,None]: time.sleep(1); print('waiting for odometer to warm up...') # wait for odometer to initialize

        while data_is_collecting:

            inst = self.receive_data()
            print(inst)
            if inst == b'@STRT':

                time.sleep(0.2)
                self.send_data(f'{self.position_odom}'.encode())
                print('Odom:', self.position_odom)

            elif inst == b'@0000': self.movement(0.0,0.0)
            elif inst == b'@FRWD': self.movement(0.22,0.0)
            elif inst == b'@LEFT': self.movement(0.0,0.75)
            elif inst == b'@RGHT': self.movement(0.0,-0.75)
            elif inst == b'@ODOM': 

                time.sleep(0.2)
                self.send_data(f'{self.position_odom}'.encode())
                print('Odom:', self.position_odom)

            # elif inst == b'@STOP': self.movement(0.0,0.0); data_is_collecting = False
            elif inst == b'@KILL':
                print('exiting...') 
                self.movement(0.0,0.0)
                data_is_collecting = False 
                self.killswitch = True
            

            elif inst == b'@RNDM':
                
                x, y = random.uniform(-3,3), random.uniform(-3,3)
                theta = random.uniform(-1,1) * 3.14159

                print(f'Randomizing position... [{x}, {y}, {theta}]')

                dx, dy = x - self.position_odom[0], y - self.position_odom[1]
                dtheta = math.atan2(dy,dx)

                print('Rotating towards dtheta:', dtheta)
                self.movement_rotate_until(dtheta,tolerance=0.01,rotation_s=0.1)
                distance = math.sqrt(dx**2 + dy**2)
                print('Moving distance:', distance)
                self.movement_forward_until_distance(distance,tolerance=0.01,linear_s=0.22)
                self.movement_rotate_until(theta,tolerance=0.01,rotation_s=0.1)
                print(f'Movement Complete. {self.position_odom}')

                self.send_data('@RNDM')

            else: continue

        print('THREAD CLOSED')

    # TODO: modify send_data and receive_data to accept port parameter
        
    
    def movement(self,linear_x,angular_z): # helper function that instantiates turtlebot movement
    
        data = Twist()
        data.linear.x = linear_x
        data.angular.z = angular_z
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


    server_ip_address = "127.0.0.1" # localhost
    rclpy.init(args=args)
    sensors_subscriber = SensorsSubscriber(server_ip_address = server_ip_address)
    rclpy.spin(sensors_subscriber)
    sensors_subscriber.destroy_node()

if __name__ == '__main__':

    main()
