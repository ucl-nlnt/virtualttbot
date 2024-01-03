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

    def __init__(self, server_ip_address, transmit_to_server = True):

        super().__init__('sensors_subscriber')

        # Subscriptions
        # self.create_subscription(LaserScan, 'scan', self.scan_listener, qos_profile_sensor_data)
        # self.create_subscription(Twist,'cmd_vel', self.cmd_vel_listener, qos_profile_sensor_data)
        # ^ does not seem to contribute much -Gab
        # self.create_subscription(Imu,'imu', self.imu_listener, qos_profile_sensor_data) # IMU data doesn't seem to be useful currently

        self.create_subscription(Odometry, 'odom', self.odometry_listener, qos_profile_sensor_data)
        self.movement_publisher = self.create_publisher(Twist, '/cmd_vel',10)
        
        # IMU variables:
        # self.linear_vel_imu = (0,0,0)
        # self.angular_vel_imu = (0,0,0)
        # self.orientation_imu = (0,0,0)

        # Odometry variables:
        self.position_odom = [None,None,None]
        self.rotation_odom = None
        self.killswitch = False

        # Instruction variables
        self.last_order = '@0000'
        self.time_last_order_issued = 0.0
        self.velocity_change = False

        # DEBUG LOCKS
        self.debug_send_data = False
        self.debug_odometer = False
        self.debug_randomizer = False
        self.debug_movement = False

        self.twist_msg = [None, None] # linear x, angular z

        # Scanner variables:
        # empty for now

        self.transmit_to_server = transmit_to_server

        if self.transmit_to_server:
            self.client = socket.socket()
            server_ip = server_ip_address

            server_port = 50000;
            flag_connected = False
            self.send_lock = True # signals that new data is allowed to be transmitted

            try:
                self.client.connect((server_ip, server_port))
                print("[SensorsSubscriber] Connected to server.")
                flag_connected = True

            except Exception as e:
                print(e)
                print("[SensorsSubscriber] Error has occured when trying to connect to server.")

            if not flag_connected:
                print("[SensorsSubscriber] Exiting program...")
                sys.exit() # Close program if unable to connect to server

        # threads
                
        self.transmission_thread = threading.Thread(target=self.data_bridge_tx_TwistOdometry)
        self.transmission_thread.start()
        if self.debug_odometer: self.odometer_printer_thread = threading.Thread(target=self.odometer_printer); self.odometer_printer_thread.start()
        print('INIT COMPLETE')

    def twist_listener(self, msg = None):
        
        if msg == None: return
        
        self.twist_msg = [msg.linear.x, msg.angular.z]

    def odometry_listener(self, msg = None):

        if msg == None: return
        # Odometer listener

        self.position_odom = [msg.pose.pose.position.x, msg.pose.pose.position.y, quaternion_to_yaw(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)] # x, y, phi
        # the orientation is from -pi to pi
        # 0 is facing north. pi/2 is facing left. -pi/2 facing right. -pi OR pi is facing right behind.
    def odometer_printer(self):

        while not self.killswitch:
            
            print(self.position_odom)
            time.sleep(1)

    def data_bridge_tx_TwistOdometry(self):

        if not self.transmit_to_server: return # error handling
        data_is_collecting = True


        while data_is_collecting:

            inst = self.receive_data()
            print(inst)
            if inst == b'@STRT': self.send_data(f'{self.position_odom}')
            elif inst == b'@0000': self.movement(0.0,0.0)
            elif inst == b'@FRWD': self.movement(0.22,0.0)
            elif inst == b'@LEFT': self.movement(0.0,0.75)
            elif inst == b'@RGHT': self.movement(0.0,-0.75)
            elif inst == b'@ODOM': 
                self.send_data(f'{self.position_odom}')
            elif inst == b'@STOP': self.movement(0.0,0.0); data_is_collecting = False
            elif inst == b'@KILL':
                print('exiting...') 
                data_is_collecting = False 
                self.killswitch = True
            elif inst == b'@ROTT': # used during automated data-gathering
                self.movement_rotate_until(float(self.receive_data().decode()),tolerance=0.1,rotation_s=0.1)
            elif inst == b'@RNDM':
                
                x, y = random.uniform(-5,5), random.uniform(-5,5)
                theta = random.uniform(-1,1) * 3.14159

                print(f'Randomizing position... {x} | {y} | {theta}')

                dx, dy = x - self.position_odom[0], y - self.position_odom[1]
                dtheta = math.atan2(dy,dx)

                self.movement_rotate_until(dtheta,tolerance=0.1,rotation_s=0.1)
                distance = math.sqrt(dx**2 + dy**2)
                self.movement_forward_until_distance(distance,tolerance=0.1,linear_s=0.22)
                self.movement_rotate_until(theta,tolerance=0.1,rotation_s=0.1)
                print(f'Movement Complete. {self.position_odom}')

                self.send_data('@RNDM')

            else: continue

        print('THREAD CLOSED')

    def send_data(self, data: bytes):

        if isinstance(data, str):
            data = data.encode()

        # NOTE: data may or may not be in string format.
            
        length_bytes = struct.pack('!I', len(data))
        
        if self.debug_send_data: print('[S] Sending byte length...')
        self.client.sendall(length_bytes)
        ack = self.client.recv(2) # wait for other side to process data size
        if ack != b'OK': print(f'[S] ERROR: unmatched send ACK. Received: {ack}')
        if self.debug_send_data: print('[S] ACK good')

        if self.debug_send_data: print('[S] Sending data...')
        self.client.sendall(data) # send data
        if self.debug_send_data: print('[S] Data sent; waiting for ACK...')
        ack = self.client.recv(2) # wait for other side to process data size
        if ack != b'OK': print(f'[S] ERROR: unmatched send ACK. Received: {ack}')
        if self.debug_send_data: print('[S] ACK good. Data send success.')

    def receive_data(self):

        # NOTE: Returns data in BINARY. You must decode it on your own

        if self.debug_send_data: print('[R] Waiting for byte length...')
        length_bytes = self.client.recv(4)
        length = struct.unpack('!I', length_bytes)[0]
        if self.debug_send_data: print(f'[R] Byte length received. Expecting: {length}')
        data, data_size = b'', 0

        self.client.send(b'OK') # allow other side to send over the data
        if self.debug_send_data: print(f'[R] ACK sent.')
        while data_size < length:

            chunk_size = min(2048, length - data_size)
            data_size += chunk_size
            data += self.client.recv(chunk_size)
            if self.debug_send_data: print(f'[R] RECV {chunk_size}')

        if self.debug_send_data: print('[R] Transmission received successfull. Sending ACK')       
        self.client.send(b'OK') # unblock other end
        if self.debug_send_data: print('[R] ACK sent.')
        return data # up to user to interpret the data
    
    def movement(self,linear_x,angular_z): # helper function that instantiates turtlebot movement
    
        data = Twist()
        data.linear.x = linear_x
        data.angular.z = angular_z
        self.movement_publisher.publish(data)
        
        return

    def movement_rotate_until(self, target_radians, tolerance=0.1, rotation_s=0.05):

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
            if diff_radians > 0:
                rotation_speed = rotation_s  # Positive value for clockwise rotation
            else:
                rotation_speed = -rotation_s  # Negative value for counterclockwise rotation

            # Execute the rotation
            if self.debug_randomizer: print(f'rotating to goal... current: {current_radians}, goal: {target_radians}')
            self.movement(0.0, rotation_speed)
            time.sleep(0.25)

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
