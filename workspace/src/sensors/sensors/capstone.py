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

# from KNetworking import DataBridgeServer_TCP, DataBridgeClient_TCP

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

###
#   NOTE on USAGE: DataBridgeClient assumes that the Server is already running. Please ensure that it's setting up first!
###

# NOTE: added DataBridge objects here cause colcon build is messing up for some reason

def find_optimal_data_bisection(packet_size: int):

    tcp_max = 64000 # ensure that the limit is not reached
    return tcp_max - (tcp_max % packet_size)

def create_chance_for_error(error_type = socket.timeout, chance = 0.5): # used to simulater errors

    if random.random() <= chance: raise error_type

def send_data_subroutine(socket_object: socket.socket, data: bytes, 
                         destination_port: int, debug: bool = False, send_acks: bool = False,
                         packet_size: int = 1460, sim_errors: bool = False):

    if isinstance(data, str):
        data = data.encode()

    if not isinstance(data, bytes):
        raise TypeError("object.send_data() requires String or Bytes as input.")

    try:

        length_bytes = struct.pack('!I', len(data))
        if debug: print(f'[S: Destination:{destination_port}] Sending byte length...')

        # Bisect into segments if length is greater than ~64KB.
        data_length = len(data)
        remainder = data_length % find_optimal_data_bisection(packet_size) # may not by 64KB; depends on packet size

        socket_object.sendall(length_bytes) # tell destination total file size
        for i in range(0,data_length - remainder,packet_size):
            if send_acks:
                ack = socket_object.recv(2) # wait for other side to process data size
                if ack != b'OK': print(f'[S: Destination:{destination_port}] ERROR: unmatched send ACK. Received: {ack}')
                if debug: print(f'[S: Destination:{destination_port}] ACK good')

            if debug: print(f'[S: Destination:{destination_port}] Sending data...')
            socket_object.sendall(data[i:i+packet_size]) # send data
            if send_acks:
                ack = socket_object.recv(2) # wait for other side to process data size
                if ack != b'OK': print(f'[S: Destination:{destination_port}] ERROR: unmatched send ACK. Received: {ack}')
                if debug: print(f'[S: Destination:{destination_port}] ACK good')

        if remainder: 
            socket_object.sendall(data[data_length - remainder:])
            if debug: print(f"sending Remainder {remainder}")
            if send_acks:
                ack = socket_object.recv(2) # wait for other side to process data size
                if ack != b'OK': print(f'[S: Destination:{destination_port}] ERROR: unmatched send ACK. Received: {ack}')
                if debug: print(f'[S: Destination:{destination_port}] ACK good')

        if debug: print(f"Transmission done.")
        return "@SNOK"
    
    except socket.timeout as errormsg:
        print(f"[Sending Subroutine -> Destination: {destination_port}]: Timeout occured.")
        print(errormsg)
        return "@SNTO"
    
    except struct.error as errormsg:
        print(f"[Sending Subrouting -> Destination: {destination_port}]: Struct error occured.")
        print(errormsg)
        return "@SNSE"
    
    except BrokenPipeError as errormsg:
        print(f"[Sending Subroutine -> Destination: {destination_port}]: BrokenPipeError occured.")
        print(errormsg)
        return "@SNBP"
    
    except ConnectionResetError as errormsg:
        print(f"[Sending Subroutine -> Destination: {destination_port}]: ConnectionResetError occured.")
        print(errormsg)
        return "@SNCR"
        
    except ConnectionError as errormsg:
        print(f"[Sending Subroutine -> Destination: {destination_port}]: ConnectionError occured.")
        print(errormsg)
        return "@SNCE"
    
    

    
def receive_data_subroutine(socket_object: socket.socket, destination_port: int,
                            debug: bool = False, send_acks: bool = False, 
                            packet_size: int = 1460, sim_errors: bool = False):

    try:
    
        if debug: print(f'[R: Destination:{destination_port}] Waiting for byte length...')
        length_bytes = socket_object.recv(4)
        length = struct.unpack('!I', length_bytes)[0]
        if debug: print(f'[R: Destination:{destination_port}] Byte length received. Expecting: {length}')
        data, data_size = b'', 0

        if send_acks:
            socket_object.send(b'OK') # allow other side to send over the data
            if debug: print(f'[R: Destination:{destination_port}] ACK sent.')

        while data_size < length:

            chunk_size = min(packet_size, length - data_size)
            chunk_data = b''

            # Section added by GPT-4; apparently, socket.socket.recv() sometimes received partial datagrams only. weird.
            # the following while loop made sure that each iteration or each assumed TCP packet gave the complete data

            while len(chunk_data) < chunk_size:
                packet = socket_object.recv(chunk_size - len(chunk_data))
                if not packet:
                    raise RuntimeError("Connection closed by the server")
                chunk_data += packet

            data += chunk_data
            data_size += len(chunk_data)
            if debug: print(f"[R: Destination:{destination_port}] Received chunk of size {len(chunk_data)}. Total received: {data_size}")
            
        if send_acks:
            if debug: print(f'[R: Destination:{destination_port}] Transmission received successfull. Sending ACK')       
            socket_object.send(b'OK') # unblock other end
            if debug: print(f'[R: Destination:{destination_port}] ACK sent.')

        return data # up to user to interpret the data
    
    except struct.error as errormsg:
        print(f"[Receiving Subrouting -> Destination: {destination_port}]: Struct error occured.")
        print(errormsg)
        return "@RCSE"

    except socket.timeout as errormsg:
        print(f"[Receiving Subroutine -> Destination: {destination_port}]: Timeout occured.")
        print(errormsg)
        return "@RCTO"
    
    except ConnectionResetError as errormsg:
        print(f"[Receiving Subroutine -> Destination: {destination_port}]: ConnectionResetError occured.")
        print(errormsg)
        return "@RCCR"

    except BrokenPipeError as errormsg:
        print(f"[Receiving Subroutine -> Destination: {destination_port}]: BrokenPipeError occured.")
        print(errormsg)
        return "@RCBP"
    
    except ConnectionError as errormsg:
        print(f"[Receiving Subroutine -> Destination: {destination_port}]: ConnectionError occured.")
        print(errormsg)
        return "@RCCE"
    

class DataBridgeClient_TCP:
    
    # TODO: Add timeout functions and heartbeat

    def __init__(self, destination_port:int, destination_ip_address:str = "127.0.0.1", enable_acks:bool = False,debug:bool = False, packet_size:int = 1460):

        self.destination_port = destination_port
        self.destination_ip_address = destination_ip_address
        self.debug = debug
        self.enable_acks = enable_acks
        self.packet_size = packet_size
        self.client = None

        # SET TO FALSE TO TURN OFF RANDOM ERRORS
        self.simulate_errors = False

        self.reconnect_to_server()

    def reconnect_to_server(self):

        self.client = socket.socket()
            
        flag_connected = False

        # Attempt to connect to the server

        try:
            self.client.connect((self.destination_ip_address,self.destination_port))
            print(f'[DataBridgeClient TCP] Connected to {self.destination_ip_address}:{self.destination_port}.')
            flag_connected = True
        except Exception as e:
            print(f'[DataBridgeClient TCP -> {self.destination_ip_address}:{self.destination_port}] encountered an error:')
            print(e)
            flag_connected = False

        print(f'[DataBridgeClient TCP -> {self.destination_ip_address}:{self.destination_port}] : Connection = {flag_connected}')
        
        if not flag_connected: raise ConnectionError("Cannot connect to server.")

    def disconnect_from_server(self):
        
        self.client.close()

    def send_data(self, data: bytes):

        code = send_data_subroutine(socket_object=self.client, data=data, destination_port=self.destination_port, debug=self.debug, send_acks=self.enable_acks, packet_size=self.packet_size)
        if code == '@SNOK': return # no error; continue
    
        elif code == '@SNBP' or code == '@SNSE': # BrokenPipe or Struct.error

            # Other end closed the connection. Destroy the current connection object
            try:
                self.client.close()
            except:
                pass

        # TODO: find a way to restart data transfer on that frame
            
        elif code == '@SNCE' or code == '@SNCR' or code == "@SNTO": # ConnectionError, ConnectionResetError, TimeOut
            
            # restart the connection maybe?
            try:
                self.client.close()
            except:
                pass

            self.reconnect_to_server()
            self.send_data(data=data) # re-attempt to send data



    def receive_data(self):

        return receive_data_subroutine(self.client, destination_port=self.destination_ip_address, debug=self.debug, send_acks=self.enable_acks, packet_size=self.packet_size)

class DataBridgeServer_TCP:

    def __init__(self, port_number: int = 50000, enable_acks = False, debug:bool = False, packet_size: int = 1460):
        
        self.port_number = port_number
        
        self.enable_acks = enable_acks
        self.debug = debug
        self.packet_size = packet_size
        # TURN OFF TO DISABLE RANDOM ERRORS
        self.simulate_errors = False
        self.client = None
        self.client_address = None
        
        self.open_server_port()

    def open_server_port(self):
        
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # WARNING: MIGHT NEED TO CHECK IF PORT IS OCCUPIED
        self.server_sock.bind(('0.0.0.0', self.port_number))
        self.server_sock.listen(0)
        
        print(f"[DataBridgeServer TCP : {self.port_number}] Server started.")
        
        self.client, self.client_address = self.server_sock.accept()
        print(f"[DataBridgeServer TCP : {self.port_number}] Client connected from {self.client_address}.")
        print(f"[DataBridgeServer TCP : {self.port_number}] Setup complete.")

    def close_server_port(self):

        self.client.shutdown(socket.SHUT_RDWR)
        self.client.close()

    def cleanup(self):
        self.close_server_port()

    def send_data(self, data: bytes):

        code = send_data_subroutine(socket_object=self.client, data=data, destination_port=self.port_number, debug=self.debug, send_acks=self.enable_acks, packet_size=self.packet_size)
        if code == '@SNOK': return # no error; continue
    
        elif code == '@SNBP' or code == '@SNSE': # BrokenPipe or Struct.error

            # Other end closed the connection. Destroy the current connection object
            self.cleanup()

        # TODO: find a way to restart data transfer on that frame
            
        elif code == '@SNCE' or code == '@SNCR' or code == "@SNTO": # ConnectionError, ConnectionResetError, TimeOut
            
            # restart the connection maybe?
            self.close_server_port()        # close current instance
            self.open_server_port()         # restart server port instance
            self.send_data(data = data)     # restart data transfer

    def receive_data(self):

        data = receive_data_subroutine(socket_object=self.client, destination_port=self.port_number, debug=self.debug, send_acks=self.enable_acks, packet_size=self.packet_size)
        if isinstance(data, bytes):
            return data
        
        if data == '@RCBP' or data == '@RCSE': # BrokenPipe or Struct.error
            
            # Other end closed the connection. Destroy the current connection object
            self.cleanup()

        elif data == '@RCCE' or data == '@RCTO' or data == '@RCCR': # ConnectionError
            
            self.close_server_port()        # close current instance
            self.open_server_port()         # restart server port instance
            self.send_data(data = data)     # restart data transfer



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
        self.sampling_delay = 0.2

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
        
        time.sleep(0.5) # wait for the server to open the 2nd port.
        self.movement_instruction_client = DataBridgeClient_TCP(destination_ip_address="localhost",
                                                        destination_port=50001)
        
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
            elif inst == b'@FRWD': self.movement(0.22,0.0)
            elif inst == b'@LEFT': self.movement(0.0,0.75)
            elif inst == b'@RGHT': self.movement(0.0,-0.75)
            elif inst == b'@STRT': self.is_collecting_data = True
            elif inst == b'@STOP': self.is_collecting_data = False

            elif inst == b'@KILL': # terminate program
                self.killswitch = True
                break

            self.movement_instruction_client.send_data(b'@CONT') # give an ACK

    def send_to_server(self):

        while True:

            if self.super_json == None or not self.is_collecting_data: time.sleep(0.1); pass; # prevent excessive CPU usage when nothing is going on

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

            time.sleep(self.sampling_delay)
    
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
