# Mostly Copy-pasted from capstone.py

import rclpy
import time
import threading
import math
import json

from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import numpy as np
from copy import deepcopy
from prompt_randomizer import prompt_randomizer
from KNetworking import DataBridgeClient_TCP, DataBridgeServer_TCP
import argparse
import ast
import zlib
import uuid
import os
import time
import sys

from new_prompt_randomizer import prompt_maker

parser = argparse.ArgumentParser()

parser.add_argument("--ip", type=str, default="localhost")
parser.add_argument("--port", type=int, default=50000) # change this if you wanna use a real and simulated turtlebot at the same time
parser.add_argument("--sampling_hz", type=int, default=10)
parser.add_argument("--max_linear_x", type=float, default=0.2)
parser.add_argument("--max_angular_z", type=float, default=1.75)
parser.add_argument("--threshold_linear", type=float, default=0.01)
parser.add_argument("--threshold_angular", type=float, default=0.01)
parser.add_argument("--samples", type=int, default=1000)
parser.add_argument("--debug", type=int, default=0)
parser.add_argument("--comm", type=int, default=1)
parser.add_argument("--sampler", type=int, default=1)
parser.add_argument("--name", help="username")

args = parser.parse_args()
print(args)

if not os.path.exists("datalogs_auto"):
    os.mkdir("datalogs_auto")

def compute_distance(coord1, coord2):

    total = 0
    for i in range(3):
        total += (coord2[i] - coord1[i])**2
    
    return math.sqrt(total)


def quaternion_to_yaw(x, y, z, w): # Generated by GPT-4

    """
    Convert a quaternion into yaw (rotation around z-axis in radians)
    """
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def yaw_difference(quaternion1, quaternion2): # Generated by GPT-4
    """
    Calculate the difference in yaw between two quaternions
    """
    yaw1 = quaternion_to_yaw(*quaternion1)
    yaw2 = quaternion_to_yaw(*quaternion2)
    
    # Calculate the difference and adjust for the circular nature of angles
    difference = yaw2 - yaw1
    difference = (difference + math.pi) % (2 * math.pi) - math.pi
    
    return difference

def normalize_radians(float_num):
    return (float_num + np.pi) % (2 * np.pi) - np.pi

def generate_trial_instruction():
    #prompt = prompt_randomizer.prompt_maker()

    maker = prompt_maker()
    prompt = maker.maker()

    x = prompt[1]
    instructions = []
    #for i in x:
    #    q = i.split(', ')
    #    s = '['
    #    for j in range(0,len(q),2):
    #        s += q[j].replace("(",'("') + '", ' + q[j+1]
    #        if j + 2 != len(q):
    #            s += ", "
    #    s += ']'
    #    instructions.append(s)
        #print(s)

    #instructions_parsed = []
    #for i in instructions:
    #    instructions_parsed += ast.literal_eval(i)

    #return (prompt[0], instructions_parsed, prompt[2])
    return (prompt[0], prompt[1], prompt[2])

def generate_random_filename():

    random_filename = str(uuid.uuid4().hex)[:16]
    return random_filename

class AutoDataCollector(Node):

    def __init__(self, comm_on = True, sampler_on = True, debug = False):

        super().__init__('AutoDataCollector_lv1_lv2')

        if args.name == None:
            self.user = input("Enter your name for logging purposes: ")
        else:
            self.user = args.name

        self.movement_publisher = self.create_publisher(Twist, '/cmd_vel',10)
        self.create_subscription(Odometry, 'odom', self.odometer_callback, qos_profile_sensor_data)  
        self.create_subscription(Twist,'cmd_vel', self.twist_callback, qos_profile_sensor_data)
        self.create_subscription(Imu,'imu', self.imu_callback, qos_profile_sensor_data) # IMU data doesn't seem to be useful currently // Gab

        self.starting_odometry_set = False
        # class-wide variables to be used
        self.distance_traveled = 0.0
        self.radians_rotated = 0.0
        self.odometry_msg_data = (None, None, None, None)
        self.odometry_msg_data_pos = (None, None, None)
        self.odometry_msg = None

        self.twist_timestamp = None
        self.last_twist_timestamp = None

        # TODO: implement for real-life turtlebot
        # self.data_transmitter = DataBridgeClient_TCP(destination_port=args.port, destination_ip_address=args.ip)
        # self.data_receiver = DataBridgeClient_TCP(destination_port=args.port + 1, destination_ip_address=args.ip)
        
        self.collect_data = False
        self.data_frame_buffer = []

        # data messages init
        self.super_json = None
        self.sampling_rate_hz = args.sampling_hz
        self.imu_msg = None
        self.twist_msg = None
        self.sampling_start = False

        # multithreading stuff
        if sampler_on:
            self.transmitter = threading.Thread(target=self.sampling_process) # sends data to controller
            self.transmitter.daemon = True
            self.transmitter.start()
        if comm_on:
            self.controller = threading.Thread(target=self.communicator)      # receives instructions from controller
            self.controller.daemon = True
            self.controller.start()
        if debug:
            self.debug_thread = threading.Thread(target=self.debug)
            self.debug_thread.daemon = True
            self.debug_thread.start()

    def debug(self):

        while not self.sampling_start: time.sleep(0.1) # wait for data to be ready

        # add commands here to debug

        print(self.rotate_x_radians_right(3 * math.pi + 1))
        print(self.rotate_x_radians_left(3 * math.pi + 1))

        sys.exit(0)

    def twist_callback(self,msg):
        if msg == None: return
        self.twist_msg = msg

    def imu_callback(self,msg):

        if msg == None: return
        self.imu_msg = msg

    def sampling_process(self):
        
        # wait for the msgs to reead
        while self.imu_msg == None: time.sleep(0.1)
        while self.odometry_msg_data == None: time.sleep(0.1)
        while self.odometry_msg_data_pos == None: time.sleep(0.1)
        while self.odometry_msg == None: time.sleep(0.1)
        
        self.__publish_twist_message(0.0, 0.0)

        twist_msg_jsonized = { # default
                    "linear":(0.0, 0.0, 0.0),
                    "angular":(0.0, 0.0, 0.0),
                    "time":self.twist_msg
                }
        
        self.sampling_start = True # unlocks data collection to ensure that there is data from the sensors
        while True:
           
            if not self.collect_data:
                time.sleep(0.05) # wait until unlocked
                continue

            imu_msg_jsonized = {
                "quarternion_orientation": (self.imu_msg.orientation.x, self.imu_msg.orientation.y, self.imu_msg.orientation.z, self.imu_msg.orientation.w),
                "orientation_covariance": [i for i in self.imu_msg.orientation_covariance],
                "angular_velocity": (self.imu_msg.angular_velocity.x, self.imu_msg.angular_velocity.y, self.imu_msg.angular_velocity.z),
                "angular_velocity_covariance": [i for i in self.imu_msg.angular_velocity_covariance],
                "linear_acceleration": (self.imu_msg.linear_acceleration.x, self.imu_msg.linear_acceleration.y, self.imu_msg.linear_acceleration.z),
                "linear_acceleration_covariance": [i for i in self.imu_msg.linear_acceleration_covariance]
            }
            
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

            if self.twist_msg != None:
                twist_msg_jsonized = {
                    "linear":(self.twist_msg.linear.x, self.twist_msg.linear.y, self.twist_msg.linear.z),
                    "angular":(self.twist_msg.angular.x, self.twist_msg.angular.y, self.twist_msg.angular.z),
                    "time":self.twist_timestamp
                }

            # if need laser scan data, will add in the future
            
            self.super_json = { 
            "laser_scan" : None, "twist":twist_msg_jsonized, "imu":imu_msg_jsonized, 
            "odometry":odometry_msg_jsonized, "battery":None, "frame_data":None, "distance_traveled":self.distance_traveled, "radians_rotated":self.radians_rotated
            }
            
            self.data_frame_buffer.append(self.super_json)
            time.sleep(1/args.sampling_hz/2)
            
    def communicator(self):
        
        total_move = 0
        total_left = 0
        total_right = 0

        total_error_move = 0
        total_error_left = 0
        total_error_right = 0

        max_error_move = 0
        max_error_left = 0
        max_error_right = 0
        loop_num = 1
        iterations = args.samples

        while not self.sampling_start: time.sleep(0.1); # wait for sensors to warm-up
        while iterations:

            self.last_twist_timestamp = None
            # check current coordinates
            x, y, _ = self.odometry_msg_data_pos
            print("Final coords:", x, y)

            if abs(x) > 3 or abs(y) > 3:

                print("Returning to origin...")
                delta, distance = math.atan2(-y, -x), math.sqrt(x**2 + y**2)
                print(delta, distance)
                self.rotate_to_orientation(delta, args.max_angular_z)
                self.move_x_meters_feedback(distance, args.max_linear_x)
                print(f"Reset location to origin. {self.odometry_msg_data_pos[0:2]}")

            print('================================================')
            prompt, instructions, ground_truth_coords = generate_trial_instruction()
            self.collect_data = True
            t_start = time.time()
            time_total = 0.0
            print("Prompt:", prompt)
            print("Instructions:", instructions)

            for i in instructions:

                ins, mag = i
                #print(i)
                mag_copy = mag

                if ins == 'MOVE':

                    err = self.move_x_meters_feedback(mag)
                    total_error_move += err
                    if err > max_error_move: max_error_move = err
                    total_move += 1
                
                if ins == 'RGHT':

                    mag = float(mag) * math.pi / 180
                    err = abs(mag_copy - self.rotate(mag,'right'))
                    total_error_right += err
                    total_right += 1

                    if err > max_error_right: max_error_right = err

                if ins == 'LEFT':

                    mag = float(mag) * math.pi / 180
                    err = abs(mag_copy - self.rotate(mag,'left'))
                    total_error_left += err
                    total_left += 1
                    if err > max_error_left: max_error_left = err

                time.sleep(0.5) # allow turtlebot to decelerate to a stop
            

            self.collect_data = False
            # Save data into a .compressed file here
            fname = generate_random_filename() + '_auto.compressed'

            
            json_file = {
                "username":self.user + ' (simulated)', "natural_language_prompt":prompt,
                "timestamp_s":time.ctime(), "timestamp_float":time.time(), "ground_truth":ground_truth_coords,
                "simulation":1, "states":self.data_frame_buffer, "instructions" : instructions
            }

            iterations -= 1 # prevent a divide by zero error
            with open(os.path.join("datalogs_auto",fname),'wb') as f:

                f.write(zlib.compress(json.dumps(json_file, indent=4).encode('utf-8')))
                print(f"[{time.ctime()}]: Simulation saved under {fname}, Progress: {args.samples - iterations} / {args.samples} ({round((args.samples - iterations) / args.samples, 2)})")    
                q = time.time() - t_start
                time_total += q
                print(f"Time taken: {round(q,2)}; Avg. time per prompt: {round(time_total / loop_num,2)}")
            
            loop_num += 1
            self.data_frame_buffer = []

            # insert code here that returns the turtlebot to near (0.0,0.0) if it is beyond
            # the 6x6 box

            try:
                print("Average Errors:", round(total_error_move / total_move, 5), round(total_error_left / total_left,5), round(total_error_right / total_right,5))
                print("Max Errors:", max_error_move, max_error_left, max_error_right)
            except:
                pass

            
    def reset_distance_and_radians(self):

        self.distance_traveled = 0.0
        self.radians_rotated = 0.0

    def odometer_callback(self,msg):

        if msg == None: return
        
        if not self.starting_odometry_set: 
            self.starting_odometry_set = True
        else:
            q = self.odometry_msg.pose.pose.position
            x, y, z = q.x, q.y, q.z # save the values before proceeding
            q = self.odometry_msg.pose.pose.orientation
            quart = (q.x,q.y,q.z,q.w)

        self.odometry_msg = msg
        self.odometry_msg_data = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.odometry_msg_data_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

        try:

            q = self.odometry_msg.pose.pose.position
            x1, y1, z1 = q.x, q.y, q.z # save the values before proceeding
            q = self.odometry_msg.pose.pose.orientation
            quart2 = (q.x,q.y,q.z,q.w)
            self.distance_traveled += math.sqrt((x1-x)**2 + (y1-y)**2 + (z1-z)**2)
            self.radians_rotated += round(abs(yaw_difference(quart2, quart)),4)
            # print((round(self.distance_traveled,2),round(self.radians_rotated * 180 / math.pi,2)))

        except Exception as e:
            print("Error encountered on odometer callback:")
            print(e)
            pass

    def print_log(self, mess:str):

        print(f"[AutoDataCollector: {time.ctime()}] " + mess)

    def __publish_twist_message(self, linear_x, angular_z):


        data = Twist()
        data.linear.x = linear_x
        data.angular.z = angular_z
        self.movement_publisher.publish(data)
        
        self.twist_timestamp = time.time()
        if self.last_twist_timestamp != None:
            print("Twist time delay:",self.twist_timestamp - self.last_twist_timestamp)
        self.last_twist_timestamp = self.twist_timestamp

        return

    def move_x_meters_feedback(self, distance, slowdown_threshold=0.4, vel_x:float = 0.2): # accurate, works up to +- 0.005 meters

        # implement here
        start = self.odometry_msg_data_pos
        self.distance_traveled = 0.0
        switch = True

        while distance - slowdown_threshold > self.distance_traveled:
            if switch:
                self.__publish_twist_message(vel_x,0.0)
                switch = False
        

        self.__publish_twist_message(0.0,0.0)
        time.sleep(0.5)

        switch = True
        while distance > self.distance_traveled:
            if switch:
                self.__publish_twist_message(0.1,0.0)
                switch = False

        self.__publish_twist_message(0.0,0.0)
        
        time.sleep(0.5)
        end = self.odometry_msg_data_pos
        print("Distance error:",round(distance-compute_distance(start, end), 4))

        return distance-compute_distance(start, end)

    def rotate(self, angular_distance_radians, direction, slowdown_threshold = 0.4, angl_vel_z:float = 1.0):

        print("Direction:", direction)
        self.radians_rotated = 0.0
        starting = self.odometry_msg_data

        switch = True # this is done to get the min and max angular values
        while angular_distance_radians-slowdown_threshold > self.radians_rotated:

            if direction.lower() == "left":
                
                if switch:
                    switch = False
                    self.__publish_twist_message(0.0, angular_z=angl_vel_z)
            elif direction.lower() == "right":
                
                if switch:
                    switch = False
                    self.__publish_twist_message(0.0, angular_z=-angl_vel_z)
            else:
                print("ERROR: invalid direction")
                return
            time.sleep(0.1)

        self.__publish_twist_message(0.0,0.0)
        time.sleep(0.5) # simulate inference

        switch = True
        while angular_distance_radians > self.radians_rotated:

            if direction.lower() == "left":

                if switch:
                    switch = False
                    self.__publish_twist_message(0.0, angular_z=0.2)
            elif direction.lower() == "right":

                if switch:
                    switch = False
                    self.__publish_twist_message(0.0,angular_z=-0.2)
            else:
                print("ERROR: invalid direction")
                return
            time.sleep(0.1)

        self.__publish_twist_message(0.0,0.0)
        time.sleep(0.5)

        ending = self.odometry_msg_data
        
        print(f"Rotated {direction} degrees [Internal]:",round(self.radians_rotated * 180 / math.pi,3))
        return round(self.radians_rotated * 180 / math.pi,3)
    
    def rotate_to_orientation(self, angle_in_radians: float, angl_vel_z: float = 1.5):
        
        desired = normalize_radians(angle_in_radians) # prevent unattainable cases
        current = normalize_radians(quaternion_to_yaw(*self.odometry_msg_data))

        delta = desired - current

        if math.isclose(delta,0,rel_tol=args.threshold_angular):
            return 
        
        elif delta > 0: # rotate left
    
            self.rotate(abs(delta), "left")

        else: # delta < 0

            self.rotate(abs(delta), "right")

def main(args=None, n_args=args):

    rclpy.init(args=args)

    sensors_subscriber = AutoDataCollector(comm_on=n_args.comm, sampler_on=n_args.sampler, debug=n_args.debug)
    rclpy.spin(sensors_subscriber)
    sensors_subscriber.destroy_node()

if __name__ == '__main__':

    main()