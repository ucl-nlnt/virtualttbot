import rclpy
import time
import threading
import socket
import sys
import struct
import random
import math
import ast

from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist

def quaternion_to_yaw(x, y, z, w):

    # AI-generated with GPT-4

    """
    Convert a quaternion into yaw (rotation around z axis in radians)
    yaw is counterclockwise
    """
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return yaw_z # in radians

class InferenceServerBridge(Node):

    def __init__(self):

        super().__init__('sensors_subscriber')

        self.create_subscription(Odometry, 'odom', self.odometry_listener, qos_profile_sensor_data)
        self.movement_publisher = self.create_publisher(Twist, '/cmd_vel',10)
        self.killswitch = False
        self.position_odom = [None, None, None] # [x, y, phi]

        # debug variables
        self.debug_send_data = False
        self.debug_randomizer = False
        self.debug_movement = False

        try:
            self.client = socket.socket()
            server_ip, server_port = "10.158.18.253", 50000
        
            self.client.connect((server_ip,server_port))
            print(f"Connected to remote server at {server_ip}:{server_port}.")

        except Exception as e:
            print('Exception has occured with error message while connecting to inference server:')
            print(e)
            sys.exit(0)

        self.transmission_thread = threading.Thread(target=self.inference)
        self.transmission_thread.start()

        print('INIT COMPLETE.')
    
    def odometry_listener(self, msg):

        if msg == None: return
        # Odometer listener

        self.position_odom = [msg.pose.pose.position.x, msg.pose.pose.position.y, quaternion_to_yaw(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)] # x, y, phi
        # the orientation is from -pi to pi
        # 0 is facing north. pi/2 is facing left. -pi/2 facing right. -pi OR pi is facing right behind.

    def inference(self):

        print('Waiting for odometer to warm up...')
        while self.position_odom == [None, None, None]: pass

        print('Odometry acquired.')

        while not self.killswitch:
            
            input_data = input("Give a natural language prompt: ")
            input_data = f"{input_data}//##//{[round(i,3) for i in self.position_odom]}"
            self.send_data(input_data.encode('utf-8'))
            print('Waiting for inference response. Please wait...')
            
            t = time.time()

            inference_data = self.receive_data()
            inference_data = inference_data.decode()

            print(f"Inference took {round(time.time() - t, 2)} seconds.")

            print('Raw inference:')
            print('//////////////////////////////////////')
            print(inference_data)
            print('//////////////////////////////////////')

            inference_data = inference_data[inference_data.find("### Answer:") + len("### Answer:"):].strip()
            inference_data = inference_data[:inference_data.find(']]') + 2]
            
            inference = ast.literal_eval(inference_data)
            previous_coords = [round(i,3) for i in self.position_odom]
            for i in inference:

                if i == ['STOP'] or i[0] == 'STOP':
                    print('MOVEMENT COMPLETE')
                    break

                x2, y2, phi2 = i
                x1, y1, phi1 = previous_coords


                if not (math.isclose(phi2, phi1, abs_tol=0.1)):
                    print('Rotating to phi.')
                    self.movement_rotate_until(phi2,tolerance=0.05,rotation_s=0.1)
                    print('Rotation done.')

                if not (math.isclose(x2,x1,abs_tol=0.2) and math.isclose(y2,y1,abs_tol=0.2)):
                    
                    print('Moving to coordinates [FORWARD]')
                    dtheta = math.atan2((y2 - y1),(x2 - x1))
                    self.movement_rotate_until(dtheta,tolerance=0.05,rotation_s=0.1)
                    print(math.sqrt((x2 - x1)**2 + (y2 - y1)**2))
                    self.movement_forward_until_distance(target_distance=math.sqrt((x2 - x1)**2 + (y2 - y1)**2), tolerance=0.1, linear_s=0.22)                
                    print('Movement complete.')

                print(f'Current Position: {[round(i,2) for i in self.position_odom]}')
                previous_coords = [round(i,2) for i in self.position_odom]

    def send_data(self, data: bytes):

        if isinstance(data, str):
            data = data.encode()

        # NOTE: data may or may not be in string format.
        try:
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

        except Exception as e:
            print(e)
            self.killswitch = True
            return None
        
    def receive_data(self):

        # NOTE: Returns data in BINARY. You must decode it on your own
        try:
        
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
        
        except Exception as e:
            print(e)
            self.killswitch = True
            return None
        
    def movement(self,linear_x,angular_z): # helper function that instantiates turtlebot movement
    
        data = Twist()
        data.linear.x = linear_x
        data.angular.z = angular_z
        self.movement_publisher.publish(data)
        
        return

    def movement_rotate_until(self, target_radians, tolerance=0.1, rotation_s=0.05):

        """
        dirrection_right = True
        current_radians = self.position_odom[2]
        t1 = target_radians - current_radians
        t2 = current_radians - target_radians
        
        if t1 < 0: t1 += 2 * math.piif msg == None: return
        # Odometer listener

        self.position_odom = [msg.pose.pose.position.x, msg.pose.pose.position.y, quaternion_to_yaw(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)] # x, y, phi
        # the orientation is from -pi to pi
        # 0 is facing north. pi/2 is facing left. -pi/2 facing right. -pi OR pi is facing right behind.
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
    bridge = InferenceServerBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()

if __name__ == '__main__':

    main()