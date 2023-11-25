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

    def __init__(self, server_ip_address,transmit_to_server = True):

        super().__init__('sensors_subscriber')
        
        # Subscriptions
        # self.create_subscription(LaserScan, 'scan', self.scan_listener, qos_profile_sensor_data)
        # self.create_subscription(Twist,'cmd_vel', self.cmd_vel_listener, qos_profile_sensor_data)
        # ^ does not seem to contribute much -Gab
        self.create_subscription(Imu,'imu', self.imu_listener, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.odometry_listener, qos_profile_sensor_data)
        self.time_last_frame = 0
        
        # IMU variables:
        self.linear_vel_imu = (0,0,0)
        self.angular_vel_imu = (0,0,0)
        self.orientation_imu = (0,0,0)

        # Odometry variables:
        self.position_odom = (0,0,0)
        self.rotation_odom = (0,0,0)

        # Scanner variables:
        # empty for now
        # 
        self.last_time_frame = -1
        
        self.transmission_thread_sensor_data = threading.Thread(target=self.data_bridge_sensor_data)

        self.transmit_to_server = transmit_to_server
        if self.transmit_to_server:
            self.sock = socket.socket()
            server_ip = server_ip_address

            server_port = 50000; 
            flag_connected = False
            
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

        self.transmission_thread_sensor_data.start()

    # LIDAR is disabled for now, not needed for basic proof-of-concept

    def imu_listener(self, msg = None):
        
        if msg == None: return
            
        # print("Angular: ", msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z) # rad/s
        # print("Linear Acceleration: ", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z) # Measured in m/s^2
        # print("Orientation: ", msg.orientation.x, msg.orientation.y, msg.orientation.z) # meters?

        # assuming that the plane is level, orientation x and y are equal to zero, so might be truncated in full Turtlebot3 implementation TO CONFIRM
        # linear x, y, z do not indicate anything along with angular x, y, and z TO CONFIRM

        self.linear_vel_imu = (round(msg.linear_acceleration.x,2), round(msg.linear_acceleration.y,2), round(msg.linear_acceleration.z,2))
        self.angular_vel_imu = (round(msg.angular_velocity.x,2), round(msg.angular_velocity.y,2), round(msg.angular_velocity.z,2))
        self.orientation_imu = (round(msg.orientation.x,2), round(msg.orientation.y,2), round(msg.orientation.z,2))

        return
    
    def odometry_listener(self, msg = None):
        
        if msg == None: return

        self.position_odom = (round(msg.pose.pose.position.x,2), round(msg.pose.pose.position.y,2), round(msg.pose.pose.position.z,2))
        self.rotation_odom = (round(msg.pose.pose.orientation.x,2), round(msg.pose.pose.orientation.y,2), round(msg.pose.pose.orientation.z,2))

        return
    
    def data_bridge_sensor_data(self):

        if not self.transmit_to_server: return # error handling
        while True:

            data = f"{self.linear_vel_imu},{self.angular_vel_imu},{self.orientation_imu}///{self.position_odom},{self.rotation_odom}".encode()
            # ^it is important to round the data to 2 decimal places to keep input prompts the the LLM small and concise.
            
            data_len = str(len(data))
            data_len = ('@' + data_len + 'X' * (8 - len(data_len))).encode() # padded on the right, 1234XXXX as an example

            self.sock.send(data_len)
            print("sending data length:",data_len)
            self.sock.recv(5) # wait for reply from server
            print("sending data:",data)
            self.sock.sendall(data)
            time.sleep(0.1)

class VelocityServer(Node):

    """
    # This was created to simplify the process of harvesting data later on. Since simple actions are all that are needed
    for the POC, scripts need to be created to automate the movement of the Turtlebot, i.e., forward, backward, etc. Blend
    measured with human-controlled data perhaps?
    """

    def __init__(self, server_ip_address):

        self.sock = socket.socket()
        server_ip = server_ip_address
    
        server_port = 42000; 
        self.flag_connected = False
        
        try:
            self.sock.connect((server_ip, server_port))
            print("[VelocityServer] Connected to server.")
            self.flag_connected = True
        
        except Exception as e:
            print(e)
            print("[VelocityServer] Error has occured when trying to connect to server.")

        if not self.flag_connected:
            print("[VelocityServer] Exiting program...")
            sys.exit() # Close program if unable to connect to server

        self.listening_thread = threading.Thread(target=self.listen_for_orders)

    def listen_for_orders(self):

        if not self.flag_connected:
            return

        # TODO: fixed length movement
        # 5 bytes per entry in dict, 1 separators -> X.XXXX Y.YYYYY
        # when 2 or 3 digit: XX.XXX YYY.YY
        # total length per vel server input: 12 bytes long
        # VelocityServer sends ACK back to before client can send extra movement commands

        while True:

            self.sock.recv()

            
def main(args=None):

    server_ip_address = "192.168.68.60" # Gab's boarding house IP address
    rclpy.init(args=args)
    sensors_subscriber = SensorsSubscriber(server_ip_address=server_ip_address, transmit_to_server=True)
    rclpy.spin(sensors_subscriber)
    sensors_subscriber.destroy_node()

if __name__ == '__main__':

    main()