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
        self.velocity_listener = self.create_subscription(Twist,'/cmd_vel',self.twist_listener,10)

        # IMU variables:
        # self.linear_vel_imu = (0,0,0)
        # self.angular_vel_imu = (0,0,0)
        # self.orientation_imu = (0,0,0)

        # Odometry variables:
        self.position_odom = [None,None,None]
        self.rotation_odom = None
        
        self.twist_msg_linear = [None, None] # x, y

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


    def twist_listener(self, msg = None):
        
        pass
    def odometry_listener(self, msg = None):

        if msg == None: return
        # Odometer listener
        if time.time() >= self.odometer_time_lock:

            self.position_odom = [msg.pose.pose.position.x, msg.pose.pose.position.y]
            self.rotation_odom = msg.pose.pose.orientation.z
            self.odometer_time_lock = time.time() + 1/10
            

            if not self.transmit_to_server: return # error handling


            export = self.position_odom + [self.rotation_odom]
            data = f"{export}".encode()
            # ^it is important to round the data to 2 decimal places to keep input prompts the the LLM small and concise.

            data_len = str(len(data))
            data_len = ('@' + data_len + 'X' * (8 - len(data_len))).encode() # padded on the right, 1234XXXX as an example

            self.sock.send(data_len)
            self.sock.recv(5) # wait for reply from server. not waiting for a reply floods the receiving server
            self.sock.sendall(data)

    def data_bridge_tx_TwistOdometry(self):
        pass
    """
    # This was created to simplify the process of harvesting data later on. Since simple actions are all that are needed
    for the POC, scripts need to be created to automate the movement of the Turtlebot, i.e., forward, backward, etc. Blend
    measured with human-controlled data perhaps?
    """

class VelocityServer:

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
    sensors_subscriber = SensorsSubscriber(server_ip_address = server_ip_address)
    rclpy.spin(sensors_subscriber)
    sensors_subscriber.destroy_node()

if __name__ == '__main__':

    main()
