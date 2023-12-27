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

class VelocityListener(Node):

    def __init__(self):

        super().__init__('cmd_vel')

        self.velocity_listener = self.create_subscription(Twist,'/cmd_vel',self.twist_callback,10)

    def twist_callback(self,msg = None):
        print(msg.linear)
        print(msg.angular)



def main(args=None):
    rclpy.init(args=args)
    vel = VelocityListener()
    rclpy.spin(vel)
    vel.destroy_node()

if __name__ == '__main__':
    main()
    
