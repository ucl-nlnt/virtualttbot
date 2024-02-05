
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

from sensor_msgs.msg import Imu

def quaternion_to_euler(quaternion):

    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """

    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

class IMUListner(Node):

    def __init__(self):
        super().__init__('imu_listener')
        self.imu_listener = self.create_subscription(
            Imu,
            '/imu',
            self.imu_listener_callback,
            10
        )

    def imu_listener_callback(self, msg):
        
        if msg == None:
            return # prevent bad erroneous msg
        
        roll, pitch, yaw = quaternion_to_euler(msg.orientation)

        self.get_logger().info(f'Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}')
        time.sleep(1)
        
def main(args = None):
    rclpy.init(args=args)
    imu_listener = IMUListner()
    rclpy.spin(imu_listener)
    imu_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
