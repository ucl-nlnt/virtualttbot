# Gazebo Quickstart

import subprocess
import time

# For Custom Worlds
world = input("Enter World: ")
name = input("Enter Name: ")

if world == "":
	world = "empty_world"

# Launch Gazebo
launch_gazebo = subprocess.Popen('export TURTLEBOT3_MODEL=burger ; ros2 launch turtlebot3_gazebo ' + world + '.launch.py', stdout=subprocess.DEVNULL, shell=True)

# Launch Pygame
cmd_pygame = 'python3 ~/virtualttbot/workspace/src/sensors/sensors/auto_data_collector_lv12.py --name ' + name
launch_pygame = subprocess.Popen(cmd_pygame, shell=True)