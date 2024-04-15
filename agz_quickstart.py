# Gazebo Quickstart

import subprocess
import getch

# For Custom Worlds
world = input("Enter World: ")
name = input("Enter Name: ")

if world == "":
	world = "empty_world"

# Launch Gazebo
launch_gazebo = subprocess.Popen('export TURTLEBOT3_MODEL=burger ; ros2 launch turtlebot3_gazebo ' + world + '.launch.py', stdout=subprocess.DEVNULL, shell=True)

# Launch Pygame
cmd_pygame = 'python3 virtualttbot/workspace/src/sensors/sensors/auto_data_collector_lv12.py --name ' + name
launch_pygame = subprocess.Popen(cmd_pygame, stdin= subprocess.PIPE, stderr=subprocess.PIPE, stdout=subprocess.PIPE, shell=True)

# capture stdout and stderr
while True:
	x = getch.getch()
	out, err = launch_pygame.communicate(x)
	out, err = launch_gazebo.communicate(x)