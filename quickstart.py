# Quickstart
# Runs all terminal commands to set up the system in one python file

# to be verified

import subprocess
from subprocess import Popen, PIPE
from multiprocessing.pool import ThreadPool

def run_in_terminal(cmds):
  for cmd in cmds:
    try:
      proc = subprocess.Popen(cmd, stdout=PIPE, shell=True)
    except:
      print("FAILED process: ", proc.stderr)
      #sys.exit(1)
  return


gazebo_or_realttb = input("Enter 'G/g' for Gazebo and 'T/t' for actual turtlebots:")

while gazebo_or_realttb != "G" and gazebo_or_realttb != "g" and gazebo_or_realttb != "T" and gazebo_or_realttb != "t":
    gazebo_or_realttb = input("Enter 'G/g' for Gazebo and 'T/t' for actual turtlebots:")

if  gazebo_or_realttb == "G" or gazebo_or_realttb == "g":
 
  # launch gazebo
  # launch_cmds = ["cd ~/turtlebot3_ws/src/", "git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git", "cd ~/turtlebot3_ws && colcon build --symlink-install", "export TURTLEBOT3_MODEL=burger", "ros2 launch turtlebot3_gazebo empty_world.launch.py"]
  launch_cmds = ["export TURTLEBOT3_MODEL=burger", "ros2 launch turtlebot3_gazebo empty_world.launch.py"]
  run_in_terminal(launch_cmds)
 
  # collecting data using gazebo
  cmd1 = ["ros2 launch turtlebot3_bringup robot.launch.py"]
  cmd2 = ["python workspace/src/sensors/sensors/capstone.py"]
  cmd3 = ["python workspace/src/sensors/sensors/annotator.py"]

  commands = [cmd1, cmd2, cmd3]
  with ThreadPool(3) as pool:
    pool.map(run_in_terminal, commands)

elif gazebo_or_realttb == "T" or gazebo_or_realttb == "t":
  # collecting data using actual turtlebots
  ttb_ip = input("Turtlebot IP:")

  cmd1 = ["ssh ubuntu@" + ttb_ip, "ros2 launch turtlebot3_bringup robot.launch.py"]
  cmd2 = ["ssh ubuntu@" + ttb_ip, "python workspace/src/sensors/sensors/capstone.py"]
  cmd3 = ["python workspace/src/sensors/sensors/annotator.py"]

  commands = [cmd1, cmd2, cmd3]

  with ThreadPool(3) as pool:
    pool.map(run_in_terminal, commands)