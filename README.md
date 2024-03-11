# NLNT Turtlebot System
A guide for setting up and using NLNT's Turtlebot Data Gathering system scripts

## Table of Contents
[Requirements](#requirements) <br/>
[Device Setup](#device-setup) <br/>
&nbsp;&nbsp;&nbsp;&nbsp;[A. Client PC](#a-client-pc) <br/>
&nbsp;&nbsp;&nbsp;&nbsp;[B. Virtual Turtlebot](#b-virtual-turtlebot) <br/>
&nbsp;&nbsp;&nbsp;&nbsp;[C. Real Turtlebot](#c-real-turtlebot) <br/>
&nbsp;&nbsp;[Conducting Data Gathering](#conducting-data-gathering) <br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Turtlebot](#turtlebot) <br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Client PC](#client-pc) <br/> 
[How to Use Data Annotator](#how-to-use-data-annotator) <br/>

## Requirements
- Clean Install of **Ubuntu 20.04 LTS**

## Device Setup

### A. Client PC
1. Install `git` on your PC

```bash
sudo apt update
sudo apt install git

# confirm installation worked correctly
git --version
```
2. Install `Github CLI` on your PC [(Official Guide)](https://github.com/cli/cli/blob/trunk/docs/install_linux.md)
*This is for easier authentication with our repos since they are set to Private in our ucl-nlnt Github Organization*
```bash
sudo mkdir -p -m 755 /etc/apt/keyrings && wget -qO- https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null \
&& sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg \
&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null \
&& sudo apt update \
&& sudo apt install gh -y
```

3. Install `ROS2 Foxy` by following the [official guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
> make sure to select **Foxy**
```bash
# install ros2 foxy packages
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh
sudo chmod 755 ./install_ros2_foxy.sh
bash ./install_ros2_foxy.sh

# install gazebo/cartographer/nav2 packages
sudo apt-get install ros-foxy-gazebo-*
sudo apt install ros-foxy-cartographer
sudo apt install ros-foxy-cartographer-ros
sudo apt install ros-foxy-navigation2
sudo apt install ros-foxy-nav2-bringup

# install turtlebot3 packages
source ~/.bashrc
sudo apt install ros-foxy-dynamixel-sdk
sudo apt install ros-foxy-turtlebot3-msgs
sudo apt install ros-foxy-turtlebot3

# build turtlebot3 packages from source code
sudo apt remove ros-foxy-turtlebot3-msgs
sudo apt remove ros-foxy-turtlebot3
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/turtlebot3_ws
colcon build --symlink-install
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc

# missing gazebo package from our lord and savior stack overflow
sudo apt-install ros-foxy-turtlebot3-gazebo

```

- Run a Gazebo simulation to verify using the [official guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)
```bash
cd ~/turtlebot3_ws/src/
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws && colcon build --symlink-install

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```
4. Login to the Github CLI using your account
```bash
gh auth login
```

5. Select a working directory for this project. Usual convention is to put it in the `Desktop` folder. Clone the repository here:
```bash
git clone https://github.com/ucl-nlnt/virtualttbot.git
```

6. Install `miniconda3` on your device using the [official guide](https://docs.anaconda.com/free/miniconda/miniconda-install/)
> Miniconda is a lighteweight package manager and environment management system for Python.

 - create a virtual environment for this repo
```bash
conda create --name vttbot
conda activate vttbot
```

 - Install **Python 3.8**
```bash
conda install python=3.8
```

 - Install the project's package dependencies
```bash
pip install -r requirements.txt
```

### B. Virtual Turtlebot
You don't need to do further setup as these are already discussed in [Client PC Setup](###a.-client-pc)

### C. Real Turtlebot
> TODO: Gab i'm not sure how to do setup hehe - Red

## Conducting Data Gathering

### Turtlebot
1. Open a terminal in the Turtlebot's working directory
- If using *Virtual Turtlebot*, `cd` into the `virtualttbot` working directory made during setup

- If using *Real Turtlebot*, connect to it using `ssh`
```bash
ssh ubuntu@<turtlebot_ip>
```
2. Run `bringup` to initiate the bot's sensors and servos
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```
> There can be issues expected when restarting the bot (such as during development). It is best to relaunch bringup every time

3. Run script for sending data 
```bash
python workspace/src/sensors/sensors/capstone.py
```

### Client PC
1. To select a webcam, run the helper script to view your device's available cameras

```bash
conda activate vttbot
python scripts/webcam_helper.py
```
> Take note of the index that you will use

2. Run the Pygame GUI
```bash
python workspace/src/sensors/sensors/annotator.py
```
> For MacOS, there are issues with multithreading support with Pygame. It is recommended to use Ubuntu instead

### How to use Data Annotator
> TODO: add instructions here