How to use:

Step 1: INSTALL ROS2 FOXY
- Notes: ROS2 Foxy will only work with `Ubuntu 20.04`, NOT 22.04 or any of the other versions of Ubuntu.
- For instructions, see the Foxy installation guide in: `https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#`
    > You do not need to install anything else in the Turtlebot except for the programs under `PC Setup in the Quickstart Guide`

Step 2: CLONE THE REPOSITORY
- If you do not have git installed, install it: `sudo apt install git`
- Clone the repository into a folder/directory of your choice:
    > Optional: `mkdir <some directory>`
    
    > `git clone https://github.com/ucl-nlnt/virtualttbot`
    
    > NOTE: Make sure you are logged into the github CLI before cloning, since this is a private repository!

Step 3: RUNNING THE DATA COLLECTING STUFF
- Open a terminal: `ctrl + alt + t`
- In the terminal, run: `ros2 launch turtlebot3_gazebo empty_world.launch.py`
- Open another terminal. In it, navigate to the workspace directory where you cloned the virtualttbot project: `cd SomePath/workspace`
- Compile the program: `colcon build && . install/setup.bash`
    > NOTE: You need to run this every time you restart your Ubuntu 20.04. I know it's silly, but otherwise ROS2 will not detect the program.
- In another terminal (again), navigate to where the same directory that contains workspace; i.e., the one that contains `data_receiver_pygame.py` and `auto_generate.py`
    > NOTE: To create your own automations with auto_generate.py, make sure that you edit `instructions.txt`. I have included examples in the commited file.
- Run either program with `python3 data_receiver_pygame.py` or `python3 auto_generate.py`
- Run the Turtlebot3-side program that allows the reciever/auto generator to interface with the Turtlebot: `ros2 run sensors mess_test`
