# tic-tac-toe-game
The project is illustrated with Franka panda and realsense 400 series. Table and walls added in the urdf to avoid collision for motion planning.

![Image](https://github.com/gesheng199832/tic-tac-toe-game/blob/master/image/I4XKC%7DE2UP(%256KVDFAXSU%40P.png)

# Project Setup
Make sure you've installed [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) and [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).

Please go to the each package under this repository for detailed explanations and installation guidance. Please install the [Realsense SDK](https://realsense.intel.com/sdk-2/#install) before you run the following steps.

For this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly
If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin build
```

Now that you have a workspace, clone or download this repo into the src directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/gesheng199832/tic-tac-toe-game.git
```

Now install missing dependencies using rosdep install:
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

Build the project:
```sh
$ cd ~/catkin_ws
$ catkin build
```

Add following to your .bashrc file:
```sh
$ source ~/catkin_ws/devel/setup.bash
```

# Hand eye calibration

```sh
$ roslaunch control_franak_test auto_calibration
$ cd ~/catkin_ws/src/tic-tac-toe-game/control_franka_test/calibration
$ python auto_calibration.py
```
copy the hand eye calibration matrix from terminal to main.py after auto calibration done

# Run the main function
Use the hand eye calibration results you've got from project2 and complete the following steps.
1. Launch the robot's movegroup node to monitor, control and visualize the robot. Please check the ip of you robot and replace the default value below.
```sh
$ roslaunch control_franka_test test
```
then run the computer vision server
```sh
% cd ~/catkin_ws/src/tic-tac-toe-game/control_franka_test/scripts
% python case_detecter_server_yzy.py
```
game start 
```sh
$ python main.py
```
