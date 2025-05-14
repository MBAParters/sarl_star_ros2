# sarl_star_ros2
ROS implementation of the paper [SARL*: Deep Reinforcement Learning based Human-Aware Navigation for Mobile Robot in Indoor Environments](https://ieeexplore.ieee.org/abstract/document/8961764) presented in ROBIO 2019. This mobile robot navigation framework is implemented on a Turtlebot2 robot platform with lidar sensors (Hokuyo or RPlidar), integrating SLAM, path planning, pedestrian detection and deep reinforcement learning algorithms.

## Introduction
We present an advanced version of the Socially Attentive Reinforcement Learning (SARL) algorithm, namely SARL*, to achieve human-aware navigation in indoor environments. Recently, deep RL has achieved great success in generating human-aware navigation policies. However, there exist some limitations in the real-world implementations: the learned navigation policies are limited to certain distances associated with the training process, and the simplification of the environment neglects obstacles other than humans. In this work, we improve the SARL algorithm by introducing a dynamic local goal setting mechanism and a map-based safe action space to tackle the above problems. 

## Method Overview
![For more details, please refer to the paper.](https://github.com/MBAParters/sarl-star-ros2/blob/master/imgs/overview.png)

## Video
![](https://github.com/MBAParters/sarl_star_ros2/blob/master/imgs/example.gif)

## Code Structure
- **[Python-RVO2](https://github.com/sybrenstuvel/Python-RVO2)**: Crowd simulator using Optimal Reciprocal Collision Avoidance algorithm.
- [**laser_filters**](http://wiki.ros.org/laser_filters): ROS package to filter out unwanted laser scans. (**optional**)
 - **[navigation2](https://wiki.ros.org/nav2_navigation)**: Modified ROS navigation stack to provide AMCL localization, costmaps and basic path planners. **Note that** our dynamic local goal setting algorithm is implemented in navigation/dwa_local_planner/src/dwa_planner_ros.cpp. Therefore, if you have installed the original ROS navigation stack before, we suggest that you uninstall it and build the stack in our repository (following the steps in the next part "Build & Install") to make sure that our modification make effect.
 - **[people](http://wiki.ros.org/people)**: ROS stack to detect and track humans using sensor information.
  - **sarl_star_ros2** : Core ROS package to run the SARL* navigation algorithm.
 - **[turtlebot4](https://wiki.ros.org/Robots/TurtleBot4)**: ROS stack to use ROS with TurtleBot.

## Build & Install
Our codes have been tested in Ubuntu 22.04 with Python 3.10.12. 
1. Install [ROS2 humble](http://wiki.ros.org/humble/Installation/Ubuntu).
2. Create and build a catkin workspace and download the codes into src/:
```
mkdir -p ~/sarl_ws/src
cd ~/sarl_ws/
cd src
git clone https://github.com/MBAParters/sarl-star-ros2.git
```
3. Install other dependencies:

```
sudo apt-get install libbullet-dev
sudo apt-get install libsdl-image1.2-dev
sudo apt-get install libsdl-dev
sudo apt-get install ros-humble-bfl
sudo apt-get install ros-humble-tf2-sensor-msgs
pip install empy
pip install configparser
```
4. Install [Python-RVO2](https://github.com/sybrenstuvel/Python-RVO2):

```
cd sarl_star/Python-RVO2/
pip install -r requirements.txt
python setup.py build
python setup.py install
```
5. Install CrowdNav (Note that the CrowdNav in this repository are modified from [the original SARL implementation](https://github.com/vita-epfl/CrowdNav)):

```
cd sarl_star/sarl_star_ros/CrowdNav/
pip install -e .
```

6. Build the workspace:

```
cd ~/sarl_ws/
colcon build
source devel/setup.bash
```

## Start the Navigation

1. Bringup the Simulation Environment
```
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py
```

2. Build a map of your environment using [slam_toolbox](https://wiki.ros.org/slam_toolbox) package:
```
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true rviz:=true
```
Then push or tele-operate the robot to explore the environment and build a map. You will be able to view the real-time navigation in [RVIZ](http://wiki.ros.org/rviz). 
To save the map, open a new terminal and run:
```
mkdir -p ~/sarl_ws/src/sarl_star/sarl_star_ros/map
cd ~/sarl_ws/src/sarl_star/sarl_star_ros/map
 ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'new_map'}}"
```

3. Start navigation using the SARL* policy:
To start the SARL* navigation, run
```
ros2 launch sarl_star_ros2 sarl_star_navigation.launch.py
ros2 run sarl_star_ros2 sarl_star_node
```
You will see the rviz window which shows the robot navigation in real time. Draw a goal pose in the map (following the instructions in [ROS RVIZ tutorials](http://wiki.ros.org/navigation/Tutorials/Using%20rviz%20with%20the%20Navigation%20Stack)), the robot will navigate using the SARL* policy, which will avoid humans (blue ballls) and static obstacles in the map, and dynamically update its local goal (red cubes) according to the global plan. The action commands given to the robot can be seen in rviz as green arrows.

## Reference
- [1] Everett M., et al."Motion Planning Among Dynamic, Decision-Making Agents with Deep Reinforcement Learning". IROS 2018.
- [2] Long P ., et al.Deep Reinforcement Learning". ICRA 2018."Towards Optimally Decentralized Multi-Robot Collision Avoidance via
- [3] Chen Y ., et al."Socially Aware Motion Planning with Deep Reinforcement Learning".IROS 2017.
- [4] Helbing D., Molnar P ."Social force model for pedestrian dynamics". Physical Review E1995.
- [5] V an den Berg J., et al."Reciprocal n-body Collision Avoidance". ISRR 2011.
- [6] Sadeghian A., et al."CAR-Net: Clairvoyant Attentive Recurrent Network". ECCV 2018.
- [7] Chen C., et al."Crowd-Robot Interaction: Crowd-aware Robot Navigation withAttention-based Deep Reinforcement Learning". ICRA 2019.
- [8] Fox D, Burgard W, Thrun S. The dynamic window approach to collision avoidance. IEEETrans Robot Autom. 1997;4(5):23-33.
- [9] Macenski S, Martín F, White R, Clavero JG. ROS 2 System Architectures. IEEE RobotAutom Lett. 2022;7(3):1-8.
- [10]Chen C, Hu S, Nikdel P , Mori G, Savva M. Crowd-Robot Interaction: Crowd-awareRobot Navigation with Attention-based Deep Reinforcement Learning. In: 2019International Conference on Robotics and Automation (ICRA); 2019 May 20-24;Montreal, Canada. Piscataway: IEEE; 2019. p. 3225-31.


