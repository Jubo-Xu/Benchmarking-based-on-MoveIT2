# Benchmarking-based-on-MoveIT2
This is a ROS2 package that is mainly used for benchmarking. The package is the basic version which is used to compare different motion planning algorithms in different conditions. There are 15 test cases in total, and are tested under three scenarious: no obstacle(simplest case), with a single obstacle, and narrow space(a common case for robotics benchmarking). The comparison is accomplished for 5 planner architectures: OMPL(open library for basic sampling based motion planning algorithms), CHOMP, STOMP, OMPL+CHOMP(OMPL as pre-planner and CHOMP as post-optimizer), OMPL+STOMP. There are three important criterias for this benchmarking. The first one is time-consumption for generating plan, the second one is the metric of smoothness(calculated by finding the maximum difference between two adjacent discretized points of a trajectory), and the last one is whether the planning is successful or not. The time consumption and smoothness are shown as a marker above the robotic arm, and whether the planning is successful is noted in terminal. The trajectory of the end-effector is also shown in RViz.

## Good for
* Provide basic benchmarking for general motion planning algorithms
* Can be easily extend for further use
* Constraints would be added

## Set Up
This [ROS2](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Install-Binary.html) package is mainly used to interact with [RViz](https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html) based on [MoveIT 2](https://moveit.ros.org/install-moveit2/binary/). So to use this package, the moveit2 package and tutorials should already be installed and built.

1. Move into the **src** folder of workspace for moveit2, which is normally named as **ws_moveit2**
```bash
cd ~/ws_moveit2/src
```
2. Clone the code into this folder and name it, for example hello_moveit
```bash
git clone https://github.com/Jubo-Xu/Benchmarking-based-on-MoveIT2.git
```
3. Back to the workspace
```bash
cd ..
```
4. source the workspace and using colcon to build the whole project
```bash
source /opt/ros/humble/setup.bash
colcon build --mixin debug
```
5. open another terminal and launch the RViz first
```bash
ros2 launch moveit2_tutorials demo.launch.py
```
6. uncheck the **MotionPlanning** box in RViz

