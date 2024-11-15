Prerequisites:
ROS noetic 


Installation:

mkdir -p catkin_ws/src
cd catkin_ws
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver && git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git src/universal_robot
sudo apt update -qq 
rosdep update 
rosdep install --from-paths src --ignore-src -y 
catkin_make
apt-get install ros-noetic-moveit-servo -y
apt-get install ros-noetic-rqt-controller-manager -y

cd src
git clone https://github.com/larics/ur_servo_mbzirc/
cd..
catkin_make



Running:

Running robot driver, moveit planner and moveit servo server:

roslaunch servoing_force_ur robot.launch

Running control node and servo node:

roslaunch servoing_force_ur control_ur.launch

Switching to diffent operation modes:

rostopic pub /operation_mode std_msgs/Int32 "data: 0" 

Data indicates operation mode
0 - go to initial scanning position, start scanning, when camera finds object move above it and servo downward to pickup object and move it other location
1 - manual do only one operation
2 - stop execution and hold position - this is initial state
3 - follow target using LIDAR
4 - test - currently not used
5 - used for docking - currently not supported
6 -  same sa 3 but with different starting position
7 - go above position received from UAV and servo downward, pickup object and move it to other location



