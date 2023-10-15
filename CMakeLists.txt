cmake_minimum_required(VERSION 2.8.3)
project(servoing_force_ur)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  control_msgs
  tf
  message_generation
  ur_kinematics
  
)
find_package(Eigen3 REQUIRED)
generate_messages(DEPENDENCIES control_msgs)

catkin_package(

)
catkin_install_python(PROGRAMS
   src/teleop_twist_keyboard.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(servoing_force_ur src/servoing_force_ur.cpp)
add_executable(move_trajectory src/move_trajectory.cpp)

add_executable(ur_control_mbzirc src/ur_control_mbzirc.cpp)



target_link_libraries(servoing_force_ur
  ${catkin_LIBRARIES}
)
target_link_libraries(move_trajectory 
  ${catkin_LIBRARIES}
)

target_link_libraries(ur_control_mbzirc
  ${catkin_LIBRARIES}
  ur10e_kin
)
