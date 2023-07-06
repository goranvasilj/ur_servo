cmake_minimum_required(VERSION 2.8.3)
project(servoing_force_ur)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  control_msgs
  tf
  message_generation
  
)
find_package(Eigen3 REQUIRED)
generate_messages(DEPENDENCIES control_msgs)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(servoing_force_ur src/servoing_force_ur.cpp)
add_executable(move_trajectory src/move_trajectory.cpp)



target_link_libraries(servoing_force_ur
  ${catkin_LIBRARIES}
)
target_link_libraries(move_trajectory
  ${catkin_LIBRARIES}
)
