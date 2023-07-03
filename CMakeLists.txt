cmake_minimum_required(VERSION 2.8.3)
project(servoing_force_ur)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
 tf
  message_generation
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(servoing_force_ur src/servoing_force_ur.cpp)


target_link_libraries(servoing_force_ur
  ${catkin_LIBRARIES}
)

