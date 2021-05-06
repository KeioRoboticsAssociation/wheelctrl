cmake_minimum_required(VERSION 3.0.2)
project(serial_test_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES joy_commander
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gen_motor_command src/gen_motor_command.cpp)
target_link_libraries(gen_motor_command ${catkin_LIBRARIES})