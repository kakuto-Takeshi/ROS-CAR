cmake_minimum_required(VERSION 3.0.2)
project(ros_car)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_car
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_car_node src/ros_car.cpp)

target_link_libraries(ros_car_node
  ${catkin_LIBRARIES}
)
