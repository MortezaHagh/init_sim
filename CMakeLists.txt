cmake_minimum_required(VERSION 3.0.2)
project(init_sim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_ros
  std_msgs
  geometry_msgs
  gazebo_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  #CATKIN_DEPENDS gazebo_ros rospy
  #DEPENDS gazebo
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)


install(DIRECTORY launch models worlds maps rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
