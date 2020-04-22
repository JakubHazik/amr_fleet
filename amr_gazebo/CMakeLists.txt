cmake_minimum_required(VERSION 2.8.3)
project(amr_gazebo)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp gazebo_ros
  DEPENDS gazebo
)
