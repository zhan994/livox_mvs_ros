cmake_minimum_required(VERSION 3.0.2)
project(start_all)

set(PACKAGE_DEPENDENCIES)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES start_all
#  CATKIN_DEPENDS roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})
