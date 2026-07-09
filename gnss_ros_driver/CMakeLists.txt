cmake_minimum_required(VERSION 3.0.2)
project(gnss_ros_driver)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -s")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

add_executable(gnss_ros_driver src/main.cpp src/um982_nmea_parser.cpp)
target_link_libraries(gnss_ros_driver ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_include_directories(gnss_ros_driver PUBLIC 
  include
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/plot_fix_bag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
