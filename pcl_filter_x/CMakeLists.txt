cmake_minimum_required(VERSION 3.0.2)
project(pcl_filter_x)

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
find_package(PCL REQUIRED)

catkin_package()

add_executable(pcl_filter_x src/main.cpp)
target_include_directories(pcl_filter_x PUBLIC ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(pcl_filter_x ${catkin_LIBRARIES} ${PCL_LIBRARIES})