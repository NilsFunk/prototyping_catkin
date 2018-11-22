cmake_minimum_required(VERSION 2.8.3)
project(data_association)
add_definitions(-std=c++11)

 # include cmake folder
set(CMAKE_MODULE_PATH
${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cfg)

# ROS related
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  rosbag
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

# find OpenCV
find_package(OpenCV REQUIRED)

catkin_package()

#Find Eigen 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(
  ${catkin_INCLUDE_DIRS} 
)
include_directories(include)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

add_executable(data_association src/data_association.cpp ${SOURCE})
target_link_libraries(data_association ${catkin_LIBRARIES} ${OpenCV_LIBS})
