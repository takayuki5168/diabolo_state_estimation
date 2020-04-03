cmake_minimum_required(VERSION 2.8.3)
project(diabolo_state_estimation)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  pcl_conversions
  pcl_ros
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(execute src/execute.cpp)
target_link_libraries(execute ${catkin_LIBRARIES})
