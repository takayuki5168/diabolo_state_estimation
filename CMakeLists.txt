cmake_minimum_required(VERSION 2.8.3)
project(diabolo_state_estimation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  )

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(execute src/execute.cpp)
target_link_libraries(execute ${catkin_LIBRARIES})
