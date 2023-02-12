cmake_minimum_required(VERSION 3.0.2)
project(bumperbot_localization)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  sensor_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    nav_msgs
    sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
    nodes/kalman_filter_node.py
    nodes/imu_republisher_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(kalman_filter src/kalman_filter.cpp)
add_dependencies(kalman_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kalman_filter ${catkin_LIBRARIES})

add_executable(kalman_filter_node nodes/kalman_filter_node.cpp)
add_dependencies(kalman_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kalman_filter_node ${catkin_LIBRARIES} kalman_filter)

add_executable(imu_republisher_node nodes/imu_republisher_node.cpp)
target_link_libraries(imu_republisher_node ${catkin_LIBRARIES})
