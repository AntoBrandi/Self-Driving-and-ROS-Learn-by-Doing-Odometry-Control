cmake_minimum_required(VERSION 3.0.2)
project(bumperbot_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2
)

find_package(Eigen3 REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf2_ros
    tf2
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_install_python(PROGRAMS 
  nodes/simple_controller_node.py
  nodes/noisy_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(simple_controller src/simple_controller.cpp)
add_dependencies(simple_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_controller ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_library(noisy_controller src/noisy_controller.cpp)
add_dependencies(noisy_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(noisy_controller ${catkin_LIBRARIES})

add_executable(simple_controller_node nodes/simple_controller_node.cpp)
add_dependencies(simple_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_controller_node ${catkin_LIBRARIES} simple_controller)

add_executable(noisy_controller_node nodes/noisy_controller_node.cpp)
add_dependencies(noisy_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(noisy_controller_node ${catkin_LIBRARIES} noisy_controller)
