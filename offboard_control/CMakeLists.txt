cmake_minimum_required(VERSION 3.8)
project(offboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(custom_msgs REQUIRED)

include_directories(include)

#################################################
add_executable(offboard_node
  src/offboard_control.cpp
)
ament_target_dependencies(offboard_node
  rclcpp
  geometry_msgs
  mavros_msgs
  nav_msgs
  tf2_ros
  visualization_msgs
  std_srvs
  tf2
  custom_msgs
)
target_include_directories(offboard_node
  PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
  $<INSTALL_INTERFACE:include>)   
######################################################



add_executable(marker_node
  src/marker_publish.cpp
)

add_executable(waypoints_node
  src/recieve_transform_waypoints.cpp
)
############################################################## OFFBOARD SERVER NODE
add_executable(offboard_server_node
  src/offboard_control_server.cpp 
)
ament_target_dependencies(offboard_server_node
  rclcpp
  geometry_msgs
  mavros_msgs
  nav_msgs
  tf2_ros
  tf2
  visualization_msgs
  std_srvs
  custom_msgs
)
 
target_include_directories(offboard_server_node
  PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
  $<INSTALL_INTERFACE:include>)
################################################################ OFFBOARD SERVER NODE



################################################################ PROCESS WAYPOINT NODE
add_executable(process_waypoint_node
  src/process_waypoint.cpp
)
target_link_libraries(process_waypoint_node Geographic)
ament_target_dependencies(marker_node
  rclcpp
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2
  tf2_ros
)
target_include_directories(process_waypoint_node
  PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
  $<INSTALL_INTERFACE:include>)
################################################################ PROCESS WAYPOINT NODE

add_executable(drone_transformation_node
  src/drone_transformation.cpp
)
ament_target_dependencies(drone_transformation_node
  rclcpp
  geometry_msgs
  mavros_msgs
  nav_msgs
  tf2_ros
  tf2
  visualization_msgs
  std_srvs
)
target_include_directories(drone_transformation_node
  PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)



ament_target_dependencies(waypoints_node
  rclcpp
  geometry_msgs
  mavros_msgs
  nav_msgs
  tf2_ros
  tf2
  visualization_msgs)



ament_target_dependencies(process_waypoint_node
  rclcpp
  geometry_msgs
  mavros_msgs
  nav_msgs
  tf2_ros
  visualization_msgs
  std_srvs
  tf2
  custom_msgs
  # GeographicLib
)


target_include_directories(marker_node
  PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/offboard_control>
  $<INSTALL_INTERFACE:include>)  

target_include_directories(waypoints_node
  PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/offboard_control>
  $<INSTALL_INTERFACE:include>)


install(TARGETS
  offboard_node
  marker_node
  waypoints_node
  offboard_server_node
  process_waypoint_node
  drone_transformation_node
  DESTINATION lib/${PROJECT_NAME}
)

install (DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

###########################################PYTHON SCRIPT###############
install(PROGRAMS
  python_script/lora_transmit.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
