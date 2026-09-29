cmake_minimum_required(VERSION 3.0.2)
project(robot_path_planning)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  geometry_msgs
  teb_local_planner
  costmap_2d
  nav_msgs
  sensor_msgs
  tf2_ros
  pluginlib
  robot_state_publisher
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2_ros robot_state_publisher
)

## Ensure that the CATKIN_PACKAGE_BIN_DESTINATION is set correctly
set(CATKIN_PACKAGE_BIN_DESTINATION bin)


## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install RViz config files
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

##Install src files
install(
    DIRECTORY src/
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)
