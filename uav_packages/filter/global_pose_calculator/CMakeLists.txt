cmake_minimum_required(VERSION 3.0.2)
project(global_pose_calculator)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  rospy 
  std_msgs 
  geometry_msgs 
  nav_msgs
  tfe_msgs
  gazebo_msgs
  tf 
  tf2_ros
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    geometry_msgs 
    nav_msgs
    tfe_msgs
    gazebo_msgs
    tf 
    tf2_ros
  DEPENDS
    # Add build and run dependencies here
    std_msgs
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Copy config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
