cmake_minimum_required(VERSION 3.0.2)
project(bezier_predictor)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  rospy
  std_msgs
  mrs_msgs
  nav_msgs
  geometry_msgs
  tfe_msgs
  
  
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS 
     rospy
  mrs_msgs
  nav_msgs
  geometry_msgs
  tfe_msgs
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
