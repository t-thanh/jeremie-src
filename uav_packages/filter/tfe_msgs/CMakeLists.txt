cmake_minimum_required(VERSION 2.8.3)
project(tfe_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs)

add_message_files( FILES Odom2DWithCovariance.msg )
generate_messages( DEPENDENCIES std_msgs geometry_msgs )

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    std_msgs
)
