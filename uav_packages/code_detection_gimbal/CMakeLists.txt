cmake_minimum_required(VERSION 3.11)
project(code_detection_gimbal)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(Aruco_detection_gimbal src/Aruco_detect.cpp)

target_link_libraries(Aruco_detection_gimbal
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(Aruco_detection_gimbal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

