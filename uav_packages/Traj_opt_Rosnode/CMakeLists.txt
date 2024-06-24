cmake_minimum_required(VERSION 3.11)
project(Traj_Opt_Rosnode)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  mrs_msgs
  message_filters
  mrs_lib
  std_msgs
  std_srvs
  mavros_msgs
)


# Eigen 3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

# OSQP
set(OsqpEigen_DIR "/home/jeremie/Master_Thesis/Quadrotor/osqp-eigen/build" CACHE PATH "Path to OsqpEigenConfig.cmake")

find_package(osqp REQUIRED)
include_directories(${OSQP_INCLUDE_DIRS})
link_directories(${OSQP_LIBRARY_DIRS})


find_package(OsqpEigen REQUIRED)


catkin_package()

include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${OSQP_INCLUDE_DIRS}
  ${OSQPEIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(Traj_Opt_Rosnode src/Traj_Opt_Rosnode.cpp)

target_link_libraries(Traj_Opt_Rosnode PRIVATE osqp::osqp)
target_link_libraries(Traj_Opt_Rosnode PUBLIC OsqpEigen::OsqpEigen)
target_link_libraries(Traj_Opt_Rosnode PUBLIC ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_dependencies(Traj_Opt_Rosnode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${osqp_EXPORTED_TARGETS} ${Eigen_EXPORTED_TARGETS} ${OsqpEigen_EXPORTED_TARGETS})

######








