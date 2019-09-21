cmake_minimum_required(VERSION 2.8.3)
project(online_tuning)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS tf geometry_msgs nav_msgs geometry_msgs roscpp rospy std_msgs moveit_core pluginlib geometric_shapes rosbag tf2_ros tf2_eigen tf2_geometry_msgs sensor_msgs moveit_visual_tools moveit_ros_planning moveit_ros_planning_interface)

#find_package (ALGLIB REQUIRED)
#INCLUDE_DIRECTORIES(${ALGLIB_INCLUDE_DIR})

find_package (Eigen3 REQUIRED NO_MODULE)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qp
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  /home/neousys/lee/quad/include/
)

link_directories(
  ${catkin_LIB_DIRS}
  /home/neousys/lee/quad/lib/
  /usr/local/lib
)

add_executable(main_code
  include/online_tuning/controller.h
  src/controller.cpp
  src/main_code.cpp
)
add_dependencies(main_code ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_code
  ${catkin_LIBRARIES}
  libquadprog.a 
)
