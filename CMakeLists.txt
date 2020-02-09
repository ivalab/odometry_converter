cmake_minimum_required(VERSION 2.8.3)
project(odometry_converter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  kobuki_description
  kobuki_gazebo_plugins
  robot_state_publisher
  yocs_cmd_vel_mux
  message_filters
  roscpp
  sensor_msgs
  tf2_ros
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES delayed_odometry
#  CATKIN_DEPENDS gazebo_plugins gazebo_ros kobuki_description kobuki_gazebo_plugins robot_state_publisher yocs_cmd_vel_mux
#  DEPENDS system_lib
)


include_directories(
${PROJECT_SOURCE_DIR}/include
${catkin_INCLUDE_DIRS}
)

set(LIBS 
${catkin_LIBRARIES}
)

add_executable(odom_converter
src/convert.cc
)
add_dependencies(odom_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_converter
${LIBS}
)
