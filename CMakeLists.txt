cmake_minimum_required(VERSION 2.8.3)
project(mrobot_controllers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  laser_geometry
  tf
  pcl_conversions
  pcl_ros
  message_generation
)

add_service_files(FILES srvController.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mrobot_controllers
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs laser_geometry tf pcl_conversions pcl_ros
  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(control_node src/main.cpp src/supervisor.cpp src/controllers.cpp src/sensors.cpp)
target_link_libraries(control_node ${catkin_LIBRARIES})
add_dependencies(control_node mrobot_controllers_generate_messages_cpp)

add_executable(lasertopoint src/lasertopoint.cpp) 
target_link_libraries(lasertopoint ${catkin_LIBRARIES})


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIY_DIRS})
add_definitions(${PCL_DEFINITIONS})
