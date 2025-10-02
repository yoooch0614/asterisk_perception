cmake_minimum_required(VERSION 3.0.2)
project(asterisk_camera_omnidirectional VERSION 0.1.0)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  camera_info_manager
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES asterisk_camera_omnidirectional
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(pov_camera_joy_controller src/pov_camera_joy_controller.cpp)
target_link_libraries(pov_camera_joy_controller ${catkin_LIBRARIES})
add_dependencies(pov_camera_joy_controller ${catkin_EXPORTED_TARGETS})


add_executable(camera_info_setter src/camera_info_setter.cpp)
target_link_libraries(camera_info_setter ${catkin_LIBRARIES})
add_dependencies(camera_info_setter ${catkin_EXPORTED_TARGETS})
