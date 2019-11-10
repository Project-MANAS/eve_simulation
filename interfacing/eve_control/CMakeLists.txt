cmake_minimum_required(VERSION 2.8.3)
project(eve_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  eve_msgs
  gazebo_msgs
)

catkin_package( CATKIN_DEPENDS
  roscpp
  eve_msgs
  gazebo_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(keyboard_controller src/keyboard_control.cpp)
target_link_libraries(keyboard_controller ${catkin_LIBRARIES})
add_dependencies(keyboard_controller ${catkin_EXPORTED_TARGETS})

add_executable(joystick_controller src/joystick_control.cc src/joystick.cc)
target_link_libraries(joystick_controller ${catkin_LIBRARIES})
add_dependencies(joystick_controller ${catkin_EXPORTED_TARGETS})