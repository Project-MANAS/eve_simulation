cmake_minimum_required(VERSION 2.8.3)
project(eve_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  eve_msgs
)

find_package(gazebo 9 REQUIRED)

catkin_package(CATKIN_DEPENDS
    gazebo_ros
    eve_msgs
    geometry_msgs
    nav_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

add_library(EvePlugin SHARED plugins/EvePlugin.cc)
target_link_libraries(EvePlugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})
add_dependencies(EvePlugin ${catkin_EXPORTED_TARGETS})

install(TARGETS EvePlugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
