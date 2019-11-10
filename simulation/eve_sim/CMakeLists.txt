cmake_minimum_required(VERSION 2.8.3)
project(eve_sim)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
        gazebo_ros
)

find_package (Qt5Widgets REQUIRED)
find_package (Qt5Core REQUIRED)
find_package(gazebo 9 REQUIRED)

catkin_package(
        CATKIN_DEPENDS gazebo_ros
)

include_directories(
        ${GAZEBO_INCLUDE_DIRS}
        ${Qt5Core_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
