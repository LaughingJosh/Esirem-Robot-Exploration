cmake_minimum_required(VERSION 3.0.2)
project(wifibot_moveit_demos)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning_interface
  roscpp
  tf2_geometry_msgs
  tf2_eigen
  
)
find_package(Curses REQUIRED)
catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    roscpp
    tf2_geometry_msgs
    tf2_eigen 
     
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${CURSES_LIBRARIES}
)


add_executable(pick_place src/pick_place.cpp)

target_link_libraries(pick_place ${catkin_LIBRARIES} ${CURSES_LIBRARIES})

install(TARGETS pick_place
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)