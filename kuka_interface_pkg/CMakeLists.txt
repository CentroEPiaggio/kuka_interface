cmake_minimum_required(VERSION 2.8.3)
project(kuka_interface_pkg)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(arms_manager_node src/arms_manager_node.cpp src/arms_manager.cpp)
add_dependencies(arms_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arms_manager_node ${catkin_LIBRARIES})

add_executable(ee_manager_node src/ee_manager_node.cpp src/ee_manager.cpp src/qbmove_communications.cpp)
add_dependencies(ee_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ee_manager_node ${catkin_LIBRARIES})

add_executable(single_lwr_manager_node src/single_lwr_manager_node.cpp src/single_lwr_manager.cpp)
add_dependencies(single_lwr_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(single_lwr_manager_node ${catkin_LIBRARIES})

add_executable(homing src/homing.cpp)
add_dependencies(homing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(homing ${catkin_LIBRARIES})

add_executable(single_lwr_homing src/single_lwr_homing.cpp)
add_dependencies(single_lwr_homing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(single_lwr_homing ${catkin_LIBRARIES})