cmake_minimum_required(VERSION 3.0.2)
project(behavior_identification)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  message_generation
  derived_object_msgs
  road_side_system
  driver_models_types
)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

add_message_files(
  FILES
  VehStateSequence.msg
  VehStateSequenceArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  message_runtime
  derived_object_msgs
  road_side_system
  driver_models_types
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/driver/planner/include/common
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
)

add_executable(car_state_classify src/car_state_classify.cpp src/car_state_classify_node.cpp)
add_dependencies(car_state_classify
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(car_state_classify ${catkin_LIBRARIES})

add_executable(identification src/identification.cpp src/identification_node.cpp)
add_dependencies(identification
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  identification
  ${catkin_LIBRARIES}
  ${Python3_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/realtime_channel_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
