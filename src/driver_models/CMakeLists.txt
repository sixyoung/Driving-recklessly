cmake_minimum_required(VERSION 3.0.2)
project(driver_models)

# C++17
add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer -O1 -g")
add_compile_options(-g -O0)
# 依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs
  tf2 tf2_ros tf2_geometry_msgs
  derived_object_msgs carla_msgs driver_models_types
  common local_planner speed_planner planner
)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

# Carla 依赖（你工程已有）
get_filename_component(WORKSPACE_PATH ${CMAKE_SOURCE_DIR} DIRECTORY)
include(${WORKSPACE_PATH}/src/Dependency.cmake)

# 导出
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs
    tf2 tf2_geometry_msgs derived_object_msgs driver_models_types
    carla_msgs common local_planner speed_planner planner
  DEPENDS Boost
)

# 头文件
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Carla_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# ===== BehaviorTree.CPP v3 轻量兼容探测（能用 target 就用，没 target 就用库路径）=====
set(BT_LINK "")
# 试图找到 ROS 导出的 imported target
find_package(behaviortree_cpp_v3 QUIET CONFIG)
if(TARGET BT::behaviortree_cpp_v3)
  set(BT_LINK BT::behaviortree_cpp_v3)
elseif(TARGET behaviortree_cpp_v3::behaviortree_cpp_v3)
  set(BT_LINK behaviortree_cpp_v3::behaviortree_cpp_v3)
else()
  # 兜底：按常见路径查找头文件与库
  find_path(BT_INCLUDE_DIR NAMES behaviortree_cpp_v3/bt_factory.h
            HINTS /opt/ros/noetic/include /usr/local/include /usr/include)
  find_library(BT_LIBRARY NAMES behaviortree_cpp_v3 behaviortree_cpp btcpp
               HINTS /opt/ros/noetic/lib /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu)
  if(BT_INCLUDE_DIR AND BT_LIBRARY)
    include_directories(${BT_INCLUDE_DIR})
    set(BT_LINK ${BT_LIBRARY})
  else()
    message(FATAL_ERROR "BehaviorTree.CPP v3 未找到：既无 imported target，也未在常见路径发现库/头。")
  endif()
endif()
message(STATUS "BehaviorTree.CPP link: ${BT_LINK}")

# 库
add_library(${PROJECT_NAME}
  src/base_driver.cpp
  src/manual_driver.cpp
  src/driver_model_manager.cpp
  src/auto_driver.cpp
  src/random_behavior_injector.cpp
  src/random_bt_yaml_loader.cpp
  src/task_scheduler.cpp
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${catkin_LIBRARIES}
    ${libCarla_LIBS}
    ${Boost_LIBRARIES}
    ${BT_LINK}     # ← 统一使用 BT_LINK（可能是 imported target，也可能是具体库路径）
    yaml-cpp
)

# 节点（只链到你的库即可，不要再直接链 BT）
add_executable(driver_model_manager_node src/driver_model_manager_node.cpp)
target_link_libraries(driver_model_manager_node PRIVATE ${PROJECT_NAME})

add_executable(test_task_scheduler_node src/test/scheduler_test_node.cpp)
target_link_libraries(test_task_scheduler_node PRIVATE ${PROJECT_NAME})

add_executable(test_qp_speed_node src/test/test_qp_speed.cpp)
target_link_libraries(test_qp_speed_node PRIVATE ${PROJECT_NAME})
# 依赖（消息等）
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
