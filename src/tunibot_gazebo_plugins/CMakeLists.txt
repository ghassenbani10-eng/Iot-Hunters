cmake_minimum_required(VERSION 3.8)
project(tunibot_gazebo_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(tunibot_human_actor SHARED src/human_actor_plugin.cpp)
target_include_directories(tunibot_human_actor PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(tunibot_human_actor ${GAZEBO_LIBRARIES})
ament_target_dependencies(tunibot_human_actor gazebo_ros rclcpp std_msgs)

install(TARGETS tunibot_human_actor LIBRARY DESTINATION lib)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/tunibot_gazebo_plugins.dsv.in")
ament_package()
