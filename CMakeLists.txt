cmake_minimum_required(VERSION 2.8.3)
project(object_server)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  tf
  interactive_markers
  message_generation
)

add_definitions(-std=c++11)
link_directories(${catkin_LIBRARY_DIRS})

add_service_files (
FILES
SetMarkers.srv
)

generate_messages(
DEPENDENCIES
std_msgs
geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_srvs tf interactive_markers
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(object_server src/object_server.cpp)
target_link_libraries(object_server ${catkin_LIBRARIES})
add_dependencies(object_server ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
