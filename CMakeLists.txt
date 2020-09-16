cmake_minimum_required(VERSION 2.8.3)
project(robot_gui_debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_message_files(
  FILES
  ParameterAdjustMsg.msg
)

add_service_files(
  FILES
  ParameterShow.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


LINK_DIRECTORIES(${CMAKE_HOME_DIRECTORY}/robot_gui_debug/lib)
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES robot_gui_debug libyaml-cpp.a
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${CMAKE_HOME_DIRECTORY}/robot_gui_debug/lib/libyaml-cpp
)

AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)

#生成动态库
add_library(${PROJECT_NAME}  ${DIR_SRCS})
add_executable(robot nodes/main.cpp ${DIR_SRCS})
add_dependencies(robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot ${OpenCV_LIBS})
target_link_libraries(robot ${catkin_LIBRARIES})
target_link_libraries(robot Threads::Threads)
target_link_libraries(robot libyaml-cpp.a)