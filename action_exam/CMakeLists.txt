cmake_minimum_required(VERSION 2.8.3)
project(action_exam)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
  actionlib
  roscpp
)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(FILES Primenumber.action)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
  LIBRARIES ros_tutorials_action
  CATKIN_DEPENDS std_msgs actionlib_msgs actionlib roscpp
  DEPENDS Boost
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(action_exam_server src/action_exam_server.cpp)
add_dependencies(action_exam_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(action_exam_server ${catkin_LIBRARIES})

add_executable(action_exam_client src/action_exam_client.cpp)
add_dependencies(action_exam_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(action_exam_client ${catkin_LIBRARIES})
