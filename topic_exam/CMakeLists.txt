cmake_minimum_required(VERSION 3.0.2)
project(topic_exam)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS roscpp std_msgs)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(topic_exam_pub src/topic_exam_pub.cpp)
target_link_libraries(topic_exam_pub ${catkin_LIBRARIES})

add_executable(topic_exam_sub src/topic_exam_sub.cpp)
target_link_libraries(topic_exam_sub ${catkin_LIBRARIES})

