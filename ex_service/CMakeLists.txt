cmake_minimum_required(VERSION 3.0.2)
project(ex_service)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs)

## Declare services
add_service_files(FILES myservice.srv)

generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS roscpp std_msgs)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ex_service_server src/ex_service_server.cpp)
add_dependencies(ex_service_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ex_service_server ${catkin_LIBRARIES})

add_executable(ex_service_client src/ex_service_client.cpp)
add_dependencies(ex_service_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ex_service_client ${catkin_LIBRARIES})

