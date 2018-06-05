cmake_minimum_required(VERSION 2.8.3)
project(majack_tutorial)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(hello_world src/hello_world.cpp)
add_dependencies(hello_world ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hello_world ${catkin_LIBRARIES})

add_executable(publisher src/publisher.cpp)
add_dependencies(publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publisher ${catkin_LIBRARIES})

add_executable(subscriber src/subscriber.cpp)
add_dependencies(subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(subscriber ${catkin_LIBRARIES})

add_executable(turtle_controller src/turtle_controller.cpp)
add_dependencies(turtle_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtle_controller ${catkin_LIBRARIES})

add_executable(turtle_draw_square src/turtle_draw_square.cpp)
add_dependencies(turtle_draw_square ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtle_draw_square ${catkin_LIBRARIES})

add_executable(pushup_maker src/pushup_maker.cpp include/majack_tutorial/pushup_maker.h)
add_dependencies(pushup_maker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pushup_maker ${catkin_LIBRARIES})
