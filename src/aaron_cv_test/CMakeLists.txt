cmake_minimum_required(VERSION 2.8.3)
project(aaron_cv_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  image_transport
  pal_detection_msgs
  cv_bridge
  geometry_msgs
  message_generation
  tf
)

find_package(OpenCV 2.4 REQUIRED )

list(APPEND OpenCV_LIBRARIES opencv_nonfree)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

catkin_package(
  LIBRARIES aaron_cv_test
  CATKIN_DEPENDS roscpp rospy image_transport cv_bridge pal_detection_msgs  geometry_msgs message_generation
  DEPENDS system_lib
)

include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(camera_test src/camera_test.cpp)
target_link_libraries(camera_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(camera_test ${PROJECT_NAME}_generate_messages_cpp)

add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)
target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})
add_dependencies(turtle_tf_listener ${PROJECT_NAME}_generate_messages_cpp)
