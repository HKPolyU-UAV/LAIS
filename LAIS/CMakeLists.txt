cmake_minimum_required(VERSION 2.8.3)
project(LAIS)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -pthread)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  message_generation
  mavros_msgs
  geometry_msgs
  cv_bridge
  image_transport
  sensor_msgs
)
find_package (Eigen3 REQUIRED)
find_package (OpenCV 4 REQUIRED)
find_package (yaml-cpp REQUIRED)
find_package (Sophus REQUIRED )

add_message_files(
  FILES
  Center.msg
  object.msg
  Objects.msg
  identify_command.msg
  yamlstate.msg
)
add_service_files(
  FILES
  Command.srv
)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS
roscpp std_msgs message_generation sensor_msgs)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(fj005
     src/fj005.cpp
     src/movement/generalmove.cpp
     src/movement/circle.cpp
     src/movement/check.cpp
     src/utils/yoloNet.cpp
     src/utils/object.cpp)

target_link_libraries(fj005
     ${OpenCV_LIBRARIES}
     ${catkin_LIBRARIES}
     ${YAML_CPP_LIBRARIES}
     ${Sophus_LIBRARIES})

add_executable(camera
     src/camera.cpp
     src/utils/yoloNet.cpp
     src/visualization/rviz_object.cpp)

target_link_libraries(camera
     ${OpenCV_LIBRARIES}
     ${catkin_LIBRARIES}
     ${YAML_CPP_LIBRARIES})

add_executable(post_processing
    src/post_processing.cpp)

target_link_libraries(post_processing
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${Sophus_LIBRARIES})

add_executable(virtual
     src/virtual.cpp
     src/utils/yoloNet.cpp)

target_link_libraries(virtual
     ${OpenCV_LIBRARIES}
     ${catkin_LIBRARIES}
     ${YAML_CPP_LIBRARIES})
