cmake_minimum_required(VERSION 3.0.0)
project(gtec_gazebosensorplugins)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11")


add_definitions(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
  gtec_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(gtec_gazebosensorplugins src/UwbPlugin.cpp)
target_link_libraries(gtec_gazebosensorplugins ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(gtec_gazebosensorplugins ${catkin_EXPORTED_TARGETS})

add_library(magnetic_distortion_plugin src/MagneticDistortionPlugin.cpp)
target_link_libraries(magnetic_distortion_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(magnetic_distortion_plugin ${catkin_EXPORTED_TARGETS})

##add_library(rtls_actor_plugin src/RTLSActorPlugin.cpp src/RTLSActorPlugin.hh)
##target_link_libraries(rtls_actor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
##add_dependencies(rtls_actor_plugin ${catkin_EXPORTED_TARGETS})

add_library(custom_skid_steer_drive src/CustomSkidSteerDrive.cpp src/CustomSkidSteerDrive.h)
target_link_libraries(custom_skid_steer_drive ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(custom_skid_steer_drive ${catkin_EXPORTED_TARGETS})


catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros
    gtec_msgs
)
##add_dependencies(${PROJECT_NAME} gtec_generate_messages_cpp)
