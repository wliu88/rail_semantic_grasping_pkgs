cmake_minimum_required(VERSION 2.8.3)
project(fetch_grasp_suggestion)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cmake_modules
  
  eigen_conversions
  geometry_msgs
  interactive_markers
  
  message_generation
  
  pcl_conversions
  pcl_ros
  rail_grasp_calculation_msgs
  rail_manipulation_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf_conversions
  tf2_geometry_msgs
  tf2_ros
)

find_package(Eigen3 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
        FILES
        RankedGrasp.msg
        RankedGraspList.msg
)

add_service_files(
        FILES
        SuggestGrasps.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        geometry_msgs
        sensor_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(suggester src/suggester.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(suggester ${catkin_LIBRARIES} ${EIGEN_INCLUDE_DIRS})

## Add cmake target dependencies of the executable/library
add_dependencies(suggester ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS suggester
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
