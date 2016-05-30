cmake_minimum_required(VERSION 2.8.3)
project(roscpp_nodewrap_tutorial)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  nodelet
  std_msgs
  std_srvs
  roscpp_nodewrap
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roscpp_nodewrap_tutorial
  CATKIN_DEPENDS
    std_msgs
    std_srvs
    roscpp_nodewrap
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(roscpp_nodewrap_tutorial
  src/ChatterNode.cpp
  src/WorkerNode.cpp
)

## Declare a cpp executable
add_executable(
  chatter_node
  src/chatter_node.cpp
)
add_executable(
  chatter_supernode
  src/chatter_supernode.cpp
)
add_executable(
  worker_node
  src/worker_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  roscpp_nodewrap_tutorial
  ${catkin_LIBRARIES}
)
target_link_libraries(
  chatter_node
  roscpp_nodewrap_tutorial
  ${catkin_LIBRARIES}
)
target_link_libraries(
  chatter_supernode
  roscpp_nodewrap_tutorial
  ${catkin_LIBRARIES}
)
target_link_libraries(
  worker_node
  roscpp_nodewrap_tutorial
  ${catkin_LIBRARIES}
)
