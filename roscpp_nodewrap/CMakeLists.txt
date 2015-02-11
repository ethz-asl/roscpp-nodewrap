cmake_minimum_required(VERSION 2.8.3)
project(roscpp_nodewrap)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  nodelet
  message_generation
  std_msgs
)

## Add service files to be generated
add_service_files(
  DIRECTORY srv
  FILES
    GetParamInfo.srv
    GetParamValueBool.srv
    GetParamValueDouble.srv
    GetParamValueInt.srv
    GetParamValueString.srv
    GetParamValueXml.srv
    FindParam.srv
    HasParam.srv
    ListParams.srv
    SetParamValueBool.srv
    SetParamValueDouble.srv
    SetParamValueInt.srv
    SetParamValueString.srv
    SetParamValueXml.srv
)

## Generate services
generate_messages(
  DEPENDENCIES std_msgs
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
  LIBRARIES roscpp_nodewrap
  CATKIN_DEPENDS message_runtime nodelet roscpp std_msgs
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
add_library(roscpp_nodewrap
  src/ConfigClient.cpp
  src/ConfigServer.cpp
  src/Exceptions.cpp
  src/NodeImpl.cpp
  src/NodeInterface.cpp
  src/Param.cpp
#   src/ParamCallbackHelper.cpp
  src/ParamClient.cpp
  src/ParamClientOptions.cpp
  src/ParamServer.cpp
  src/ParamServerOptions.cpp
  src/ParamServiceHelper.cpp
  src/ParamType.cpp
  src/Signal.cpp
)
add_dependencies(roscpp_nodewrap roscpp_nodewrap_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(
  roscpp_nodewrap
  ${catkin_LIBRARIES}
)
