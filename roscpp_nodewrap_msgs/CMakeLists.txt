cmake_minimum_required(VERSION 2.8.3)
project(roscpp_nodewrap_msgs)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  std_srvs
)

####################################
## Generate messages and services ##
####################################
## Add message files to be generated
add_message_files(
  DIRECTORY msg
  FILES
    FrequencyEstimates.msg
    LatencyEstimates.msg
    PeriodEstimates.msg
)

## Add service files to be generated
add_service_files(
  DIRECTORY srv
  FILES
    GetWorkerFrequency.srv
    GetWorkerState.srv
    HasWorker.srv
    ListWorkers.srv
)

## Generate services
generate_messages(
  DEPENDENCIES std_msgs std_srvs
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
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    std_srvs
)
