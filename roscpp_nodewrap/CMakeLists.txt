cmake_minimum_required(VERSION 2.8.3)
project(roscpp_nodewrap)

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
  diagnostic_updater
  roscpp_nodewrap_msgs
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
  CATKIN_DEPENDS
    roscpp
    nodelet
    diagnostic_updater
    roscpp_nodewrap_msgs
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
  src/AsyncWorker.cpp
  src/CompositeTask.cpp
  src/DiagnosticsExceptions.cpp
  src/DiagnosticTask.cpp
  src/DiagnosticTaskManager.cpp
  src/DiagnosticTaskOptions.cpp
  src/DiagnosticUpdater.cpp
  src/FrequencyStatistics.cpp
  src/FrequencyTask.cpp
  src/FrequencyTaskOptions.cpp
  src/FunctionTask.cpp
  src/FunctionTaskOptions.cpp
  src/LatencyStatistics.cpp
  src/LatencyTask.cpp
  src/LatencyTaskOptions.cpp
  src/MessageLatencyTask.cpp
  src/MessageStampFrequencyTask.cpp
  src/NodeInterface.cpp
  src/NodeImpl.cpp
  src/PeriodStatistics.cpp
  src/PeriodTask.cpp
  src/PeriodTaskOptions.cpp
  src/Publisher.cpp
  src/PublisherImpl.cpp
  src/PublisherOptions.cpp
  src/PublisherStatusTask.cpp
  src/PublisherStatusTaskOptions.cpp
  src/ServiceClient.cpp
  src/ServiceClientImpl.cpp
  src/ServiceClientOptions.cpp
  src/ServiceClientStatusTask.cpp
  src/ServiceClientStatusTaskOptions.cpp
  src/ServiceServer.cpp
  src/ServiceServerImpl.cpp
  src/ServiceServerOptions.cpp
  src/ServiceServerStatusTask.cpp
  src/ServiceServerStatusTaskOptions.cpp
  src/Signal.cpp
  src/StatefulFrequencyTask.cpp
  src/Subscriber.cpp
  src/SubscriberImpl.cpp
  src/SubscriberOptions.cpp
  src/SubscriberStatusTask.cpp
  src/SubscriberStatusTaskOptions.cpp
  src/SyncWorker.cpp
  src/Timer.cpp
  src/TimerInfo.cpp
  src/TimerManager.cpp
  src/TimerQueueCallback.cpp
  src/Worker.cpp
  src/WorkerEvent.cpp
  src/WorkerExceptions.cpp
  src/WorkerImpl.cpp
  src/WorkerManager.cpp
  src/WorkerOptions.cpp
  src/WorkerQueueCallback.cpp
  src/WorkerStatusTask.cpp
  src/WorkerStatusTaskOptions.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  roscpp_nodewrap
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
