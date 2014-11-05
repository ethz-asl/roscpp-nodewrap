remake_ros_package(
  roscpp_nodewrap
  DEPENDS roscpp nodelet
  DESCRIPTION "library and development headers"
)
remake_ros_package(
  roscpp_nodewrap_tutorial
  DEPENDS roscpp std_msgs std_srvs roscpp_nodewrap
  DESCRIPTION "tutorial"
)
