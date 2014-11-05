remake_ros_package_add_plugin(
  roscpp_nodewrap_tutorial nodelet
  ChatterNode.cpp
  LINK roscpp_nodewrap
)
remake_ros_plugin_add_class(
  nodelet ChatterNode nodelet::Nodelet
  CLASS_TYPE nodewrap::Nodelet<nodewrap::ChatterNode>
  DESCRIPTION "Example chatter node"
)

remake_ros_package_add_executable(
  chatter_node
  LINK roscpp_nodewrap_tutorial
)
remake_ros_package_add_executable(
  chatter_supernode
  LINK roscpp_nodewrap_tutorial
)
