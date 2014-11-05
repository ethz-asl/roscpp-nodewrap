cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  roscpp-nodewrap-${ROS_DISTRIBUTION}
  VERSION 0.1
  RELEASE alpha
  SUMMARY "C++ implementation of ROS node(let) wrappers."
  AUTHOR "Ralf Kaestner"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://github.com/ethz-asl/roscpp-nodewrap"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX roscpp-nodewrap
  INSTALL /usr
)

remake_doc(
  man INSTALL share
  html
)

remake_ros_pack_deb()
remake_pack_deb(
  ARCH all
  COMPONENT doc
  DESCRIPTION "documentation"
)

remake_var_name(GIT_REVISION_VAR ${REMAKE_PROJECT_FILENAME} GIT_REVISION)

if(ROS_DISTRIBUTION STRLESS "groovy")
  remake_ros_distribute_deb(
    DISTRIBUTION lucid
    SECTION misc
    ARCH "i386 amd64"
    UPLOAD ppa:kralf/asl
    DEPENDS remake pkg-config
    PASS CMAKE_BUILD_TYPE ${GIT_REVISION_VAR}
    FORCE_CONSISTENCY
  )
endif(ROS_DISTRIBUTION STRLESS "groovy")

if(ROS_DISTRIBUTION STRLESS "indigo")
  remake_ros_distribute_deb(
    DISTRIBUTION precise
    SECTION misc
    ARCH "i386 amd64"
    UPLOAD ppa:kralf/asl
    DEPENDS remake pkg-config
    PASS CMAKE_BUILD_TYPE ${GIT_REVISION_VAR}
    FORCE_CONSISTENCY
  )
endif(ROS_DISTRIBUTION STRLESS "indigo")

if(ROS_DISTRIBUTION STREQUAL "indigo")
  remake_ros_distribute_deb(
    DISTRIBUTION trusty
    SECTION misc
    ARCH "i386 amd64"
    UPLOAD ppa:kralf/asl
    DEPENDS remake pkg-config
    PASS CMAKE_BUILD_TYPE ${GIT_REVISION_VAR}
    FORCE_CONSISTENCY
  )
endif(ROS_DISTRIBUTION STREQUAL "indigo")
