cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  libpointmatcher
  VERSION 1.2.1
  RELEASE stable
  SUMMARY "An Iterative Closest Point (ICP) library for 2D/3D mapping."
  AUTHOR "François Pomerleau"
  AUTHOR "Stephane Magnenat"
  CONTACT "stephane.magnenat@mavt.ethz.ch"
  HOME "http://github.com/ethz-asl/libpointmatcher"
  LICENSE "BSD 3-Clause License (BSD-3-Clause)"
  README README.md
  INSTALL /usr
  CONFIGURATION_DESTINATION /etc/pointmatcher
  HEADER_DESTINATION include/pointmatcher
  PREFIX pointmatcher-
)

remake_add_directories(
  data
)
remake_doc(
  html
)

remake_pack_deb(
  DEPENDS libnabo libyaml-cpp0.3|yaml-cpp[0-9.]* libboost-system[0-9.]*
    libboost-filesystem[0-9.]*
)
remake_pack_deb(
  COMPONENT examples
  DEPENDS libpointmatcher
  DESCRIPTION "examples"
)
remake_pack_deb(
  COMPONENT test
  DEPENDS libpointmatcher libpointmatcher-examples
  DESCRIPTION "testing"
)
remake_pack_deb(
  COMPONENT eval
  DEPENDS libpointmatcher libboost-program-options[0-9.]*
    libboost-thread[0-9.]* libncurses5
  DESCRIPTION "evaluation"
)
remake_pack_deb(
  COMPONENT dev
  DEPENDS libpointmatcher libnabo-dev libboost-system-dev
    libboost-filesystem-dev libeigen${LIBPOINTMATCHER_EIGEN_VERSION}-dev
  DESCRIPTION "development headers"
)
remake_pack_deb(
  ARCH all
  COMPONENT doc
  DESCRIPTION "documentation"
)

remake_distribute_deb(
  DISTRIBUTION lucid
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libnabo-dev yaml-cpp0.2.6-dev libboost-system-dev libncurses5-dev
    libboost-filesystem-dev libboost-program-options-dev libboost-thread-dev 
    libeigen3-dev libgtest-dev remake pkg-config doxygen
  PASS CMAKE_BUILD_TYPE LIBPOINTMATCHER_GIT_REVISION
)
remake_distribute_deb(
  DISTRIBUTION precise
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libnabo-dev yaml-cpp libboost-system-dev libncurses5-dev
    libboost-filesystem-dev libboost-program-options-dev libboost-thread-dev 
    libeigen3-dev libgtest-dev remake pkg-config doxygen
  PASS CMAKE_BUILD_TYPE LIBPOINTMATCHER_GIT_REVISION
)
remake_distribute_deb(
  DISTRIBUTION trusty
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libnabo-dev libyaml-cpp0.3-dev libboost-system-dev libncurses5-dev
    libboost-filesystem-dev libboost-program-options-dev libboost-thread-dev 
    libeigen3-dev libgtest-dev remake pkg-config doxygen
  PASS CMAKE_BUILD_TYPE LIBPOINTMATCHER_GIT_REVISION
)
