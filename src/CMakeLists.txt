remake_find_package(
  Boost REQUIRED
  COMPONENTS system filesystem program_options thread
  RESULT_VAR Boost_FOUND
)
remake_find_package(eigen3 CONFIG)
if(EIGEN3_FOUND)
  remake_project_set(EIGEN_VERSION 3 CACHE INTERNAL
    "Version of the eigen linear algebra library")
else(EIGEN3_FOUND)
  remake_find_package(eigen2 CONFIG)
  remake_project_set(EIGEN_VERSION 2 CACHE INTERNAL
    "Version of the eigen linear algebra library")
endif(EIGEN3_FOUND)
remake_find_package(libnabo CONFIG)
remake_find_package(yaml-cpp CONFIG)
remake_find_library(ncurses ncurses.h PACKAGE ncurses)
remake_find_package(Threads REQUIRED)

remake_include(
  lib
  ${Boost_INCLUDE_DIRS}
  ${EIGEN${LIBPOINTMATCHER_EIGEN_VERSION}_INCLUDE_DIRS}
  ${LIBNABO_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${NCURSES_HEADERS}
)

remake_add_directories(lib bin)
