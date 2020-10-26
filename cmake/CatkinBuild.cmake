
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-O3)
endif(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")

# Set catkin package dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  libnabo
)

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono date_time filesystem program_options system thread timer)

# Catkin package macro
catkin_package(
  INCLUDE_DIRS
    pointmatcher
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    yaml_cpp_pm
    pointmatcher
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
)

########################
## Library definition ##
########################
# Yaml-cpp
file(GLOB YAML_PRIVATE_HEADERS "contrib/yaml-cpp-pm/src/[a-zA-Z]*.h")
file(GLOB YAML_SOURCES "contrib/yaml-cpp-pm/src/[a-zA-Z]*.cpp")
add_library(yaml_cpp_pm
  ${YAML_SOURCES}
  ${YAML_PRIVATE_HEADERS}
)

target_include_directories(yaml_cpp_pm
  PUBLIC
    contrib/yaml-cpp-pm/include
  PRIVATE
    contrib/yaml-cpp-pm/src
)

# Libpointmatcher
add_library(pointmatcher
  ${POINTMATCHER_SRC}
  ${POINTMATCHER_HEADERS}
)

target_include_directories(pointmatcher
  PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/pointmatcher
    ${CMAKE_SOURCE_DIR}/pointmatcher/DataPointsFilters
    ${CMAKE_SOURCE_DIR}/pointmatcher/DataPointsFilters/utils
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${yaml_cpp_pm_INCLUDE_DIRS}
)

target_link_libraries(pointmatcher
  ${catkin_LIBRARIES}
  Boost::chrono
  Boost::date_time
  Boost::filesystem
  Boost::program_options
  Boost::thread
  Boost::timer
  Boost::system
  yaml_cpp_pm
)

#############
## Install ##
#############
install(
  TARGETS
    pointmatcher
    yaml_cpp_pm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    ${CMAKE_SOURCE_DIR}/pointmatcher/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/pointmatcher
)

##########
## Test ##
##########
# TODO(ynava) Currently unit tests are only enabled if the build type is NOT debug.
# When the CI pipeline supports optimized unit test runs, we should remove this extra condition.
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_pointmatcher
      utest/utest.cpp
      utest/ui/DataFilters.cpp
      utest/ui/DataPoints.cpp
      utest/ui/ErrorMinimizers.cpp
      utest/ui/Inspectors.cpp
      utest/ui/IO.cpp
      utest/ui/Loggers.cpp
      utest/ui/Matcher.cpp
      utest/ui/Outliers.cpp
      utest/ui/Transformations.cpp
  )

  target_include_directories(test_pointmatcher PRIVATE
    PRIVATE
      ${CMAKE_SOURCE_DIR}
      ${CMAKE_SOURCE_DIR}/pointmatcher
      ${CMAKE_SOURCE_DIR}/pointmatcher/DataPointsFilters
      ${CMAKE_SOURCE_DIR}/pointmatcher/DataPointsFilters/utils
    SYSTEM
      ${EIGEN3_INCLUDE_DIR}
      ${Boost_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
  )

  target_link_libraries(test_pointmatcher
    pointmatcher
    yaml_cpp_pm
    ${catkin_LIBRARIES}
  )

  set(TEST_DATA_FOLDER "${CMAKE_SOURCE_DIR}/examples/data/")
  add_definitions(-DUTEST_TEST_DATA_PATH="${TEST_DATA_FOLDER}/")
endif(CATKIN_ENABLE_TESTING)