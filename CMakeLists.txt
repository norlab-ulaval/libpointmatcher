cmake_minimum_required(VERSION 2.6)

project(libpointmatcher)
option(COMPILE_SANDBOX "Set to ON to compile sandbox binaries" OFF)

# c++ 0x
#if("${CMAKE_CXX_COMPILER_ID}" MATCHES "clang") 
#	set(CMAKE_COMPILER_IS_CLANGXX)
#endif ("${CMAKE_CXX_COMPILER_ID}" MATCHES "clang")
if(CMAKE_COMPILER_IS_GNUCXX)
	add_definitions(-std=gnu++0x)
endif(CMAKE_COMPILER_IS_GNUCXX)
# well, this breaks libpointmatcher on non-unix platform,
# please contribute the define for your favorite compiler
# It must support (compiler support in parenthesis):
# - N2672: Initializer lists (gcc)
# - N1984: auto-typed variables (gcc, clang)

#include(UseDoxygen)

# boost
find_package(Boost COMPONENTS system filesystem REQUIRED)

# eigen 3
find_path(EIGEN_INCLUDE_DIR Eigen/Core
	/usr/local/include/eigen
	/usr/include/eigen
  /opt/ros/diamondback/stacks/geometry/eigen/include
)

# nabo
find_path(NABO_INCLUDE_DIR nabo/nabo.h
	/usr/local/include
	/usr/include
)
find_library(NABO_LIBRARY NAMES libnabo.a libnabo PATHS
	/usr/local/lib
	/usr/lib
)

# gtest
find_path(GTEST_INCLUDE_DIR gtest/gtest.h
    /usr/include
)
find_library(GTEST_LIBRARY gtest
    /usr/lib/
)

# look for OpenCL library
if (WIN32)
	find_library(OPENCL_LIBRARIES opencl64)
	if (!OPENCL_LIBRARIES)
		find_library(OPENCL_LIBRARIES opencl32)
	endif (!OPENCL_LIBRARIES)
else (WIN32)
	find_library(OPENCL_LIBRARIES OpenCL ENV LD_LIBRARY_PATH)
endif (WIN32)
# if found, add
if (OPENCL_LIBRARIES)
	set(EXTRA_LIBS ${OPENCL_LIBRARIES})
endif (OPENCL_LIBRARIES)

# include all libs so far
include_directories(${CMAKE_SOURCE_DIR} ${EIGEN_INCLUDE_DIR} ${NABO_INCLUDE_DIR} ${Boost_LIBRARY_DIRS})
add_definitions(-Wall)

# main pointmatcher lib
add_library(pointmatcher
	pointmatcher/Core.cpp
	pointmatcher/DataPointsFilters.cpp
	pointmatcher/Matchers.cpp
	pointmatcher/OutlierFilters.cpp
	pointmatcher/ErrorMinimizers.cpp
	pointmatcher/Transformations.cpp
	pointmatcher/TransformationCheckers.cpp
	pointmatcher/Inspectors.cpp
	pointmatcher/IO.cpp
)
install(TARGETS pointmatcher ARCHIVE DESTINATION lib)
install(FILES
	pointmatcher/PointMatcher.h
	pointmatcher/TypeAliases.h
	pointmatcher/Core.h
	pointmatcher/IO.h
	DESTINATION include/pointmatcher
)

# Example programs
add_executable(icp examples/icp.cpp)
target_link_libraries(icp pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} rt)

add_executable(align_sequence examples/align_sequence.cpp)
target_link_libraries(align_sequence pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} rt)

add_executable(convertCSVtoVTK examples/convertCSVtoVTK.cpp)
target_link_libraries(convertCSVtoVTK pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} rt)

# Sandbox programs
if(COMPILE_SANDBOX)
  add_executable(sandbox_icp sandbox/icp.cpp)
  target_link_libraries(sandbox_icp pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} rt)

  add_executable(sandbox_align_seq sandbox/align_sequence.cpp)
  target_link_libraries(sandbox_align_seq pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} rt)

  add_executable(sandbox_CSVtoVTK sandbox/convertCSVtoVTK.cpp)
  target_link_libraries(sandbox_CSVtoVTK pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} rt)

endif(COMPILE_SANDBOX)

if(GTEST_INCLUDE_DIR AND GTEST_LIBRARY)
  include_directories(${GTEST_INCLUDE_DIR})
  add_executable(utest utest/utest.cpp)
  target_link_libraries(utest pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${GTEST_LIBRARY} rt)
else(GTEST_INCLUDE_DIR AND GTEST_LIBRARY)
  message("Google test (gtest) not found - Skipping unit tests")
endif(GTEST_INCLUDE_DIR AND GTEST_LIBRARY)
