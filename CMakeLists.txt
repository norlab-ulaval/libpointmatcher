cmake_minimum_required(VERSION 2.6)

include (CheckSymbolExists)

project(libpointmatcher)

# c++ 0x
#if("${CMAKE_CXX_COMPILER_ID}" MATCHES "clang") 
#	set(CMAKE_COMPILER_IS_CLANGXX)
#endif ("${CMAKE_CXX_COMPILER_ID}" MATCHES "clang")
if(CMAKE_COMPILER_IS_GNUCXX)
	execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
	if (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
		add_definitions(-std=gnu++0x)
	else (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
		message(SEND_ERROR "You need GCC version 4.4 or greater to compile this library")
	endif (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
	add_definitions(-Wstrict-overflow=0)
else(CMAKE_COMPILER_IS_GNUCXX)
	message(SEND_ERROR "You need partial C++0x support for N2351, N2672, N1984. Currently this build toolchain supports only GCC >= 4.4. If your compiler supports these specifications, please send us a patch.")
endif(CMAKE_COMPILER_IS_GNUCXX)
# well, this breaks libpointmatcher on non-unix platform,
# please contribute the define for your favorite compiler
# It must support (compiler support in parenthesis):
# - N2351: shared pointers in standard
# - N2672: Initializer lists (gcc)
# - N1984: auto-typed variables (gcc, clang)

#include(UseDoxygen)

# initially
set(EXTRA_LIBS "")

# boost
find_package(Boost COMPONENTS thread system filesystem REQUIRED)

# eigen 3
find_path(EIGEN_INCLUDE_DIR Eigen/Core
	/usr/local/include/eigen3
	/usr/include/eigen3
	/opt/ros/diamondback/stacks/geometry/eigen/include
)

# nabo
find_path(NABO_INCLUDE_DIR nabo/nabo.h
	/usr/local/include
	/usr/include
)
find_library(NABO_LIBRARY NAMES libnabo1.a libnabo1 PATHS
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
	set(EXTRA_LIBS ${EXTRA_LIBS} ${OPENCL_LIBRARIES})
endif (OPENCL_LIBRARIES)

# yaml-cpp
find_path(yaml-cpp_INCLUDE_DIRS yaml-cpp/yaml.h
	/usr/local/include
	/usr/include
)
find_library(yaml-cpp_LIBRARIES yaml-cpp PATHS
	/usr/local/lib
	/usr/lib
NO_DEFAULT_PATH)
if (yaml-cpp_INCLUDE_DIRS AND yaml-cpp_LIBRARIES)
	include_directories(${yaml-cpp_INCLUDE_DIRS})
	set(EXTRA_LIBS ${EXTRA_LIBS} ${yaml-cpp_LIBRARIES})
	add_definitions(-DHAVE_YAML_CPP)
endif (yaml-cpp_INCLUDE_DIRS AND yaml-cpp_LIBRARIES)

# link rt support if POSIX timers are used
check_symbol_exists(_POSIX_TIMERS "unistd.h;time.h" POSIX_TIMERS)
if (POSIX_TIMERS)
	set(EXTRA_LIBS ${EXTRA_LIBS} rt)
endif (POSIX_TIMERS)

# include all libs so far
include_directories(${CMAKE_SOURCE_DIR} ${EIGEN_INCLUDE_DIR} ${NABO_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
add_definitions(-Wall)

# main pointmatcher lib
add_library(pointmatcher
	pointmatcher/Timer.cpp
	pointmatcher/Histogram.cpp
	pointmatcher/Parametrizable.cpp
	pointmatcher/Logger.cpp
	pointmatcher/Core.cpp
	pointmatcher/DataPointsFilters.cpp
	pointmatcher/Matchers.cpp
	pointmatcher/OutlierFilters.cpp
	pointmatcher/ErrorMinimizers.cpp
	pointmatcher/Transformations.cpp
	pointmatcher/TransformationCheckers.cpp
	pointmatcher/Inspectors.cpp
	pointmatcher/IO.cpp
	pointmatcher/Bibliography.cpp
)
install(TARGETS pointmatcher ARCHIVE DESTINATION lib)
install(FILES
	pointmatcher/PointMatcher.h
	pointmatcher/Histogram.h
	pointmatcher/Parametrizable.h
	pointmatcher/Registrar.h
	DESTINATION include/pointmatcher
)

# Example programs
add_executable(icp examples/icp.cpp)
target_link_libraries(icp pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_THREAD_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} rt)

add_executable(icp_simple examples/icp_simple.cpp)
target_link_libraries(icp_simple pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_THREAD_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} rt)

add_executable(align_sequence examples/align_sequence.cpp)
target_link_libraries(align_sequence pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} rt)

add_executable(convertCSVtoVTK examples/convertCSVtoVTK.cpp)
target_link_libraries(convertCSVtoVTK pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} rt)

add_executable(list_modules examples/list_modules.cpp)
target_link_libraries(list_modules pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_THREAD_LIBRARY} rt)


if(GTEST_INCLUDE_DIR AND GTEST_LIBRARY)
	include_directories(${GTEST_INCLUDE_DIR})
  add_executable(utest utest/utest.cpp)
  target_link_libraries(utest pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${GTEST_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} rt)
else(GTEST_INCLUDE_DIR AND GTEST_LIBRARY)
  message("Google test (gtest) not found - Skipping unit tests")
endif(GTEST_INCLUDE_DIR AND GTEST_LIBRARY)
