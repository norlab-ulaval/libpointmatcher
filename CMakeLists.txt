cmake_minimum_required(VERSION 2.6)

include (CheckSymbolExists)

project(libpointmatcher)

# Extract version from header
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
execute_process(
	COMMAND grep "POINTMATCHER_VERSION " pointmatcher/PointMatcher.h
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	RESULT_VARIABLE GREP_VERSION_RESULT
	OUTPUT_VARIABLE PROJECT_VERSION
	OUTPUT_STRIP_TRAILING_WHITESPACE
)
if (NOT GREP_VERSION_RESULT EQUAL 0)
	message(SEND_ERROR "Cannot grep version number: ${GREP_VERSION_RESULT}")
endif (NOT GREP_VERSION_RESULT EQUAL 0)
string(REGEX REPLACE ".*\"(.*)\".*" "\\1" PROJECT_VERSION "${PROJECT_VERSION}" )

# Documentation
set(DOXYFILE_LATEX false)
include(UseDoxygen)

# initially
set(EXTRA_LIBS "")

# boost
find_package(Boost COMPONENTS thread filesystem system program_options date_time REQUIRED)
if (Boost_MINOR_VERSION GREATER 47)
	find_package(Boost COMPONENTS thread filesystem system program_options date_time chrono REQUIRED)
endif (Boost_MINOR_VERSION GREATER 47)

# eigen 3
find_path(EIGEN_INCLUDE_DIR Eigen/Core
	/usr/local/include/eigen3
	/usr/include/eigen3
)

# nabo
find_path(NABO_INCLUDE_DIR nabo/nabo.h
	/usr/local/include
)
find_library(NABO_LIBRARY NAMES libnabo.a nabo PATHS
	/usr/local/lib
)

# optionally, look for OpenCL library
set(USE_OPEN_CL "false" CACHE BOOL "Set to ON to look for OpenCL, if your libnabo was compiled with CL support")
if (USE_OPEN_CL)
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
endif(USE_OPEN_CL)

# yaml-cpp
find_path(yaml-cpp_INCLUDE_DIRS yaml-cpp/yaml.h
	/usr/local/include
)
find_library(yaml-cpp_LIBRARIES yaml-cpp PATHS
	/usr/local/lib
)
if (yaml-cpp_INCLUDE_DIRS AND yaml-cpp_LIBRARIES)
	include_directories(${yaml-cpp_INCLUDE_DIRS})
	set(EXTRA_LIBS ${EXTRA_LIBS} ${yaml-cpp_LIBRARIES})
	add_definitions(-DHAVE_YAML_CPP)
	set(yamlcpp_FOUND)
	message("-- yaml-cpp found, text-based configuration enabled")
else (yaml-cpp_INCLUDE_DIRS AND yaml-cpp_LIBRARIES)
	message("-- yaml-cpp not found, text-based configuration and related applications disabled")
endif (yaml-cpp_INCLUDE_DIRS AND yaml-cpp_LIBRARIES)

# link rt support if POSIX timers are used
check_symbol_exists(_POSIX_TIMERS "unistd.h;time.h" POSIX_TIMERS)
if (POSIX_TIMERS AND NOT APPLE)
	set(EXTRA_LIBS ${EXTRA_LIBS} rt)
endif (POSIX_TIMERS AND NOT APPLE)

# include all libs so far
include_directories(${CMAKE_SOURCE_DIR} ${EIGEN_INCLUDE_DIR} ${NABO_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
add_definitions(-Wall)

# Pointmatcher lib and install
set(POINTMATCHER_SRC
	pointmatcher/Logger.cpp
	pointmatcher/Exceptions.cpp
	pointmatcher/DataPoints.cpp
	pointmatcher/Matches.cpp
	pointmatcher/ICP.cpp
	pointmatcher/Registry.cpp
	pointmatcher/DataPointsFilter.cpp
	pointmatcher/Matcher.cpp
	pointmatcher/OutlierFilter.cpp
	pointmatcher/ErrorMinimizer.cpp
	pointmatcher/Transformation.cpp
	pointmatcher/TransformationChecker.cpp
	pointmatcher/Inspector.cpp
	pointmatcher/IO.cpp
	pointmatcher/Bibliography.cpp
	pointmatcher/Timer.cpp
	pointmatcher/Histogram.cpp
	pointmatcher/Parametrizable.cpp
	pointmatcher/LoggerImpl.cpp
	pointmatcher/DataPointsFiltersImpl.cpp
	pointmatcher/MatchersImpl.cpp
	pointmatcher/OutlierFiltersImpl.cpp
	pointmatcher/ErrorMinimizersImpl.cpp
	pointmatcher/TransformationsImpl.cpp
	pointmatcher/TransformationCheckersImpl.cpp
	pointmatcher/InspectorsImpl.cpp
)
set(SHARED_LIBS "false" CACHE BOOL "To build shared (true) or static (false) library")
if (SHARED_LIBS)
	add_library(pointmatcher SHARED ${POINTMATCHER_SRC})
	install(TARGETS pointmatcher LIBRARY DESTINATION lib)
else(SHARED_LIBS)
	add_library(pointmatcher ${POINTMATCHER_SRC})
	install(TARGETS pointmatcher ARCHIVE DESTINATION lib)
endif(SHARED_LIBS)
set_target_properties(pointmatcher PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

# create doc before installing
set(DOC_INSTALL_TARGET "share/doc/${PROJECT_NAME}/api" CACHE STRING "Target where to install doxygen documentation")
add_dependencies(pointmatcher doc)
install(FILES
	pointmatcher/PointMatcher.h
	pointmatcher/PointMatcherPrivate.h
	pointmatcher/Parametrizable.h
	pointmatcher/Registrar.h
	pointmatcher/Timer.h
	pointmatcher/IO.h
	DESTINATION include/pointmatcher
)
install(FILES README.md DESTINATION share/doc/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/html DESTINATION ${DOC_INSTALL_TARGET})

# Example programs
add_subdirectory(examples)
# Evaluation programs
add_subdirectory(evaluations)

# Unit testing
enable_testing()
add_subdirectory(utest)
