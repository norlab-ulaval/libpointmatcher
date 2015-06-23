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

# Check if 32 bit platform
# By default, libpointmatcher is not compatible with and will not build on a 
# 32 bit system
if( NOT CMAKE_SIZEOF_VOID_P EQUAL 8 ) 
    MESSAGE(SEND_ERROR "32 bits compiler detected. Libpointmatcher is only supported in 64 bits." )
    SET( EX_PLATFORM 32 )
    SET( EX_PLATFORM_NAME "x86" )
endif( NOT CMAKE_SIZEOF_VOID_P EQUAL 8 )

## WARNING: unsupported
## To force install as a 32 bit library, set BUILD_32 to true
if( BUILD_32 )
	MESSAGE(STATUS "Building as a 32 bit library")
	SET(CMAKE_CXX_FLAGS "-m32")
endif( BUILD_32 )

# Ensure proper build type
if (NOT CMAKE_BUILD_TYPE)
  message("-- No build type specified; defaulting to CMAKE_BUILD_TYPE=Release.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
else (NOT CMAKE_BUILD_TYPE)
  if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("\n=================================================================================")
    message("\n-- Build type: Debug. Performance will be terrible!")
    message("-- Add -DCMAKE_BUILD_TYPE=Release to the CMake command line to get an optimized build.")
    message("\n=================================================================================")
  endif (CMAKE_BUILD_TYPE STREQUAL "Debug")
endif (NOT CMAKE_BUILD_TYPE)


#================= extra building definitions ==============================
if (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-O3)
endif(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")

# For Windows
if( MSVC ) # VS2012 does not support tuples correctly yet
	add_definitions( /D _VARIADIC_MAX=10 )
endif()

#======================= installation =====================================

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH
  "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
	set(DEF_INSTALL_CMAKE_DIR CMake)
else()
	set(DEF_INSTALL_CMAKE_DIR lib/cmake/pointmatcher)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "Installation directory for CMake files")


# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
	set(var INSTALL_${p}_DIR)
	if(NOT IS_ABSOLUTE "${${var}}")
		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
	endif()
endforeach()

# Install package.xml for catkin
install(FILES package.xml DESTINATION "share/${PROJECT_NAME}")

#===========================================================================

# Documentation
set(DOXYFILE_LATEX false)
include(UseDoxygen)

# initially
set(EXTERNAL_LIBS "")
set(EXTRA_DEPS "")

# compile local version of gtest and yaml-cpp
add_subdirectory(contrib)


#--------------------
# DEPENDENCY:  boost
#--------------------
find_package(Boost COMPONENTS thread filesystem system program_options date_time REQUIRED)
if (Boost_MINOR_VERSION GREATER 47)
	find_package(Boost COMPONENTS thread filesystem system program_options date_time chrono REQUIRED)
endif (Boost_MINOR_VERSION GREATER 47)
include_directories(${Boost_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${Boost_LIBRARIES})


#--------------------
# DEPENDENCY: eigen 3
#--------------------
find_path(EIGEN_INCLUDE_DIR Eigen/Core
	/usr/local/include/eigen3
	/usr/include/eigen3
)
include_directories(${EIGEN_INCLUDE_DIR})
#note: no library to add, eigen rely only on headers 

#TODO: this should be a more standard way
#find_package(Eigen3 REQUIRED)
#message("-- eigen3 found, version ${Eigen3_VERSION}")



#--------------------
# DEPENDENCY: nabo
#--------------------
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${LIBNABO_INSTALL_DIR}/share/libnabo/cmake/")
find_package(libnabo REQUIRED)
#include(libnaboConfig)
include_directories(${libnabo_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${libnabo_LIBRARIES})
message("-- libnabo found, version ${libnabo_VERSION}")


#--------------------
# DEPENDENCY: OpenCL (optional)
#--------------------
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
		set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${OPENCL_LIBRARIES})
    message("-- openCL found, parallel computer enabled for kd-tree")
	endif (OPENCL_LIBRARIES)
endif(USE_OPEN_CL)


#--------------------
# DEPENDENCY: yaml-cpp (local or system, optional)
#--------------------
option(USE_SYSTEM_YAML_CPP "Use system version of yaml-cpp rather than one packaged with libpointmatcher" FALSE)

if(USE_SYSTEM_YAML_CPP)
    message(STATUS "Looking for yaml-cpp on system")
    find_path(yaml-cpp_INCLUDE_DIRS yaml-cpp/yaml.h
            /usr/local/include
    )
    find_library(yaml-cpp_LIBRARIES yaml-cpp PATHS
            /usr/local/lib
    )
    if(yaml-cpp_INCLUDE_DIRS AND yaml-cpp_LIBRARIES)
            include_directories(${yaml-cpp_INCLUDE_DIRS})
            add_definitions(-DSYSTEM_YAML_CPP)
            set(yamlcpp_FOUND)
            set (EXTERNAL_LIBS ${EXTERNAL_LIBS} ${yaml-cpp_LIBRARIES} )
            message("-- yaml-cpp found, text-based configuration enabled")
    else(yaml-cpp_INCLUDE_DIRS AND yaml-cpp_LIBRARIES)
            message("-- yaml-cpp not found, text-based configuration and related applications disabled")
    endif(yaml-cpp_INCLUDE_DIRS AND yaml-cpp_LIBRARIES)
else(USE_SYSTEM_YAML_CPP)
        include_directories(contrib/yaml-cpp-pm/include)
        
#note: this is not working....
        #get_property(yaml-cpp-pm_INCLUDE TARGET yaml-cpp-pm PROPERTY INCLUDE_DIRECTORIES)
        #include_directories(${yaml-cpp-pm_INCLUDE})
        
        get_property(yaml-cpp-pm_LIB TARGET yaml-cpp-pm PROPERTY LOCATION)
        set (EXTERNAL_LIBS ${EXTERNAL_LIBS} ${yaml-cpp-pm_LIB} )
        set (EXTRA_DEPS ${EXTRA_DEPS} yaml-cpp-pm)
        set(yamlcpp_FOUND)

        get_property(yaml-cpp-pm_VERSION TARGET yaml-cpp-pm PROPERTY VERSION)
        message("-- using built-in yaml-cpp, version ${yaml-cpp-pm_VERSION}")
        message("   -- text-based configuration enabled")
endif(USE_SYSTEM_YAML_CPP)


#--------------------
# DEPENDENCY: rt (optional)
#--------------------
# link rt support if POSIX timers are used
check_symbol_exists(_POSIX_TIMERS "unistd.h;time.h" POSIX_TIMERS)
if (POSIX_TIMERS AND NOT APPLE)
	set(EXTERNAL_LIBS ${EXTERNAL_LIBS} rt)
endif (POSIX_TIMERS AND NOT APPLE)


# SOURCE

# Pointmatcher lib and install
set(POINTMATCHER_SRC
	pointmatcher/Logger.cpp
	pointmatcher/Exceptions.cpp
	pointmatcher/DataPoints.cpp
	pointmatcher/Matches.cpp
	pointmatcher/ICP.cpp
	pointmatcher/Registry.cpp
	pointmatcher/Registrar.cpp
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

include_directories(${CMAKE_SOURCE_DIR})


file(GLOB POINTMATCHER_HEADERS "pointmatcher/*.h")

# MacOS deals with shared/dynamic library differently. For
# simplicity, we only authorize static library in that case.
if(APPLE)

	add_library(pointmatcher ${POINTMATCHER_SRC} ${POINTMATCHER_HEADERS} )
	install(TARGETS pointmatcher ARCHIVE DESTINATION ${INSTALL_LIB_DIR})

else(APPLE)
	set(SHARED_LIBS "true" CACHE BOOL "To build shared (true) or static (false) library")

	if (SHARED_LIBS)
		add_library(pointmatcher SHARED ${POINTMATCHER_SRC} ${POINTMATCHER_HEADERS})
		install(TARGETS pointmatcher LIBRARY DESTINATION ${INSTALL_LIB_DIR})
	else(SHARED_LIBS)
		add_library(pointmatcher ${POINTMATCHER_SRC} ${POINTMATCHER_HEADERS} )
		install(TARGETS pointmatcher ARCHIVE DESTINATION ${INSTALL_LIB_DIR})
	endif(SHARED_LIBS)
endif(APPLE)


# link all libraries to libpointmatcher
add_definitions(-Wall)
#target_link_libraries(pointmatcher ${yaml-cpp_LIBRARIES} ${libnabo_LIBRARIES})
target_link_libraries(pointmatcher ${EXTERNAL_LIBS})

if(EXTRA_DEPS)
	add_dependencies(pointmatcher ${EXTRA_DEPS})
endif(EXTRA_DEPS)
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
	DESTINATION ${INSTALL_INCLUDE_DIR}/pointmatcher
)
install(FILES README.md DESTINATION share/doc/${PROJECT_NAME})
if (DOXYGEN_FOUND)
	install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/html DESTINATION ${DOC_INSTALL_TARGET})
endif (DOXYGEN_FOUND)


#=============== trigger other makefile ======================

# Example programs
add_subdirectory(examples)
# Evaluation programs
add_subdirectory(evaluations)
# Unit testing
add_subdirectory(utest)


#=================== allow find_package() =========================
# 
# the following case be used in an external project requiring libnabo:
#  ...
#  find_package(libpointmatcher) 
#  include_directories(${libpointmatcher_INCLUDE_DIRS}) 
#  target_link_libraries(executableName ${libpointmatcher_LIBRARIES})
#  ...

# 1- local build #

# Register the local build in case one doesn't use "make install"
export(PACKAGE libpointmatcher)

file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}" "${INSTALL_INCLUDE_DIR}")
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}")

# Create variable for the local build tree
get_property(CONF_INCLUDE_DIRS DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

# Create variable with the library location
get_target_property(POINTMATCHER_LIB pointmatcher LOCATION)

# Configure config file for local build tree
configure_file(libpointmatcherConfig.cmake.in
  "${PROJECT_BINARY_DIR}/libpointmatcherConfig.cmake" @ONLY)

# 2- installation build #

# Change the include location for the case of an install location
set(CONF_INCLUDE_DIRS ${INSTALL_INCLUDE_DIR} ${CONF_INCLUDE_DIRS} )

#FIXME: this will only be applied to installed files. Confirm that we want that.
# gather all the includes but remove ones in the source tree
# because we added an include for the local yaml-cpp-pm we should also remove it
list(REMOVE_ITEM CONF_INCLUDE_DIRS ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/contrib/yaml-cpp-pm/include)

# Change the library location for an install location
get_filename_component(POINTMATCHER_LIB_NAME ${POINTMATCHER_LIB} NAME)
set(POINTMATCHER_LIB ${INSTALL_LIB_DIR}/${POINTMATCHER_LIB_NAME})

# We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
configure_file(libpointmatcherConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/libpointmatcherConfig.cmake" @ONLY)

# The same versioning file can be used for both cases
configure_file(libpointmatcherConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/libpointmatcherConfigVersion.cmake" @ONLY)
 

# Install the libpointmatcherConfig.cmake and libpointmatcherConfigVersion.cmake
install(FILES
   "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/libpointmatcherConfig.cmake"
	 "${PROJECT_BINARY_DIR}/libpointmatcherConfigVersion.cmake"
		   DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# useful for TRADR european project. TODO: check to use the other install
install (FILES "${PROJECT_BINARY_DIR}/libpointmatcherConfig.cmake" DESTINATION "share/${PROJECT_NAME}/cmake/")


#====================== uninstall target ===============================
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
