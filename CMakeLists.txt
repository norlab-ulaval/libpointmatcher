cmake_minimum_required(VERSION 2.6)

project(libpointmatcher)

#include(UseDoxygen)

# boost
#find_package(Boost REQUIRED)

# eigen 3
find_path(EIGEN_INCLUDE_DIR Eigen/Core
	/usr/local/include/eigen
	/usr/include/eigen
)

# nabo
find_path(NABO_INCLUDE_DIR nabo/nabo.h
	/usr/local/include
	/usr/include
)
find_library(NABO_LIBRARY libnabo
	/usr/local/lib
	/usr/lib
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
include_directories(${CMAKE_SOURCE_DIR} ${EIGEN_INCLUDE_DIR} ${NABO_INCLUDE_DIR})
add_definitions(-Wall)

# main pointmatcher lib
add_library(pointmatcher
	pointmatcher/Core.cpp
	pointmatcher/DataPointsFilters.cpp
	pointmatcher/Matchers.cpp
	pointmatcher/OutlierFilters.cpp
	pointmatcher/ErrorMinimizer.cpp
	pointmatcher/Transformation.cpp
	pointmatcher/TransformationChecker.cpp
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

# test programs
add_executable(icp examples/icp.cpp)
target_link_libraries(icp pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} rt)

add_executable(alignclouds examples/alignclouds.cpp)
target_link_libraries(alignclouds pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} rt)

add_executable(testvtk examples/testvtk.cpp)
target_link_libraries(testvtk pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} rt)

