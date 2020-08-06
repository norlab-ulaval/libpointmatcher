cmake_minimum_required(VERSION 2.8.12)

include_directories(${CMAKE_SOURCE_DIR}/pointmatcher)
include_directories(${CMAKE_SOURCE_DIR}/pointmatcher/DataPointsFilters)
include_directories(${CMAKE_SOURCE_DIR}/pointmatcher/DataPointsFilters/utils)
include_directories(${CMAKE_SOURCE_DIR}/pointmatcher/ErrorMinimizers)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

foreach (file ${POINTMATCHER_SRC})
	set(PYTHON_SRC ${PYTHON_SRC} ../${file})
endforeach()

set(PYBIND11_SOURCES
#pointmatcher module
	pointmatcher/datapoints.cpp
	pointmatcher/datapointsfilter.cpp
	pointmatcher/datapointsfilters.cpp
	pointmatcher/errorminimizer.cpp
	pointmatcher/icp.cpp
	pointmatcher/icpchainbase.cpp
	pointmatcher/icpsequence.cpp
	pointmatcher/impls/inspectors_impl.cpp
	pointmatcher/impls/matchers_impl.cpp
	pointmatcher/impls/outlierfilters_impl.cpp
	pointmatcher/impls/transformations_impl.cpp
	pointmatcher/impls/transformationcheckers_impl.cpp
	pointmatcher/impl.cpp
	pointmatcher/inspector.cpp
	pointmatcher/matcher.cpp
	pointmatcher/matches.cpp
	pointmatcher/outlierfilter.cpp
	pointmatcher/outlierfilters.cpp
	pointmatcher/pointmatcher.cpp
	pointmatcher/transformation.cpp
	pointmatcher/transformations.cpp
	pointmatcher/transformationchecker.cpp
	pointmatcher/transformationcheckers.cpp

	modules/pointmatcher_module.cpp

#pointmatchersupport module
	pointmatchersupport/bibliography.cpp
	pointmatchersupport/logger.cpp
	pointmatchersupport/logger_impl.cpp
	pointmatchersupport/parametrizable.cpp
	pointmatchersupport/registrars/datapointsfilter_registrar.cpp
	pointmatchersupport/registrars/errorminimizer_registrar.cpp
	pointmatchersupport/registrars/inspector_registrar.cpp
	pointmatchersupport/registrars/logger_registrar.cpp
	pointmatchersupport/registrars/matcher_registrar.cpp
	pointmatchersupport/registrars/outlierfilter_registrar.cpp
	pointmatchersupport/registrars/transformation_registrar.cpp
	pointmatchersupport/registrars/transformationchecker_registrar.cpp
	pointmatchersupport/registrar.cpp

	modules/pointmatchersupport_module.cpp

#errorminimizers module
	errorminimizers/identity.cpp
	errorminimizers/pointtoplane.cpp
	errorminimizers/pointtoplanewithcov.cpp
	errorminimizers/pointtopoint.cpp
	errorminimizers/pointtopointsimilarity.cpp
	errorminimizers/pointtopointwithcov.cpp

	modules/errorminimizers_module.cpp

#datapointfilters module
	datapointsfilters/boundingbox.cpp
	datapointsfilters/covariancesampling.cpp
	datapointsfilters/cutatdescriptorthreshold.cpp
	datapointsfilters/distancelimit.cpp
	datapointsfilters/ellipsoids.cpp
	datapointsfilters/fixstepsampling.cpp
	datapointsfilters/gestalt.cpp
	datapointsfilters/identity.cpp
	datapointsfilters/incidenceangle.cpp
	datapointsfilters/maxdensity.cpp
	datapointsfilters/maxpointcount.cpp
	datapointsfilters/maxquantileonaxis.cpp
	datapointsfilters/normalspace.cpp
	datapointsfilters/observationdirection.cpp
	datapointsfilters/octreegrid.cpp
	datapointsfilters/orientnormals.cpp
	datapointsfilters/randomsampling.cpp
	datapointsfilters/removenan.cpp
	datapointsfilters/removesensorbias.cpp
	datapointsfilters/samplingsurfacenormal.cpp
	datapointsfilters/shadow.cpp
	datapointsfilters/simplesensornoise.cpp
	datapointsfilters/sphericality.cpp
	datapointsfilters/surfacenormal.cpp

	modules/datapointsfilters_module.cpp

# main module
	pypointmatcher.cpp)

find_package(pybind11 REQUIRED)

if (pybind11_FOUND)
	message(STATUS "pybind11 v${pybind11_VERSION}\n")

	pybind11_add_module(pypointmatcher
		${PYTHON_SRC}
		${PYBIND11_SOURCES})

	target_link_libraries(pypointmatcher PRIVATE ${EXTERNAL_LIBS})

	add_dependencies(pypointmatcher ${EXTRA_DEPS})
endif ()