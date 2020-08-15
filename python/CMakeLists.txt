find_package(pybind11 2.5.0 REQUIRED)

if ("${PYTHON_INSTALL_TARGET}" STREQUAL "")
	message(STATUS "PYTHON_INSTALL_TARGET variable is not set, setting to a default one...")
	execute_process(COMMAND "python3" "-c" "import site; print(site.getsitepackages()[0])"
					OUTPUT_VARIABLE PYTHON_INSTALL_TARGET OUTPUT_STRIP_TRAILING_WHITESPACE)
endif ()

message(STATUS "The Python module will be install at this location : ${PYTHON_INSTALL_TARGET}")

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

	#errorminimizers module
	errorminimizers/identity.cpp
	errorminimizers/pointtoplane.cpp
	errorminimizers/pointtoplanewithcov.cpp
	errorminimizers/pointtopoint.cpp
	errorminimizers/pointtopointsimilarity.cpp
	errorminimizers/pointtopointwithcov.cpp

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

	modules/pointmatcher_module.cpp
	modules/pointmatchersupport_module.cpp
	modules/datapointsfilters_module.cpp
	modules/errorminimizers_module.cpp

	# main module
	pypointmatcher.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
					${CMAKE_SOURCE_DIR}/pointmatcher
					${CMAKE_SOURCE_DIR}/pointmatcher/DataPointsFilters
					${CMAKE_SOURCE_DIR}/pointmatcher/DataPointsFilters/utils
					${CMAKE_SOURCE_DIR}/pointmatcher/ErrorMinimizers)

if (pybind11_FOUND)
	message(STATUS "pybind11 v${pybind11_VERSION}")

	pybind11_add_module(pypointmatcher ${PYBIND11_SOURCES})

	target_link_libraries(pypointmatcher
						  PRIVATE
						  pointmatcher
						  ${EXTERNAL_LIBS})

	add_dependencies(pypointmatcher pointmatcher)

	install(TARGETS pypointmatcher LIBRARY DESTINATION ${PYTHON_INSTALL_TARGET})
else ()
	message(FATAL_ERROR "pybind11 version required! Please follow the \"Compiling \
libpointmatcher's with Python\" instructions from the official libpointmatcher's documentation.")
endif ()

