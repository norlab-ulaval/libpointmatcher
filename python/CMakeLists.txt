if("${PYTHON_INSTALL_TARGET}" STREQUAL "")
	message(STATUS "PYTHON_INSTALL_TARGET variable is not set, setting a default value...")
	execute_process(COMMAND "python3" "-c" "import site; print(site.getsitepackages()[0])"
	                OUTPUT_VARIABLE PYTHON_INSTALL_TARGET OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()

message(STATUS "The Python module will be install at this location : ${PYTHON_INSTALL_TARGET}")

set(PYBIND11_SOURCES
    #pointmatcher module
    pointmatcher/data_points.cpp
    pointmatcher/data_points_filter.cpp
    pointmatcher/data_points_filters.cpp
    pointmatcher/error_minimizer.cpp
    pointmatcher/icp.cpp
    pointmatcher/icp_chain_base.cpp
    pointmatcher/icp_sequence.cpp
    pointmatcher/impls/inspectors_impl.cpp
    pointmatcher/impls/matchers_impl.cpp
    pointmatcher/impls/outlier_filters_impl.cpp
    pointmatcher/impls/transformations_impl.cpp
    pointmatcher/impls/transformation_checkers_impl.cpp
    pointmatcher/impl.cpp
    pointmatcher/inspector.cpp
    pointmatcher/io.cpp
    pointmatcher/matcher.cpp
    pointmatcher/matches.cpp
    pointmatcher/outlier_filter.cpp
    pointmatcher/outlier_filters.cpp
    pointmatcher/point_matcher.cpp
    pointmatcher/transformation.cpp
    pointmatcher/transformations.cpp
    pointmatcher/transformation_checker.cpp
    pointmatcher/transformation_checkers.cpp

    #pointmatchersupport module
    pointmatchersupport/bibliography.cpp
    pointmatchersupport/logger.cpp
    pointmatchersupport/logger_impl.cpp
    pointmatchersupport/parametrizable.cpp
    pointmatchersupport/registrars/data_points_filter_registrar.cpp
    pointmatchersupport/registrars/error_minimizer_registrar.cpp
    pointmatchersupport/registrars/inspector_registrar.cpp
    pointmatchersupport/registrars/logger_registrar.cpp
    pointmatchersupport/registrars/matcher_registrar.cpp
    pointmatchersupport/registrars/outlier_filter_registrar.cpp
    pointmatchersupport/registrars/transformation_registrar.cpp
    pointmatchersupport/registrars/transformation_checker_registrar.cpp
    pointmatchersupport/registrar.cpp

    #errorminimizers module
    errorminimizers/identity.cpp
    errorminimizers/point_to_plane.cpp
    errorminimizers/point_to_plane_with_cov.cpp
    errorminimizers/point_to_point.cpp
    errorminimizers/point_to_point_similarity.cpp
    errorminimizers/point_to_point_with_cov.cpp

    #datapointfilters module
    datapointsfilters/bounding_box.cpp
    datapointsfilters/covariance_sampling.cpp
    datapointsfilters/cut_at_descriptor_threshold.cpp
    datapointsfilters/distance_limit.cpp
    datapointsfilters/ellipsoids.cpp
    datapointsfilters/fix_step_sampling.cpp
    datapointsfilters/gestalt.cpp
    datapointsfilters/identity.cpp
    datapointsfilters/incidence_angle.cpp
    datapointsfilters/max_density.cpp
    datapointsfilters/max_pointcount.cpp
    datapointsfilters/max_quantile_on_axis.cpp
    datapointsfilters/normal_space.cpp
    datapointsfilters/observation_direction.cpp
    datapointsfilters/octree_grid.cpp
    datapointsfilters/orient_normals.cpp
    datapointsfilters/random_sampling.cpp
    datapointsfilters/remove_nan.cpp
    datapointsfilters/remove_sensor_bias.cpp
    datapointsfilters/sampling_surface_normal.cpp
    datapointsfilters/shadow.cpp
    datapointsfilters/simple_sensor_noise.cpp
    datapointsfilters/sphericality.cpp
    datapointsfilters/surface_normal.cpp

    modules/point_matcher_module.cpp
    modules/point_matcher_support_module.cpp
    modules/data_points_filters_module.cpp
    modules/error_minimizers_module.cpp

    # main module
    pypoint_matcher.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_SOURCE_DIR}/pointmatcher
                    ${CMAKE_SOURCE_DIR}/pointmatcher/DataPointsFilters
                    ${CMAKE_SOURCE_DIR}/pointmatcher/DataPointsFilters/utils
                    ${CMAKE_SOURCE_DIR}/pointmatcher/ErrorMinimizers)

if(USE_SYSTEM_PYBIND11)
	find_package(pybind11 2.5.0 REQUIRED)
	message(STATUS "pybind11 v${pybind11_VERSION}")
else()
	add_subdirectory(pybind11)
	set(pybind11_FOUND TRUE)
endif()

if(pybind11_FOUND)
	pybind11_add_module(pypointmatcher ${PYBIND11_SOURCES})

	target_link_libraries(pypointmatcher
	                      PUBLIC
	                      pointmatcher)

	add_dependencies(pypointmatcher pointmatcher)

	install(TARGETS pypointmatcher LIBRARY DESTINATION ${PYTHON_INSTALL_TARGET})
else()
	message(FATAL_ERROR "pybind11 is required! Please follow the \"Compiling \
libpointmatcher's with Python\" instructions from the official libpointmatcher's documentation.")
endif()

