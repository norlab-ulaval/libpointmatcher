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
    pointmatcher/point_cloud_generator.cpp
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
    datapointsfilters/add_descriptor.cpp
    datapointsfilters/angle_limit.cpp
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

# Requires CMake 3.15
find_package(Python COMPONENTS Interpreter Development REQUIRED)
if (Python_FOUND)
    message(STATUS "Python found: ${Python_VERSION}")
else()
    message(FATAL_ERROR "Python not found")
endif()

# Use Python to locate pybind11 installed via pip
execute_process(COMMAND ${Python_EXECUTABLE} -c "import pybind11; print(pybind11.get_cmake_dir())"
                OUTPUT_VARIABLE pybind11_DIR
                OUTPUT_STRIP_TRAILING_WHITESPACE)

find_package(pybind11 CONFIG REQUIRED)
if(pybind11_FOUND)
    message(STATUS "Found pybind11 v${pybind11_VERSION} at ${PYBIND11_CMAKE_DIR}")
	pybind11_add_module(pypointmatcher_native ${PYBIND11_SOURCES})

	target_link_libraries(pypointmatcher_native
	                      PUBLIC
	                      pointmatcher)

	add_dependencies(pypointmatcher_native pointmatcher)
	message(STATUS "The Python module will be install at this location : ${CMAKE_CURRENT_SOURCE_DIR}/pypointmatcher")

	install(TARGETS pypointmatcher_native LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/pypointmatcher)
else()
	message(FATAL_ERROR "pybind11 is required! Please follow the \"Compiling \
libpointmatcher's with Python\" instructions from the official libpointmatcher's documentation.")
endif()
