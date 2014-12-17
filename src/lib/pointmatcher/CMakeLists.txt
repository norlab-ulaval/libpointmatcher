remake_add_library(
  pointmatcher PREFIX OFF
  LINK ${Boost_LIBRARIES} ${YAML_CPP_LIBRARIES} ${LIBNABO_LIBRARIES}
)
remake_add_headers(
  IO.h
  PointMatcher.h
  PointMatcherPrivate.h
  Parametrizable.h
  Registrar.h
  Timer.h
)
remake_pkg_config_generate(
  REQUIRES eigen${LIBPOINTMATCHER_EIGEN_VERSION} libnabo
)
