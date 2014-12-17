remake_file_configure(
  default*.yaml Sampling*.yaml
  DESTINATION ${CMAKE_BINARY_DIR}/test
)
remake_file_configure(
  bad*.yaml
  DESTINATION ${CMAKE_BINARY_DIR}/test/unit_tests
)
remake_add_configurations(
  default*.yaml Sampling*.yaml
  INSTALL test
)
