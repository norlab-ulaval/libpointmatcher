remake_file_configure(
  *.ref_trans
  DESTINATION ${CMAKE_BINARY_DIR}/test/icp_data
)
remake_add_files(*.ref_trans INSTALL test)
