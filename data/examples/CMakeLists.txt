remake_file_configure(
  2D_oneBox.csv
  2D_twoBoxes.csv
  car_cloud400.csv
  car_cloud401.csv
  cloud.00000.vtk
  cloud.00001.vtk
  cloud.00002.vtk
  DESTINATION ${CMAKE_BINARY_DIR}/test
)
remake_add_files(*.vtk *.csv INSTALL examples)
