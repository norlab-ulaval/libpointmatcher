Release Notes
=============

Wish list for next release
--------------------------

 * Improve *.csv point cloud support
 * Improve *.vtk legacy format to handle UNSTRUCTURED_GRID format
 * Fix portability problem with FileLogger on Windows
 * Support for OpenMP parallel computing
 * Handle larger point clouds (100 million points and more)

Already implemented in the current master:
  
 * Better user documentation and tutorials (5 Fev. 2014 thanks to Samuel Charreyron)
 * Avoid point cloud copies when filtering (10 Jan. 2014 thanks to Oleg Alexandrov)
 * Add CMake support for find_package (20 Sept. 2013)
 * Added support for PLY polygon file format (ascii) (14 March, 2014)


Version 1.1.0
--------------

 * Removed C++0x dependency 
 * Added compatibility with Visual Studio 11
 * Fixed various bugs
 * Added optimisation of 2-D pose with 3-D data
 
Version 1.0.0
-------------

 * First release with a stable API
