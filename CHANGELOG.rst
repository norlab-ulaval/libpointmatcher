^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libpointmatcher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1.4.4 (2024-12-16)
------------------
**New features:**
* Added Angle Limit Datapointsfilter, which filters points if they lie inside or outside of a given spherical wedge
* Added support for binary PLY files IO

**Major changes:**
* Min. Cmake version pumped to 3.15
* Removed dependency on boost filesystem
* Python bindings compile with recent versions of pybind11

**Other changes:**
* Updated Python GitHub action
* Updated gitignore
* Updated docs


1.4.3 (2024-06-28)
------------------
* Class to generate point cloud primitives
* Updated documentation
* Github actions for code API
* Function that lists all available Filter parameters


1.4.2 (2024-03-23)
------------------
* Add dockerhub release logic and improve main readme by @RedLeader962 in https://github.com/norlab-ulaval/libpointmatcher/pull/550
* Update .gitingore with auto-generated patterns for C++, Python, JetBrains IDEs ,and VSCode by @boxanm in https://github.com/norlab-ulaval/libpointmatcher/pull/555
* build: add ubuntu jammy to the repository suported version by @RedLeader962 in https://github.com/norlab-ulaval/libpointmatcher/pull/557
* fix: Change unit tests floating point type to double and add a precision argument to output streams by @boxanm in https://github.com/norlab-ulaval/libpointmatcher/pull/558
* Update the minimum required Cmake version to 3.10.2 by @boxanm in https://github.com/norlab-ulaval/libpointmatcher/pull/560
* fix: Issue 534 transformation tests failing on some platforms by @boxanm in https://github.com/norlab-ulaval/libpointmatcher/pull/559
* Added orientation descriptor in RigidTransformation and SimilarityTransformation compute functions. by @simonpierredeschenes in https://github.com/norlab-ulaval/libpointmatcher/pull/553

1.4.1 (2024-03-19)
------------------
* Update package.xml version properly

1.4.0 (2023-12-15)
------------------
* fix: N2ST path resolution in dependencies-doc docker image
* refactor: move libpointmatcher build-system logic to norlab-build-system submodule
* fix: build-system side unstable compilation issue fix
* Fixes to CMake library management config generation
* New DataPoints filter for descriptor augmentation  Enhancement
* Fix 2D transformation tests in debug
* Fix Transformations test build when using Eigen3.4
* Disable static boost linkage by default
* Update CompilationPython.md
* Update README.md
* Update readme_test.md for docker daemon test
* Create readme_test.md
* Updated the inner loop counter
* build-system minor mod
* libpointmatcher build-system
* Fix omega descriptor export
* Handle libnabo config mode
* Build python binding as regular package
* Fix time values after applying Sampling surface normal filter
* Add seed to Random sampling filter
* Add more details in pypointmatcher's installation docs
* Fixed compilation on Visual Studio 2022
* Fix 4DoF PointToPlane error minimizer crash
* Use the LOG_INFO_STREAM macro instead of std::cerr
* package format=3
* Create LICENSE file based on BSD license as per package.xml
* catkin not required for pure cmake packages
* Histogram<T>::computeStats() without sorting the Histogram
* Windows: Fix Narrowing conversion of seed in MaxPointCount
* Change storage ordering of the eigen vectors descriptors
* Apply the transformation to eigen vectors
* Fix the surface normals datapoints filter covariance matrix bug
* Fix weird behavior of MaxPointCountDataPointsFilter
* Bug fix in SurfaceNormalOutlierFilter
* Update CompilationWindows.md
* [Matches/OutlierFiltersImpl] Made convergence error messages more informative
* RemoveBeyond option for the maxQuantileDistance filter
* refactor: Drop support for yaml-cpp old API  Enhancement
* Automaticaly find eigen3
* Strong Windows installation tutorial update
* Add libpointmatcher_INCLUDE_DIR to match with CGAL
* Add Boost_LIBRARIES to pointmatcher_LIBRARIES
* Add option to weight by reading pc
* Fixes for windows
* Fix windows doc
* Include iso646.h to add and, or, not macros
* Typo in PCL lib name
* Implemented an in-place method for transforming DataPoints objects
* The keyword "or" is not supported by windows compilers.
* Link against pthread
* [DataPoints] Added check to prevent unsigned int underflow in getEuclideanDim()
* [feature/spdf] Add SpectralDecompositionFilter (SpDF)
* Fixed the differences between examples and documentation (#409)
* Fix build of downstream packages.
* Reorganization of the compiling tutorials for ubuntu and macos
* Added reference for PointToPlaneWithCov ErrorMinimizer.
* [WIP] feature/python_module : Adding libpointmatcher's Python bindings (#222)  Enhancement
* Feature/speedup random sampling filter
* Replaced the remaining raw pointers with shared pointers in Registrar.h
* Adding the new outlierfilters documentation to mkdocs .yml
* Outlier filter documentation added
* Fix/typo Fixes tutorials building failure
* Fixes tutorials formatting problem (#373)
* [WIP] Fix/typo : Tutorials improvements
* Fix out-of-bounds access
* Add missing force4DOF param to PointToPlaneWithCov
* Modernize cmake; make cmake compatible with git submoduling
* Feature/4 dof for point to plane minimizer
* Feature/geometry data points filter for master
* Get rid of the Visual C++ warnings
* Change matrix type
* Update link to documentation in readme
* Fix segfault happening in ICPSequence class
* Added missing include that made windows compilation fail.
* Change icp chain image to an svg
* Add support for Travis
* Fix typo
* Improve speed of Normal Space filtering
* Fix/normal space hashing
* Fix/clamp normals

1.3.1 (2019-03-04)
------------------
* Added documentation for people using ROS.
* Increased libnabo minimal version to 1.0.7.
* Added interface to inform if maximum number of iterations was reached.
* Fixed portability issue of the FileLogger.
* Fixed unit tests on Windows.
* Fixed parameter-less modules having 'unknown' as class name.
* Updated Windows compilation tutorial.
* Fixed compilation problems on Windows.
* Fixed PointToPlan error residual.
* Changed DOI resolver link in documentation.
* Added validation for the input transformation matrix in ICP.cpp.
* Removed duplication of PointToPoint compute in PointToPointWithCov.
* Added the RemoveSensorBias filter.
* Splitted ErrorMinimizersImpl.cpp into multiple files.

1.3.0 (2018-10-26)
------------------
* Removed some boost utilities supported by c++11
* Replaced raw pointers by std smart pointers

1.2.4 (2018-10-15)
------------------
* Support of Eigen 3.3
* Introduced SurfaceNormalDataPointsFilter, OctreeGridDataPointsFilter and NormalSpaceDataPointsFilter
* A lot of bugs were fixed

1.2.3 (2015-05-15)
------------------
* Support including other versions of YAML in compilation units that also include the YAML version packed with libpointmatcher (PR #80)
* Changed immutability concept for SupportLabel to support MSVC 2012 (#78)
* Fixed build system related bugs (#79, #70, ..).
* updated build_map example, added better error message, added better information prints
* cleaned CMakeList and added missing dependencies for external projetcs
* avoid possibility of building dynamic library on MacOS
* updated Mac build instructions
* Tim3xx laser support on Simple Noise filter (#64)
* Modified default covariance return in PointToPlaneWithCovErrorMinimizer (#59)
* update usage text and retab
* Removed compilation warnings
* add unit test for ICPSequence
* added application of reference data points filters for ICPSequence objects (#56)
* Merge branch 'master' of github.com:ethz-asl/libpointmatcher
* fix problem with libnabo linking (#54)
* Adapted the code to handle 2D point clouds and decided to split the initial/icp/complete transformation matrices in 3 different files. It should be easier to post process the transformations.
* Changed matrix for matrices as output suffix
* Changed the ICP example (pmicp) to accept initial translation/rotation input and allow to output the transformation matrices
* CutBelowLevelDataPointsFilter (PR #48)
* split unit tests (PR #47)
* Delete roadmap.txt
* change year to 2014
* correct bug in DataPoints operator==
* add a method to remove features or descriptors
* add empty function for removing features and descriptors
* add functions to DataPoints avoiding error on rows and cols
* fill missing documentation
* resolve warning from unsigned to int in IO.cpp
* add extra empty line in utest
* add extra unit tests and resolve remaining bugs
* Refactored how to load PLY files
* Allow 2D descriptors (##45)
* Allow saving 2D descriptors coming from a 2Dmap, that are converted to 3D when writing to the file but needed after if we want to load the map as 2D.
* Contributors: Francis Colas, Francisco J Perez Grau, François Pomerleau, HannesSommer, Philipp Kruesi, Renaud Dube, Simon Lynen, chipironcin, pomerlef, smichaud, v01d

1.2.2 (2014-08-05)
------------------
* Yaml-cpp0.3 now built with libpointmatcher for compatibility with newer Ubuntu systems using yaml-cpp0.5

1.2.1
-----------
* Fixed bug with soft outlier weights in error minimization
* Fixed some issues for releasing into ROS ecosystem
* Contributors: François Pomerleau, Mike Bosse, Samuel Charreyron, Simon Lynen
