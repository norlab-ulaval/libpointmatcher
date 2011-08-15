libpointmatcher is a modular ICP library, useful for robotics and computer vision.

libpointmatcher depends on:

 * [Eigen] version 3, a modern C++ matrix and linear-algebra library,
 * [libnabo], a fast K Nearest Neighbour library for low-dimensional spaces.

libpointmatcher is being developed by François Pomerleau and [Stéphane Magnenat](http://stephane.magnenat.net) as part of our work at [ASL-ETH](http://www.asl.ethz.ch).


Compilation
===========

libpointmatcher uses [CMake] as build system.
The complete compilation process depends on the system you are using (Linux, Mac OS X or Windows).
You will find a nice introductory tutorial in [this video](http://www.youtube.com/watch?v=CLvZTyji_Uw).

Prerequisites
-------------

If your operating system does not provide it, you must get [Eigen].
It only needs to be downloaded and extracted.
[libnabo] must be downloaded and installed, please follow the [libnabo]'s documentation to do so.

Quick compilation and installation under Unix
---------------------------------------------

Under Unix, assuming that [Eigen] and [libnabo] are installed system-wide, you can compile (with optimisation and debug information) and install libpointmatcher in `/usr/local` with the following commands run in the top-level directory of libpointmatcher's sources:

	SRC_DIR=`pwd`
	BUILD_DIR=${SRC_DIR}/build
	mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}
	cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ${SRC_DIR}
	# if Eigen or libnabo are not available system-wide, run at that point: 
	#   cmake-gui .
	# cmake-gui allows you to tell the location of the header
	# and the static libraries of Eigen and libnabo
	make
	sudo make install

These lines will compile libpointmatcher in a `build` sub-directory and therefore keep your source tree clean.
Note that you could compile libpointmatcher anywhere you have write access, such as in `/tmp/libpointmatcher.
This out-of-source build is a nice feature of [CMake] under Unixes.

If [Eigen] or [libnabo] are not installed system-wide, you might have to tell [CMake] where to find them.
You can do this with a command-line tool, `ccmake`, or with a graphical tool, `cmake-gui`.
Please read the [CMake documentation] for more information.
Note that if you have [ROS] (Diamondback or later) installed, Eigen 3 should be availabe in the path `/opt/ros/diamondback/stacks/geometry/eigen/include`.


Test
====

In 2D:

	./icp ${SRC_DIR}/examples/data/2D_oneBox.csv ${SRC_DIR}/examples/data/data/2D_twoBoxes.csv

In 3D:

	./icp ${SRC_DIR}/examples/data/car_cloud401.csv ${SRC_DIR}/examples/data/car_cloud400.csv

Use [Paraview] to view the results.
On [Ubuntu], you can install [Paraview] with `apt-get`:
	sudo-apt get install paraview


Bug reporting
=============

Please use [github's issue tracker](http://github.com/ethz-asl/libpointmatcher/issues) to report bugs.


License
=======

libpointmatcher is released under a permissive BSD license.

[Ubuntu]: http://www.ubuntu.com
[CMake]: http://www.cmake.org
[CMake documentation]: http://www.cmake.org/cmake/help/cmake2.6docs.html
[git]: http://git-scm.com
[Eigen]: http://eigen.tuxfamily.org
[libnabo]: http://github.com/ethz-asl/libnabo
[ROS]: http://www.ros.org/
[Paraview]: http://www.paraview.org/
