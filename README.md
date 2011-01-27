libpointmatcher is a modular ICP library, useful for robotics and computer vision.

libpointmatcher depends on:

 * [Eigen], a modern C++ matrix and linear-algebra library,
 * [libnabo], a fast K Nearest Neighbour library for low-dimensional spaces.
 
libpointmatcher is being developed by François Pomerleau and Stéphane Magnenat as part of our work at [ASL-ETH](http://www.asl.ethz.ch).


Compilation
-----------

libpointmatcher uses [CMake] as build system.
Just create a directory, go inside it and type:

	cmake SRC_DIR
    
where `SRC_DIR` is the top-level directory of libpointmatcher's sources.
If the dependencies are not installed system wide, you might have to tell [CMake] where to find them.
In case of doubt, read the [CMake documentation].

You first need to fetch and compile [libnabo].
To do so, you need [cmake], [git] and [Eigen].
On [Ubuntu], you can install these with `apt-get`:

	sudo apt-get install git-core cmake cmake-qt-gui libeigen2-dev

Then, you need to clone and build [libnabo]:

	git clone http://github.com/ethz-asl/libnabo
	mkdir build
	cd build
	cmake ..

Then, in the directory in which you are building libpointmatcher, launch `cmake-gui .` and specify the location of [libnabo]'s headers and static library.


Test
----

In 2D:
	./icp SRC_DIR/tests/data/2D_oneBox.csv SRC_DIR/tests/data/data/2D_twoBoxes.csv

In 3D:
	./icp SRC_DIR/tests/data/car_cloud401.csv SRC_DIR/tests/data/car_cloud400.csv

Use [Paraview] to view the results.
On [Ubuntu], you can install [Paraview] with `apt-get`:
	sudo-apt get install paraview


Bug reporting
-------------

Please use [github's issue tracker](http://github.com/ethz-asl/libpointmatcher/issues) to report bugs.


License
-------

libpointmatcher is released under a permissive BSD license.

[Ubuntu]: http://www.ubuntu.com
[CMake]: http://www.cmake.org
[CMake documentation]: http://www.cmake.org/cmake/help/cmake2.6docs.html
[git]: http://git-scm.com
[Eigen]: http://eigen.tuxfamily.org
[libnabo]: http://github.com/ethz-asl/libnabo
[Paraview]: http://www.paraview.org/
