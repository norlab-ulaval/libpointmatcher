

Documentation and Tutorials
===========================
libpointmatcher is a modular ICP library, useful for robotics and computer vision.

**Quick link for the tutorial pages: [Tutorials](doc/Tutorials.md ).**

Those tutorials are written using Markdown syntax and stored in the project's `/doc` folder.  Their scope ranges from introductory material on performing point cloud registration to instructions for the more experienced developer on how to extend the library's codebase. 

Libpointmatcher's source code is fully documented based on doxygen to provide an easy API to developers. An example of this API can be found [here](http://docs.ros.org/groovy/api/libpointmatcher/html/), but it is suggested to use the one build for your version in `doc/html`.  


libpointmatcher depends on:

 * [Eigen] version 3, a modern C++ matrix and linear-algebra library,
 * [libnabo] version 1.0.1, a fast K Nearest Neighbour library for low-dimensional spaces.
 * [Doxygen], a documentation-generation tool
 * [yaml-cpp], a YAML 1.2 parser and emitter (optional)
 
and was compiled on:
  * [Ubuntu](/doc/Compilation.md)
  * [Windows](/doc/CompilationWindows.md)
  * [Mac OS X](/doc/CompilationMac.md)

libpointmatcher is being developed by [François Pomerleau](mailto:f.pomerleau@gmail.com) and [Stéphane Magnenat](http://stephane.magnenat.net) as part of our work at [ASL-ETH](http://www.asl.ethz.ch).

You can read the latest changes in the [release notes](doc/ReleaseNotes.md).

Pre-built binaries
==================

Ubuntu
------

Binaries for Ubuntu Linux are available on our PPA:

 * [Lucid](https://launchpad.net/~stephane.magnenat/+archive/lucid)
 * [Precise](https://launchpad.net/~stephane.magnenat/+archive/precise)
 * [Quantal](https://launchpad.net/~stephane.magnenat/+archive/quantal)
 * [Raring](https://launchpad.net/~stephane.magnenat/+archive/raring)
 
Once you have added the PPA to your system, you can install libpointmatcher using

```
sudo apt-get install libpointmatcher-dev
```

Other
-----

If you have built binaries for other systems, please tell us.

Prerequisites
-------------
If your operating system does not provide it, you must get [Eigen].
It only needs to be downloaded and extracted.
[libnabo] must be downloaded and installed, please follow the [libnabo]'s documentation to do so.
If you want to use YAML-enabled config files, you have to install [yaml-cpp]; on Ubuntu, [ROS repositories](http://www.ros.org/wiki/electric/Installation/Ubuntu) provide `yaml-cpp0.2.6-dev`.

#####Important Note:

A new version (version 0.5.\*) of yaml-cpp has been released with significant changes to the API.  As of now, **Pointmatcher only supports the older api (version 0.3.0)**.  Make sure that your cmake configuration points to the old API or you will not be able to compile.


Compilation & Installation 
==========================
For beginner users who are not familiar with compiling and installing a library in Linux, go [here](doc/Compilation.md) for detailed instructions on how to compile libpointmatcher from the source code.  If you are comfortable with Linux and CMake and have already installed the prerequisites above, the following commands should install libpointmatcher on your system.

```
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make
sudo make install
```

Testing
=======
Libpointmatcher ships with a version of the Google testing framework [GTest](https://code.google.com/p/googletest/).  Unit tests are located in utest/ and are compiled with libpointmatcher.  To run the tests and make sure that your compiled version is working correctly, run the test executable in your build directory:
```
cd build
utest/utest --path ../examples/data/
```

Bug reporting
=============

Please use [github's issue tracker](http://github.com/ethz-asl/libpointmatcher/issues) to report bugs.


Citing
======

If you use libpointmatcher in an academic context, please cite the following publication:

	@article{Pomerleau12comp,
		author = {Pomerleau, Fran{\c c}ois and Colas, Francis and Siegwart, Roland and Magnenat, St{\'e}phane},
		title = {{Comparing ICP Variants on Real-World Data Sets}},
		journal = {Autonomous Robots},
		year = {2013},
		volume = {34},
		number = {3},
		pages = {133--148},
		month = feb
	}

and/or

	@INPROCEEDINGS{pomerleau11tracking,
		author = {Fran{\c c}ois Pomerleau and St{\'e}phane Magnenat and Francis Colas and Ming Liu and Roland Siegwart},
		title = {Tracking a Depth Camera: Parameter Exploration for Fast ICP},
		booktitle = {Proc. of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
		publisher = {IEEE Press},
		pages = {3824--3829},
		year = {2011}
	}


Projects and Partners
=====================

If you are using libpointmatcher in your project and you would like to have it listed here, please contact [François Pomerleau](mailto:f.pomerleau@gmail.com) or [Stéphane Magnenat](http://stephane.magnenat.net).

 * European Project [NIFTi](http://www.nifti.eu/) (FP7 ICT-247870): Search and rescue project in dynamic environments. Results: [video of multi-floor reconstruction](http://www.youtube.com/watch?v=lP5Mj-TGaiw) and [video of railyard reconstruction](http://www.youtube.com/watch?v=ygIvzWVfPYk). All results with real-time computation.
 * NASA Ames [Stereo Pipeline](http://ti.arc.nasa.gov/tech/asr/intelligent-robotics/ngt/stereo/): Planetary reconstruction from satellite observations. Results: used for Mars, Moon and Earth point clouds.
 * Armasuisse S+T UGV research program [ARTOR](http://www.artor.ethz.ch/): Development of techniques for reliable autonomous navigation of a wheeled robot in rough, outdoor terrain. Results: [video of urban and dynamic 3D reconstruction](http://www.youtube.com/watch?v=UCCAUf64tD0) and [video of open space 3D reconstruction](http://www.youtube.com/watch?v=M5Y99o7um88) with real-time computation.
 * Swiss National Science Foundation - [Limnobotics](http://www.limnobotics.ch/): Robotic solution for toxic algae monitoring in lacs. Result: [video of 3D shore reconstruction](http://www.youtube.com/watch?v=g8l-Xq4qYeE) with real-time computation.

For a larger list of work realized with libpointmatcher, please see the page [Applications And Publications](/doc/ApplicationsAndPub.md).


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
[yaml-cpp]: http://code.google.com/p/yaml-cpp/
[Doxygen]: http://www.stack.nl/~dimitri/doxygen/
