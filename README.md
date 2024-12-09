<div align="center">

![banner](doc/images/banner_light.jpeg)

[//]: # ( ==== Description ====================================================================== )
**_libpointmatcher_ is a modular library implementing the Iterative Closest Point (ICP) algorithm<br>
for aligning point clouds. It has applications in robotics and computer vision.**
<br>
The library is written in C++ for efficiency with [bindings in Python](doc/index.md#python-).
<br>
<br>

[//]: # (====Badges===============================================================================)

<img alt="GitHub Repo stars" src="https://img.shields.io/github/stars/norlab-ulaval/libpointmatcher">
<img alt="GitHub forks" src="https://img.shields.io/github/forks/norlab-ulaval/libpointmatcher">
<img alt="GitHub License" src="https://img.shields.io/github/license/norlab-ulaval/libpointmatcher">
<img alt="GitHub release (with filter)" src="https://img.shields.io/github/v/release/norlab-ulaval/libpointmatcher">
<a href="http://132.203.26.125:8111"><img src="https://img.shields.io/static/v1?label=JetBrains TeamCity&message=CI/CD&color=green?style=plastic&logo=teamcity" /></a>
<a href="https://hub.docker.com/repository/docker/norlabulaval/libpointmatcher/"> <img alt="Docker Image Version (latest semver)" src="https://img.shields.io/docker/v/norlabulaval/libpointmatcher?logo=docker&label=libpointmatcher"> </a>

<br>
<br>

[//]: # (====Awesome badges=======================================================================)

[![Mentioned in Awesome LIDAR](https://awesome.re/mentioned-badge.svg)](https://github.com/szenergy/awesome-lidar#basic-matching-algorithms)
&nbsp; &nbsp; &nbsp;
[![Mentioned in Awesome Robotics Libraries](https://awesome.re/mentioned-badge.svg)](http://jslee02.github.io/awesome-robotics-libraries/#3d-mapping)
&nbsp; &nbsp; &nbsp;
[![Mentioned in Awesome Robotics](https://awesome.re/mentioned-badge.svg)](https://github.com/ahundt/awesome-robotics#point-clouds)
<br>
<sup>
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;
<a href="https://github.com/szenergy/awesome-lidar#basic-matching-algorithms">LIDAR</a>
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;
<a href="https://github.com/ahundt/awesome-robotics#point-clouds">Robotics</a>
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;
<a href="http://jslee02.github.io/awesome-robotics-libraries/#3d-mapping">Robotics Libraries</a>
</sup>
<br>
<br>
<hr style="color:lightgray;background-color:lightgray">
</div>


[//]: # (====Supported OS and aarch===============================================================)

### Supported OS And Architecture
_libpointmatcher_ is tested on our build system under the following architecture and OS:
- Ubuntu bionic (18.04), focal (20.04) and jammy (22.04)
- x86 and arm64/v8

Note:
- _libpointmatcher_ reportedly works on MacOs OsX (latest) and Windows (latest)


---

[//]: # (====Release note=========================================================================)

### ★ Version `>= 1.4.0` Release Note
This release of _libpointmatcher_ introduces the integration of [norlab-build-system (NBS)](https://github.com/norlab-ulaval/norlab-build-system) as a _git submodule_ for codebase development and testing.

Execute the following to clone the repository with its submodule:
```shell
git clone --recurse-submodules https://github.com/norlab-ulaval/libpointmatcher.git
```
If _libpointmatcher_ was previously cloned, execute the following to fetch its new submodule
```shell
git submodule update --remote --recursive --init
```

### ★ Contributing Instructions
See [contributing_instructions.md](doc/contributing/contributing_instructions.md)
for instructions related to bug reporting, code contribution and for setting up the `libpointmatcher-build-system`
on your workstation to speed up your local development workflow.


[//]: # (====Body=================================================================================)
# Documentation and Tutorials

**Quick link for the tutorial pages: [Tutorials](http://libpointmatcher.readthedocs.org/).

Those tutorials are written using Markdown syntax and stored in the project's `/doc` folder.  Their scope ranges from introductory material on performing point cloud registration to instructions for the more experienced developer on how to extend the library's codebase.

Libpointmatcher's source code is fully documented based on doxygen to provide an easy API to developers. An example of this API can be found [here](https://norlab.ulaval.ca/libpointmatcher-doc/), but it is suggested to use the one build for your version in `doc/html`.

libpointmatcher was orginaly developed by [François Pomerleau](mailto:f.pomerleau@gmail.com) and [Stéphane Magnenat](http://stephane.magnenat.net) as part of our work at [ASL-ETH](http://www.asl.ethz.ch).
It is now maintained by the Northern Robotics Laboratory ([Norlab](https://norlab.ulaval.ca/)), led by François Pomerleau.

You can read the latest changes in the [release notes](doc/ReleaseNotes.md).




# Quick Start
Although we suggest to use the [tutorials](doc/index.md), here is a quick version of it:

The library has a light dependency list:

 * [Eigen] version 3, a modern C++ matrix and linear-algebra library,
 * [boost] version 1.48 and up, portable C++ source libraries,
 * [libnabo] version 1.0.7, a fast K Nearest Neighbour library for low-dimensional spaces,

and was compiled on:
  * Ubuntu ([see how](/doc/CompilationUbuntu.md))
  * Mac OS X ([see how](/doc/CompilationMac.md))
  * Windows ([see how](/doc/CompilationWindows.md) - partially supported)

### Docker images

Run the following commands to pull and run libpointmatcher in a docker container

```shell
docker pull norlabulaval/libpointmatcher:latest-ubuntu-focal

docker run -it --rm norlabulaval/libpointmatcher:latest-ubuntu-focal
```

See
available [libpointmatcher image tags](https://hub.docker.com/repository/docker/norlabulaval/libpointmatcher/)
on dockerhub.

To install docker related dependencies on ubuntu, execute the following
```shell
cd ./build_system/lpm_utility_script

# Execute docker tools install script i.e. docker daemon, docker compose, docker buildx
bash lpm_install_docker_tools.bash
```


### Compilation & Installation

For beginner users unfamiliar with compiling and installing a library in Linux, go [here](doc/CompilationUbuntu.md) for detailed instructions on compiling libpointmatcher from the source code.

For conveniences, you can use the provided installer script for ubuntu
```shell
bash libpointmatcher_dependencies_installer.bash

# Use the --help flag to see the list of optional flag
bash libpointmatcher_installer.bash [<optional flag>]
```

If you are comfortable with Linux and CMake and have already installed the prerequisites above, the following commands should install libpointmatcher on your system.

```bash
mkdir build && cd build
cmake ..
make
sudo make install
```


### Testing

Libpointmatcher ships with a version of the Google testing framework [GTest](https://github.com/google/googletest).  Unit tests are located in the `utest/` directory and are compiled with libpointmatcher (CMake variable `BUILD_TESTS` must be set to `TRUE` before compiling).  To run the tests and make sure that your compiled version is working correctly, run the test executable in your build directory:

```bash
cd build
utest/utest --path ../examples/data/
```

### Linking to external projects.

We mainly develop for __cmake projects__ and we provide example files under [`examples/demo_cmake/`](https://github.com/norlab-ulaval/libpointmatcher/tree/master/examples/demo_cmake) to help you in your own project. We also provide a __QT Creator__ example in [`examples/demo_QT/`](https://github.com/norlab-ulaval/libpointmatcher/tree/master/examples/demo_Qt), which manually lists all the dependencies in the file [`demo.pro`](https://github.com/norlab-ulaval/libpointmatcher/blob/master/examples/demo_Qt/demo.pro). You would need to ajust those paths to point at the appropriate locations on your system. For a more detailed procedure, check the [Linking Projects to libpointmatcher](doc/LinkingProjects.md) section.

## File formats

The library support different file formats for importing or exporting data:

* csv (Comma Separated Values)
* vtk (Visualization Toolkit Files)
* ply (Polygon File Format)
* pcd (Point Cloud Library Format)

Those functionnalities are available without increasing the list of dependencies at the expense of
limited functionality support. For more details, see the
tutorial [Importing and Exporting Point Clouds](doc/ImportExport.md). Example executables using
those file formats from the command line can be found in the `/examples` directory and are
described [here](doc/ICPIntro.md) in more detail.


---

# Citing

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

# Extra Reading

If you are interested in learning more about different registration algorithms, we recently put together a literature review surveying multiple solutions. The review is organized in the same way as the library and many examples are provided based on real deployments.

F. Pomerleau, F. Colas and R. Siegwart (2015), "_A Review of Point Cloud Registration Algorithms for Mobile Robotics_", __Foundations and Trends® in Robotics__: Vol. 4: No. 1, pp 1-104.  https://doi.org/10.1561/2300000035

If you don't have access to the journal, you can download it from [here](https://www.researchgate.net/publication/277558596_A_Review_of_Point_Cloud_Registration_Algorithms_for_Mobile_Robotics).

# More Point Clouds

We also produced those freely available data sets to test different registration solutions:

[_Challenging data sets for point cloud registration algorithms_](http://projects.asl.ethz.ch/datasets/doku.php?id=laserregistration:laserregistration)

![alt tag](http://projects.asl.ethz.ch/datasets/lib/exe/fetch.php?cache=&media=laserregistration:asldataset_weblarge.jpg)

You can download the files in CSV or VTK formats, which are directly supported by the library I/O module.


# Projects and Partners

If you are using libpointmatcher in your project and you would like to have it listed here, please contact [François Pomerleau](mailto:f.pomerleau@gmail.com).

 * European Project [NIFTi](http://www.nifti.eu/) (FP7 ICT-247870): Search and rescue project in dynamic environments. Results: [video of multi-floor reconstruction](http://www.youtube.com/watch?v=lP5Mj-TGaiw) and [video of railyard reconstruction](http://www.youtube.com/watch?v=ygIvzWVfPYk). All results with real-time computation.
 * NASA Ames [Stereo Pipeline](https://ti.arc.nasa.gov/tech/asr/groups/intelligent-robotics/ngt/stereo/): Planetary reconstruction from satellite observations. Results: used for Mars, Moon, and Earth point clouds.
 * Armasuisse S+T UGV research program [ARTOR](http://www.artor.ethz.ch/): Development of techniques for reliable autonomous navigation of a wheeled robot in rough, outdoor terrain. Results: [video of urban and dynamic 3D reconstruction](http://www.youtube.com/watch?v=UCCAUf64tD0) and [video of open space 3D reconstruction](http://www.youtube.com/watch?v=M5Y99o7um88) with real-time computation.
 * Swiss National Science Foundation - [Limnobotics](http://www.limnobotics.ch/): Robotic solution for toxic algae monitoring in lacs. Result: [video of 3D shore reconstruction](http://www.youtube.com/watch?v=g8l-Xq4qYeE) with real-time computation.
 * [CGAL](https://www.cgal.org/) includes our library for their registration pipeline.
 * [Norlab](https://norlab.ulaval.ca/) is maintaining and using the library for its research on autonomous navigation in harsh environments.
 * [ANYbotics AG](https://www.anybotics.com) is investigating autonomous navigation algorithms using this library.
 * [Point Laz Mining LiDAR Expert](https://www.pointlaz.com/) is scanning mine shafts to ensure infrastructure safety.
 * [Point Laz Mining LiDAR Expert](https://www.pointlaz.com/) is scanning mine shafts to ensure infrastructure safety.
 * [DREAM lab](https://dream.georgiatech-metz.fr/research/woodseer/) use libpointmatcher to reconstruct wood logs in 3D.
For a larger list of work realized with libpointmatcher, please see the page [Applications And Publications](/doc/ApplicationsAndPub.md).


# License

libpointmatcher is released under a permissive BSD license. Enjoy!

[Ubuntu]: http://www.ubuntu.com
[CMake]: http://www.cmake.org
[CMake documentation]: https://cmake.org/cmake/help/v3.10/
[git]: http://git-scm.com
[Eigen]: http://eigen.tuxfamily.org
[libnabo]: https://github.com/norlab-ulaval/libnabo
[ROS]: http://www.ros.org/
[Paraview]: http://www.paraview.org/
[yaml-cpp]: https://github.com/jbeder/yaml-cpp
[Doxygen]: https://www.doxygen.nl/index.html
[boost]: http://www.boost.org/


---


![alt tag](doc/images/banner_dark.jpeg)
