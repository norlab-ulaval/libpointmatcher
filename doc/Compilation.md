# Compiling and Installing Libpointmatcher on your Computer

## Foreword
*The following instructions are aimed at users of Ubuntu Linux.  The steps from this tutorial were performed on **Ubuntu 13.10** (Saucy Salamander).  These instructions should be identical previous versions of Ubuntu.  Other Linux variants should follow a very similar process as with Mac OSX and other \*nix operating systems.*

## Installing Libpointmatcher from Pre-built Binaries (Ubuntu)
A pre-built version of the library is available on the [following](https://launchpad.net/~stephane.magnenat) Personal Package Archive (PPA). Instructions on how to add a PPA to Ubuntu can be found [here](https://launchpad.net/+help-soyuz/ppa-sources-list.html).  Once the PPA has been added to your system, simply run:

```
sudo apt-get install libpointmatcher-dev
```
to install libpointmatcher to your system.

## Some Required Basic Tools

### a. Installing Boost
[Boost](www.boost.org) is a widely-used C++ library and is included in most Linux distributions.  You can quickly check if Boost is installed on your system by running

```
ldconfig -p | grep libboost
```

If you see a list of results then you can skip to the next section.  If not, you most likely have to install Boost.

Instructions for downloading and installing boost to Unix systems can be found [here](http://www.boost.org/doc/libs/1_55_0/more/getting_started/unix-variants.html).  Boost can also be installed as a package by running

```
sudo apt-get install libboost-all-dev
```

### b. Installing Git
[Git](http://git-scm.com/) is a version control system similar to SVN designed for collaboration on large code projects.  Because Libpointmatcher is hosted on Github, you should the git application to keep track of code revisions, and bug fixes pushed to the public repository.

Git should already be installed on your system but you can check if it is by running

```
git --version 
```
If Git is installed, you should see a message of the form
```
git version 1.8.3.2
```
If not refer to the Git homepage for installation instructions or install via the package manager by running
```
sudo apt-get install git-core
```
### c. Installing CMake
[CMake](http://www.cmake.org/) is a cross-platform build system and is used for building the libpointmatcher library.  Refer to the homepage for installation instructions, or you can once again use the package manager
```
sudo apt-get install cmake cmake-gui
```
*NOTE:* CMake has a GUI called cmake-gui which can be useful for configuring builds.  We recommend you install this as well as it will be referred to in this tutorial.

## 1. Installing Eigen
The Eigen linear algebra is required before installing libpointmatcher and can be found [here](http://eigen.tuxfamily.org/).  Either download and compile Eigen following instructions from the package website or simply install package via apt by running:
```
sudo apt-get libeigen3-dev
```

## 2. Installing Libnabo
Libnabo is a library for performing fast nearest-neighbor searches in low-dimensional spaces.  It can be found [here](https://github.com/ethz-asl/libnabo).  Clone the source repository into a local directory of your choice.
```
mkdir ~/Libraries/
cd ~/Libraries
git clone git://github.com/ethz-asl/libnabo.git
cd libnabo
``` 

Now you can compile Libnabo by entering the following commands
```
SRC_DIR=`pwd`
BUILD_DIR=${SRC_DIR}/build
mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ${SRC_DIR}
make
sudo make install
```
This will compile Libnabo in a `/build` directory and install it on your system.

*Note:* If Eigen or Boost are not in their regular system locations you will have to indicate their location by setting the corresponding CMake flags





