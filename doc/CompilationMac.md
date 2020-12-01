| [Tutorials Home](index.md) | [Previous](CompilationUbuntu.md) | [Next](CompilationWindows.md) |
| :--- | :---: | ---: |

# Compiling and installing libpointmatcher on Mac OS X

## In short...

If you are used to development project, here is what you need:

|Name           |Version  <br> (Tested Feb. 16, 2015) |
|---------------|-----------------------|
|MacOS          | 10.10.2               |
|Xcode          | todo                  |      
|gcc            | 4.2.1                 |
|brew           | 0.9.5                 |
|git            | 1.9.3                 |
|cmake          | 3.0.2                 |
|doxygen (opt.) | 1.8.9.1               |
| | |
|_Dependency:_ ||
|boost          | 1.57.0                |
|eigen          | 3.2.4                 |
|libnabo        | [from source](https://github.com/ethz-asl/libnabo) |


__Note:__ Other versions will most probably work but you'll have to try yourself to know for sure.

The rest of this tutorial will guide you through the different requirements step by step.

## Detailed Installation Instructions

### Some Basic Requirements

#### a. Installing Xcode via the App Store (OS X 10.10.2  and later)

Mac OS X does not come with a built-in C++ command-line compiler.  You must therefore install XCode by visiting the App Store.

Once Xcode is installed on your machine, launch it.  Navigate to preferences, to the downloads tab.  In the components list, install the Command Line Tools component.

You should now have a working version of gcc.  You can check by running the following command in your terminal:

```bash
gcc --version
```

A message similar to the following should appear

```text
Configured with: --prefix=/Applications/Xcode.app/Contents/Developer/usr --with-gxx-include-dir=/usr/include/c++/4.2.1
Apple LLVM version 6.0 (clang-600.0.56) (based on LLVM 3.5svn)
Target: x86_64-apple-darwin14.1.0
Thread model: posix
```

#### b. Installing Homebrew

Because Mac OS X does not come with a built-in package manager like in Ubuntu, you need to install one on your own.  A package manager is handy because it allows you to install, uninstall, update and maintain software packages with ease.  There are several possibilities including [Macports](http://www.macports.org/) and [Homebrew](http://brew.sh/).  While both are good options, we have a slight preference for homebrew which is easier to use.

You do not need a package manager to install libpointmatcher, but it simplifies things.  The following instructions will make use of homebrew and will thus assume that it is installed on your system.

Installing Homebrew is extremely easy and can be done by entering the following single command in your terminal

```bash
ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```
Once the scripts finishes installing, you are good to go!

#### c. Installing Boost

[Boost](https://www.boost.org/) is a widely-used C++ library and is included in most Linux distributions.  Mac OS X does not ship with Boost so if you have never used it before, you probably need to install it.  You can install the latest version of boost with the following homebrew command:

```bash
brew install boost
```

#### d. Installing Git

[Git](http://git-scm.com/) is a version control system similar to SVN designed for collaboration on large code projects.  Because libpointmatcher is hosted on Github, you should the git application to keep track of code revisions, and bug fixes pushed to the public repository.

After installing the Xcode Command Line Tools, Git should already be installed on your system but you can check that it is there by running

```bash
git --version 
```

If Git is installed, you should see a message of the form

```text
git version 1.9.3
```

If not refer to the Git homepage for installation instructions or install via homebrew by running

```bash
brew install git
```

#### e. Installing CMake

[CMake](http://www.cmake.org/) is a cross-platform build system and is used for building the libpointmatcher library.  Refer to the homepage for installation instructions, or you can once again use homebrew

```bash
brew install cmake
```

### 1. Installing Eigen

The Eigen linear algebra is required before installing libpointmatcher and can be found [here](http://eigen.tuxfamily.org/).  Either download and compile Eigen following instructions from the package website or simply install the package via homebrew by running:

```bash
brew install eigen
```

### 2. Installing libnabo

libnabo is a library for performing fast nearest-neighbor searches in low-dimensional spaces.  It can be found [here](https://github.com/ethz-asl/libnabo).  Clone the source repository into a local directory of your choice.

```bash
mkdir ~/Libraries/
cd ~/Libraries
git clone git://github.com/ethz-asl/libnabo.git
cd libnabo
```

Now you can compile libnabo by entering the following commands

```bash
SRC_DIR=$PWD
BUILD_DIR=${SRC_DIR}/build
mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ${SRC_DIR}
make
```

This will compile libnabo into a `/build` directory.

To make sure that everything is working properly, you can run the unit tests:

```bash
make test
```

This will run multiple nearest-neighbor searches performances and may take some minutes.

Then, install the library on your system by running the following command :

```bash
sudo make install
```

*Note:* If Eigen or Boost are not in their regular system locations you will have to indicate their location by setting the corresponding CMake flags. You can use the following command to default flags: Go [here](#possible-caveats) to see how it can be achieve.

### 3. Installing libpointmatcher

First, you need to clone the source repository into a local directory.  As an example we reuse the Libraries directory that was created to contain the libnabo sources.

```bash
cd ~/Libraries/
git clone git://github.com/ethz-asl/libpointmatcher.git
cd libpointmatcher
```

But, before compiling libpointmatcher, a `/build` directory must be created. Just like with libnabo, run the following commands :

```bash
SRC_DIR=$PWD
BUILD_DIR=${SRC_DIR}/build
mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}
cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo ${SRC_DIR}
```

Before moving on to the compilation and installation steps, here are some optional features that can be enable.

#### Compiling the documentation (optional)

Libpointmatcher is documented directly in the source-code using [Doxygen](https://www.doxygen.nl/index.html). If Doxygen is installed on your system, an html version of the documentation will be compiled in `/usr/local/share/doc/libpointmatcher/`. To install Doxygen on Mac OS X, run:

```bash
brew install doxygen
```

After you have installed Doxygen, you can enable the documentation by setting the CMake variable `GENERATE_API_DOC` to `TRUE`. This can be achieved by the command line:

```bash
cmake -D GENERATE_API_DOC=TRUE ${SRC_DIR}
```

Compiling libpointmatcher will generate the documentation, which you can simply open the `/usr/local/share/doc/libpointmatcher/api/html/index.html` file to view the API documentation in a browser.

#### Compiling unit tests (optionnal)

If you want to verify that the version of libpointmatcher you have compiled is stable, you can enable them by setting the CMake variable `BUILD_TESTS` to `TRUE`. It can be done via the command line:

```bash
cmake -D BUILD_TESTS=TRUE ${SRC_DIR}
```

Then, once the compilation process is completed, the unit tests can be run with the following command line:

```bash
utest/utest --path ${SRC_DIR}/examples/data/
```

#### Compilation

Now, to compile libpointmatcher into the `/build` directory, run the following commands.

```bash
make -j N
```
*Note:* It is highly recommended to add the `-j N` optionnal argument to the `make` command in order to speed up the compilation process. Replace `N` by the number of parallel jobs you want to compile at the same time.

#### Installation

Finally, to install libpointmatcher on your system, run the following command:

```bash
sudo make install
```

### 4. Possible Caveats

If Eigen, libnabo, yaml-cpp, or GTest are not found during the installation, you will have to manually supply their installation locations by setting the CMake flags.  You can do so using the ccmake tool.

```bash
cd build
ccmake .
```

You can then set `EIGEN_INCLUDE_DIR`, `NABO_INCLUDE_DIR`, `NABO_LIBRARY`, `yaml-cpp_INCLUDE_DIRS`, `yaml-cpp_LIBRARIES` to point to your installation directories.  Then, configure the make files by pressing `c`, then `g` to generate and rerun the following commands inside `/build`:

```bash
make
sudo make install
```

# Having problems?

Some dependencies changed and we don't keep track of all combinations possible. Before reporting a problem, make sure to include the versions you are using. 

Here are useful commands for the different version:

MacOS version:

```bash
sw_vers -productVersion 
```

Compiler version:

```bash
gcc --version
```

Homebrew:

```bash
brew --version
```

Boost:

```bash
brew info boost
```

Git:

```bash
git --version
```

CMake:

```bash
cmake --version
```

Eigen:

```bash
brew info eigen
```

Doxygen:

```bash
brew info doxygen
```