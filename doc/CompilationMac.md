| [Tutorials Home](Tutorials.md)    | | [Next](Datafilters.md) |
| ------------- |:-------------:| -----:|

# Compiling and Installing libpointmatcher on your Computer (Mac OS X Instructions)
######Latest update April 18, 2014 by Samuel Charreyron

## Foreword
*The following instructions are aimed at users of Mac OS X.  The steps from this tutorial were performed on __OS X 10.9.2__ (Mavericks).  These instructions should be identical on previous versions of Mac OS X.*

### Some Basic Requirements 
#### a. Installing Xcode via the App Store (OS X 10.6.6  and later)
Mac OS X does not come with a built-in C++ command-line compiler.  You must therefore install XCode by visiting the App Store.

Once Xcode is installed on your machine, launch it.  Navigate to preferences, to the downloads tab.  In the components list, install the Command Line Tools component.

You should now have a working version of gcc.  You can check by running the following command in your terminal:

	gcc --version

A message similar to the following should appear

	Configured with: --prefix=/Applications/Xcode.app/Contents/Developer/usr --with-gxx-include-dir=/usr/include/c++/4.2.1
	Apple LLVM version 5.1 (clang-503.0.40) (based on LLVM 3.4svn)
	Target: x86_64-apple-darwin13.1.0
	Thread model: posix

#### b. Installing Homebrew
Because Mac OS X does not come with a built-in package manager like in Ubuntu, you need to install one on your own.  A package manager is handy because it allows you to install, uninstall, update and maintain software packages with ease.  There are several possibilities including [Macports](http://www.macports.org/) and [Homebrew](http://brew.sh/).  While both are good options, we have a slight preference for homebrew which is easier to use.

You do not need a package manager to install libpointmatcher, but it simplifies things.  The following instructions will make use of homebrew and will thus assume that it is installed on your system.

Installing Homebrew is extremely easy and can be done by entering the following single command in your terminal

```
ruby -e "$(curl -fsSL https://raw.github.com/Homebrew/homebrew/go/install)"
```
Once the scripts finishes installing, you are good to go!

#### c. Installing Boost
[Boost](www.boost.org) is a widely-used C++ library and is included in most Linux distributions.  Mac OS X does not ship with Boost so if you have never used it before, you probably need to install it.  You can install the latest version of boost with the following homebrew command:

	brew install boost

#### d. Installing Git
[Git](http://git-scm.com/) is a version control system similar to SVN designed for collaboration on large code projects.  Because libpointmatcher is hosted on Github, you should the git application to keep track of code revisions, and bug fixes pushed to the public repository.

After installing the Xcode Command Line Tools, Git should already be installed on your system but you can check that it is there by running

```
git --version 
```
If Git is installed, you should see a message of the form
```
git version 1.8.3.2
```
If not refer to the Git homepage for installation instructions or install via homebrew by running
```
brew install git
```
#### e. Installing CMake
[CMake](http://www.cmake.org/) is a cross-platform build system and is used for building the libpointmatcher library.  Refer to the homepage for installation instructions, or you can once again use homebrew

	brew install cmake

### 1. Installing Eigen
The Eigen linear algebra is required before installing libpointmatcher and can be found [here](http://eigen.tuxfamily.org/).  Either download and compile Eigen following instructions from the package website or simply install the package via homebrew by running:

	brew install eigen

### 2. Installing libnabo
libnabo is a library for performing fast nearest-neighbor searches in low-dimensional spaces.  It can be found [here](https://github.com/ethz-asl/libnabo).  Clone the source repository into a local directory of your choice.
```
mkdir ~/Libraries/
cd ~/Libraries
git clone git://github.com/ethz-asl/libnabo.git
cd libnabo
``` 

Now you can compile libnabo by entering the following commands
```
SRC_DIR=`pwd`
BUILD_DIR=${SRC_DIR}/build
mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ${SRC_DIR}
make
sudo make install
```
This will compile libnabo in a `/build` directory and install it on your system.

*Note:* If Eigen or Boost are not in their regular system locations you will have to indicate their location by setting the corresponding CMake flags.

### 3. Installing yaml-cpp 0.3.0 (Optional)
Configuration files can be managed using YAML in libpointmatcher.  This allows users to edit configuration files in a readable format.  To support this, you need to install [yaml-cpp](http://code.google.com/p/yaml-cpp/).  

Homebrew installs the latest version of yaml-cpp, which is not compatible with Pointmatcher.  **It is important that you install the older version (0.3.0) of lib-yaml.**  You can install the older version of yaml-cpp in homebrew by doing the following:

	brew versions yaml-cpp

This should produce a list like the following 

	0.5.1    git checkout e2d162d Library/Formula/yaml-cpp.rb
	0.5.0    git checkout 5fc0071 Library/Formula/yaml-cpp.rb
	0.3.0    git checkout 6e32f8c Library/Formula/yaml-cpp.rb
	0.2.5    git checkout ebe6663 Library/Formula/yaml-cpp.rb

Copy the command corresponding to version 0.3.0 and enter it in the terminal

	git checkout 6e32f8c Library/Formula/yaml-cpp.rb

Finally, install yaml-cpp

	brew install yaml-cpp

### 4. Compiling the Documentation
Libpointmatcher is documented directly in the source-code using [Doxygen](http://www.stack.nl/~dimitri/doxygen/).  If Doxygen is installed on your system, an html version of the documentation will be compiled in `/usr/local/share/doc/libpointmatcher/`.  To install Doxygen in Ubuntu, run:

	brew install doxygen

Once you have compiled libpointmatcher in step 6, you can simply open `/usr/local/share/doc/libpointmatcher/api/html/index.html` in a browser to view the API documentation.

### 5. Installing libpointmatcher
Clone the source repository into a local directory.  As an example we reuse the Libraries directory that was created to contain the libnabo sources.
```
cd ~/Libraries/
git clone git://github.com/ethz-asl/libpointmatcher.git
cd libpointmatcher
```
Now, libpointmatcher is compiled into a `/build` directory.
```
SRC_DIR=`pwd`
BUILD_DIR=${SRC_DIR}/build
mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ${SRC_DIR}
make
```

You can optionally verify that the version of libpointmatcher you have compiled is stable by running the unit tests.
```
utest/utest --path ../examples/data/
```

Finally, to install libpointmatcher to your system run the following:
```
sudo make install
```

#### Possible Caveats
If Eigen, libnabo, yaml-cpp, or GTest are not found during the installation, you will have to manually supply their installation locations by setting the CMake flags.  You can do so using the ccmake tool.
```
cd build
ccmake ..
```

You can then set `EIGEN_INCLUDE_DIR`, `NABO_INCLUDE_DIR`, `NABO_LIBRARY`, `yaml-cpp_INCLUDE_DIRS`, `yaml-cpp_LIBRARIES` to point to your installation directories as shown in the screenshot above.  Then, generate the make files by clicking generate and rerun the following inside `/build`:
```
make
sudo make install
```