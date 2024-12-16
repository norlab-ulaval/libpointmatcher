# Compiling and installing libpointmatcher
This tutorial will guide you through the different steps to install libpointmatcher and its dependencies.
### Supported platforms
=== "Ubuntu"
    | Name           |     Version  <br> (Tested on our CI/CD server)     |       Version  <br> (Tested on our CI/CD server)        | Version  <br> (Tested on our CI/CD server) |
    |:---------------|:--------------------------------------------------:|:-------------------------------------------------------:|:------------------------------------------:|
    | Ubuntu         |            bionic 18.04.1 LTS (64 bit)             |                focal 20.04 LTS (64 bit)                 |          jammy 22.04 LTS (64 bit)          |
    | Architecture   |                  x86 and arm64/v8                  |                     x86 and arm64/v8                    |              x86 and arm64/v8              |

    __Note:__ we only support 64-bit systems because of some issues with Eigen. Other versions will most probably work but you'll have to try yourself to know for sure.

=== "MacOS"
    | Name         | Version  <br> (Tested in Summer 2024) |
    |--------------|---------------------------------------|
    | MacOS        | 14.1.1                                |
    | Architecture | arm64/v8                              |


=== "Windows"

    | Name            | Version <br> (Tested February 11, 2021) |
    | ------          | -------------  |
    | Windows         | 10 v1909 64bit |

    Windows is currently not officially supported, but we can confirm that you can install `libpointmatcher` in [WSL](https://learn.microsoft.com/en-us/windows/wsl/about).

    If you want to contribute to this documentation, your help is more than welcome.

    Before reporting new building issues, have a look in the current/past list of issues. Add as many details as you can since you will most probably receive answers from developers that cannot reproduce the problem on their side. Here are some of them:

    - Your directory structure need to be well organized as mentioned in [Issue #136](https://github.com/norlab-ulaval/libpointmatcher/issues/136).
    - There might be some problems related to libnabo as mentioned in [Issue #118](https://github.com/norlab-ulaval/libpointmatcher/issues/118).

    Special thanks to the following users in helping us with the Windows support:

    - [kwill](https://github.com/kwill) for keeping the documentation up-to-date and investing the time to get libpointmatcher compiling on Windows.
    - [braddodson](https://github.com/braddodson) for porting a version of libpointmatcher to `C#` with a limited set of features. The code can be found here: https://github.com/braddodson/pointmatcher.net



### Dependencies
Libpointmatcher relies on a number of standard libraries, most of which can be installed with standard package managers. The following table lists the minimum requirements necessary to instal libpointmatcher.

| Library name | g++   | cmake  | boost  |  eigen  | yaml-cpp | libnabo | doxygen (opt.) |
|:-------------|-------|:------:|:------:|:-------:|:--------:|:-------:|:--------------:|
| **Version**  | 9.4.0 | 3.15 | 1.65.1 | 3.3.4-4 |   0.5    |  1.1.1  |   1.8.13-10    |

## Detailed Installation Instructions
The rest of this tutorial will guide you through the different requirements step by step.

#### Getting your platform ready
Some installation steps are platform-dependent and need to be done beforehand.

=== "Ubuntu"
    You will need to install the GNU Compiler Collection, also known as GCC. The minimum supported version is 9.4.0.
    ```bash
    sudo apt update
    sudo apt install build-essential
    ```

    ??? warning "Ubuntu 18 Bionic"

        While newer versions of Ubuntu come with gcc and g++ on newer versions than 9.4.0 and support all modern C++17 standard library features, it is not the case for Ubuntu Bionic.
        You will therefore need to install it manually.
        Run
        ```shell
        sudo add-apt-repository ppa:ubuntu-toolchain-r/test
        sudo apt-get update &&
        sudo apt-get install gcc-9 g++-9
        sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-9
        ```

=== "MacOS"
    ###### Installing Xcode via the App Store (OS X 10.10.2 and later)

    Mac OS X does not come with a built-in C++ command-line compiler.  You must therefore install XCode by visiting the App Store.

    Once Xcode is installed on your machine, launch it.  Navigate to preferences, to the downloads tab.  In the components list, install the Command Line Tools component.

    You should now have a working version of gcc. You can check by running the following command in your terminal:

    ```bash
    gcc --version | grep gcc
    ```

    A message similar to the following should appear

    ```text
    gcc (Homebrew GCC 13.2.0) 13.2.0
    ```

    ###### Installing Homebrew

    Because Mac OS X does not come with a built-in package manager like in Ubuntu, you need to install one on your own.  A package manager is handy because it allows you to install, uninstall, update and maintain software packages with ease.  There are several possibilities including [Macports](http://www.macports.org/) and [Homebrew](http://brew.sh/).  While both are good options, we have a slight preference for homebrew which is easier to use.

    You do not need a package manager to install libpointmatcher, but it simplifies things.  The following instructions will make use of homebrew and will thus assume that it is installed on your system.

    Installing Homebrew is extremely easy and can be done by entering the following single command in your terminal

    ```bash
    ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
    ```
    Once the scripts finishes installing, you are good to go!
=== "Windows"

    | Name            | Download Link                                           | Version <br> (Tested February 11, 2021) |
    | ------          | ----                                                    | -------------  |
    | Visual Studio   | <https://visualstudio.microsoft.com/>                   | 2019 16.8.5    |
    | MSVC++ Compiler | _(with Visual Studio)_                                  | 14.2           |
    | Git             | <https://git-scm.com/downloads/>                        | 2.30.1         |
    | CMake           | <https://cmake.org/>                                    | 3.19.0         |
    | Boost           | <https://www.boost.org/users/download/>                 | 1.75.0         |
    | Eigen3          | <http://eigen.tuxfamily.org/index.php>                  | 3.3.9          |
    | grep            | <http://gnuwin32.sourceforge.net/packages/grep.htm>     | 2.5.4          |


    - It's recommended to use **Windows PowerShell** with administrator privileges as your CLI
    - All necessary environment variables will be configured so that **CMake automatically finds all libraries** and you don't have to specify the libraries' paths to CMake each time you use it
    - When adding any environment variable, add for All users (system variables)
    - Environment variables are case-insensitive (PATH = Path)
    - Add new environment variables in PATH on top of the list (to avoid conflicts)
    - You must **restart** your CLI and CMake for  new environment variables to take effect
    - In this tutorial, `C:\dev` will be used as root directory for all installations. The folder can be wherever you want, but it is strongly recommended that the path **has no spaces**.

#### Installing Boost

[Boost](https://www.boost.org/) is a widely-used C++ library and is included in most Linux distributions.  You can quickly check if Boost is installed on your system by running
Instructions for downloading and installing boost to Unix systems can be found [here](https://www.boost.org/doc/libs/1_65_1/more/getting_started/unix-variants.html).  Boost can also be installed as a package by running

=== "Ubuntu"
    ```bash
    sudo apt-get install libboost-all-dev
    ```
=== "MacOS"
    ```bash
    brew install boost
    ```
=== "Windows"

    1. Download `boost_<version>.zip`
    2. Extract `boost_<version>` in `C:\dev`
    3. Go to your Boost source directory with your CLI, and do:

       ```powershell
       .\bootstrap.bat
       .\b2.exe
       ```

    4. Set the following three environment variables:
        `BOOST_LIBRARYDIR = C:\dev\boost_<version>\stage\lib`
        `BOOST_INCLUDEDIR = C:\dev\boost_<version>`
        `BOOST_DIR = C:\dev\boost_<version>\stage\lib\cmake\Boost-<version>`
    5. Add `C:\dev\boost_<version>\stage\lib` to `Path` environment variable


#### Installing CMake

[CMake](http://www.cmake.org/) is a cross-platform build system and is used for building the libpointmatcher library.  Refer to the homepage for installation instructions, or you can once again use the package manager
The easist and prefered way to install CMake is with pip:
```bash
pip install cmake
```
Alternatively, you can install CMake using your package manager (apt, brew, etc.).
However, note that some older Ubuntu versions (Ubuntu 18 and lower) don't come with the required version of CMake (3.15+).
In this case, you will need to install CMake from the official website.

=== "Ubuntu"
    ```bash
    sudo apt-get install cmake cmake-gui
    ```
=== "MacOS"
    ```bash
    brew install cmake
    ```
=== "Windows"
    You can download and install cmake from their [website](https://cmake.org/download/).

*NOTE:* CMake has a GUI called cmake-gui which can be useful for configuring builds.  We recommend you install this as well since it will be referred to in this tutorial.

#### Installing Eigen

The Eigen linear algebra library is required before installing libpointmatcher and can be found [here](http://eigen.tuxfamily.org/).  Either download and compile Eigen following instructions from the package website or simply install package via apt by running:

=== "Ubuntu"
    ```bash
    sudo apt-get install libeigen3-dev
    ```
=== "MacOS"
    ```bash
    brew install eigen
    ```
=== "Windows"
    1. Download `eigen-<version>.zip`
    2. Extract `eigen-<version>` in `C:\dev`
    3. Set `EIGEN3_INC_DIR` environment variable to `C:\dev\eigen-<version>` (folder with `signature_of_eigen3_matrix_library` file)

#### Installing yaml-cpp

The yaml-cpp library allows to load libpointmatcher configurations from convenient .yaml files. The library can be installed via apt by running:

=== "Ubuntu"
    ```bash
    sudo apt-get install libyaml-cpp-dev
    ```
=== "MacOS"
    The straightforward way to install yaml-cpp library through brew was, as for the end of 2023, not functional
    ```bash
    brew install yaml-cpp
    ```
    Instead, you can install the library from sources. Follow https://github.com/jbeder/yaml-cpp/tree/master
    ```bash
    mkdir ~/Libraries/
    cd ~/Libraries
    git clone -b yaml-cpp-0.7.0 git@github.com:jbeder/yaml-cpp.git
    cd yaml-cpp
    ```
    Now you can compile and install yaml-cpp by entering the following commands

    ```bash
    SRC_DIR=$PWD
    BUILD_DIR=${SRC_DIR}/build
    mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}
    cmake -DBUILD_TESTING=FALSE ..
    make
    sudo make install
    ```
=== "Windows"
    TODO

#### Installing libnabo

libnabo is a library for performing fast nearest-neighbor searches in low-dimensional spaces.  It can be found [here](https://github.com/norlab-ulaval/libnabo).  Clone the source repository into a local directory of your choice.

=== "Ubuntu"
    ```bash
    mkdir ~/Libraries/
    cd ~/Libraries
    git clone git://github.com/norlab-ulaval/libnabo.git
    cd libnabo
    ```
=== "MacOS"
    ```bash
    mkdir ~/Libraries/
    cd ~/Libraries
    git clone git://github.com/norlab-ulaval/libnabo.git
    cd libnabo
    ```
=== "Windows"
    You will first need to install Grep in order to install libnabo.

    1. Download the last Setup "Complete package" of Grep for Windows
    2. Execute the Setup and extract the `GnuWin32` folder in `C:\dev`
    3. Add the path to the Grep .exe (`C:\dev\GnuWin32\bin`) files to the `Path` environment variable.

    Now, you can proceed with libnabo download.

    1. Go to your desired directory with your CLI (here `C:\dir`)
    2. Do the following commands
       ```powershell
       git clone https://github.com/norlab-ulaval/libnabo
       mkdir .\libnabo\build
       cd .\libnabo\build\
       cmake-gui ..
       ```
       CMake-Gui will open up
    3. Click on the button **Configure** and specify the generator for the project (Visual Studio 16 2019)
    >    An error will be reported, because CMake does not know yet where to find the libraries. The next steps will tell it where to find them.
    4. Set the CMake variable `EIGEN_INCLUDE_DIR` to `C:\dev\eigen-<version>`
    5. Click on the button Configure, Generate and then Open Project
       Visual Studio will open up
    >    Maybe you will have messages about Doxygen, OpenCL, ANN, FLANN and Python missing. They are not necessary to install libnano.

Now you can compile libnabo by entering the following commands

=== "Ubuntu"
    ```bash
    SRC_DIR=$PWD
    BUILD_DIR=${SRC_DIR}/build
    mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ${SRC_DIR}
    make
    ```
=== "MacOS"
    ```bash
    SRC_DIR=$PWD
    BUILD_DIR=${SRC_DIR}/build
    mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ${SRC_DIR}
    make
    ```
=== "Windows"
    Following the previous steps,
    1. Put your "Solution Configuration" in `Release` mode
    2. Build the `ALL_BUILD` project
    3. Rebuild for each configuration mode you will use, i.e. Release/Debug/RelWithDebInfo/MinSizeRel
       (Libnabo build a different .lib for each configuration mode)

To make sure that everything is working properly, you can run the unit tests:

=== "Ubuntu"
    ```bash
    make test
    ```
=== "MacOS"
    ```bash
    make test
    ```
=== "Windows"
    TODO

This will run multiple nearest-neighbor searches performances and may take some minutes.

Then, install the library on your system by running the following command :

=== "Ubuntu"
    ```bash
    sudo make install
    ```
    *Note:* If Eigen or Boost are not in their regular system locations you will have to indicate their location by setting the corresponding CMake flags. Go [here](#possible-caveats) to see how it can be achieved.

=== "MacOS"
    ```bash
    sudo make install
    ```
    *Note:* If Eigen or Boost are not in their regular system locations you will have to indicate their location by setting the corresponding CMake flags. Go [here](#possible-caveats) to see how it can be achieved.

=== "Windows"
    TODO

### Installing libpointmatcher

First, you need to clone the source repository into a local directory. As an example, we reuse the Libraries directory that was created to contain the libnabo sources.

=== "Ubuntu"
    ```bash
    cd ~/Libraries/
    git clone git://github.com/norlab-ulaval/libpointmatcher.git
    cd libpointmatcher
    ```
=== "MacOS"
    ```bash
    cd ~/Libraries/
    git clone git://github.com/norlab-ulaval/libpointmatcher.git
    cd libpointmatcher
    ```
=== "Windows"

But, before compiling libpointmatcher, a `/build` directory must be created. Just like with libnabo, run the following commands :

=== "Ubuntu"
    ```bash
    SRC_DIR=${PWD}
    BUILD_DIR=${SRC_DIR}/build
    mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}
    cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo ${SRC_DIR}
    ```
=== "MacOS"
    ```bash
    SRC_DIR=$PWD
    BUILD_DIR=${SRC_DIR}/build
    mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}
    cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo ${SRC_DIR}
    ```
=== "Windows"
    ```powershell
       git clone https://github.com/norlab-ulaval/libpointmatcher
       mkdir .\libpointmatcher\build
       mkdir .\libpointmatcher\build\install
       cd .\libpointmatcher\build\
       cmake-gui ..
    ```

       CMake-Gui will open up
    1. Click on the button **Configure** and specify the generator for the project (Visual Studio 16 2019)
    2. Set the CMake variable `EIGEN_INCLUDE_DIR` to `C:\dev\eigen-<version>`
    3. Set the CMake variable `CMAKE_INSTALL_PREFIX` to `C:/dev/libpointmatcher/build/install`
    >    **CAUTION !** Use forward slash `/`
Before moving on to the compilation and installation steps, here are some optional features that you can enable.

#### Compiling the documentation (optional)

Libpointmatcher is documented directly in the source-code using [Doxygen](https://www.doxygen.nl/index.html).  If Doxygen is installed on your system, an html version of the documentation will be compiled in `/usr/local/share/doc/libpointmatcher/`.  To install Doxygen on Ubuntu, run:

=== "Ubuntu"
    ```bash
    sudo apt-get install doxygen
    ```
=== "MacOS"
    ```bash
    brew install doxygen
    ```
=== "Windows"
    Doxygen can be downloaded and installed from the official [website](https://www.doxygen.nl/download.html).

You will also need LaTeX for the equations rendering :

=== "Ubuntu"
    ```bash
    sudo apt-get install doxygen-latex
    ```
=== "MacOS"
    ```bash
    brew install --cask mactex
    ```
=== "Windows"
    This hasn't been tested, but it appears that you can use the [MiKTeX](https://miktex.org/) TeX distribution for Windows.

After you have installed Doxygen and LaTeX, you can enable the documentation by setting the CMake variable `GENERATE_API_DOC` to `TRUE`. This can be achieved through CMake-GUI or by the command line in your `build` directory:

```bash
cmake -D GENERATE_API_DOC=TRUE ${SRC_DIR}
```

Compiling libpointmatcher will generate the documentation, which you can simply open the `/usr/local/share/doc/libpointmatcher/api/html/index.html` file to view the API documentation in a browser.

#### Compiling unit tests (optional)

If you want to verify that the version of libpointmatcher you have compiled is stable, you can enable them by setting the CMake variable `BUILD_TESTS` to `TRUE`. It can be done with CMake-GUI or via the command line:

=== "Ubuntu"
    ```bash
    cmake -D BUILD_TESTS=TRUE ${SRC_DIR}
    ```
=== "MacOS"
    ```bash
    cmake -D BUILD_TESTS=TRUE ${SRC_DIR}
    ```
=== "Windows"
    Set the CMake variable `BUILD_TESTS` to `TRUE`


Then, once the compilation process is completed, the unit tests can be run with the following command line:

=== "Ubuntu"
    ```bash
    utest/utest --path ${SRC_DIR}/examples/data/
    ```
=== "MacOS"
    ```bash
    utest/utest --path ${SRC_DIR}/examples/data/
    ```
=== "Windows"
    From build directory
    ```powershell
    utest/utest --path ../examples/data/
    ```

#### Compilation

Now, to compile libpointmatcher into the `/build` directory, run the following command:

=== "Ubuntu"
    ```bash
    make -j N
    ```
    *Note:* It is highly recommended to add the `-j N` optional argument to the `make` command in order to speed up the compilation process. Replace `N` by the number of parallel jobs you want to compile at the same time.

=== "MacOS"
    ```bash
    make -j N
    ```
    *Note:* It is highly recommended to add the `-j N` optional argument to the `make` command in order to speed up the compilation process. Replace `N` by the number of parallel jobs you want to compile at the same time.

=== "Windows"
    Following the previous configuration in CMake GUI:

    1. Click on the button Configure, Generate and then Open Project. Visual Studio will open up
    2. Put your "Solution Configuration" in `Release` mode
    3. Build the `BUILD` project

#### Installation

Finally, to install libpointmatcher on your system, run the following command:

=== "Ubuntu"
    ```bash
    sudo make install
    ```
=== "MacOS"
    ```bash
    sudo make install
    ```
=== "Windows"
    Following the previous configuration in CMake GUI:

    1. Build the `INSTALL` project
    >    We have to install the library and not only build it, because otherwise all CMake files won't be able to be found by programs using libpointmatcher
    2. Set `libpointmatcher_DIR` environment variable to `C:\dev\libpointmatcher\build\install\share\libpointmatcher\cmake`



# Having problems?
If Eigen, libnabo, yaml-cpp, or GTest are not found during the installation, you will have to manually supply their installation locations by setting the CMake flags. You can do so using the cmake-gui tool.
You can then set `EIGEN_INCLUDE_DIR`, `NABO_INCLUDE_DIR`, `NABO_LIBRARY`, `yaml-cpp_INCLUDE_DIRS`, `yaml-cpp_LIBRARIES` to point to your installation directories.
Some dependencies changed, and we don't keep track of all combinations possible. Before reporting a problem, make sure to include the versions you are using. You can run the bash script `./utest/listVersions<your-platform>.sh` and copy-paste its output when [reporting an issue on github](https://github.com/norlab-ulaval/libpointmatcher/issues). You may need to ensure that the file is executable:

=== "Ubuntu"
    ```bash
    chmod +x ./utest/listVersionsUbuntu.sh
    ./utest/listVersionsUbuntu.sh
    ```
=== "MacOS"
    ```bash
    chmod +x ./utest/listVersionsMacOS.sh
    ./utest/listVersionsMacOS.sh
    ```
=== "Windows"
    ```
    TODO
    ```
