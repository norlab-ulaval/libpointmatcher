| [Tutorials Home](index.md) | [Previous](CompilationMac.md) | [Next](Introduction.md) |
| :--- | :---: | ---: |

# Installation of libpointmatcher on Windows using MSVC

This tutorial will guide you through the different steps to install libpointmatcher and its dependencies.

-----  

This tutorial is divided up like this:

- Introduction
  - [Resources](#resources) (Library versions & Download links)
  - [Notes](#notes)
- Installation
  - [Prerequisites](#prerequisites)
  - [Boost](#install-boost)
  - [Eigen3](#install-eigen3)
  - [Grep](#install-grep)
  - [libnabo](#install-libnabo)
  - [libpointmatcher](#install-libpointmatcher)

## Introduction  

### Resources  

| Name            | Download Link                                           | Version <br> (Tested February 11, 2021) |
| ------          | ----                                                    | -------------  |
| Windows         |                                                         | 10 v1909 64bit |
| Visual Studio   | <https://visualstudio.microsoft.com/>                   | 2019 16.8.5    |
| MSVC++ Compiler | _(with Visual Studio)_                                  | 14.2           |
| Git             | <https://git-scm.com/downloads/>                        | 2.30.1         |
| CMake           | <https://cmake.org/>                                    | 3.19.0         |
| Boost           | <https://www.boost.org/users/download/>                 | 1.75.0         |
| Eigen3          | <http://eigen.tuxfamily.org/index.php>                  | 3.3.9          |
| grep            | <http://gnuwin32.sourceforge.net/packages/grep.htm>     | 2.5.4          |
| libnabo         | <https://github.com/ethz-asl/libnabo>                   | Commit 16250bf |
| libpointmatcher | <https://github.com/ethz-asl/libpointmatcher>           | Commit e9a832d |

### Notes  

- It's recommended to use **Windows PowerShell** with administrator privileges as your CLI
- All necessary environment variables will be configured so that **CMake automatically finds all libraries** and you don't have to specify the libraries' paths to CMake each time you use it  
- When adding any environment variable, add for All users (system variables)
- Environment variables are case-insensitive (PATH = Path)
- Add new environment variables in PATH on top of the list (to avoid conflicts)
- You must **restart** your CLI and CMake for  new environment variables to take effect
- In this tutorial, `C:\dev` will be used as root directory for all installations. The folder can be wherever you want, but it is strongly recommended that the path **has no spaces**.

## Installation  

### Prerequisites  

- C++ compiler (get MSVC compiler by installing Visual Studio)
- Git
- CMake

### Install Boost  

1. Download `boost_<version>.zip`
1. Extract `boost_<version>` in `C:\dev`
1. Go to your Boost source directory with your CLI, and do:

    ```bash
    .\bootstrap.bat
    .\b2.exe
    ```

1. Set the following three environment variables:  
     `BOOST_LIBRARYDIR = C:\dev\boost_<version>\stage\lib`  
     `BOOST_INCLUDEDIR = C:\dev\boost_<version>`  
     `BOOST_DIR = C:\dev\boost_<version>\stage\lib\cmake\Boost-<version>`  
1. Add `C:\dev\boost_<version>\stage\lib` to `Path` environment variable

### Install Eigen3  

> Eigen is header only (see Eigen's ["Getting stated" page](http://eigen.tuxfamily.org/dox/GettingStarted.html)) and don't need to be build.

1. Download `eigen-<version>.zip`
1. Extract `eigen-<version>` in `C:\dev`
1. Set `EIGEN3_INC_DIR` environment variable to `C:\dev\eigen-<version>` (folder with `signature_of_eigen3_matrix_library` file)

### Install Grep

1. Download the last Setup "Complete package" of Grep for Windows
1. Execute the Setup and extract the `GnuWin32` folder in `C:\dev`
1. Add the path to the Grep .exe (`C:\dev\GnuWin32\bin`) files to the `Path` environment variable.

### Install libnabo  

> You need to install [Eigen3](#install-eigen3) and [Grep](#install-grep) before installing libnabo !

1. Go to your desired directory with your CLI (here `C:\dir`)
1. Do the following commands

    ```bash
    git clone https://github.com/ethz-asl/libnabo
    mkdir .\libnabo\build
    cd .\libnabo\build\
    cmake-gui ..
    ```

    CMake-Gui will open up
1. Click on the button **Configure** and specify the generator for the project (Visual Studio 16 2019)
    > An error will be reported, because CMake does not know yet where to find the libraries. The next steps will tell it where to find them.
1. Set the CMake variable `EIGEN_INCLUDE_DIR` to `C:\dev\eigen-<version>`
1. Click on the button Configure, Generate and then Open Project
    Visual Studio will open up
    > Maybe you will have messages about Doxygen, OpenCL, ANN, FLANN and Python missing. They are not necessary to install libnano.
1. Put your "Solution Configuration" in `Release` mode
1. Build the `ALL_BUILD` project
1. Rebuild for each configuration mode you will use, i.e. Release/Debug/RelWithDebInfo/MinSizeRel  
    (Libnabo build a different .lib for each configuration mode)

### Install libpointmatcher  

> You need to install [libnabo](#install-libnabo) before installing libpointmatcher !

1. Go to your desired directory with your CLI (here `C:\dir`)
1. Do the following commands

    ```bash
    git clone https://github.com/ethz-asl/libpointmatcher
    mkdir .\libpointmatcher\build
    mkdir .\libpointmatcher\build\install
    cd .\libpointmatcher\build\
    cmake-gui ..
    ```

    CMake-Gui will open up
1. Click on the button **Configure** and specify the generator for the project (Visual Studio 16 2019)
1. Set the CMake variable `EIGEN_INCLUDE_DIR` to `C:\dev\eigen-<version>`
1. Set the CMake variable `CMAKE_INSTALL_PREFIX` to `C:/dev/libpointmatcher/build/install`
    > **CAUTION !** Use forward slash `/`
1. Click on the button Configure, Generate and then Open Project  
    Visual Studio will open up
1. Put your "Solution Configuration" in `Release` mode
1. Build the `INSTALL` project
    > We have to install the library and not only build it, because otherwise all CMake files won't be able to be found by programs using libpointmatcher
1. Set `libpointmatcher_DIR` environment variable to `C:\dev\libpointmatcher\build\install\share\libpointmatcher\cmake`

## Reporting Issues

Currently, we don't have a developer fully supporting compilation on Windows. If you can help refreshing this documentation, your help is more than welcome.

Before reporting new building issues, have a look in the current/past list of issues. Add as many details as you can since you will most probably receive answers from developers that cannot reproduce the problem on their side. Here are some of them:

- Your directory structure need to be well organized as mentioned in [Issue #136](https://github.com/ethz-asl/libpointmatcher/issues/136).
- There might be some problems related to libnabo as mentioned in [Issue #118](https://github.com/ethz-asl/libpointmatcher/issues/118).

## Special Thanks

Special thanks to the following users in helping us with the Windows support:

- [kwill](https://github.com/kwill) for keeping the documentation up-to-date and investing the time to get libpointmatcher compiling on Windows.
- [braddodson](https://github.com/braddodson) for porting a version of libpointmatcher to `C#` with a limited set of features. The code can be found here: https://github.com/braddodson/pointmatcher.net
