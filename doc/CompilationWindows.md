| [Tutorials Home](Tutorials.md)    | | [Next](Datafilters.md) |
| ------------- |:-------------:| -----:|

# Compiling and Installing libpointmatcher on Windows
######Latest update March 29, 2014 by François Pomerleau

## Foreword
*The following instructions are aimed at users of Windows.  The steps from this tutorial were performed on __Windows 7__.  These instructions should be identical previous versions of Windows.*


## Compiling using MSVC (Microsoft Visual Studio)

### In Short...

If you are used to development project, here what you need:


| Name   | Link | Version tested|
| ------ | ---- | ------------- |
|  git | <http://windows.github.com/> | v1.0 |
|  libpointmatcher sources   | <https://github.com/ethz-asl/libpointmatcher> | |
| libnabo sources | <https://github.com/ethz-asl/libnabo> |  |
| Visual Studio |  <http://www.microsoft.com/visualstudio/eng/downloads>  | Visual Studio 2012 Express for Windows Desktop |
| CMake | <http://www.cmake.org/cmake/resources/software.html> | cmake-2.8.11.2-win32-x86.exe|
| Eigen 3 | <http://eigen.tuxfamily.org/index.php?title=Main_Page#Download>  |v3.2.0 |
| Boost | <http://www.boost.org/users/download/> | v1.54.0 |
| yaml-cpp | <https://code.google.com/p/yaml-cpp/downloads/list>| v0.3.0, **not working with v0.5.0**|
| grep tool | <http://gnuwin32.sourceforge.net/packages/grep.htm>| v2.5.4 |
| gtest | <https://code.google.com/p/googletest/> | v1.7.0 |



### Building Boost
1. Open a console that knows the path to the MSVC compiler command (cl). We suggest to use **Windows PowerShell**. An alternative is from the Start menu in the Visual Studio section; for instance for VS 2012, it is called Developer Command Prompt for VS2012.
1. Go to your Boost source directory, and do:

    ```
    $ bootstrap
    $ b2 install --prefix=build address-model=64
    ```

1. It may take awhile to finish.


### Build libnabo 
1. Start **CMake Gui**

1. Add the path of your libnabo sources in the field _Where is the source code_.
1. Add a folder named build in the field _Where to build the binary_. This will allow you to do out-of-source compilation.
1. Click on the button Configure
    1. Select the generator for the project (Visual Studio 11 Win 64)
    1. Error will be reported, because CMake does not know yet where to find the libraries. The next steps will tell it where to find them.

1. Locate _your eigen folder_ in the field **EIGEN_INCLUDE_DIR**

1. Add the following boolean variable and set it to `true`: **Boost_USE_STATIC_LIBS**

1. Add the following PATH variable and set it to _(your boost folder)_/build: **BOOST_ROOT**

1. Change the variable **CMAKE_CONFIGURATION_TYPES** to `RelWithDebInfo`

1. Click on the button Configure again, then on Generate

1. Locate the Microsoft Visual Studio Solution file in the your build folder (libnabo.sln) and open it. Visual Studio should open.

1. Build the solution: BUILD -> Build Solution

    Command line alternative for building: in _(your libnabo folder)_/build:
    
    ```
    $ msbuild /m:2 libnabo.sln
    ```
    
    Note that the flag /m:X defines the number of core to use.
    

### Building yaml-cpp
1. Start CMake Gui, follow the same building step

1. Change the variable **CMAKE_CONFIGURATION_TYPES** to `RelWithDebInfo`

1. Click on the button Configure, then on Generate

1. In visual Studio, build the solution: BUILD -> Build Solution

    Command line alternative: in _(your yaml-cpp folder)_/build:
    
    ```
    $ msbuild /m:2 YAML_CPP.sln
    ```
    
    Note that the flag /m:X defines the number of core to use.
    

### Building gtest
1. Open the file CMakeList.txt and add at the end:

    ```
    if( MSVC ) # VS2012 does not support tuples correctly yet
    	   add_definitions( /D _VARIADIC_MAX=10 )
    endif()
    ```
    
1. Start CMake-Gui to generate a MSVC solution

1. Change the variable **CMAKE_CONFIGURATION_TYPES** to `RelWithDebInfo`

1. Change the variable **gtest_force_shared_crt** to `TRUE`

1. Click on the button Configure, then on Generate

1. In visual Studio, build the solution: BUILD -> Build Solution

    Command line alternative: in _(your gtest folder)_/build:
    
    ```
    $ msbuild /m:2 gtest.sln
    ```
    
    Note that the flag /m:X defines the number of core to use.
    
### Build libpointmatcher
1. Start CMake Gui

1. Follow the same building steps

1. Add the following boolean variable and set it to true: Boost_USE_STATIC_LIBS

1. Add the following PATH variable and set it to [boost folder]/build: BOOST_ROOT

1. Change the variable **CMAKE_CONFIGURATION_TYPES** to `RelWithDebInfo`

1. You will need to fill manually the following variables:
    
    1. **EIGEN_INCLUDE_DIR** to _(your eigen folder)_
    1. **NABO_INCLUDE_DIR** to _(your nabo folder)_
    1. **NABO_LIBRARY** to _(your nabo folder)_/build/RelWithDebInfo/nabo.lib
    1. **yaml-cpp_INCLUDE_DIRS**
    1. **yaml-cpp_LIBRARIES**
    1. **GTEST_INCLUDE_DIR**
    1. **GTEST_LIBRARY**
    1. **GTEST_MAIN_LIBRARY**
    
1. In visual Studio, build the solution: BUILD -> Build Solution

    Command line alternative: in _(your libpointmatcher folder)_/build:
    
    ```
    $ msbuild /m:2 libpointmatcher.sln
    ```
    
    Note that the flag /m:X defines the number of core to use.
