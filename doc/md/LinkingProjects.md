| [Tutorials Home](Tutorials.md)    | [Previous](Configuration.md) | [Next](Pointclouds.md) |
| ------------- |:-------------:| -----:|

# Linking Projects to libpointmatcher
######Latest update January 23, 2014 by Samuel Charreyron
Once you have followed the [compilation instructions](Compilation.md) and installed libpointmatcher to your system, you can use libpointmatcher in your project.

## Option 1: Using CMake (Recommended)
Because libpointmatcher was build using CMake, it can be conveniently included in other CMake projects.  You can simply use the `find_package` functionality of CMake to locate the installation directory of libpointmatcher.  Add `$POINTMATCHER_INCLUDE_DIRS` to the list of include directories in your project and link the appropriate executables to `$POINTMATCHER_LIBRARIES`.

In this following example, we build a very simple CMake project containing one executable in `myProgram.cpp` which depends on libpointmatcher.

```cmake
cmake_minimum_required (VERSION 2.6)
project (myProject)

find_package(pointmatcher 1.1.0 REQUIRED)
include_directories("${POINTMATCHER_INCLUDE_DIRS}")
message(STATUS "Using libpointmatcher version ${pointmatcher_VERSION}")

add_executable(myProgram myProgram.cpp)
target_link_libraries(myProgram ${POINTMATCHER_LIBRARIES})
```

## Option 2: Using Eclipse
### Using the Native Eclipse Builder
We will demonstrate how to create an Eclipse project containing a simple executable which depends on libpointmatcher.  You must have [Eclipse CDT](http://www.eclipse.org/cdt/) installed to develop with libpointmatcher in Eclipse.  

Create a new C++ project by clicking `File > New > C++ Project`.  You can name your project "PointmatcherEclipseDemo" and in toolchains select the default toolchain for your system (most likely Linux GCC).  Click `Finish` to add your project to your Eclipse workspace.  

You must then configure the project by going to `Project > Properties > C/C++Build > Settings`.  Navigate to `C++ Compiler > Includes` and add the libpointmatcher include path to the `Include paths (-I)` list.  Next, go to `C++ Linker/Libraries` and add the the following three dependencies to the "Libraries (-l)" list: 

* pointmatcher
* boost_system
* nabo

Click `Ok` to save the configuration.  Create a new source file by clicking `File > New > Source File` and name it "Demo.cpp".  In this file you can type the following:
 
```cpp
#include <pointmatcher/PointMatcher.h>
#include <iostream>

typedef PointMatcher<float> PM;


int main(int argc, char *argv[]) {
	PM::ICP icp;
	icp.setDefault();

	std::cout << "ICP configured to default." << std::endl;
	return 0;
}
```
The program will create an ICP chain, configure it to the default settings and exit subsequently.  Click on `Project > Build Project` and check that the project compiles successfully.  Finally run the program by clicking `Run > Run`. The message "ICP configured to default." should be displayed in the console.       

## Option 3: Using Compiler Flags
If you are compiling a very simple program without the use of a builder, simply include the libpointmatcher header files by setting the include flag in your compiler.  Example:
```
g++ -I/usr/local/include/pointmatcher -o myProgram.o -c myProgram.cpp
```
You can then link to the pointmatcher library using:
```
g++ myProgram.o   -o myProgram -lpointmatcher -lnabo -lboost_system -lyaml-cpp -lboost_filesystem -lrt
```
Nevertheless, it is always more convenient to use a builder such as CMake.
