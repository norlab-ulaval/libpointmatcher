# Applying Data Point Filters

## Overview

The following will go through the steps needed to write a simple program which configures a chain of data filters and applies this chain to a point cloud.  For information on data filters, refer to the [data point filters tutorial](DataFilters.md).  The filtered point cloud is then saved to disk.  The source code associated with this tutorial can be found in [examples/demo_cmake/convert.cpp](https://github.com/norlab-ulaval/libpointmatcher/blob/master/examples/demo_cmake/convert.cpp).

***IMPORTANT:*** This tutorial makes use of YAML configuration files.  If you did not install yaml-cpp before installing libpointmatcher, you must do so before following these instructions.  Information on installing yaml-cpp can be found in the installation instructions for [Ubuntu](CompilationUbuntu.md), [Windows](CompilationWindows.md) and [Mac OS X](CompilationMac.md).

## Source Description

First, we include the libpointmatcher and standard library header files.

=== "C++"
    ```cpp
    #include "pointmatcher/PointMatcher.h"
    #include <cassert>
    #include <iostream>
    #include <fstream>
    ```

Next, we enter the `std` and `PointMatcherSupport` namespace scopes and declare type aliases.

=== "C++"
    ```cpp
    using namespace std;
    using namespace PointMatcherSupport;
    
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
    ```

The following function displays the usage message.  This program can be run with three arguments.  The first is the path to a yaml configuration file, the second a path to the input point cloud, and the third is a filename to be used to save the filtered point cloud.  If the first argument is omitted, the input point cloud is copied to the output point cloud.  The usage message will be displayed whenever the program is run with an incorrect number of arguments.

=== "C++"
    ```cpp
    void usage(char *argv[])
    {
        cerr << "Usage " << argv[0] << " [CONFIG.yaml] INPUT.csv/.vtk OUTPUT.csv/.vtk" << endl;
        cerr << endl << "Example:" << endl;
        cerr << argv[0] << " ../examples/data/default-convert.yaml ../examples/data/cloud.00000.vtk /tmp/output.vtk" << endl << endl;
    }
    ```

Now entering inside the main function of the program:  We create a logger to which warnings and errors are written.

=== "C++"
    ```cpp
    setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
    ```

We then load the input point cloud into a new `DataPoints` object `d`.

=== "C++"
    ```cpp
    DP d(DP::load(argv[argc-2]));
    ```

Next, the number of arguments is checked, and we attempt to load the configuration file.  If the configuration file could not be opened, an error message is printed and the program returns.  If it is loaded successfully, a chain of data filters is created and represented in a `DataPointsFilters` object.  The input point cloud is filtered by the chain using the `DataPointsFilters::apply(DataPoints d)` function. 

=== "C++"
    ```cpp
    if (argc == 4)
        {
            ifstream ifs(argv[1]);
            if (!ifs.good())
            {
                cerr << "Cannot open config file " << argv[1] << endl;
                usage(argv);
                return 2;
            }
            PM::DataPointsFilters f(ifs);
            f.apply(d);
    
        }
    ```

Finally, the filtered point cloud is written to the location specified by the user and the program returns successfully.

=== "C++"
    ```cpp
    d.save(argv[argc-1]);
        
    return 0;
    ```

## Configuring the Data Filters Chain

An example configuration file can be found at [examples/data/default-convert.yaml](https://github.com/norlab-ulaval/libpointmatcher/blob/master/examples/data/default-convert.yaml).  A diagram of this chain is shown in Figure 1.

|**Figure 1**: Default chain of data filters in `default-convert.yaml`|
|:---:|
| ![Default Configuration Chain](images/DefaultConvertChain.png) |


### Bounding Box Filter

The first element is a [bounding box filter](DataFilters.md#boundingboxhead). It is configured to reject points inside a 8m x 8m x 8m cube centered at the origin. 

```yaml
- BoundingBoxDataPointsFilter:
   xMin: -4.0
   xMax: 4.0
   yMin: -4.0
   yMax: 4.0
   zMin: -4.0
   zMax: 4.0
   removeInside: 1
```

### Sampling Surface Normal Filter

A [sampling surface normal filter](DataFilters.md#samplingnormhead) can be used in an attempt to subsample points while maintaining the shape structure of objects in the point cloud.  The point cloud is divided into boxes containing at most 10 points.  Points along flat surfaces are contained in larger boxes and are sparsely sampled while points on more complex surfaces are densely sampled. 

```yaml
- SamplingSurfaceNormalDataPointsFilter:
    knn: 10
```

### Observation Direction Filter

The points are annotated with an observation direction pointing to the sensor location with the [observation direction filter](DataFilters.md#obsdirectionhead).

```yaml
- ObservationDirectionDataPointsFilter
```

### Orient Normals Filter

The [last filter](DataFilters.md#orientnormalshead) is used to reorient normal vectors so that they point in the same direction.  This filter uses the observation directions extracted by the previous filter.

```yaml
- OrientNormalsDataPointsFilter
```

## Where To Go From Here

For more information on building your own configurations, refer to the tutorial on [building yaml configurations](Configuration.md).

