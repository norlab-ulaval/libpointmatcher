| [Tutorials Home](Tutorials.md)    | [Previous](BasicRegistration.md) | [Next]() |
| ------------- |:-------------:| -----:|

# Creating Custom Configurations with YAML
######Latest update January 16, 2014 by Samuel Charreyron

***IMPORTANT:*** This tutorial makes use of YAML configuration files.  If you did not install yaml-cpp before installing libpointmatcher, you must do so before following these instructions.  Information on installing yaml-cpp can be found in the [installation instructions](Compilation.md).

## Overview
The implementation of the ICP algorithm in libpointmatcher is modular and can be tailored to the user's needs by writing custom configurations.  Configurations in libpointmatcher are defined in YAML files.  Very briefly, [YAML](http://www.yaml.org/) is a document format which allows data to be written in a way which is both readable by a program and by a human.  For a more detailed treatment of the language, refer to the [project's specification page](http://www.yaml.org/spec/1.2/spec.html).

## Configuration of a Chain of DataPointFilters
The first libpointmatcher object that can be constructed by YAML files is `DataPointsFilters` which represents a chain of data filters.  The configuration is loaded by calling its constructor with a string representing the path to the configuration file as an argument.  The configuration file is structures as follows:

A filter is included by preceding its name by a dash -.  It's parameters are set in an indented section with each line representing a parameter.  When a parameter is not specified, the default values are used.

```yaml
- DataPointsFilter1
    param1: param1Value
    param2: param2Value
- DataPointsFilter2
```

Note that the order in which filters are included is important.  The first reason is that each filtering step alters the point cloud and the order in which each filtering step is done is important.  The second reason is that some filters require descriptors.  The filters generating these descriptors must thus be included further up the chain.  For more information on the different data filters available in libpointmatcher, their parameters and requirements, refer to the [data filters tutorial](Datafilters.md).

## Configuration of an ICP Chain
