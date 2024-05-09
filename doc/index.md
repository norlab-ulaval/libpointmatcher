![alt tag](images/banner_light.jpeg#only-light)
![alt tag](images/banner_dark.jpeg#only-dark)

---

# What is libpointmatcher about? 

libpointmatcher is a library that implements the Iterative Closest
Point (ICP) algorithm for alignment of point clouds. It supports both
point-to-point and point-to-plane ICP. With the former, it is able to
solve not only for a rigid transform, but also for a scale change
between the clouds (that is, a similarity transform).

More precisely, given two point clouds, R (the reference) and S (the
source), ICP tries to find the best rigid (or similarity) transform T
so that T * S = R.

The [Wikipedia article on ICP](https://en.wikipedia.org/wiki/Iterative_closest_point) has more
information.

libpointmatcher implements a [set of filters](DataFilters.md) to help
denoise and subsample the input point clouds. It supports a [variety
of file types](ImportExport.md) and it can be configured via both
[YAML files](Configuration.md) and an [in-memory API](ICPWithoutYaml.md).

libpointmatcher is written in C++.

# Tutorials

This page lists the available tutorials for libpointmatcher. The [Beginner](#beginner) section is aimed at the more casual user and contains high-level information on the various steps of point cloud registration. The [Advanced](#advanced) section is targeted at those with existing experience with point cloud registration and proficiency in C++ development.  Those who wish to contribute to libpointmatcher can follow the guidelines in the [Developer](#developer) section. Finally, the [Python](#python) section is intended for those who are interested in using libpointmatcher with *Python*.

## Beginner <a name="beginner"></a>

- [How to compile libpointmatcher](Compilation.md) <a name="compilation"></a>
- [What can I do with libpointmatcher?](ApplicationsAndPub.md)
- [What the different data filters do?](DataFilters.md)
- [Example: Applying a chain of data filters](ApplyingDataFilters.md)
- [What are the different outlier filters?](OutlierFiltersFamilies.md)
- [Example: An introduction to ICP](ICPIntro.md)
- [The ICP chain configuration and its variants](DefaultICPConfig.md)
- [Configuring libpointmatcher using YAML](Configuration.md)
- [Supported file types and importing/exporting point clouds](ImportExport.md)

## Advanced <a name="advanced"></a>

- [How to link a project to libpointmatcher?](LinkingProjects.md)
- [How to use libpointmatcher in ROS?](UsingInRos.md)
- [How are point clouds represented?](PointClouds.md)
- [Example: Writing a program which performs ICP](BasicRegistration.md)
- [How to move a point cloud using a rigid transformation?](Transformations.md)
- [Example: Configure an ICP solution without yaml](ICPWithoutYaml.md)
- Measuring Hausdorff distance, Haussdorff quantile and mean residual error? [See this discussion for code examples.](https://github.com/norlab-ulaval/libpointmatcher/issues/125)
- How to compute the residual error with `ErrorMinimizer::getResidualError(...)` [See the example code provided here.](https://github.com/norlab-ulaval/libpointmatcher/issues/193#issue-203885636)
- How to build a global map from a sequence of scans? [See the example align_sequence.cpp](https://github.com/norlab-ulaval/libpointmatcher/blob/master/examples/align_sequence.cpp ).
- How to minimize the error with translation, rotation and scale? [See this example.](https://github.com/norlab-ulaval/libpointmatcher/issues/188#issuecomment-270960696)
- How to do a nearest neighbor search between two point clouds without an ICP object? [See the comments here.](https://github.com/norlab-ulaval/libpointmatcher/issues/193#issuecomment-276093785)
- How to construct a `DataPoints` from my own point cloud? [See the unit test on `Datapoints` here.](https://github.com/norlab-ulaval/libpointmatcher/blob/master/utest/ui/DataFilters.cpp#L52)
- How to link against [PCL](https://pointclouds.org/)? [See the comments here.](https://github.com/norlab-ulaval/libpointmatcher/issues/176#issuecomment-734067786)

## Developer <a name="developer"></a>

- [Creating a DataPointsFilter](DataPointsFilterDev.md)
- [Creating a Transformation](TransformationDev.md)
- [Creating unit tests](UnitTestDev.md)
- [Contributing Instructions](contributing/contributing_instructions.md)
  - instructions for setting up the `libpointmatcher-build-system` on your workstation to speed up your local development
  - bug reporting
  - code contribution 
  workflow
- [Commit Message References](contributing/commit_msg_reference.md) (**Note:** this is optional for now but will eventually move our release workflow to semantic-versioning)

## Python <a name="python"></a>

- [Compiling libpointmatcher with Python](CompilationPython.md)
- [Using libpointmatcher with Python](PythonModule.md)

**Note**: if you don't find what you need, don't hesitate to propose or participate to new tutorials. 
