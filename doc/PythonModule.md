| [Tutorials Home](index.md) | [Previous](CompilationPython.md) |
| :--- | ---: |

# Using libpointmatcher with Python

This tutorial presents the different things to know before using *pypointmatcher*, the libpointmatcher's Python module.

## Differences between the C++ and Python APIs

Despite the fact that pypointmatcher and libpointmatcher have very similar APIs, the fact remains that they differ in some ways. So, why not start by listing these differences.

#### STL containers vs Python data structures

pybind11 provides automatic conversion between C++'s STL containers and their Python equivalent data structures. That is, `std::vector`/`std::deque`/`std::list`/`std::array` are converted into a Python `list`, `std::set`/`std::unordered_set` are converted into a Python `set` and finally `std::map`/`std::unordered_map` are converted into a Python `dict`. 

Although this can be very useful, it comes with some major downsides, [which you can see here](https://pybind11.readthedocs.io/en/latest/advanced/cast/stl.html#automatic-conversion), hence the need to make the classes inheriting from STL containers and some `typedef` *"opaque"*. For the moment, only the `std::vector` and the `std::map` containers are made *"opaque"*, i.e. they keep the same name as the one used in libpointmatcher, but they adopt the behavior of a `list` and a `dict` respectively.

> ***Note:*** This also means that these opaque STL containers must be used in constructors and methods that accept one of these types as parameter.
 
For more information about pybind11 STL containers conversion, visit [this section](https://pybind11.readthedocs.io/en/latest/advanced/cast/stl.html) of the official documentation.

#### Eigen vs Numpy

pybind11 also provides transparent conversion between *Eigen*'s `Matrix`, `Map` and `SparseMatrix` and *numpy*'s `array` and `ndarray` data types. That is, you can seamlessly use *numpy*'s `ndarray` instead of *Eigen*'s `Vector` or `Matrix`.

For more information about pybind11 *Eigen* to *numpy* data type conversion, visit [this section](https://pybind11.readthedocs.io/en/latest/advanced/cast/eigen.html) of the official documentation.

#### Overloaded methods based on constness

In libpointmatcher, more precisely in the `DataPoints` class, some methods are overloaded based on constness, i.e. they will be called with a constant `DataPoints`. So, to avoid ambiguous calls, the suffix `_const` has been appended to the method names. E.g. in the `compute_overlap.py` example, the `getDescriptorViewByName("inliers")` method was calling the `const` version before this fix. For more information on pybind11 overloaded method mechanisms, visit [this section](https://pybind11.readthedocs.io/en/latest/classes.html#overloaded-methods) of the official documentation.

#### Contructors/methods with std::istream or std::ostream as paramater

Some constructors and methods of libpointmatcher have as parameter either an `std::istream` to build an object from a YAML configuration file or an `std::ostream` to dump information. pybind11 doesn't allow to call these constructors/methods with their Python equivalent, i.e. `sys.stdin` and `sys.stdout`. So, to get around this problem, the constructors/methods having a `std::istream` as parameter must be used with a `std::string` instead and those having a `std::ostream` must be used without parameter.

## Structure of pypointmatcher

Before going further, here is the general structure of pypointmatcher to give you a better idea of how to use it.

```textmate
pypointmatcher # The main module.
|
|_ pointmatcher # The submodule containing functions and classes that are dependent on scalar types.
|
|_ pointmatchersupport # The submodule containing everything defined in the
|                       # pointmatchersupport namespace, i.e. functions and classes which are not dependent on scalar types.
|
|_ datapointsfilters # The submodule containing the differents DataPointsFilters.
|
|_ errorminimizers # The submodule containing the differents ErrorMinimizers.
```

## General use of pypointmatcher

To help you get familiar with pypointmatcher, some C++ examples have been translated to show you how it is possible to use the Python version of libpointmatcher.

Now that you know the major differences between the C++ and Python API, we suggest newcomers and those who are less familiar with the library to follow, or re-follow, the tutorials in the beginner and advanced sections using the Python examples instead, in order to have a better understanding of how libpointmatcher can be used with Python.

Experienced users of libpointmatcher can, if they wish, take a look at the different Python examples located in the `examples/python/` directory and compare their uses with their C++ counterparts, keeping in mind what makes them different from each other.

The major difference is that they don't require command line arguments like the C++ version. All you have to do is open a terminal and go to the `examples/python/` directory and run one of the examples as a Python module script:

```bash
python3 icp_simple.py
```

> ***Note:*** All the information to run a default ICP algorithm is already in the examples, it is up to the user to change these values in order to run a custom ICP algorithm. So take the time to understand what the code does in the examples before making changes.
