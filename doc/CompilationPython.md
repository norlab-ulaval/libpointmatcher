# Compiling libpointmatcher with Python

This tutorial presents the different steps of compiling *pypointmatcher*, the libpointmatcher's Python module, on Ubuntu and Mac OS X.

## Prerequisites

To get started, you will need the same prerequisites as libpointmatcher, but also some additional dependencies as listed here:

| Name             | Version <br> (Tested October 2023 on Ubuntu 22.04) |
| :--------------- |:--------------------------------------------------:|
| pybind11         |                       2.5.0                        |
| Python3          |                      3.10.12                       |
| python3-dev      |                       3.10.6                       |
| catch            |                       1.12.1                       |
| pytest           |                       7.4.2                        |

> ***Note:*** *This guide assumes you're using a virtual python environment. For this, you can use the [virtualenv tool](https://virtualenv.pypa.io/en/stable/). If you are not familiar with Python virtual environments, you can [read this tutorial](https://realpython.com/python-virtual-environments-a-primer/), which explains very well the reasons for using a virtual environment, or [watch this video tutorial](https://youtu.be/nnhjvHYRsmM)*

Install `pytest` with `pip`:

```bash
pip3 install pytest wheel build
```

And `catch` and `python3-dev` with your package manager:

=== "Ubuntu"
    ```bash
    sudo apt install catch python3-dev
    ```
=== "MacOS"
    ```bash
    brew install catch2
    ```

#### Case-sensitivity
Note that to build pypointmatcher, your filesystem must be case-sensitive.
This applies especially to MacOS, where the APFS is case-insensitive for legacy reasons.
To overcome this, open `Disk Utility` and add a new case-sensitive APFS volume to your container.
Then, move libpointmatcher to the new volume.

The rest of this tutorial will guide you through the necessary steps to compile pypointmatcher.

## pybind11

pybind11 is a library used to create Python bindings of existing C++ code and vice versa. So, in order to be able to compile pypointmatcher, you must either install pybind11 on your system or add it as a git submodule in the libpointmatcher's `contrib/` directory. You must then create a symbolic link to this git submodule in the `python/` directory. Go [here](#installing-pybind11) for the installation steps or [here](#adding-pybind11) for the git sudmodule steps.

### Installing pybind11 <a name="installing-pybind11"></a>

The very first step is to clone [pybind11](https://github.com/pybind/pybind11) into a directory of your choice.

At the moment, pypointmatcher is only tested for compilation with **version 2.5.0** of pybind11. To install the right version, run the following commands:

```bash
cd pybind11
git checkout v2.5.0
```

Once this is done, run the following commands:

```bash
mkdir build && cd build
cmake ..

# With multiple versions of python3
# cmake -DPYTHON_EXECUTABLE=$(python3.6 -c "import sys; print(sys.executable)") ..

make check -j 4
```

This will both compile and run pybind11 tests. Next, you can install pybind11 by running this command:

```bash
sudo make install
```

Once this is done, return to libpointmatcher's `build/` directory.

You're now ready to proceed to the [configuration step](#configuration).

### Configuring the variables <a name="configuration"></a>

#### Enabling the compilation

By default, pypointmatcher compilation is disabled. In order to compile it, you must set the CMake variable `BUILD_PYTHON_MODULE` to `ON`:

```bash
cmake -DBUILD_PYTHON_MODULE=ON ..
```

Everything is now set up to proceed to the compilation and the installation.

> **Note** *If you want to compile boost with static linkage, then make sure that it was compiled with position independent code. pybind compiles dynamic library. Static linking in dynamic library can be done if the static library was compiled with position independent code.* 

## Compilation

Now, to compile pypointmatcher into the `build/` directory, run the following command:

```bash
make -j 4
sudo make install
```

## Installation

To install the module on your system, first open the `python` directory:
```bash
cd  <my-project-folder>/libpointmatcher/python
```
and install the bindings through pip:
```bash
python3 -m build --wheel --no-isolation --outdir /tmp/libpointmatcher
pip3 install /tmp/libpointmatcher/pypointmatcher-*.whl
```
Finally, verify that `pypointmatcher` can be imported as a regular Python package:
```bash
python -c "from pypointmatcher import *"
```
