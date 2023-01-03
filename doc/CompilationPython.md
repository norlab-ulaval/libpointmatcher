| [Tutorials Home](index.md) | [Previous](UnitTestDev.md) | [Next](PythonModule.md) |
| :--- | :---: | ---: |

# Compiling libpointmatcher with Python

This tutorial presents the different steps of compiling *pypointmatcher*, the libpointmatcher's Python module, on Ubuntu and Mac OS X.

## Prerequisites

To get started, you will need the same prerequisites as libpointmatcher, but also some additional dependencies as listed here:

| Name | Version <br> (Tested August 2020 on Ubuntu 18.04) |
| :--- | :---: |
| pybind11 | 2.5.0 |
| Python3 | 3.6.9 |
|||
| **Dependencies** | |
| python3-dev | 3.6.7 |
| catch | 1.10.0 |
| pytest | 5.4.3 |

`pytest` needs to be installed with `pip`:

```bash
pip3 install pytest wheel
```

But `catch` and `python3-dev` need to be installed with a package manager:

*Ubuntu users:*

```bash
sudo apt install catch python3-dev
```

*Mac OS users*:

```bash
brew install catch2
```

The rest of this tutorial will guide you through the necessary steps to compile pypointmatcher.

## pybind11

pybind11 is a library used to create Python bindings of existing C++ code and vice versa. So, in order to be able to compile pypointmatcher, you must either install pybind11 on your system or add it as a git submodule in the libpointmatcher's `contrib/` directory. You must then create a symbolic link to this git submodule in the `python/` directory. Go [here](#installing-pybind11) for the installation steps or [here](#adding-pybind11) for the git sudmodule steps.

### Installing pybind11 (recommended) <a name="installing-pybind11"></a>

The very first step is to clone [pybind11](https://github.com/pybind/pybind11) into a directory of your choice.
 
At the moment, pypointmatcher can only be compiled with **version 2.5.0** of pybind11. To install the right version, run the following commands:
 
```bash
cd pybind11
git checkout v2.5.0
```
 
Once this is done, run the following commands:

```bash
mkdir build && cd build
cmake ..
make check -j 4
```

This will both compile and run pybind11 tests. Next, you can install pybind11 by running this command:

```bash
sudo make install
```

Once this is done, return to libpointmatcher's `build/` directory.

You're now ready to proceed to the [configuration step](#configuration).

### Adding pybind11 as a `git` submodule <a name="adding-pybind11"></a>

An alternative to installing pybind11 on your system is to add its repository as a git submodule and create a symbolic link into the `python/` directory. To do this, you will first need to clone the repository as a git submodule by running the following commands in your terminal from the `contrib/` directory.

```bash
cd contrib
git submodule add https://github.com/pybind/pybind11.git
```

This will add pybind11 as a git submodule of libpointmatcher into the `contrib/` directory. Then, still from the `contrib/` directory, run this command to create a symbolic link to pybind11 in the `python/` directory:

```bash
ln -sr pybind11 ../python/pybind11
```

At the moment, pypointmatcher can only be compiled with **version 2.5.0** of pybind11. To install the right version, run the following commands:

```bash
cd pybind11
git checkout v2.5.0
```

Finally, tell CMake that you want to use pybind11 as a git submodule by setting the `USE_SYSTEM_PYBIND11` variable to `OFF`:

```bash
cmake -D USE_SYSTEM_PYBIND11=OFF ..
```

> ***IMPORTANT:*** When this method is used, it is very important to checkout the version **2.5.0** of pybind11 or it will be impossible to generate the build files.

Once this is done, return to libpointmatcher's `build/` directory.

You're now ready to proceed to the [configuration step](#configuration).

## Configuring the variables <a name="configuration"></a>

> ***Note:*** *It is recommended to create a virtual environment before proceeding with the next steps. For this, you can use the [virtualenv tool](https://virtualenv.pypa.io/en/stable/). If you are not familiar with Python virtual environments, you can [read this tutorial](https://realpython.com/python-virtual-environments-a-primer/), which explains very well the reasons for using a virtual environment, or [watch this video tutorial](https://youtu.be/nnhjvHYRsmM)*

#### Enabling the compilation

By default, pypointmatcher compilation is disabled. In order to compile it, you must set the CMake variable `BUILD_PYTHON_MODULE` to `ON` and `PYTHON_INSTALL_TARGET`:
 
```bash
cmake -DBUILD_PYTHON_MODULE=ON -DPYTHON_INSTALL_TARGET:PATH="./python/pypointmatcher" ..
```

Everything is now set up to proceed to the compilation and the installation.

## Compilation

Now, to compile pypointmatcher into the `build/` directory, run the following command:

```bash
cmake --build . -j N --target install
```

where `N` is the number of jobs (or threads) you allow at once on your computer for the compilation. If `-j` is omitted the native build tool's default number is used.

> ***Note:*** *Depending on your system, the compilation can take quite some time, so consider leaving the `-j` command with no argument in order to speed up this step.*

## Installation

And finally, to install the module on your system, run the following command:

```console
pip3 install ./python/pypointmatcher/*.whl
```
