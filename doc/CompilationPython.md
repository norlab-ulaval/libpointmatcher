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
pip3 install pytest
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

#### Specifying the path

First, you need to specify where you want the module to be installed. To do so, you must provide the path by setting the CMake variable `PYTHON_INSTALL_TARGET` with an absolute path to your Python environment `site-packages` location. This can be achieve manually or automatically.

##### The manual way:

Launch the Python interpreter and run the following commands to find the path to the `site-packages/` directory of your current Python environment:

```bash
>>> import site
>>> site.getsitepackages()
```

> ***Note:*** If you are using the system's Python environment, replace the `getsitepackages()` function call by `getusersitepackages()`.

This will output a list of installation paths for your current Python environment. Now, choose the one that is located in the `python_env_path/lib/python3.x/` directory. The command to run should look like this:

```bash
cmake -D PYTHON_INSTALL_TARGET=python_env_path/lib/python3.x/site-packages ..
```

> ***NOTE:*** Replace the `x` with your Python minor version number.

##### The automatic way:

If you don't want to set the path manually, here's a command that should automatically pick the right one for you:

```bash
cmake -D PYTHON_INSTALL_TARGET=$(python3 -c "import site; print(site.getsitepackages()[0])") ..
```

> ***Note:*** If you are using the system's Python environment, replace the `site.getsitepackages()[0]` by `site.getusersitepackages()`.

> ***IMPORTANT:*** *This last example is the default behavior if no path has been set before compiling the module.* ***Please, make sure that this corresponds to a valid location or the module will be installed in a wrong location and this will lead to an import error.***

#### Enabling the compilation

By default, pypointmatcher compilation is disabled. In order to compile it, you must set the CMake variable `BUILD_PYTHON_MODULE` to `ON`:
 
```bash
cmake -D BUILD_PYTHON_MODULE=ON ..
```

Everything is now set up to proceed to the compilation and the installation.

## Compilation

Now, to compile pypointmatcher into the `build/` directory, run the following command:

```bash
make pypointmatcher -j N
```

where `N` is the number of jobs (or threads) you allow at once on your computer for the compilation. If no argument is passed after `-j`, there will be no limit to the number of jobs.

> ***Note:*** *Depending on your system, the compilation can take quite some time, so consider leaving the `-j` command with no argument in order to speed up this step.*

## Installation

And finally, to install the module on your system, run the following command:

```bash
sudo make install
```

