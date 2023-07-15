#!/bin/bash -i
#
#
set -e
#set -v

# ....Project root logic...........................................................................................
TMP_CWD=$(pwd)

if [[ "$(basename $(pwd))" != "build_system" ]]; then
  cd ../
fi


# ....Load environment variables from file.........................................................................
set -o allexport
source ./.env
source ./.env.prompt
set +o allexport

# skip GUI dialog by setting everything to default
export DEBIAN_FRONTEND=noninteractive

# ....Helper function..............................................................................................
# import shell functions from Libpointmatcher-build-system utilities library
source ./function_library/prompt_utilities.bash

# ====Begin========================================================================================================
print_formated_script_header 'lpm_install_dependencies_ubuntu.bash' =


# ................................................................................................................
echo
print_msg "Install development utilities"
echo

sudo apt-get update &&
  sudo apt-get install --assume-yes \
    lsb-release \
    build-essential \
    ca-certificates \
    curl \
    wget \
    git &&
  sudo rm -rf /var/lib/apt/lists/*

# Compiler related package
sudo apt-get update &&
  sudo apt-get install --assume-yes \
    g++ \
    gcc \
    make \
    cmake \
    cmake-gui &&
  sudo rm -rf /var/lib/apt/lists/*

cmake --version

source ./ubuntu/lpm_install_python_dev_tools.bash

## ToDo: assessment >> check if next bloc ↓↓ is needed
#sudo apt-get update \
#    && sudo apt-get install --assume-yes \
#        python3-opengl \
#        python3-numpy \
#        python-is-python3 \
#        python3-vcstool \
#    && sudo rm -rf /var/lib/apt/lists/*;

# ................................................................................................................
echo
print_msg "Install Libpointmatcher dependencies › 'Boost'"
echo
# https://www.boost.org/doc/libs/1_79_0/more/getting_started/unix-variants.html

sudo apt-get update &&
  sudo apt-get install --assume-yes \
    libboost-all-dev &&
  sudo rm -rf /var/lib/apt/lists/*

# ................................................................................................................
echo
print_msg "Install Libpointmatcher dependencies › 'Eigen'"
echo
# https://eigen.tuxfamily.org/index.php

sudo apt-get update &&
  sudo apt-get install --assume-yes \
    libeigen3-dev &&
  sudo rm -rf /var/lib/apt/lists/*

# ................................................................................................................
print_msg "Create required dir structure"
mkdir -p "${LPM_INSTALLED_LIBRARIES_PATH}"

# ................................................................................................................
echo
print_msg "Install Libpointmatcher dependencies › 'Libnabo'"
echo
# https://github.com/ethz-asl/libnabo

# ToDo: assessment >> ANN and FLANN should be required only for `make test` benchmarks

## Note:ANN was not mentionned in doc --> probably because it's only used in benchmark test
## ANN is a library written in C++, which supports data structures and algorithms for both exact and approximate nearest neighbor searching in arbitrarily high dimensions.
## https://www.cs.umd.edu/~mount/ANN/
#cd "${LPM_INSTALLED_LIBRARIES_PATH}"
#wget https://www.cs.umd.edu/~mount/ANN/Files/1.1.2/ann_1.1.2.tar.gz
#tar xzf ann_1.1.2.tar.gz
#cd ann_1.1.2/
#make linux-g++
#sudo cp lib/libANN.a /usr/local/lib/
#sudo cp include/ANN/ANN.h /usr/local/include/
## shellcheck disable=SC2103
#cd ..
#
#
## Note:FLANN was not mentionned in doc --> probably because it's only used in benchmark test
## Fast Library for Approximate Nearest Neighbors - development
## FLANN is a library for performing fast approximate nearest neighbor searches in high dimensional spaces.
## https://github.com/flann-lib/flann
#sudo apt-get update \
#    && sudo apt-get install --assume-yes \
#        libflann-dev \
#    && sudo rm -rf /var/lib/apt/lists/*

cd "${LPM_INSTALLED_LIBRARIES_PATH}"
git clone https://github.com/ethz-asl/libnabo.git &&
  cd libnabo &&
  mkdir build && cd build &&
  cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo .. &&
  make -j $(nproc) &&
  make test &&
  sudo make install

#    && git checkout 1.0.7 \

# ToDo:on task end >> next bloc ↓↓
#pwd && tree -L 3

# ................................................................................................................
print_msg "Install libpointmatchaer dev tools"

sudo apt-get update &&
  sudo apt-get install --assume-yes \
    libyaml-cpp-dev &&
  sudo rm -rf /var/lib/apt/lists/*


print_msg_done "Libpointmatcher dependencies installed"
print_formated_script_footer 'lpm_install_dependencies_ubuntu.bash' =
# ====Teardown=====================================================================================================
cd "${TMP_CWD}"

