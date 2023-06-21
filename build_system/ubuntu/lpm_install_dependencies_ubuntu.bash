#!/bin/bash -i

# Note on unit test:
#    $ docker pull --platform linux/arm64 ubuntu:20.04
#    $ docker build --platform linux/arm64 -f Dockerfile.dependencies -t test-libpointmatcher-dependencies:ubuntu.20.04 .
#    $ docker run -a --name iAmTestLibpointmatcherDependenciesContainer -t -i test-libpointmatcher-dependencies:ubuntu.20.04

set -e

# Load environment variable from file
set -o allexport; source ../.env; set +o allexport


# skip GUI dialog by setting everything to default
export DEBIAN_FRONTEND=noninteractive

# .... Create required dir structure ...................................................................................
mkdir -p "${LPM_INSTALLED_LIBRARIES_PATH}"
cd "${LPM_INSTALLED_LIBRARIES_PATH}"

# ==== Install tools ===================================================================================================

# ... install development utilities ....................................................................................
sudo apt-get update &&
  sudo apt-get install --assume-yes \
    lsb-release \
    curl \
    wget \
    git \
    g++ \
    gcc \
    make \
    cmake \
    cmake-gui \
    build-essential \
    ca-certificates &&
  sudo rm -rf /var/lib/apt/lists/*

sudo apt-get update &&
  sudo apt-get install --assume-yes \
    python3-dev \
    python3-pip &&
  sudo rm -rf /var/lib/apt/lists/*

python3 -m pip install --upgrade pip

## ToDo: assessment >> check if next bloc ↓↓ is needed
#sudo apt-get update \
#    && sudo apt-get install --assume-yes \
#        python3-opengl \
#        python3-numpy \
#        python-is-python3 \
#        python3-vcstool \
#    && sudo rm -rf /var/lib/apt/lists/*;

# ==== Install Libpointmatcher dependencies ============================================================================
# . . Install boost. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# https://www.boost.org/doc/libs/1_79_0/more/getting_started/unix-variants.html
sudo apt-get update &&
  sudo apt-get install --assume-yes \
    libboost-all-dev &&
  sudo rm -rf /var/lib/apt/lists/*

# . . Install eigen . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . ..
# https://eigen.tuxfamily.org/index.php
sudo apt-get update &&
  sudo apt-get install --assume-yes \
    libeigen3-dev &&
  sudo rm -rf /var/lib/apt/lists/*

# . . Install libnabo . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . ..
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
