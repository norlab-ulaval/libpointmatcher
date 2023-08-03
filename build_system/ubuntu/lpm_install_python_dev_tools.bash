#!/bin/bash
#
# Install python development tools
#
# usage:
#   $ bash build_system/lpm_install_python_dev_tools.bash
#
set -e

# .................................................................................................................
if [[ $(uname) == 'Linux' ]]; then

  # Retrieve ubuntu version number
  UBUNTU_VERSION=$(grep '^VERSION_ID' /etc/os-release)
  if [[ ${UBUNTU_VERSION} == '18.04' ]]; then
    # Case: Ubuntu 18.04 (bionic) ==> python 2

    sudo apt-get update &&
      sudo apt-get install --assume-yes \
        python-dev \
        python-numpy &&
      sudo rm -rf /var/lib/apt/lists/*

    # Work around to install pip in python2
    curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
    sudo python2 get-pip.py

  else

    sudo apt-get update &&
      sudo apt-get install --assume-yes \
        python3-dev \
        python3-pip \
        python3-numpy &&
      sudo rm -rf /var/lib/apt/lists/*

      # python3-opengl \

    if [[ ${UBUNTU_VERSION} == '20.04' ]]; then
      sudo apt-get update &&
        sudo apt-get install --assume-yes \
          python-is-python3 &&
        sudo rm -rf /var/lib/apt/lists/*
    fi

    python3 -m pip install --upgrade pip

  fi
fi
