#!/bin/bash -i
#
# Libpointmatcher dependencies installer
#
# Usage:
#   $ source lpm_install_dependencies_general_ubuntu.bash
#
set -e # Note: we want the installer to always fail-fast (it wont affect the build system policy)

# ....Project root logic...........................................................................
TMP_CWD=$(pwd)

if [[ "$(basename $(pwd))" != "build_system" ]]; then
  cd ../
fi

# ....Load environment variables from file.........................................................
set -o allexport
source ./.env
source ./.env.prompt
set +o allexport

# skip GUI dialog by setting everything to default
export DEBIAN_FRONTEND=noninteractive

# ....Helper function..............................................................................
## import shell functions from Libpointmatcher-build-system utilities library
source ./function_library/prompt_utilities.bash
source ./function_library/terminal_splash.bash
source ./function_library/general_utilities.bash

# Set environment variable LPM_IMAGE_ARCHITECTURE
source ./lpm_utility_script/lpm_export_which_architecture.bash

# ====Begin========================================================================================
SHOW_SPLASH_IDU="${SHOW_SPLASH_IDU:-true}"

if [[ "${SHOW_SPLASH_IDU}" == 'true' ]]; then
  norlab_splash "${LPM_SPLASH_NAME}" "https://github.com/${LPM_LIBPOINTMATCHER_SRC_DOMAIN}/${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}"
fi

print_formated_script_header "lpm_install_dependencies_general_ubuntu.bash (${LPM_IMAGE_ARCHITECTURE})" "${LPM_LINE_CHAR_INSTALLER}"

# .................................................................................................
teamcity_service_msg_blockOpened "Install development utilities"

sudo apt-get update &&
  sudo apt-get install --assume-yes \
    lsb-release \
    build-essential \
    ca-certificates \
    curl \
    wget \
    git \
    g++ \
    gcc \
    catch \
    make \
    cmake \
    cmake-gui &&
  sudo rm -rf /var/lib/apt/lists/*

cmake --version

teamcity_service_msg_blockClosed
# .................................................................................................

if [[ ${IS_TEAMCITY_RUN} == true ]]; then
  print_msg "The install script is run in teamCity >> the python install step was executed earlier in the Dockerfile.dependencies"
else
  print_msg "The install script is executed in stand alone mode"
  source ./ubuntu/lpm_install_python_dev_tools.bash
fi

# .................................................................................................
teamcity_service_msg_blockOpened "Install Libpointmatcher dependencies › Boost"
# https://www.boost.org/doc/libs/1_79_0/more/getting_started/unix-variants.html

sudo apt-get update &&
  sudo apt-get install --assume-yes \
    libboost-all-dev &&
  sudo rm -rf /var/lib/apt/lists/*

teamcity_service_msg_blockClosed
# .................................................................................................
teamcity_service_msg_blockOpened "Install Libpointmatcher dependencies › Eigen"
# https://eigen.tuxfamily.org/index.php

sudo apt-get update &&
  sudo apt-get install --assume-yes \
    libeigen3-dev &&
  sudo rm -rf /var/lib/apt/lists/*

teamcity_service_msg_blockClosed
# .................................................................................................
teamcity_service_msg_blockOpened "Install Libpointmatcher dev tools"

sudo apt-get update &&
  sudo apt-get install --assume-yes \
    libyaml-cpp-dev &&
  sudo rm -rf /var/lib/apt/lists/*

teamcity_service_msg_blockClosed

#echo " " && print_msg_done "Libpointmatcher general dependencies installed"
print_formated_script_footer "lpm_install_dependencies_general_ubuntu.bash (${LPM_IMAGE_ARCHITECTURE})" "${LPM_LINE_CHAR_INSTALLER}"
# ====Teardown=====================================================================================
cd "${TMP_CWD}"
