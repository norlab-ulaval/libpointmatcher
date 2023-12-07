#!/bin/bash -i
#
# Libpointmatcher dependencies installer
#
# Usage:
#   $ source lpm_install_dependencies_general_ubuntu.bash
#
set -e # Note: we want the installer to always fail-fast (it wont affect the build system policy)

# skip GUI dialog by setting everything to default
export DEBIAN_FRONTEND=noninteractive

# ....Project root logic...........................................................................
TMP_CWD=$(pwd)

LPM_PATH=$(git rev-parse --show-toplevel)
cd "${LPM_PATH}/build_system" || cd "${LPM_PATH}" || exit

# ....Load environment variables from file.........................................................
set -o allexport
source .env
set +o allexport

# ....Helper function..............................................................................
# import shell functions from utilities library
source "${LPM_PATH}/build_system/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"

# Set environment variable NBS_IMAGE_ARCHITECTURE
source "${LPM_PATH}/build_system/lpm_utility_script/lpm_export_which_architecture.bash"

# ====Begin========================================================================================
SHOW_SPLASH_IDU="${SHOW_SPLASH_IDU:-true}"

if [[ "${SHOW_SPLASH_IDU}" == 'true' ]]; then
  norlab_splash "${NBS_SPLASH_NAME}" "https://github.com/${NBS_REPOSITORY_DOMAIN}/${NBS_REPOSITORY_NAME}"
fi

print_formated_script_header "lpm_install_dependencies_general_ubuntu.bash (${NBS_IMAGE_ARCHITECTURE})" "${NBS_LINE_CHAR_INSTALLER}"

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

# (Priority) ToDo: add check to see if executed in a docker container. Current check does not do what its intended
if [[ ${IS_TEAMCITY_RUN} == true ]]; then
  print_msg "The install script is run in teamCity >> the python install step was executed earlier in the Dockerfile.dependencies"
else
  print_msg "The install script is executed in stand alone mode"
#  source ./ubuntu/lpm_install_python_dev_tools.bash
  cd "${LPM_PATH}/build_system/utilities/norlab-build-system/src/container_tools" || exit
  bash "./nbs_install_python_dev_tools.bash"
#  source "${LPM_PATH}/build_system/utilities/norlab-build-system/src/container_tools/nbs_install_python_dev_tools.bash"
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
print_formated_script_footer "lpm_install_dependencies_general_ubuntu.bash (${NBS_IMAGE_ARCHITECTURE})" "${NBS_LINE_CHAR_INSTALLER}"
# ====Teardown=====================================================================================
cd "${TMP_CWD}"
