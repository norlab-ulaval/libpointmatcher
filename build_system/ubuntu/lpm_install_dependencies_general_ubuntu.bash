#!/bin/bash -i
# =================================================================================================
#
# Libpointmatcher dependencies installer
#
# Usage:
#   $ source lpm_install_dependencies_general_ubuntu.bash [--test-run]
#
# =================================================================================================
set -e # Note: we want the installer to always fail-fast (it wont affect the build system policy)

declare -a  APT_FLAGS

# skip GUI dialog by setting everything to default
export DEBIAN_FRONTEND=noninteractive

# ....Project root logic...........................................................................
TMP_CWD=$(pwd)

LPM_PATH=$(git rev-parse --show-toplevel)
cd "${LPM_PATH}/build_system" || cd "${LPM_PATH}" || exit 1

# ....Load environment variables from file.........................................................
set -o allexport
source .env
set +o allexport

# ....Helper function..............................................................................
# import shell functions from utilities library
N2ST_PATH=${N2ST_PATH:-"${LPM_PATH}/build_system/utilities/norlab-shell-script-tools"}
source "${N2ST_PATH}/import_norlab_shell_script_tools_lib.bash"

# Set environment variable IMAGE_ARCH_AND_OS
cd "${N2ST_PATH}"/src/utility_scripts/ && source "which_architecture_and_os.bash"

# ====Begin========================================================================================
SHOW_SPLASH_IDU="${SHOW_SPLASH_IDU:-true}"

if [[ "${SHOW_SPLASH_IDU}" == 'true' ]]; then
  norlab_splash "${NBS_SPLASH_NAME}" "https://github.com/${NBS_REPOSITORY_DOMAIN:?err}/${NBS_REPOSITORY_NAME:?err}"
fi

print_formated_script_header "lpm_install_dependencies_general_ubuntu.bash (${IMAGE_ARCH_AND_OS:?err})" "${MSG_LINE_CHAR_INSTALLER}"

# ....Script command line flags....................................................................
while [ $# -gt 0 ]; do

  case $1 in
  --test-run)
    APT_FLAGS=( --dry-run )
    shift
    ;;
  --?* | -?*)
    echo "$0: $1: unrecognized option" >&2 # Note: '>&2' = print to stderr
    shift
    ;;
  *) # Default case
    break
    ;;
  esac

done


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
  && sudo apt-get install --assume-yes "${APT_FLAGS[@]}" \
    g++ \
    gcc \
    catch \
    make \
    cmake \
    cmake-gui &&
  sudo rm -rf /var/lib/apt/lists/*

##cmake --version

teamcity_service_msg_blockClosed
# .................................................................................................

# (Priority) ToDo: add check to see if executed in a docker container. Current check does not do what its intended
if [[ ${IS_TEAMCITY_RUN} == true ]]; then
  print_msg "The install script is run in teamCity >> the python install step was executed earlier in the Dockerfile.dependencies"
else
  print_msg "The install script is executed in stand alone mode"
  cd "${NBS_PATH:?err}/src/utility_scripts" || exit 1
  bash "./nbs_install_python_dev_tools.bash"
fi

# .................................................................................................
teamcity_service_msg_blockOpened "Install Libpointmatcher dependencies › Boost"
# https://www.boost.org/doc/libs/1_79_0/more/getting_started/unix-variants.html

sudo apt-get update &&
  sudo apt-get install --assume-yes "${APT_FLAGS[@]}" \
    libboost-all-dev &&
  sudo rm -rf /var/lib/apt/lists/*

teamcity_service_msg_blockClosed
# .................................................................................................
teamcity_service_msg_blockOpened "Install Libpointmatcher dependencies › Eigen"
# https://eigen.tuxfamily.org/index.php

sudo apt-get update &&
  sudo apt-get install --assume-yes "${APT_FLAGS[@]}" \
    libeigen3-dev &&
  sudo rm -rf /var/lib/apt/lists/*

teamcity_service_msg_blockClosed
# .................................................................................................
teamcity_service_msg_blockOpened "Install Libpointmatcher dev tools"

sudo apt-get update &&
  sudo apt-get install --assume-yes "${APT_FLAGS[@]}" \
    libyaml-cpp-dev &&
  sudo rm -rf /var/lib/apt/lists/*

teamcity_service_msg_blockClosed


print_formated_script_footer "lpm_install_dependencies_general_ubuntu.bash (${IMAGE_ARCH_AND_OS})" "${MSG_LINE_CHAR_INSTALLER}"
# ====Teardown=====================================================================================
cd "${TMP_CWD}"
