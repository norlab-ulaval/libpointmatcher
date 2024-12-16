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

NBS_PATH=${NBS_PATH:-"${LPM_PATH}/build_system/utilities/norlab-build-system"}
N2ST_PATH=${N2ST_PATH:-"${LPM_PATH}/build_system/utilities/norlab-shell-script-tools"}

# ....Load environment variables from file.........................................................
set -o allexport
source .env
set +o allexport

# ....Helper function..............................................................................
# import shell functions from utilities library
source "${N2ST_PATH}/import_norlab_shell_script_tools_lib.bash"


# Set environment variable IMAGE_ARCH_AND_OS
cd "${N2ST_PATH}"/src/utility_scripts/ && source "which_architecture_and_os.bash"

# ====Begin========================================================================================
SHOW_SPLASH_IDU="${SHOW_SPLASH_IDU:-true}"

if [[ "${SHOW_SPLASH_IDU}" == 'true' ]]; then
  n2st::norlab_splash "${NBS_SPLASH_NAME}" "https://github.com/${NBS_REPOSITORY_DOMAIN:?err}/${NBS_REPOSITORY_NAME:?err}"
fi

n2st::print_formated_script_header "lpm_install_dependencies_general_ubuntu.bash (${IMAGE_ARCH_AND_OS:?err})" "${MSG_LINE_CHAR_INSTALLER}"

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
n2st::teamcity_service_msg_blockOpened "Install development utilities"

sudo apt-get update &&
  sudo apt-get install --assume-yes \
    lsb-release \
    build-essential \
    ca-certificates \
    curl \
    wget \
    git \
    software-properties-common \
  && sudo apt-get install --assume-yes "${APT_FLAGS[@]}" \
    gcc \
    g++ \
    catch \
    make \
    python3-pip
  sudo rm -rf /var/lib/apt/lists/*
##cmake --version

# Retrieve ubuntu version number: DISTRIB_RELEASE
source /etc/lsb-release
print_msg "Ubuntu version is ${DISTRIB_RELEASE}"
if [[ ${DISTRIB_RELEASE} == '18.04' ]]; then
  # Update Bionic outdated compiler
  # Ref https://github.com/norlab-ulaval/libpointmatcher/pull/581#issuecomment-2284415233
  sudo add-apt-repository ppa:ubuntu-toolchain-r/test
  sudo apt-get update &&
    sudo apt-get install --assume-yes "${APT_FLAGS[@]}" \
      gcc-9 \
      g++-9 &&
  sudo rm -rf /var/lib/apt/lists/*
  sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-9
fi

n2st::teamcity_service_msg_blockClosed
# .................................................................................................

n2st::teamcity_service_msg_blockOpened "Install a newer CMake version"
dpkg -l | grep -q "^ii  cmake" && sudo apt remove --purge --auto-remove cmake
if [[ ${DISTRIB_RELEASE} == '18.04' ]]; then
    sudo apt update && \
    sudo apt install -y software-properties-common lsb-release && \
    sudo apt clean all
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
    sudo apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
    sudo apt update
    sudo apt install -y cmake
else
    wget https://bootstrap.pypa.io/get-pip.py
    PIP_BREAK_SYSTEM_PACKAGES=1 python3 get-pip.py
    PIP_BREAK_SYSTEM_PACKAGES=1 python3 -m pip install --upgrade pip
    PIP_BREAK_SYSTEM_PACKAGES=1 python3 -m pip install cmake
fi
print_msg "Cmake version is $(cmake --version)"
n2st::teamcity_service_msg_blockClosed
# .................................................................................................

# (Priority) ToDo: add check to see if executed in a docker container. Current check does not do what its intended
if [[ ${IS_TEAMCITY_RUN} == true ]]; then
  n2st::print_msg "The install script is run in teamCity >> the python install step was executed earlier in the Dockerfile.dependencies"
else
  n2st::print_msg "The install script is executed in stand alone mode"
  cd "${NBS_PATH:?err}/src/utility_scripts" || exit 1
  bash "./nbs_install_python_dev_tools.bash"
fi

# .................................................................................................
n2st::teamcity_service_msg_blockOpened "Install Libpointmatcher dependencies › Boost"
# https://www.boost.org/doc/libs/1_79_0/more/getting_started/unix-variants.html

sudo apt-get update &&
  sudo apt-get install --assume-yes "${APT_FLAGS[@]}" \
    libboost-all-dev &&
  sudo rm -rf /var/lib/apt/lists/*

n2st::teamcity_service_msg_blockClosed
# .................................................................................................
n2st::teamcity_service_msg_blockOpened "Install Libpointmatcher dependencies › Eigen"
# https://eigen.tuxfamily.org/index.php

sudo apt-get update &&
  sudo apt-get install --assume-yes "${APT_FLAGS[@]}" \
    libeigen3-dev &&
  sudo rm -rf /var/lib/apt/lists/*

n2st::teamcity_service_msg_blockClosed
# .................................................................................................
n2st::teamcity_service_msg_blockOpened "Install Libpointmatcher dev tools"

sudo apt-get update &&
  sudo apt-get install --assume-yes "${APT_FLAGS[@]}" \
    libyaml-cpp-dev &&
  sudo rm -rf /var/lib/apt/lists/*

n2st::teamcity_service_msg_blockClosed


n2st::print_formated_script_footer "lpm_install_dependencies_general_ubuntu.bash (${IMAGE_ARCH_AND_OS})" "${MSG_LINE_CHAR_INSTALLER}"
# ====Teardown=====================================================================================
cd "${TMP_CWD}" || exit
