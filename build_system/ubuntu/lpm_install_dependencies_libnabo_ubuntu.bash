#!/bin/bash -i
# =================================================================================================
#
# Libpointmatcher dependencies installer
#
# Usage:
#   $ source lpm_install_dependencies_libnabo_ubuntu.bash [--test-run]
#
# Global
#   - Read the array APPEND_TO_CMAKE_FLAG
#
#     Usage:
#     $ export APPEND_TO_CMAKE_FLAG=( "-D CMAKE_INSTALL_PREFIX=/opt" ) \
#           && lpm_install_dependencies_libnabo_ubuntu.bash
#
# =================================================================================================
set -e # Note: we want the installer to always fail-fast (it wont affect the build system policy)

# ToDo: Add mention about 'CMAKE_INSTALL_PREFIX' in the doc install step as a fix

declare -a CMAKE_FLAGS

# skip GUI dialog by setting everything to default
export DEBIAN_FRONTEND=noninteractive

# ....Project root logic...........................................................................
TMP_CWD=$(pwd)

LPM_PATH=$(git rev-parse --show-toplevel)
cd "${LPM_PATH}/build_system" || exit 1

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
  n2st::norlab_splash "${NBS_SPLASH_NAME:?err}" "https://github.com/${NBS_REPOSITORY_DOMAIN:?err}/${NBS_REPOSITORY_NAME:?err}"
fi

n2st::print_formated_script_header "lpm_install_dependencies_libnabo_ubuntu.bash (${IMAGE_ARCH_AND_OS:?err})" "${MSG_LINE_CHAR_INSTALLER}"


# ....Script command line flags....................................................................
while [ $# -gt 0 ]; do

  case $1 in
  --test-run)
    TEST_RUN=true
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



# ....cmake flags..................................................................................
CMAKE_FLAGS=( -D CMAKE_BUILD_TYPE=RelWithDebInfo "${APPEND_TO_CMAKE_FLAG[@]}" )

# .................................................................................................
n2st::teamcity_service_msg_blockOpened "Install Libpointmatcher dependencies › Libnabo"
# https://github.com/norlab-ulaval/libnabo

## Note:
#   - ANN is not mentioned in doc because it's only required for `make test` benchmarks
#   - Leave it commented in code for future references
## ANN is a library written in C++, which supports data structures and algorithms for both exact and approximate nearest neighbor searching in arbitrarily high dimensions.
## https://www.cs.umd.edu/~mount/ANN/
#cd "${NBS_LIB_INSTALL_PATH}"
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
## Note:
#   - FLANN is not mentioned in doc because it's only required for `make test` benchmarks
#   - Leave it commented in code for future references
## Fast Library for Approximate Nearest Neighbors - development
## FLANN is a library for performing fast approximate nearest neighbor searches in high dimensional spaces.
## https://github.com/flann-lib/flann
#sudo apt-get update \
#    && sudo apt-get install --assume-yes \
#        libflann-dev \
#    && sudo rm -rf /var/lib/apt/lists/*

# ....Repository cloning step......................................................................
n2st::print_msg "Create required dir structure"
mkdir -p "${NBS_LIB_INSTALL_PATH}"

cd "${NBS_LIB_INSTALL_PATH}"
git clone https://github.com/norlab-ulaval/libnabo.git &&
  cd libnabo &&
  mkdir build && cd build

# git checkout 1.0.7

# ....Cmake install step...........................................................................
n2st::teamcity_service_msg_compilationStarted "cmake"

n2st::print_msg "Execute ${MSG_DIMMED_FORMAT}
cmake ${CMAKE_FLAGS[*]} ${NBS_LIB_INSTALL_PATH}/libnabo
${MSG_END_FORMAT}"

if [[ $TEST_RUN  == true ]]; then
  n2st::print_msg "Test-run mode: Skipping cmake"
  BUILD_EXIT_CODE=0
  INSTALL_EXIT_CODE=0
else

  cmake  "${CMAKE_FLAGS[@]}" "${NBS_LIB_INSTALL_PATH}/libnabo"

  make -j $(nproc)
  sudo make install

  # (NICE TO HAVE) ToDo: refactor (ref task NMO-428 refactor: drop libnabo `make test` step after libnabo-build-system deployment)
  #  make -j $(nproc) && make test && sudo make install
fi

n2st::teamcity_service_msg_compilationFinished

n2st::teamcity_service_msg_blockClosed

echo " " && n2st::print_msg_done "Libpointmatcher dependencies installed"
n2st::print_formated_script_footer "lpm_install_dependencies_libnabo_ubuntu.bash (${IMAGE_ARCH_AND_OS})" "${MSG_LINE_CHAR_INSTALLER}"

# ====Teardown=====================================================================================
cd "${TMP_CWD}"
