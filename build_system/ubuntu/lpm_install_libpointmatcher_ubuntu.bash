#!/bin/bash -i
#
# Note:
#   - this script required package: g++, make, cmake, build-essential, git and all libpointmatcher dependencies
#   - execute `lpm_install_dependencies_ubuntu.bash` first

# Note on unit test:
#    $ docker pull --platform linux/arm64 test-libpointmatcher-dependencies:ubuntu.20.04
#    $ docker build --platform linux/arm64 -f Dockerfile.libpointmatcher -t test-libpointmatcher:ubuntu.20.04 .
#    $ docker run -a --name iAmTestLibpointmatcherContainer -t -i test-libpointmatcher:ubuntu.20.04

set -e

# Load environment variable from file
set -o allexport; source ../.env; set +o allexport


# skip GUI dialog by setting everything to default
export DEBIAN_FRONTEND=noninteractive

# .... Create required dir structure .............................................................................

mkdir -p "${LPM_INSTALLED_LIBRARIES_PATH}"
cd "${LPM_INSTALLED_LIBRARIES_PATH}"


# ....helper function..............................................................................................
SP="    "

function print_help_in_terminal() {

  echo -e "\$ ${0} [<optional argument>]

${SP}
${SP}\033[1m<optional argument>:\033[0m
${SP}  -h, --help         Get help
${SP}  --compile-test     Compile the libpointmatcher unit-test in ${LPM_INSTALLED_LIBRARIES_PATH}/libpointmatcher/build
${SP}  --generate-doc     Generate the libpointmatcher doc in /usr/local/share/doc/libpointmatcher/api/html/index.html

"
}

# ....Script command line flags....................................................................................

## todo: on task end >> comment next dev bloc ↓↓
#echo "${0}: all arg >>" \
#  && echo "${@}"

BUILD_TESTS_FLAG=FALSE
GENERATE_API_DOC_FLAG=FALSE

USER_ARG=""

for arg in "$@"; do
  case $arg in
  -h|--help)
    print_help_in_terminal
    exit
    ;;
  --compile-test)
    BUILD_TESTS_FLAG=TRUE
    shift
    ;;
  --generate-doc)
    GENERATE_API_DOC_FLAG=TRUE
    shift
    ;;
  --) # no more option
    shift
    break
    ;;
  -?* | --?*)
    echo "$0: $1: unrecognized option" >&2 # Note: '>&2' = print to stderr
    ;;
  *) # Default case
    break
    ;;
  esac

  shift
done


# ==== Install tools =============================================================================================
sudo apt-get update &&
  sudo apt-get install --assume-yes \
    libyaml-cpp-dev &&
  sudo rm -rf /var/lib/apt/lists/*

if [ ${GENERATE_API_DOC_FLAG} == 'TRUE' ]; then
  sudo apt-get update &&
    sudo apt-get install --assume-yes \
      doxygen \
      texlive-full &&
    sudo rm -rf /var/lib/apt/lists/*
fi

# ==== Install libpointmatcher ====================================================================================
# https://github.com/ethz-asl/libpointmatcher/tree/master

#git clone https://github.com/ethz-asl/libpointmatcher.git &&
git clone https://github.com/${LPM_LIBPOINTMATCHER_SRC_DOMAIN}/${LPM_LIBPOINTMATCHER_SRC_REPO}.git &&
  cd libpointmatcher &&
  mkdir build && cd build &&
  cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo \
    -D BUILD_TESTS=${BUILD_TESTS_FLAG} \
    -D GENERATE_API_DOC=${GENERATE_API_DOC_FLAG} \
    .. &&
  make -j $(nproc) &&
  sudo make install

#   -DCMAKE_INSTALL_PREFIX=/usr/local/ \
#    && git checkout 1.3.1 \

# ==== Execute libpointmatcher unit-test===========================================================================
if [ ${BUILD_TESTS_FLAG} == 'TRUE' ]; then
  cd "${LPM_INSTALLED_LIBRARIES_PATH}/libpointmatcher/build"
  utest/utest --path "${LPM_INSTALLED_LIBRARIES_PATH}/libpointmatcher/examples/data/"
fi

## ToDo: assessment >> dont think it should be part of the libpointmatcher install script
#cd "${LPM_INSTALLED_LIBRARIES_PATH}"
#git clone https://github.com/norlab-ulaval/norlab_icp_mapper.git \
#    && mkdir -p norlab_icp_mapper/build && cd norlab_icp_mapper/build \
#    && cmake -DCMAKE_BUILD_TYPE=Release .. \
#    && make -j $(nproc) \
#    && sudo make install
