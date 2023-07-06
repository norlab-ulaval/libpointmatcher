#!/bin/bash -i
#
# Note:
#   - this script required package: g++, make, cmake, build-essential, git and all libpointmatcher dependencies
#   - execute `lpm_install_dependencies_ubuntu.bash` first
#
# Note on unit test:
#    $ docker pull --platform linux/arm64 test-libpointmatcher-dependencies:ubuntu.20.04
#    $ docker build --platform linux/arm64 -f Dockerfile.libpointmatcher -t test-libpointmatcher:ubuntu.20.04 .
#    $ docker run -a --name iAmTestLibpointmatcherContainer -t -i test-libpointmatcher:ubuntu.20.04
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

function print_help_in_terminal() {
  echo -e "\$ ${0} [<optional argument>]

  \033[1m<optional argument>:\033[0m
    -h, --help                 Get help
    --compile-test             Compile the libpointmatcher unit-test in ${LPM_INSTALLED_LIBRARIES_PATH}/libpointmatcher/build
    --generate-doc             Generate the libpointmatcher doc in /usr/local/share/doc/libpointmatcher/api/html/index.html
    --build-system-install     Skip the git clone install step and assume the script is runned from a container in the build system

  "
}


# ====Begin========================================================================================================
print_formated_script_header 'lpm_install_libpointmatcher_ubuntu.bash' =



# ....Script command line flags....................................................................................

BUILD_TESTS_FLAG=FALSE
GENERATE_API_DOC_FLAG=FALSE
BUILD_SYSTEM_INSTALL=FALSE

#for arg in "$@"; do
while [ $# -gt 0 ]; do

#  echo -e "'\$*' before: ${MSG_DIMMED_FORMAT}$*${MSG_END_FORMAT}" # ToDo: on task end >> delete this line ←
#  echo -e "\$1: ${1}    \$2: $2" # ToDo: on task end >> delete this line ←
##  echo -e "\$arg: ${arg}" # ToDo: on task end >> delete this line ←


  case $1 in
  -h | --help)
    print_help_in_terminal
    exit
    ;;
  # ToDo: unit-test
  --compile-test)
    BUILD_TESTS_FLAG=TRUE
    shift
    ;;
  # ToDo: unit-test
  --generate-doc)
    GENERATE_API_DOC_FLAG=TRUE
    shift
    ;;
  # ToDo: unit-test
  --build-system-install)
    BUILD_SYSTEM_INSTALL=TRUE
    shift
    ;;
  --) # no more option
    shift
    break
    ;;
  --?* | -?*)
    echo "$0: $1: unrecognized option" >&2 # Note: '>&2' = print to stderr
    ;;
  *) # Default case
    break
    ;;
  esac


#  echo -e "'\$*' after: ${MSG_DIMMED_FORMAT}$*${MSG_END_FORMAT}" # ToDo: on task end >> delete this line ←
#  echo -e "after \$1: ${1}    \$2: $2" # ToDo: on task end >> delete this line ←
#  echo

done

#echo -e "'\$*' on DONE: ${MSG_DIMMED_FORMAT}$*${MSG_END_FORMAT}" # ToDo: on task end >> delete this line ←
#
## ToDo: on task end >> delete next bloc ↓↓
#echo -e "${MSG_DIMMED_FORMAT}
#BUILD_TESTS_FLAG=${BUILD_TESTS_FLAG}
#GENERATE_API_DOC_FLAG=${GENERATE_API_DOC_FLAG}
#BUILD_SYSTEM_INSTALL=${BUILD_SYSTEM_INSTALL}
#${MSG_END_FORMAT}"
#
#echo "printenv" && printenv # ToDo: on task end >> delete this line ←

# ................................................................................................................
print_msg "Create required dir structure"

mkdir -p "${LPM_INSTALLED_LIBRARIES_PATH}"
cd "${LPM_INSTALLED_LIBRARIES_PATH}"

## ................................................................................................................
## ToDo: on task end >> delete next bloc ↓↓
#print_msg "Install tools"
#
#sudo apt-get update &&
#  sudo apt-get install --assume-yes \
#    libyaml-cpp-dev &&
#  sudo rm -rf /var/lib/apt/lists/*
#
#if [[ ${GENERATE_API_DOC_FLAG} == 'TRUE' ]]; then
#  sudo apt-get update &&
#    sudo apt-get install --assume-yes \
#      doxygen \
#      texlive-full &&
#    sudo rm -rf /var/lib/apt/lists/*
#fi

# ................................................................................................................
print_msg "Install Libpointmatcher"
# https://github.com/ethz-asl/libpointmatcher/tree/master

# (CRITICAL) ToDo: va (ref task TASK)
if [[ ${BUILD_SYSTEM_INSTALL} == 'FALSE' ]]; then
  #git clone https://github.com/ethz-asl/libpointmatcher.git
  git clone https://github.com/"${LPM_LIBPOINTMATCHER_SRC_DOMAIN}"/"${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}".git
  #   && git checkout 1.3.1
fi

cd "${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}"
mkdir build && cd build

cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo \
  -D BUILD_TESTS=${BUILD_TESTS_FLAG} \
  -D GENERATE_API_DOC=${GENERATE_API_DOC_FLAG} \
  ..
#   -DCMAKE_INSTALL_PREFIX=/usr/local/ \

make -j $(nproc)
sudo make install

print_msg_done "Libpointmatcher installed"
print_formated_script_footer 'lpm_install_libpointmatcher_ubuntu.bash' =
# ====Teardown=====================================================================================================
cd "${TMP_CWD}"
