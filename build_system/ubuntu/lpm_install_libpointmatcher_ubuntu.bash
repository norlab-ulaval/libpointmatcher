#!/bin/bash -i
#
# Install libpointmatcher
#
# Usage:
#   $ bash lpm_install_libpointmatcher_ubuntu.bash [<optional flag>]
#
# Arguments:
#   [--install-path </dir/abs/path/>]     The directory where to install libpointmatcher (absolute path)
#                                           (default location defined in the .env)
#   [--libpointmatcher-version v1.3.1]    Install libpointmatcher release tag version (default to master branch head)
#   [--compile-test]                      Compile the libpointmatcher unit-test
#   [--generate-doc]                      Generate the libpointmatcher doxygen documentation
#                                           in /usr/local/share/doc/libpointmatcher/api/html/index.html
#   [--build-system-CI-install]           Set special configuration for CI/CD build system:
#                                           skip the git clone install step and assume the repository is already
#                                           pulled and checkout on the desired branch
#   [-h, --help]                          Get help
#
# Note:
#   - this script required package: g++, make, cmake, build-essential, git and all libpointmatcher dependencies
#   - execute `lpm_install_dependencies_ubuntu.bash` first
#
set -e
#set -v

# ....Default......................................................................................................
LIBPOINTMATCHER_VERSION='head'
BUILD_TESTS_FLAG=FALSE
GENERATE_API_DOC_FLAG=FALSE
BUILD_SYSTEM_CI_INSTALL=FALSE

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

## skip GUI dialog by setting everything to default
#export DEBIAN_FRONTEND=noninteractive

# ....Helper function..............................................................................................
# import shell functions from Libpointmatcher-build-system utilities library
source ./function_library/prompt_utilities.bash

function print_help_in_terminal() {
  echo -e "\$ ${0} [<optional argument>]

  \033[1m<optional argument>:\033[0m
    --install-path </dir/abs/path/>       The directory where to install libpointmatcher (absolute path)
                                            (default location ${MSG_DIMMED_FORMAT}${LPM_INSTALLED_LIBRARIES_PATH:?'err LPM_INSTALLED_LIBRARIES_PATH env variable was not fetched from the .env'}${MSG_END_FORMAT})
    --libpointmatcher-version v1.3.1      Install libpointmatcher release tag version (default to master branch head)
    --compile-test                        Compile the libpointmatcher unit-test
                                            in ${MSG_DIMMED_FORMAT}${LPM_INSTALLED_LIBRARIES_PATH}/libpointmatcher/build${MSG_END_FORMAT}
    --generate-doc                        Generate the libpointmatcher doxygen documentation
                                            in ${MSG_DIMMED_FORMAT}/usr/local/share/doc/libpointmatcher/api/html/index.html${MSG_END_FORMAT}
    --build-system-CI-install             Set special configuration for CI/CD build system:
                                            skip the git clone install step and assume the repository is already
                                            pulled and checkout on the desired branch
    -h, --help                            Get help

  "
}

# ====Begin========================================================================================================
print_formated_script_header 'lpm_install_libpointmatcher_ubuntu.bash' "${LPM_LINE_CHAR_INSTALLER}"

# ....Script command line flags....................................................................................

while [ $# -gt 0 ]; do

    echo -e "'\$*' before: ${MSG_DIMMED_FORMAT}$*${MSG_END_FORMAT}" # ToDo: on task end >> delete this line ←
    echo -e "\$1: ${1}    \$2: $2" # ToDo: on task end >> delete this line ←
  #  echo -e "\$arg: ${arg}" # ToDo: on task end >> delete this line ←

  case $1 in
  # ToDo: unit-test
  --install-path)
    unset LPM_INSTALLED_LIBRARIES_PATH
    LPM_INSTALLED_LIBRARIES_PATH="${2}"
    shift # Remove argument (--install-path)
    shift # Remove argument value
    ;;
  # ToDo: unit-test
  --libpointmatcher-version)
    LIBPOINTMATCHER_VERSION="${2}"
    shift # Remove argument (--libpointmatcher-version)
    shift # Remove argument value
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
  --build-system-CI-install)
    BUILD_SYSTEM_CI_INSTALL=TRUE
    shift
    ;;
  -h | --help)
    print_help_in_terminal
    exit
    ;;
  --) # no more option
    shift
    break
    ;;
  --?* | -?*)
    echo "$0: $1: unrecognized option" >&2 # Note: '>&2' = print to stderr
    shift
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
# ToDo: on task end >> delete next bloc ↓↓
echo -e "${MSG_DIMMED_FORMAT}
LPM_INSTALLED_LIBRARIES_PATH=${LPM_INSTALLED_LIBRARIES_PATH}
BUILD_TESTS_FLAG=${BUILD_TESTS_FLAG}
GENERATE_API_DOC_FLAG=${GENERATE_API_DOC_FLAG}
BUILD_SYSTEM_CI_INSTALL=${BUILD_SYSTEM_CI_INSTALL}
${MSG_END_FORMAT}"
#
#echo "printenv" && printenv # ToDo: on task end >> delete this line ←

# ................................................................................................................
print_msg "Create required dir structure"

mkdir -p "${LPM_INSTALLED_LIBRARIES_PATH}"
cd "${LPM_INSTALLED_LIBRARIES_PATH}"

# ................................................................................................................
print_msg "Install Libpointmatcher"
# https://github.com/ethz-asl/libpointmatcher/tree/master

if [[ ${BUILD_SYSTEM_CI_INSTALL} == FALSE ]]; then

  if [[ -d ${LPM_LIBPOINTMATCHER_SRC_REPO_NAME} ]]; then
    print_msg_error_and_exit "The specified install directory ${MSG_DIMMED_FORMAT}${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}${MSG_END_FORMAT} at ${MSG_DIMMED_FORMAT}${LPM_INSTALLED_LIBRARIES_PATH}/${MSG_END_FORMAT} already exist, specify an other one using ${MSG_DIMMED_FORMAT}--install-path </install/dir/path/>${MSG_END_FORMAT}."
  fi

  # #git clone https://github.com/ethz-asl/libpointmatcher.git
  git clone https://github.com/"${LPM_LIBPOINTMATCHER_SRC_DOMAIN}"/"${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}".git

  # (CRITICAL) ToDo: validate (ref task NMO-252 ⚒︎ → Implement pull repo release tag version logic)
  if [[ "${LIBPOINTMATCHER_VERSION}" != 'head' ]]; then
    git checkout "${LIBPOINTMATCHER_VERSION}"
  fi
fi

cd "${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}"
REPO_ABS_PATH=$(pwd)
mkdir build && cd build

# (CRITICAL) ToDo: validate >> REPO_ABS_PATH
# (CRITICAL) ToDo: validate >> GENERATE_API_DOC install dir
cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo \
  -D BUILD_TESTS=${BUILD_TESTS_FLAG} \
  -D GENERATE_API_DOC=${GENERATE_API_DOC_FLAG} \
  ${REPO_ABS_PATH}

#   -DCMAKE_INSTALL_PREFIX=/usr/local/ \

make -j $(nproc)
sudo make install

print_msg_done "Libpointmatcher installed at ${MSG_DIMMED_FORMAT}${LPM_INSTALLED_LIBRARIES_PATH}/${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}${MSG_END_FORMAT}"
print_formated_script_footer 'lpm_install_libpointmatcher_ubuntu.bash' "${LPM_LINE_CHAR_INSTALLER}"
# ====Teardown=====================================================================================================
cd "${TMP_CWD}"
