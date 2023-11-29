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
#   [--repository-version v1.3.1]         Install libpointmatcher release tag version (default to master branch latest)
#   [--compile-test]                      Compile the libpointmatcher unit-test
#   [--generate-doc]                      Generate the libpointmatcher doxygen documentation
#                                           in /usr/local/share/doc/libpointmatcher/api/html/index.html
#   [--build-system-CI-install]           Set special configuration for CI/CD build system:
#                                           skip the git clone install step and assume the repository is already
#                                           pulled and checkout on the desired branch
#   [--cmake-build-type RelWithDebInfo]         The type of cmake build: None Debug Release RelWithDebInfo MinSizeRel
#                                           (default to RelWithDebInfo)
#   [-h, --help]                          Get help
#
# Global
#   - Read OVERRIDE_NBS_CMAKE_INSTALL_PREFIX
#     Usage:
#       export OVERRIDE_NBS_CMAKE_INSTALL_PREFIX=( "-D CMAKE_INSTALL_PREFIX=/opt" ) \
#           && source lpm_install_libpointmatcher_ubuntu.bash
#
# Note:
#   - this script required package: g++, make, cmake, build-essential, git and all libpointmatcher dependencies
#   - execute `lpm_install_dependencies_general_ubuntu.bash` first
#
set -e # Note: we want the installer to always fail-fast (it wont affect the build system policy)

# ....Default......................................................................................
REPOSITORY_VERSION='latest'
BUILD_TESTS_FLAG=FALSE
GENERATE_API_DOC_FLAG=FALSE
BUILD_SYSTEM_CI_INSTALL=FALSE
CMAKE_BUILD_TYPE=RelWithDebInfo


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

## skip GUI dialog by setting everything to default
export DEBIAN_FRONTEND=noninteractive

# ....Helper function..............................................................................
## import shell functions from norlab-build-system utilities library
source ./function_library/prompt_utilities.bash
source ./function_library/terminal_splash.bash
source ./function_library/general_utilities.bash

## Set environment variable NBS_IMAGE_ARCHITECTURE
source ./lpm_utility_script/lpm_export_which_architecture.bash

function print_help_in_terminal() {
  echo -e "\$ ${0} [<optional argument>]

  \033[1m<optional argument>:\033[0m
    --install-path </dir/abs/path/>       The directory where to install (absolute path)
                                            (default location ${MSG_DIMMED_FORMAT}${NBS_LIB_INSTALL_PATH:?'err NBS_LIB_INSTALL_PATH env variable was not fetched from the .env'}${MSG_END_FORMAT})
    --repository-version v1.3.1           Install release tag version (default to master branch latest)
    --compile-test                        Compile the unit-test
                                            in ${MSG_DIMMED_FORMAT}${NBS_LIB_INSTALL_PATH}/${NBS_REPOSITORY_NAME}/build${MSG_END_FORMAT}
    --generate-doc                        Generate the libpointmatcher doxygen documentation
                                            in ${MSG_DIMMED_FORMAT}/usr/local/share/doc/${NBS_REPOSITORY_NAME}/api/html/index.html${MSG_END_FORMAT}
    --build-system-CI-install             Set special configuration for CI/CD build system:
                                            skip the git clone install step and assume the repository is already
                                            pulled and checkout on the desired branch
    --cmake-build-type RelWithDebInfo           The type of cmake build: None Debug Release RelWithDebInfo MinSizeRel
    -h, --help                            Get help

  "
}

# ====Begin========================================================================================
SHOW_SPLASH_ILU="${SHOW_SPLASH_ILU:-true}"

if [[ "${SHOW_SPLASH_ILU}" == 'true' ]]; then
  norlab_splash "${NBS_SPLASH_NAME}" "https://github.com/${NBS_REPOSITORY_DOMAIN}/${NBS_REPOSITORY_NAME}"
fi

print_formated_script_header "lpm_install_libpointmatcher_ubuntu.bash (${NBS_IMAGE_ARCHITECTURE})" "${NBS_LINE_CHAR_INSTALLER}"


# ....Script command line flags....................................................................

while [ $# -gt 0 ]; do

  case $1 in
  --install-path)
    unset NBS_LIB_INSTALL_PATH
    NBS_LIB_INSTALL_PATH="${2}"
    shift # Remove argument (--install-path)
    shift # Remove argument value
    ;;
  --repository-version)
    REPOSITORY_VERSION="${2}"
    shift # Remove argument (--repository-version)
    shift # Remove argument value
    ;;
  --cmake-build-type)
    CMAKE_BUILD_TYPE="${2}"
    shift # Remove argument (--cmake-build-type)
    shift # Remove argument value
    ;;
  --compile-test)
    BUILD_TESTS_FLAG=TRUE
    shift
    ;;
  --generate-doc)
    GENERATE_API_DOC_FLAG=TRUE
    shift
    ;;
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

done


# ....Override.....................................................................................
declare -ar DEFAULT_CMAKE_INSTALL_PREFIX=( "-D CMAKE_INSTALL_PREFIX=${NBS_LIB_INSTALL_PATH:?err}" )
declare -a OVERRIDE_NBS_CMAKE_INSTALL_PREFIX
declare -a NBS_CMAKE_INSTALL_PREFIX=( "${OVERRIDE_NBS_CMAKE_INSTALL_PREFIX[@]:-${DEFAULT_CMAKE_INSTALL_PREFIX[@]}}" )


# .................................................................................................
teamcity_service_msg_blockOpened "Install ${NBS_REPOSITORY_NAME}"

print_msg "Directories (pre libpointmatcher install)$(tree -L 2 ${NBS_LIB_INSTALL_PATH})"

mkdir -p "${NBS_LIB_INSTALL_PATH}"
cd "${NBS_LIB_INSTALL_PATH}"

if [[ ${BUILD_SYSTEM_CI_INSTALL} == FALSE ]]; then

  if [[ -d ${NBS_REPOSITORY_NAME}/pointmatcher ]]; then
    print_msg_error_and_exit "${MSG_DIMMED_FORMAT}${NBS_REPOSITORY_NAME}${MSG_END_FORMAT} source code was already checkout in the specified install directory ${MSG_DIMMED_FORMAT}${NBS_LIB_INSTALL_PATH}/${MSG_END_FORMAT}, specify an other one using ${MSG_DIMMED_FORMAT}--install-path </install/dir/path/>${MSG_END_FORMAT}."
  fi

  git clone https://github.com/"${NBS_REPOSITORY_DOMAIN}"/"${NBS_REPOSITORY_NAME}".git

  if [[ "${REPOSITORY_VERSION}" != 'latest' ]]; then
    cd "${NBS_REPOSITORY_NAME}"/

    git fetch --tags
    git tag --list

    # Remove prefix 'v' from version tag
    GITHUB_TAG="${REPOSITORY_VERSION/v/}"
    print_msg "GITHUB_TAG=${GITHUB_TAG}"

    git checkout tags/"${GITHUB_TAG}"
  fi
fi

cd "${NBS_REPOSITORY_NAME}"/
mkdir -p build && cd build

teamcity_service_msg_compilationStarted "cmake"

# (CRITICAL) ToDo: validate >> GENERATE_API_DOC install dir
cmake -D CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
  -D BUILD_TESTS=${BUILD_TESTS_FLAG} \
  -D GENERATE_API_DOC=${GENERATE_API_DOC_FLAG} \
  "${NBS_CMAKE_INSTALL_PREFIX[@]}" \
  "${NBS_LIB_INSTALL_PATH}/${NBS_REPOSITORY_NAME}"

# Note:
#   - Previously use intall flag quick-hack to work around the install issue.
#   - Keep it here as futur reference
#  -D LIBNABO_INSTALL_DIR="${NBS_LIB_INSTALL_PATH}/libnabo" \

BUILD_EXIT_CODE=$?

make -j $(nproc)
sudo make install

INSTALL_EXIT_CODE=$?

### List all CMake build options and their default values
###   ref: https://stackoverflow.com/questions/16851084/how-to-list-all-cmake-build-options-and-their-default-values
#cmake -LAH

print_msg "Directories (post ${NBS_REPOSITORY_NAME} install)$(tree -L 2 ${NBS_LIB_INSTALL_PATH})"

teamcity_service_msg_compilationFinished
teamcity_service_msg_blockClosed

SUCCESS_MSG="${NBS_REPOSITORY_NAME} installed successfully at ${MSG_DIMMED_FORMAT}${NBS_LIB_INSTALL_PATH}/${NBS_REPOSITORY_NAME}${MSG_END_FORMAT}"
FAILURE_MSG="${NBS_REPOSITORY_NAME} installer exited with error"

if [[ ${IS_TEAMCITY_RUN} == true ]]; then
  # Report message to build log
  if [[ ${BUILD_EXIT_CODE} == 0 ]] && [[ ${INSTALL_EXIT_CODE} == 0 ]]; then
    echo -e "##teamcity[message text='${MSG_BASE_TEAMCITY} ${SUCCESS_MSG}' status='NORMAL']"
  else
    if [[ ${BUILD_EXIT_CODE} != 0 ]]; then
      echo -e "##teamcity[message text='${MSG_BASE_TEAMCITY} ${FAILURE_MSG}' errorDetails='$BUILD_EXIT_CODE' status='ERROR']"
      print_formated_script_footer "lpm_install_libpointmatcher_ubuntu.bash (${NBS_IMAGE_ARCHITECTURE})" "${NBS_LINE_CHAR_INSTALLER}"
      exit $BUILD_EXIT_CODE
    else
      echo -e "##teamcity[message text='${MSG_BASE_TEAMCITY} ${FAILURE_MSG}' errorDetails='$INSTALL_EXIT_CODE' status='ERROR']"
      print_formated_script_footer "lpm_install_libpointmatcher_ubuntu.bash (${NBS_IMAGE_ARCHITECTURE})" "${NBS_LINE_CHAR_INSTALLER}"
      exit $INSTALL_EXIT_CODE
    fi
  fi
else
  if [[ ${BUILD_EXIT_CODE} == 0 ]] && [[ ${INSTALL_EXIT_CODE} == 0 ]]; then
    echo " " && print_msg_done "${SUCCESS_MSG}"
  else
    print_msg_error "${FAILURE_MSG}"
    print_formated_script_footer "lpm_install_libpointmatcher_ubuntu.bash (${NBS_IMAGE_ARCHITECTURE})" "${NBS_LINE_CHAR_INSTALLER}"
    if [[ ${BUILD_EXIT_CODE} != 0 ]]; then
      exit $BUILD_EXIT_CODE
    else
      exit $INSTALL_EXIT_CODE
    fi
  fi
fi

print_formated_script_footer "lpm_install_libpointmatcher_ubuntu.bash (${NBS_IMAGE_ARCHITECTURE})" "${NBS_LINE_CHAR_INSTALLER}"
# ====Teardown=====================================================================================
cd "${TMP_CWD}"
