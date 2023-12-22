#!/bin/bash -i
# =================================================================================================
#
# Install libpointmatcher
#
# Usage:
#   $ bash lpm_install_libpointmatcher_ubuntu.bash [<optional flag>]
#
# Arguments:
#   --install-path </dir/abs/path/>     The directory where to install libpointmatcher (absolute path)
#                                           (default location defined in the .env)
#   --repository-version v1.3.1         Install libpointmatcher release tag version (default to master branch latest)
#   --compile-test                      Compile the libpointmatcher unit-test
#   --generate-doc                      Generate the libpointmatcher doxygen documentation
#                                           in /usr/local/share/doc/libpointmatcher/api/html/index.html
#   --cmake-build-type RelWithDebInfo   The type of cmake build: None Debug Release RelWithDebInfo MinSizeRel
#                                           (default to RelWithDebInfo)
#   --build-system-CI-install           Set special configuration for CI/CD build system:
#                                           skip the git clone install step and assume the repository is already
#                                           pulled and checkout on the desired branch
#   --test-run                          CI/CD build system Test-run mode
#   -h, --help                          Get help
#
# Global
#   - Read the array APPEND_TO_CMAKE_FLAG
#
#     Usage:
#      $ export APPEND_TO_CMAKE_FLAG=( -D CMAKE_INSTALL_PREFIX=/opt ) \
#           && source lpm_install_libpointmatcher_ubuntu.bash
#
# Note:
#   - this script required package: g++, make, cmake, build-essential, git and all libpointmatcher dependencies
#   - execute `lpm_install_dependencies_general_ubuntu.bash` first
#
# =================================================================================================
set -e # Note: we want the installer to always fail-fast (it wont affect the build system policy)

declare -a CMAKE_FLAGS

CALLER_NAME="$(basename "$0" )"

# ....Default......................................................................................
REPOSITORY_VERSION='latest'
BUILD_TESTS_FLAG=FALSE
GENERATE_API_DOC_FLAG=FALSE
BUILD_SYSTEM_CI_INSTALL=FALSE
CMAKE_BUILD_TYPE=RelWithDebInfo

# skip GUI dialog by setting everything to default
export DEBIAN_FRONTEND=noninteractive

# ....Project root logic...........................................................................
TMP_CWD=$(pwd)

LPM_PATH=$(git rev-parse --show-toplevel)
cd "${LPM_PATH}/build_system" || exit

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

function print_help_in_terminal() {
  echo -e "\$ ${0} [<optional argument>]

  \033[1m<optional argument>:\033[0m
    --install-path </dir/abs/path/>       The directory where to install (absolute path)
                                            (default location ${MSG_DIMMED_FORMAT}${NBS_LIB_INSTALL_PATH:?err}${MSG_END_FORMAT})
    --repository-version v1.3.1           Install release tag version (default to master branch latest)
    --compile-test                        Compile the unit-test
                                            in ${MSG_DIMMED_FORMAT}${NBS_LIB_INSTALL_PATH}/${NBS_REPOSITORY_NAME:?err}/build${MSG_END_FORMAT}
    --generate-doc                        Generate the libpointmatcher doxygen documentation
                                            in ${MSG_DIMMED_FORMAT}/usr/local/share/doc/${NBS_REPOSITORY_NAME}/api/html/index.html${MSG_END_FORMAT}
    --cmake-build-type RelWithDebInfo     The type of cmake build: None Debug Release RelWithDebInfo MinSizeRel
    --build-system-CI-install             Set special configuration for CI/CD build system:
                                            skip the git clone install step and assume the repository is already
                                            pulled and checkout on the desired branch
    --test-run                            CI/CD build system Test-run mode
    -h, --help                            Get help

  Global
    - Read the array APPEND_TO_CMAKE_FLAG

      Usage:
       $ export APPEND_TO_CMAKE_FLAG=( -D CMAKE_INSTALL_PREFIX=/opt ) && source lpm_install_libpointmatcher_ubuntu.bash

  "
}

# ====Begin========================================================================================
SHOW_SPLASH_ILU="${SHOW_SPLASH_ILU:-true}"

if [[ "${SHOW_SPLASH_ILU}" == 'true' ]]; then
  norlab_splash "${NBS_SPLASH_NAME:?err}" "https://github.com/${NBS_REPOSITORY_DOMAIN:?err}/${NBS_REPOSITORY_NAME:?err}"
fi

print_formated_script_header "${CALLER_NAME} (${IMAGE_ARCH_AND_OS:?err})" "${MSG_LINE_CHAR_INSTALLER}"


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
  --test-run)
    TEST_RUN=true
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


# ....Cmake flags..................................................................................
CMAKE_FLAGS=( -D CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" -D BUILD_TESTS="${BUILD_TESTS_FLAG}" -D GENERATE_API_DOC="${GENERATE_API_DOC_FLAG}" "${APPEND_TO_CMAKE_FLAG[@]}" )

# (CRITICAL) ToDo: validate >> GENERATE_API_DOC install dir

# Note:
#   - Previously use intall flag quick-hack to work around the install issue.
#   - Keep it here as futur reference
#  -D LIBNABO_INSTALL_DIR="${NBS_LIB_INSTALL_PATH}/libnabo" \

# ----Install steps--------------------------------------------------------------------------------
teamcity_service_msg_blockOpened "Install ${NBS_REPOSITORY_NAME}"

mkdir -p "${NBS_LIB_INSTALL_PATH}"
print_msg "Directories (pre libpointmatcher install)$(tree -L 2 "${NBS_LIB_INSTALL_PATH}")"
cd "${NBS_LIB_INSTALL_PATH}"

# ....Repository cloning step......................................................................
if [[ ${BUILD_SYSTEM_CI_INSTALL} == FALSE ]]; then

  if [[ -d ${NBS_REPOSITORY_NAME}/.git ]]; then
    print_msg_error_and_exit "${MSG_DIMMED_FORMAT}${NBS_REPOSITORY_NAME}${MSG_END_FORMAT} source code was already checkout in the specified install directory ${MSG_DIMMED_FORMAT}${NBS_LIB_INSTALL_PATH}/${MSG_END_FORMAT}, specify an other one using ${MSG_DIMMED_FORMAT}--install-path </install/dir/path/>${MSG_END_FORMAT}."
  fi

  git clone https://github.com/"${NBS_REPOSITORY_DOMAIN}"/"${NBS_REPOSITORY_NAME}".git

  if [[ "${REPOSITORY_VERSION}" != 'latest' ]]; then
    cd "${NBS_REPOSITORY_NAME}"

    git fetch --tags
    git tag --list

    # Remove prefix 'v' from version tag
    GITHUB_TAG="${REPOSITORY_VERSION/v/}"

    git checkout tags/"${GITHUB_TAG}"

    print_msg "Repository checkout at tag $(git symbolic-ref -q --short HEAD || git describe --tags --exact-match)"
  fi
fi

cd "${NBS_LIB_INSTALL_PATH}/${NBS_REPOSITORY_NAME}"
mkdir -p build && cd build

# ....Cmake install step...........................................................................
teamcity_service_msg_compilationStarted "cmake"

print_msg "Execute ${MSG_DIMMED_FORMAT}
cmake ${CMAKE_FLAGS[*]} ${NBS_LIB_INSTALL_PATH}/${NBS_REPOSITORY_NAME}
${MSG_END_FORMAT}"

if [[ $TEST_RUN  == true ]]; then
  print_msg "Test-run mode: Skipping cmake"
  BUILD_EXIT_CODE=0
  INSTALL_EXIT_CODE=0
else
  cmake "${CMAKE_FLAGS[@]}" "${NBS_LIB_INSTALL_PATH}/${NBS_REPOSITORY_NAME}"

  BUILD_EXIT_CODE=$?

  make -j $(nproc)
  sudo make install

  INSTALL_EXIT_CODE=$?

  ## List all CMake build options and their default values
  ##   ref: https://stackoverflow.com/questions/16851084/how-to-list-all-cmake-build-options-and-their-default-values
  #cmake -LAH
fi

print_msg "Directories (post ${NBS_REPOSITORY_NAME} install)$(tree -L 2 ${NBS_LIB_INSTALL_PATH})"

teamcity_service_msg_compilationFinished
teamcity_service_msg_blockClosed

# ....Install/build feedback.......................................................................
SUCCESS_MSG="${NBS_REPOSITORY_NAME} installed successfully at ${MSG_DIMMED_FORMAT}${NBS_LIB_INSTALL_PATH}/${NBS_REPOSITORY_NAME}${MSG_END_FORMAT}"
FAILURE_MSG="${NBS_REPOSITORY_NAME} installer exited with error"

if [[ ${IS_TEAMCITY_RUN} == true ]]; then
  # Report message to build log
  if [[ ${BUILD_EXIT_CODE} == 0 ]] && [[ ${INSTALL_EXIT_CODE} == 0 ]]; then
    echo -e "##teamcity[message text='${MSG_BASE_TEAMCITY} ${SUCCESS_MSG}' status='NORMAL']"
  else
    if [[ ${BUILD_EXIT_CODE} != 0 ]]; then
      echo -e "##teamcity[message text='${MSG_BASE_TEAMCITY} ${FAILURE_MSG}' errorDetails='$BUILD_EXIT_CODE' status='ERROR']"
      print_formated_script_footer "${CALLER_NAME} (${IMAGE_ARCH_AND_OS})" "${MSG_LINE_CHAR_INSTALLER}"
      exit $BUILD_EXIT_CODE
    else
      echo -e "##teamcity[message text='${MSG_BASE_TEAMCITY} ${FAILURE_MSG}' errorDetails='$INSTALL_EXIT_CODE' status='ERROR']"
      print_formated_script_footer "${CALLER_NAME} (${IMAGE_ARCH_AND_OS})" "${MSG_LINE_CHAR_INSTALLER}"
      exit $INSTALL_EXIT_CODE
    fi
  fi
else
  if [[ ${BUILD_EXIT_CODE} == 0 ]] && [[ ${INSTALL_EXIT_CODE} == 0 ]]; then
    echo " " && print_msg_done "${SUCCESS_MSG}"
  else
    print_msg_error "${FAILURE_MSG}"
    print_formated_script_footer "${CALLER_NAME} (${IMAGE_ARCH_AND_OS})" "${MSG_LINE_CHAR_INSTALLER}"
    if [[ ${BUILD_EXIT_CODE} != 0 ]]; then
      exit $BUILD_EXIT_CODE
    else
      exit $INSTALL_EXIT_CODE
    fi
  fi
fi

print_formated_script_footer "${CALLER_NAME} (${IMAGE_ARCH_AND_OS})" "${MSG_LINE_CHAR_INSTALLER}"

# ====Teardown=====================================================================================
cd "${TMP_CWD}"
