#!/bin/bash
# =================================================================================================
#
# Install libpointmatcher
#
# Usage:
#   $ bash libpointmatcher_installer.bash [<optional flag>]
#
# Arguments:
#   --install-path </dir/abs/path/>     The directory where to install libpointmatcher (absolute path)
#                                           (default location defined in the .env)
#   --repository-version 1.4.0         Install libpointmatcher release tag version (default to master branch latest)
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
# Note:
#   - this script required package: g++, make, cmake, build-essential, git and all libpointmatcher dependencies
#   - execute `libpointmatcher_dependencies_installer.bash` first
#
# =================================================================================================
PARAMS="$@"

MSG_DIMMED_FORMAT="\033[1;2m"
MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"

function lpm::install_libpointmatcher(){

  # ....path resolution logic......................................................................
  LPM_ROOT="$(dirname "$(realpath "$0")")"

  cd "${LPM_ROOT}" || exit 1

  # ....Load environment variables from file.......................................................
  # . . Source LPM environment variables  . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  if [[ -f .env.libpointmatcher ]]; then
   set -o allexport && source .env.libpointmatcher && set +o allexport
  else
   echo -e "${MSG_ERROR_FORMAT}[LPM ERROR]${MSG_END_FORMAT} .env.libpointmatcher unreachable. Cwd $(pwd)" 1>&2
   exit 1
  fi

  # . . Source NBS dependencies . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cd "${NBS_PATH}" || exit 1
  source import_norlab_build_system_lib.bash

  # . . Source LPM-build-system environment variables . . . . . . . . . . . . . . . . . . . . . . .
  cd "${LPM_BUILD_SYSTEM_PATH}" || exit 1
  if [[ -f .env ]]; then
   set -o allexport && source .env && set +o allexport
  else
   echo -e "${MSG_ERROR_FORMAT}[LPM ERROR]${MSG_END_FORMAT} .env unreachable. Cwd $(pwd)" 1>&2
   exit 1
  fi

  # ====Begin======================================================================================
  n2st::norlab_splash "${NBS_SPLASH_NAME}" "https://github.com/${NBS_REPOSITORY_DOMAIN:?err}/${NBS_REPOSITORY_NAME:?err}"
  export SHOW_SPLASH_ILU=false

  # ....Install general dependencies...............................................................
  cd "${LPM_PATH:?err}"/build_system/ubuntu || exit 1

  # shellcheck disable=SC2068
  source lpm_install_libpointmatcher_ubuntu.bash ${PARAMS[@]}

  n2st::print_msg_done "Libpointmatcher install script completed. Have fun"
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  lpm::install_libpointmatcher
else
  # This script is being sourced, ie: __name__="__source__"
  echo -e "${MSG_ERROR_FORMAT}[LPM ERROR]${MSG_END_FORMAT} Execute this script in a subshell i.e.: $ bash libpointmatcher_installer.bash" 1>&2
  exit 1
fi
