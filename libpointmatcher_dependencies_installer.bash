#!/bin/bash
# =================================================================================================
#
# Install all libpointmatcher dependencies
#
# Redirect the execution to libpointmatcher-build-system library
#
# Usage:
#   $ bash libpointmatcher_dependencies_installer.bash [--test-run]
#
# =================================================================================================
PARAMS="$@"

MSG_DIMMED_FORMAT="\033[1;2m"
MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"

function lpm::install_libpointmatcher_dependencies(){

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
  cd "${N2ST_PATH}" || exit 1
  source import_norlab_shell_script_tools_lib.bash

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
  export SHOW_SPLASH_IDU=false
  export SHOW_SPLASH_IDDU=false

  # ....Install general dependencies...............................................................
  cd "${LPM_PATH:?err}"/build_system/ubuntu || exit 1
  # shellcheck disable=SC2086
  source lpm_install_dependencies_general_ubuntu.bash ${PARAMS[@]}

  # ....Install libnabo............................................................................
  cd "${LPM_PATH:?err}"/build_system/ubuntu || exit 1

  # shellcheck disable=SC2086
  source ./lpm_install_dependencies_libnabo_ubuntu.bash ${PARAMS[@]}

  # ....Install documentation related dependencies.................................................
  cd "${LPM_PATH:?err}"/build_system/ubuntu || exit 1
  # shellcheck disable=SC2086
  source lpm_install_doc_dependencies_ubuntu.bash ${PARAMS[@]}

  n2st::print_msg_done "All Libpointmatcher dependencies installed"
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  lpm::install_libpointmatcher_dependencies
else
  # This script is being sourced, ie: __name__="__source__"
  echo -e "${MSG_ERROR_FORMAT}[LPM ERROR]${MSG_END_FORMAT} Execute this script in a subshell i.e.: $ bash libpointmatcher_dependencies_installer.bash" 1>&2
  exit 1
fi
