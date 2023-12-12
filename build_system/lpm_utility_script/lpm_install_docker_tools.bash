#!/bin/bash
#
# Utility script to install docker related tools and execute basic configuration
#
# usage:
#   $ bash ./lpm_utility_script/lpm_install_docker_tools.bash
#

function lpm::install_docker_tools() {
  local TMP_CWD
  TMP_CWD=$(pwd)

  # ....Project root logic.........................................................................
  LPM_PATH=$(git rev-parse --show-toplevel)

  # ....Load environment variables from file.......................................................
  cd "${LPM_PATH}/build_system" || exit
  set -o allexport
  source .env
  set +o allexport

  # ....Helper function............................................................................
  # import shell functions from utilities library
  N2ST_PATH=${N2ST_PATH:-"${LPM_PATH}/build_system/utilities/norlab-shell-script-tools"}
  source "${N2ST_PATH}/import_norlab_shell_script_tools_lib.bash"

#  # ====Begin=====================================================================================
  cd "${N2ST_PATH}"/src/utility_scripts/ && bash install_docker_tools.bash

  # ====Teardown===================================================================================
  cd "${TMP_CWD}"
}

# ::::main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
lpm::install_docker_tools
