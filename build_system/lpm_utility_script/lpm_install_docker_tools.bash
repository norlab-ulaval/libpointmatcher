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

  # ====Begin=====================================================================================
  cd "${LPM_PATH}"/build_system/utilities/norlab-build-system/install_scripts/ \
    && bash nbs_install_docker_tools.bash

  # ====Teardown===================================================================================
  cd "${TMP_CWD}"
}

# ::::main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
lpm::install_docker_tools
