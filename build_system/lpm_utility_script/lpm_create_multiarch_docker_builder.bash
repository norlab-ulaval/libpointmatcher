#!/bin/bash
#
# Utility script to create a multi-architecture docker builder
#
# usage:
#   $ bash ./lpm_utility_script/lpm_create_multiarch_docker_builder.bash
#
function lpm::create_multiarch_docker_builder() {
  local TMP_CWD
  TMP_CWD=$(pwd)

  # ....Project root logic.........................................................................
  LPM_PATH=$(git rev-parse --show-toplevel)

  # ====Begin=====================================================================================
  cd "${LPM_PATH}"/build_system/utilities/norlab-build-system/install_scripts/ \
    && bash nbs_create_multiarch_docker_builder.bash

  # ====Teardown===================================================================================
  cd "${TMP_CWD}"
}

# ::::main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
lpm::create_multiarch_docker_builder
