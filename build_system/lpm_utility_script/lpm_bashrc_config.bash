#!/bin/bash
#
# (Optional) .bashrc config script
#
# Usage:
#   $ bash lpm_bashrc_config.bash
#

function lpm::configure_bashrc() {
  local TMP_CWD
  TMP_CWD=$(pwd)

  # ....Project root logic.........................................................................
  LPM_PATH=$(git rev-parse --show-toplevel)

  # ....Load environment variables from file.......................................................
  cd "${LPM_PATH}/build_system" || exit
  set -o allexport
  source .env
  source .env.prompt
  set +o allexport

  # ....Helper function............................................................................
  ## import shell functions from build-system utilities library
  source ./function_library/prompt_utilities.bash

  # ====Begin======================================================================================
  print_formated_script_header 'lpm_bashrc_config.bash' "${NBS_LINE_CHAR_UTIL}"


  if [[ "$(basename ${LPM_PATH})" != "${NBS_REPOSITORY_NAME}" ]]; then
    print_msg_error_and_exit "Can't find directory ${MSG_DIMMED_FORMAT}${NBS_REPOSITORY_NAME}${MSG_END_FORMAT}"
  fi


  # ....Config bashrc..............................................................................
  # Add the following lines to .bashsrc if no alias prefixed with `lpm_` exist
  if [[ -z $(alias | grep -i lpm_) ]]; then
    (
      echo
      echo "# libpointmatcher build-system aliases"
      echo "export LPM_PATH=${LPM_PATH}"
      echo "alias lpm_cd='cd ${LPM_PATH}'"
      echo "alias lpmm_cd='cd ${LPM_PATH}/build_system'"
      echo
    ) >> ~/.bashrc && source ~/.bashrc
    #  echo "alias lpm_attach='cd $DN_PATH && bash dn_attach.bash'"
  fi

  print_msg_done "New aliases with prefix 'lpm' added to .bashrc"

  print_formated_script_footer 'lpm_bashrc_config.bash' "${NBS_LINE_CHAR_UTIL}"

  # ====Teardown===================================================================================
  cd "${TMP_CWD}"
}

# ::::main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
lpm::configure_bashrc
