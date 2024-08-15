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
  cd "${LPM_PATH}/build_system" || exit 1
  set -o allexport
  source .env
  set +o allexport

  # ....Helper function............................................................................
  # import shell functions from utilities library
  N2ST_PATH=${N2ST_PATH:-"${LPM_PATH}/build_system/utilities/norlab-shell-script-tools"}
  source "${N2ST_PATH}/import_norlab_shell_script_tools_lib.bash"

  # ====Begin======================================================================================
  n2st::print_formated_script_header 'lpm_bashrc_config.bash' "${MSG_LINE_CHAR_UTIL}"


  if [[ "$(basename ${LPM_PATH})" != "${NBS_REPOSITORY_NAME}" ]]; then
    n2st::print_msg_error_and_exit "Can't find directory ${MSG_DIMMED_FORMAT}${NBS_REPOSITORY_NAME}${MSG_END_FORMAT}"
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

  n2st::print_msg_done "New aliases with prefix 'lpm' added to .bashrc"

  n2st::print_formated_script_footer 'lpm_bashrc_config.bash' "${MSG_LINE_CHAR_UTIL}"

  # ====Teardown===================================================================================
  cd "${TMP_CWD}"
}

# ::::main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
lpm::configure_bashrc
