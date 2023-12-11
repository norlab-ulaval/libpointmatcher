#!/bin/bash
#
# Tools to export the NBS_IMAGE_ARCHITECTURE environment variable with the host architecture and OS type.
#
# Usage:
#   $ source ./lpm_utility_script/lpm_export_which_architecture.bash
#
# Globals:
#   Read NBS_LINE_CHAR_UTIL
# Arguments:
#   none
# Outputs:
#   environment variable NBS_IMAGE_ARCHITECTURE will be exported either as
#       - NBS_IMAGE_ARCHITECTURE=x86-linux
#       - NBS_IMAGE_ARCHITECTURE=arm64-linux
#       - NBS_IMAGE_ARCHITECTURE=arm64-l4t
#       - NBS_IMAGE_ARCHITECTURE=arm64-darwin
#   depending on which architecture and OS type the script is running:
#     - ARCH: aarch64, arm64, x86_64
#     - OS: Linux, Darwin, Window
#
# (NICE TO HAVE) ToDo: assessment >> check the convention used by docker >> os[/arch[/variant]]
#       linux/arm64/v8
#       darwin/arm64/v8
#       l4t/arm64/v8
#     ref: https://docs.docker.com/compose/compose-file/05-services/#platform
#
# Returns:
#   exit 1 in case of unsupported processor architecture

function lpm::export_which_architecture() {
  local TMP_CWD
  TMP_CWD=$(pwd)

  # ....Project root logic.........................................................................
  LPM_PATH=$(git rev-parse --show-toplevel)
  cd "${LPM_PATH}/build_system" || exit

  # ....Helper function..............................................................................................
  # import shell functions from utilities library
  N2ST_PATH=${N2ST_PATH:-"${LPM_PATH}/build_system/utilities/norlab-shell-script-tools"}
  source "${N2ST_PATH}/import_norlab_shell_script_tools_lib.bash"

  # ====Begin======================================================================================
  #print_formated_script_header 'lpm_export_which_architecture.bash' "${NBS_LINE_CHAR_UTIL}"

  if [[ $(uname -m) == "aarch64" ]]; then
    if [[ -n $(uname -r | grep tegra) ]]; then
      export NBS_IMAGE_ARCHITECTURE='arm64-l4t'
    elif [[ $(uname) == "Linux" ]]; then
        export NBS_IMAGE_ARCHITECTURE='arm64-linux'
    else
      echo -e "${MSG_ERROR} Unsupported OS for aarch64 processor"
    fi
  elif [[ $(uname -m) == "arm64" ]] && [[ $(uname) == "Darwin" ]]; then
    export NBS_IMAGE_ARCHITECTURE='arm64-darwin'
  elif [[ $(uname -m) == "x86_64" ]] && [[ $(uname) == "Linux" ]]; then
    export NBS_IMAGE_ARCHITECTURE='x86-linux'
  else
    print_msg_error_and_exit "Unsupported processor architecture"
  fi

  print_msg "Setting NBS_IMAGE_ARCHITECTURE=${NBS_IMAGE_ARCHITECTURE}"
  #print_formated_script_footer 'lpm_export_which_architecture.bash' "${NBS_LINE_CHAR_UTIL}"
  # ====Teardown===================================================================================
  cd "${TMP_CWD}"
}

# ::::main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
lpm::export_which_architecture
