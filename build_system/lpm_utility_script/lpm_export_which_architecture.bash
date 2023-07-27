#!/bin/bash
#
# Tools to export the LPM_IMAGE_ARCHITECTURE environment variable with the host architecture and OS type.
#
# LPM_IMAGE_ARCHITECTURE will be exported either as 'arm64-l4t', 'arm64-darwin', 'arm64-linux' or 'x86-linux'
# depending on which architecture and OS type the script is running:
#   - ARCH: aarch64, arm64, x86_64
#   - OS: Linux, Darwin, Window
#
# Usage;
#   $ bash ./lpm_utility_script/lpm_export_which_architecture.bash
#
set -e

if [[ "$(basename $(pwd))" != "build_system" ]]; then
  cd ../
fi

# ....Load environment variables from file.........................................................................
set -o allexport
source .env.prompt
set +o allexport

# ....Helper function..............................................................................................
# import shell functions from Libpointmatcher-build-system utilities library
source ./function_library/prompt_utilities.bash

# ====Begin========================================================================================================
print_formated_script_header 'lpm_export_which_architecture.bash' "${LPM_LINE_CHAR_UTIL}"


if [[ $(uname -m) == "aarch64" ]]; then
  if [[ -n $(uname -r | grep tegra) ]]; then
    export LPM_IMAGE_ARCHITECTURE='arm64-l4t'
  elif [[ $(uname) == "Linux" ]]; then
      export LPM_IMAGE_ARCHITECTURE='arm64-linux'
  else
    echo -e "${MSG_ERROR} Unsupported OS for aarch64 processor"
  fi
elif [[ $(uname -m) == "arm64" ]] && [[ $(uname) == "Darwin" ]]; then
  export LPM_IMAGE_ARCHITECTURE='arm64-darwin'
elif [[ $(uname -m) == "x86_64" ]] && [[ $(uname) == "Linux" ]]; then
  export LPM_IMAGE_ARCHITECTURE='x86-linux'
else
  print_msg_error_and_exit "Unsupported processor architecture"
fi

print_msg "Setting LPM_IMAGE_ARCHITECTURE=${LPM_IMAGE_ARCHITECTURE}"
print_formated_script_footer 'lpm_export_which_architecture.bash' "${LPM_LINE_CHAR_UTIL}"
# ====Done=========================================================================================================
