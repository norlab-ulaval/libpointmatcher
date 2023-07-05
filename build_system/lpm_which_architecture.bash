#!/bin/bash
#
# Tools to set the LPM_IMAGE_ARCHITECTURE environment variable with the host architecture and OS type.
#
# LPM_IMAGE_ARCHITECTURE will be exported either as 'arm64-l4t', 'arm64-darwin', 'x86' or 'arm64'
# depending on which architecture and OS type the script is running:
#   - ARCH: aarch64, arm64, x86_64
#   - OS: Linux, Darwin, Window
#
# Usage;
#   $ bash lpm_which_architecture.bash
#
set -e

# ....Load environment variables from file.........................................................................
set -o allexport
source .env.prompt
set +o allexport

# ....Helper function..............................................................................................
# import shell functions from Libpointmatcher-build-system utilities library
source ./function_library/prompt_utilities.bash

# ====Begin========================================================================================================
print_formated_script_header 'lpm_which_architecture.bash' .

if [[ $(uname -m) == "aarch64" ]]; then
  if [[ -n $(uname -r | grep tegra) ]]; then
    export LPM_IMAGE_ARCHITECTURE='arm64-l4t'
  else
    echo -e "${LPM_MSG_ERROR} Unsupported OS for aarch64 processor"
  fi
elif [[ $(uname -m) == "arm64" ]]; then
  if [[ $(uname) == "Darwin" ]]; then
    export LPM_IMAGE_ARCHITECTURE='arm64-darwin'
  else
    export LPM_IMAGE_ARCHITECTURE='arm64' # ToDo: validate
  fi
elif [[ $(uname -m) == "x86_64" ]]; then
  export LPM_IMAGE_ARCHITECTURE='x86'
else
  print_msg_error_and_exit "Unsupported processor architecture"
fi

print_msg_done "Which LPM_IMAGE_ARCHITECTURE env? ${LPM_IMAGE_ARCHITECTURE}"
draw_horizontal_line_across_the_terminal_window .
# ====Done=========================================================================================================
