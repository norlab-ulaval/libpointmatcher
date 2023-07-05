#!/bin/bash
#
# (Optional) .bashrc config script
#
set -e

# ....Load environment variables from file.........................................................................
set -o allexport
source .env
source .env.prompt
set +o allexport

# ....Helper function..............................................................................................
# import shell functions from Libpointmatcher-build-system utilities library
source ./function_library/prompt_utilities.bash

# ....Project root logic...........................................................................................
TMP_CWD=$(pwd)

if [[ "$(basename $(pwd))" != "${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}" ]]; then
  cd ..
elif [[ "$(basename $(pwd))" == "${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}" ]]; then
  LPM_PATH=$(pwd)
else
  print_msg_error_and_exit "Can't find directory ${NTSI_MSG_DIMMED_FORMAT}${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}${NTSI_MSG_END_FORMAT}"
fi

# ====Begin========================================================================================================
print_formated_script_header 'lpm_bashrc_config.bash' .

# ....Config bashrc................................................................................................
# Add the following lines to .bashsrc if no alias prefixed with `lpm_` exist
if [[ -z $(alias | grep -i lpm_) ]]; then
  (
    echo
    echo "# libpointmatcher build-system aliases"
    echo "export LPM_PATH=${LPM_PATH}"
    echo "alias lpm_cd='cd $LPM_PATH'"
    echo "alias lpmm_cd='cd $LPM_PATH/build_system'"
    echo
  ) >> ~/.bashrc && source ~/.bashrc
  #  echo "alias lpm_attach='cd $DN_PATH && bash dn_attach.bash'"
fi

print_msg_done "New aliases with prefix 'lpm' added to .bashrc"

draw_horizontal_line_across_the_terminal_window .
# ====Teardown=====================================================================================================
cd "${TMP_CWD}"

