#!/bin/bash -i
#
# Usage:
#   $ bash entrypoint_execute_lpm_unittest.bash [<any>]
#
# Parameter
#   <any> (Optional) Everything passed here will be executed at the end of this script
#
#set -e

# ====Build system tools===========================================================================================
cd "${LPM_INSTALLED_LIBRARIES_PATH}/${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}/build_system"

# ....Load environment variables from file.........................................................................
set -o allexport
source .env
set +o allexport

# ....Helper function..............................................................................................
## import shell functions from Libpointmatcher-build-system utilities library
source ./function_library/prompt_utilities.bash

# ==== Check libopintmatcher dependencies versions=================================================================
cd "${LPM_INSTALLED_LIBRARIES_PATH}/${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}"
sudo chmod +x ./utest/listVersionsUbuntu.sh
utest/listVersionsUbuntu.sh

# ==== Execute libpointmatcher unit-test===========================================================================
# .................................................................................................................
cd "${LPM_INSTALLED_LIBRARIES_PATH}/${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}/build"

if [[ -d ./utest ]]; then
  cd "${LPM_INSTALLED_LIBRARIES_PATH}/${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}/build_system"
  source entrypoint_execute_lpm_unittest.bash
else
  print_msg_warning "Directory ${MSG_DIMMED_FORMAT}utest/${MSG_END_FORMAT} was not created during compilation.
  Skipping Libpointmatcher unit-test."
fi



# ====Continue=====================================================================================================
#exit "$(echo $?)"
exec "$@"
