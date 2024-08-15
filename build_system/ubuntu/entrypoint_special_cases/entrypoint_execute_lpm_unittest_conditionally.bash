#!/bin/bash -i
#
# Docker entrypoint for running libpointmatcher unit-test
#
# Usage:
#   $ bash entrypoint_execute_lpm_unittest_conditionally.bash [<any>]
#
# Parameter
#   <any> (Optional) Everything passed here will be executed at the end of this script
#
#set -e

# ====Build system tools===========================================================================
cd "${NBS_LIB_INSTALL_PATH}/${NBS_REPOSITORY_NAME}/build_system"

# ....Load environment variables from file.........................................................
set -o allexport
source .env
set +o allexport

# ....Helper function..............................................................................
# import shell functions from utilities library
source "./utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"

# ==== Check libopintmatcher dependencies versions=================================================
cd "${NBS_LIB_INSTALL_PATH}/${NBS_REPOSITORY_NAME}"
sudo chmod +x ./utest/listVersionsUbuntu.sh
utest/listVersionsUbuntu.sh

# ==== Execute libpointmatcher unit-test===========================================================
# .................................................................................................
cd "${NBS_LIB_INSTALL_PATH}/${NBS_REPOSITORY_NAME}/build"

if [[ -d ./utest ]]; then
  source "${NBS_LIB_INSTALL_PATH}/${NBS_REPOSITORY_NAME}/build_system/ubuntu/lpm_execute_libpointmatcher_unittest.bash"
else
  n2st::print_msg_warning "Directory ${MSG_DIMMED_FORMAT}utest/${MSG_END_FORMAT} was not created during compilation.
  Skipping Libpointmatcher unit-test."
fi

# ====Continue=====================================================================================
#exit "$(echo $?)"
exec "$@"
