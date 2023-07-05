#!/bin/bash
#
# Usage:
#   $ bash lpm_entrypoint_execute_unittest.bash [<any>]
#
# Parameter
#   <any> (Optional) Everything passed here will be executed at the end of this script
#
set -e

# ....Load environment variables from file.........................................................................
set -o allexport
source ../.env
set +o allexport

# ==== Execute libpointmatcher unit-test===========================================================================
cd "${LPM_INSTALLED_LIBRARIES_PATH}/${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}/build"
utest/utest --path "${LPM_INSTALLED_LIBRARIES_PATH}/${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}/examples/data/"

# ====Continue=====================================================================================================
exec "${@}"
