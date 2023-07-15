#!/bin/bash
#
# Usage:
#   $ bash entrypoint_execute_lpm_unittest.bash [<any>]
#
# Parameter
#   <any> (Optional) Everything passed here will be executed at the end of this script
#
set -e

# ....Load environment variables from file.........................................................................
set -o allexport
source ../.env
set +o allexport

# ==== Build libpointmatcher checkout branch ======================================================================
source lpm_install_libpointmatcher_ubuntu.bash \
  --libpointmatcher-version ${LIBPOINTMATCHER_VERSION:?'err variable not set'} \
  ${LIBPOINTMATCHER_INSTALL_SCRIPT_FLAG}

# ==== Execute libpointmatcher unit-test===========================================================================
cd "${LPM_INSTALLED_LIBRARIES_PATH}/${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}/build"
utest/utest --path "${LPM_INSTALLED_LIBRARIES_PATH}/${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}/examples/data/"

# (Priority) ToDo: implement (ref task NMO-266 LPM unit-test › gtest feedback for TC build step pass/fail status)

# ====Continue=====================================================================================================
#exec "${@}"
exec bash
