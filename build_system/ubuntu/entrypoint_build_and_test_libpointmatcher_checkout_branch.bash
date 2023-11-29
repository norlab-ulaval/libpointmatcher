#!/bin/bash -i
#
# Docker entrypoint for running libpointmatcher installer and unit-test
#
# Usage:
#   $ bash entrypoint_execute_lpm_unittest.bash [<any-cmd>]
#
# Parameter
#   <any-cmd>      Optional command executed in a subprocess at the end of the entrypoint script.
#

# ....Load environment variables from file.........................................................................
set -o allexport
source ../.env
set +o allexport

# ==== Build libpointmatcher checkout branch ======================================================================
source lpm_install_libpointmatcher_ubuntu.bash \
  --repository-version ${REPOSITORY_VERSION:?'err variable not set'} \
  --cmake-build-type ${LIBPOINTMATCHER_CMAKE_BUILD_TYPE} \
  ${LIBPOINTMATCHER_INSTALL_SCRIPT_FLAG}

# ==== Execute libpointmatcher unit-test===========================================================================
cd "${NBS_LIB_INSTALL_PATH}/${NBS_REPOSITORY_NAME}/build"
utest/utest --path "${NBS_LIB_INSTALL_PATH}/${NBS_REPOSITORY_NAME}/examples/data/"

# ====Continue=====================================================================================================
exec "$@"
