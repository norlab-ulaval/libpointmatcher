#!/bin/bash -i
#
# Docker entrypoint for running libpointmatcher installer
#
# Usage:
#   $ bash entrypoint_build_libpointmatcher_checkout_branch.bash [<any-cmd>]
#
# Parameter
#   <any-cmd>      Optional command executed in a subprocess at the end of the entrypoint script.
#

# ====Build system tools===========================================================================
cd "${NBS_LIB_INSTALL_PATH}/${NBS_REPOSITORY_NAME}/build_system"

# ....Load environment variables from file.........................................................
set -o allexport
source .env
set +o allexport

# ==== Build libpointmatcher checkout branch ======================================================
cd ./ubuntu/
bash lpm_install_libpointmatcher_ubuntu.bash \
  --repository-version ${REPOSITORY_VERSION:?'err variable not set'} \
  --cmake-build-type ${CMAKE_BUILD_TYPE} \
  ${INSTALL_SCRIPT_FLAG}

# ====Continue=====================================================================================
exec "$@"
