#!/bin/bash
# =================================================================================================
#
# Execute build matrix specified in .env.build_matrix.libpointmatcher
#
# Redirect the execution to 'nbs_execute_compose_over_build_matrix.bash' from the norlab-build-system library
#
# Usage:
#   $ bash lpm_crawl_libpointmatcher_build_matrix.bash [<optional flag>] [-- <any docker cmd+arg>]
#
#   $ bash lpm_crawl_libpointmatcher_build_matrix.bash -- build --dry-run
#
# Run script with the '--help' flag for details
#
# =================================================================================================
PARAMS="$@"

# ....path resolution logic........................................................................
LPM_ROOT="$(dirname "$(realpath "$0")")/.."
LPM_BUILD_SYSTEM_PATH="${LPM_ROOT}/build_system"
NBS_PATH="${LPM_BUILD_SYSTEM_PATH}/utilities/norlab-build-system"
N2ST_PATH="${LPM_BUILD_SYSTEM_PATH}/utilities/norlab-shell-script-tools"

# ....Load environment variables from file.........................................................
cd "${LPM_BUILD_SYSTEM_PATH}" || exit 1
set -o allexport && source .env && set +o allexport

# ....Source NBS dependencies......................................................................
cd "${NBS_PATH}" || exit 1
source import_norlab_build_system_lib.bash

# ====begin========================================================================================
cd "${NBS_PATH}/src/utility_scripts" || exit 1

DOTENV_BUILD_MATRIX_REALPATH=${LPM_BUILD_SYSTEM_PATH}/.env.build_matrix.libpointmatcher

# Note: do not double quote PARAMS or threat it as a array otherwise it will cause error
# shellcheck disable=SC2086
source nbs_execute_compose_over_build_matrix.bash "${DOTENV_BUILD_MATRIX_REALPATH}" $PARAMS

