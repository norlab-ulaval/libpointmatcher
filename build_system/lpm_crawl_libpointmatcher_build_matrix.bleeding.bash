#!/bin/bash
#
# Execute build matrix specified in .env.build_matrix.libpointmatcher.bleeding
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

# ....path resolution logic........................................................................
NBS_PATH_TO_SRC_SCRIPT="$(realpath "${BASH_SOURCE[0]}")"
LPM_ROOT_DIR="$(dirname "${NBS_PATH_TO_SRC_SCRIPT}")/.."
N2ST_ROOT_DIR="$(dirname "${NBS_PATH_TO_SRC_SCRIPT}")/utilities/norlab-shell-script-tools"

# ....Load environment variables from file.........................................................
set -o allexport && source ${LPM_ROOT_DIR}/build_system/.env && set +o allexport

DOTENV_BUILD_MATRIX=${LPM_ROOT_DIR}/build_system/.env.build_matrix.libpointmatcher.bleeding

# ....Helper function..............................................................................
# import shell functions from utilities library
source "${N2ST_ROOT_DIR}"/import_norlab_shell_script_tools_lib.bash

# ====Begin========================================================================================
cd "${LPM_ROOT_DIR}"/build_system/utilities/norlab-build-system/src/utility_scripts
source nbs_execute_compose_over_build_matrix.bash "${DOTENV_BUILD_MATRIX}" "$@"
