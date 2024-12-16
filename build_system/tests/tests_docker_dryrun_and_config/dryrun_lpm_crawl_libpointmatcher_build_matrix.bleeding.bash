#!/bin/bash

# ....path resolution logic........................................................................
_PATH_TO_SCRIPT="$(realpath "${BASH_SOURCE[0]}")"
LPM_ROOT_DIR="$(dirname "${_PATH_TO_SCRIPT}")"
cd "${LPM_ROOT_DIR}/../../"

# ====begin========================================================================================
#bash lpm_crawl_libpointmatcher_build_matrix.bleeding.bash --fail-fast -- build --dry-run ci_PR_amd64 ci_PR_arm64
bash lpm_crawl_libpointmatcher_build_matrix.bleeding.bash --fail-fast -- build --dry-run
