#!/bin/bash

clear

# ....path resolution logic........................................................................
_PATH_TO_SCRIPT="$(realpath "${BASH_SOURCE[0]}")"
LPM_ROOT_DIR="$(dirname "${_PATH_TO_SCRIPT}")"
cd "${LPM_ROOT_DIR}/../../"

# ====begin========================================================================================
bash -c "export BUILDX_BUILDER=local-builder-multiarch-virtual \
         && bash lpm_crawl_libpointmatcher_build_matrix.bash \
                  --fail-fast \
                  -- config --quiet"
