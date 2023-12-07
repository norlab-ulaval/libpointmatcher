#!/bin/bash

clear

# ....path resolution logic........................................................................
_PATH_TO_SCRIPT="$(realpath "${BASH_SOURCE[0]}")"
LPM_ROOT_DIR="$(dirname "${_PATH_TO_SCRIPT}")"
cd "${LPM_ROOT_DIR}/../"

# ====begin========================================================================================
bash build_and_run_IamBuildSystemTester.bash "bash ./nbs_execute_compose.bash docker-compose.libpointmatcher.yaml \
    --cmake-build-type None \
    --os-version bionic \
    --repository-version 1.3.1 \
    --os-name osx \
    -- build --dry-run dependencies"
