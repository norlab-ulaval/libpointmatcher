#!/bin/bash

clear

# ....path resolution logic........................................................................
NBS_PATH_TO_SRC_SCRIPT="$(realpath "${BASH_SOURCE[0]}")"
NBS_ROOT_DIR="$(dirname "${NBS_PATH_TO_SRC_SCRIPT}")"
cd "${NBS_ROOT_DIR}/../"

# ====begin========================================================================================
bash build_and_run_IamBuildSystemTester.bash "bash ./nbs_execute_compose.bash docker-compose.libpointmatcher.yaml \
    --cmake-build-type None \
    --os-version bionic \
    --repository-version 1.3.1 \
    --os-name osx \
    -- build --dry-run dependencies"
