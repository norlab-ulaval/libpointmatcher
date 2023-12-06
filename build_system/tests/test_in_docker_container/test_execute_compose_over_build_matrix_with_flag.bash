#!/bin/bash

clear

# ....path resolution logic........................................................................
NBS_PATH_TO_SRC_SCRIPT="$(realpath "${BASH_SOURCE[0]}")"
NBS_ROOT_DIR="$(dirname "${NBS_PATH_TO_SRC_SCRIPT}")"
cd "${NBS_ROOT_DIR}/../"

# ====begin========================================================================================
bash build_and_run_IamBuildSystemTester.bash "bash ./nbs_execute_compose_over_build_matrix.bash \
    --repository-version-build-matrix-override PR1 \
    --cmake-build-type-build-matrix-override RelWithDebInfo \
    --os-name-build-matrix-override osx \
    --osx-version-build-matrix-override ventura \
    --ubuntu-version-build-matrix-override jammy \
    -- build --dry-run dependencies"
