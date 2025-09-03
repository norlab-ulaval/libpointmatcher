#!/bin/bash

clear

# ....path resolution logic........................................................................
_PATH_TO_SCRIPT="$(realpath "${BASH_SOURCE[0]}")"
LPM_ROOT_DIR="$(dirname "${_PATH_TO_SCRIPT}")"
cd "${LPM_ROOT_DIR}/../../.."

# ====begin========================================================================================
bash build_system/tests/tests_docker_interactive/build_and_run_IamBuildSystemTester.bash "bash ./lpm_crawl_libpointmatcher_build_matrix.bash \
    --repository-version-build-matrix-override PR1 \
    --cmake-build-type-build-matrix-override RelWithDebInfo \
    --os-name-build-matrix-override osx \
    --osx-version-build-matrix-override ventura \
    --ubuntu-version-build-matrix-override jammy \
    -- build --dry-run dependencies"
