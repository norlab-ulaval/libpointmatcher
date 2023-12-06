#!/bin/bash


# ....path resolution logic........................................................................
NBS_PATH_TO_SRC_SCRIPT="$(realpath "${BASH_SOURCE[0]}")"
NBS_ROOT_DIR="$(dirname "${NBS_PATH_TO_SRC_SCRIPT}")/utilities/norlab-build-system"


#source ./nbs_execute_compose_over_build_matrix.bash "$@"
source "${NBS_ROOT_DIR}"/src/build_scripts/nbs_execute_compose_over_build_matrix.bash "$@"
