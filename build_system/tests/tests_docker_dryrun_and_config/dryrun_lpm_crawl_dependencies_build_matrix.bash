#!/bin/bash

# ....path resolution logic........................................................................
_PATH_TO_SCRIPT="$(realpath "${BASH_SOURCE[0]}")"
LPM_ROOT_DIR="$(dirname "${_PATH_TO_SCRIPT}")"
cd "${LPM_ROOT_DIR}/../../"

# ====begin========================================================================================
#FLAGS=( '--fail-fast' '--' 'build' '--dry-run' )
#bash lpm_crawl_dependencies_build_matrix.bash ${FLAGS[@]}

#bash lpm_crawl_dependencies_build_matrix.bash --fail-fast -- build --dry-run dependencies-general dependencies dependencies-doc
#bash lpm_crawl_dependencies_build_matrix.bash --fail-fast -- build --dry-run dependencies-general

bash lpm_crawl_dependencies_build_matrix.bash --fail-fast -- build --dry-run
