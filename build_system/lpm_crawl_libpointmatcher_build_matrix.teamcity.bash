#!/bin/bash

# ....Architecture..............................................
TC_LPM_BUILD_ARCH=%TC_LPM_BUILD_ARCH%
if [[ -z ${TC_LPM_BUILD_ARCH} ]]; then
  TC_LPM_BUILD_ARCH=ci_PR_multiarch
fi
echo "Build architecture: ${TC_LPM_BUILD_ARCH}"

# ....Setup flags for dn_build_all.bash.........................
LPM_CRAWL_BUILD_MATRIX_FLAGS=()
# Add flag from TeamCity custom parameters
LPM_CRAWL_BUILD_MATRIX_FLAGS+=($(echo %LPM_CRAWL_BUILD_MATRIX_FLAGS% | tr " " "\n"))
if [[ -n "${LPM_CRAWL_BUILD_MATRIX_FLAGS}" ]]; then
  echo "LPM_CRAWL_BUILD_MATRIX_FLAGS=("
  for each in "${LPM_CRAWL_BUILD_MATRIX_FLAGS[@]}" ; do
      echo "  ${each}"
  done
  echo ")"
fi

LPM_CRAWL_BUILD_MATRIX_FLAGS+=("--repository-version-build-matrix-override" "latest")
LPM_CRAWL_BUILD_MATRIX_FLAGS+=("--fail-fast")
LPM_CRAWL_BUILD_MATRIX_FLAGS+=("--" "build" "${TC_LPM_BUILD_ARCH:?err}")

# ....Dry-run..............................................
TC_DRY_RUN=%TC_DRY_RUN%
if [[ ${TC_DRY_RUN} == true ]]; then
  echo "Run in dry-run mode"
  LPM_CRAWL_BUILD_MATRIX_FLAGS+=("--dry-run")
fi

# ....Final.....................................................
echo -e "execute: bash lpm_crawl_libpointmatcher_build_matrix.bash ${LPM_CRAWL_BUILD_MATRIX_FLAGS[*]}"
bash lpm_crawl_libpointmatcher_build_matrix.bash "${LPM_CRAWL_BUILD_MATRIX_FLAGS[@]}"
