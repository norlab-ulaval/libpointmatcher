#!/bin/bash
#
# Execute build matrix on docker compose docker-compose.libpointmatcher.build.yaml
#
# Usage:
#   $ bash lpm_build_and_push_all_images.bash [<any>]
#
# Arguments:
#   <any-docker-compose-flag> (Optional) any docker compose flag
#
set -e

# ....Project root logic...........................................................................................
TMP_CWD=$(pwd)

# ....Load environment variables from file.........................................................................
set -o allexport
source .env
source .env.prompt
set +o allexport

# ....Helper function..............................................................................................
# import shell functions from Libpointmatcher-build-system utilities library
source ./function_library/prompt_utilities.bash

# ====Begin========================================================================================================
print_formated_script_header 'lpm_build_and_push_all_images.bash' =

# ..................................................................................................................
print_msg "Set env variable: LPM_IMAGE_ARCHITECTURE"
source ./lpm_which_architecture.bash

# ..................................................................................................................
#docker compose -f <theComposeFile> build --no-cache

print_msg "Build all images from 'docker-compose.libpointmatcher.build.yaml'"

for LIBPOINTMATCHER_TAG in "${LPM_LIBPOINTMATCHER_VERSIONS[@]}"; do
  export LIBPOINTMATCHER_TAG

  export DEPENDENCIES_BASE_IMAGE=ubuntu

  for UBUNTU_VERSION in "${LPM_UBUNTU_SUPPORTED_VERSIONS[@]}"; do

    export DEPENDENCIES_BASE_IMAGE_TAG="${UBUNTU_VERSION}"
    export OS_TAG="${DEPENDENCIES_BASE_IMAGE}${UBUNTU_VERSION}"

    print_msg "Building tag '${OS_TAG}-${LPM_IMAGE_ARCHITECTURE}-${LIBPOINTMATCHER_TAG}'"

    #    docker compose -f docker-compose.libpointmatcher.build.yaml build --push
    docker compose -f docker-compose.libpointmatcher.build.yaml build \
      --build-arg LIBPOINTMATCHER_TAG="${LIBPOINTMATCHER_TAG}" \
      --build-arg DEPENDENCIES_BASE_IMAGE_TAG="${UBUNTU_VERSION}" \
      --build-arg OS_TAG="${OS_TAG}" \
      "${@}"

    #      --build-arg LIBPOINTMATCHER_TAG="1.3.1"
    #      --build-arg DEPENDENCIES_BASE_IMAGE_TAG="20.04"
    #      --build-arg OS_TAG="ubuntu20.04"

    print_msg_done "Build done"
  done
done

print_msg_done "FINAL › All builds done"
draw_horizontal_line_across_the_terminal_window =
# ====Teardown=====================================================================================================
cd "${TMP_CWD}"

