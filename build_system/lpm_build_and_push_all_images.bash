#!/bin/bash

# Set env variable: LPM_IMAGE_ARCHITECTURE
source ./lpm_which_architecture.bash

set -o allexport; source .env; source .env.prompt; set +o allexport

# ....Build all images..............................................................................................
#docker compose -f <theComposeFile> build --no-cache

echo -e "${LPM_MSG_BASE} Building 'docker-compose.libpointmatcher.build.yaml'"

for LIBPOINTMATCHER_TAG in "${LPM_LIBPOINTMATCHER_VERSIONS[@]}" ; do
  export LIBPOINTMATCHER_TAG

  export DEPENDENCIES_BASE_IMAGE=ubuntu

  for UBUNTU_VERSION in "${LPM_UBUNTU_SUPPORTED_VERSIONS[@]}" ; do

    export DEPENDENCIES_BASE_IMAGE_TAG="${UBUNTU_VERSION}"
    export OS_TAG="${DEPENDENCIES_BASE_IMAGE}${UBUNTU_VERSION}"

    echo -e "${LPM_MSG_BASE} Building tag '${OS_TAG}-${LPM_IMAGE_ARCHITECTURE}-${LIBPOINTMATCHER_TAG}'"

#    docker compose -f docker-compose.libpointmatcher.build.yaml build --push
    docker compose -f docker-compose.libpointmatcher.build.yaml build

    echo -e "${LPM_MSG_DONE} Build and push done"
  done
done


echo -e "${LPM_MSG_DONE} All build and push done"
