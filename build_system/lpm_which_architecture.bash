#!/bin/bash

set -e

# Load environment variable from file
set -o allexport; source .env.prompt; set +o allexport

# LPM_IMAGE_ARCHITECTURE: arm64-l4t, arm64-darwin, x86, arm64
# ARCH: aarch64, arm64, x86_64
# OS: Linux, Darwin, Window

if [[ $(uname -m) == "aarch64" ]]; then
  if [[ -n $(uname -r | grep tegra) ]]; then
    export LPM_IMAGE_ARCHITECTURE='arm64-l4t'
  else
    echo -e "${LPM_MSG_ERROR} Unsuported OS for aarch64 processor"
  fi
elif [[ $(uname -m) == "arm64" ]]; then
  if [[ $(uname) == "Darwin" ]]; then
    export LPM_IMAGE_ARCHITECTURE='arm64-darwin'
  else
    export LPM_IMAGE_ARCHITECTURE='arm64' # ToDo: validate
  fi
elif [[ $(uname -m) == "x86_64" ]]; then
  export LPM_IMAGE_ARCHITECTURE='x86'
else
  echo -e "${LPM_MSG_ERROR} Unsuported processor architecture"
fi

echo -e "${LPM_MSG_BASE} which LPM_IMAGE_ARCHITECTURE env? $LPM_IMAGE_ARCHITECTURE"
