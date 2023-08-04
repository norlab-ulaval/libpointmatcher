#!/bin/bash
#
# Utility script to install docker related tools and execute basic configuration
#
# usage:
#   $ bash ./lpm_utility_script/lpm_install_docker_tools.bash
#
set -e

# ....Project root logic...........................................................................................
TMP_CWD=$(pwd)

if [[ "$(basename $(pwd))" != "build_system" ]]; then
  cd ../
fi


# ....Load environment variables from file.........................................................................
set -o allexport
source .env
source .env.prompt
set +o allexport

# ....Load helper function.........................................................................................
## import shell functions from Libpointmatcher-build-system utilities library
source ./function_library/prompt_utilities.bash

# ====Begin========================================================================================================
print_formated_script_header 'lpm_install_docker_tools.bash'

# ....Check pre-condition..........................................................................................

# .................................................................................................................
echo
print_msg "Install utilities"
echo

sudo apt-get update &&
  sudo apt-get upgrade --assume-yes &&
  sudo apt-get install --assume-yes \
    ca-certificates \
    curl \
    lsb-release \
    gnupg \
    apt-utils &&
  sudo rm -rf /var/lib/apt/lists/*

# .................................................................................................................
echo
print_msg "Install Docker tools" "${LPM_LINE_CHAR_UTIL}"

# . . Add Docker’s official GPG key:. . . . . . . . . . . . . . . . . . . . . . . . . . .
sudo mkdir -m 0755 -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# . . set up the docker repository:. . . . . . . . . . . . . . . . . . . . . . . . . . .
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null

# .................................................................................................................
print_msg "Install Docker-compose"
echo

sudo apt-get update &&
  sudo apt-get upgrade &&
  sudo apt-get install --assume-yes \
    docker-ce \
    docker-ce-cli \
    containerd.io \
    docker-buildx-plugin \
    docker-compose-plugin

# .................................................................................................................
echo
print_msg "Configure docker"
echo

print_msg "Manage Docker as a non-root user"
# Config so that we dont have to preface docker command with sudo everytime
# Ref: https://docs.docker.com/engine/install/linux-postinstall/
echo

sudo groupadd -f docker
sudo usermod -a -G docker "$(whoami)"

print_msg "${NTSI_ADMIN_USER} added to docker group"

print_formated_script_footer 'lpm_install_docker_tools.bash' "${LPM_LINE_CHAR_UTIL}"
# ====Teardown=====================================================================================================
cd "${TMP_CWD}"
