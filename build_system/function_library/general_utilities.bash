#!/bin/bash
#
# General purpose function library
#
# Requirement: This script must be sourced from directory 'build_system'
#
# usage:
#   $ source ./function_library/general_utilities.bash
#
set -e

# ....Pre-condition................................................................................................
if [[ "$(basename $(pwd))" != "build_system" ]]; then
  echo -e "\nERROR: This script must be sourced from directory 'build_system'!\n cwd: $(pwd)"
  exit 1
fi


# ....Project root logic...........................................................................................
TMP_CWD=$(pwd)

# ....Load environment variables from file.........................................................................
set -o allexport
source .env
source .env.prompt
set +o allexport

# ....Load helper function.........................................................................................
# import shell functions from utilities library
source ./function_library/prompt_utilities.bash


# =================================================================================================================
# Show docker command and execute it. Just enclose the command with argument in quote.
#
# Usage:
#   $ show_and_execute_docker "<docker command with argument>"
#
#   Example usage:
#   $ show_and_execute_docker "run --name IamBuildSystemTester -t -i --rm lpm.ubuntu20.buildsystem.test"
#
# =================================================================================================================
function show_and_execute_docker() {
    local FULL_DOCKER_COMMAND=$1
#    local MSG_DIMMED_FORMAT="\033[1;2m"
#    local MSG_END_FORMAT="\033[0m"

    if [ -f /.dockerenv ]; then
      echo
      print_msg_warning "Skipping the execution of Docker command\n
      ${MSG_DIMMED_FORMAT}$ docker ${FULL_DOCKER_COMMAND}${MSG_END_FORMAT}\n\nsince the script is executed inside a docker container ... and starting Docker daemon inside a container is complicated to setup and overkill for our testing case."
    else
      print_msg "Execute command ${MSG_DIMMED_FORMAT}$ docker ${FULL_DOCKER_COMMAND}${MSG_END_FORMAT}"

      # shellcheck disable=SC2086
      docker ${FULL_DOCKER_COMMAND}

      print_msg_done "Command ${MSG_DIMMED_FORMAT}$ docker ${FULL_DOCKER_COMMAND}${MSG_END_FORMAT} completed and exited docker."
    fi
}

