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
## import shell functions from Libpointmatcher-build-system utilities library
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
#  local FULL_DOCKER_COMMAND=("$@")
#  local FULL_DOCKER_COMMAND="$*"
  #    local MSG_DIMMED_FORMAT="\033[1;2m"
  #    local MSG_END_FORMAT="\033[0m"

#  # (Priority) ToDo: on task end >> delete next bloc ↓↓
#  SCRIPT_ARG=("$@")
#  SCRIPT_ARG_ALT="$*"
#  print_msg_warning "show_and_execute_docker:"
#  echo -e ${SCRIPT_ARG[*]}
#  echo -e ${FULL_DOCKER_COMMAND}

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

# =================================================================================================================
# Send TeamCity blockOpened/blockClosed service message
#   or print the message to console when executed outside a TeamCity Agent run.
#
# Usage:
#   $ teamcity_service_msg_blockOpened "<theMessage>"
#   $ ... many steps ...
#   $ teamcity_service_msg_blockClosed
#
# Globals:
#   Read        'IS_TEAMCITY_RUN'
#   Read|write  'CURRENT_BLOCK_SERVICE_MSG'
# Outputs:
#   Output either
#     - a TeamCity blockOpened/blockClosed service message
#     - or print to console
#     - or an error if teamcity_service_msg_blockOpened is not closed using teamcity_service_msg_blockClosed
#
# Reference:
#   - TeamCity doc: https://www.jetbrains.com/help/teamcity/service-messages.html#Blocks+of+Service+Messages
#
# =================================================================================================================
function teamcity_service_msg_blockOpened() {
  local THE_MSG=$1
  if [[ ${CURRENT_BLOCK_SERVICE_MSG} ]]; then
    print_msg_error_and_exit "The TeamCity bloc service message ${MSG_DIMMED_FORMAT}${CURRENT_BLOCK_SERVICE_MSG}${MSG_END_FORMAT} was not closed using function ${MSG_DIMMED_FORMAT}teamcity_service_msg_blockClosed${MSG_END_FORMAT}."
  else
    export CURRENT_BLOCK_SERVICE_MSG="${THE_MSG}"
  fi

  if [[ ${IS_TEAMCITY_RUN} == true ]]; then
    echo "##teamcity[blockOpened name='${MSG_BASE_TEAMCITY} ${THE_MSG}']"
  else
    echo && print_msg "${THE_MSG}" && echo
  fi
}

function teamcity_service_msg_blockClosed() {
  if [[ ${IS_TEAMCITY_RUN} == true ]]; then
    echo "##teamcity[blockClosed name='${MSG_BASE_TEAMCITY} ${CURRENT_BLOCK_SERVICE_MSG}']"
  fi
  # Reset the variable since the bloc is closed
  unset CURRENT_BLOCK_SERVICE_MSG
}

# =================================================================================================================
# Send TeamCity compilationStarted/compilationFinished service message
#   or print the message to console when executed outside a TeamCity Agent run.
#
# Usage:
#   $ teamcity_service_msg_compilationStarted "<theMessage>"
#   $ ... many compilation steps ...
#   $ teamcity_service_msg_compilationFinished
#
# Globals:
#   Read        'IS_TEAMCITY_RUN'
#   Read|write  'CURRENT_COMPILATION_SERVICE_MSG_COMPILER'
# Outputs:
#   Output either
#     - a TeamCity compilationStarted/compilationFinished service message
#     - or print to console
#     - or an error if teamcity_service_msg_compilationStarted is not closed using teamcity_service_msg_compilationFinished
#
# Reference:
#   - TeamCity doc: https://www.jetbrains.com/help/teamcity/service-messages.html#Reporting+Compilation+Messages
#
# =================================================================================================================
function teamcity_service_msg_compilationStarted() {
  local THE_MSG=$1
  if [[ ${CURRENT_COMPILATION_SERVICE_MSG_COMPILER} ]]; then
    print_msg_error_and_exit "The TeamCity compilation service message ${MSG_DIMMED_FORMAT}${CURRENT_COMPILATION_SERVICE_MSG_COMPILER}${MSG_END_FORMAT} was not closed using function ${MSG_DIMMED_FORMAT}teamcity_service_msg_compilationFinished${MSG_END_FORMAT}."
  else
    export CURRENT_COMPILATION_SERVICE_MSG_COMPILER="${THE_MSG}"
  fi

  if [[ ${IS_TEAMCITY_RUN} == true ]]; then
    echo "##teamcity[compilationStarted compiler='${MSG_BASE_TEAMCITY} ${THE_MSG}']"
  else
    echo && print_msg "${THE_MSG}" && echo
  fi
}

function teamcity_service_msg_compilationFinished() {
  if [[ ${IS_TEAMCITY_RUN} == true ]]; then
    echo "##teamcity[compilationFinished compiler='${MSG_BASE_TEAMCITY} ${CURRENT_COMPILATION_SERVICE_MSG_COMPILER}']"
  fi
  # Reset the variable since the bloc is closed
  unset CURRENT_COMPILATION_SERVICE_MSG_COMPILER
}
