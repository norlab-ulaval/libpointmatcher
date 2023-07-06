#!/bin/bash -i
#
# Build and run a single container based on docker compose docker-compose.libpointmatcher.yaml
#
# Usage:
#   $ bash lpm_execute_compose.bash [<optional flag>] [-- <any docker cmd+arg>]
#
# Arguments:
#   [--libpointmatcher-version latest]    The libpointmatcher release tag (default: see LPM_VERSION)
#   [--os-name ubuntu]                    The operating system name. Either 'ubuntu' or 'osx' (default: see OS_NAME)
#   [--os-version jammy]                  Name named operating system version, see .env for supported version
#                                           (default: see OS_VERSION)
#   [--job-id 666]
#   [-- <any docker cmd+arg>]             Any argument passed after '--' will be passed to docker compose
#                                           as docker command and arguments
#                                           (default: see DOCKER_COMPOSE_CMD_ARGS)
#   [-h, --help]                          Get help
#   [--debug]
#
set -e


# ....Default......................................................................................................
LPM_VERSION='latest'
OS_NAME='ubuntu'
OS_VERSION='20.04'
LPM_JOB_ID='0'
DOCKER_COMPOSE_CMD_ARGS='up --build --force-recreate'

# ....Project root logic...........................................................................................
TMP_CWD=$(pwd)

# ....Load environment variables from file.........................................................................
set -o allexport
source .env
source .env.build_matrix
source .env.prompt
set +o allexport

# ....Helper function..............................................................................................
# import shell functions from Libpointmatcher-build-system utilities library
source ./function_library/prompt_utilities.bash
source ./function_library/general_utilities.bash

function print_help_in_terminal() {
  echo -e "\n
\$ ${0} [<optional flag>] [-- <any docker cmd+arg>]
  \033[1m
    <optional argument>:\033[0m
      -h, --help                              Get help
      --libpointmatcher-version latest        The libpointmatcher release tag (default to latest 'latest')
      --os-name ubuntu                        The operating system name. Either 'ubuntu' or 'osx' (default to 'ubuntu')
      --os-version jammy                      Name named operating system version, see .env for supported version
                                              (default to 'jammy')
      --job-id 1
      --debug
  \033[1m
    [-- <any docker cmd+arg>]\033[0m                 Any argument passed after '--' will be passed to docker compose as docker
                                              command and arguments (default to '${DOCKER_COMPOSE_CMD_ARGS}')
"
}


# ====Begin========================================================================================================
print_formated_script_header 'lpm_execute_compose.bash' =

# ....Script command line flags....................................................................................

while [ $# -gt 0 ]; do

#  echo -e "'\$*' before: ${MSG_DIMMED_FORMAT}$*${MSG_END_FORMAT}" # ToDo: on task end >> delete this line ←
#  echo -e "\$1: ${1}    \$2: $2" # ToDo: on task end >> delete this line ←

  case $1 in
  --libpointmatcher-version)
    LPM_VERSION="${2}"
    shift # Remove argument (--libpointmatcher-version)
    shift # Remove argument value
    ;;
  --os-name)
    OS_NAME="${2}"
    shift # Remove argument (--os-name)
    shift # Remove argument value
    ;;
  --os-version)
    OS_VERSION="${2}"
    shift # Remove argument (--os-version)
    shift # Remove argument value
    ;;
  --job-id)
    LPM_JOB_ID="${2}"
    shift # Remove argument (--job-id)
    shift # Remove argument value
    ;;
  --debug)
#    set -v
#    set -x
    export BUILDKIT_PROGRESS=plain
    shift # Remove argument (--debug)
    ;;
  -h | --help)
    print_help_in_terminal
    exit
    ;;
  --) # no more option
    shift
    DOCKER_COMPOSE_CMD_ARGS="$*"
    break
    ;;
  *) # Default case
    break
    ;;
  esac

#  echo -e "'\$*' after: ${MSG_DIMMED_FORMAT}$*${MSG_END_FORMAT}" # ToDo: on task end >> delete this line ←
#  echo -e "after \$1: ${1}    \$2: $2" # ToDo: on task end >> delete this line ←
#  echo

done

#echo -e "'\$*' on DONE: ${MSG_DIMMED_FORMAT}$*${MSG_END_FORMAT}" # ToDo: on task end >> delete this line ←

## ToDo: on task end >> delete next bloc ↓↓
#echo -e " ${MSG_DIMMED_FORMAT}
#LPM_VERSION=${LPM_VERSION}
#OS_NAME=${OS_NAME}
#OS_VERSION=${OS_VERSION}
#DOCKER_COMPOSE_CMD_ARGS=${DOCKER_COMPOSE_CMD_ARGS}
#${MSG_END_FORMAT} "


# ..................................................................................................................
# Set environment variable LPM_IMAGE_ARCHITECTURE
source ./lpm_utility_script/lpm_export_which_architecture.bash

# ..................................................................................................................
print_msg "Executing docker compose command on ${MSG_DIMMED_FORMAT}docker-compose.libpointmatcher.yaml${MSG_END_FORMAT} with command ${MSG_DIMMED_FORMAT}${DOCKER_COMPOSE_CMD_ARGS}${MSG_END_FORMAT}"

# Note: LPM_VERSION will be used to fetch the repo at release tag (ref task NMO-252)
export LPM_VERSION="${LPM_VERSION}"
export LPM_JOB_ID="${LPM_JOB_ID}"
export DEPENDENCIES_BASE_IMAGE="${OS_NAME}"
export DEPENDENCIES_BASE_IMAGE_TAG="${OS_VERSION}"
export LPM_IMAGE_TAG="${LPM_VERSION}-${DEPENDENCIES_BASE_IMAGE}${DEPENDENCIES_BASE_IMAGE_TAG}-${LPM_IMAGE_ARCHITECTURE:?'err: variable not set'}"

print_msg "Image tag ${MSG_DIMMED_FORMAT}${LPM_IMAGE_TAG}${MSG_END_FORMAT}"
print_msg "Environment variables set for this build run:\n\n${MSG_DIMMED_FORMAT}$(printenv | grep -i -e LPM_ -e DEPENDENCIES_BASE_IMAGE -e BUILDKIT)${MSG_END_FORMAT}\n"

## docker compose [-f <theComposeFile> ...] [options] [COMMAND] [ARGS...]
## docker compose [-f <theComposeFile> ...] build --no-cache --push
## docker compose build [OPTIONS] [SERVICE...]
## docker compose run [OPTIONS] SERVICE [COMMAND] [ARGS...]

show_and_execute_docker "compose -f docker-compose.libpointmatcher.yaml ${DOCKER_COMPOSE_CMD_ARGS}"

print_formated_script_footer 'lpm_execute_compose.bash' =
# ====Teardown=====================================================================================================
cd "${TMP_CWD}"

