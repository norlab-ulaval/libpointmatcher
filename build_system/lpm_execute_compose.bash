#!/bin/bash
#
# Build and run a single container based on docker compose docker-compose.libpointmatcher.yaml
#
# Usage:
#   $ bash lpm_execute_compose.bash [<optional flag>] [-- <any docker cmd+arg>]
#
#   $ bash lpm_execute_compose.bash -- run --rm ci
#
# Arguments:
#   [--libpointmatcher-version v1.3.1]    The libpointmatcher release tag (default: see LIBPOINTMATCHER_VERSION)
#   [--libpointmatcher-cmake-build-type RelWithDebInfo]
#                                         Change the libpointmatcher compilation mode.
#                                         Either 'None' 'Debug' 'Release' 'RelWithDebInfo' or 'MinSizeRel'
#   [--os-name ubuntu]                    The operating system name. Either 'ubuntu' or 'osx' (default: see OS_NAME)
#   [--os-version jammy]                  Name named operating system version, see .env for supported version
#                                           (default: see OS_VERSION)
#   [-- <any docker cmd+arg>]             Any argument passed after '--' will be passed to docker compose
#                                           as docker command and arguments (default: 'up --build --force-recreate')
#                                         Note: passing script flag via docker --build-arg can be tricky,
#                                               pass them in the docker-compose.yaml if you experience problem.
#   [--docker-debug-logs]                 Set Docker builder log output for debug (i.e.BUILDKIT_PROGRESS=plain)
#   [--fail-fast]                         Exit script at first encountered error
#   [-h, --help]                          Get help
#
## set -e   # Note: use the --fail-fast flag instead


# ....Default......................................................................................................
LIBPOINTMATCHER_VERSION='head'
LIBPOINTMATCHER_CMAKE_BUILD_TYPE='RelWithDebInfo'
OS_NAME='ubuntu'
OS_VERSION='focal'
#LPM_JOB_ID='0'
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
## import shell functions from Libpointmatcher-build-system utilities library
source ./function_library/prompt_utilities.bash
source ./function_library/general_utilities.bash
source ./function_library/terminal_splash.bash

function print_help_in_terminal() {
  echo -e "\n
\$ ${0} [<optional flag>] [-- <any docker cmd+arg>]
  \033[1m
    <optional argument>:\033[0m
      -h, --help                              Get help
      --libpointmatcher-version v1.3.1        The libpointmatcher release tag (default to master branch head)
      --libpointmatcher-cmake-build-type RelWithDebInfo
                                              Change the libpointmatcher compilation mode.
                                              Either 'None' 'Debug' 'Release' 'RelWithDebInfo' or 'MinSizeRel'
      --os-name ubuntu                        The operating system name. Either 'ubuntu' or 'osx' (default to 'ubuntu')
      --os-version jammy                      Name named operating system version, see .env for supported version
                                                (default to 'jammy')
      --docker-debug-logs                     Set Docker builder log output for debug (i.e.BUILDKIT_PROGRESS=plain)
      --fail-fast                             Exit script at first encountered error
  \033[1m
    [-- <any docker cmd+arg>]\033[0m                 Any argument passed after '--' will be passed to docker compose as docker
                                              command and arguments (default to '${DOCKER_COMPOSE_CMD_ARGS}').
                                              Note: passing script flag via docker --build-arg can be tricky,
                                                    pass them in the docker-compose.yaml if you experience problem.
"
#      --job-id                                Append job ID for CI test image
}

# ....TeamCity service message logic................................................................................
if [[ ${TEAMCITY_VERSION} ]]; then
  export IS_TEAMCITY_RUN=true
  TC_VERSION="TEAMCITY_VERSION=${TEAMCITY_VERSION}"
else
  export IS_TEAMCITY_RUN=false
fi
print_msg "IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN} ${TC_VERSION}"

# ====Begin========================================================================================================
SHOW_SPLASH_EC="${SHOW_SPLASH_EC:-true}"
#echo "\$SHOW_SPLASH_EC=${SHOW_SPLASH_EC}"

if [[ "${SHOW_SPLASH_EC}" == 'true' ]]; then
  norlab_splash "${LPM_BUILD_SYSTEM_SPLASH_NAME}" "https://github.com/${LPM_LIBPOINTMATCHER_SRC_DOMAIN}/${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}"
fi

print_formated_script_header 'lpm_execute_compose.bash' "${LPM_LINE_CHAR_BUILDER_LVL2}"

# ....Script command line flags....................................................................................
while [ $# -gt 0 ]; do

#  echo -e "'\$*' before: ${MSG_DIMMED_FORMAT}$*${MSG_END_FORMAT}" # ToDo: on task end >> delete this line ←
#  echo -e "\$1: ${1}    \$2: $2" # ToDo: on task end >> delete this line ←

  case $1 in
  --libpointmatcher-version)
    LIBPOINTMATCHER_VERSION="${2}"
    shift # Remove argument (--libpointmatcher-version)
    shift # Remove argument value
    ;;
  --libpointmatcher-cmake-build-type)
    LIBPOINTMATCHER_CMAKE_BUILD_TYPE="${2}"
    shift # Remove argument (--libpointmatcher-cmake-build-type)
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
#  --job-id)
#    LPM_JOB_ID="${2}"
#    shift # Remove argument (--job-id)
#    shift # Remove argument value
#    ;;
  --docker-debug-logs)
#    set -v
#    set -x
    export BUILDKIT_PROGRESS=plain
    shift # Remove argument (--docker-debug-logs)
    ;;
  --fail-fast)
    set -e
    shift # Remove argument (--fail-fast)
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
#LIBPOINTMATCHER_VERSION=${LIBPOINTMATCHER_VERSION}
#LIBPOINTMATCHER_CMAKE_BUILD_TYPE=${LIBPOINTMATCHER_CMAKE_BUILD_TYPE}
#OS_NAME=${OS_NAME}
#OS_VERSION=${OS_VERSION}
#DOCKER_COMPOSE_CMD_ARGS=${DOCKER_COMPOSE_CMD_ARGS}
#${MSG_END_FORMAT} "

# ..................................................................................................................
# Note: LIBPOINTMATCHER_VERSION will be used to fetch the repo at release tag (ref task NMO-252)
export LIBPOINTMATCHER_VERSION="${LIBPOINTMATCHER_VERSION}"
export LIBPOINTMATCHER_CMAKE_BUILD_TYPE="${LIBPOINTMATCHER_CMAKE_BUILD_TYPE}"
export DEPENDENCIES_BASE_IMAGE="${OS_NAME}"
export DEPENDENCIES_BASE_IMAGE_TAG="${OS_VERSION}"
#export LPM_JOB_ID="${LPM_JOB_ID}"

# ToDo: implement case (ref task NMO-225 ⚒︎ → Docker image multi-arch support) >> remove the target arch from the tag. L4T will be used in the OS tag
#export LPM_IMAGE_TAG="${LIBPOINTMATCHER_VERSION}-${DEPENDENCIES_BASE_IMAGE}${DEPENDENCIES_BASE_IMAGE_TAG}-${LPM_IMAGE_ARCHITECTURE:?'err: variable not set'}"
export LPM_IMAGE_TAG="${LIBPOINTMATCHER_VERSION}-${DEPENDENCIES_BASE_IMAGE}-${DEPENDENCIES_BASE_IMAGE_TAG}"

print_msg "Environment variables set for compose:\n
${MSG_DIMMED_FORMAT}    LIBPOINTMATCHER_VERSION=${LIBPOINTMATCHER_VERSION} ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    LIBPOINTMATCHER_CMAKE_BUILD_TYPE=${LIBPOINTMATCHER_CMAKE_BUILD_TYPE} ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    DEPENDENCIES_BASE_IMAGE=${DEPENDENCIES_BASE_IMAGE} ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    DEPENDENCIES_BASE_IMAGE_TAG=${DEPENDENCIES_BASE_IMAGE_TAG} ${MSG_END_FORMAT}
"

print_msg "Executing docker compose command on ${MSG_DIMMED_FORMAT}docker-compose.libpointmatcher.yaml${MSG_END_FORMAT} with command ${MSG_DIMMED_FORMAT}${DOCKER_COMPOSE_CMD_ARGS}${MSG_END_FORMAT}"
print_msg "Image tag ${MSG_DIMMED_FORMAT}${LPM_IMAGE_TAG}${MSG_END_FORMAT}"
#${MSG_DIMMED_FORMAT}$(printenv | grep -i -e LPM_ -e DEPENDENCIES_BASE_IMAGE -e BUILDKIT)${MSG_END_FORMAT}

## docker compose [-f <theComposeFile> ...] [options] [COMMAND] [ARGS...]
## docker compose [-f <theComposeFile> ...] build --no-cache --push
## docker compose build [OPTIONS] [SERVICE...]
## docker compose run [OPTIONS] SERVICE [COMMAND] [ARGS...]

show_and_execute_docker "compose -f docker-compose.libpointmatcher.yaml ${DOCKER_COMPOSE_CMD_ARGS}"


print_msg "Environment variables used by compose:\n
${MSG_DIMMED_FORMAT}    LIBPOINTMATCHER_VERSION=${LIBPOINTMATCHER_VERSION} ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    LIBPOINTMATCHER_CMAKE_BUILD_TYPE=${LIBPOINTMATCHER_CMAKE_BUILD_TYPE} ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    DEPENDENCIES_BASE_IMAGE=${DEPENDENCIES_BASE_IMAGE} ${MSG_END_FORMAT}
${MSG_DIMMED_FORMAT}    DEPENDENCIES_BASE_IMAGE_TAG=${DEPENDENCIES_BASE_IMAGE_TAG} ${MSG_END_FORMAT}"

print_formated_script_footer 'lpm_execute_compose.bash' "${LPM_LINE_CHAR_BUILDER_LVL2}"
# ====Teardown=====================================================================================================
cd "${TMP_CWD}"
