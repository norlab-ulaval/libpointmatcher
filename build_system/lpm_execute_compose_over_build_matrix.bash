#!/bin/bash
#
# Execute build matrix on docker compose docker-compose.libpointmatcher.yaml
#
# Usage:
#   $ bash lpm_execute_compose_over_build_matrix.bash [<optional flag>] [-- <any docker cmd+arg>]
#
#   $ bash lpm_execute_compose_over_build_matrix.bash -- up --build --force-recreate
#
# Arguments:
#   [--libpointmatcher-version-build-matrix-override head]
#                     The libpointmatcher release tag. Override must be a single value
#                     (default to array sequence specified in .env.build_matrix)
#   [--os-name-build-matrix-override ubuntu]
#                     The operating system name. Override must be a single value
#                     (default to array sequence specified in .env.build_matrix)
#   [--os-version-build-matrix-override jammy]
#                     Named operating system version. Override must be a single value
#                     (default to array sequence specified in .env.build_matrix)
#   [-- <any docker cmd+arg>]
#                     Any argument passed after '--' will be passed to docker compose as docker command and arguments
#                       (default: see DOCKER_COMPOSE_CMD_ARGS)
#   [-h, --help]      Get help
#
set -e
#set -v
#set -x

# ....Default......................................................................................................
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
# import shell functions from Libpointmatcher-build-system utilities library
source ./function_library/prompt_utilities.bash
source ./function_library/general_utilities.bash
source ./function_library/terminal_splash.bash

function print_help_in_terminal() {
  echo -e "\n
\$ ${0} [<optional flag>] [-- <any docker cmd+arg>]
  \033[1m
    <optional argument>:\033[0m
      -h, --help          Get help
      --libpointmatcher-version-build-matrix-override head
                          The libpointmatcher release tag. Override must be a single value
                          (default to array sequence specified in .env.build_matrix)
      --os-name-build-matrix-override ubuntu
                          The operating system name. Override must be a single value
                          (default to array sequence specified in .env.build_matrix)
      --os-version-build-matrix-override jammy
                          Named operating system version. Override must be a single value
                          (default to array sequence specified in .env.build_matrix)

  \033[1m
    [-- <any docker cmd+arg>]\033[0m                 Any argument passed after '--' will be passed to docker compose as docker
                                              command and arguments (default to '${DOCKER_COMPOSE_CMD_ARGS}')
"
}

# ====Begin========================================================================================================
norlab_splash "${LPM_BUILD_SYSTEM_SPLASH_NAME}" "https://github.com/${LPM_LIBPOINTMATCHER_SRC_DOMAIN}/${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}"

print_formated_script_header 'lpm_execute_compose_over_build_matrix.bash' "${LPM_LINE_CHAR_BUILDER_LVL1}"

# ....Script command line flags....................................................................................

while [ $# -gt 0 ]; do

    echo -e "'\$*' before: ${MSG_DIMMED_FORMAT}$*${MSG_END_FORMAT}" # ToDo: on task end >> delete this line ←
    echo -e "\$1: ${1}    \$2: ${2}" # ToDo: on task end >> delete this line ←

  case $1 in
  --libpointmatcher-version-build-matrix-override)
    unset LPM_LIBPOINTMATCHER_VERSIONS
    LPM_LIBPOINTMATCHER_VERSIONS=("$2")
    shift # Remove argument (--libpointmatcher-version-build-matrix-override)
    shift # Remove argument value
    ;;
  --os-name-build-matrix-override)
    unset LPM_SUPPORTED_OS
    LPM_SUPPORTED_OS=("$2")
    shift # Remove argument (--os-name-build-matrix-override)
    shift # Remove argument value
    ;;
  --ubuntu-version-build-matrix-override)
    unset LPM_UBUNTU_SUPPORTED_VERSIONS
    LPM_UBUNTU_SUPPORTED_VERSIONS=("$2")
    shift # Remove argument (--ubuntu-version-build-matrix-override)
    shift # Remove argument value
    ;;
  --osx-version-build-matrix-override)
    unset LPM_OSX_SUPPORTED_VERSIONS
    LPM_OSX_SUPPORTED_VERSIONS=("$2")
    shift # Remove argument (--osx-version-build-matrix-override)
    shift # Remove argument value
    ;;
#  --job-id)
#      LPM_JOB_ID="${2}"
#      shift # Remove argument (--job-id)
#      shift # Remove argument value
#      ;;
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
#
## ToDo: on task end >> delete next bloc ↓↓
#echo -e " ${MSG_DIMMED_FORMAT}
#LPM_LIBPOINTMATCHER_VERSIONS=${LPM_LIBPOINTMATCHER_VERSIONS[*]}
#LPM_SUPPORTED_OS=${LPM_SUPPORTED_OS[*]}
#LPM_UBUNTU_SUPPORTED_VERSIONS=${LPM_UBUNTU_SUPPORTED_VERSIONS[*]}
#LPM_OSX_SUPPORTED_VERSIONS=${LPM_OSX_SUPPORTED_VERSIONS[*]}
#DOCKER_COMPOSE_CMD_ARGS=${DOCKER_COMPOSE_CMD_ARGS}
#${MSG_END_FORMAT} "


## ..................................................................................................................
## Set environment variable LPM_IMAGE_ARCHITECTURE
#source ./lpm_utility_script/lpm_export_which_architecture.bash


# ..................................................................................................................
print_msg "Build images specified in ${MSG_DIMMED_FORMAT}'docker-compose.libpointmatcher.yaml'${MSG_END_FORMAT} following the buildmatrix specified in ${MSG_DIMMED_FORMAT}.env.build_matrix${MSG_END_FORMAT}"

# Freeze build matrix env variable to prevent override by lpm_execute_compose.bash when reloading .env/build_matrix
FREEZED_LPM_LIBPOINTMATCHER_VERSIONS=("${LPM_LIBPOINTMATCHER_VERSIONS[@]}")
FREEZED_LPM_SUPPORTED_OS=("${LPM_SUPPORTED_OS[@]}")
FREEZED_LPM_UBUNTU_SUPPORTED_VERSIONS=("${LPM_UBUNTU_SUPPORTED_VERSIONS[@]}")
FREEZED_LPM_OSX_SUPPORTED_VERSIONS=("${LPM_OSX_SUPPORTED_VERSIONS[@]}")

# Note: EACH_LPM_VERSION is used for container labeling and to fetch the repo at release tag (todo ref task NMO-252)
# (Priority) ToDo: implement other OS support (ref task NMO-213 OsX arm64-Darwin and NMO-210 OsX x86 CD components)
for EACH_LPM_VERSION in "${FREEZED_LPM_LIBPOINTMATCHER_VERSIONS[@]}"; do

  for EACH_OS_NAME in "${FREEZED_LPM_SUPPORTED_OS[@]}"; do
    unset CRAWL_OS_VERSIONS

    if [[ ${EACH_OS_NAME} == 'ubuntu' ]]; then
      CRAWL_OS_VERSIONS=("${FREEZED_LPM_UBUNTU_SUPPORTED_VERSIONS[@]}")
    elif [[ ${EACH_OS_NAME} == 'osx' ]]; then
      CRAWL_OS_VERSIONS=("${FREEZED_LPM_OSX_SUPPORTED_VERSIONS[@]}")
    else
      print_msg_error_and_exit "${EACH_OS_NAME} no supported!"
    fi

    for EACH_OS_VERSION in "${CRAWL_OS_VERSIONS[@]}"; do

#      export LPM_JOB_ID=${LPM_JOB_ID}

      SHOW_SPLASH_EC='false'

      source ./lpm_execute_compose.bash --libpointmatcher-version "${EACH_LPM_VERSION}" \
                                        --os-name "${EACH_OS_NAME}" \
                                        --os-version "${EACH_OS_VERSION}" \
                                        -- "${DOCKER_COMPOSE_CMD_ARGS}"
#                                        --job-id "${LPM_JOB_ID}" \

      # Collect image tags exported by lpm_execute_compose.bash
      IMAGE_TAG_CRAWLED=("${IMAGE_TAG_CRAWLED[@]}" "${LPM_IMAGE_TAG}")

    done
  done
done


# ToDo: on task end >> delete next bloc ↓↓
echo -e " ${MSG_DIMMED_FORMAT}
FREEZED_LPM_LIBPOINTMATCHER_VERSIONS=(${FREEZED_LPM_LIBPOINTMATCHER_VERSIONS[*]})
FREEZED_LPM_SUPPORTED_OS=(${FREEZED_LPM_SUPPORTED_OS[*]})
FREEZED_LPM_UBUNTU_SUPPORTED_VERSIONS=(${FREEZED_LPM_UBUNTU_SUPPORTED_VERSIONS[*]})
FREEZED_LPM_OSX_SUPPORTED_VERSIONS=(${FREEZED_LPM_OSX_SUPPORTED_VERSIONS[*]})
${MSG_END_FORMAT} "



print_msg_done "FINAL › Build matrix completed with command
${MSG_DIMMED_FORMAT}
      $ docker compose -f docker-compose.libpointmatcher.yaml ${DOCKER_COMPOSE_CMD_ARGS}
${MSG_END_FORMAT}
Tag crawled:
${MSG_DIMMED_FORMAT}"
for tag in "${IMAGE_TAG_CRAWLED[@]}" ; do
    echo -e "   ${tag}"
done
echo -e "${MSG_END_FORMAT}"

print_formated_script_footer 'lpm_execute_compose_over_build_matrix.bash' "${LPM_LINE_CHAR_BUILDER_LVL1}"
# ====Teardown=====================================================================================================
cd "${TMP_CWD}"
