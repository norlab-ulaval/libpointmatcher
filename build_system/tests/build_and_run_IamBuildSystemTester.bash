#!/bin/bash
#
# Build and run docker container for testing build system user script.
#   - Image:      lpm.ubuntu20.buildsystem.test
#   - Container:  IamBuildSystemTester
#
# Usage:
#   $ bash ./[build_system/[tests/]]build_and_run_IamBuildSystemTester.bash [<cmd+arg appended to docker run tail>]
#
set -e
#set -v
#set -x

DOCKER_CMD_ARGS=${*}

clear

TMP_CWD=$(pwd)

# ....Helper function..............................................................................
if [[ "$(basename $(pwd))" == "tests" ]]; then
  cd ../
fi


# ....path resolution logic........................................................................
_PATH_TO_SCRIPT="$(realpath "${BASH_SOURCE[0]}")"
LPM_ROOT_DIR="$(dirname "${_PATH_TO_SCRIPT}")/.."

set -o allexport && source .env && set +o allexport
#tree -L 1 $LPM_ROOT_DIR

# ....Helper function..............................................................................................
# import shell functions from utilities library
source "${LPM_ROOT_DIR}/build_system/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"

#source ./function_library/prompt_utilities.bash
#source ./function_library/general_utilities.bash

# ....Project root logic...........................................................................
if [[ "$(basename $(pwd))" == "build_system" ]]; then
  cd ../
fi

## ToDo: validate >> Logic in the next bloc was pushed in the Dockerfile. Check if it still make sense before deleting.
#if [[ -f LICENSE ]]; then
#  echo "Build context at project root '$(basename $(pwd))'"
#else
#  echo "Project root not reached! Current workdir: $(pwd)"
#  exit 1
#fi


# ====Begin========================================================================================
print_formated_script_header 'build_and_run_IamBuildSystemTester.bash' "${NBS_LINE_CHAR_TEST}"

# ....Build image..................................................................................
echo
print_msg "Building 'lpm.ubuntu20.buildsystem.test'"
# Note: Build context must be at repository root

#--no-cache
show_and_execute_docker "build -f build_system/tests/Dockerfile.build_system_test -t lpm.ubuntu20.buildsystem.test ."

# ....Run container................................................................................
print_msg "Starting 'IamBuildSystemTester'"

if [[ -n ${DOCKER_CMD_ARGS} ]]; then
  print_msg "Passing command ${MSG_DIMMED_FORMAT}\"${DOCKER_CMD_ARGS}\"${MSG_END_FORMAT} to docker run"
fi

show_and_execute_docker "run --name IamBuildSystemTester -t -i --rm lpm.ubuntu20.buildsystem.test ${DOCKER_CMD_ARGS}"

print_formated_script_footer 'build_and_run_IamBuildSystemTester.bash' "${NBS_LINE_CHAR_TEST}"
# ====Teardown=====================================================================================
cd "${TMP_CWD}"
