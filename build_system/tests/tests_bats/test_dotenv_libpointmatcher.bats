#!/usr/bin/env bats
#
# Usage in docker container
#   $ REPO_ROOT=$(pwd) && RUN_TESTS_IN_DIR='tests'
#   $ docker run -it --rm -v "$REPO_ROOT:/code" bats/bats:latest "$RUN_TESTS_IN_DIR"
#
#   Note: "/code" is the working directory in the bats official image
#
# bats-core ref:
#   - https://bats-core.readthedocs.io/en/stable/tutorial.html
#   - https://bats-core.readthedocs.io/en/stable/writing-tests.html
#   - https://opensource.com/article/19/2/testing-bash-bats
#       ↳ https://github.com/dmlond/how_to_bats/blob/master/test/build.bats
#
# Helper library: 
#   - https://github.com/bats-core/bats-assert
#   - https://github.com/bats-core/bats-support
#   - https://github.com/bats-core/bats-file
#

BATS_HELPER_PATH=/usr/lib/bats
if [[ -d ${BATS_HELPER_PATH} ]]; then
  load "${BATS_HELPER_PATH}/bats-support/load"
  load "${BATS_HELPER_PATH}/bats-assert/load"
  load "${BATS_HELPER_PATH}/bats-file/load"
  load "${SRC_CODE_PATH}/${N2ST_BATS_TESTING_TOOLS_RELATIVE_PATH}/bats_helper_functions"
  #load "${BATS_HELPER_PATH}/bats-detik/load" # << Kubernetes support
else
  echo -e "\n[\033[1;31mERROR\033[0m] $0 path to bats-core helper library unreachable at \"${BATS_HELPER_PATH}\"!" 1>&2
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi

# ====Setup========================================================================================

DOTENV_FILE=".env.libpointmatcher"
TESTED_FILE_PATH="."

# executed once before starting the first test (valide for all test in that file)
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR

  ## Uncomment the following for debug, the ">&3" is for printing bats msg to stdin
#  pwd >&3 && tree -L 1 -a -hug >&3
#  printenv >&3
}

# executed before each test
setup() {
  cd "$TESTED_FILE_PATH" || exit 1
}

# ====Teardown=====================================================================================

# executed after each test
teardown() {
  bats_print_run_env_variable_on_error
}

## executed once after finishing the last test (valide for all test in that file)
#teardown_file() {
#}

# ====Test casses==================================================================================
@test "${DOTENV_FILE} › Env variables set ok" {
  # ....Pre-condition..............................................................................
  assert_empty "${PROJECT_PROMPT_NAME}"
  assert_empty "${PROJECT_GIT_REMOTE_URL}"
  assert_empty "${PROJECT_GIT_NAME}"
  assert_empty "${PROJECT_SRC_NAME}"
  assert_empty "${PROJECT_PATH}"

  assert_empty "${LPM_PROMPT_NAME}"
  assert_empty "${LPM_GIT_REMOTE_URL}"
  assert_empty "${LPM_GIT_NAME}"
  assert_empty "${LPM_PATH}"
  assert_empty "${LPM_SRC_NAME}"

  assert_empty "${LPM_BUILD_SYSTEM_PATH}"
  assert_empty "${N2ST_PATH}"
  assert_empty "${NBS_PATH}"

  # ....Source .env.project........................................................................
  set -o allexport
  source "${BATS_DOCKER_WORKDIR:?err}/${DOTENV_FILE}"
  set +o allexport

#  printenv | grep -e 'CONTAINER_PROJECT_' -e 'PROJECT_' >&3

  # ....Tests......................................................................................
  assert_equal "${PROJECT_PROMPT_NAME}" "LPM"
  assert_regex "${PROJECT_GIT_REMOTE_URL}" "https://github.com/norlab-ulaval/libpointmatcher"'(".git")?'
  assert_equal "${PROJECT_GIT_NAME}" "libpointmatcher"
  assert_equal "${PROJECT_SRC_NAME}" "libpointmatcher"
  assert_equal "${PROJECT_PATH}" "/code/libpointmatcher"

  assert_equal "${LPM_PROMPT_NAME}" "LPM"
  assert_regex "${LPM_GIT_REMOTE_URL}" "https://github.com/norlab-ulaval/libpointmatcher"'(".git")?'
  assert_equal "${LPM_GIT_NAME}" "libpointmatcher"
  assert_equal "${LPM_SRC_NAME}" "libpointmatcher"
  assert_equal "${LPM_PATH}" "/code/libpointmatcher"

  assert_equal "${LPM_BUILD_SYSTEM_PATH}" "/code/libpointmatcher/build_system"
  assert_equal "${N2ST_PATH}" "/code/libpointmatcher/build_system/utilities/norlab-shell-script-tools"
  assert_equal "${NBS_PATH}" "/code/libpointmatcher/build_system/utilities/norlab-build-system"
}
