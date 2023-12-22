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
  load "${SRC_CODE_PATH}/build_system/tests/tests_bats/bats_helper_functions"
  #load "${BATS_HELPER_PATH}/bats-detik/load" # << Kubernetes support
else
  echo -e "\n[\033[1;31mERROR\033[0m] $0 path to bats-core helper library unreachable at \"${BATS_HELPER_PATH}\"!" 1>&2
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi

# ====Setup========================================================================================

TESTED_FILE="lpm_crawl_libpointmatcher_build_matrix.bleeding.bash"
TESTED_FILE_PATH="./build_system/"
COMPOSE_FILE="docker-compose.libpointmatcher.yaml"
DOTENV_BUILD_MATRIX="${SRC_CODE_PATH}"/build_system/.env.build_matrix.libpointmatcher.bleeding
DOTENV_BUILD_MATRIX_NAME=$( basename "${DOTENV_BUILD_MATRIX}" )

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

@test "${TESTED_FILE} › docker image › execute ok › expect pass" {
#  skip "tmp mute" # ToDo: on task end >> delete this line ←

  run bash "${TESTED_FILE}" "${DOTENV_BUILD_MATRIX}" --fail-fast -- build
#  OPTIONS=( "${DOTENV_BUILD_MATRIX}" "--ubuntu-version-build-matrix-override jammy" "--fail-fast" "--" "config --quiet" )
#  run bash "${TESTED_FILE}" "${OPTIONS[@]}"

  assert_success
  assert_output --regexp .*"Starting".*"${TESTED_FILE}".*"\[NBS\]".*"Build images specified in".*"'${COMPOSE_FILE}'".*"following".*"${DOTENV_BUILD_MATRIX_NAME}"
  assert_output --regexp .*"\[NBS done\]".*"FINAL › Build matrix completed with command".*"\$".*"docker compose -f ${COMPOSE_FILE}"
#  build --dry-run
  assert_output --regexp "Status of tag crawled:".*"Pass".*"› latest".*"Completed".*"${TESTED_FILE}".*
}

# ....Test --help flag related logic...............................................................

@test "${TESTED_FILE} › --help as first argument › execute ok › expect pass" {
#  skip "tmp dev" # ToDo: on task end >> delete this line ←
  run bash "${TESTED_FILE}" --help
  test_generic_help_flag_logic
}

@test "${TESTED_FILE} › second arg: --help › execute ok › expect pass" {
#  skip "tmp dev" # ToDo: on task end >> delete this line ←

  run bash "${TESTED_FILE}" --fail-fast --help
  test_generic_help_flag_logic
}

## ToDo: implement >> test for IS_TEAMCITY_RUN==true casses
## (NICE TO HAVE) ToDo: implement >> test for python intsall casses with regard to distribution
