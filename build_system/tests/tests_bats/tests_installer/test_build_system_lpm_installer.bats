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
TESTED_FILE3="lpm_install_dependencies_libnabo_ubuntu.bash"
TESTED_FILE4="lpm_install_libpointmatcher_ubuntu.bash"
TESTED_FILE_PATH3="./build_system/ubuntu"

# executed once before starting the first test (valide for all test in that file)
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR

  ## Uncomment the following for debug, the ">&3" is for printing bats msg to stdin
#  pwd >&3 && tree -L 1 -a -hug >&3
#  printenv >&3
}

# executed before each test
setup() {
  echo "(Bats) Source project .env"
  set -o allexport
  source "${BATS_DOCKER_WORKDIR:?err}/build_system/.env"
  set +o allexport

  cd "$TESTED_FILE_PATH" || exit 1
}

# ====Teardown=====================================================================================

# executed after each test
teardown() {
  echo "(Bats) Reset install directory"
  rm -rf "${NBS_LIB_INSTALL_PATH}"

#  bats_print_run_env_variable_on_error
}

## executed once after finishing the last test (valide for all test in that file)
#teardown_file() {
#}

# ====Test casses==================================================================================
# ....Build system install script..................................................................

@test "Dependencies    | run $TESTED_FILE3 › expect pass" {
  cd "${BATS_DOCKER_WORKDIR}"

  run bash ./${TESTED_FILE_PATH3}/$TESTED_FILE3 --test-run

  assert_success

  assert_output --regexp .*"\[".*"LPM".*"\]".*"Install Libpointmatcher dependencies".*"Libnabo"

  assert_output --regexp .*"\[".*"LPM".*"\]".*"Execute".*"cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo /opt/percep3d_libraries/libnabo"
  refute_output --regexp .*"\[".*"LPM".*"\]".*"Execute".*"cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo -D CMAKE_INSTALL_PREFIX=/opt/percep3d_libraries /opt/percep3d_libraries/libnabo"

  assert_output --regexp .*"\[".*"LPM done".*"\]".*"Libpointmatcher dependencies installed"
}

@test "Dependencies    | run $TESTED_FILE3 (read APPEND_TO_CMAKE_FLAG ok) › expect pass" {
  cd "${BATS_DOCKER_WORKDIR}"

  export APPEND_TO_CMAKE_FLAG=( -D CMAKE_INSTALL_PREFIX=/opt/test_dir )
  run source ./${TESTED_FILE_PATH3}/$TESTED_FILE3 --test-run

  assert_success

  assert_output --regexp .*"\[".*"LPM".*"\]".*"Install Libpointmatcher dependencies".*"Libnabo"

  assert_output --regexp .*"\[".*"LPM".*"\]".*"Execute".*"cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo -D CMAKE_INSTALL_PREFIX=/opt/test_dir /opt/percep3d_libraries/libnabo"
  refute_output --regexp .*"\[".*"LPM".*"\]".*"Execute".*"cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo /opt/percep3d_libraries/libnabo"

  assert_output --regexp .*"\[".*"LPM done".*"\]".*"Libpointmatcher dependencies installed"
}

@test "Libpointmatcher | run $TESTED_FILE4 (default) › expect pass" {
  cd "${BATS_DOCKER_WORKDIR}"

  run bash "./${TESTED_FILE_PATH3}/$TESTED_FILE4" --test-run

  assert_success

  assert_output --regexp .*"\[".*"LPM".*"\]".*"Install libpointmatcher"

  assert_output --regexp .*"\[".*"LPM".*"\]".*"Execute".*"cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo -D BUILD_TESTS=FALSE -D GENERATE_API_DOC=FALSE /opt/percep3d_libraries/libpointmatcher"
  refute_output --regexp .*"\[".*"LPM".*"\]".*"Execute".*"cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo -D BUILD_TESTS=FALSE -D GENERATE_API_DOC=FALSE -D CMAKE_INSTALL_PREFIX=/opt/percep3d_libraries /opt/percep3d_libraries/libpointmatcher"

  assert_output --regexp .*"\[".*"LPM done".*"\]".*"libpointmatcher installed successfully at".*
}

@test "Libpointmatcher | run $TESTED_FILE4 (default) (read APPEND_TO_CMAKE_FLAG ok) › expect pass" {
  cd "${BATS_DOCKER_WORKDIR}"

  export APPEND_TO_CMAKE_FLAG=( -D CMAKE_INSTALL_PREFIX=/opt/test_dir )
  run source "./${TESTED_FILE_PATH3}/$TESTED_FILE4" --test-run

  assert_success

  assert_output --regexp .*"\[".*"LPM".*"\]".*"Install libpointmatcher"

  assert_output --regexp .*"\[".*"LPM".*"\]".*"Execute".*"cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo -D BUILD_TESTS=FALSE -D GENERATE_API_DOC=FALSE -D CMAKE_INSTALL_PREFIX=/opt/test_dir /opt/percep3d_libraries/libpointmatcher"
  refute_output --regexp .*"\[".*"LPM".*"\]".*"Execute".*"cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo -D BUILD_TESTS=FALSE -D GENERATE_API_DOC=FALSE /opt/percep3d_libraries/libpointmatcher"

  assert_output --regexp .*"\[".*"LPM done".*"\]".*"libpointmatcher installed successfully at".*
}


@test "Libpointmatcher | run $TESTED_FILE4 (argument check) › expect pass" {
  cd "${BATS_DOCKER_WORKDIR}"

  run bash "./${TESTED_FILE_PATH3}/$TESTED_FILE4" --test-run \
             --install-path /opt/test_dir \
             --repository-version 1.4.0 \
             --compile-test \
             --generate-doc \
             --cmake-build-type Release

  assert_success
  assert_output --regexp .*"\[".*"LPM".*"\]".*"Install libpointmatcher"

  assert_output --partial "switching to 'tags/1.4.0'."

  assert_output --regexp .*"\[".*"LPM".*"\]".*"Repository checkout at tag 1.4.0"

  assert_output --regexp .*"\[".*"LPM".*"\]".*"Execute".*"cmake -D CMAKE_BUILD_TYPE=Release -D BUILD_TESTS=TRUE -D GENERATE_API_DOC=TRUE /opt/test_dir/libpointmatcher"
  refute_output --regexp .*"\[".*"LPM".*"\]".*"Execute".*"cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo -D BUILD_TESTS=FALSE -D GENERATE_API_DOC=FALSE /opt/percep3d_libraries/libpointmatcher"


  assert_output --regexp .*"\[".*"LPM done".*"\]".*"libpointmatcher installed successfully at".*

}
