#!/bin/bash
# =================================================================================================
# Execute '<my-superproject>' repo shell script tests via 'norlab-shell-script-tools' library
#
#   1. ToDo: Copy this script somewhere in your superproject
#   2. (optional) ToDo: set N2ST_PATH="<path/to/submodule/norlab-shell-script-tools>"
#   3. Execute the script
#
# Usage:
#  $ bash run_bats_core_test_in_n2st.bash ['<test-directory>[/<this-bats-test-file.bats>]' ['<image-distro>']]
#
# Arguments:
#   - ['<test-directory>']     The directory from which to start test, default to 'tests'
#   - ['<test-directory>/<this-bats-test-file.bats>']  A specific bats file to run, default will
#                                                      run all bats file in the test directory
#
# Globals:
#   Read N2ST_PATH    Default to "./utilities/norlab-shell-script-tools"
#
# =================================================================================================
OPTIONS="$@"

N2ST_PATH="build_system/utilities/norlab-shell-script-tools"

function n2st::run_n2st_testsing_tools(){
  local TMP_CWD
  TMP_CWD=$(pwd)

  # ....Project root logic.........................................................................
  LPM_PATH=$(git rev-parse --show-toplevel)
  N2ST_PATH=${N2ST_PATH:-"./utilities/norlab-shell-script-tools"}

  # ....Execute N2ST run_bats_tests_in_docker.bash.................................................
  cd "$LPM_PATH"
  bash "${N2ST_PATH}/tests/bats_testing_tools/run_bats_tests_in_docker.bash" "$OPTIONS"

  # ....Teardown...................................................................................
  cd "$TMP_CWD"
  }

n2st::run_n2st_testsing_tools

