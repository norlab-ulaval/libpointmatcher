#!/bin/bash

function test_generic_help_flag_logic() {
  assert_success

  assert_output --regexp .*"Starting".*"${TESTED_FILE}".*"\$".*"${TESTED_FILE} \[--help\] <.env.build_matrix.*> \[<optional flag>\] \[-- <any docker cmd\+arg>\]".*"Mandatory argument:".*"<.env.build_matrix.*>".*"Optional arguments:".*"-h, --help".*"--docker-debug-logs".*"--fail-fast"

  refute_output --regexp .*"Starting".*"${TESTED_FILE}".*"\[NBS\]".*"Build images specified in".*"'${COMPOSE_FILE}'".*"following".*"${DOTENV_BUILD_MATRIX_NAME}"
}
