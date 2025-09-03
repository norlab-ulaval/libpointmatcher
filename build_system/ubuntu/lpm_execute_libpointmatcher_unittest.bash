#!/bin/bash
#
# Run Libpointmatcher unit test
#
# Usage:
#   $ bash lpm_execute_libpointmatcher_unittest.bash
#
# Notes:
#   The script propagate the utest exit code on exit

# ....Project root logic...........................................................................
TMP_CWD=$(pwd)

LPM_PATH=$(git rev-parse --show-toplevel)
cd "${LPM_PATH}/build_system" || exit 1

# ....Load environment variables from file.........................................................
set -o allexport
source .env
set +o allexport

# ....Helper function..............................................................................
# import shell functions from utilities library
N2ST_PATH=${N2ST_PATH:-"${LPM_PATH}/build_system/utilities/norlab-shell-script-tools"}
source "${N2ST_PATH}/import_norlab_shell_script_tools_lib.bash"

# ====Begin========================================================================================
n2st::print_formated_script_header 'lpm_execute_libpointmatcher_unittest.bash' ':'

cd "${NBS_LIB_INSTALL_PATH}/${NBS_REPOSITORY_NAME}/build" || exit 1

if [[ ${IS_TEAMCITY_RUN} == true ]] || [[ ${TEAMCITY_VERSION} ]]; then
  echo -e "##teamcity[testSuiteStarted name='gtest']"
  echo -e "##teamcity[testStarted name='gtest' captureStandardOutput='true']"
else
  n2st::print_msg "Starting Libpointmatcher GoogleTest unit-test"
fi

# .................................................................................................
sudo chmod +x utest/utest
utest/utest --path "${NBS_LIB_INSTALL_PATH}/${NBS_REPOSITORY_NAME}/examples/data/"
UTEST_EXIT_CODE=$?
# .................................................................................................

SUCCESS_MSG="Libpointmatcher GoogleTest unit-test completed successfully"
FAILURE_MSG="Libpointmatcher GoogleTest unit-test completed with error"

if [[ ${IS_TEAMCITY_RUN} == true ]] || [[ ${TEAMCITY_VERSION} ]]; then
  echo -e "##teamcity[testFinished name='gtest']"

  # Report message to build log
  if [[ ${UTEST_EXIT_CODE} == 0 ]]; then
    echo -e "##teamcity[message text='${MSG_BASE_TEAMCITY} ${SUCCESS_MSG}' status='NORMAL']"
  else
    echo -e "##teamcity[message text='${MSG_BASE_TEAMCITY} ${FAILURE_MSG}' errorDetails='$UTEST_EXIT_CODE' status='ERROR']" 1>&2
  fi

  echo -e "##teamcity[testSuiteFinished name='gtest']"
else

  if [[ ${UTEST_EXIT_CODE} == 0 ]]; then
    n2st::print_msg_done "${SUCCESS_MSG}"
  else
    n2st::print_msg_error "${FAILURE_MSG}"
  fi

fi

n2st::print_formated_script_footer 'lpm_execute_libpointmatcher_unittest.bash' ':'
# ====Teardown=====================================================================================
cd "${TMP_CWD}" || exit 1
exit $UTEST_EXIT_CODE
