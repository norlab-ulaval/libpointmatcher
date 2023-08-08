#!/bin/bash
#
# Run Libpointmatcher unit test
#
# Usage:
#   $ bash lpm_execute_libpointmatcher_unittest.bash
#
# Notes:
#   The script propagate the utest exit code on exit

# ....Project root logic...........................................................................................
TMP_CWD=$(pwd)

if [[ "$(basename $(pwd))" != "build_system" ]]; then
  cd ../
fi

# ....Load environment variables from file.........................................................................
set -o allexport
source .env
source .env.prompt
set +o allexport

# ....Load helper function.........................................................................................
# import shell functions from utilities library
source ./function_library/prompt_utilities.bash

# ====Begin========================================================================================================
print_formated_script_header 'lpm_execute_libpointmatcher_unittest.bash' ':'

cd "${LPM_INSTALLED_LIBRARIES_PATH}/${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}/build"

if [[ ${IS_TEAMCITY_RUN} == true ]] || [[ ${TEAMCITY_VERSION} ]]; then
  echo -e "##teamcity[testSuiteStarted name='gtest']"
  echo -e "##teamcity[testStarted name='gtest' captureStandardOutput='true']"
else
  print_msg "Starting Libpointmatcher GoogleTest unit-test"
fi

# .................................................................................................................
sudo chmod +x utest/utest
utest/utest --path "${LPM_INSTALLED_LIBRARIES_PATH}/${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}/examples/data/"
UTEST_EXIT_CODE=$?
# .................................................................................................................

SUCCESS_MSG="Libpointmatcher GoogleTest unit-test completed successfully"
FAILURE_MSG="Libpointmatcher GoogleTest unit-test completed with error"

if [[ ${IS_TEAMCITY_RUN} == true ]] || [[ ${TEAMCITY_VERSION} ]]; then
  echo -e "##teamcity[testFinished name='gtest']"

  # Report message to build log
  if [[ ${UTEST_EXIT_CODE} == 0 ]]; then
    echo -e "##teamcity[message text='${MSG_BASE_TEAMCITY} ${SUCCESS_MSG}' status='NORMAL']"
  else
    echo -e "##teamcity[message text='${MSG_BASE_TEAMCITY} ${FAILURE_MSG}' errorDetails='$UTEST_EXIT_CODE' status='ERROR']"
  fi

  echo -e "##teamcity[testSuiteFinished name='gtest']"
else

  if [[ ${UTEST_EXIT_CODE} == 0 ]]; then
    print_msg_done "${SUCCESS_MSG}"
  else
    print_msg_error "${FAILURE_MSG}"
  fi

fi

print_formated_script_footer 'lpm_execute_libpointmatcher_unittest.bash' ':'
# ====Teardown=====================================================================================================
cd "${TMP_CWD}"
exit $UTEST_EXIT_CODE
