#!/bin/bash -i
#
#
set -e
#set -v

# ....Project root logic...........................................................................................
TMP_CWD=$(pwd)

if [[ "$(basename $(pwd))" != "build_system" ]]; then
  cd ../
fi


# ....Load environment variables from file.........................................................................
set -o allexport
source ./.env
source ./.env.prompt
set +o allexport

## skip GUI dialog by setting everything to default
#export DEBIAN_FRONTEND=noninteractive

# ....Helper function..............................................................................................
## import shell functions from Libpointmatcher-build-system utilities library
source ./function_library/prompt_utilities.bash
source ./function_library/terminal_splash.bash
source ./function_library/general_utilities.bash

## Set environment variable LPM_IMAGE_ARCHITECTURE
source ./lpm_utility_script/lpm_export_which_architecture.bash

# ====Begin========================================================================================================
SHOW_SPLASH_IDDU="${SHOW_SPLASH_IDDU:-true}"

if [[ "${SHOW_SPLASH_IDDU}" == 'true' ]]; then
  norlab_splash "${LPM_SPLASH_NAME}" "https://github.com/${LPM_LIBPOINTMATCHER_SRC_DOMAIN}/${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}"
fi

print_formated_script_header "lpm_install_doc_dependencies_ubuntu.bash (${LPM_IMAGE_ARCHITECTURE})" "${LPM_LINE_CHAR_INSTALLER}"


# ................................................................................................................
teamcity_service_msg_blockOpened "Install libpointmatcher documentation related dependencies"

## Package required when GENERATE_API_DOC flag is set to true
## Note: 'texlive-full' is ~6GB. 'doxygen-latex' is a slim version tailor made for doxygen code documentation task
sudo apt-get update &&
  sudo apt-get install --assume-yes \
    doxygen \
    doxygen-latex \
  && sudo rm -rf /var/lib/apt/lists/*

teamcity_service_msg_blockClosed

echo " " && print_msg_done "Libpointmatcher documentation related dependencies installed"
print_formated_script_footer "lpm_install_doc_dependencies_ubuntu.bash (${LPM_IMAGE_ARCHITECTURE})" "${LPM_LINE_CHAR_INSTALLER}"
# ====Teardown=====================================================================================================
cd "${TMP_CWD}"

