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
# import shell functions from Libpointmatcher-build-system utilities library
source ./function_library/prompt_utilities.bash
source ./function_library/terminal_splash.bash

# Set environment variable LPM_IMAGE_ARCHITECTURE
source ./lpm_utility_script/lpm_export_which_architecture.bash

# ====Begin========================================================================================================
SHOW_SPLASH_IDDU="${SHOW_SPLASH_IDDU:-true}"

if [[ "${SHOW_SPLASH_IDDU}" == 'true' ]]; then
  norlab_splash "${LPM_SPLASH_NAME}" "https://github.com/${LPM_LIBPOINTMATCHER_SRC_DOMAIN}/${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}"
fi

print_formated_script_header "lpm_install_doc_dependencies_ubuntu.bash (${LPM_IMAGE_ARCHITECTURE})" "${LPM_LINE_CHAR_INSTALLER}"


# ................................................................................................................
echo
print_msg "Install Libpointmatcher documentation related dependencies"
echo

# Package required when GENERATE_API_DOC flag is set to true
sudo apt-get update &&
  sudo apt-get install --assume-yes \
    doxygen \
    doxygen-latex \
  && sudo rm -rf /var/lib/apt/lists/*


print_msg_done "Libpointmatcher documentation related dependencies installed"
print_formated_script_footer "lpm_install_doc_dependencies_ubuntu.bash (${LPM_IMAGE_ARCHITECTURE})" "${LPM_LINE_CHAR_INSTALLER}"
# ====Teardown=====================================================================================================
cd "${TMP_CWD}"

