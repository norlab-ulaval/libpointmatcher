#!/bin/bash

ifconfig

cd "${LPM_INSTALLED_LIBRARIES_PATH}"
#cd "${LPM_INSTALLED_LIBRARIES_PATH}/${LPM_LIBPOINTMATCHER_SRC_REPO_NAME}"

pwd
tree -L 1

echo "dir size"
du -h --max-depth=2
echo

# ====Continue=====================================================================================================
exec "${@}"

