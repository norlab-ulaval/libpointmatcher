#!/bin/bash
#
# Docker entrypoint for development task
#
# Usage:
#   $ bash entrypoint.bash [<any-cmd>]
#
# Parameter
#   <any-cmd>      Optional command executed in a subprocess at the end of the entrypoint script.
#

ifconfig

cd "${NBS_LIB_INSTALL_PATH}"

pwd
tree -L 1

echo "dir size"
du -h --max-depth=2
echo

# ====Continue=====================================================================================================
exec "$@"

