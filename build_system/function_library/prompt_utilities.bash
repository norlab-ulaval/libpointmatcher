#!/bin/bash
#
# Library of function related to console formatting
#
# Requirement: This script must be sourced from directory 'build_system'
#
# Usage:
#   $ source ./function_library/prompt_utilities.bash
#
set -e

# ....Pre-condition................................................................................................
if [[ "$(basename $(pwd))" != "build_system" ]]; then
  echo -e "\nERROR: This script must be sourced from directory 'build_system'!\n cwd: $(pwd)"
  exit 1
fi

# ....Load environment variables from file.........................................................................
set -o allexport
source .env
source .env.prompt
set +o allexport


# =================================================================================================================
# Print formated message to prompt
#
# Usage:
#   $ _print_msg "<mesageType>" "<the message string>"
#
# Arguments:
#   <mesageType>          The message type: "BASE", "DONE", "WARNING", "AWAITING_INPUT"
#   <the message string>  The message string
# Outputs:
#   The formated message to stdout
# Returns:
#   none
# =================================================================================================================
function _print_msg_formater() {
  local MSG_TYPE=${1}
  local MSG=${2}


  if [[ "${MSG_TYPE}" == "BASE" ]]; then
      MSG_TYPE=${MSG_BASE}
  elif [[ "${MSG_TYPE}" == "DONE" ]]; then
      MSG_TYPE=${MSG_DONE}
  elif [[ "${MSG_TYPE}" == "WARNING" ]]; then
    MSG_TYPE=${MSG_WARNING}
  elif [[ "${MSG_TYPE}" == "AWAITING_INPUT" ]]; then
    MSG_TYPE=${MSG_AWAITING_INPUT}
  else
    echo "from ${FUNCNAME[1]} › ${FUNCNAME[0]}: Unrecognized msg type '${MSG_TYPE}' (!)"
    exit 1
  fi

#  echo ""
  echo -e "${MSG_TYPE} ${MSG}"
#  echo ""
}

function print_msg() {
    local MSG=${1}
    _print_msg_formater "BASE" "${MSG}"
}
function print_msg_done() {
    local MSG=${1}
    _print_msg_formater "DONE" "${MSG}"
}
function print_msg_warning() {
    local MSG=${1}
    _print_msg_formater "WARNING" "${MSG}"
}
function print_msg_awaiting_input() {
    local MSG=${1}
    _print_msg_formater "AWAITING_INPUT" "${MSG}"
}

# =================================================================================================================
# Print formated error message to prompt
#
# Usage:
#   $ print_msg_error_and_exit "<error msg string>"
#
# Globals:
#   Read 'TMP_CWD': Required to be set prior to function usage. eg: TMP_CWD=$(pwd)
# Arguments:
#   <error msg string>  The error message to print
# Outputs:
#   The formated error message to to stderr
# Returns:
#   none
# =================================================================================================================
function print_msg_error_and_exit() {
  local ERROR_MSG=$1

  echo ""
  echo -e "${MSG_ERROR}: ${ERROR_MSG}" >&2
  echo "Exiting now."
  echo ""
  cd "${TMP_CWD}"
  exit 1
}


# =================================================================================================================
# Draw horizontal line the entire width of the terminal
# Source: https://web.archive.org/web/20230402083320/http://wiki.bash-hackers.org/snipplets/print_horizontal_line#a_line_across_the_entire_width_of_the_terminal
#
# Usage:
#   $ draw_horizontal_line_across_the_terminal_window [<SYMBOL>]
#
# Arguments:
#   [<SYMBOL>] Symbole for the line, default to '='
# Outputs:
#   The screen wide line
# Returns:
#   none
# =================================================================================================================
function draw_horizontal_line_across_the_terminal_window() {
  local SYMBOL="${1:-=}"

  # (NICE TO HAVE) ToDo: validate >> var TERM is setup in Dockerfile.dependencies instead. Erase TPUT_FLAG logic.
#  if [[ ${TERM} == '' ]]; then
#      TPUT_FLAG='-T xterm'
##      TPUT_FLAG='-T xterm-256color'
#  fi
#  TPUT_FLAG='-T xterm'
  TPUT_FLAG=''

  printf '%*s\n' "${COLUMNS:-$(tput ${TPUT_FLAG} cols)}" '' | tr ' ' "${SYMBOL}"
}

# =================================================================================================================
# Print a formatted script header or footer
#
# Usage:
#   $ print_formated_script_header "<script name>" [<SYMBOL>]
#   $ print_formated_script_footer "<script name>" [<SYMBOL>]
#
# Arguments:
#   <script name>   The name of the script that is executing the function. Will be print in the header
#   [<SYMBOL>]      Symbole for the line, default to '='
# Outputs:
#   Print formated string to stdout
# Returns:
#   none
# =================================================================================================================
function print_formated_script_header() {
  local SCRIPT_NAME="${1}"
  local SYMBOL="${2:-=}"
  echo
  draw_horizontal_line_across_the_terminal_window "${SYMBOL}"
  echo -e "Starting ${MSG_DIMMED_FORMAT}${SCRIPT_NAME}${MSG_END_FORMAT}"
  echo
}

function print_formated_script_footer() {
  local SCRIPT_NAME="${1}"
  local SYMBOL="${2:-=}"
  echo
  echo -e "Completed ${MSG_DIMMED_FORMAT}${SCRIPT_NAME}${MSG_END_FORMAT}"
  draw_horizontal_line_across_the_terminal_window "${SYMBOL}"
  echo
}


# =================================================================================================================
# Print formated 'back to script' message
#
# Usage:
#   $ print_formated_back_to_script_msg "<script name>" [<SYMBOL>]
#
# Arguments:
#   <script name>   The name of the script that is executing the function. Will be print in the header
#   [<SYMBOL>]      Symbole for the line, default to '='
# Outputs:
#   Print formated string to stdout
# Returns:
#   none
# =================================================================================================================
function print_formated_back_to_script_msg() {
  local SCRIPT_NAME="${1}"
  local SYMBOL="${2:-=}"
  echo
  draw_horizontal_line_across_the_terminal_window -
  echo -e "Back to ${MSG_DIMMED_FORMAT}configure_teamcity_server.bash${MSG_END_FORMAT}"
  echo
}

# =================================================================================================================
# Print formatted file preview
#
# Usage:
#   $ print_formated_file_preview_begin "<file name>"
#   $ <the_command_which_echo_the_file>
#   $ print_formated_file_preview_end
#
# Arguments:
#   <file name>   The name of the file
# Outputs:
#   Print formated file preview to stdout
# Returns:
#   none
# =================================================================================================================
function print_formated_file_preview_begin() {
  local FILE_NAME="${1}"
  echo
  echo -e "${MSG_DIMMED_FORMAT}"
  draw_horizontal_line_across_the_terminal_window .
  echo "${FILE_NAME} <<< EOF"
}

function print_formated_file_preview_end() {
  echo "EOF"
  draw_horizontal_line_across_the_terminal_window .
  echo -e "${MSG_END_FORMAT}"
  echo
}

