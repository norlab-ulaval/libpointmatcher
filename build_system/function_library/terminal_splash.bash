#!/bin/bash
#
# Requirement: This script must be executed from project root 'NorLab-TeamCity-Server-infrastructure'
#
# Usage:
#   $ source function_library/terminal_splash.bash
#
#set -e # (NICE TO HAVE) ToDo: fixme!! >> script exit if "set -e" is enable
#set -v

# ....Pre-condition................................................................................................
if [[ "$(basename $(pwd))" != "build_system" ]]; then
  echo -e "\nERROR: This script must be sourced from directory 'build_system'!\n cwd: $(pwd)"
  exit 1
fi


# =================================================================================================================
# Dynamic printf centering tool. Centering based on the terminal screen width at runtime.
#
# Usage:
#   $ source function_library/terminal_splash.bash
#   $ echo_centering_str <theString> [<theStyle> [thePadCharacter]]
#
#   $ echo_centering_str "···•· ${TITLE} ··•••" "\033[1;37m" "\033[0m·"
#
# Globals: 
#   none
# Arguments:
#   <theString>           The string to center
#   <theStyle>            The style appended at the begining of the line (default to "\033[1;37m")
#   <thePadCharacter>     The padding character to use (default to "\033[0m·")
# Outputs:
#   Output the line to STDOUT
# Returns:
#   none
# =================================================================================================================
function echo_centering_str() {
  local the_str=${1:?'Missing a mandatory parameter error'}
  local the_style="${2:-""}"
  local the_pad_cha="${3:-" "}"
  local str_len=${#the_str}

  ## ToDo: on task end >> mute next bloc ↓↓
  #echo "\$TERM=${TERM}"
  #echo "\$COLUMNS=${COLUMNS}"

  # Ref https://bash.cyberciti.biz/guide/$TERM_variable
  TPUT_FLAG=''
  if [[ -z ${TERM} ]]; then
    TPUT_FLAG='-T xterm-256color'
  elif [[ ${TERM} == dumb ]]; then
    # "dumb" is the one set on TeamCity Agent
    TPUT_FLAG='-T xterm-256color'
  fi

  # (NICE TO HAVE) ToDo:
  #     - var TERM should be setup in Dockerfile.dependencies.
  #     - print a warning message if TERM is not set

  local terminal_width
#  terminal_width=$(tput ${TPUT_FLAG} cols)
  terminal_width="${COLUMNS:-$(tput ${TPUT_FLAG} cols)}"
  local total_padding_len=$(( $terminal_width - $str_len ))
  local single_side_padding_len=$(( $total_padding_len / 2 ))
  local pad
  pad=$(printf -- "$the_pad_cha%.0s" $(seq $single_side_padding_len))
  printf -- "${pad}${the_style}${the_str}\033[0m${pad}\n"                        # <-- Quick hack
}


# =================================================================================================================
# Terminal splash screen
#
# Credit: ASCII art generated using image generator at https://asciiart.club
#
# Usage:
#
#   $ source function_library/terminal_splash.bash
#   $ norlab_splash [title [url]]
#
#   $ norlab_splash "NorLab" "https://github.com/norlab-ulaval"
#
# Globals:
#   none
# Arguments:
#   <title>   The title printed in the center of the splash screen (default 'NorLab')
#   <url>     The url printed at the bottom of the splash screen (default 'https://github.com/norlab-ulaval')
# Outputs:
#   Output the splash screen to STDOUT
# Returns:
#   none
#
# References:
#   - Bash tips: Colors and formatting (ANSI/VT100 Control sequences):
#     https://misc.flogisoft.com/bash/tip_colors_and_formatting#bash_tipscolors_and_formatting_ansivt100_control_sequences
#   - ASCII art generated using image generator at https://asciiart.club
#
# Dev workflow: run the following command
#
#   $ source build_system/function_library/terminal_splash.bash \
#      && norlab_splash "NorLab" "https://github.com/norlab-ulaval"
#
# =================================================================================================================
function norlab_splash() {
  local TITLE=${1:-'NorLab'}
  local OPTIONAL_URL=${2:-'https://github.com/norlab-ulaval'}

  # iceboxed: Background in white
  #     >> would be nice but result might be inconsistent across terminal type >> Not worth the trouble.
  # GRAY_FG_WHITE_BG="\e[0;37;107m"

  # Foreground colors
  #   - 2=Dim
  #   - 37=light gray
  #   - 90=dark gray
  #   - 30=black
  #   - 97=white
  # Formatting
  #   - 1=Bold/bright
  #   - 2=Dim
  #   - 4=underline
  local SNOW_FG=97
  local TITLE_FG=97
  local URL_FG=37
  local SNOW_FORMATTING=2
  local TITLE_FORMATTING=1
  local URL_FORMATTING=2

  # ToDo: validate
  local SS='  '
  if [[ ${IS_TEAMCITY_RUN} == true ]] || [[ ${TEAMCITY_VERSION} ]]; then
    SS=''
  fi

  echo " "
  echo " "
  echo " "
  echo_centering_str "${SS}⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣶⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀" "\033[${SNOW_FORMATTING};${SNOW_FG}m" " "
  echo_centering_str "${SS}⠀⠀⠀⠀⠀⠀⠀⢿⣷⣼⣿⣤⣿⡗⠀⠀⠀⠀⠀⠀⠀⠀" "\033[${SNOW_FORMATTING};${SNOW_FG}m" " "
  echo_centering_str "${SS}⠀⢀⣤⡀⣿⣿⠀⠀⠉⣿⣿⡿⠁⠀⠀⣿⡟⣀⣤⠀⠀⠀" "\033[${SNOW_FORMATTING};${SNOW_FG}m" " "
  echo_centering_str "${SS}⠀⠀⠙⣻⣿⣿⣧⠀⠀⢸⣿⠀⠀⢀⣿⣿⣿⣟⠉⠀⠀⠀" "\033[${SNOW_FORMATTING};${SNOW_FG}m" " "
  echo_centering_str "${SS}⠀⠘⠛⠛⠉⠉⠙⠿⣿⣾⣿⣷⣿⠟⠉⠉⠙⠛⠛⠀⠀⠀" "\033[${SNOW_FORMATTING};${SNOW_FG}m" " "
  echo_centering_str "···•· ${TITLE} ··•••" "\033[${TITLE_FORMATTING};${TITLE_FG}m" "\033[0m·"
  echo_centering_str "${SS}⠀⢠⣶⣤⣄⣀⣤⣶⣿⢿⣿⢿⣿⣶⣄⣀⣤⣤⣶⠀⠀⠀" "\033[${SNOW_FORMATTING};${SNOW_FG}m" " "
  echo_centering_str "${SS}⠀⠀⣨⣿⣿⣿⡟⠁⠀⢸⣿⠀⠀⠉⣿⣿⣿⣯⣀⠀⠀⠀" "\033[${SNOW_FORMATTING};${SNOW_FG}m" " "
  echo_centering_str "${SS}⠀⠈⠛⠁⣿⣿⢀⠀⣠⣿⣿⣷⡀⠀⠈⣿⣧⠉⠛⢀⠀⠀" "\033[${SNOW_FORMATTING};${SNOW_FG}m" " "
  echo_centering_str "${SS}⠀⠀⠀⠀⠀⠀⠀⣾⡿⢻⣿⠙⣿⡷⠀⠈⠀⠀⠀⠀⠀⠀" "\033[${SNOW_FORMATTING};${SNOW_FG}m" " "
  echo_centering_str "${SS}⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⠛⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀" "\033[${SNOW_FORMATTING};${SNOW_FG}m" " "
  echo " "
  echo_centering_str "https://norlab.ulaval.ca" "\033[${URL_FORMATTING};${URL_FG}m" " "
  echo_centering_str "${OPTIONAL_URL}" "\033[${URL_FORMATTING};${URL_FG}m" " "
  echo " "
  echo " "

}
