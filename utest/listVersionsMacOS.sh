#!/bin/bash 

echo -e "\n Copy-paste those information when reporting a bug in libpointmatcher:\n"

echo -e "Name \t\t| Version"
echo -e "----------------|-------------------------------"
echo -e "MacOS: \t|" $(sw_vers -productVersion)
echo -e "architecture: \t|" $(getconf LONG_BIT)"-bit"
echo -e "gcc: \t\t|" $(gcc --version | grep gcc)
echo -e "git: \t\t|" $(git --version)
echo -e "homebrew: \t\t|" $(brew --version)
echo -e "cmake: \t\t|" $(cmake --version)
echo -e "boost: \t\t|" $(brew info boost | grep "==> boost")
echo -e "eigen: \t|\t" $(brew info eigen | grep "==> eigen")
echo -e "doxygen: \t|" $(brew info doxygen | grep "==> doxygen")
echo -e "\n\n"
