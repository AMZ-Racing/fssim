#!/bin/bash

# Juraj Kabzan <kabzanj@ethz.ch> - AMZ Driverless 2018
# Adapted from work by Huub Hendrikx <hhendrik@ethz.ch>


# TODO: check whether cpplint is installed
CPPLINT_PACKAGE="cpplint"

ABSOLUTE_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOP_LEVEL_DIR_NAME="$(basename "${ABSOLUTE_PATH}")"
FSSIM_ROOT=$( cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)
FSSIM_ROSDEP_DIR="${ABSOLUTE_PATH}/fssim_continuous_integration/rosdep"
ROSDEP_SOURCES_TARGET="/etc/ros/rosdep/sources.list.d"

red=$'\e[1;31m'
green=$'\e[1;32m'
blue=$'\e[1;34m'
end=$'\e[0m'

printf "\n"
printf "FSSIM dependency updater\n"
printf "Version 2.0 - 2018"
printf "\n\n"

###############################################
# Update the binary dependencies using rosdep #
###############################################
read -p "Do you want to install/update the projects dependencies of FSSIM(Y/n)? " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]] || [ -z $REPLY ]; then
    PKG_OK=$(dpkg-query -W --showformat='${Status}\n' git-lfs|grep "install ok installed")
    echo Checking for git-lfs: $PKG_OK
	if [ "" == "$PKG_OK" ]; then
        MACHINE_TYPE=`uname -m`
    	if [ ${MACHINE_TYPE} == 'x86_64' ]; then
    		printf "Installing git-lfs"
    		curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
    		sudo apt install git-lfs
    	fi
    fi
    # Place our AMZ dependencies in the rosdep sources folder
    printf "\n\nCopying FSSIM's ROS dependency list to rosdep sources folder\n"
    printf "Target folder: ${ROSDEP_SOURCES_TARGET}\n"
    printf "Sudo might ask for your password now!\n"
    COPY_FAILED_MESSAGE="[${red}COPY FAILED: The dependency update might fail now!${end}]\n"
    sudo cp "${FSSIM_ROSDEP_DIR}/fssim-dependencies.yaml" "${ROSDEP_SOURCES_TARGET}/" || { printf "$COPY_FAILED_MESSAGE" ; }
    sudo cp "${FSSIM_ROSDEP_DIR}/10-fssim-dependencies.list" "${ROSDEP_SOURCES_TARGET}/" || { printf "$COPY_FAILED_MESSAGE" ; }

    sudo apt-get update
    # Update rosdep
    rosdep update

    # Install the dependencies
    rosdep install --from-paths "${ABSOLUTE_PATH}/" -r -i -y || { printf "[${red}ROSDEP FAILED${end}]\n" ;}
    
fi
