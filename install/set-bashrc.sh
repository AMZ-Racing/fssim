#!/bin/bash
#
# AMZ-Driverless
# Copyright (c) 2018 Authors:
#   - Juraj Kabzan <kabzanj@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

FSSIM_DIR="$( cd -P "$(dirname "${BASH_SOURCE[0]}")" && cd .. && pwd )"

# add environment variables to bashrc
if [ -z "${FSSIM}" ] && [ -z $(grep -m 1 "FSSIM" ${HOME}/.bashrc) ]; then
    echo FSSIM_DIR
    export FSSIM=$FSSIM_DIR

    echo "Exporting FSSIM as ${FSSIM}"
    echo "Adding FSSIM variables to ~/.bashrc.";
    echo $'\n#FSSIM' >> ${HOME}/.bashrc
    echo "export FSSIM=$FSSIM_DIR" >> ${HOME}/.bashrc
    echo -e "\nUpdated ~/.bashrc with environment variables"
    
elif [ ! "${FSSIM}" == "${FSSIM_DIR}" ]; then
    echo -e "FSSIM is already set to a different folder: ${FSSIM}.\n"\
         "Remove lines similar to the following from your  ~/.bashrc (and close all terminals).\n"\
         "export FSSIM=${FSSIM}\n"
    RED='\033[0;31m'
    echo -e "${RED}###### FAIL ######"
    exit 1;
fi