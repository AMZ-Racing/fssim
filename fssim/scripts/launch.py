#!/usr/bin/env python
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


# System Includes

# ROS Include
import rospkg

# Arguments
import argparse

# YAML
import yaml

# Python include
import time
import os

# Process
from fssim.shell import *


class Launcher:

    def __init__(self, args):
        with open(args.config, 'r') as f:
            self.config = yaml.load(f)
        self.args = args

    def setup_report_file(self):
        report_yaml = {"title": self.config["simulation_name"],
                       "number_repetitions": len(self.config["repetitions"]),
                       "repetitions": {}}

        if not os.path.isdir(self.args.output):
            os.makedirs(self.args.output)
            
        self.report_file = self.args.output + '/output.yaml'

        with open(self.report_file, 'w+') as yamlfile:
            yaml.safe_dump(report_yaml, yamlfile)  # Also note the safe_dump

    def start(self):
        self.setup_report_file()

        for i, settings in enumerate(self.config['repetitions']):
            print "STARITNG REPETITION: ", i
            path = rospkg.RosPack().get_path("fssim") + "/scripts/automated_res.py"
            launching_script = "python {} --config {} --id {} --output {} ".format(path, self.args.config, i, self.args.output)
            print launching_script
            self.fssim_cmd = Command(launching_script)
            self.fssim_cmd.run()
            self.fssim_cmd.join()
            time.sleep(5.0)

        print "EXITING LAUNCHER SCRIPT"
        sys.exit(0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Process some integers.')
    parser.add_argument("--config", dest = "config", metavar = "FILE", help = "Config YAML file")
    parser.add_argument("--output", dest = "output", help = "Output YAML file")
    args = parser.parse_args()

    print '\033[91m' + " Welcome to:                                                                                        " + '\033[0m'
    print '\033[91m' + "                  `--.`                                                                             " + '\033[0m'
    print '\033[91m' + "                  +hyhys+:`                                                                         " + '\033[0m'
    print '\033[91m' + "                  +yyyyyyyhyo:.                                                                     " + '\033[0m'
    print '\033[91m' + "                  +yyyyyyyyyyyhyo/-`                                                                " + '\033[0m'
    print '\033[91m' + "  yyyyyyyyyyyyyyyyyyyysoo++ooosyyhyys/                                                              " + '\033[0m'
    print '\033[91m' + "  yyyyyyyyyyyyyyyys:.           :hhyyy+                                                             " + '\033[0m'
    print '\033[91m' + " `yyyyyyyyyyyyyyy:              -hhyyys                                                             " + '\033[0m'
    print '\033[91m' + " `yyyyyyyo------.        -://:-.`yyyyys                                                             " + '\033[0m'
    print '\033[91m' + " `hyyyyyh/              /hyhyyyyyyhyyys-//-                  -ssooo++//:--..`                       " + '\033[0m'
    print '\033[91m' + " .hyyyyyy+````````       :syyyyyyyyyyyyhhyy+`              .+yyyyyyyyyhyhhyhhyyysoo+/:-.`           " + '\033[0m'
    print '\033[91m' + " .hyyyyyyhhhhhhhhy:        -oyyyyyyyyyyyyyyyyo+::::::://+oyhyyyyyyyyyyyyyyyyyyyyyyyyyyyyhyyo+/-`    " + '\033[0m'
    print '\033[91m' + " -yyyyyyyyyyyyyyyyys:`       `+yyyyyyyyyyyyyyyssshhyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy-   " + '\033[0m'
    print '\033[91m' + " /yyyyyyyyyyyyyyyyyyyy/`       .syhyyyyyyyyyyy/:/hyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy/    " + '\033[0m'
    print '\033[91m' + " +hyyyyyy.`      `:yyyyy+       `yhyyyy:  ../y` .h/  .   -`  .yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyo`     " + '\033[0m'
    print '\033[91m' + " oyyyyyyy`        -yyyyyo`       yyyyyh+. -+yy  -h:  os  -y.  yyyyyyyyyyyyyyyyyyyyyyyyyyyyys-       " + '\033[0m'
    print '\033[91m' + " shyyyyyy            ``         /yhyyys:/.  os  -y-  yo  /y. `yyyyyyyyyyyyyyhyyhysshyyyyyyy+/::--`  " + '\033[0m'
    print '\033[91m' + " yyyyyyyy                     .+yhhyhyy++++syy++oyo+oys++syo+oyyyyyyhyyyyyhhy+-`  -++++++++++++++.  " + '\033[0m'
    print '\033[91m' + " ........                   -syyyyyo///////+/+++++++++++++++++++++++oyyyyyys-                       " + '\033[0m'
    print '\033[91m' + "                             `.--.`                                  `-::-`                         " + '\033[0m'

    fssim = Launcher(args)
    fssim.start()
    #
