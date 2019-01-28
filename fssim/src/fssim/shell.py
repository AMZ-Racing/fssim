# AMZ-Driverless
# Copyright (c) 2018 Authors:
#   - Huub Hendrikx <hhendrik@student.ethz.ch>
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

import os
import sys
import shlex
import time
import signal

if os.name == 'posix' and sys.version_info[0] < 3:
    import subprocess32 as subprocess
else:
    import subprocess

try:
    from subprocess import DEVNULL  # Python 3.
except ImportError:
    DEVNULL = open(os.devnull, 'wb')


class Command:

    def __init__(self, cmd, env=None):
        self._cmd = shlex.split(cmd)
        self._env = os.environ.copy()
        self._popen = None

        if env is None:
            env = {}

        for key, value in env.iteritems():
            self._env[key] = value

    def run(self):
        self._popen = subprocess.Popen(self._cmd, env=self._env, stdout = DEVNULL)
        # print "popen: ", self._popen, " id", self._popen.pid

    def is_running(self):
        if self._popen:
            self._popen.poll()
            if self._popen.returncode is None:
                return True
        return False

    def join(self):
        while self.is_running():
            time.sleep(1)

    def ensure_terminated(self, status=""):
        if self._popen:
            try:
                os.kill(self._popen.pid, signal.SIGINT)
            except OSError:
                print "Process does not exist"
                return
            time.sleep(0.5)
            if self._popen:
                self._popen.poll()

                if self._popen.returncode is None:
                    self._popen.terminate()
                    time.sleep(0.2)
                    self._popen.poll()

                while self._popen.returncode is None:
                    time.sleep(1)
                    if status:
                        print(status)
                    self._popen.poll()
        else:
            print "ERROR Popen is NONE"