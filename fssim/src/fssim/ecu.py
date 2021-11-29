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

# ROS Include
import rospy

# Numpy
import numpy as np

# Msgs
from nav_msgs.msg import Odometry
from fssim_common.msg import Mission, State

# Geometry import
from shapely.geometry import Point

# YAML
import yaml

# System import
import os

from enum import Enum


class EcuState(Enum):
    NOT_STARTED = 0
    READY_TO_DRIVE = 1
    FINNISHED_DISCIPLINE = 2
    EMERGENCY_STATE = 3


class Ecu:

    def __init__(self):
        print("ECU")
        self.state = EcuState.NOT_STARTED
        self.car_state_stoped = 0.0

    def mission_finished(self, mission):
        if mission.finished:
            rospy.logwarn("Discipline has been Finnished")
            self.state = EcuState.FINNISHED_DISCIPLINE

    def request_stop(self):
        return self.state == EcuState.EMERGENCY_STATE

    def start(self):
        self.state = EcuState.READY_TO_DRIVE

    def update_state(self, state):
        '''

        :param state:
        :type state: State
        :return:
        '''
        cur_time = rospy.Time().to_sec()
        if state.vx < 0.1:
            if self.car_state_stoped == 0.0:
                self.car_state_stoped = cur_time
            elif cur_time - self.car_state_stoped > 5.0 and self.state == EcuState.READY_TO_DRIVE:
                self.state = EcuState.EMERGENCY_STATE
                rospy.logwarn("Emergency State, too long standing still")
        else:
            self.car_state_stoped = 0.0
