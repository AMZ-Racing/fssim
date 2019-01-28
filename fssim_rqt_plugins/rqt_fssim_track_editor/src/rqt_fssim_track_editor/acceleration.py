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

import numpy as np

from enum import Enum
from scipy.interpolate import interp1d

## PyQt Import
from PyQt5.QtGui import QColor

## XML
from lxml import etree

# YAML
import yaml

# For Matlab
from scipy.io import savemat

# ROS related imports
import rosbag
from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

from scipy.spatial.distance import dice


class Acceleration:

    def __init__(self):
        self.clear()

    def clear(self):
        # Cones

        self.cones_left = []
        self.cones_right = []
        self.cones_orange = []
        self.cones_orange_big = []
        self.tk_device = []
        self.starting_pose_front_wing = []


    def generate(self, widget):
        self.clear()

        # Settings
        cone_radius = 0.15
        length_normal = widget.doubleSpinBoxAccelLength.value()
        length_end_zone = widget.doubleSpinBoxAccelEndZoneLegth.value()
        cone_distance = widget.doubleSpinBoxAccelConeDistance.value()
        track_width = widget.doubleSpinBoxAccelTrackWidth.value()
        mirror = np.array([1, -1])

        # Normal Cones
        length = 0
        iterator = 1
        while length < length_normal - cone_distance:
            cone = np.array([cone_distance * iterator, track_width / 2 + cone_radius])
            self.cones_left.append(cone)
            self.cones_right.append(cone * mirror)
            iterator = iterator + 1
            length = length + cone_distance

        # End Zone Cones
        length2 = 0
        iterator2 = 1
        cone = []
        while length2 < length_end_zone:
            cone = np.array([length_normal + cone_distance * iterator2, track_width / 2 + cone_radius])
            self.cones_orange.append(cone)
            self.cones_orange.append(cone * mirror)
            iterator2 = iterator2 + 1
            length2 = length2 + cone_distance

        # Interpolate end-zone
        last_left = cone
        last_right = cone * mirror
        d_last_cones = last_left - last_right
        nm_to_add = int(track_width) - 1
        step_last_cones = d_last_cones / nm_to_add

        for i in range(0, nm_to_add, 1):
            cone_to_add = last_right + step_last_cones
            self.cones_orange.append(cone_to_add)
            last_right = cone_to_add

        # TK Cones (timekeeping)
        cone_tk_front_1 = np.array([-0.3, track_width / 2 + cone_radius])
        cone_tk_front_2 = np.array([+0.3, track_width / 2 + cone_radius])
        cone_tk_end_1 = np.array([-0.3 + 75, track_width / 2 + cone_radius])
        cone_tk_end_2 = np.array([+0.3 + 75, track_width / 2 + cone_radius])
        self.cones_orange_big.extend(
            [cone_tk_front_1, cone_tk_front_1 * mirror, cone_tk_front_2, cone_tk_front_2 * mirror,
             cone_tk_end_1, cone_tk_end_1 * mirror, cone_tk_end_2, cone_tk_end_2 * mirror])

        tk_device1_left = (cone_tk_front_1 + cone_tk_front_2) / 2.0
        tk_device1_left[1] = tk_device1_left[1] + 0.5
        tk_device1_right = mirror * tk_device1_left

        tk_device2_left = (cone_tk_end_1 + cone_tk_end_2) / 2.0
        tk_device2_left[1] = tk_device2_left[1] + 0.5
        tk_device2_right = tk_device2_left * mirror

        self.tk_device.extend([tk_device1_left, tk_device1_right, tk_device2_left, tk_device2_right])

        x = widget.doubleSpinBoxAccelX.value()
        y = widget.doubleSpinBoxAccelY.value()
        yaw = widget.doubleSpinBoxAccelYaw.value()
        self.starting_pose_front_wing = [x,y,yaw]
