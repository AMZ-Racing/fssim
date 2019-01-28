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


class SkidPad:

    def __init__(self):
        self.clear()

    def clear(self):
        # Cones
        self.cones_left = []
        self.cones_right = []
        self.orange_cones = []
        self.orange_big_cones = []
        self.tk_device = []
        self.starting_pose_front_wing = []

    def generate(self, widget):
        self.clear()

        # Settings
        cone_radius = 0.15
        inner_radius = widget.doubleSpinBoxSkidpadInnerRadius.value() / 2 - cone_radius
        outer_radius = widget.doubleSpinBoxSkidpadOuterRadius.value() / 2 + cone_radius
        number_of_inner_cones = widget.spinBoxSkidpadNumberOfInnerCOnes.value()
        vertical_track_width = widget.doubleSpinBoxSkidpadVerticalTrackWidth.value()
        cone_distance_front = -11.25
        cone_distance_front_2 = -15

        cone_distance_back_start = widget.doubleSpinBoxSkidpadConeDistanceBackStart.value()
        cone_distance_back_end = widget.doubleSpinBoxSkidpadBackEnd.value()
        cone_distance_back = widget.doubleSpinBoxSkidPadBackConesDistance.value()

        mirror = np.array([1, -1])

        # Time keeping cones
        tk_cone_distance_front = -1.3
        tk_cone_distance_back = 1.3
        tk_cone_distance_side = vertical_track_width / 2 + 0.5

        # Shift of circles
        circle_dx = 0
        circle_dy = 18.25 / 2
        shift = np.array([circle_dx, circle_dy])

        # Generate angles
        angles = np.linspace(0, 2 * np.pi, number_of_inner_cones, False)
        # print angles

        inner_cones_left = []
        outer_cones_left = []
        inner_cones_right = []
        outer_cones_right = []
        extra_cones = []
        tk_cones = []

        for angle in angles:
            unit = np.array([np.sin(angle), np.cos(angle)])

            cone = unit * inner_radius - shift
            if (np.abs(cone[1]) >= vertical_track_width / 2 and cone[1] < 0):
                inner_cones_left.append(cone)

            cone = unit * outer_radius - shift
            if (np.abs(cone[1]) > vertical_track_width / 2 and cone[1] < 0):
                outer_cones_left.append(cone)

        # Mirror cones to the other side
        for cone in inner_cones_left:
            inner_cones_right.append(-cone)

        for cone in outer_cones_left:
            outer_cones_right.append(-cone)

        # Place the extra cones
        cone_front = np.array([cone_distance_front, vertical_track_width / 2 + cone_radius])
        cone_front_2 = np.array([cone_distance_front_2, vertical_track_width / 2 + cone_radius])
        # cone_back = np.array([cone_distance_back_start, vertical_track_width / 2 + cone_radius])
        extra_cones.extend([cone_front, cone_front * mirror, cone_front_2, cone_front_2 * mirror])

        # Place end cones
        length1 = 0
        iterator1 = 0
        cone = []
        while length1 < cone_distance_back_end - cone_distance_back_start:
            cone = np.array([cone_distance_back_start + cone_distance_back * iterator1, vertical_track_width / 2 + cone_radius])
            extra_cones.append(cone)
            extra_cones.append(cone * mirror)
            iterator1 = iterator1 + 1
            length1 = length1 + cone_distance_back

        # Interpolate end-zone
        last_left = cone
        last_right = cone * mirror
        d_last_cones = last_left - last_right
        nm_to_add = int(vertical_track_width) - 1
        step_last_cones = d_last_cones / nm_to_add

        for i in range(0,nm_to_add, 1):
            cone_to_add = last_right + step_last_cones
            extra_cones.append(cone_to_add)
            last_right = cone_to_add


        # Time keeping cones
        tk_cone_front = np.array([tk_cone_distance_front, tk_cone_distance_side])
        tk_cone_back = np.array([tk_cone_distance_back, tk_cone_distance_side])
        tk_cones.extend([tk_cone_front, tk_cone_front * mirror, tk_cone_back, tk_cone_back * mirror])

        self.cones_left = outer_cones_right + inner_cones_left
        self.cones_right = inner_cones_right + outer_cones_left
        self.orange_cones = extra_cones
        self.orange_big_cones = tk_cones

        tk_device1_left = (tk_cone_front + tk_cone_back) / 2.0
        tk_device1_left[1] = tk_device1_left[1] + 0.5
        tk_device1_right = mirror * tk_device1_left

        self.tk_device.extend([tk_device1_left, tk_device1_right])

        x = widget.doubleSpinBoxStartingSkidpadX.value()
        y = widget.doubleSpinBoxStartingSkidpadY.value()
        yaw = widget.doubleSpinBoxSkidpadSkidpadYaw.value()
        self.starting_pose_front_wing = [x,y,yaw]

