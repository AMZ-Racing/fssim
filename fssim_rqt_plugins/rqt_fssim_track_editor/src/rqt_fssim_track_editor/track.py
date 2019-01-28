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

from skidpad import *
from acceleration import *

import os

class Type(Enum):
    LEFT = 1
    RIGHT = 2
    ORANGE = 3
    ORANGE_BIG = 4
    TK_DEVICE = 5
    UNKNOWN = 6


class Mode(Enum):
    EDIT = 1
    DRAW = 2
    ERASE = 3


class TrackSettings():
    width = 4.0
    cone_dist = 4.0

    color_left = QColor(0, 0, 255)
    color_right = QColor(255, 255, 0)
    color_orange = QColor(255, 165, 0)
    color_cone_generic = QColor(0, 0, 0)
    color_middle = QColor(255, 0, 0)


class Track():
    def __init__(self):

        self.clear()

        self.track_settings = {'width': 3.5, "d_cones": 3, 'color_left': QColor(0, 0, 255),
                               'color_right': QColor(255, 255, 0), 'color_middle': 'r'}
        self.settings = TrackSettings()  # type: TrackSettings

        self.computed_cones = False

        # Init Disciplines
        self.skidpad = SkidPad()
        self.acceleration = Acceleration()

    def clear(self):
        self.cones_left = []
        self.cones_right = []
        self.cones_orange = []
        self.cones_orange_big = []
        self.tk_device = []
        self.starting_pose_front_wing = [0, 0, 0]

        self.middle = []
        self.cones = []
        self.s = []
        self.phi = []
        point = np.array([0, 0])
        self.control_points = [point]
        self.control_points_s = np.array([0])
        self.computed_cones = False

        self.list_snapshots = []

    def change_settings(self, width, d_cones):
        self.settings.width = width
        self.settings.cone_dist = d_cones

    def interpolate(self, circular = True):
        if not self.computed_cones:
            if circular:
                self.control_points = np.vstack((self.control_points, self.control_points[0, :]))
                self.control_points_s = np.append(self.control_points_s,
                                                  self.control_points_s[-1] + np.linalg.norm(
                                                      self.control_points[-2, :] -
                                                      self.control_points[-1, :]))
        else:
            print "WARNING: Track was already computed"
            return

        kind = "cubic"
        fx = interp1d(self.control_points_s, self.control_points[:, 0], kind = kind)
        fy = interp1d(self.control_points_s, self.control_points[:, 1], kind = kind)
        xnew = np.linspace(0, self.control_points_s[-1], num = 100, endpoint = True)
        self.middle = np.column_stack((fx(xnew), fy(xnew)))

        self.compute_cones()
        self.add_tk_device(self.settings.width)

        starting_s = self.control_points_s[-1] - 6
        dx = (fx(starting_s + 0.1) - fx(starting_s - 0.1)) / (2 * 0.1)
        dy = (fy(starting_s + 0.1) - fy(starting_s - 0.1)) / (2 * 0.1)
        self.starting_pose_front_wing = [fx(starting_s).tolist(), fy(starting_s).tolist(), float(np.arctan2(dy, dx))]

    def add_tk_device(self, width, x_offet = 0.0):
        # Time keeping cones
        tk_cone_distance_front = x_offet - 1.3
        tk_cone_distance_back = x_offet + 1.3
        tk_cone_distance_side = width / 2 + 0.5
        mirror = np.array([1, -1])
        tk_cone_front = np.array([tk_cone_distance_front, tk_cone_distance_side])
        tk_cone_back = np.array([tk_cone_distance_back, tk_cone_distance_side])
        self.cones_orange_big.extend([tk_cone_front, tk_cone_front * mirror, tk_cone_back, tk_cone_back * mirror])

        # TK Device
        tk_device1_left = (tk_cone_front + tk_cone_back) / 2.0
        tk_device1_left[1] = tk_device1_left[1] + 0.5
        tk_device1_right = mirror * tk_device1_left

        self.tk_device.extend([tk_device1_left, tk_device1_right])


    def generate_skidpad(self, widget):
        self.skidpad.generate(widget)
        self.cones_right = self.skidpad.cones_right
        self.cones_left = self.skidpad.cones_left
        self.cones_orange = self.skidpad.orange_cones
        self.cones_orange_big = self.skidpad.orange_big_cones
        self.tk_device = self.skidpad.tk_device
        self.starting_pose_front_wing = self.skidpad.starting_pose_front_wing

        self.middle = np.zeros((0, 2))
        self.control_points = np.zeros((0, 2))
        self.cones = np.zeros((0, 2))

    def generate_acceleration(self, widget):
        self.acceleration.generate(widget)
        self.cones_right = self.acceleration.cones_right
        self.cones_left = self.acceleration.cones_left
        self.cones_orange = self.acceleration.cones_orange
        self.cones_orange_big = self.acceleration.cones_orange_big
        self.tk_device = self.acceleration.tk_device
        self.starting_pose_front_wing = self.acceleration.starting_pose_front_wing

        self.middle = np.zeros((0, 2))
        self.control_points = np.zeros((0, 2))
        self.cones = np.zeros((0, 2))

    def compute_cones(self):
        last_cone = -1

        m = self.middle.shape[0]
        self.phi = np.zeros(m)
        self.s = np.zeros(m)

        track_width = self.settings.width / 2
        d_cone = self.settings.cone_dist

        xy_last = self.middle[0, :]

        for i in range(1, m - 1):
            xy_cur = self.middle[i, :]
            dx = xy_cur[0] - xy_last[0]
            dy = xy_cur[1] - xy_last[1]
            xy_last = xy_cur

            phi_tmp = (np.arctan2(dy, dx))
            if phi_tmp < 0:
                phi_tmp = 2 * np.pi + phi_tmp
            self.phi[i] = phi_tmp

            self.s[i] = np.sqrt(dx * dx + dy * dy) + self.s[i - 1]
            ds = self.s[i] - last_cone
            if ds > d_cone or last_cone == -1:
                last_cone = self.s[i]

                T = np.matrix(
                    [[np.cos(phi_tmp), -np.sin(phi_tmp), xy_cur[0]], [np.sin(phi_tmp), np.cos(phi_tmp), xy_cur[1]]])

                xy_cone_left = np.array([0, track_width, 1])
                xy_cone_left = np.squeeze(np.asarray(np.dot(T, xy_cone_left)))
                self.cones_left.append(xy_cone_left)

                xy_cone_right = np.array([0, -track_width, 1])
                xy_cone_right = np.squeeze(np.asarray(np.dot(T, xy_cone_right)))

                self.cones_right.append(xy_cone_right)

        self.computed_cones = True

    def add_point_on_middle_line(self, point):

        self.control_points = np.vstack((self.control_points, point))
        self.control_points_s = np.append(self.control_points_s,
                                          self.control_points_s[-1] + np.linalg.norm(self.control_points[-2, :] -
                                                                                     self.control_points[-1, :]))
        return True

        # self._draw_line(self.control_points[-2, :], self.control_points[-1, :])

    def get_color(self, side):
        if side == Type.LEFT:
            return self.settings.color_left
        elif side == Type.RIGHT:
            return self.settings.color_right
        elif side == Type.ORANGE:
            return self.settings.color_orange
        else:
            return self.settings.color_cone_generic

    def get_control_point(self, i):
        return self.control_points[i, :]

    def get_size(self, side):
        if side == Type.RIGHT:
            return len(self.cones_right)
        elif side == Type.LEFT:
            return len(self.cones_left)
        elif side == Type.ORANGE:
            return len(self.cones_orange)
        elif side == Type.ORANGE_BIG:
            return len(self.cones_orange_big)
        elif side == Type.TK_DEVICE:
            return len(self.tk_device)

    def get_cone_pos(self, side, i):
        if side == Type.LEFT:
            return str(self.cones_left[i][0]) + " " + str(self.cones_left[i][1]) + " 0 0 0 0"
        elif side == Type.RIGHT:
            return str(self.cones_right[i][0]) + " " + str(self.cones_right[i][1]) + " 0 0 0 0"
        elif side == Type.ORANGE:
            return str(self.cones_orange[i][0]) + " " + str(self.cones_orange[i][1]) + " 0 0 0 0"
        elif side == Type.ORANGE_BIG:
            return str(self.cones_orange_big[i][0]) + " " + str(self.cones_orange_big[i][1]) + " 0 0 0 0"
        elif side == Type.TK_DEVICE:
            return str(self.tk_device[i][0]) + " " + str(self.tk_device[i][1]) + " 0 0 0 0"

    def save_array_to_xml(self, root, name, array):
        right = etree.SubElement(root, name)
        size = array.shape[0]
        etree.SubElement(right, "size").text = str(size)
        for i in range(0, size):
            etree.SubElement(right, "pose").text = str(array[i, 0]) + " " + str(array[i, 1])

    def add_cone_left(self, cone):
        d = 10000
        i_best = -1
        for i,c in enumerate(self.cones_left):
            dx = c[0] - cone[0]
            dy = c[1] - cone[1]
            d_cur = np.hypot(dx,dy)
            if d_cur < d:
                i_best = i
                d = d_cur


    def save_1d_array_to_xml(self, root, name, array):
        right = etree.SubElement(root, name)
        size = array.shape[0]
        etree.SubElement(right, "size").text = str(size)
        for i in range(0, size):
            etree.SubElement(right, "pose").text = str(array[i])

    def save_to_xml(self, path):
        root = etree.Element("track")

        self.save_array_to_xml(root, "left", self.cones_left)
        self.save_array_to_xml(root, "right", self.cones_right)
        self.save_array_to_xml(root, "orange", self.cones_orange)
        if len(self.cones_left) == len(self.middle) and len(self.cones_right) == len(self.middle):
            self.save_array_to_xml(root, "middle", self.middle)
            self.save_1d_array_to_xml(root, "s", self.s)
            self.save_1d_array_to_xml(root, "phi", self.phi)
            self.save_array_to_xml(root, "control_points", self.control_points)
            self.save_1d_array_to_xml(root, "control_points_s", self.control_points_s)

        tree = etree.ElementTree(root)
        tree.write(path, pretty_print = True, xml_declaration = True, encoding = 'UTF-8')

    def get_1d_array_from_xml(self, child_of_root):
        array = np.zeros((0, 1))
        i = 0
        for element in child_of_root:
            pos = [float(x) for x in element.text.split()]
            if element.tag == "size":
                array = np.zeros((int(pos[0]), 1))
            else:
                array[i] = pos[0]
                i = i + 1
        return array

    def get_array_from_xml(self, child_of_root):
        array = np.zeros((0, 2))
        i = 0
        for element in child_of_root:
            pos = [float(x) for x in element.text.split()]
            if element.tag == "size":
                array = np.zeros((int(pos[0]), 2))
            else:
                array[i, :] = pos
                i = i + 1
        return array

    def load_track_from_xml(self, path):
        xml = etree.parse(path)
        root = xml.getroot()  # type: etree._ElementTree
        for child_of_root in root:
            if child_of_root.tag == "s":
                self.s = self.get_1d_array_from_xml(child_of_root)
            elif child_of_root.tag == "phi":
                self.phi = self.get_1d_array_from_xml(child_of_root)
            elif child_of_root.tag == "control_points_s":
                self.control_points_s = self.get_1d_array_from_xml(child_of_root)
            elif child_of_root.tag == "left":
                self.cones_left = self.get_array_from_xml(child_of_root)
            elif child_of_root.tag == "right":
                self.cones_right = self.get_array_from_xml(child_of_root)
            elif child_of_root.tag == "middle":
                self.middle = self.get_array_from_xml(child_of_root)
            elif child_of_root.tag == "control_points":
                self.control_points = self.get_array_from_xml(child_of_root)
        self.computed_cones = True

    def read_polygon_to_array(self, polygon):
        array = np.zeros((0, 2))
        for p in polygon.polygon.points:
            xy = np.array([p.x, p.y])
            array = np.vstack([array, xy])
        return array

    def load_track_from_bag(self, path,outside,inside,center):
        bag = rosbag.Bag(path)
        self.clear()
        for topic, msg, t in bag.read_messages(topics = [outside,inside,center]):
            if len(msg.polygon.points) == 0:
                continue
            if topic == outside:
                self.cones_right = self.read_polygon_to_array(msg)
            elif topic == inside:
                self.cones_left = self.read_polygon_to_array(msg)
            elif topic == center:
                self.control_points = self.read_polygon_to_array(msg)

        if self.cones_left.size and self.cones_right.size:
            self.computed_cones = True

        self.add_tk_device(4.0, x_offet = 6.0)
        self.starting_pose_front_wing = [0.0,0.0,0.0]

        bag.close()

    def import_snapshots(self, path):
        bag = rosbag.Bag(path)
        self.list_snapshots = []
        count = 0
        for topic, msg, t in bag.read_messages('/lidar_mode1/cones'):
            self.list_snapshots.append(msg)
            count = count + 1
        bag.close()

    def export_to_yaml(self, path, name, create_dir=True):
        print path, name
        dict_track = {"cones_left": np.array(self.cones_left).tolist(),
                      "cones_right": np.array(self.cones_right).tolist(),
                      "cones_orange": np.array(self.cones_orange).tolist(),
                      "cones_orange_big": np.array(self.cones_orange_big).tolist(),
                      "tk_device": np.array(self.tk_device).tolist(),
                      "middle_points":np.array(self.middle).tolist(),
                      "starting_pose_front_wing": self.starting_pose_front_wing}

        if create_dir:
            dir = path + '/' + name
            if not os.path.exists(dir):
                os.makedirs(dir)
        else:
            dir = path

        file_path = dir + '/' + name + ".yaml"
        with open(file_path, 'w') as outfile:
            yaml.dump(dict_track, outfile, default_flow_style = False)
        print "[INFO] Saving track to: ",file_path

    def export_to_mat(self, path, name):
        if len(self.cones_left) == len(self.middle) and len(self.cones_right) == len(self.middle):
            dict_track = {"left": self.cones_left.tolist(),
                          "right": self.cones_right.tolist(),
                          "middle": self.middle.tolist(),
                          "s": self.s.tolist(),
                          "phi": self.phi.tolist(),
                          "control_points_s": self.control_points_s.tolist(),
                          "control_points": self.control_points.tolist()}
        else:
            dict_track = {"left": self.cones_left,
                          "right": self.cones_right}
        # print dict_track
        dir = path + '/'+ name
        if not os.path.exists(dir):
            os.makedirs(dir)
        file_path = dir +'/'+ name + '.mat'
        savemat(file_path, dict_track, oned_as = 'row')
        print "[INFO] Saving track to: ",file_path
