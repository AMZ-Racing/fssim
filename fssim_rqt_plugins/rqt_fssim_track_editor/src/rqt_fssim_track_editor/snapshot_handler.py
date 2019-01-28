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
import scipy.io as io

# ROS related imports
import rosbag

# ROS Msgs
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

from scipy.spatial.distance import dice

from message_filter import BagMessageFilter

class SnapshotHandler():

    def __init__(self):

        self.clear()

    def clear(self):
        self.list_snapshots = []

        self.min = 0
        self.max = 100
        self.cones = np.zeros((0, 2))
        self.cur_state = np.zeros((0,3))
        self.topic_names = []
        self.bag_filter = BagMessageFilter

    def get_topic_names(self, path, msg):
        self.bag_filter = BagMessageFilter(path)
        return self.bag_filter.get_topic_names(msg)

    def load_snap_from_list(self, i):
        self.cones = np.zeros((0, 2))
        msg = self.list_snapshots[i]
        gen = point_cloud2.read_points(msg)
        for p in gen:
            xy = np.array([p[0], p[1]])
            self.cones = np.vstack([self.cones, xy])

    def import_snapshots(self, topic_name):
        print "Importing Snapshots from: ", topic_name

        bag = rosbag.Bag(self.bag_filter.file_name)

        self.list_snapshots = []
        self.states = np.zeros((0, 3))
        count = 0
        state_topic = '/slam/pose'
        for topic, msg, t in bag.read_messages([topic_name,state_topic]):
            if topic == topic_name:
                self.list_snapshots.append(msg)
                count = count + 1
        self.max = count
        bag.close()

        self.load_snap_from_list(0)
        print "Importing DONE"


    def load_topic_names(self, path):
        bag = rosbag.Bag(path)

        for topic, msg, t in bag.read_messages():
            if type(msg) is PointCloud2:
                self.topic_names.append(topic)

        bag.close()

    def export(self, path, xml, matlab, yaml):
        if xml:
            self.save_to_xml(path)
        if yaml:
            self.export_to_yaml(path)
        if matlab:
            self.export_to_mat(path)

    def save_array_to_xml(self, root, name, array):
        right = etree.SubElement(root, name)
        size = array.shape[0]

        if size == array.size and size == 3:
            etree.SubElement(right, "size").text = str(1)
            save = str(array[0]) + " " + str(array[1]) + " " + str(array[2])
            etree.SubElement(right, "pose").text = save
            return

        etree.SubElement(right, "size").text = str(size)
        for i in range(0, size):
            save = ""
            if len(array[i]) == 2:
                save = str(array[i, 0]) + " " + str(array[i, 1])
            elif len(array[i] == 3):
                save = str(array[i, 0]) + " " + str(array[i, 1]) + " " + str(array[i, 2])
            etree.SubElement(right, "pose").text = save

    def save_to_xml(self, path):
        root = etree.Element("track")
        self.save_array_to_xml(root, "cones", self.cones)
        self.save_array_to_xml(root, "pos", np.transpose(self.cur_state))
        tree = etree.ElementTree(root)
        tree.write(path + '.xml', pretty_print=True, xml_declaration=True, encoding='UTF-8')

    def export_to_yaml(self, path):
        dict_track = {"cones": self.cones.tolist(), "pos" : self.cur_state.tolist()}
        # print dict_track
        with open(path + ".yaml", 'w') as outfile:
            yaml.dump(dict_track, outfile, default_flow_style=False)

    def export_to_mat(self, path):
        dict_track = {"cones": self.cones, "pos": self.cur_state}
        io.savemat(path + '.mat', dict_track)


    def export_all(self, path, xml, matlab, yaml):

        for i in range(0,self.max):
            self.load_snap_from_list(i)
            new_path = path + '_' + str(i)
            self.export(new_path, xml, matlab, yaml)