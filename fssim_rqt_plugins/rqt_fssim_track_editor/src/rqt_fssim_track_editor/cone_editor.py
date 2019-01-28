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
import scipy.io as sio

from qt_gui.plugin import Plugin
from PyQt5.QtWidgets import QWidget, QGraphicsScene, QGraphicsView, QGraphicsLineItem
from PyQt5 import QtGui, QtCore
from PyQt5.QtGui import QColor, QPen, QBrush
from PyQt5.QtCore import *

from track import *
from snapshot_handler import *


class TrackViewScene(QGraphicsScene):
    cur_scale = 1.0
    _px_per_m = 10

    enable_editing = False

    def __init__(self, context, cone_view):
        super(TrackViewScene, self).__init__()

        self._context = context
        self.cone_view = cone_view  # type: QGraphicsView
        self._map_height = cone_view.frameGeometry().height()
        self._map_width = cone_view.frameGeometry().width()
        self._landmarks = []
        self._car_diameter = 5.0
        self._cone_diameter = self.m_to_px(0.5)

        self.cone_view.setDragMode(1)

        self._mode = Mode.DRAW

        self._grid_alpha = 255
        self._grid_m = 5
        self._draw_grid(self._grid_alpha, self._grid_m)

        self.mousePressEvent = self.on_mouse_down
        # self.mouseMoveEvent = self.on_mouse_move

        self.track = Track()

        self.snapshots = SnapshotHandler()

        self.model_path = ""
        self.tracks_path = ""

        self.draw_rect([0, 0], 0.5, 0.5)

    def set_cone_diameter(self, size):
        self._cone_diameter = self.m_to_px(size)

    def interpolate(self, circular = True):
        self.track.interpolate(circular)
        self.update_all()

    def generate_skipdpad(self, widget):
        self.track.generate_skidpad(widget)
        self.update_all()

    def generate_acceleration(self, widget):
        self.track.generate_acceleration(widget)
        self.update_all()

    def draw_entire_track(self):
        self.draw_track(self.track.middle)
        self.draw_cones(self.track.cones_right, self.track.get_color(Type.RIGHT))
        self.draw_cones(self.track.cones_orange, self.track.get_color(Type.ORANGE))
        self.draw_cones(self.track.cones_left, self.track.get_color(Type.LEFT))
        self.draw_big_cones(self.track.cones_orange_big, self.track.get_color(Type.ORANGE))
        self.draw_tk_device(self.track.tk_device)
        self.draw_cones(self.track.cones, self.track.get_color(Type.UNKNOWN))
        self.draw_lines(self.track.control_points)
        self.draw_axes(self.track.starting_pose_front_wing)

    def draw_snapshot(self):
        self.clear()
        self.update_grid(self._grid_alpha, self._grid_m)
        self.draw_cones(self.snapshots.cones)

    def draw_snapshot_i(self, i):
        self.snapshots.load_snap_from_list(i)
        self.draw_snapshot()

    def change_enabled(self, enabled):
        if enabled:
            self.enable_editing = True
            self.cone_view.setDragMode(0)
        else:
            self.enable_editing = False
            self.cone_view.setDragMode(1)

    def update_grid(self, alpha=20, grid_size=5, draw_track=True):
        self._map_height = self.cone_view.frameGeometry().height()
        self._map_width = self.cone_view.frameGeometry().width()
        self.clear()

        self._draw_grid(alpha, grid_size)

        if draw_track:
            self.draw_entire_track()

    def update_all(self):
        self.clear()
        self.update_grid(self._grid_alpha, self._grid_m)
        self.draw_entire_track()

    def show_event(self, event):
        self.cone_view.fitInView(self.sceneRect(), Qt.KeepAspectRatio)

    def change_view(self, i):
        if i == 2:
            self.draw_snapshot()
        elif i == 0:
            self.update_all()

    def add_land_mark(self, x, y):
        pen = QPen(QColor(100, 200, 0), 0.5, Qt.SolidLine, Qt.RoundCap)

    def clearTrack(self):
        self.track.clear()
        self.update_grid(self._grid_alpha, self._grid_m, False)

    #####################################
    ## GETTERS & SETTERS
    #####################################

    def set_px_per_m(self, val):
        self._px_per_m = val

    def set_mode(self, mode):
        self._mode = mode

    def set_cone_add_side(self, side):
        self._side = side

    #####################################
    ## EVENT HANDLERS
    #####################################

    def wheelEvent(self, event):
        if event.delta() > 0:
            factor = 1.2
            if self.cur_scale < 100:
                self.cur_scale = self.cur_scale * factor
        else:
            factor = 0.8
            if self.cur_scale > 0.1:
                self.cur_scale = self.cur_scale * factor

        if self.cur_scale > 0.1 and self.cur_scale < 10:
            self.cone_view.scale(factor, factor)

        self.update_grid(self._grid_alpha, self._grid_m)

    def on_mousewheel(self, event):
        pass

    def handle_btn_export(self, name, yaml, mat):
        path = self.tracks_path + "/" + name
        if yaml:
            self.track.export_to_yaml(self.tracks_path, name)
        if mat:
            self.track.export_to_mat(self.tracks_path, name)

        self.export_model(path, name)

    def export_model(self, path, name):
        root = etree.Element("model")
        etree.SubElement(root, "name").text = "track"
        etree.SubElement(root, "version").text = "1.0"
        etree.SubElement(root, "sdf", version="1.4").text = name + ".sdf"
        etree.SubElement(root, "description").text = "random track"
        tree = etree.ElementTree(root)
        tree.write(self.model_path + "/track/model.config", pretty_print=True, xml_declaration=True, encoding='UTF-8')

        root = etree.Element("sdf", version="1.4")
        model = etree.SubElement(root, "model", name="some track")

        for i in range(0, self.track.get_size(Type.RIGHT)):
            include = etree.SubElement(model, "include")
            etree.SubElement(include, "uri").text = "model://fssim_gazebo/models/cone_blue"
            etree.SubElement(include, "pose").text = self.track.get_cone_pos(Type.RIGHT, i)
            etree.SubElement(include, "name").text = "cone_right"

        for i in range(0, self.track.get_size(Type.LEFT)):
            include = etree.SubElement(model, "include")
            etree.SubElement(include, "uri").text = "model://fssim_gazebo/models/cone_yellow"
            etree.SubElement(include, "pose").text = self.track.get_cone_pos(Type.LEFT, i)
            etree.SubElement(include, "name").text = "cone_left"

        for i in range(0, self.track.get_size(Type.ORANGE)):
            include = etree.SubElement(model, "include")
            etree.SubElement(include, "uri").text = "model://fssim_gazebo/models/cone_orange"
            etree.SubElement(include, "pose").text = self.track.get_cone_pos(Type.ORANGE, i)
            etree.SubElement(include, "name").text = "cone_orange"

        for i in range(0, self.track.get_size(Type.ORANGE_BIG)):
            include = etree.SubElement(model, "include")
            etree.SubElement(include, "uri").text = "model://fssim_gazebo/models/cone_orange_big"
            etree.SubElement(include, "pose").text = self.track.get_cone_pos(Type.ORANGE_BIG, i)
            etree.SubElement(include, "name").text = "cone_orange_big"

        for i in range(0, self.track.get_size(Type.TK_DEVICE)):
            include = etree.SubElement(model, "include")
            etree.SubElement(include, "uri").text = "model://fssim_gazebo/models/time_keeping"
            etree.SubElement(include, "pose").text = self.track.get_cone_pos(Type.TK_DEVICE, i)
            etree.SubElement(include, "name").text = "tk_device_" + str(i)


        tree = etree.ElementTree(root)
        gazebo_models = self.model_path + "/track/" + name
        tree.write(gazebo_models + ".sdf", pretty_print=True, xml_declaration=True, encoding='UTF-8')
        self.track.export_to_yaml(self.model_path + "/track/tracks_yaml", name,create_dir=False)

        print "[INFO] Saving track to: ",gazebo_models + ".sdf"

    def handle_btn_import(self, path,outside,inside,center):
        if path.endswith('.bag'):
            self.track.load_track_from_bag(path,outside,inside,center)
            self.update_all()
        else:
            print "[ERROR] Wrong file extension. Only ROSBAG supported"

    def on_mouse_up(self, event):
        pass

    def on_mouse_move(self, event):
        print event

    def on_mouse_down(self, event):
        if not self.enable_editing:
            return

        scene_point = event.scenePos()
        point = np.array([(scene_point.x()), (scene_point.y())])
        point = self.get_m_from_px(point)
        if self._mode == Mode.DRAW:
            if self.track.add_point_on_middle_line(point):
                point_from = (self.track.get_control_point(-2))
                point_to = (self.track.get_control_point(-1))
                self.draw_line(point_from, point_to)
                self.draw_rect(point_to, 0.5, 0.5)
        elif self._mode == Mode.EDIT and self.track.computed_cones:
            if self._side == Type.RIGHT:
                self.track.cones_right = np.vstack([self.track.cones_right, point])
            elif self._side == Type.LEFT:
                self.track.cones_left = np.vstack([self.track.cones_left, point])
            self.update_all()
        elif self._mode == Mode.ERASE:
            counter = 0
            dist_min = 100.0
            index = 0
            for p in self.track.cones_left:
                dist = np.linalg.norm(p - point)
                if dist < dist_min:
                    dist_min = dist
                    index = counter
                counter = counter + 1

            if dist_min < 0.5:
                self.track.cones_left = np.delete(self.track.cones_left, index, 0)

            counter = 0
            dist_min = 100.0
            index = 0
            for p in self.track.cones_right:
                dist = np.linalg.norm(p - point)
                if dist < dist_min:
                    dist_min = dist
                    index = counter
                counter = counter + 1
            if dist_min < 0.5:
                self.track.cones_right = np.delete(self.track.cones_right, index, 0)

            self.update_all()

    #####################################
    ## DRAWING FUNCTIONS
    #####################################

    def _draw_axes(self):
        pos_from = self.get_px_pos_from_m([0, 0])
        pos_to = self.get_px_pos_from_m([5, 0])

        grid_lines = QPen(QColor(255, 0, 0))
        grid_lines.setWidth(5)
        self.addLine(pos_from[0], pos_from[1], pos_to[0], pos_to[1], grid_lines)

        pos_to = self.get_px_pos_from_m([0, 5])
        grid_lines = QPen(QColor(0, 0, 255))
        grid_lines.setWidth(5)

        self.addLine(pos_from[0], pos_from[1], pos_to[0], pos_to[1], grid_lines)

    def draw_axes(self, pos):
        heading = pos[2]
        x = pos[0]
        y = pos[1]
        length = 2.5
        xy = [x,y]
        xy_to = [x + length * np.cos(heading), y + length * np.sin(heading)]

        pos_from = self.get_px_pos_from_m(xy)
        pos_to = self.get_px_pos_from_m(xy_to)

        self.addLine(pos_from[0], pos_from[1], pos_to[0], pos_to[1], QPen(QColor(255, 0, 0)))

        heading = heading + np.pi / 2.0
        xy_to = [x + length * np.cos(heading), y + length * np.sin(heading)]

        pos_from = self.get_px_pos_from_m(xy)
        pos_to = self.get_px_pos_from_m(xy_to)

        self.addLine(pos_from[0], pos_from[1], pos_to[0], pos_to[1], QPen(QColor(0, 0, 255)))


    def _draw_grid(self, alpha, grid_size):

        self._grid_alpha = alpha
        self._grid_m = grid_size

        self._draw_axes()

        max_x = 200
        max_y = 200

        grid_lines = QPen(QColor(105, 105, 105, alpha))

        for x in range(0, max_x, grid_size):
            pos_from = self.get_px_pos_from_m([x, -max_y])
            pos_to = self.get_px_pos_from_m([x, max_y])

            self.addLine(pos_from[0], pos_from[1], pos_to[0], pos_to[1], grid_lines)

            pos_from = self.get_px_pos_from_m([max_x, x])
            pos_to = self.get_px_pos_from_m([-max_x, x])

            self.addLine(pos_from[0], pos_from[1], pos_to[0], pos_to[1], grid_lines)

            pos_from = self.get_px_pos_from_m([-x, -max_y])
            pos_to = self.get_px_pos_from_m([-x, max_y])

            self.addLine(pos_from[0], pos_from[1], pos_to[0], pos_to[1], grid_lines)

            pos_from = self.get_px_pos_from_m([max_x, -x])
            pos_to = self.get_px_pos_from_m([-max_x, -x])

            self.addLine(pos_from[0], pos_from[1], pos_to[0], pos_to[1], grid_lines)

    def _draw_cone(self, x, y, diameter=10, color=QColor(100, 200, 0)):
        point = self.get_px_pos_from_m([x, y])
        cone_pen = QPen(color, 2, Qt.SolidLine, Qt.RoundCap)
        cone_ellipse = self.addEllipse(point[0] - diameter / 2,
                                       point[1] - diameter / 2,
                                       diameter,
                                       diameter,
                                       cone_pen)

    def draw_line(self, start, end, color=QColor(0, 0, 100)):
        cone_pen = QPen(color, 2, Qt.DashLine, Qt.RoundCap)
        start = self.get_px_pos_from_m(start)
        end = self.get_px_pos_from_m(end)
        self.addLine(start[0], start[1], end[0], end[1], cone_pen)

    def draw_rect(self, pos, width, height, color=QColor(0, 0, 100)):
        cone_pen = QPen(color, 2, Qt.SolidLine, Qt.RoundCap)
        width = self.m_to_px(width)
        height = self.m_to_px(height)

        start = self.get_px_pos_from_m(pos)
        start[0] = start[0] - width / 2.0
        start[1] = start[1] - height / 2.0
        self.addRect(start[0], start[1], width, height, cone_pen)

    def draw_track(self, track, color=QColor(100, 200, 0)):
        for i, row in enumerate(track):
            if i != 0:
                self.draw_line(track[i - 1], track[i], color=color)

    def draw_cones(self, track, color=QColor(100, 200, 0)):
        for x, y in track:
            self._draw_cone(x, y, diameter=self._cone_diameter, color=color)

    def draw_big_cones(self, track, color=QColor(100, 200, 0)):
        for x, y in track:
            self._draw_cone(x, y, diameter=self._cone_diameter * 2.0, color=color)

    def draw_tk_device(self, track, color=QColor(255, 0, 0)):
        for x, y in track:
            self._draw_cone(x, y, diameter=self._cone_diameter * 2.0, color=color)

    def draw_lines(self, lines, color=QColor(0, 0, 100)):
        size = len(lines)

        if size < 3:
            return
        for i in range(1, size):
            last = lines[i - 1, :]
            pos = lines[i, :]
            self.draw_line(last, pos, color)
            self.draw_rect(pos, 0.5, 0.5)

    #####################################
    ## CONVERTERS
    #####################################

    def m_to_px(self, x):
        return x * self._px_per_m

    def px_to_m(self, px, py):
        return [self.px_to_m(px), self.px_to_m(py)]

    def px_to_m(self, px):
        return px / self._px_per_m

    def get_px_pos_from_m(self, p):
        p_augmented = np.array([p[0], -p[1], 1])
        p_res = np.dot(self.get_transform_px_to_m(), p_augmented)
        return  np.array([p_res[0, 0], p_res[0, 1]])

    def get_m_from_px(self, p):
        p_augmented = np.array([p[0], p[1], 1])
        p_res = np.dot(np.linalg.inv(self.get_transform_px_to_m()), p_augmented)
        return np.array([p_res[0, 0], -p_res[0, 1]])

    def get_transform_px_to_m(self):
        # Inv = np.matrix([[1, 0], [0, -1]])
        angle = 3.0 / 2.0 * np.pi
        c = np.cos(angle)
        s = np.sin(angle)
        Rot = np.matrix([[c, -s], [s, c]])

        Multip = np.matrix([[self._px_per_m, 0], [0, self._px_per_m]])

        InvRot = Multip * Rot

        trans = [self._map_width / 2.0, self._map_height / 2.0]
        T = np.matrix([[InvRot[0, 0], InvRot[0, 1], trans[0]],
                       [InvRot[1, 0], InvRot[1, 1], trans[1]],
                       [0, 0, 1]])
        return T
