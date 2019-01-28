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

import os
import rospy
import rospkg
import glob
import numpy as np

from python_qt_binding import loadUi

from qt_gui.plugin import Plugin

from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import *
from PyQt5 import QtGui, QtCore
from PyQt5.QtWidgets import QFileDialog
from PyQt5.QtWebKit import *

from cone_editor import *

class TrackEditorPlugin(Plugin):

    def __init__(self, context):
        super(TrackEditorPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('TrackEditorPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        # if not args.quiet:
        #     print 'arguments: ', args
        #     print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_fssim_track_editor'), 'resource', 'TrackEditorPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('TrackEditorPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._cones_view = TrackViewScene(context, self._widget.track_view)
        self._widget.track_view.setScene(self._cones_view)

        # Buttons
        self._widget.btn_export.clicked.connect(self.handle_export)
        self._widget.btn_import.clicked.connect(self.handle_load_track)
        self._widget.btn_computeTrack.clicked.connect(self.handle_compute_track)
        self._widget.btn_import_bag_snap.clicked.connect(self.handle_input_snap)
        self._widget.pushButtonGenerateSkidpad.clicked.connect(self.pushButton_skidpad)

        # Sliders
        self._widget.horizontal_slider_grid_transparency.valueChanged.connect(self.slider_changed_grid_transparency)
        self._widget.horSlid_snap.valueChanged.connect(self.slider_changed_snap)

        # Spin Box
        self._widget.spin_box_grid_size.valueChanged.connect(self.spin_box_grid_size)
        self._widget.spin_box_px_per_m.valueChanged.connect(self.spin_box_px_per_m)

        # Double Spin Box
        self._widget.d_spin_box_cone_dist.valueChanged.connect(self.track_settings_change)
        self._widget.d_spin_box_track_width.valueChanged.connect(self.track_settings_change)
        self._widget.doubleSpinBoxConeSize.valueChanged.connect(self._cones_view.set_cone_diameter)

        # Widget Events
        self._widget.resizeEvent = self.resize_event

        # Radio Buttons
        self._widget.rB_left.toggled.connect(self.handle_rb_left)
        self._widget.rB_right.toggled.connect(self.handle_rb_right)
        self._widget.rB_draw_track.toggled.connect(self.handle_rb_draw_track)
        self._widget.rB_erase.toggled.connect(self.handle_rb_erase)

        # ComboBox
        self._widget.comboBox_snap_topics.currentIndexChanged[str].connect(self.comboBox_snap_topics_changed)

        # Set filter for keyboard press
        self._widget.installEventFilter(self)

        # Setup Paths
        rospack = rospkg.RosPack()
        path_fssim_gazebo = rospack.get_path('fssim_gazebo')
        self._cones_view.model_path = path_fssim_gazebo + "/models"
        self._cones_view.tracks_path = rospack.get_path('rqt_fssim_track_editor') + "/tracks"

        # Variabples for snapshots importer
        self._topics_for_snpshot_found = False

    def pushButton_skidpad(self):
        id =  self._widget.toolBoxDiscipline.currentIndex()
        if id is 0:
            print "Generating SKIDPAD"
            self._cones_view.generate_skipdpad(self._widget)
            self._widget.line_edit_track_name.setText("skidpad")
        elif id is 1:
            print "Generating Acceleration"
            self._cones_view.generate_acceleration(self._widget)
            self._widget.line_edit_track_name.setText("acceleration")

    def slider_changed_snap(self, event):
        self._cones_view.draw_snapshot_i(event)

    def comboBox_snap_topics_changed(self,event):
        if self._topics_for_snpshot_found:
            self._cones_view.snapshots.import_snapshots(event)
            self._cones_view.change_view(2)
            self._widget.horSlid_snap.setMaximum(self._cones_view.snapshots.max)
            print "INFO: Snapshot imported"

    def handle_input_snap(self):
        if not self._topics_for_snpshot_found:
            name = QFileDialog.getOpenFileName(self._widget, 'Load from Bag')
            self._widget.comboBox_snap_topics.clear()

            found_topics = self._cones_view.snapshots.get_topic_names(name[0], PointCloud2)
            self._widget.comboBox_snap_topics.addItems(found_topics)
            self._topics_for_snpshot_found = True

    def handle_btn_read_snaps(self):
        # self._cones_view.snapshots.import_snapshots(name[0],'/lidar_mode1/cones')
        self._cones_view.change_view(2)
        # self._widget.horSlid_snap.setMaximum(self._cones_view.snapshots.max)


    def track_settings_change(self, event):
        d_cones =  self._widget.d_spin_box_cone_dist.value()
        width = self._widget.d_spin_box_track_width.value()
        self._cones_view.track.change_settings(width, d_cones)

    def eventFilter(self, object, event):
        if event.type() == QEvent.ShortcutOverride:
            self.on_key_press(event)
            return True
        elif event.type() == QEvent.KeyRelease:
            self.on_key_release(event)
            return True

        return False

    def spin_box_grid_size(self, event):
        self._cones_view.update_grid(self._widget.horizontal_slider_grid_transparency.value(), event)

    def spin_box_px_per_m(self, event):
        self._cones_view.set_px_per_m(event)
        self._cones_view.update_grid(self._widget.horizontal_slider_grid_transparency.value(),
                                     self._widget.spin_box_grid_size.value())

    def slider_changed_grid_transparency(self, event):
        self._cones_view.update_grid(event, self._widget.spin_box_grid_size.value())

    def resize_event(self, event):
        # print "resize", event.size(), self._widget.track_view.frameGeometry()
        self._cones_view.update_grid()

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def on_key_press(self, event):
        key = event.key()
        if key == Qt.Key_Control:
            self._cones_view.change_enabled(True)

    def on_key_release(self, event):
        key = event.key()
        if key == Qt.Key_Control:
            self._cones_view.change_enabled(False)
        elif key == Qt.Key_C and self._cones_view.enable_editing:
            self._cones_view.clearTrack()

    def handle_rb_left(self):
        self._cones_view.set_cone_add_side(Type.LEFT)
        self._cones_view.set_mode(Mode.EDIT)

    def handle_rb_draw_track(self):
        self._cones_view.set_mode(Mode.DRAW)

    def handle_rb_erase(self):
        self._cones_view.set_mode(Mode.ERASE)

    def handle_rb_right(self):
        self._cones_view.set_mode(Mode.EDIT)
        self._cones_view.set_cone_add_side(Type.RIGHT)

    def handle_compute_track(self):
        self._cones_view.interpolate(self._widget.checkBox_circular.isChecked())

    def handle_load_track(self, str):
        name = QFileDialog.getOpenFileName(self._widget, 'Import from File',os.environ['HOME'],"*.bag")
        outside = self._widget.lineEdit_track_topic_outside.text()
        inside = self._widget.lineEdit_track_topic_inside.text()
        center = self._widget.lineEdit_track_topic_center.text()
        self._cones_view.handle_btn_import(name[0],outside,inside,center)

    def handle_export(self):
        mat = self._widget.chB_matlab.isChecked()
        yaml = self._widget.chB_yaml.isChecked()
        # name = QFileDialog.getSaveFileName(self._widget, 'Save File')
        self._cones_view.handle_btn_export(self._widget.line_edit_track_name.text(),yaml, mat)
