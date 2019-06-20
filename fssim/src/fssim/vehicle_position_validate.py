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
import rospkg
import rospy
import tf
import tf.transformations
import tf2_ros

# Numpy
import numpy as np

# Msgs
from fssim_common.msg import SimHealth, ResState, TopicsHealth, Mission, Track, State
from geometry_msgs.msg import Point, Quaternion

# Geometry import
from shapely.geometry import Point

# YAML
import yaml

# System import
import os

# Geometry import
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


def to_point(marker):
    """
    Return a point from marker
    :param marker: marker containing position
    :return:
    """
    return Point(marker.pose.position.x, marker.pose.position.y)


def is_marker_blue(marker):
    """
    Check color of marker
    :param marker:
    :return: True if marker is blue
    """
    # type: (Marker) -> bool
    return marker.color.r == 0.0 and marker.color.g == 0.0 and marker.color.b == 1.0


def is_inside(trans, polygon_outside, polygon_inside):
    """
    Check if a point is inside of outside and outside of inside polygon
    :param trans:
    :param polygon_outside:
    :param polygon_inside:
    :return: true if yes
    """
    # type: (list, Polygon, Polygon) -> bool
    p = Point(trans[0], trans[1])
    return (not polygon_inside.contains(p) and polygon_outside.contains(p))


def ccw(A, B, C):
    """Tests whether the turn formed by A, B, and C is ccw"""
    return (B.x - A.x) * (C.y - A.y) > (B.y - A.y) * (C.x - A.x)


def is_left(trans, A, B):
    return ccw(A, B, Point(trans[0], trans[1]))


def is_left_cones(trans, cones):
    A = Point(cones[0][0], cones[0][1])
    B = Point(cones[-1][0], cones[-1][1])
    return is_left(trans, A, B)


class VehiclePositionCheck:

    def __init__(self, mission, do_track_check, track_details = None):

        # ROS Subscribers
        self.sub_track = rospy.Subscriber('/fssim/track', Track, self.callback_track)

        self.received_track = False
        self.ignore_track_check = not do_track_check
        self.mission = mission

        # If we find detailed track description we can set also initial position
        self.track_details = track_details

        # TF Initializations
        self.listener = tf.TransformListener()
        self.br = tf2_ros.StaticTransformBroadcaster()

        # Track Cones
        self.cones_left = []
        self.cones_right = []

    def is_track_valid(self):
        return self.received_track

    def is_car_in_track(self):
        return self.ignore_track_check or self.is_all_car_in_out_of_track()

    def get_track_init_pos(self):
        self.pose_init = Point(self.track_details['starting_pose_front_wing'])
        yaw = self.track_details['starting_pose_front_wing'][2]
        qt = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        return self.pose_init, Quaternion(*qt)

    def get_trans(self, target):
        try:
            (trans, rot) = self.listener.lookupTransform('/fssim_map', '/fssim/vehicle/' + target, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # rospy.logwarn("Could not find transform from %s to %s", 'fssim_map', 'fluela/base_link')
            rospy.sleep(1)
            return [0.0, 0.0]
        return trans

    def is_car_left_from_line(self, line):
        left_front_wheel = is_left_cones(self.get_trans("left_front_wheel"), line)
        right_front_wheel = is_left_cones(self.get_trans("right_front_wheel"), line)
        right_rear_wheel = is_left_cones(self.get_trans("right_rear_wheel"), line)
        left_rear_wheel = is_left_cones(self.get_trans("left_rear_wheel"), line)
        return left_front_wheel and right_front_wheel and right_rear_wheel and left_rear_wheel

    def is_car_right_from_line(self, line):
        left_front_wheel = not is_left_cones(self.get_trans("left_front_wheel"), line)
        right_front_wheel = not is_left_cones(self.get_trans("right_front_wheel"), line)
        right_rear_wheel = not is_left_cones(self.get_trans("right_rear_wheel"), line)
        left_rear_wheel = not is_left_cones(self.get_trans("left_rear_wheel"), line)
        return left_front_wheel and right_front_wheel and right_rear_wheel and left_rear_wheel

    def in_bounding_box(self,pos, size = 25):
        return pos[0] <= size and pos[0] >= -size and pos[1] <= size and pos[1] >= -size

    def is_all_car_in_out_of_track(self):
        if self.mission == 'trackdrive':
            # All 4 wheels must be in track in order to declare it fully in track
            is_left_front = is_inside(self.get_trans("left_front_wheel"), self.polygon_outside, self.polygon_inside)
            is_right_front = is_inside(self.get_trans("right_front_wheel"), self.polygon_outside, self.polygon_inside)
            is_right_rear = is_inside(self.get_trans("right_rear_wheel"), self.polygon_outside, self.polygon_inside)
            is_left_rear = is_inside(self.get_trans("left_rear_wheel"), self.polygon_outside, self.polygon_inside)
            return is_left_front or is_right_front or is_right_rear or is_left_rear
        elif self.mission == 'acceleration':
            is_right_from_left_cones = self.is_car_right_from_line(self.cones_left)
            is_left_from_right_cones = self.is_car_left_from_line(self.cones_right)
            return is_right_from_left_cones and is_left_from_right_cones
        elif self.mission == 'skidpad':
            left_front_wheel = self.in_bounding_box(self.get_trans("left_front_wheel"))
            right_front_wheel = self.in_bounding_box(self.get_trans("right_front_wheel"))
            right_rear_wheel = self.in_bounding_box(self.get_trans("right_rear_wheel"))
            left_rear_wheel = self.in_bounding_box(self.get_trans("left_rear_wheel"))
            return left_front_wheel and right_front_wheel and right_rear_wheel and left_rear_wheel

    def callback_track(self, data):
        '''

        :param data: Track message
        :type data: Track
        :return:
        '''
        rospy.logwarn("Track was received")

        self.cones_left = []
        self.cones_right = []

        for c in data.cones_left:
            self.cones_left.append([c.x, c.y])
        for c in data.cones_right:
            self.cones_right.append([c.x, c.y])

        if len(data.tk_device_start) == 2:
            self.start_A = Point(data.tk_device_start[0].x, data.tk_device_start[0].y)
            self.start_B = Point(data.tk_device_start[1].x, data.tk_device_start[1].y)
        if len(data.tk_device_end) == 2:
            self.end_A = Point(data.tk_device_end[0].x, data.tk_device_end[0].y)
            self.end_B = Point(data.tk_device_end[1].x, data.tk_device_end[1].y)
        else:
            self.end_A = self.start_A
            self.end_B = self.start_B

        if len(self.cones_left) is 0 or len(self.cones_right) is 0:
            return
        polygon_left = Polygon(self.cones_left)
        polygon_right = Polygon(self.cones_right)

        if polygon_right.contains(Point(self.cones_left[-1][0], self.cones_left[-1][1])):
            self.polygon_outside = polygon_right
            self.polygon_inside = polygon_left
        else:
            self.polygon_outside = polygon_left
            self.polygon_inside = polygon_right

        self.received_track = True
