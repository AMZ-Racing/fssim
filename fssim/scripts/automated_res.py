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


# ROS Include
import rospkg
import rospy
import tf
import tf.transformations
import tf2_ros

# ROS Msgs
from fssim_common.msg import SimHealth, ResState, TopicsHealth, Mission, Track, State
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, TransformStamped

# Python Import
import os

# YAML
import yaml

# Arguments
import argparse

# Process
from fssim.shell import *
from fssim.statistics import *
from fssim.ecu import *
from fssim.vehicle_position_validate import *


def generate_track_model(track_name):
    """
    Hacky way of changing track in gazebo
    :param track_name:
    """
    # type: (str) -> None

    rospack = rospkg.RosPack()
    fssim_gazebo = rospack.get_path('fssim_gazebo')
    xacro_file = fssim_gazebo + '/models/track/model.xacro'
    xacro_out_file = fssim_gazebo + '/models/track/model.config'
    command = "xacro --inorder {} track_name:={} > {}".format(xacro_file, track_name, xacro_out_file)
    xacro_proccess = subprocess.check_output(command, shell = True, stderr = subprocess.STDOUT)


def parse_path(path, pkg_config_storage):
    '''
    Parses path based on whether it is global path or wrt to pkg_pkg_config_storage
    :param path:
    :param pkg_config_storage:
    :return: path to string
    '''
    # type: (str, str) -> str
    if path[0] == '/':  # Global path
        return path
    else:  # Path wrt pkg_config_storage
        return rospkg.RosPack().get_path(pkg_config_storage) + "/" + path


class AutomatedRes:

    def __init__(self, arg):

        # Use sim clock
        rospy.set_param('/use_sim_time', 'true')

        # TF Initializations
        self.listener = tf.TransformListener()
        self.br = tf2_ros.StaticTransformBroadcaster()

        # Set Default variables
        self.delay_after_start_command = 2.0
        self.sim_health = SimHealth()

        self.ecu = Ecu()

        # ROS Subscribers
        self.sub_topics_health = rospy.Subscriber('/fssim/topics_health', TopicsHealth, self.callback_topics_health)
        self.sub_state = rospy.Subscriber('/fssim/base_pose_ground_truth', State, self.callback_state)
        self.sub_mission_finished = rospy.Subscriber('/fssim/mission_finished', Mission, self.callback_mission_finished)

        # ROS Publishers
        self.pub_res = rospy.Publisher('/fssim/res_state', ResState, queue_size = 1)
        self.pub_health = rospy.Publisher('/fssim/health', SimHealth, queue_size = 1)
        self.pub_mission = rospy.Publisher('/fssim/mission', Mission, queue_size = 1, latch = True)
        self.pub_initialpose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 1)

        with open(arg.config, 'r') as f:
            self.sim_config = yaml.load(f)
        self.sim_config_id = arg.sim_id

        self.checks_is_in_track = self.sim_config["res"]["checks"]["is_in_track"]

        self.robot_name = self.sim_config["robot_name"]
        rospy.set_param("robot_name", self.robot_name)

        # Initialize all needed Commands
        self.cmd_autonomous_system = None
        self.cmd_fssim = None
        self.cmd_rosbag = None

        self.mission = Mission()
        if "skidpad" in self.sim_config['repetitions'][self.sim_config_id]['track_name']:
            self.mission.mission = "skidpad"
            self.checks_is_in_track = False
        elif "acceleration" in self.sim_config['repetitions'][self.sim_config_id]['track_name']:
            self.mission.mission = "acceleration"
            self.checks_is_in_track = False
        else:
            self.mission.mission = "trackdrive"

        rospy.logwarn("Mission: %s", self.mission.mission)
        self.pub_mission.publish(self.mission)

        self.output_folder = arg.output
        self.statistics = LapStaticstic(self.output_folder, self.mission.mission)

        self.track_checks = VehiclePositionCheck(self.mission.mission, self.checks_is_in_track)

        signal.signal(signal.SIGINT, self.signal_handler)

        # Initialize starting time
        self.start_time = 0

    def run(self):
        """
        Run one repetition
        :param self: The class
        :rtype: None
        """

        # Launch FSSIM
        success = self.launch_fssim(self.sim_config['repetitions'][self.sim_config_id],
                                    self.sim_config['pkg_config_storage'])
        if not success:
            return

        # Send initial position if available
        if self.track_checks.track_details is not None:
            rospy.logwarn("Setting Initial Pose")
            pose_init = PoseWithCovarianceStamped()
            pose_init.pose.pose.position, pose_init.pose.pose.orientation = self.track_checks.get_track_init_pos()
            self.pub_initialpose.publish(pose_init)
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "fssim_map"
            t.child_frame_id = 'map'
            t.transform.translation.z = 0.0
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.rotation.w = 1.0
            self.br.sendTransform(t)

        # Wait for short time
        rospy.sleep(5.0)

        # Launch Autonomous System
        self.cmd_autonomous_system = self.launch_file(
            self.sim_config['repetitions'][self.sim_config_id]['autonomous_stack'])

        # Launch Recording
        raw_rosbag_launch_cmd = self.sim_config['launch_file']['rosbag_record']
        gen_rosbag_name = self.statistics.get_rosbag_name(self.output_folder, self.sim_config_id)
        if gen_rosbag_name is not None:
            self.cmd_rosbag = self.launch_file(raw_rosbag_launch_cmd.format(gen_rosbag_name))

        # Wait for short time
        rospy.sleep(5.0)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # Check if all 4 wheels are inside of track and print warning if not
            is_inside_of_track = self.track_checks.is_car_in_track()

            if not is_inside_of_track:
                rospy.logwarn("Car Is not in the track")
                rospy.sleep(0.5)

            # Stop sim after max time achieved
            time_expired = self.sim_time_expired()

            # If we do too many laps stop as well
            request_stop_given_mission = self.statistics.request_stop()

            # Terminate simulation is any of conditions are met
            stop_simulation = not is_inside_of_track or time_expired or self.ecu.state == EcuState.FINNISHED_DISCIPLINE or request_stop_given_mission

            if not stop_simulation and not self.sim_health.vehicle_started:

                # Log Starting time
                self.start_time = rospy.Time.now().to_sec()

                # Start ECU
                self.ecu.start()

                # Enable throttle for the car
                rospy.logwarn("Sending RES GO")
                self.send_res(True)

                # Mark car as available for high level control to be moved
                self.sim_health.vehicle_started = True

            elif not is_inside_of_track and self.sim_health.vehicle_started or stop_simulation:

                rospy.logwarn("Sending EMERGENCY STOP")
                self.send_res(False)

                # This sleeps ensures that there is still at least 5s of rosbag available
                rospy.sleep(5.0)

                # Request shutdown of all commands
                self.sim_health.request_shutdown = True
                break

            # Stream health of FSSIM
            self.pub_health.publish(self.sim_health)
            try:
                rate.sleep()
            except:
                print "ROSCORE has been killed during sleep"

        # Write statisctics into a file
        self.statistics.write_report(self.sim_config_id, self.ecu.state == EcuState.FINNISHED_DISCIPLINE)

        rospy.logwarn("EXITING CURRENT SCENARIO")

        # Terminate everything what belongs to this repetition
        self.terminate_all_launched_commands()

        rospy.logwarn("***** EXITING Repetition *****")
        rospy.signal_shutdown("EXIT AUTOMATED RES")

    def sim_time_expired(self):
        time_expired = False
        repetition_time = rospy.Time.now().to_sec() - self.start_time
        if 'kill_after' in self.sim_config and self.sim_config['kill_after'] is not 0:
            if self.sim_health.vehicle_started and repetition_time > self.sim_config['kill_after']:
                time_expired = True
                rospy.logerr("Simulation Time EXPIRED with: %f", repetition_time)
        return time_expired

    def launch_fssim(self, settings, pkg_config_storage = 'fssim_description'):
        """
        Parse all settings and launch FSSIM
        :param settings: dictionary containing all settings
        :param pkg_config_storage: package wrt to which the settings are locates (If settings are passed as a global path
        then this parameter is neglected
        """
        # type: (dict, str) -> None

        # We are sure the config is going to be in fssim_description
        if pkg_config_storage is None:
            pkg_config_storage = 'fssim_description'

        # Create Track Config
        track_name = settings['track_name']
        generate_track_model(track_name)

        track_name_without_extension = os.path.splitext(track_name)[0]
        rospy.logwarn("track: %s", track_name_without_extension)
        track_detailed_description = rospkg.RosPack().get_path(
            'fssim_gazebo') + '/models/track/tracks_yaml/' + track_name_without_extension + '.yaml'

        if os.path.isfile(track_detailed_description):
            rospy.logwarn("Found detailed description")
            with open(track_detailed_description, 'r') as f:
                self.track_details = yaml.load(f)
            self.track_checks.track_details = self.track_details

        # Parse Config paths
        config_folder = rospkg.RosPack().get_path('fssim_description') + '/cars/' + self.robot_name
        sensors_config_file = config_folder + '/config/sensors.yaml'
        car_dimensions_file = config_folder + '/config/distances.yaml'
        car_config_file = config_folder + '/config/car.yaml'
        if 'sensors_config_file' in settings:
            sensors_config_file = parse_path(settings['sensors_config_file'], pkg_config_storage)
        if 'car_dimensions_file' in settings:
            car_dimensions_file = parse_path(settings['car_dimensions_file'], pkg_config_storage)
        if 'car_config_file' in settings:
            car_config_file = parse_path(settings['car_config_file'], pkg_config_storage)

        # Print Config files
        rospy.logwarn("Sensors Config: %s", sensors_config_file)
        rospy.logwarn("Car Config:     %s", car_config_file)
        rospy.logwarn("Car Dim:        %s", car_dimensions_file)
        rospy.logwarn("Track Name:     %s", track_name)

        # Construct fssim command
        fssim_command = "roslaunch fssim fssim.launch " \
                        "sensors_config_file:={} " \
                        "car_config_file:={} " \
                        "car_dimensions_file:={} " \
                        "robot_name:={}".format(sensors_config_file, car_config_file, car_dimensions_file,
                                                self.robot_name)

        ##################
        ##  Start FSSIM ##
        ##################
        self.cmd_fssim = self.launch_file(fssim_command)

        # Wait until track message is received
        while (not rospy.is_shutdown()) \
                and (not self.track_checks.is_track_valid() or not self.statistics.state_received):
            rospy.logwarn_throttle(1, "Waiting for FSSIM to load")
            rospy.Rate(2).sleep()

        if rospy.is_shutdown():
            return False

        # Set starting and end point
        self.statistics.start_A = self.track_checks.start_A
        self.statistics.start_B = self.track_checks.start_B
        self.statistics.end_A = self.track_checks.end_A
        self.statistics.end_B = self.track_checks.end_B

        return True

    def launch_file(self, launch_file):
        '''
        Start launch file and return the Command
        :param launch_file:
        :type launch_file: str
        :return: Command which was launched
        :rtype: Command
        '''
        if launch_file is not None:
            process = Command(launch_file)
            process.run()
            rospy.logwarn("LAUNCHING: \n%s", launch_file)
            rospy.sleep(self.delay_after_start_command)
            # Wait for short time
            rospy.sleep(5.0)
            rospy.logwarn("LAUNCHED AUTONOMOUS SYSTEM")
            return process
        else:
            rospy.logwarn("Empty Launch File was given")
            return None

    def callback_topics_health(self, data):
        '''
        Callback for ROS Subscriber
        :param data:
        :type data: SimHealth
        :return:
        :rtype: None
        '''
        self.sim_health.topics_health = data

    def callback_state(self, data):
        '''
        Callback for ROS Subscriber
        :param data: ROS Msg
        :type data: Odometry
        :return: None
        '''
        self.statistics.update_state(data)
        self.ecu.update_state(data)

    def callback_mission_finished(self, data):
        '''
        Callback for ROS Subscriber
        :param data: ROS Msg
        :type data: Mission
        :return: None
        '''
        self.ecu.mission_finished(data)
        if data.finished:
            rospy.logwarn("Received Mission Finnished TRUE -->> Sending RES STOP")
            self.send_res(False)

    def terminate_all_launched_commands(self):
        '''
        Terminate everything was has been launched
        :return: None
        '''
        if self.cmd_rosbag is not None:
            self.cmd_rosbag.ensure_terminated()
        if self.cmd_autonomous_system is not None:
            self.cmd_autonomous_system.ensure_terminated()
        self.cmd_fssim.ensure_terminated()

    def send_res(self, trigger = False):
        '''
        Send either res go signal or emergency button
        :param trigger:
        :return:
        '''
        res = ResState()
        res.push_button = trigger
        res.emergency = not trigger
        self.pub_res.publish(res)

    def signal_handler(self, signal, frame):
        self.terminate_all_launched_commands()
        rospy.logwarn('####################################################################')
        rospy.logwarn('########################## EXITING FSSIM! ##########################')
        rospy.logwarn('####################################################################')
        rospy.signal_shutdown("EXIT AUTOMATED RES")

    def get_trans(self, target):
        try:
            (trans, rot) = self.listener.lookupTransform('/fssim_map', '/fssim/vehicle/' + target, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # rospy.logwarn("Could not find transform from %s to %s", 'fssim_map', 'fluela/base_link')
            rospy.sleep(1)
            return [0.0, 0.0]
        return trans


def start_roscore():
    # Create new roscore if non-existent
    roscore = Command("roscore")
    try:
        rosgraph.Master('/rostopic').getPid()
        rospy.logwarn("ROSCORE IS ALREADY RUNNING")
    except socket.error:
        roscore.run()
        time.sleep(1.0)  # wait a bit to be sure the roscore is really launched
        print '\033[93m', "ROSCORE STARTED HERE", '\033[0m'
    return roscore


if __name__ == '__main__':
    import rosgraph
    import socket

    parser = argparse.ArgumentParser(description = 'Process some integers.')
    parser.add_argument("--config", dest = "config", metavar = "FILE", help = "Config YAML file",
                        default = rospkg.RosPack().get_path('fssim') + '/config/simulation.yaml')
    parser.add_argument("--output", dest = "output", metavar = "FOLDER", help = "Output YAML file")
    parser.add_argument("--id", dest = "sim_id", help = "Config ID in YAML file", default = 0, type = int)
    args, unknown = parser.parse_known_args()
    args.output = os.path.abspath(args.output) if args.output is not None else None

    roscore = start_roscore()

    rospy.init_node('automated_res')

    automated_res = AutomatedRes(args)

    try:
        automated_res.run()
    except:
        rospy.logerr("SOMETHING GOT WRONG")
        automated_res.terminate_all_launched_commands()

    roscore.ensure_terminated()

    print "THIS REPETITION IS OVER"
