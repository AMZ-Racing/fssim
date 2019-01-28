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

# Python Includes
import time
import yaml

# Threading
import threading

# ROS Include
import rospy
import rostopic

# ROS Msgs
from fssim_common.msg import TopicsHealth, TopicState


def get_rostopic_hz(topic, window_size=-1, filter_expr=None, use_wtime=False, avg=2):
    """
    Periodically print the publishing rate of a topic to console until
    shutdown
    :param topic: topic names, ``list`` of ``str``
    :param window_size: number of messages to average over, -1 for infinite, ``int``
    :param filter_expr: Python filter expression that is called with m, the message instance
    """
    rostopic._check_master()
    if rospy.is_shutdown():
        return
    rt = rostopic.ROSTopicHz(window_size, filter_expr=filter_expr, use_wtime=use_wtime)
    msg_class, real_topic, _ = rostopic.get_topic_class(topic, blocking=False)

    if real_topic is None:
        return 0.0

    # pause hz until topic is published
    # we use a large buffer size as we don't know what sort of messages we're dealing with.
    # may parameterize this in the future
    if filter_expr is not None:
        # have to subscribe with topic_type
        rospy.Subscriber(real_topic, msg_class, rt.callback_hz, callback_args=topic)
    else:
        rospy.Subscriber(real_topic, rospy.AnyMsg, rt.callback_hz, callback_args=topic)

    # if rospy.get_param('use_sim_time', False):
    #     print("WARNING: may be using simulated time",file=sys.stderr)

    rostopic._sleep(1.0)
    hz_avg = 0.0
    for i in range(0, avg):
        hz_cur = rt.get_hz(topic)
        if hz_cur:
            hz_avg = hz_avg + hz_cur[0]
        rostopic._sleep(0.5)
    return hz_avg / avg


def get_resulting_min_frequency(topic_info, precision=10.0):
    f = topic_info.expected_frequency
    return f - f * (precision / 100.0)


def toRos(topic):
    msg = TopicState()
    msg.topic_name = topic['topic']
    msg.expected_frequency = topic['expected_frequency']
    msg.passed = False
    msg.measured_frequency = 0.0
    return msg


class MonitorTopics():

    def __init__(self, topics_frequency):
        self.topics_health = TopicsHealth()
        self.pub_topics_health = rospy.Publisher("/fssim/topics_health", TopicsHealth, queue_size=1)
        for i, t in enumerate(topics_frequency):
            self.topics_health.topics_check.append(toRos(t))

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.topics_health.topics_check, self.topics_health.topics_check_passed = self.check_topics_hz(
                self.topics_health.topics_check)
            self.pub_topics_health.publish(self.topics_health)
            try:
                rate.sleep()
            except:
                print "ROSCORE has been killed during sleep"

    def check_topics_hz(self, topic_config, precision=10.0):
        success = True
        for i, t in enumerate(topic_config):
            if rospy.is_shutdown():
                break
            hz_cur = get_rostopic_hz(t.topic_name, avg=1)
            hz_exp = get_resulting_min_frequency(t, precision)
            topic_config[i].measured_frequency = hz_cur
            if hz_cur >= hz_exp:
                rospy.logdebug("Topic: %s passed!", t.topic_name)
                topic_config[i].passed = True
            else:
                success = False
                rospy.logwarn("Topic: %s failed with real: %f [Hz] and expected: %f [Hz]", t.topic_name, hz_cur,
                              hz_exp)
                topic_config[i].passed = False

        return topic_config, success


if __name__ == '__main__':
    rospy.init_node('topics_monitor')

    topics_config = rospy.get_param("~topics_frequency_config")

    with open(topics_config, 'r') as f:
        topics_frequency_config = yaml.load(f)

    topic_monitor = MonitorTopics(topics_frequency_config['topics'])
    topic_monitor.run()
