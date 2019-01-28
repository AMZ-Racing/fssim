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

# ROS related imports
import rosbag


class BagMessageFilter:

    def __init__(self, filename = None):
        self.file_name = filename

    def get_topic_names(self, msg):
        print " Opening bag:",self.file_name
        bag = rosbag.Bag(self.file_name)

        val = bag.get_type_and_topic_info()[1].values()
        types = []
        for i in range(0, len(val)):
            types.append(val[i][0])

        indices =  [i for i, j in enumerate(types) if j == msg._type]
        topics = bag.get_type_and_topic_info()[1].keys()
        matching_topics = [topics[j] for i, j in enumerate(indices)]

        bag.close()
        return matching_topics


