/*
 * AMZ-Driverless
 * Copyright (c) 2018 Authors:
 *   - Juraj Kabzan <kabzanj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef FSSIM_GAZEBO_PLUGINS_ROS_UTILLS_HPP
#define FSSIM_GAZEBO_PLUGINS_ROS_UTILLS_HPP

namespace gazebo {
namespace ros_utils {

inline tf::Vector3 toVector(const Eigen::Vector3d &v) {
    tf::Vector3 ret;
    ret.setX(v.x());
    ret.setY(v.y());
    ret.setZ(v.z());
    return ret;
}

inline Eigen::Vector3d toVector(const tf::Vector3 &v) {
    return {v.x(), v.y(), v.z()};
}

}  // namespace ros_utiks
}  // namespace gazebo

#endif //FSSIM_GAZEBO_PLUGINS_ROS_UTILLS_HPP
