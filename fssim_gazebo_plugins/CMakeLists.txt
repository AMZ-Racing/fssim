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

cmake_minimum_required(VERSION 2.8.3)
project(fssim_gazebo_plugins)
set(CMAKE_CXX_STANDARD 11)

include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
        ${SDFormat_INCLUDE_DIRS}
        ${OGRE_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
        gazebo_msgs
        roscpp
        fssim_common
        sensor_msgs
        nav_msgs
        tf
        pcl_ros
        visualization_msgs)

find_package(gazebo 9 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
        INCLUDE_DIRS
        CATKIN_DEPENDS tf pcl_ros
        LIBRARIES pcl_ros
)

add_subdirectory(gazebo_utils)
add_subdirectory(gazebo_race_car_model)
add_subdirectory(gazebo_cone_sensor)
add_subdirectory(gazebo_track)

