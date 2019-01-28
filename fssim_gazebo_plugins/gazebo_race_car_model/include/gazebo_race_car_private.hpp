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


#ifndef FSSIM_GAZEBO_RACECARPLUGINPRIVATE_H
#define FSSIM_GAZEBO_RACECARPLUGINPRIVATE_H

// ROS Includes
#include <ros/ros.h>

// Gazebo Includes
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Plugin.hh>

// ROS RACE CAR PLUGIN
#include "vehicle.hpp"

namespace gazebo {

class gazebo_race_car_private {
 public:
    boost::shared_ptr<ros::NodeHandle> rosnode;

    physics::WorldPtr world;

    physics::ModelPtr model;

    transport::NodePtr gznode;

 public:

    event::ConnectionPtr updateConnection;

    fssim::VehiclePtr vehicle;

    common::Time lastSimTime;

    transport::SubscriberPtr keyboardSub;

    std::mutex mutex;

    transport::PublisherPtr worldControlPub;

};

} // namespace gazebo

#endif //FSSIM_GAZEBO_RACECARPLUGINPRIVATE_H
