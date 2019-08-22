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

#ifndef FSSIM_GAZEBO_WHEEL_H
#define FSSIM_GAZEBO_WHEEL_H

// Gazebo Include
#include <gazebo/physics/physics.hh>
#include <gazebo/common/PID.hh>

// STD Include
#include <string>

// FSSIM Include
#include "gazebo_utils/include/gazebo_utills.hpp"
#include "config.hpp"
#include "definitions.hpp"

// Ignition Robotics Includes
#include <ignition/math/Pose3.hh>

#include "ros/ros.h"
#include "ros/console.h"

// ROS Msgs
#include "visualization_msgs/MarkerArray.h"

namespace gazebo {
namespace fssim {

class Wheel {
 public:

    Wheel(physics::ModelPtr &_model,
          sdf::ElementPtr &_sdf,
          std::string _name,
          transport::NodePtr &gznode,
          boost::shared_ptr<ros::NodeHandle> &nh);

    virtual void printInfo();

    double getFy(double alpha, double Fz);

    virtual bool isSteering() { return false; }

    void setParam(const Param::Tire &param) { param_ = param; }

    virtual void setAngle(const double delta) {}

    const ignition::math::Vector3<double> &getCenterPos() const;

 private:

    double getCollisionRadius(physics::CollisionPtr _coll);

    physics::ModelPtr &model_;          // Reference to the model

    std::string full_name_;             // Name of the element

    Param::Tire param_;                 // Pacejka Tire parameters
    double      radius;                      // Radious of the tire from STL [m]

    ignition::math::Vector3<double> center_pos_;  // Center location of the tire
};

}  // namespace fssim
}  // namespace gazebo

#endif //FSSIM_GAZEBO_WHEEL_H
