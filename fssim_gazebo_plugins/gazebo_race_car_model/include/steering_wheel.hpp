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

#ifndef FSSIM_GAZEBO_STEERING_WHEEL_HPP
#define FSSIM_GAZEBO_STEERING_WHEEL_HPP

#include "wheel.hpp"

namespace gazebo {
namespace fssim {

class WheelStearing : public Wheel {
 public:

    WheelStearing(physics::ModelPtr &_model,
                  sdf::ElementPtr &_sdf,
                  const std::string _name,
                  transport::NodePtr &gznode,
                  boost::shared_ptr<ros::NodeHandle> &nh) :
        Wheel(_model, _sdf, _name, gznode, nh) {
        max_steer_            = 1.0;
        std::string full_name = _model->GetName() + "::" + _sdf->Get<std::string>(_name + "_steering");
        getJoint(steering_joint_, _model, full_name);
    }

    void setAngle(const double delta) override {
        steering_joint_->SetPosition(0, delta);
    }

    void printInfo() override {
        Wheel::printInfo();
        ROS_DEBUG("\t - STEERING");
    }

    bool isSteering() override { return true; }

 private:

    physics::JointPtr steering_joint_;      // Joint Holder

    double max_steer_;                      // Maximal steering angle [rad]
};
}
}

#endif //FSSIM_GAZEBO_STEERING_WHEEL_HPP
