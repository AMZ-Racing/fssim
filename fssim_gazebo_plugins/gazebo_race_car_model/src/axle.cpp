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

// Vehicle Includes
#include "axle.hpp"
#include "steering_wheel.hpp"

namespace gazebo {
namespace fssim {

template<class WheelType>
Axle<WheelType>::Axle(physics::ModelPtr &_model,
                      sdf::ElementPtr &_sdf,
                      const std::string name,
                      transport::NodePtr &gznode,
                      boost::shared_ptr<ros::NodeHandle> &nh) : wheel_l_(_model, _sdf, name + "_left_wheel", gznode,
                                                                         nh),
                                                                wheel_r_(_model, _sdf, name + "_right_wheel", gznode,
                                                                         nh),
                                                                name_(name) {

    const auto vec3 = wheel_l_.getCenterPos() - wheel_r_.getCenterPos();
    axle_width_ = vec3.Length();

    axlePos = (wheel_l_.getCenterPos() + wheel_r_.getCenterPos()) / 2.0;

    axle_factor_ = name == "front" ? 1 : -1;
}

template<class WheelType>
const ignition::math::Vector3<double> &Axle<WheelType>::getAxlePos() const {
    return axlePos;
}

template<class WheelType>
void Axle<WheelType>::printInfo() {
    std::string str = "- Axle " + name_ + ":";
    ROS_DEBUG("%s", str.c_str());
    ROS_DEBUG("\t - axle_width_: %f", axle_width_);
    ROS_DEBUG("\t - lever_arm_length_: %f", lever_arm_length_);
    ROS_DEBUG("\t - axle_factor: %i", axle_factor_);
    ROS_DEBUG("\t - is steering: L: %i | R: %i", wheel_l_.isSteering(), wheel_r_.isSteering());
    wheel_l_.printInfo();
    wheel_r_.printInfo();
}

template<class WheelType>
void Axle<WheelType>::getSlipAngles(const State &x, const Input &u, double &alphaL, double &alphaR) {
    double v_x = std::max(1.0, x.v_x);
    alphaL = std::atan((x.v_y + axle_factor_ * lever_arm_length_ * x.r) / (v_x - 0.5 * axle_width_ * x.r))
             - u.delta * wheel_l_.isSteering();
    alphaR = std::atan((x.v_y + axle_factor_ * lever_arm_length_ * x.r) / (v_x + 0.5 * axle_width_ * x.r))
             - u.delta * wheel_r_.isSteering();
}

template<class WheelType>
void Axle<WheelType>::getFy(const State &x, const Input &u, const double Fz, AxleTires &Fy, AxleTires *alpha) {
    double alphaL, alphaR;
    getSlipAngles(x, u, alphaL, alphaR);

    const double Fz_axle = getDownForce(Fz);

    Fy.left  = wheel_l_.getFy(alphaL, Fz_axle);
    Fy.right = wheel_r_.getFy(alphaR, Fz_axle);

    if (alpha != nullptr) {
        alpha->left  = alphaL;
        alpha->right = alphaR;
    }
}

template<class WheelType>
void Axle<WheelType>::setSteering(const double delta) {
    wheel_l_.setAngle(delta);
    wheel_r_.setAngle(delta);
}

template<class WheelType>
void Axle<WheelType>::setLeverArm(const double _car_length, const double _weight_factor, const double _width) {
    car_length_       = _car_length;
    lever_arm_length_ = car_length_ * _weight_factor;
    weight_factor_    = _weight_factor;
    axle_width_       = _width;
}

template<class WheelType>
double Axle<WheelType>::getDownForce(const double Fz) {
    double FzAxle = 0.5 * (1.0 - weight_factor_) * Fz;
    return FzAxle;
}

template<class WheelType>
void Axle<WheelType>::setParam(const Param param) {
    wheel_l_.setParam(param.tire);
    wheel_r_.setParam(param.tire);
}

template
class Axle<WheelStearing>;
template
class Axle<Wheel>;

}  // namespace fssim
}  // namespace gazebo