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

#ifndef FSSIM_GAZEBO_AXLE_HPP
#define FSSIM_GAZEBO_AXLE_HPP

#include "steering_wheel.hpp"

namespace gazebo {
namespace fssim {

struct AxleTires {
    double left;
    double right;

    double avg() const { return (left + right) / 2.0; }
};

template<class WheelType>
class Axle {
 public:

    Axle(physics::ModelPtr &_model,
         sdf::ElementPtr &_sdf,
         const std::string name,
         transport::NodePtr &gznode,
         boost::shared_ptr<ros::NodeHandle> &nh);

    const ignition::math::Vector3<double> &getAxlePos() const;

    void printInfo();

    void getSlipAngles(const State &x, const Input &u, double &alphaL, double &alphaR);

    void getFy(const State &x, const Input &u, const double Fz, AxleTires &Fy, AxleTires *alpha = nullptr);

    void setSteering(double delta);

    void setLeverArm(double _car_length, double _weight_factor, double _width);

    double getDownForce(const double Fz);

    void setParam(Param param);

    const WheelType &getWheelLeft() const { return wheel_l_; }
    const WheelType &getWheelRight() const { return wheel_r_; }

 private:
    // Name of the model
    std::string name_;

    // Left and Right wheel
    WheelType wheel_l_;
    WheelType wheel_r_;

    // Center Position of the axle
    ignition::math::Vector3<double> axlePos;

    // Parameters
    double axle_width_;        // Width [m]
    double lever_arm_length_;  // COG to middle axle [m]
    double car_length_;        // Length of the car [m]
    double weight_factor_;     // Weight distribution [m]

    int axle_factor_;      // Decides whether its front or rear [no units]

};

typedef Axle<Wheel>         RearAxle;
typedef Axle<WheelStearing> FrontAxle;

} // namespace fssim
} // namespace gazebo

#endif //FSSIM_GAZEBO_AXLE_HPP
