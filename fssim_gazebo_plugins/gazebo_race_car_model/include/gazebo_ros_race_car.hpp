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


#ifndef GAZEBO_ROS_RACE_CAR_HPP
#define GAZEBO_ROS_RACE_CAR_HPP

// Structure holding run-time important objects
#include "gazebo_race_car_private.hpp"

namespace gazebo {

class RaceCarModelPlugin : public ModelPlugin {
 public:

    RaceCarModelPlugin();

    ~RaceCarModelPlugin() override;

    void Reset() override;

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

 private:

    void update();

    void publishInfo();

    bool isLoopTime(const common::Time &time, double &dt);

    std::unique_ptr<gazebo_race_car_private> dataPtr;
};

} // namespace gazebo
#endif // GAZEBO_ROS_RACE_CAR_HPP
