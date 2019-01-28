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

#ifndef FSSIM_GAZEBO_DEFINITIONS_HPP
#define FSSIM_GAZEBO_DEFINITIONS_HPP

#include "fssim_common/State.h"

namespace gazebo {
namespace fssim {

struct State {

    State operator*(const double &dt) const {
        return {dt * x, dt * y, dt * yaw, dt * v_x, dt * v_y, dt * r, dt * a_x, dt * a_y};
    }

    State operator+(const State &x2) const {
        return {x + x2.x, y + x2.y, yaw + x2.yaw, v_x + x2.v_x, v_y + x2.v_y, r + x2.r, a_x + x2.a_x, a_y + x2.a_y};
    }

    std::stringstream &operator<<(std::stringstream &os) {
        os << getString();
    }

    inline std::string getString() const {
        std::string str = "x:" + std::to_string(x)
                          + "| y:" + std::to_string(y)
                          + "| yaw:" + std::to_string(yaw)
                          + "| v_x:" + std::to_string(v_x)
                          + "| v_y:" + std::to_string(v_y)
                          + "| r:" + std::to_string(r)
                          + "| a_x:" + std::to_string(a_x)
                          + "| a_y:" + std::to_string(a_y);
        return str;
    }

    fssim_common::State toRos(const ros::Time &time) const {
        fssim_common::State state;
        state.header.stamp    = time;
        state.header.frame_id = "fssim_map";
        state.x               = x;
        state.y               = y;
        state.yaw             = yaw;
        state.vx              = v_x;
        state.vy              = v_y;
        state.r               = r;
        return state;
    }

    void validate() { v_x = std::max(0.0, v_x); }

    double x;
    double y;
    double yaw;
    double v_x;
    double v_y;
    double r;
    double a_x;
    double a_y;
};

struct Input {
    Input() : dc(0.0), delta(0.0) {}
    std::string getString() {
        return "dc:" + std::to_string(dc) + " | delta:" + std::to_string(delta);
    }
    double dc;
    double delta;
};

} // namespace fssim
} // namespace gazebo
#endif //FSSIM_GAZEBO_DEFINITIONS_HPP
