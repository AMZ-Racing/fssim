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

#ifndef FSSIM_GAZEBO_GAZEBOUTILLS_H
#define FSSIM_GAZEBO_GAZEBOUTILLS_H

// STD Includes
#include <string>

// Eigen Include
#include <Eigen/Dense>

namespace gazebo {

template<class Joint, class Model>
void getJoint(Joint &joint, Model &model, const std::string &name) {
    joint = model->GetJoint(name);
    if (!joint) {
        gzerr << "could not find " << name << " joint" << std::endl;
    }
}

template<class Link, class Model>
void getLink(Link &link, Model &model, const std::string &name) {
    link = model->GetLink(name);
    if (!link) {
        gzerr << "could not find " << name << " link" << std::endl;
    }
}

template<class type>
type getParam(sdf::ElementPtr &sdf, std::string paramName, type paramDefault) {
    type para;
    if (sdf->HasElement(paramName)) {
        para = sdf->Get<type>(paramName);
    } else {
        gzwarn << "Setting default value for: " << paramName << ":" << paramDefault << std::endl;
        para = paramDefault;
    }
    return para;
}

template<class type>
type getParam(sdf::ElementPtr &sdf, std::string paramName) {
    type para;
    if (sdf->HasElement(paramName)) {
        para = sdf->Get<type>(paramName);
    } else {
        para = type();
    }
    return para;
}

} // namespace gazebo

inline Eigen::Vector3d toVector(const boost::shared_ptr<gazebo::physics::Entity> &v) {
    return {v->WorldPose().Pos().X(), v->WorldPose().Pos().Y(), 0.0};
}

#endif //FSSIM_GAZEBO_GAZEBOUTILLS_H
