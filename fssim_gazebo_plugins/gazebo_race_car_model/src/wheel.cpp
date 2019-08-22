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

// Main Include
#include "wheel.hpp"

namespace gazebo {
namespace fssim {

Wheel::Wheel(gazebo::physics::ModelPtr &_model,
             sdf::ElementPtr &_sdf,
             const std::string _name,
             gazebo::transport::NodePtr &gznode,
             boost::shared_ptr<ros::NodeHandle> &nh) : model_(_model) {

    full_name_ = _model->GetName() + "::" + _sdf->Get<std::string>(_name);
    physics::JointPtr joint;

    getJoint(joint, _model, full_name_);
    unsigned int id = 0;
    radius = getCollisionRadius(joint->GetChild()->GetCollision(id));

    center_pos_ = joint->GetChild()->GetCollision(id)->WorldPose().Pos();
}

double Wheel::getCollisionRadius(gazebo::physics::CollisionPtr _coll) {
    if (!_coll || !(_coll->GetShape())) { return 0; }

    if (_coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE)) {
        physics::CylinderShape *cyl =
                                   static_cast<physics::CylinderShape *>(_coll->GetShape().get());
        return cyl->GetRadius();
    } else if (_coll->GetShape()->HasType(physics::Base::SPHERE_SHAPE)) {
        physics::SphereShape *sph =
                                 static_cast<physics::SphereShape *>(_coll->GetShape().get());
        return sph->GetRadius();
    }
    return 0;
}

void Wheel::printInfo() {
    ROS_DEBUG("Name: %s", full_name_.c_str());
}

double Wheel::getFy(const double alpha, const double Fz) {
    const double B    = param_.B;
    const double C    = param_.C;
    const double D    = param_.D;
    const double E    = param_.E;
    const double mu_y = D * std::sin(C * std::atan(B * (1.0 - E) * alpha + E * std::atan(B * alpha)));
    const double Fy   = Fz * mu_y;
    return Fy;
}

const ignition::math::Vector3<double> &Wheel::getCenterPos() const { return center_pos_; }

}  // namespace fssim
}  // namespace gazebo