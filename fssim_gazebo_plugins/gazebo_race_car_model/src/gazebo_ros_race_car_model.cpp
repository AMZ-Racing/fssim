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
#include "gazebo_ros_race_car.hpp"

// STD Include
#include <algorithm>
#include <fstream>
#include <mutex>
#include <thread>

namespace gazebo {

RaceCarModelPlugin::RaceCarModelPlugin()
    : dataPtr(new gazebo_race_car_private) {
    int  argc  = 0;
    char *argv = nullptr;
    ros::init(argc, &argv, "RaceCarModelPlugin");
    this->dataPtr->rosnode = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());
}

RaceCarModelPlugin::~RaceCarModelPlugin() {
    this->dataPtr->updateConnection.reset();
}

void RaceCarModelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    gzmsg << "Loading RaceCarModelPlugin" << std::endl;
    gzmsg << "RaceCarModelPlugin loading params" << std::endl;

    this->dataPtr->model = _model;

    this->dataPtr->world = this->dataPtr->model->GetWorld();

    this->dataPtr->gznode = transport::NodePtr(new transport::Node());

    this->dataPtr->gznode->Init();

    this->dataPtr->vehicle = std::unique_ptr<fssim::Vehicle>(
        new fssim::Vehicle(_model, _sdf, this->dataPtr->rosnode, this->dataPtr->gznode));
    this->dataPtr->vehicle->printInfo();

    this->dataPtr->updateConnection =
        event::Events::ConnectWorldUpdateBegin(std::bind(&RaceCarModelPlugin::update, this));

    this->dataPtr->worldControlPub = this->dataPtr->gznode->Advertise<msgs::WorldControl>("~/world_control");

    this->dataPtr->lastSimTime = this->dataPtr->world->SimTime();
}

void RaceCarModelPlugin::Reset() {
    this->dataPtr->lastSimTime = 0;
}

void RaceCarModelPlugin::update() {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

    publishInfo();

    common::Time curTime = this->dataPtr->world->SimTime();
    double       dt      = 0.0;
    if (!isLoopTime(this->dataPtr->world->SimTime(), dt)) {
        return;
    }

    this->dataPtr->lastSimTime = curTime;

    this->dataPtr->vehicle->update(dt);
}

void RaceCarModelPlugin::publishInfo() {
    this->dataPtr->vehicle->publish(this->dataPtr->world->SimTime().Double());
}

bool RaceCarModelPlugin::isLoopTime(const common::Time &time, double &dt) {
    dt = (time - this->dataPtr->lastSimTime).Double();
    if (dt < 0.0) {
        this->Reset();
        return false;
    } else if (ignition::math::equal(dt, 0.0)) {
        return false;
    }
    return true;
}

GZ_REGISTER_MODEL_PLUGIN(RaceCarModelPlugin)
} // namespace gazebo

