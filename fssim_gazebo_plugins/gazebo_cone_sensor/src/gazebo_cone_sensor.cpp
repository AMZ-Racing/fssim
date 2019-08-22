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

// STD Include
#include <algorithm>
#include <fstream>
#include <mutex>
#include <thread>
#include <chrono>
#include <thread>

// Main Include
#include "gazebo_cone_sensor.hpp"

using namespace gazebo;

/////////////////////////////////////////////////
ConeSensor::ConeSensor() : rosnode(), track_(rosnode) {
}

/////////////////////////////////////////////////
ConeSensor::~ConeSensor() {
}

void ConeSensor::Reset() {
    ModelPlugin::Reset();
}

void ConeSensor::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    int  argc  = 0;
    char *argv = nullptr;
    ros::init(argc, &argv, _sdf->Get<std::string>("node_name"));

    ROS_INFO("Loading ConeSensor");
    model_ = _parent;

    gznode = transport::NodePtr(new transport::Node());
    gznode->Init();
    updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ConeSensor::Update, this));

    track_.load(_parent, _sdf);
    dt_required_ = 1.0 / track_.getRate();
}
void ConeSensor::Update() {
    std::lock_guard<std::mutex> lock(mutex);

    common::Time cur_time = model_->GetWorld()->SimTime();
    double       dt;

    if (!isLoopTime(cur_time, dt)) {
        return;
    }

    last_sim_time_ = cur_time;

    track_.update();
}

bool ConeSensor::isLoopTime(const common::Time &time, double &dt) {
    dt = (time - last_sim_time_).Double();
    if (dt < 0) {
        this->Reset();
        return false;
    } else if (dt >= dt_required_) {
        return true;
    }
    return false;
}

GZ_REGISTER_MODEL_PLUGIN(ConeSensor)
