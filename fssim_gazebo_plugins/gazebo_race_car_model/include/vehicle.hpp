/*
 * AMZ-Driverless
 * Copyright (c) 2018 Authors:
 *   - Juraj Kabzan <kabzanj@gmail.com>
 *   - Miguel de la Iglesia Valls <dmiguel@ethz.ch>
 *   - Manuel Dangel <mdangel@student.ethz.ch> 
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
#ifndef FSSIM_GAZEBO_VEHICLE_H
#define FSSIM_GAZEBO_VEHICLE_H

// FSSIM Includes
#include "axle.hpp"
#include "aero.hpp"

// ROS Msgs
#include "fssim_common/CarInfo.h"
#include "fssim_common/Cmd.h"
#include "fssim_common/WheelSpeeds.h"
#include "fssim_common/ResState.h"
#include "fssim_common/Mission.h"
#include "fssim_common/State.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

// Utills
#include "gazebo_utils/include/gazebo_to_ros.hpp"

// ROS
#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

namespace gazebo {
namespace fssim {

class Vehicle {
 public:
    Vehicle(physics::ModelPtr &_model,
            sdf::ElementPtr &_sdf,
            boost::shared_ptr<ros::NodeHandle> &nh,
            transport::NodePtr &gznode);

    void onRes(const fssim_common::ResStateConstPtr &msg);

    void update(double dt);

    void printInfo();

    void publish(double sim_time);

 private:

    State f(const State &x,
            const Input &u,
            double Fx,
            double M_TV,
            const AxleTires &FyF,
            const AxleTires &FyR);

    State f_kin_correction(const State &x_in,
                           const State &x_state,
                           const Input &u,
                           const double Fx,
                           const double M_TV,
                           const AxleTires &FyF,
                           const AxleTires &FyR,
                           const double dt);


    void setPositionFromWorld();

    void publishTf(const State &x);

    void publishCarInfo(const AxleTires &alphaF,
                        const AxleTires &alphaR,
                        const AxleTires &FyF,
                        const AxleTires &FyR,
                        const double Fx) const;

    double getFx(const State &x, const Input &u);

    double getMTv(const State &x, const Input &u) const;

    void onCmd(const fssim_common::CmdConstPtr &msg);

    void onInitialPose(const geometry_msgs::PoseWithCovarianceStamped &msg);

    void initModel(sdf::ElementPtr &_sdf);

    void initVehicleParam(sdf::ElementPtr &_sdf);

    double getNormalForce(const State &x);

    void setModelState(const State &x);

    double getGaussianNoise(double mean, double var) const;

    State &getState() { return state_; }

    Input &getInput() { return input_; }

 private:

    // ROS Nodehandle
    boost::shared_ptr<ros::NodeHandle> nh_;

    // ROS Publishrs
    ros::Publisher pub_ground_truth_;
    ros::Publisher pub_car_info_;

    // ROS Subscribers
    ros::Subscriber sub_cmd_;
    ros::Subscriber sub_initial_pose_;
    ros::Subscriber sub_res_;

    // ROS TF
    tf::TransformBroadcaster tf_br_;

    /// Pointer to the parent model
    physics::ModelPtr model;

    // Variable for debug
    double resistance_force, down_force;

    /// Chassis link and Base Link
    physics::LinkPtr chassisLink;
    physics::LinkPtr base_link_;

    // Front and Rear Axle
    FrontAxle front_axle_;
    RearAxle  rear_axle_;

    // Car Infor and RES
    fssim_common::CarInfo  car_info_;
    fssim_common::ResState res_state_;

    // Parameters
    Param param_;

    // States
    State state_;
    Input input_;

    struct InputStamp{
        InputStamp(const ros::Time t_, const double v_){
            t = t_; v = v_;
        }
        ros::Time t;
        double v;
    };

    ros::Time time_last_cmd_;


    std::list<InputStamp> deltas_;
    std::list<InputStamp> dcs_;

    // Consider Aerodynamics
    Aero aero_;

    // Name of the System
    std::string robot_name_;
};

typedef std::unique_ptr<Vehicle> VehiclePtr;

} // namespace fssim
} // namespace gazebo

#endif //FSSIM_GAZEBO_VEHICLE_H
