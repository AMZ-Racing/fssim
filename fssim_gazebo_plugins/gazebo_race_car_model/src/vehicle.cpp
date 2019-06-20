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

#include "vehicle.hpp"
#include "gazebo_utils/include/noise.hpp"

namespace gazebo {
namespace fssim {

Vehicle::Vehicle(physics::ModelPtr &_model,
                 sdf::ElementPtr &_sdf,
                 boost::shared_ptr<ros::NodeHandle> &nh,
                 transport::NodePtr &gznode)
    : nh_(nh),
      model(_model),
      front_axle_(_model, _sdf, "front", gznode, nh),
      rear_axle_(_model, _sdf, "rear", gznode, nh),
      aero_(param_.aero) {

    // ROS Publishers
    pub_ground_truth_ = nh->advertise<fssim_common::State>("/fssim/base_pose_ground_truth", 1);
    pub_car_info_     = nh->advertise<fssim_common::CarInfo>("/fssim/car_info", 1);

    // ROS Subscribers
    sub_res_          = nh->subscribe("/fssim/res_state", 1, &Vehicle::onRes, this);
    sub_cmd_          = nh->subscribe("/fssim/cmd", 1, &Vehicle::onCmd, this);
    sub_initial_pose_ = nh->subscribe("/initialpose", 1, &Vehicle::onInitialPose, this);

    // Initializatoin
    initModel(_sdf);
    initVehicleParam(_sdf);

    // Set Axle parameters
    front_axle_.setLeverArm(param_.kinematic.l, 1.0 - param_.kinematic.w_front, param_.kinematic.b_F);
    rear_axle_.setLeverArm(param_.kinematic.l, param_.kinematic.w_front, param_.kinematic.b_R);
    front_axle_.setParam(param_);
    rear_axle_.setParam(param_);

    setPositionFromWorld();

    time_last_cmd_ = ros::Time::now() - ros::Duration(10);
    input_.delta = 0;
    input_.dc = 0;
}

void Vehicle::setPositionFromWorld() {
    auto       pos   = model->GetWorldPose();
    const auto vel   = model->GetWorldLinearVel();
    const auto accel = model->GetWorldLinearAccel();
    const auto r     = model->GetWorldAngularVel();

    state_.x   = -1;
    state_.y   = pos.pos.y;
    state_.yaw = pos.rot.GetYaw();
    state_.v_x = 0.0;
    state_.v_y = 0.0;
    state_.r   = 0.0;
    state_.a_x = 0.0;
    state_.a_y = 0.0;
}

void Vehicle::initModel(sdf::ElementPtr &_sdf) {

    std::string chassisLinkName = model->GetName() + "::" + _sdf->Get<std::string>("chassis");
    getLink(chassisLink, model, chassisLinkName);

    std::string baseLinkName = model->GetName() + "::" + _sdf->Get<std::string>("base_link");
    getLink(base_link_, model, baseLinkName);

    // then the wheelbase is the distance between the axle centers
    auto vec3 = front_axle_.getAxlePos() - rear_axle_.getAxlePos();
    param_.kinematic.l = vec3.GetLength();
}

void Vehicle::initVehicleParam(sdf::ElementPtr &_sdf) {
    robot_name_ = getParam<std::string>(_sdf, "robot_name");

    std::string yaml_name = "config.yaml";

    yaml_name = getParam(_sdf, "yaml_config", yaml_name);
    initParam(param_, yaml_name);

    yaml_name = getParam(_sdf, "yaml_sensors", yaml_name);
    initParamSensors(param_, yaml_name);
}

void Vehicle::publish(const double sim_time) {

}

void Vehicle::update(const double dt) {
    //static ros::Time t_past = ros::Time::now();
    const auto t_now = ros::Time::now();
    //const double dt_new = (t_now - t_past).toSec();
    //std::cerr << "Elapsed time: " << dt << std::endl;
    //t_past = t_now;
     
    // Extract the commands
    while(!deltas_.empty()){
        auto first = deltas_.front();
        if((t_now - first.t).toSec() < dt){ // 0){
            break; 
        }
        input_.delta = first.v;
        deltas_.pop_front();
    }
    
    while(!dcs_.empty()){
        auto first = dcs_.front();
        if((t_now - first.t).toSec() < dt){ //0){
            break; 
        }
        input_.dc = first.v;
        dcs_.pop_front();
    }

    // Brake if cmd are not coming
    input_.dc = car_info_.torque_ok && (t_now - time_last_cmd_).toSec() < 1.0 ? input_.dc : -1.0;

    double Fz = getNormalForce(state_);

    // Tire Forces
    AxleTires FyF{}, FyR{}, alphaF{}, alphaR{};
    front_axle_.getFy(state_, input_, Fz, FyF, &alphaF);
    rear_axle_.getFy(state_, input_, Fz, FyR, &alphaR);
    front_axle_.setSteering(input_.delta);

    // Drivetrain Model
    const double Fx   = getFx(state_, input_);
    const double M_Tv = getMTv(state_, input_);

    // Dynamics
    const auto x_dot_dyn  = f(state_, input_, Fx, M_Tv, FyF, FyR);
    const auto x_next_dyn = state_ + x_dot_dyn * dt;
    state_ = x_next_dyn;
    state_ = f_kin_correction(x_next_dyn, state_, input_, Fx, M_Tv, FyF, FyR, dt);
    state_.validate();

    // Publish Everything
    setModelState(state_);
    publishTf(state_);

    // Overlay Noise on Velocities
    auto state_pub = state_.toRos(ros::Time::now());
    //state_pub.vx += noise::getGaussianNoise(0.0, param_.sensors.noise_vx_sigma);
    //state_pub.vy += noise::getGaussianNoise(0.0, param_.sensors.noise_vy_sigma);
    //state_pub.r += noise::getGaussianNoise(0.0, param_.sensors.noise_r_sigma);
    pub_ground_truth_.publish(state_pub);

    publishCarInfo(alphaF, alphaR, FyF, FyR, Fx);
}

void Vehicle::onRes(const fssim_common::ResStateConstPtr &msg) {
    res_state_ = *msg;
    if (res_state_.push_button) { car_info_.torque_ok = static_cast<unsigned char>(true); }
    if (res_state_.emergency) { car_info_.torque_ok = static_cast<unsigned char>(false); }
}

void Vehicle::printInfo() {
    front_axle_.printInfo();
    rear_axle_.printInfo();
}

State Vehicle::f(const State &x,
                 const Input &u,
                 const double Fx,
                 const double M_TV,
                 const AxleTires &FyF,
                 const AxleTires &FyR) {
    const double FyF_tot = FyF.left + FyF.right;
    const double FyR_tot = FyR.left + FyR.right;
    const double v_x     = std::max(1.0, x.v_x);

    const double m_lon = param_.inertia.m + param_.driveTrain.m_lon_add;

    State x_dot{};
    x_dot.x   = std::cos(x.yaw) * x.v_x - std::sin(x.yaw) * x.v_y;
    x_dot.y   = std::sin(x.yaw) * x.v_x + std::cos(x.yaw) * x.v_y;
    x_dot.yaw = x.r;
    x_dot.v_x = (x.r * x.v_y) + (Fx - std::sin(u.delta) * (FyF_tot)) / m_lon;
    x_dot.v_y = ((std::cos(u.delta) * FyF_tot) + FyR_tot) / param_.inertia.m - (x.r * v_x);
    x_dot.r   = ((std::cos(u.delta) * FyF_tot * param_.kinematic.l_F
                  + std::sin(u.delta) * (FyF.left - FyF.right) * 0.5 * param_.kinematic.b_F)
                 - ((FyR_tot) * param_.kinematic.l_R)
                 + M_TV) / param_.inertia.I_z;
    x_dot.a_x = 0;
    x_dot.a_y = 0;

    return x_dot;
}

std::ostream &operator<<(std::ostream &os, const State s) {
    os << s.getString();
}

State Vehicle::f_kin_correction(const State &x_in,
                                const State &x_state,
                                const Input &u,
                                const double Fx,
                                const double M_TV,
                                const AxleTires &FyF,
                                const AxleTires &FyR,
                                const double dt) {
    State        x       = x_in;

    const double v_x_dot = Fx / (param_.inertia.m + param_.driveTrain.m_lon_add);
    const double v       = std::hypot(state_.v_x, state_.v_y);
    const double v_blend = 0.5 * (v - 1.5);
    const double blend   = std::fmax(std::fmin(1.0, v_blend), 0.0);

    x.v_x = blend * x.v_x + (1.0 - blend) * (x_state.v_x + dt * v_x_dot);

    const double v_y = std::tan(u.delta) * x.v_x * param_.kinematic.l_R / param_.kinematic.l;
    const double r   = std::tan(u.delta) * x.v_x / param_.kinematic.l;

    x.v_y = blend * x.v_y + (1.0 - blend) * v_y;
    x.r   = blend * x.r + (1.0 - blend) * r;
    return x;
}

void Vehicle::publishTf(const State &x) {
    // Position
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x.x, x.y, 0.0));

    // Orientation
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, x.yaw);
    transform.setRotation(q);

    // Send TF
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/fssim_map", "/fssim/vehicle/base_link"));
}

double Vehicle::getFx(const State &x, const Input &u) {
    const double dc = x.v_x <= 0.0 && u.dc < 0.0 ? 0.0 : u.dc;
    resistance_force = aero_.getFdrag(x) + param_.driveTrain.cr0;
    const double Fx = dc * param_.driveTrain.cm1 - resistance_force;
    return Fx;
}

double Vehicle::getMTv(const State &x, const Input &u) const {
    const double shrinkage = param_.torqueVectoring.shrinkage;
    const double K_stab    = param_.torqueVectoring.K_stability;
    const double l         = param_.kinematic.l;

    const double delta = u.delta;
    const double v_x   = x.v_x;
  
    return 0.0;
}

void Vehicle::onCmd(const fssim_common::CmdConstPtr &msg) {
    time_last_cmd_ = ros::Time::now();
    auto msg_stamp = msg->stamp;
    if(std::abs((msg_stamp - time_last_cmd_).toSec()) < 5 ){
      time_last_cmd_ = msg_stamp; 
    }

    deltas_.emplace_back(time_last_cmd_,  msg->delta);
    dcs_.emplace_back(time_last_cmd_,  msg->dc);
    //input_.delta   = msg->delta;
    //input_.dc      = msg->dc;
    
}

void Vehicle::onInitialPose(const geometry_msgs::PoseWithCovarianceStamped &msg) {
    state_.x   = msg.pose.pose.position.x;
    state_.y   = msg.pose.pose.position.y;
    state_.yaw = tf::getYaw(msg.pose.pose.orientation);
    state_.v_x = state_.v_y = state_.r = state_.a_x = state_.a_y = 0.0;
}

double Vehicle::getNormalForce(const State &x) {
    down_force = param_.inertia.g * param_.inertia.m + aero_.getFdown(x);
    return down_force;
}

void Vehicle::setModelState(const State &x) {
    const math::Pose    pose(x.x, x.y, 0.0, 0, 0.0, x.yaw);
    const math::Vector3 vel(x.v_x, x.v_y, 0.0);
    const math::Vector3 angular(0.0, 0.0, x.r);
    model->SetWorldPose(pose);
    model->SetAngularVel(angular);
    model->SetLinearVel(vel);
}

double Vehicle::getGaussianNoise(double mean, double var) const {
    std::normal_distribution<double> distribution(mean, var);
    // construct a trivial random generator engine from a time-based seed:
    long                             seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine       generator(seed);
    return distribution(generator);
}

void Vehicle::publishCarInfo(const AxleTires &alphaF,
                             const AxleTires &alphaR,
                             const AxleTires &FyF,
                             const AxleTires &FyR,
                             const double Fx) const {
    // Publish Car Info
    fssim_common::CarInfo car_info;
    car_info.header.stamp = ros::Time::now();
    
    car_info.resistance_force = resistance_force;
    car_info.down_force = down_force;
    car_info.dc        = input_.dc;
    car_info.delta     = input_.delta;
    car_info.alpha_f   = alphaF.avg();
    car_info.alpha_f_l = alphaF.left;
    car_info.alpha_f_r = alphaF.right;

    car_info.Fy_f   = FyF.avg();
    car_info.Fy_f_l = FyF.left;
    car_info.Fy_f_r = FyF.right;

    car_info.alpha_r   = alphaR.avg();
    car_info.alpha_r_l = alphaR.left;
    car_info.alpha_r_r = alphaR.right;

    car_info.Fy_r   = FyR.avg();
    car_info.Fy_r_l = FyR.left;
    car_info.Fy_r_r = FyR.right;

    car_info.Fx = Fx;
    pub_car_info_.publish(car_info);
}

} // namespace fssim
} // namespace gazebo
