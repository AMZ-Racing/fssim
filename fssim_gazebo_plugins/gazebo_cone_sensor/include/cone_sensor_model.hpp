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

#ifndef CONE_SENSOR_MODEL_HPP
#define CONE_SENSOR_MODEL_HPP

// Eigen Includes
#include <Eigen/Dense>

// ROS
#include "ros/ros.h"

// ROS Msgs
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "visualization_msgs/MarkerArray.h"
#include "cone/cone.h"

// Gazebo
#include <gazebo/physics/physics.hh>

// ROS Tf
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace gazebo {

class ConeSensorModel {
 public:

    enum SIDE {
        LEFT   = 0,
        RIGHT  = 1,
        ORANGE = 2
    };

    struct Config {
        double      observation_radius;
        std::string topic_name;
        std::string transfer_to_frame;
        bool overwrite_transfer_to_frame;

        double observation_likelihood_left;
        double observation_likelihood_right;
        double observation_likelihood_orange;
        double distance_dependent_observation;

        double likelihood_yellow;
        double likelihood_blue;
        double likelihood_orange;

        double distance_dependent_misclass;

        double gaussian_noise_xy_mu;
        double gaussian_noise_xy_sigma;

        double gaussian_noise_mu_angular;
        double gaussian_noise_sigma_angular;

        double gaussian_noise_mu_radial;
        double gaussian_noise_sigma_radial;

        double color_observation_radius;

        double rate;

        double cut_cones_below_x;

        double delay;
        double delay_noise_sigma;

        void load(const sdf::ElementPtr &_sdf) {
            topic_name = _sdf->Get<std::string>("topic_name");

            observation_radius = _sdf->Get<double>("observation_radius");

            observation_likelihood_left   = _sdf->Get<double>("observation_likelihood_left");
            observation_likelihood_right  = _sdf->Get<double>("observation_likelihood_right");
            observation_likelihood_orange = _sdf->Get<double>("observation_likelihood_orange");

            gaussian_noise_xy_mu    = _sdf->Get<double>("gaussian_noise_xy_mu");
            gaussian_noise_xy_sigma = _sdf->Get<double>("gaussian_noise_sigma");

            likelihood_yellow = _sdf->Get<double>("likelihood_yellow");
            likelihood_blue   = _sdf->Get<double>("likelihood_blue");
            likelihood_orange = _sdf->Get<double>("likelihood_orange");
        }

        void print() const {
            ROS_INFO("ConeSensorModel: ");
            ROS_INFO("\ttopic_name: %s", topic_name.c_str());
            ROS_INFO("\ttransfer_to_frame: %s", transfer_to_frame.c_str());

            ROS_INFO("\tobservation_radius: %f", observation_radius);

            ROS_INFO("\tobservation_likelihood_left: %f", observation_likelihood_left);
            ROS_INFO("\tobservation_likelihood_right: %f", observation_likelihood_right);
            ROS_INFO("\tobservation_likelihood_orange: %f", observation_likelihood_orange);

            ROS_INFO("\tgaussian_noise_xy_mu: %f", gaussian_noise_xy_mu);
            ROS_INFO("\tgaussian_noise_xy_sigma: %f", gaussian_noise_xy_sigma);

            ROS_INFO("\tlikelihood_yellow: %f", likelihood_yellow);
            ROS_INFO("\tlikelihood_blue: %f", likelihood_blue);
            ROS_INFO("\tlikelihood_orange: %f", likelihood_orange);

            ROS_INFO("\trate: %f", rate);
            ROS_INFO("\tdelay: %f", delay);
            ROS_INFO("\tdelay_noise_sigma: %f", delay_noise_sigma);

            ROS_INFO("\tcolor_observation_radius: %f", color_observation_radius);
        }

    };

    typedef pcl::PointCloud<Cone> PointCloud;

    ConeSensorModel(ros::NodeHandle &_node);

    bool load(const physics::ModelPtr &model, const sdf::ElementPtr &_sdf);

    void update();

    double getRate() const { return config.rate; }

 private:

    void updateTrack();

    void findObservedCones(const Eigen::Vector3d &p,
                           const std::vector<boost::shared_ptr<gazebo::physics::Entity>> &cones,
                           double likelihood,
                           double color_likelihood,
                           PointCloud &obs,
                           SIDE side) const;

    ConeColourProbability computeColorProbability(const Eigen::Vector3d &cone,
                                                  double d,
                                                  double color_likelihood,
                                                  double likelihood_factor,
                                                  SIDE side) const;

    void print(const std::vector<Eigen::Vector2d> &vect) const;

    Eigen::Vector3d transform(const std::string &from, const std::string &to, const Eigen::Vector3d &p) const;

    bool getState(const ros::Time &now, Eigen::Vector3d &p) const;
    bool observed(double likelihood) const;

    bool checkInit();

    void addRadialNoise(PointCloud &cloud);

    void addNoise(Cone &c);

 private:
    std::vector<boost::shared_ptr<gazebo::physics::Entity>> left_;
    std::vector<boost::shared_ptr<gazebo::physics::Entity>> right_;
    std::vector<boost::shared_ptr<gazebo::physics::Entity>> orange_;

    std::string vehicle_frame_;

    Config config;

    ros::NodeHandle &nh_;

    ros::Publisher pub_cones_;
    ros::Publisher pub_markers_;

    PointCloud point_cloud_;

    std::map<std::string, int> id_counter;

    physics::ModelPtr model_;
    physics::ModelPtr parent_model_;

    tf::TransformListener listener_;

    bool loaded_sacesfully_;
};
} // namespace gazebo

#endif //CONE_SENSOR_MODEL_HPP
