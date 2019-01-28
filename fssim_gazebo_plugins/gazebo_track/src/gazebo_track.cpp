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

// Gazebo Includes
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

// Eigen Includes
#include <Eigen/Dense>

// ROS Includes
#include <ros/ros.h>

// ROS Msgs
#include "visualization_msgs/MarkerArray.h"
#include "fssim_common/Track.h"

namespace gazebo {

class TrackStreamer : public ModelPlugin {
 public:
    enum SIDE {
        LEFT,
        RIGHT,
        ORANGE,
        ORANGE_BIG,
        TK_DEVICE
    };

    struct Point2d {
        double x;
        double y;
    };

    Point2d toPoint2d(const boost::shared_ptr<gazebo::physics::Entity> &e) const {
        return {e->GetWorldPose().pos.x, e->GetWorldPose().pos.y};
    }

    geometry_msgs::Point toGeomPoint(const boost::shared_ptr<gazebo::physics::Entity> &e) const {
        geometry_msgs::Point p;
        p.x = e->GetWorldPose().pos.x;
        p.y = e->GetWorldPose().pos.y;
        p.z = 0.0;
        return p;
    }

    void fillPoints(std::vector<geometry_msgs::Point> &points,
                    const std::vector<boost::shared_ptr<gazebo::physics::Entity>> &ent) const {
        points.resize(ent.size());
        for (unsigned int i = 0; i < ent.size(); ++i) {
            points[i] = toGeomPoint(ent[i]);
        }
    }

    void fillHeader(visualization_msgs::Marker &marker, const std::string &ns, int id, const ros::Time &time) const {
        marker.header.frame_id = "fssim_map";
        marker.header.stamp    = time;
        marker.lifetime        = ros::Duration();
        marker.ns              = ns;
        marker.id              = id;
    }

    void computeTKLine(const std::vector<boost::shared_ptr<gazebo::physics::Entity>> &tk_device_start,
                       const std::string &ns, const ros::Time &time,
                       visualization_msgs::MarkerArray &array) const {
        if (tk_device_start.empty()) { return; }

        assert(tk_device_start.size() == 2);

        visualization_msgs::Marker marker;
        fillHeader(marker, ns, 0, time);
        marker.type               = visualization_msgs::Marker::LINE_LIST;
        marker.action             = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x            = 0.25;
        marker.color.a            = 1.0;
        marker.color.r            = 1.0;
        marker.color.g            = 0.0;
        marker.color.b            = 1.0;
        marker.points             = std::vector<geometry_msgs::Point>(2);
        for (unsigned int i = 0; i < tk_device_start.size(); i++) {
            const auto p       = toPoint2d(tk_device_start[i]);
            marker.points[i].x = p.x;
            marker.points[i].y = p.y;
        }
        array.markers.push_back(marker);
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override {
        // ROS Init
        int  argc  = 0;
        char *argv = nullptr;
        ros::init(argc, &argv, "track_streamer");

        pub_track =
            nh.advertise<visualization_msgs::MarkerArray>(_sdf->Get<std::string>("markers") + "/markers", 1, true);
        pub_custome_track = nh.advertise<fssim_common::Track>(_sdf->Get<std::string>("markers"), 1, true);

        const auto model                                          = _parent->GetWorld()->GetModel("track");

        std::vector<boost::shared_ptr<gazebo::physics::Entity>> left;
        std::vector<boost::shared_ptr<gazebo::physics::Entity>> right;
        std::vector<boost::shared_ptr<gazebo::physics::Entity>> orange;
        std::vector<boost::shared_ptr<gazebo::physics::Entity>> orange_big;
        std::vector<boost::shared_ptr<gazebo::physics::Entity>> tk_device_start;
        std::vector<boost::shared_ptr<gazebo::physics::Entity>> tk_device_end;
        for (unsigned int                                       i = 0; i < model->GetChildCount(); ++i) {
            const auto child  = model->GetChild(i);
            const auto entity = boost::dynamic_pointer_cast<gazebo::physics::Entity>(child);
            const auto name   = entity->GetName();
            if (name == "cone_left::link") {
                left.push_back(entity);
            } else if (name == "cone_right::link") {
                right.push_back(entity);
            } else if (name == "cone_orange::link") {
                orange.push_back(entity);
            } else if (name == "cone_orange_big::link") {
                orange_big.push_back(entity);
            } else if (name.find("tk_device") != std::string::npos) {
                if (name.find("0") != std::string::npos or name.find("1") != std::string::npos) {
                    tk_device_start.push_back(entity);
                } else if (name.find("2") != std::string::npos or name.find("3") != std::string::npos) {
                    tk_device_end.push_back(entity);
                }
            }
        }

        visualization_msgs::MarkerArray track_markers;
        const auto                      stamp                     = ros::Time();
        {
            track_markers.markers.clear();
            for (size_t i = 0; i < left.size(); ++i) {
                addMarker(stamp, toPoint2d(left[i]), SIDE::LEFT, "track", static_cast<int32_t>(i), track_markers);
            }
            for (size_t i = 0; i < right.size(); ++i) {
                addMarker(stamp, toPoint2d(right[i]), SIDE::RIGHT, "track", static_cast<int32_t>(left.size() + i),
                          track_markers);
            }
            for (size_t i = 0; i < orange.size(); ++i) {
                addMarker(stamp, toPoint2d(orange[i]), SIDE::ORANGE, "track",
                          static_cast<int32_t>(left.size() + right.size() + i),
                          track_markers);
            }
            for (size_t i = 0; i < orange_big.size(); ++i) {
                addMarker(stamp, toPoint2d(orange_big[i]), SIDE::ORANGE_BIG, "track",
                          static_cast<int32_t>(left.size() + right.size() + orange.size() + i),
                          track_markers);
            }
            for (size_t i = 0; i < tk_device_start.size(); ++i) {
                addMarker(stamp, toPoint2d(tk_device_start[i]), SIDE::TK_DEVICE, "track",
                          static_cast<int32_t>(left.size() + right.size() + orange.size() + orange_big.size() + i),
                          track_markers);
            }
            for (size_t i = 0; i < tk_device_end.size(); ++i) {
                addMarker(stamp, toPoint2d(tk_device_end[i]), SIDE::TK_DEVICE, "track",
                          static_cast<int32_t>(left.size() + right.size() + orange.size() + orange_big.size()
                                               + tk_device_start.size() + i),
                          track_markers);
            }
        }

        computeTKLine(tk_device_start, "tk_start", stamp, track_markers);
        computeTKLine(tk_device_end, "tk_end", stamp, track_markers);

        pub_track.publish(track_markers);

        fssim_common::Track track;
        fillPoints(track.cones_left, left);
        fillPoints(track.cones_right, right);
        fillPoints(track.cones_orange, orange);
        fillPoints(track.cones_orange_big, orange_big);
        fillPoints(track.tk_device_start, tk_device_start);
        fillPoints(track.tk_device_end, tk_device_end);
        pub_custome_track.publish(track);

    }

    void addMarker(const ros::Time &time,
                   const Point2d &p,
                   const SIDE side,
                   const std::string &ns,
                   const int32_t id,
                   visualization_msgs::MarkerArray &array) {
        visualization_msgs::Marker marker;
        marker.header.frame_id    = "fssim_map";
        marker.header.stamp       = time;
        marker.lifetime           = ros::Duration();
        marker.ns                 = ns;
        marker.id                 = id;
        marker.type               = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action             = visualization_msgs::Marker::ADD;
        marker.pose.position.x    = p.x;
        marker.pose.position.y    = p.y;
        marker.pose.position.z    = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x            = 1.0;
        marker.scale.y            = 1.0;
        marker.scale.z            = 1.0;
        marker.color.a            = 1.0; // Don't forget to set the alpha!
        if (side == SIDE::LEFT) {
            marker.color.r       = 0.0;
            marker.color.g       = 0.0;
            marker.color.b       = 1.0;
            marker.mesh_resource = "package://fssim_gazebo/meshes/cone/cone_blue.dae";

        } else if (side == SIDE::RIGHT) {
            marker.color.r       = 1.0;
            marker.color.g       = 1.0;
            marker.color.b       = 0.0;
            marker.mesh_resource = "package://fssim_gazebo/meshes/cone/cone_yellow.dae";

        } else if (side == SIDE::ORANGE) {
            marker.color.r       = 1.0;
            marker.color.g       = static_cast<float>(165.0 / 255.0);
            marker.color.b       = 0.0;
            marker.mesh_resource = "package://fssim_gazebo/meshes/cone/cone_orange.dae";
        } else if (side == SIDE::ORANGE_BIG) {
            marker.color.r       = 1.0;
            marker.color.g       = static_cast<float>(165.0 / 255.0);
            marker.color.b       = 0.0;
            marker.mesh_resource = "package://fssim_gazebo/meshes/cone/cone_orange_big.dae";
        } else if (side == SIDE::TK_DEVICE) {
            marker.color.r       = 1.0;
            marker.color.g       = 0.0;
            marker.color.b       = 0.0;
            marker.mesh_resource = "package://fssim_gazebo/meshes/cone/cone_orange_big.dae";
        }
        array.markers.push_back(marker);
    }

    ros::NodeHandle nh;
    ros::Publisher  pub_track;
    ros::Publisher  pub_custome_track;

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(TrackStreamer)

} // namespace gazebo