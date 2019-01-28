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

#ifndef FSSIM_GAZEBO_CONFIG_HPP_HPP
#define FSSIM_GAZEBO_CONFIG_HPP_HPP
#include "ros/common.h"
#include "yaml-cpp/yaml.h"

struct Param {
    struct Inertia {
        double m;
        double m_driver;
        double g;
        double I_z;
        void print() {
            ROS_DEBUG("Inertia: \n "
                      "\tm: %f\n"
                      "\tm_driver: %f\n"
                      "\tg: %f\n"
                      "\tI_z: %f", m, m_driver, g, I_z);
        }
    };

    struct Kinematic {
        double l;
        double b_F;
        double b_R;
        double w_front;
        double l_F;
        double l_R;
        double h_cg;
        void print() {
            ROS_DEBUG("Kinematic: \n "
                      "\tl: %f\n"
                      "\tb_F: %f\n"
                      "\tb_R: %f\n"
                      "\tw_front: %f\n"
                      "\tl_F: %f\n"
                      "\tl_R: %f\n"
                      "\th_cg: %f", l, b_F, b_R, w_front, l_F, l_R, h_cg);
        }
    };

    struct Tire {
        double tire_coefficient;
        double B;
        double C;
        double D;
        double E;
        void print() {
            ROS_DEBUG("Tire: \n "
                      "\tB: %f\n"
                      "\tC: %f\n"
                      "\tD: %f\n"
                      "\tE: %f", B, C, D, E);
        }
    };

    struct Aero {
        double c_down;
        double c_drag;
        void print() {
            ROS_DEBUG("Aero: \n "
                      "\tc_down: %f\n"
                      "\tc_drag: %f", c_down, c_drag);
        }
    };

    struct DriveTrain {
        int    nm_wheels;
        double inertia;
        double r_dyn;
        double m_lon_add;
        double cm1;
        double cr0;
        void print() {
            ROS_DEBUG("DriveTrain: \n "
                      "\tnm_wheels: %i\n"
                      "\tinertia: %f\n"
                      "\tr_dyn: %f\n"
                      "\tCm1: %f\n"
                      "\tCr0: %f", nm_wheels, inertia, r_dyn, cm1, cr0);
        }
    };

    struct TorqueVectoring {
        double K_FFW;
        double K_p;
        double shrinkage;
        double K_stability;
        void print() {
            ROS_DEBUG("TorqueVectoring: \n "
                      "\tK_FFW: %f\n"
                      "\tK_p: %f\n"
                      "\tshrinkage: %f\n"
                      "\tK_stability: %f", K_FFW, K_p, shrinkage, K_stability);
        }
    };

    struct Sensors {
        double noise_vx_sigma;
        double noise_vy_sigma;
        double noise_r_sigma;

        void print() {
            ROS_DEBUG("Sensors: \n "
                      "\tnoise_vx_sigma: %f\n"
                      "\tnoise_vy_sigma: %f\n"
                      "\tnoise_r_sigma: %f", noise_vx_sigma, noise_vy_sigma, noise_r_sigma);
        }
    };

    Inertia         inertia;
    Kinematic       kinematic;
    Tire            tire;
    Aero            aero;
    DriveTrain      driveTrain;
    TorqueVectoring torqueVectoring;
    Sensors         sensors;
};

namespace YAML {
template<>
struct convert<Param::Inertia> {
    static bool decode(const Node &node, Param::Inertia &cType) {
        cType.m        = node["m"].as<double>();
        cType.m_driver = node["m_driver"].as<double>();
        cType.g        = node["g"].as<double>();
        cType.I_z      = node["I_z"].as<double>();
        ROS_DEBUG("LOADED Inertia");
        cType.print();
        return true;
    }
};

template<>
struct convert<Param::Kinematic> {
    static bool decode(const Node &node, Param::Kinematic &cType) {
        cType.l       = node["l"].as<double>();
        cType.b_F     = node["b_F"].as<double>();
        cType.b_R     = node["b_R"].as<double>();
        cType.w_front = node["w_front"].as<double>();
        cType.h_cg    = node["h_cg"].as<double>();
        cType.l_F     = cType.l * (1 - cType.w_front);
        cType.l_R     = cType.l * cType.w_front;
        ROS_DEBUG("LOADED Kinematic");
        cType.print();
        return true;
    }
};

inline double mean(const double a, const double b) {
    return (a + b) / 2.0;
}

template<>
struct convert<Param::Tire> {
    static bool decode(const Node &node, Param::Tire &cType) {
        cType.tire_coefficient = node["tire_coefficient"].as<double>();
        cType.B = node["B"].as<double>() / cType.tire_coefficient;
        cType.C = node["C"].as<double>();
        cType.D = node["D"].as<double>() * cType.tire_coefficient;
        cType.E = node["E"].as<double>();
        ROS_DEBUG("LOADED Tire");
        cType.print();
        return true;
    }
};

template<>
struct convert<Param::Aero> {
    static bool decode(const Node &node, Param::Aero &cType) {
        cType.c_down =
            node["C_Down"]["a"].as<double>() * node["C_Down"]["b"].as<double>() * node["C_Down"]["c"].as<double>();
        cType.c_drag =
            node["C_drag"]["a"].as<double>() * node["C_drag"]["b"].as<double>() * node["C_drag"]["c"].as<double>();
        ROS_DEBUG("LOADED Aero");
        cType.print();
        return true;
    }
};

template<>
struct convert<Param::DriveTrain> {
    static bool decode(const Node &node, Param::DriveTrain &cType) {
        cType.inertia   = node["inertia"].as<double>();
        cType.r_dyn     = node["r_dyn"].as<double>();
        cType.nm_wheels = node["nm_wheels"].as<int>();
        cType.cr0       = node["Cr0"].as<double>();
        cType.cm1       = node["Cm1"].as<double>();
        cType.m_lon_add = cType.nm_wheels * cType.inertia / (cType.r_dyn * cType.r_dyn);
        ROS_DEBUG("LOADED DriveTrain");
        cType.print();
        return true;
    }
};

template<>
struct convert<Param::TorqueVectoring> {
    static bool decode(const Node &node, Param::TorqueVectoring &cType) {
        cType.K_FFW       = node["K_FFW"].as<double>();
        cType.K_p         = node["K_p"].as<double>();
        cType.shrinkage   = node["shrinkage"].as<double>();
        cType.K_stability = node["K_stability"].as<double>();
        ROS_DEBUG("LOADED TorqueVectoring");
        cType.print();
        return true;
    }
};

template<>
struct convert<Param::Sensors> {
    static bool decode(const Node &node, Param::Sensors &cType) {
        cType.noise_vx_sigma = node["noise_vx_sigma"].as<double>();
        cType.noise_vy_sigma = node["noise_vy_sigma"].as<double>();
        cType.noise_r_sigma  = node["noise_r_sigma"].as<double>();
        ROS_DEBUG("LOADED Sensors");
        cType.print();
        return true;
    }
};

}

inline void initParam(Param &param, std::string &yaml_file) {
    YAML::Node config = YAML::LoadFile(yaml_file);
    ROS_DEBUG("STARTING THIS YAML CraP: %s ********************************", yaml_file.c_str());
    const auto car    = config["car"];

    param.inertia         = car["inertia"].as<Param::Inertia>();
    param.kinematic       = car["kinematics"].as<Param::Kinematic>();
    param.tire            = car["tire"].as<Param::Tire>();
    param.aero            = car["aero"].as<Param::Aero>();
    param.driveTrain      = car["drivetrain"].as<Param::DriveTrain>();
    param.torqueVectoring = car["torque_vectoring"].as<Param::TorqueVectoring>();
}

inline void initParamSensors(Param &param, std::string &yaml_file) {
    YAML::Node config = YAML::LoadFile(yaml_file);
    ROS_DEBUG("STARTING SENSORS THIS YAML: %s ********************************", yaml_file.c_str());
    const auto car    = config["sensors"];

    param.sensors = car["velocity"].as<Param::Sensors>();
}

#endif //FSSIM_GAZEBO_CONFIG_HPP_HPP
