/*
 * AMZ Driverless Project
 *
 * Copyright (c) 2018 Authors:
 *   - Nikhil Bharadwaj Gosala <gosalan@ethz.ch>
 *
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */
#ifndef AUTONOMOUS_MSGS_PARSER_CONE_OUTPUT_H
#define AUTONOMOUS_MSGS_PARSER_CONE_OUTPUT_H

#define PCL_NO_RECOMPILE
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "cone_colour_probability.h"
#include "cone_position_uncertainty.h"

namespace gazebo
{
    struct Cone
    {
        PCL_ADD_POINT4D;
        float intensity;
        ConePositionUncertainty cone_position_uncertainty;
        ConeColourProbability cone_colour_probability;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // Ensures that all values are aligned in memory;
    }EIGEN_ALIGN16; // Enforce SSE padding for correct memory alignment

    inline Cone constructCone(const float x, const float y, const float z, const float intensity,
                                    const float covariance_xx, const float covariance_xy, const float covariance_yx, const float covariance_yy,
                                    const float probability_blue, const float probability_yellow, const float probability_orange, const float probability_other)
    {
        Cone cone;

        cone.x = x;
        cone.y = y;
        cone.z = z;

        cone.intensity = intensity;

        cone.cone_position_uncertainty.covariance_xx = covariance_xx;
        cone.cone_position_uncertainty.covariance_xy = covariance_xy;
        cone.cone_position_uncertainty.covariance_yx = covariance_yx;
        cone.cone_position_uncertainty.covariance_yy = covariance_yy;

        cone.cone_colour_probability.probability_blue = probability_blue;
        cone.cone_colour_probability.probability_yellow = probability_yellow;
        cone.cone_colour_probability.probability_orange = probability_orange;
        cone.cone_colour_probability.probability_other = probability_other;

        return cone;
    }
}

POINT_CLOUD_REGISTER_POINT_STRUCT(gazebo::Cone,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (float, cone_position_uncertainty.covariance_xx, covariance_xx)
                                          (float, cone_position_uncertainty.covariance_xy, covariance_xy)
                                          (float, cone_position_uncertainty.covariance_yx, covariance_yx)
                                          (float, cone_position_uncertainty.covariance_yy, covariance_yy)
                                          (float, cone_colour_probability.probability_blue, probability_blue)
                                          (float, cone_colour_probability.probability_yellow, probability_yellow)
                                          (float, cone_colour_probability.probability_orange, probability_orange)
                                          (float, cone_colour_probability.probability_other, probability_other)
)


#endif //AUTONOMOUS_MSGS_PARSER_CONE_OUTPUT_H
