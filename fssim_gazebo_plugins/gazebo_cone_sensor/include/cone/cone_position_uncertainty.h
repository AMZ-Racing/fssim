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
#ifndef AUTONOMOUS_MSGS_PARSER_CONE_POSITION_UNCERTAINTY_H
#define AUTONOMOUS_MSGS_PARSER_CONE_POSITION_UNCERTAINTY_H

namespace gazebo
{
    struct ConePositionUncertainty
    {
        float covariance_xx;
        float covariance_xy;
        float covariance_yx;
        float covariance_yy;
    };
}

#endif //AUTONOMOUS_MSGS_PARSER_CONE_POSITION_UNCERTAINTY_H
