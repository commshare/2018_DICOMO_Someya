// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef MULTIAGENT2_IDM_H
#define MULTIAGENT2_IDM_H

#include <assert.h>
#include <algorithm>

namespace MultiAgent2 {

inline
double CalcSquare(const double& x) { return (x*x); }


inline
double CalculateIntelligentDriversModelAcceleration(
    const double& maxVelocity,
    const double& maxAcceleration,
    const double& maxBrakingAcceleration,
    const double& minGap,
    const double& timeHeadway,
    const double& currentVelocity,
    const double& distanceToObstruction,
    const double& obstructionVelocity)
{
    using std::max;

    const double accelerationExponent = 4.0;

    assert(maxVelocity >= 0.0);
    assert(maxAcceleration >= 0.0);
    assert(maxBrakingAcceleration >= 0.0);
    assert(currentVelocity >= 0.0);
    assert(distanceToObstruction >= 0.0);
    assert(obstructionVelocity >= 0.0);

    const double deltaVelocity = currentVelocity - obstructionVelocity;

    // Note: No "s1" parameter term (non-linear gap) (i.e. "s1" is set to 0).

    const double subTerm =
        ((currentVelocity * timeHeadway) +
         ((currentVelocity * deltaVelocity) / (2.0 * sqrt(maxAcceleration * maxBrakingAcceleration))));

    const double sStar = minGap + max(0.0, subTerm);

    const double acceleration =
        maxAcceleration *
        ((1  - pow((currentVelocity / maxVelocity), accelerationExponent) -
         CalcSquare(sStar/distanceToObstruction)));

    if ((currentVelocity == 0.0) && (acceleration < 0)) {
        return (0.0);
    }//if//

    // Issue: extreme (negative) acceleration.

    return (acceleration);

}//CalculateIntelligentDriversModelAcceleration//




inline
double CalculateSpeedLimitChangeAcceleration(
    const double& maxBrakingAcceleration,
    const double& currentVelocity,
    const double& distanceToVelocityChange,
    const double& desiredVelocity,
    const double& timeGranularitySecs)
{
    assert(maxBrakingAcceleration >= 0.0);

    if (desiredVelocity >= currentVelocity) {
        return 0.0;
    }//if//

    const double deltaVelocity = (currentVelocity - desiredVelocity);
    const double deltaTime = (deltaVelocity / maxBrakingAcceleration);

    assert(deltaTime >= 0.0);

    const double averageVelocity = ((currentVelocity + desiredVelocity)/ 2.0);
    const double brakingDistance = deltaTime * averageVelocity;
    const double extraDistanceMeters = timeGranularitySecs * currentVelocity;

    if ((brakingDistance + extraDistanceMeters) < distanceToVelocityChange) {
        // Too far ahead.

        return 0.0;
    }
    else {
        // Adjust desired acceleration to be the exact deacceleration (should be very close to
        // desired).

        return ((brakingDistance/distanceToVelocityChange) * -maxBrakingAcceleration);
    }//if//

}//CalculateSpeedLimitChangeAcceleration//




struct IntelligentDriverModelParameters {
    //Future? double maxVelocity;
    double maxAccelerationMetersSec;
    //Future? double maxBrakingAccelerationMetersSec;
    double comfortableDeaccelerationMetersSec;
    double minGapMeters;
    double timeHeadwaySecs;
    double searchDistanceCutoffMeters;
    double forcedLaneChangeStopDistancePerLaneMeters;

};//IntelligentDriverModelParameters//


}//namespace//


#endif
