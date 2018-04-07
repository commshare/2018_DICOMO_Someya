// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef MULTIAGENT2_GIS_H
#define MULTIAGENT2_GIS_H

#include "multiagent2_common.h"
#include <limits.h>
#include <string>
#include <vector>

namespace MultiAgent2 {

using std::string;
using std::vector;

class GisRoadSegment {
public:
    static const unsigned int InvalidRoadId = UINT_MAX;

    GisRoadSegment(
        const unsigned int& initRoadId,
        const string& initRoadName,
        const double& initFirstLocationXMeters,
        const double& initFirstLocationYMeters,
        const double& initSecondLocationXMeters,
        const double& initSecondLocationYMeters,
        const double& initRoadWidthMeters,
        const unsigned int initNumberForwardLanes,
        const unsigned int initNumberBackwardLanes,
        const double& initLaneWidthMeters,
        const double& initSpeedLimitMetersSec)
        :
        roadId(initRoadId),
        roadName(initRoadName),
        firstLocationXMeters(initFirstLocationXMeters),
        firstLocationYMeters(initFirstLocationYMeters),
        secondLocationXMeters(initSecondLocationXMeters),
        secondLocationYMeters(initSecondLocationYMeters),
        roadWidthMeters(initRoadWidthMeters),
        numberForwardLanes(initNumberForwardLanes),
        numberBackwardLanes(initNumberBackwardLanes),
        laneWidthMeters(initLaneWidthMeters),
        speedLimitMetersSec(initSpeedLimitMetersSec)
    {}

    unsigned int GetRoadId() const { return (roadId); }
    const string& GetRoadName() const { return roadName; }
    double GetFirstLocationXMeters() const { return (firstLocationXMeters); }
    double GetFirstLocationYMeters() const { return (firstLocationYMeters); }
    double GetSecondLocationXMeters() const { return (secondLocationXMeters); }
    double GetSecondLocationYMeters() const { return (secondLocationYMeters); }

    double GetRoadWidthMeters() const { return (roadWidthMeters); }

    unsigned int GetNumberForwardLanes() const { return numberForwardLanes; }
    unsigned int GetNumberBackwardLanes() const { return numberBackwardLanes; }
    double GetLaneWidthMeters() const { return laneWidthMeters; }
    double GetSpeedLimitMetersSec() const { return speedLimitMetersSec; }
    double GetRoadMedianWidthMeters() const { return 0.0; }   //Future

private:
    unsigned int roadId;
    string roadName;
    double firstLocationXMeters;
    double firstLocationYMeters;
    double secondLocationXMeters;
    double secondLocationYMeters;
    double roadWidthMeters;
    double speedLimitMetersSec;

    unsigned int numberForwardLanes;
    unsigned int numberBackwardLanes;
    double laneWidthMeters;

};//GisRoadSegment//



void ReadRoads(
    const string& gisRoadsFilenamePrefix,
    double& minX,
    double& maxX,
    double& minY,
    double& maxY,
    vector<GisRoadSegment>& roadSegments);


struct GisIntersection {
    unsigned int intersectionId;
    string intersectionName;
    double centerXMeters;
    double centerYMeters;


    GisIntersection(
        const unsigned int initIntersectionId,
        const string& initIntersectionName,
        const double& initCenterXMeters,
        const double& initCenterYMeters)
    :
        intersectionId(initIntersectionId),
        intersectionName(initIntersectionName),
        centerXMeters(initCenterXMeters),
        centerYMeters(initCenterYMeters)
    {}
};


void ReadIntersections(
    const string& gisIntersectionsFilenamePrefix,
    double& minX,
    double& maxX,
    double& minY,
    double& maxY,
    vector<GisIntersection>& intersections);


class GisBuildingPolygon {
public:
    unsigned int buildingId;
    vector<TwoDVector> buildingVertices;
};

void ReadBuildingPolygons(
    const string& gisBuildingFilenamePrefix,
    double& minX,
    double& maxX,
    double& minY,
    double& maxY,
    vector<GisBuildingPolygon>& buildingPolygons);


class WaypointPortal {
public:
    typedef unsigned int WaypointIdType;
    static const WaypointIdType InvalidWaypointId = UINT_MAX;

    WaypointPortal(
        const double& x1,
        const double& y1,
        const double& x2,
        const double& y2,
        const double& initRadius)
        :
        firstVertex(x1, y1),
        centerVertex(((x1+x2)/2.0), ((y1+y2)/2.0)),
        secondVertex(x2, y2),
        width(CalcDistance(x1, y1, x2, y2)),
        radius(initRadius),
        radiusSquared(initRadius * initRadius)
    {}

    TwoDVector GetFirstPoint() const { return (firstVertex); }
    TwoDVector GetCenterPoint() const { return (centerVertex); }
    TwoDVector GetSecondPoint() const { return (secondVertex); }
    double GetWidth() const { return (width); }
    double GetRadius() const { return (radius); }
    bool IsInsideWaypoint(const TwoDVector& point) const;

private:
    TwoDVector firstVertex;
    TwoDVector secondVertex;
    TwoDVector centerVertex;
    double width;
    double radius;
    double radiusSquared;

};//WaypointPortal//


inline
bool WaypointPortal::IsInsideWaypoint(const TwoDVector& point) const
{
    const TwoDVector closestPoint =
        CalcClosestPointOnLineSegmentToAPoint(firstVertex, secondVertex, point);

    return (CalcDistanceSquared(closestPoint, point) <= radiusSquared);

}//IsInsideWaypoint//


void ReadWaypointPortals(
    const string& gisWaypointFilenamePrefix,
    const double& closeEnoughToWaypointDistanceMeters,
    vector<WaypointPortal>& waypointPortals);

}//namespace//

#endif



