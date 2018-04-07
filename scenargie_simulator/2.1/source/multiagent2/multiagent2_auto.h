// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef MULTIAGENT2_AUTO_H
#define MULTIAGENT2_AUTO_H

#include <scensim_netsim.h>
#include <deque>
#include "multiagent2_gis.h"
#include "multiagent2_idm.h"
#include "multiagent2_routing.h"

namespace MultiAgent2 {

using ScenSim::ParameterDatabaseReader;
using ScenSim::SimulationEngine;
using ScenSim::SimulationEngineInterface;
using ScenSim::SimulationEvent;
using ScenSim::TimeType;
using ScenSim::ZERO_TIME;
using ScenSim::INFINITE_TIME;
using ScenSim::RandomNumberGeneratorSeedType;
using ScenSim::ObjectMobilityPosition;
using ScenSim::DeleteTrailingSpaces;
using ScenSim::IsAConfigFileCommentLine;
using ScenSim::ConvertDoubleSecsToTime;
using ScenSim::ConvertTimeToDoubleSecs;
using ScenSim::ConvertStringToLowerCase;
using ScenSim::DegreesPerRadian;

using std::deque;
using std::cerr;
using std::endl;
using std::istringstream;
using std::make_shared;
using std::shared_ptr;

//-----------------------------------------------------------------------------

const double StraightRoadMaxDeltaAzimuthRad = 0.1745329;
const double MinGapForTimestepErrorsMeters = 0.1;

typedef unsigned int RoadSegmentIdType;
const RoadSegmentIdType InvalidRoadSegmentId = UINT_MAX;

typedef GraphVertexIdType IntersectionIdType;
const IntersectionIdType InvalidIntersectionId = UINT_MAX;


inline
bool ReadVehicleModelIsEnabled(const ParameterDatabaseReader& parameterDatabaseReader)
{
    return (
        (parameterDatabaseReader.ParameterExists(parameterNamePrefix + "vehicle-model-is-on")) &&
        (parameterDatabaseReader.ReadBool(parameterNamePrefix + "vehicle-model-is-on")));
}


enum class IntersectionBehaviorType {
    KeepGoing,
    SignalControlled,
    StopAndYield,
    //Yield,
};


enum class RoadTurnDirectionType {
    TurnCrossingCenterLine,     //Right-hand Traffic: Left turn, Left-hand Traffic: Right turn.
    Straight,
    TurnNotCrossingCenterLine,  //Right-hand Traffic: Right turn, Left-hand Traffic: Left turn.
    Invalid,
};



const RoadTurnDirectionType CalcTurnDirection(
    const bool isARightHandTrafficRoad,
    const double& maxAbsAzimuthForStraightRad,
    const double& turnAzimuthClockwiseRad)
{
    if (std::abs(turnAzimuthClockwiseRad) <= maxAbsAzimuthForStraightRad) {
        return (RoadTurnDirectionType::Straight);
    }//if//

    if (turnAzimuthClockwiseRad >= 0.0) {
        if (isARightHandTrafficRoad) {
            return (RoadTurnDirectionType::TurnNotCrossingCenterLine);
        }
        else {
            return (RoadTurnDirectionType::TurnCrossingCenterLine);
        }//if//
    }
    else {
        if (isARightHandTrafficRoad) {
            return (RoadTurnDirectionType::TurnCrossingCenterLine);
        }
        else {
            return (RoadTurnDirectionType::TurnNotCrossingCenterLine);
        }//if//
    }//if//

}//CalcTurnDirection//



//--------------------------------------------------------------------------------------------------

enum class TrafficLightColorType {
    Green, Yellow, Red,
};


class TrafficLight {
public:
    virtual TimeType GetGreenLightDuration() const = 0;
    virtual TimeType GetYellowLightDuration() const = 0;
    virtual TimeType GetRedLightDuration() const = 0;
    virtual TrafficLightColorType GetCurrentColor(const TimeType& currentTime) const = 0;
    virtual TimeType GetNextChangeTime(const TimeType& currentTime) const = 0;

};//TrafficLight//


class SimpleTrafficLight: public TrafficLight {
public:

    SimpleTrafficLight(
        const TimeType& initStartTime,
        const TimeType& initGreenLightDuration,
        const TimeType& initYellowLightDuration,
        const TimeType& initRedLightDuration)
    :
        startTime(initStartTime),
        greenLightDuration(initGreenLightDuration),
        yellowLightDuration(initYellowLightDuration),
        redLightDuration(initRedLightDuration),
        cycleDuration(initGreenLightDuration + initYellowLightDuration + initRedLightDuration)
    {
        assert(cycleDuration != ZERO_TIME);
    }

    virtual TimeType GetGreenLightDuration() const override { return (greenLightDuration); }
    virtual TimeType GetYellowLightDuration() const override { return (yellowLightDuration); }
    virtual TimeType GetRedLightDuration() const override { return (redLightDuration); }
    virtual TrafficLightColorType GetCurrentColor(const TimeType& currentTime) const override;
    virtual TimeType GetNextChangeTime(const TimeType& currentTime) const override;
private:

    TimeType startTime;
    TimeType greenLightDuration;
    TimeType yellowLightDuration;
    TimeType redLightDuration;
    TimeType cycleDuration;

};//SimpleTrafficLight//



inline
TrafficLightColorType SimpleTrafficLight::GetCurrentColor(const TimeType& currentTime) const
{
    if (currentTime < startTime) {
        return (TrafficLightColorType::Red);
    }//if//

    const TimeType cycleTime = (currentTime - startTime) % cycleDuration;
    if (cycleTime < greenLightDuration) {
        return (TrafficLightColorType::Green);
    }
    else if (cycleTime < (greenLightDuration + yellowLightDuration)) {
        return (TrafficLightColorType::Yellow);
    }
    else {
        return (TrafficLightColorType::Red);
    }//if//

}//GetCurrentColor//



inline
TimeType SimpleTrafficLight::GetNextChangeTime(const TimeType& currentTime) const
{
    if (currentTime < startTime) {
        return startTime;
    }//if//

    const TimeType cycleTime = (currentTime - startTime) % cycleDuration;
    if (cycleTime < greenLightDuration) {
        return (currentTime + (greenLightDuration - cycleTime));
    }
    else if (cycleTime < (greenLightDuration + yellowLightDuration)) {

        return (currentTime + (greenLightDuration + yellowLightDuration - cycleTime));
    }
    else {
        return (currentTime + (cycleDuration - cycleTime));
    }//if//

}//GetNextChangeTime//



//--------------------------------------------------------------------------------------------------


// Sort by position and then lane (following behavior vs lane change).

struct PositionInRoadType {
    double positionInRoadMeters;
    unsigned int laneNumber;

    PositionInRoadType() {}
    PositionInRoadType(
        const double& initPositionInRoad,
        const unsigned int initLaneNumber)
        :
        positionInRoadMeters(initPositionInRoad),
        laneNumber(initLaneNumber)
    {}


    bool operator<(const PositionInRoadType& right) const
    {
        return (((*this).positionInRoadMeters < right.positionInRoadMeters) ||
                (((*this).positionInRoadMeters == right.positionInRoadMeters) &&
                 ((*this).laneNumber < right.laneNumber)));
    }
};


struct RoadSegmentIdAndWhichEndType {
    RoadSegmentIdType roadId;
    bool isAtFrontOfRoad;

    RoadSegmentIdAndWhichEndType() { }
    RoadSegmentIdAndWhichEndType(
        const RoadSegmentIdType& initRoadId,
        const bool initIsAtFrontOfRoad)
    :
        roadId(initRoadId),
        isAtFrontOfRoad(initIsAtFrontOfRoad)
    {}
};


//--------------------------------------------------------------------------------------------------



class RoadSegment {
public:
    static const unsigned int MaxNumberLanes = 16;
    static const unsigned int InvalidLaneNum = UINT_MAX;

    struct ConnectionLaneInfoType {
        ConnectionLaneInfoType() :
            outgoingLaneNum(InvalidLaneNum), nextRoadLaneNum(InvalidLaneNum),
            intersectionLaneRoadId(InvalidRoadSegmentId) {}

        unsigned int outgoingLaneNum;
        unsigned int nextRoadLaneNum;
        RoadSegmentIdType intersectionLaneRoadId;
    };

    struct ConnectionInfoType {
        IntersectionIdType intersectionId;
        bool isAContinuationIntersection;
        bool isAtFrontOfRoad;
        RoadSegmentIdType nextRoadId;
        bool nextRoadConnectedAtFront;
        RoadTurnDirectionType turnDirection;

        vector<ConnectionLaneInfoType> intersectionLanes;

        double speedLimitMetersSec;

        IntersectionBehaviorType behavior;
        shared_ptr<TrafficLight> trafficLightPtr;

        ConnectionInfoType()
            :
            nextRoadId(InvalidRoadSegmentId),
            intersectionId(InvalidIntersectionId),
            isAContinuationIntersection(false),
            turnDirection(RoadTurnDirectionType::Invalid)
        {}

        bool IsOneOfTheLanes(const unsigned int laneNumber) const;
        const ConnectionLaneInfoType& GetLaneInfo(const unsigned int outgoingLaneNumber) const;

        bool IsOneOfTheDestinationLanes(const unsigned int laneNumber) const;
        const ConnectionLaneInfoType& GetLaneInfoForDestinationLane(
            const unsigned int destinationLaneNumber) const;

    };//ConnectionInfoType//

    //------------------------------------------

    RoadSegment(
        const RoadSegmentIdType& roadId,
        const bool isARightHandTrafficRoad,
        const TwoDVector& firstCenterLocation,
        const TwoDVector& nextCenterLocation,
        const unsigned int numberForwardLanes,
        const unsigned int numberBackwardLanes,
        const double& laneWidthMeters,
        const double& speedLimitMetersSec,
        const bool isAnIntersectionLanePseudoRoad);

    RoadSegment() { }

    RoadSegmentIdType GetRoadSegmentId() const { return (roadSegmentId); }

    void SetFrontIntersectionId(const IntersectionIdType& newIntersectionId) {
        frontIntersectionId = newIntersectionId;
    }

    void SetEndIntersectionId(const IntersectionIdType& newIntersectionId) {
        endIntersectionId = newIntersectionId;
    }

    IntersectionIdType GetFrontIntersectionId() const { return (frontIntersectionId); }
    IntersectionIdType GetEndIntersectionId() const { return (endIntersectionId); }

    IntersectionIdType GetIntersectionId(const bool isAtFront) const
    {
        if (isAtFront) {
            return (GetFrontIntersectionId());
        }
        else {
            return (GetEndIntersectionId());
        }
    }

    bool GetIsARightHandTrafficRoadNetwork() const { return (isARightHandTrafficRoad); }

    bool GetIsAnIntersectionLanePseudoRoad() const { return (isAnIntersectionLanePseudoRoad); }

    void AddSubsegment(
        const TwoDVector& roadCenterPosition,
        const double& roadMedianWidth);

    void AddConnection(const ConnectionInfoType& connection);

    double GetLengthMeters() const { return (lengthMeters); }

    double GetSpeedLimitMetersSec() const { return (speedLimitMetersSec); }

    const TwoDVector& GetFrontLocation() const;
    const TwoDVector GetFrontIntersectionLaneLocation(const unsigned int laneNumber) const;
    void GetFrontSubsegment(TwoDVector& otherLocation, TwoDVector& frontLocation) const;

    const TwoDVector& GetEndLocation() const;
    const TwoDVector GetEndIntersectionLaneLocation(const unsigned int laneNumber) const;
    void GetEndSubsegment(TwoDVector& otherLocation, TwoDVector& endLocation) const;

    double GetLaneWidthMeters() const { return (laneWidthMeters); }

    // "Forward" is arbitrarily from the "front" to "end" direction.
    // "Backward" is "end" to "front" direction.

    bool IsAForwardLane(const unsigned int laneIndex) const {
        assert(laneIndex < GetNumberLanes());
        return (laneIndex < numberForwardLanes);
    }
    unsigned int GetNumberForwardLanes() const { return (numberForwardLanes); }
    unsigned int GetNumberBackwardLanes() const { return (numberBackwardLanes); }
    unsigned int GetNumberLanes() const { return (numberForwardLanes + numberBackwardLanes); }

    bool HasForwardLanes() const { return (numberForwardLanes > 0); }
    bool HasBackwardLanes() const { return (numberBackwardLanes > 0); }

    bool HasIncomingLanes(const bool isAtFront) const;

    bool ConnectionExists(
        const RoadSegmentIdType& nextRoadId,
        const bool connectionIsAtFront) const;

    const ConnectionInfoType& GetConnectionInfo(const RoadSegmentIdType& nextRoadId) const;

    const ConnectionInfoType& GetConnectionInfo(
        const RoadSegmentIdType& nextRoadId,
        const bool connectionIsAtFront) const;

    double CalcDistanceLeftMeters(const PositionInRoadType& roadPosition) const;

    TwoDVector GetXyPosition(const PositionInRoadType& roadPosition) const;

    void GetXyPositionAndAzimuth(
        const PositionInRoadType& roadPosition,
        TwoDVector& xyPosition,
        double& azimuthClockwiseRad) const;

    // TBD: more accurate road dimensions (median).

    double GetWidthMeters() const { return (laneWidthMeters * GetNumberLanes()); }

    // Used for making intersections from simple road graph.

    void TrimRoadAtFront(const double& trimLengthMeters);
    void TrimRoadAtEnd(const double& trimLengthMeters);


private:
    RoadSegmentIdType roadSegmentId;
    bool isARightHandTrafficRoad;
    bool isAnIntersectionLanePseudoRoad;
    double laneWidthMeters;
    unsigned int numberForwardLanes;
    unsigned int numberBackwardLanes;


    struct RoadVertexInfoType {
        TwoDVector roadCenterLocation;
        double roadMedianWidth;

        RoadVertexInfoType() { }

        RoadVertexInfoType(
            const TwoDVector& initRoadCenterLocation,
            const double& initRoadMedianWidth)
        :
            roadCenterLocation(initRoadCenterLocation),
            roadMedianWidth(initRoadMedianWidth)
        {}
    };

    vector<RoadVertexInfoType> roadVertices;
    double lengthMeters;
    double speedLimitMetersSec;

    IntersectionIdType frontIntersectionId;
    IntersectionIdType endIntersectionId;

    vector<ConnectionInfoType> frontIntersectionConnections;
    vector<ConnectionInfoType> endIntersectionConnections;

    //----------------------------------------------------------------------------------------------

    double CalcLaneOffsetMeters(const unsigned int laneIndex) const;

};//RoadSegment//



inline
RoadSegment::RoadSegment(
    const RoadSegmentIdType& initRoadId,
    const bool initIsARightHandTrafficRoad,
    const TwoDVector& firstCenterLocation,
    const TwoDVector& nextCenterLocation,
    const unsigned int initNumberForwardLanes,
    const unsigned int initNumberBackwardLanes,
    const double& initLaneWidthMeters,
    const double& initSpeedLimitMetersSec,
    const bool initIsAnIntersectionLanePseudoRoad)
    :
    roadSegmentId(initRoadId),
    isARightHandTrafficRoad(initIsARightHandTrafficRoad),
    lengthMeters(0.0),
    numberForwardLanes(initNumberForwardLanes),
    numberBackwardLanes(initNumberBackwardLanes),
    laneWidthMeters(initLaneWidthMeters),
    isAnIntersectionLanePseudoRoad(initIsAnIntersectionLanePseudoRoad),
    speedLimitMetersSec(initSpeedLimitMetersSec),
    frontIntersectionId(InvalidIntersectionId),
    endIntersectionId(InvalidIntersectionId)
{
    roadVertices.push_back(
        RoadVertexInfoType(
            firstCenterLocation,
            0.0));

    roadVertices.push_back(
        RoadVertexInfoType(
            nextCenterLocation,
            0.0));

    lengthMeters = (nextCenterLocation - firstCenterLocation).Length();

}//RoadSegment//


inline
bool RoadSegment::HasIncomingLanes(const bool isAtFront) const
{
    if (isAtFront) {
        return (HasForwardLanes());
    }
    else {
        return (HasBackwardLanes());
    }//if//
}

inline
bool RoadSegment::ConnectionInfoType::IsOneOfTheLanes(const unsigned int laneNumber) const
{
    for(unsigned int i = 0; (i <intersectionLanes.size()); i++) {
        if (intersectionLanes[i].outgoingLaneNum == laneNumber) {
            return true;
        }//if//
    }//for//

    return false;
}


inline
bool RoadSegment::ConnectionInfoType::IsOneOfTheDestinationLanes(const unsigned int laneNumber) const
{
    for(unsigned int i = 0; (i <intersectionLanes.size()); i++) {
        if (intersectionLanes[i].nextRoadLaneNum == laneNumber) {
            return true;
        }//if//
    }//for//

    return false;
}


inline
const RoadSegment::ConnectionLaneInfoType& RoadSegment::ConnectionInfoType::GetLaneInfo(
    const unsigned int outgoingLaneNumber) const
{
    for(unsigned int i = 0; (i <intersectionLanes.size()); i++) {
        if (intersectionLanes[i].outgoingLaneNum == outgoingLaneNumber) {
            return (intersectionLanes[i]);
        }//if//
    }//for//

    assert(false && "Lane not found!"); abort(); return (intersectionLanes.front());
}



inline
const RoadSegment::ConnectionLaneInfoType& RoadSegment::ConnectionInfoType::GetLaneInfoForDestinationLane(
     const unsigned int destinationLaneNumber) const
{
    for(unsigned int i = 0; (i <intersectionLanes.size()); i++) {
        if (intersectionLanes[i].nextRoadLaneNum == destinationLaneNumber) {
            return (intersectionLanes[i]);
        }//if//
    }//for//

    assert(false && "Lane not found!"); abort(); return (intersectionLanes.front());
}



inline
void RoadSegment::AddSubsegment(
    const TwoDVector& roadCenterLocation,
    const double& roadMedianWidth)
{
    const TwoDVector& backLocation = roadVertices.back().roadCenterLocation;

    roadVertices.push_back(
        RoadVertexInfoType(
            roadCenterLocation,
            roadMedianWidth));

    lengthMeters += (backLocation - roadCenterLocation).Length();

}//AddSubsegment//


inline
void RoadSegment::AddConnection(const ConnectionInfoType& connection)
{
    if (connection.isAtFrontOfRoad) {
        frontIntersectionConnections.push_back(connection);
    }
    else {
        endIntersectionConnections.push_back(connection);
    }//if//

}//AddConnection//


inline
bool RoadSegment::ConnectionExists(
    const RoadSegmentIdType& nextRoadId,
    const bool connectionIsAtFront) const
{
    if (connectionIsAtFront) {
        for (unsigned int i = 0; (i < frontIntersectionConnections.size()); i++) {
            const ConnectionInfoType& connection = frontIntersectionConnections[i];
            if (connection.nextRoadId == nextRoadId) {
                return (true);
            }//if//
        }//for//
    }
    else {
        for (unsigned int i = 0; (i < endIntersectionConnections.size()); i++) {
            const ConnectionInfoType& connection = endIntersectionConnections[i];
            if (connection.nextRoadId == nextRoadId) {
                return (true);
            }//if//
        }//for//
    }//if//

    return false;

}//ConnectionExists//


inline
const RoadSegment::ConnectionInfoType& RoadSegment::GetConnectionInfo(
    const RoadSegmentIdType& nextRoadId) const
{
    for (unsigned int i = 0; (i < frontIntersectionConnections.size()); i++) {
        const ConnectionInfoType& connection = frontIntersectionConnections[i];
        if (connection.nextRoadId == nextRoadId) {
            return (connection);
        }//if//
    }//for//

    for (unsigned int i = 0; (i < endIntersectionConnections.size()); i++) {
        const ConnectionInfoType& connection = endIntersectionConnections[i];
        if (connection.nextRoadId == nextRoadId) {
            return (connection);
        }//if//
    }//for//

    cerr << "Error in Multiagent2 vehicle model: Connection was not found (input file):" << endl;
    cerr << "  Road Index " << GetRoadSegmentId() << " to " << nextRoadId << endl;
    exit(1);

    return (*(new ConnectionInfoType()));

}//GetConnectionInfo//


inline
const RoadSegment::ConnectionInfoType& RoadSegment::GetConnectionInfo(
    const RoadSegmentIdType& nextRoadId,
    const bool connectionIsAtFront) const
{
    if (connectionIsAtFront) {
        for (unsigned int i = 0; (i < frontIntersectionConnections.size()); i++) {
            const ConnectionInfoType& connection = frontIntersectionConnections[i];
            if (connection.nextRoadId == nextRoadId) {
                return (connection);
            }//if//
        }//for//
    }
    else {
        for (unsigned int i = 0; (i < endIntersectionConnections.size()); i++) {
            const ConnectionInfoType& connection = endIntersectionConnections[i];
            if (connection.nextRoadId == nextRoadId) {
                return (connection);
            }//if//
        }//for//
    }//if//

    cerr << "Error in Multiagent2 vehicle model: Connection was not found (input file):" << endl;
    cerr << "  Road Index " << GetRoadSegmentId() << " to " << nextRoadId << endl;
    exit(1);

    return (*(new ConnectionInfoType()));

}//GetConnectionInfo//



inline
const TwoDVector& RoadSegment::GetFrontLocation() const
{
    return (roadVertices.front().roadCenterLocation);
}


inline
const TwoDVector& RoadSegment::GetEndLocation() const
{
    return (roadVertices.back().roadCenterLocation);
}


inline
void RoadSegment::GetFrontSubsegment(TwoDVector& otherLocation, TwoDVector& frontLocation) const
{
    assert(roadVertices.size() >= 2);
    otherLocation = roadVertices[1].roadCenterLocation;
    frontLocation = roadVertices.front().roadCenterLocation;
}


inline
void RoadSegment::GetEndSubsegment(TwoDVector& otherLocation, TwoDVector& endLocation) const
{
    assert(roadVertices.size() >= 2);
    otherLocation = roadVertices[roadVertices.size()-2].roadCenterLocation;
    endLocation = roadVertices.back().roadCenterLocation;
}


inline
const TwoDVector RoadSegment::GetFrontIntersectionLaneLocation(const unsigned int laneNumber) const
{
    return (GetXyPosition(PositionInRoadType(0.0, laneNumber)));
}


inline
const TwoDVector RoadSegment::GetEndIntersectionLaneLocation(const unsigned int laneNumber) const
{
    return (GetXyPosition(PositionInRoadType(GetLengthMeters(), laneNumber)));
}


inline
double RoadSegment::CalcDistanceLeftMeters(const PositionInRoadType& roadPosition) const
{
    if (IsAForwardLane(roadPosition.laneNumber)) {
        return (lengthMeters - roadPosition.positionInRoadMeters);
    }
    else {
        return (roadPosition.positionInRoadMeters);
    }//if//
}



inline
double RoadSegment::CalcLaneOffsetMeters(const unsigned int laneIndex) const
{
    // Lane numbering scheme for "right hand side": 543|012 (Backward|Forward).
    // Lane numbering scheme for "left  hand side": 210|345 (Forward|Backward).

    double offsetMeters;

    if (IsAForwardLane(laneIndex)) {
        offsetMeters = ((laneIndex + 0.5) * laneWidthMeters);
    }
    else {
        offsetMeters = (((laneIndex - numberForwardLanes) + 0.5) * (-laneWidthMeters));
    }//if//

    if (!isARightHandTrafficRoad) {
        offsetMeters = -offsetMeters;
    }//if//

    return (offsetMeters);

}//CalcLaneOffsetMeters//



inline
TwoDVector RoadSegment::GetXyPosition(const PositionInRoadType& roadPosition) const
{
    assert(roadPosition.positionInRoadMeters >= 0.0);
    assert(roadPosition.positionInRoadMeters <= GetLengthMeters());

    TwoDVector position;

    TwoDVector lineVertex1;
    TwoDVector lineVertex2;
    double offsetFromVertex1;

    if (roadPosition.positionInRoadMeters == 0.0) {
        lineVertex1 = roadVertices[0].roadCenterLocation;
        lineVertex2 = roadVertices[1].roadCenterLocation;
        offsetFromVertex1 = 0.0;
    }
    else if (roadPosition.positionInRoadMeters == GetLengthMeters()) {
        lineVertex1 = roadVertices[roadVertices.size()-2].roadCenterLocation;
        lineVertex2 = roadVertices[roadVertices.size()-1].roadCenterLocation;
        offsetFromVertex1 = (lineVertex2 - lineVertex1).Length();
    }
    else {
        double distanceSoFar = 0.0;
        for (unsigned int i = 1; (i < roadVertices.size()); i++) {

            const double subSegmentLength =
                (roadVertices[i].roadCenterLocation - roadVertices[i-1].roadCenterLocation).Length();

            if (roadPosition.positionInRoadMeters <= (distanceSoFar + subSegmentLength)) {

                lineVertex1 = roadVertices[i-1].roadCenterLocation;
                lineVertex2 = roadVertices[i].roadCenterLocation;
                offsetFromVertex1 = (roadPosition.positionInRoadMeters - distanceSoFar);

                break;
            }//if//

            distanceSoFar += subSegmentLength;
        }//for//
    }//if//

    const double laneOffsetMeters = CalcLaneOffsetMeters(roadPosition.laneNumber);

    return (
        CalcPointOnPerpendicularLine(
            lineVertex1,
            lineVertex2,
            offsetFromVertex1,
            laneOffsetMeters));

}//GetXyPosition//


inline
void RoadSegment::GetXyPositionAndAzimuth(
    const PositionInRoadType& roadPosition,
    TwoDVector& xyPosition,
    double& azimuthClockwiseRad) const
{

    assert(roadPosition.positionInRoadMeters >= 0.0);
    assert(roadPosition.positionInRoadMeters <= GetLengthMeters());

    TwoDVector position;

    TwoDVector lineVertex1;
    TwoDVector lineVertex2;
    double offsetFromVertex1;

    if (roadPosition.positionInRoadMeters == 0.0) {
        lineVertex1 = roadVertices[0].roadCenterLocation;
        lineVertex2 = roadVertices[1].roadCenterLocation;
        offsetFromVertex1 = 0.0;
    }
    else if (roadPosition.positionInRoadMeters == GetLengthMeters()) {
        lineVertex1 = roadVertices[roadVertices.size()-2].roadCenterLocation;
        lineVertex2 = roadVertices[roadVertices.size()-1].roadCenterLocation;
        offsetFromVertex1 = (lineVertex2 - lineVertex1).Length();
    }
    else {
        double distanceSoFar = 0.0;
        for (unsigned int i = 1; (i < roadVertices.size()); i++) {

            const double subSegmentLength =
                (roadVertices[i].roadCenterLocation - roadVertices[i-1].roadCenterLocation).Length();

            if (roadPosition.positionInRoadMeters <= (distanceSoFar + subSegmentLength)) {

                lineVertex1 = roadVertices[i-1].roadCenterLocation;
                lineVertex2 = roadVertices[i].roadCenterLocation;
                offsetFromVertex1 = (roadPosition.positionInRoadMeters - distanceSoFar);

                break;
            }//if//

            distanceSoFar += subSegmentLength;
        }//for//
    }//if//

    const double laneOffsetMeters = CalcLaneOffsetMeters(roadPosition.laneNumber);

    xyPosition =
        CalcPointOnPerpendicularLine(
            lineVertex1,
            lineVertex2,
            offsetFromVertex1,
            laneOffsetMeters);

    if (IsAForwardLane(roadPosition.laneNumber)) {
        azimuthClockwiseRad =  CalcAzimuthOfLineClockwiseFromNorthRad(lineVertex1, lineVertex2);
    }
    else {
        azimuthClockwiseRad =  CalcAzimuthOfLineClockwiseFromNorthRad(lineVertex2, lineVertex1);
    }//if//

}//GetXyPositionAndAzimuth//





inline
void RoadSegment::TrimRoadAtFront(const double& trimLengthMeters)
{
    assert(trimLengthMeters < GetLengthMeters());

    double leftToTrim = trimLengthMeters;
    TwoDVector frontLocation = roadVertices.front().roadCenterLocation;
    double firstSegmentLengthMeters;

    // Get rid of (unlikely) very short road vertices.

    while (true) {
        firstSegmentLengthMeters = (roadVertices[1].roadCenterLocation - frontLocation).Length();

        if (firstSegmentLengthMeters > leftToTrim) {
            break;
        }
        else {
            leftToTrim -= firstSegmentLengthMeters;
            (*this).lengthMeters -= firstSegmentLengthMeters;
            (*this).roadVertices.erase(roadVertices.begin());
            frontLocation = roadVertices.front().roadCenterLocation;
        }//if//
    }//while//

    // Replace first vertex.

    roadVertices.front().roadCenterLocation =
        CalcIntermediatePoint(
            frontLocation,
            roadVertices[1].roadCenterLocation,
            (leftToTrim / firstSegmentLengthMeters));

    (*this).lengthMeters -= leftToTrim;

}//TrimRoadAtFront//

inline
void RoadSegment::TrimRoadAtEnd(const double& trimLengthMeters)
{
    assert(trimLengthMeters < GetLengthMeters());

    double leftToTrim = trimLengthMeters;
    TwoDVector endLocation = roadVertices.back().roadCenterLocation;
    double firstSegmentLengthMeters;

    // Get rid of (unlikely) very short road vertices.

    while (true) {
        firstSegmentLengthMeters =
            (roadVertices[roadVertices.size() - 2].roadCenterLocation - endLocation).Length();

        if (firstSegmentLengthMeters > leftToTrim) {
            break;
        }
        else {
            leftToTrim -= firstSegmentLengthMeters;
            (*this).lengthMeters -= firstSegmentLengthMeters;
            (*this).roadVertices.pop_back();
            endLocation = roadVertices.back().roadCenterLocation;
        }//if//
    }//while//

    // Replace first vertex.

    roadVertices.back().roadCenterLocation =
        CalcIntermediatePoint(
            endLocation,
            roadVertices[roadVertices.size() - 2].roadCenterLocation,
            (leftToTrim / firstSegmentLengthMeters));

    (*this).lengthMeters -= leftToTrim;

}//TrimRoadAtEnd//



//--------------------------------------------------------------------------------------------------



struct RoadConnectionMapKeyType {
    RoadSegmentIdType roadSegmentId;
    RoadSegmentIdType destinationRoadSegmentId;

    RoadConnectionMapKeyType() { }
    RoadConnectionMapKeyType(
        const RoadSegmentIdType& initRoadSegmentId,
        const RoadSegmentIdType& initDestinationRoadSegmentId)
        :
        roadSegmentId(initRoadSegmentId),
        destinationRoadSegmentId(initDestinationRoadSegmentId)
    {}

    bool operator<(const RoadConnectionMapKeyType& right) const
    {
        return (((*this).roadSegmentId < right.roadSegmentId) ||
                (((*this).roadSegmentId == right.roadSegmentId) &&
                 ((*this).destinationRoadSegmentId < right.destinationRoadSegmentId)));
    }

};//RoadConnectionMapKeyType//



//--------------------------------------------------------------------------------------------------


class RoadSegmentState;


class AutoRoadNetwork {
public:
    AutoRoadNetwork(const ParameterDatabaseReader& theParameterDatabaseReader);

    RoadSegmentIdType GetRoadIndexFromName(const string& aLine, const string& roadName) const;
    const string& GetRoadNameFromIndex(const RoadSegmentIdType& roadId) const;

    bool GetIsARightHandTrafficRoadNetwork() const { return (isARightHandTrafficRoadNetwork); }

    unsigned int GetNumberRoadSegments() { return (static_cast<unsigned int>(roadSegments.size())); }

    const RoadSegment& GetRoadSegment(const RoadSegmentIdType& roadSegmentId) const
        { return (roadSegments.at(roadSegmentId)); }

    void CalcRoute(
        const RoadSegmentIdType& currentRoadId,
        const PositionInRoadType& currentRoadPosition,
        const RoadSegmentIdType& destinationRoadId,
        const PositionInRoadType& destinationRoadPosition,
        deque<RoadSegmentIdType>& newRoute);

    PositionInRoadType GetClosestConnectingRoadPosition(
        const RoadSegmentIdType& currentRoadId,
        const unsigned int currentLaneNumber,
        const RoadSegmentIdType& nextRoadId) const;

    void CheckRoadPosition(
        const unsigned int roadId,
        const unsigned int laneNumber,
        const double& roadPositionMeters,
        const string& errorMessage) const;

    void FindRoadAndLaneGoingBackwards(
        const RoadSegmentIdType roadId,
        const unsigned int laneNumber,
        bool& wasFound,
        RoadSegmentIdType& backwardsRoadId,
        bool& backwardsRoadIsConnectionAtFront,
        unsigned int& backwardsLaneNum) const;

    bool ThereIsAConflictingIntersectionCrossTraffic(
        const vector<RoadSegmentState>& roadStates,
        const RoadSegmentIdType& currentRoadId,
        const unsigned int currentLaneNumber,
        const RoadSegmentIdType& nextRoadId,
        const TimeType& neededConflictFreeTime) const;

private:
    bool isARightHandTrafficRoadNetwork;

    map<string, unsigned int> guiRoadNameToIndexMap;
    vector<string> guiRoadIndexToNameMap;

    map<RoadConnectionMapKeyType, RoadSegment::ConnectionInfoType> inputRoadConnectionMap;

    vector<RoadSegment> originalRoadSegments;

    // The roads are modified (shortened to outside the intersections).

    vector<RoadSegment> roadSegments;

    struct IntersectionInfoType {
        vector<RoadSegmentIdAndWhichEndType> externalRoadIds;
        vector<RoadSegmentIdType> internalLaneIds;
        // Precomputed tables:
        vector<vector<bool> > externalRoadConflictTable;
    };

    vector<IntersectionInfoType> intersections;

    // Abstract graph representation of road network for routing algorithm:

    WeightedGraph<double> roadGraph;


    //-------------------------------------------------------

    TwoDVector GetIntersectionPosition(const IntersectionIdType& intersectionId) const;

    void CheckRoute(
        const RoadSegmentIdType& startRoadId,
        const unsigned int& startLaneNum,
        const RoadSegmentIdType& destinationRoadId,
        const unsigned int& destinationLaneNum,
        const deque<RoadSegmentIdType>& aRoute) const;

    void CreateRoadGraph();

    RoadSegmentIdType LookupRoadIdFromIntersectionIds(
        const IntersectionIdType& intersectionId1,
        const IntersectionIdType& intersectionId2) const;

    void ShortenRoadAtIntersection(
        const RoadSegmentIdType& roadId,
        const bool intersectionIsAtFront);

    void AddConnectionsToRoad(
        const RoadSegmentIdType& roadId,
        const bool isAtFront);

    void ReadRoadConnectionInfo(
        const string& connectionsFilename,
        map<RoadConnectionMapKeyType, RoadSegment::ConnectionInfoType>& roadConnectionMap) const;

};//AutoRoadNetwork//



//--------------------------------------


inline
AutoRoadNetwork::AutoRoadNetwork(const ParameterDatabaseReader& theParameterDatabaseReader)
{
    using std::min;
    using std::max;

    string shapeFileDirPath;
    if (theParameterDatabaseReader.ParameterExists("gis-object-file-path")) {
        shapeFileDirPath = theParameterDatabaseReader.ReadString("gis-object-file-path");
    }//if//

    const string gisRoadsFilenamePrefix = shapeFileDirPath + "road";

    if (!ReadVehicleModelIsEnabled(theParameterDatabaseReader)) {
        return;
    }//if//

    isARightHandTrafficRoadNetwork =
        theParameterDatabaseReader.ReadBool(parameterNamePrefix + "right-hand-traffic-roads");

    string connectionsFilename = "connections.txt";

    if (theParameterDatabaseReader.ParameterExists(parameterNamePrefix + "connections-filename")) {

        connectionsFilename =
            theParameterDatabaseReader.ReadString(parameterNamePrefix + "connections-filename");
    }//if/

    double newMinX;
    double newMaxX;
    double newMinY;
    double newMaxY;

    vector<GisRoadSegment> gisRoadSegments;

    ReadRoads(
        gisRoadsFilenamePrefix,
        newMinX,
        newMaxX,
        newMinY,
        newMaxY,
        gisRoadSegments);

    if (gisRoadSegments.empty()) {
        return;
    }//if//

    for(unsigned int i = 0; (i < gisRoadSegments.size()); i++) {
        (*this).guiRoadNameToIndexMap[gisRoadSegments[i].GetRoadName()] = i;
        (*this).guiRoadIndexToNameMap.push_back(gisRoadSegments[i].GetRoadName());
    }//for//


    unsigned int roadId = GisRoadSegment::InvalidRoadId;

    for(unsigned int i = 0; (i < gisRoadSegments.size()); i++) {
        const GisRoadSegment& gisRoad = gisRoadSegments[i];

        if (gisRoad.GetRoadId() != roadId) {
            roadId = gisRoad.GetRoadId();

            RoadSegment newRoad(
                i,
                isARightHandTrafficRoadNetwork,
                TwoDVector(
                    gisRoad.GetFirstLocationXMeters(),
                    gisRoad.GetFirstLocationYMeters()),
                TwoDVector(
                    gisRoad.GetSecondLocationXMeters(),
                    gisRoad.GetSecondLocationYMeters()),
                gisRoad.GetNumberForwardLanes(),
                gisRoad.GetNumberBackwardLanes(),
                gisRoad.GetLaneWidthMeters(),
                gisRoad.GetSpeedLimitMetersSec(),
                false);

            originalRoadSegments.push_back(newRoad);
        }
        else {
            originalRoadSegments.back().AddSubsegment(
                TwoDVector(
                    gisRoad.GetSecondLocationXMeters(),
                    gisRoad.GetSecondLocationYMeters()),
                gisRoad.GetRoadMedianWidthMeters());
        }//if//
    }//for//

    ReadRoadConnectionInfo(connectionsFilename, (*this).inputRoadConnectionMap);

    (*this).CreateRoadGraph();

}//AutoRoadNetwork//



inline
RoadSegmentIdType AutoRoadNetwork::GetRoadIndexFromName(
    const string& aLine, const string& roadName) const
{
    typedef map<string, unsigned int>::const_iterator IterType;
    const IterType iter = guiRoadNameToIndexMap.find(roadName);
    if (iter == guiRoadNameToIndexMap.end()) {
        cerr << "Error in configuration file: Road name \"" << roadName << "\" was not found." << endl;
        cerr << "  For configuration file line: " << endl;
        cerr << "  " << aLine << endl;
        exit(1);
    }//if//

    return (iter->second);
}

inline
const string& AutoRoadNetwork::GetRoadNameFromIndex(const RoadSegmentIdType& roadId) const
{
    return (guiRoadIndexToNameMap.at(roadId));
}



inline
void AutoRoadNetwork::ReadRoadConnectionInfo(
    const string& connectionsFilename,
    map<RoadConnectionMapKeyType, RoadSegment::ConnectionInfoType>& roadConnectionMap) const
{
    roadConnectionMap.clear();

    std::ifstream connectionInfoStream(connectionsFilename);

    if (connectionInfoStream.fail()) {
        cerr << "Error opening file: " << connectionsFilename << endl;
        exit(1);
    }//if//

    const string errorMessage = "Error in file: " + connectionsFilename;

    while (!connectionInfoStream.eof()) {

        string aLine;
        getline(connectionInfoStream, aLine);

        if (connectionInfoStream.eof()) {
            break;
        }
        else if (connectionInfoStream.fail()) {
            cerr << "Error reading connection configuration file: " << connectionsFilename << endl;
            exit(1);
        }//if//

        DeleteTrailingSpaces(aLine);
        if (IsAConfigFileCommentLine(aLine)) {
            continue;
        }//if//

        istringstream lineStream(aLine);

        RoadConnectionMapKeyType connectionKey;

        string roadName;
        lineStream >> roadName;
        ConvertStringToLowerCase(roadName);

        connectionKey.roadSegmentId = GetRoadIndexFromName(aLine, roadName);

        unsigned int laneNumber;

        lineStream >> laneNumber;

        unsigned int numberOfLanes;
        lineStream >> numberOfLanes;

        string destRoadName;
        lineStream >> destRoadName;
        ConvertStringToLowerCase(destRoadName);

        connectionKey.destinationRoadSegmentId = GetRoadIndexFromName(aLine, destRoadName);

        unsigned int destinationLaneNumber;
        lineStream >> destinationLaneNumber;

        double speedLimitMetersSec;
        lineStream >> speedLimitMetersSec;

        string behaviorString;
        lineStream >> behaviorString;
        ConvertStringToLowerCase(behaviorString);

        IntersectionBehaviorType behavior;
        if (behaviorString == "signal") {
            behavior = IntersectionBehaviorType::SignalControlled;
        }
        else if (behaviorString == "keepgoing") {
            behavior = IntersectionBehaviorType::KeepGoing;
        }
        else if (behaviorString == "stop") {
            behavior = IntersectionBehaviorType::StopAndYield;
        }
        else {
            cerr << "Error in Connection file: Unknown beavior: " << behaviorString << endl;
            exit(1);
        }//if//

        double startTimeSecs;
        lineStream >> startTimeSecs;

        double greenLightDurationSecs;
        lineStream >> greenLightDurationSecs;

        double yellowLightDurationSecs;
        lineStream >> yellowLightDurationSecs;

        double redLightDurationSecs;
        lineStream >> redLightDurationSecs;

        if ((lineStream.fail()) || (!lineStream.eof()))  {
            cerr << errorMessage << endl;
            exit(1);
        }//if//

        typedef map<RoadConnectionMapKeyType, RoadSegment::ConnectionInfoType>::iterator IterType;
        IterType iter = roadConnectionMap.find(connectionKey);
        if (iter != roadConnectionMap.end()) {
            cerr << "Error in connection configuration file: Redundant connection entry:" << endl;
            cerr << "   Start Road #: " << (connectionKey.roadSegmentId + 1) << " Dest road #: "
                << (connectionKey.destinationRoadSegmentId + 1) << endl;
            exit(1);
        }//if//

        std::pair<RoadConnectionMapKeyType, RoadSegment::ConnectionInfoType> newConnectionPair;
        newConnectionPair.first = connectionKey;

        RoadSegment::ConnectionInfoType& newConnection = newConnectionPair.second;

        for (unsigned int i = 0; (i < numberOfLanes); i++) {
            RoadSegment::ConnectionLaneInfoType laneInfo;

            laneInfo.outgoingLaneNum = (laneNumber + i);
            laneInfo.nextRoadLaneNum = (destinationLaneNumber + i);
            // Note: Special intersection lane road segment is created later.
            laneInfo.intersectionLaneRoadId = InvalidRoadSegmentId;
            newConnection.intersectionLanes.push_back(laneInfo);
        }//for//

        // Note: Special intersection lane road segment is created later.

        newConnection.nextRoadId = connectionKey.destinationRoadSegmentId;
        newConnection.speedLimitMetersSec = speedLimitMetersSec;
        newConnection.behavior = behavior;

        const RoadSegment& road = originalRoadSegments.at(connectionKey.roadSegmentId);
        const RoadSegment& otherRoad = originalRoadSegments.at(newConnection.nextRoadId);

        newConnection.isAtFrontOfRoad =
            ((road.GetFrontLocation() == otherRoad.GetFrontLocation()) ||
             (road.GetFrontLocation() == otherRoad.GetEndLocation()));

        if (newConnection.isAtFrontOfRoad) {
            newConnection.nextRoadConnectedAtFront =
                (road.GetFrontLocation() == otherRoad.GetFrontLocation());
        }
        else {
            newConnection.nextRoadConnectedAtFront =
                (road.GetEndLocation() == otherRoad.GetFrontLocation());
        }//if//

        newConnection.intersectionId = InvalidIntersectionId;

        if (behavior == IntersectionBehaviorType::SignalControlled) {
            newConnection.trafficLightPtr =
                make_shared<SimpleTrafficLight>(
                ConvertDoubleSecsToTime(startTimeSecs),
                ConvertDoubleSecsToTime(greenLightDurationSecs),
                ConvertDoubleSecsToTime(yellowLightDurationSecs),
                ConvertDoubleSecsToTime(redLightDurationSecs));
        }//if//

        roadConnectionMap.insert(newConnectionPair);
    }//while//

}//ReadRoadConnectionInfo//






// Destination lane selection may be ambiguous/complex ==> temporarily input destination lane.
// Requires information about the straight through turn lanes.

//Future inline
//Future unsigned int CalcNextRoadDestinationLaneNumber(
//Future     const RoadSegment& road,
//Future     const bool intersectionIsAtFront,
//Future     const unsigned int outgoingLaneNum,
//Future     const RoadSegment& otherRoad,
//Future     const bool otherRoadIsAtFront,
//Future     const RoadTurnDirectionType& turnDirection)
//Future {
//Future     unsigned int laneNumber;
//Future     unsigned int numberLanes;
//Future
//Future     if (intersectionIsAtFront) {
//Future         laneNumber = outgoingLaneNumber;
//Future         numberLanes = road.GetNumberForwardLanes();
//Future         assert(laneNumber < road.GetNumberForwardLanes());
//Future     }
//Future     else {
//Future         laneNumber = outgoingLaneNumber - road.GetNumberForwardLanes();
//Future         numberLanes = road.GetNumberBackwardLanes();
//Future     }//if//
//Future
//Future     assert(laneNumber < numberLanes);
//Future
//Future
//Future     if (otherRoadIsAtFront) {
//Future         otherRoadNumberLanes = otherRoad.GetNumberForwardLanes();
//Future     }
//Future     else {
//Future         otherRoadNumberLanes = otherRoad.GetNumberBackwardLanes();
//Future     }//if//
//Future
//Future         assert(laneNumber < otherRoad.GetNumberForwardLanes());
//Future         return (laneNumber);
//Future     }
//Future     else {
//Future         assert(laneNumber < otherRoad.GetNumberBackwardLanes());
//Future         return(laneNumber + otherRoad.GetNumberForwardLanes());
//Future     }//if//
//Future
//Future }//CalcNextRoadDestinationLaneNumber//



inline
void BuildIntersectionLane(
    const RoadSegment& road,
    const bool intersectionIsAtFront,
    const unsigned int outgoingLaneNum,
    const RoadSegment& otherRoad,
    const bool otherRoadIsAtFront,
    const unsigned int nextRoadLaneNumber,
    const RoadSegmentIdType newRoadId,
    const double& speedLimitMetersSec,
    RoadSegment& newIntersectionLaneRoad)
{
    TwoDVector laneStartLocation;

    if (intersectionIsAtFront) {
        laneStartLocation = road.GetFrontIntersectionLaneLocation(outgoingLaneNum);
    }
    else {
        laneStartLocation = road.GetEndIntersectionLaneLocation(outgoingLaneNum);
    }//if//

    TwoDVector laneEndLocation;

    if (otherRoadIsAtFront) {
        laneEndLocation = otherRoad.GetFrontIntersectionLaneLocation(nextRoadLaneNumber);
    }
    else {
        laneEndLocation = otherRoad.GetEndIntersectionLaneLocation(nextRoadLaneNumber);
    }//if//

    // Add Fancy curved lane generation here.

    newIntersectionLaneRoad =
        RoadSegment(
            newRoadId,
            road.GetIsARightHandTrafficRoadNetwork(),
            laneStartLocation,
            laneEndLocation,
            1,
            0,
            0.0,
            speedLimitMetersSec,
            true);

    // Add pseudo connection info.  Mostly used to satisify route search logic.

    RoadSegment::ConnectionInfoType pseudoConnection;

    pseudoConnection.isAtFrontOfRoad = false;
    pseudoConnection.behavior = IntersectionBehaviorType::KeepGoing;
    pseudoConnection.intersectionId = InvalidIntersectionId;
    pseudoConnection.isAContinuationIntersection = false;
    pseudoConnection.turnDirection = RoadTurnDirectionType::Straight;
    pseudoConnection.nextRoadId = otherRoad.GetRoadSegmentId();
    pseudoConnection.nextRoadConnectedAtFront = otherRoadIsAtFront;
    pseudoConnection.speedLimitMetersSec = DBL_MAX;
    RoadSegment::ConnectionLaneInfoType laneInfo;
    laneInfo.nextRoadLaneNum = nextRoadLaneNumber;
    laneInfo.outgoingLaneNum = 0;
    laneInfo.intersectionLaneRoadId = InvalidRoadSegmentId;
    pseudoConnection.intersectionLanes.push_back(laneInfo);

    newIntersectionLaneRoad.AddConnection(pseudoConnection);

}//BuildIntersectionLane//



inline
void AutoRoadNetwork::ShortenRoadAtIntersection(
    const RoadSegmentIdType& roadId,
    const bool intersectionIsAtFront)
{
    using std::max;

    RoadSegment& originalRoad = originalRoadSegments.at(roadId);

    TwoDVector aRoadLocation;
    IntersectionIdType intersectionId;
    TwoDVector intersectionLocation;

    if (intersectionIsAtFront) {
        intersectionId = originalRoad.GetFrontIntersectionId();
        originalRoad.GetFrontSubsegment(aRoadLocation, intersectionLocation);
    }
    else {
        intersectionId = originalRoad.GetEndIntersectionId();
        originalRoad.GetEndSubsegment(aRoadLocation, intersectionLocation);
    }//if//

    const vector<RoadSegmentIdAndWhichEndType>& roadInfos =
        intersections[intersectionId].externalRoadIds;

    // Calculate other road's azimuths to road.

    vector<double> azimuthAngles(roadInfos.size(), 0.0);

    for(unsigned int i = 0; (i < roadInfos.size()); i++) {
        if (roadInfos[i].roadId == originalRoad.GetRoadSegmentId()) {
            continue;
        }//if//

        const RoadSegment& otherRoad = originalRoadSegments[roadInfos[i].roadId];
        const bool otherRoadIsConnectedAtFront =
            (intersectionId == otherRoad.GetFrontIntersectionId());

        TwoDVector otherRoadLocation;
        TwoDVector checkLocation;

        if (otherRoadIsConnectedAtFront) {
            otherRoad.GetFrontSubsegment(otherRoadLocation, checkLocation);
        }
        else {
            otherRoad.GetEndSubsegment(otherRoadLocation, checkLocation);
        }//if//

        assert(checkLocation == intersectionLocation);

        const double azimuthAngleRad =
            CalcClockwiseAzimuthAngleChangeRad(
                aRoadLocation, intersectionLocation, otherRoadLocation);

        azimuthAngles[i] = azimuthAngleRad;
    }//for//

    unsigned int continuationRoadIndex = UINT_MAX;

    for(unsigned int i = 0; (i < azimuthAngles.size()); i++) {
        if ((roadInfos[i].roadId != originalRoad.GetRoadSegmentId()) &&
            (azimuthAngles[i] < StraightRoadMaxDeltaAzimuthRad)) {

            continuationRoadIndex = i;
            break;
        }//if//
    }//for//

    // Find Widest Cross road:

    double widestCrossroadMeters = 0.0;

    for(unsigned int i = 0; (i < roadInfos.size()); i++) {
        if ((roadInfos[i].roadId == originalRoad.GetRoadSegmentId()) ||
            (i == continuationRoadIndex)) {
            continue;
        }//if//

        const RoadSegment& otherRoad = originalRoadSegments[roadInfos[i].roadId];
        widestCrossroadMeters = max(widestCrossroadMeters, otherRoad.GetWidthMeters());

    }//for//

    if (widestCrossroadMeters > 0.0) {
        RoadSegment& road = roadSegments.at(roadId);

        if (intersectionIsAtFront) {
            road.TrimRoadAtFront(widestCrossroadMeters / 2.0);
        }
        else {
            road.TrimRoadAtEnd(widestCrossroadMeters / 2.0);
        }//if//
    }//if//

}//ShortenRoadAtIntersection//



inline
void AutoRoadNetwork::AddConnectionsToRoad(
    const RoadSegmentIdType& roadId,
    const bool connectionIsAtFront)
{
    if (roadId >= originalRoadSegments.size()) {
        // Road is a intersection lane pseudo road.
        return;
    }//if//

    IntersectionIdType intersectionId;
    if (connectionIsAtFront) {
        intersectionId = roadSegments.at(roadId).GetFrontIntersectionId();
    }
    else {

        intersectionId = roadSegments.at(roadId).GetEndIntersectionId();
    }//if/

    const vector<RoadSegmentIdAndWhichEndType>& roadInfos =
        intersections[intersectionId].externalRoadIds;

    if (roadInfos.size() == 1) {
        // No connections.
        assert(roadInfos[0].roadId == roadId);
        return;
    }

    const RoadSegment& originalRoad = originalRoadSegments.at(roadId);

    TwoDVector aRoadLocation;
    TwoDVector intersectionLocation;

    if (connectionIsAtFront) {
        originalRoad.GetFrontSubsegment(aRoadLocation, intersectionLocation);
    }
    else {
        originalRoad.GetEndSubsegment(aRoadLocation, intersectionLocation);
    }//if//


    // Calculate other road's azimuths to road.

    vector<double> azimuthAngles(roadInfos.size(), 0.0);

    for(unsigned int i = 0; (i < roadInfos.size()); i++) {
        if (roadInfos[i].roadId == roadId) {
            continue;
        }//if//

        const RoadSegment& otherRoad = originalRoadSegments[roadInfos[i].roadId];
        const bool otherRoadIsConnectedAtFront =
            (intersectionId == otherRoad.GetFrontIntersectionId());

        TwoDVector otherRoadLocation;
        TwoDVector checkLocation;

        if (otherRoadIsConnectedAtFront) {
            otherRoad.GetFrontSubsegment(otherRoadLocation, checkLocation);
        }
        else {
            otherRoad.GetEndSubsegment(otherRoadLocation, checkLocation);
        }//if//

        assert(checkLocation == intersectionLocation);

        const double azimuthAngleRad =
            CalcClockwiseAzimuthAngleChangeRad(
                aRoadLocation, intersectionLocation, otherRoadLocation);

        azimuthAngles[i] = azimuthAngleRad;

    }//for//


    // Process read-in connection information.

    typedef map<RoadConnectionMapKeyType, RoadSegment::ConnectionInfoType>::const_iterator IterType;

    IterType iter = inputRoadConnectionMap.lower_bound(RoadConnectionMapKeyType(roadId, 0));
    while ((iter != inputRoadConnectionMap.end()) && iter->first.roadSegmentId == roadId) {
        const RoadConnectionMapKeyType& key = iter->first;
        RoadSegment::ConnectionInfoType connection = iter->second;

        if (connection.isAtFrontOfRoad != connectionIsAtFront) {
            ++iter;
            continue;
        }//if//

        connection.intersectionId = intersectionId;
        connection.isAContinuationIntersection = true;

        // Only build intersection lanes for full intersections, i.e. don't build
        // zero length connector road.

        if (((connectionIsAtFront) &&
             (roadSegments.at(roadId).GetFrontLocation() !=
              originalRoadSegments.at(roadId).GetFrontLocation())) ||
            ((!connectionIsAtFront) &&
             (roadSegments.at(roadId).GetEndLocation() !=
              originalRoadSegments.at(roadId).GetEndLocation()))) {

            connection.isAContinuationIntersection = false;

            for (unsigned int i = 0;(i < connection.intersectionLanes.size()); i++) {
                RoadSegment::ConnectionLaneInfoType& laneInfo = connection.intersectionLanes[i];

                laneInfo.intersectionLaneRoadId = static_cast<RoadSegmentIdType>(roadSegments.size());

                // Warning: Modifying roadSegments (invalidates references).

                roadSegments.resize(roadSegments.size()+1);
                RoadSegment& newIntersectionLaneRoad = roadSegments.back();

                const RoadSegment& otherRoad = roadSegments.at(key.destinationRoadSegmentId);
                RoadSegment& road = (*this).roadSegments.at(roadId);

                BuildIntersectionLane(
                    road,
                    connectionIsAtFront,
                    laneInfo.outgoingLaneNum,
                    otherRoad,
                    roadInfos[i].isAtFrontOfRoad,
                    laneInfo.nextRoadLaneNum,
                    laneInfo.intersectionLaneRoadId,
                    connection.speedLimitMetersSec,
                    newIntersectionLaneRoad);

            }//for//
        }//if//

        for(unsigned j = 0; (j < roadInfos.size()); j++) {
            if (connection.nextRoadId == roadInfos[j].roadId) {
                connection.turnDirection =
                    CalcTurnDirection(
                        isARightHandTrafficRoadNetwork,
                        StraightRoadMaxDeltaAzimuthRad,
                        azimuthAngles[j]);
                break;
            }//if//
        }//for//

        (*this).roadSegments.at(roadId).AddConnection(connection);

        ++iter;

    }//while//

    if (inputRoadConnectionMap.empty()) {

        // Temporary code for 1 lane road network test case support.  Auto generate connections.

        assert((originalRoad.GetNumberForwardLanes() <= 1) && (originalRoad.GetNumberBackwardLanes() <= 1));

        for(unsigned int i = 0; (i < roadInfos.size()); i++) {
            if (roadInfos[i].roadId == roadId) {
                continue;
            }//if//

            const RoadSegment& otherRoad = roadSegments[roadInfos[i].roadId];
            const bool otherRoadIsConnectedAtFront =
                (intersectionId == otherRoad.GetFrontIntersectionId());

            RoadSegment::ConnectionInfoType connection;

            connection.intersectionId = intersectionId;
            connection.isAContinuationIntersection = false;
            connection.speedLimitMetersSec = otherRoad.GetSpeedLimitMetersSec();
            connection.behavior = IntersectionBehaviorType::KeepGoing;
            connection.isAtFrontOfRoad = connectionIsAtFront;
            connection.nextRoadId = roadInfos[i].roadId;
            connection.nextRoadConnectedAtFront = otherRoadIsConnectedAtFront;

            RoadSegment::ConnectionLaneInfoType laneInfo;

            if (connectionIsAtFront) {
                laneInfo.outgoingLaneNum = 1;
            }
            else {
                laneInfo.outgoingLaneNum = 0;
            }//if//

            if(otherRoadIsConnectedAtFront) {
                laneInfo.nextRoadLaneNum  = 0;
            }
            else {
                laneInfo.nextRoadLaneNum  = 1;
            }//if//

            laneInfo.intersectionLaneRoadId = InvalidRoadSegmentId;

            connection.intersectionLanes.push_back(laneInfo);

            (*this).roadSegments.at(roadId).AddConnection(connection);

        }//for//
    }//if//

}//AddConnectionsToRoad//





inline
void AutoRoadNetwork::CreateRoadGraph()
{
    (*this).roadGraph.Clear();

    map<TwoDVector, IntersectionIdType> intersectionIndexMap;
    typedef map<TwoDVector, unsigned int>::iterator IterType;

    for(RoadSegmentIdType i = 0; (i < originalRoadSegments.size()); i++) {

        RoadSegment& road = originalRoadSegments[i];

        IntersectionIdType frontIntersectionId;

        IterType iter = intersectionIndexMap.find(road.GetFrontLocation());

        if (iter != intersectionIndexMap.end()) {
            frontIntersectionId = iter->second;
        }
        else {
            (*this).roadGraph.AddNewVertex(frontIntersectionId);
            intersectionIndexMap[road.GetFrontLocation()] = frontIntersectionId;
            if (frontIntersectionId >= intersections.size()) {
                (*this).intersections.resize(frontIntersectionId + 1);
            }//if//
        }//if//

        (*this).intersections[frontIntersectionId].externalRoadIds.push_back(
            RoadSegmentIdAndWhichEndType(i, true));

        GraphVertexIdType endIntersectionId;

        iter = intersectionIndexMap.find(road.GetEndLocation());

        if (iter != intersectionIndexMap.end()) {
            endIntersectionId = iter->second;
        }
        else {
            (*this).roadGraph.AddNewVertex(endIntersectionId);
            intersectionIndexMap[road.GetEndLocation()] = endIntersectionId;

            if (endIntersectionId >= intersections.size()) {
                (*this).intersections.resize(endIntersectionId + 1);
            }//if//
        }//if//

        (*this).intersections[endIntersectionId].externalRoadIds.push_back(
            RoadSegmentIdAndWhichEndType(i, false));

        road.SetFrontIntersectionId(frontIntersectionId);
        road.SetEndIntersectionId(endIntersectionId);

        if (road.HasForwardLanes()) {
            (*this).roadGraph.AddEdge(frontIntersectionId, endIntersectionId, road.GetLengthMeters());
        }//if//

        if (road.HasBackwardLanes()) {
            (*this).roadGraph.AddEdge(endIntersectionId, frontIntersectionId, road.GetLengthMeters());
        }//if//
    }//for//

    // Shorten roads to border of intersection.
    // Note: Original Roads kept in originalRoadSegments.

    (*this).roadSegments = originalRoadSegments;

    for(RoadSegmentIdType i = 0; (i < roadSegments.size()); i++) {

        (*this).ShortenRoadAtIntersection(i, true);
        (*this).ShortenRoadAtIntersection(i, false);

    }//for//


    // Add connections to roads. Connection pseudo intersection lane roads added to
    // end of roadSegments.

    const size_t numberRoads = roadSegments.size();

    for(RoadSegmentIdType i = 0; (i < numberRoads); i++) {

        (*this).AddConnectionsToRoad(i, true);
        (*this).AddConnectionsToRoad(i, false);

    }//for//

}//CreateRoadGraph//





inline
RoadSegmentIdType AutoRoadNetwork::LookupRoadIdFromIntersectionIds(
    const GraphVertexIdType& intersectionId1,
    const GraphVertexIdType& intersectionId2) const
{
    const vector<RoadSegmentIdAndWhichEndType>& intersection1RoadIds =
        intersections.at(intersectionId1).externalRoadIds;

    const vector<RoadSegmentIdAndWhichEndType>& intersection2RoadIds =
        intersections.at(intersectionId2).externalRoadIds;

    for(unsigned int i = 0; (i < intersection1RoadIds.size()); i++) {
        for(unsigned int j = 0; (j < intersection2RoadIds.size()); j++) {
            if (intersection1RoadIds[i].roadId == intersection2RoadIds[j].roadId) {
                return (intersection1RoadIds[i].roadId);
            }//if//
        }//for//
    }//for//

    assert(false && "Road was not found"); abort(); return 0;

}//LookupRoadIdFromRoadGraphVertexIds//


inline
TwoDVector AutoRoadNetwork::GetIntersectionPosition(const IntersectionIdType& intersectionId) const
{
    assert(intersectionId < intersections.size());

    const RoadSegmentIdAndWhichEndType& roadInfo =
        intersections[intersectionId].externalRoadIds.front();

    const RoadSegment& road = roadSegments[roadInfo.roadId];

    if (roadInfo.isAtFrontOfRoad) {
        return (road.GetFrontLocation());
    }
    else {
        return (road.GetEndLocation());
    }//if//

}//GetIntersectionPosition//


inline
void AutoRoadNetwork::CheckRoute(
    const RoadSegmentIdType& startRoadId,
    const unsigned int& startLaneNum,
    const RoadSegmentIdType& destinationRoadId,
    const unsigned int& destinationLaneNum,
    const deque<RoadSegmentIdType>& aRoute) const
{
    for(unsigned int i = 1; (i < aRoute.size()); i++) {
       if(aRoute[i-1] == aRoute[i]) {
           cerr << "Multiagent2 Vehicle Model Error: Route can only be achieved with a U-Turn:" << endl;
           cerr << "   From " << GetRoadNameFromIndex(startRoadId) << " Lane " << startLaneNum << " to "
                << GetRoadNameFromIndex(destinationRoadId) << " Lane " << destinationLaneNum << "." << endl;
           exit(1);
       }//if//
    }//for//

}//CheckRoute//



inline
void AutoRoadNetwork::CalcRoute(
    const RoadSegmentIdType& currentRoadId,
    const PositionInRoadType& currentRoadPosition,
    const RoadSegmentIdType& destinationRoadId,
    const PositionInRoadType& destinationRoadPosition,
    deque<RoadSegmentIdType>& newRoute)
{
    class DistanceHeuristicFunction: public AstarHeuristicFunction {
    public:
        DistanceHeuristicFunction(const AutoRoadNetwork& initAutoRoadNetworkRef) :
            autoRoadNetworkRef(initAutoRoadNetworkRef) { }

        virtual double estimate(
            const GraphVertexIdType& startVertexId,
            const GraphVertexIdType& endVertexId) const override
        {
            return (
                CalcDistance(
                    autoRoadNetworkRef.GetIntersectionPosition(startVertexId),
                    autoRoadNetworkRef.GetIntersectionPosition(endVertexId)));
        }

    private:
        const AutoRoadNetwork& autoRoadNetworkRef;

        DistanceHeuristicFunction(const DistanceHeuristicFunction&) = delete;
        // Temporary function object whose lifetime should be just during a function call.
        void* operator new(size_t) = delete;

    };//DistanceHeuristicFunction//


    //-----------------------------------------------------

    newRoute.clear();

    const RoadSegment& road = roadSegments.at(currentRoadId);

    if (currentRoadId == destinationRoadId) {

        if (((road.IsAForwardLane(currentRoadPosition.laneNumber)) &&
             ((!road.IsAForwardLane(destinationRoadPosition.laneNumber)) ||
              (currentRoadPosition.positionInRoadMeters > destinationRoadPosition.positionInRoadMeters))) ||
             ((!road.IsAForwardLane(currentRoadPosition.laneNumber)) &&
             ((road.IsAForwardLane(destinationRoadPosition.laneNumber)) ||
              (currentRoadPosition.positionInRoadMeters < destinationRoadPosition.positionInRoadMeters)))) {

            cerr << "Multiagent2: error in vehicle routing: Start and destination are the same road but" << endl;
            cerr << "   are on the different sides of street or same side but backwards." << endl;
            exit(1);
        }//if//

        newRoute.push_back(currentRoadId);
        return;

    }//if//

    GraphVertexIdType startVertexId;

    if (road.IsAForwardLane(currentRoadPosition.laneNumber)) {
        startVertexId = road.GetEndIntersectionId();
    }
    else {
        startVertexId = road.GetFrontIntersectionId();
    }//if//

    const RoadSegment& destinationRoad = roadSegments.at(destinationRoadId);

    GraphVertexIdType destinationVertexId;

    // For hack to avoid U-turns.  Set weight of reverse direction on destination road to extreme
    // values.


    const double HugeWeightMeters = 1.0e10;


    double savedWeight = HugeWeightMeters;
    bool wasSwapped;

    if (destinationRoad.IsAForwardLane(destinationRoadPosition.laneNumber)) {
        destinationVertexId = destinationRoad.GetFrontIntersectionId();
        roadGraph.SwapEdgeWeight(
            destinationRoad.GetEndIntersectionId(),
            destinationRoad.GetFrontIntersectionId(),
            savedWeight,
            wasSwapped);
    }
    else {
        destinationVertexId = destinationRoad.GetEndIntersectionId();
        roadGraph.SwapEdgeWeight(
            destinationRoad.GetFrontIntersectionId(),
            destinationRoad.GetEndIntersectionId(),
            savedWeight,
            wasSwapped);
    }//if//

    vector<GraphVertexIdType> bestPath;

    AstarSearchGraph(
        DistanceHeuristicFunction(*this),
        roadGraph,
        startVertexId,
        destinationVertexId,
        bestPath);

    if (wasSwapped) {

        // Return graph weights back to normal


        if (destinationRoad.IsAForwardLane(destinationRoadPosition.laneNumber)) {
            bool notUsed;
            roadGraph.SetEdgeWeight(
                destinationRoad.GetEndIntersectionId(),
                destinationRoad.GetFrontIntersectionId(),
                savedWeight,
                notUsed);
        }
        else {
            bool notUsed;
            roadGraph.SetEdgeWeight(
                destinationRoad.GetFrontIntersectionId(),
                destinationRoad.GetEndIntersectionId(),
                savedWeight,
                notUsed);
        }//if//
    }//if//

    newRoute.push_back(currentRoadId);

    for(unsigned int i = 1; (i < bestPath.size()); i++) {
        newRoute.push_back(LookupRoadIdFromIntersectionIds(bestPath[i-1], bestPath[i]));
    }//for//

    newRoute.push_back(destinationRoadId);

    CheckRoute(
        currentRoadId,
        currentRoadPosition.laneNumber,
        destinationRoadId,
        destinationRoadPosition.laneNumber,
        newRoute);

}//CalcRoute//



inline
PositionInRoadType AutoRoadNetwork::GetClosestConnectingRoadPosition(
    const RoadSegmentIdType& currentRoadId,
    const unsigned int currentLaneNumber,
    const RoadSegmentIdType& nextRoadId) const
{
    const RoadSegment& currentRoad = roadSegments.at(currentRoadId);
    const RoadSegment& nextRoad = roadSegments.at(nextRoadId);

    const RoadSegment::ConnectionInfoType& connection = currentRoad.GetConnectionInfo(nextRoadId);

    PositionInRoadType position;

    assert(connection.IsOneOfTheLanes(currentLaneNumber));

    position.laneNumber = connection.GetLaneInfo(currentLaneNumber).nextRoadLaneNum;

    position.positionInRoadMeters = 0.0;

    if (!connection.nextRoadConnectedAtFront) {
        position.positionInRoadMeters = nextRoad.GetLengthMeters();
    }//if//

    return position;

}//GetConnectingRoadPosition//



inline
void AutoRoadNetwork::CheckRoadPosition(
    const unsigned int roadId,
    const unsigned int laneNumber,
    const double& roadPositionMeters,
    const string& errorMessage) const
{
    if (roadId >= roadSegments.size()) {
        cerr << errorMessage << endl;
        cerr << "   Road Index = " << roadId << " does not exist." << endl;
        exit(1);
    }//if//

    const RoadSegment& aRoad = roadSegments[roadId];

    if ((roadPositionMeters < 0.0) || (roadPositionMeters > aRoad.GetLengthMeters())) {

        cerr << errorMessage << endl;
        cerr << "   For Road Name = " << guiRoadIndexToNameMap.at(roadId)
             << ": Position = " << roadPositionMeters << " is invalid." << endl;
        exit(1);

    }//if//

    if (laneNumber >= aRoad.GetNumberLanes()) {
        cerr << errorMessage << endl;
        cerr << "   For Road Name = " << guiRoadIndexToNameMap.at(roadId)
             << ": Lane = " << laneNumber << " is invalid." << endl;
        exit(1);

    }//if//

}//CheckRoadPosition//



inline
void ReadIntelligentDriverModelParameters(
    const ParameterDatabaseReader& parameterDatabaseReader,
    IntelligentDriverModelParameters& idmParams)
{
    idmParams.maxAccelerationMetersSec =
        parameterDatabaseReader.ReadDouble("intelligent-driver-model-max-acceleration-meters-sec");

    idmParams.comfortableDeaccelerationMetersSec =
        parameterDatabaseReader.ReadDouble("intelligent-driver-model-desired-braking-deacceleration-meters-sec");

    if (idmParams.comfortableDeaccelerationMetersSec <= 0.0) {
        cerr << "Error in \"intelligent-driver-model-comfortable-deacceleration-meters-sec\" parameter:" << endl;
        cerr <<     "Braking acceleration should be positive value." << endl;
        exit(1);
    }//if//

    idmParams.minGapMeters =
        parameterDatabaseReader.ReadDouble("intelligent-driver-model-min-gap-meters");

    idmParams.timeHeadwaySecs =
        parameterDatabaseReader.ReadDouble("intelligent-driver-model-time-headway-secs");

    idmParams.searchDistanceCutoffMeters =
        parameterDatabaseReader.ReadDouble("intelligent-driver-model-search-distance-cutoff-meters");

    idmParams.forcedLaneChangeStopDistancePerLaneMeters =
        parameterDatabaseReader.ReadDouble(
            parameterNamePrefix + "forced-lane-change-stop-distance-per-lane-meters");


}//ReadIntelligentDriverModelParameters//



//------------------------------------------------------------------------------


class Vehicle: public ScenSim::NetworkNode {
public:
    typedef unsigned int VehicleIdType;

    Vehicle(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const RandomNumberGeneratorSeedType& runSeed,
        const VehicleIdType& vehicleId,
        const ScenSim::NodeIdType& nodeId);

    VehicleIdType GetVehicleId() const { return (vehicleId); }
    double GetLengthMeters() const { return 4.0; }//TBD
    double GetHalfLengthMeters() const { return (GetLengthMeters() / 2.0); }

    bool IsEnabled() const {
        return (startTime <= simulationEngineInterfacePtr->CurrentTime());
    }

    void SetStartTime(const TimeType& newStartTime) { startTime = newStartTime; }
    TimeType GetStartTime() const { return (startTime); }

    void SetRoadId(const RoadSegmentIdType&  newRoadId) {
        (*this).currentRoadId = newRoadId;
        (*this).nextRoadId = newRoadId;
    }

    RoadSegmentIdType GetCurrentRoadId() const { return (currentRoadId); }

    void SetRoadPosition(
        const PositionInRoadType& newPosition,
        const TwoDVector& newXyPosition,
        const double& newAttitudeAzimuthClockwiseFromNorthRad);

    const PositionInRoadType& GetCurrentPosition() const { return (currentPosition); }

    void SetDestinationRoadId(const RoadSegmentIdType&  newRoadId) { (*this).destinationRoadId = newRoadId; }
    RoadSegmentIdType GetDestinationRoadId() const { return (destinationRoadId); }

    void SetDestinationRoadPosition(const PositionInRoadType& position) {
        (*this).destinationRoadPosition = position;
    }
    const PositionInRoadType& GetDestinationPosition() const { return (destinationRoadPosition); }

    void SetVelocity(const double& velocityMetersSec) {
        assert(velocityMetersSec >= 0.0);
        currentVelocity = velocityMetersSec;
        nextVelocity = velocityMetersSec;
    }

    double GetCurrentVelocity() const { return (currentVelocity); }

    //const PositionInRoadType& GetCurrentWaypointPosition() const;

    void SetNextRoadId(const RoadSegmentIdType& roadId) { (*this).nextRoadId = roadId; }
    void SetNextPosition(
        const PositionInRoadType& position,
        const TwoDVector& xyPosition,
        const double& attitudeAzimuthClockwiseFromNorthRad)
    {
        (*this).nextPosition = position;
        (*this).nextXyPosition = xyPosition;
        (*this).nextAttitudeAzimuthClockwiseFromNorthRad = attitudeAzimuthClockwiseFromNorthRad;
    }

    void SetNextVelocity(const double& velocity) {
        assert(velocity >= 0.0);

        (*this).nextVelocity = velocity;
    }

    RoadSegmentIdType GetNextRoadId() const { return (nextRoadId); }

    void SetRoute(const deque<RoadSegmentIdType>& newRoute) { (*this).currentRoute = newRoute; }
    bool HasARoute() const { return (!currentRoute.empty()); }
    const deque<RoadSegmentIdType>& GetCurrentRoute() const { return (currentRoute); }
    void DeleteFrontOfCurrentRoute() { (*this).currentRoute.pop_front(); }
    void ClearCurrentRoute() { (*this).currentRoute.clear(); }

    void UpdateToNextTimestep();

    virtual const ObjectMobilityPosition GetCurrentLocation() const override;

private:
    TimeType startTime;

    VehicleIdType vehicleId;
    RoadSegmentIdType destinationRoadId;
    PositionInRoadType destinationRoadPosition;

    RoadSegmentIdType currentRoadId;
    PositionInRoadType currentPosition;

    double currentVelocity;

    RoadSegmentIdType  nextRoadId;
    PositionInRoadType nextPosition;
    double nextVelocity;

    deque<RoadSegmentIdType> currentRoute;

    // Redundant, so don't have to lookup road information.

    TwoDVector currentXyPosition;
    double attitudeAzimuthClockwiseFromNorthRad;

    TwoDVector nextXyPosition;
    double nextAttitudeAzimuthClockwiseFromNorthRad;

    double GetXPos() const { return (currentXyPosition.GetX()); }
    double GetYPos() const { return (currentXyPosition.GetY()); }

};//Vehicle//




inline
Vehicle::Vehicle(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const RandomNumberGeneratorSeedType& runSeed,
    const VehicleIdType& initVehicleId,
    const ScenSim::NodeIdType& nodeId)
    :
    NetworkNode(
        theParameterDatabaseReader,
        ScenSim::GlobalNetworkingObjectBag(),
        simulationEngineInterfacePtr,
        nullptr,
        nodeId,
        runSeed,
        true),
    vehicleId(initVehicleId),
    startTime(ZERO_TIME)
{
}


inline
void Vehicle::SetRoadPosition(
    const PositionInRoadType& newPosition,
    const TwoDVector& newXyPosition,
    const double& newAttitudeAzimuthClockwiseFromNorthRad)
{
    (*this).currentPosition = newPosition;
    (*this).nextPosition = newPosition;
    (*this).currentXyPosition = newXyPosition;
    (*this).nextXyPosition = newXyPosition;
    (*this).attitudeAzimuthClockwiseFromNorthRad = newAttitudeAzimuthClockwiseFromNorthRad;
    (*this).nextAttitudeAzimuthClockwiseFromNorthRad = newAttitudeAzimuthClockwiseFromNorthRad;
}


inline
void Vehicle::UpdateToNextTimestep()
{
    (*this).currentRoadId = nextRoadId;
    (*this).currentPosition = nextPosition;
    (*this).currentXyPosition = nextXyPosition;
    (*this).currentVelocity = nextVelocity;
    (*this).attitudeAzimuthClockwiseFromNorthRad = nextAttitudeAzimuthClockwiseFromNorthRad;


}

inline
const ObjectMobilityPosition Vehicle::GetCurrentLocation() const
{
    return (
        ObjectMobilityPosition(
            INFINITE_TIME,
            ZERO_TIME,
            GetXPos(), GetYPos(), 0.0, false,
            (attitudeAzimuthClockwiseFromNorthRad * DegreesPerRadian),
            0.0, 0.0, 0.0, 0.0));
}




//=============================================================================================



class RoadSegmentState {
public:
    void UpdateVehicleOrder();

    unsigned int GetNumberVehicles() const { return (static_cast<unsigned int>(vehicles.size())); }

    unsigned int GetVehicleIndex(const Vehicle::VehicleIdType& vehicleId) const;

    void GetFirstVehicleIndex(
        const unsigned int laneNumber,
        const bool isGoingForwardDirection,
        bool& wasFound,
        unsigned int& vehicleIndex) const;

    void GetNextVehicleIndex(
        const unsigned int vehicleIndex,
        const unsigned int laneNumber,
        const bool isGoingForward,
        bool& wasFound,
        unsigned int& nextVehicleIndex) const;

    void GetPreviousVehicleIndex(
        const unsigned int vehicleIndex,
        const unsigned int laneNumber,
        const bool isGoingForward,
        bool& wasFound,
        unsigned int& previousVehicleIndex) const;


    const Vehicle& GetVehicle(const unsigned int vehicleIndex) const
    {
        return (*vehicles.at(vehicleIndex));

    }

    const shared_ptr<Vehicle>& GetVehiclePtr(const unsigned int vehicleIndex) const
    {
        return (vehicles.at(vehicleIndex));

    }


    const PositionInRoadType& GetVehiclePosition(const unsigned int vehicleIndex) const
    {
        return (GetVehicle(vehicleIndex).GetCurrentPosition());
    }

    void AddVehicle(const shared_ptr<Vehicle>& vehiclePtr)
    {
        assert(std::find(vehicles.begin(), vehicles.end(), vehiclePtr) == vehicles.end());
        vehicles.push_back(vehiclePtr);
    }

    void DeleteVehicleFromRoad(const Vehicle::VehicleIdType& vehicleId);

private:

    //Future: "Jaywalker" locations, i.e. not in signal controlled crosswalk.

    vector<shared_ptr<Vehicle> > vehicles;

};//RoadSegmentState//




inline
unsigned int RoadSegmentState::GetVehicleIndex(const Vehicle::VehicleIdType& vehicleId) const
{
    for(unsigned int i = 0; (i < vehicles.size()); i++) {
        if (vehicles[i]->GetVehicleId() == vehicleId) {
            return i;
        }//if//
    }//for//

    assert(false && "Vehicle Not Found"); abort();
    return 0;

}//for//


inline
void RoadSegmentState::GetFirstVehicleIndex(
     const unsigned int laneNumber,
     const bool isGoingInForwardDirection,
     bool& wasFound,
     unsigned int& vehicleIndex) const
{
    wasFound = false;

    if (isGoingInForwardDirection) {
        for(unsigned int i = 0; (i < vehicles.size()); i++) {
            if (vehicles[i]->GetCurrentPosition().laneNumber == laneNumber) {
                vehicleIndex = i;
                wasFound = true;
                return;
            }//if//
        }//for//
    }
    else {
        for(int i = (static_cast<int>(vehicles.size()) - 1); (i >= 0); i--) {
            if (vehicles[i]->GetCurrentPosition().laneNumber == laneNumber) {
                vehicleIndex = static_cast<unsigned int>(i);
                wasFound = true;
                return;
            }//if//
        }//for//
    }//for//

}//GetFirstVehicleIndex//


inline
void RoadSegmentState::GetNextVehicleIndex(
    const unsigned int vehicleIndex,
    const unsigned int laneNumber,
    const bool isGoingForward,
    bool& wasFound,
    unsigned int& nextVehicleIndex) const
{
    wasFound = false;

    if (isGoingForward) {
        for(unsigned int i = (vehicleIndex + 1); (i < vehicles.size()); i++) {
            if (vehicles[i]->GetCurrentPosition().laneNumber == laneNumber) {
                nextVehicleIndex = i;
                wasFound = true;
                return;
            }//if//
        }//for//
    }
    else {
        for(int i = (static_cast<int>(vehicleIndex) - 1); (i >= 0); i--) {
            if (vehicles[i]->GetCurrentPosition().laneNumber == laneNumber) {
                nextVehicleIndex = static_cast<unsigned int>(i);
                wasFound = true;
                return;
            }//if//
        }//for//
    }//for//

}//GetNextVehicleIndex//


inline
void RoadSegmentState::GetPreviousVehicleIndex(
    const unsigned int vehicleIndex,
    const unsigned int laneNumber,
    const bool isGoingForward,
    bool& wasFound,
    unsigned int& previousVehicleIndex) const
{
    wasFound = false;

    if (!isGoingForward) {
        for(unsigned int i = (vehicleIndex + 1); (i < vehicles.size()); i++) {
            if (vehicles[i]->GetCurrentPosition().laneNumber == laneNumber) {
                previousVehicleIndex = i;
                wasFound = true;
            }//if//
        }//for//
    }
    else {
        for(int i = (static_cast<int>(vehicleIndex) - 1); (i >= 0); i--) {
            if (vehicles[i]->GetCurrentPosition().laneNumber == laneNumber) {
                previousVehicleIndex = static_cast<unsigned int>(i);
                wasFound = true;
            }//if//
        }//for//
    }//for//

}//GetPreviousVehicleIndex//





inline
void RoadSegmentState::UpdateVehicleOrder()
{
    struct Comparitor {
        bool operator()(const shared_ptr<Vehicle>& left, const shared_ptr<Vehicle>& right) const
        {
            return (left->GetCurrentPosition() < right->GetCurrentPosition());
        }
    };

    std::sort(vehicles.begin(), vehicles.end(), Comparitor());

}//UpdateVehicleOrder//



inline
void RoadSegmentState::DeleteVehicleFromRoad(const Vehicle::VehicleIdType& vehicleId)
{
    for(unsigned int i = 0; (i < vehicles.size()); i++) {
        if (vehicles[i]->GetVehicleId() == vehicleId) {
            vehicles.erase(vehicles.begin() + i);
            return;
        }//if//
    }//for//

    assert(false && "Vehicle Was Not Found"); abort();
}






//--------------------------------------------------------------------------------------------------

inline
bool AutoRoadNetwork::ThereIsAConflictingIntersectionCrossTraffic(
    const vector<RoadSegmentState>& roadStates,
    const RoadSegmentIdType& currentRoadId,
    const unsigned int currentLaneNumber,
    const RoadSegmentIdType& nextRoadId,
    const TimeType& neededConflictFreeTime) const
    //Future?-Optimiz: TimeType& earliestConflictTime,
    //Future?-Optimiz: TimeType& earliestPossibleConflictFreeTime)
{
    using std::min;


    const RoadSegment& road = GetRoadSegment(currentRoadId);
    const RoadSegment::ConnectionInfoType& connectionInfo = road.GetConnectionInfo(nextRoadId);

    assert(false && "Reminder: intersectionId must be set");

    const IntersectionInfoType& intersectionInfo = intersections.at(connectionInfo.intersectionId);

    const vector<bool>& roadIsConflictingWithConnection =
        intersectionInfo.externalRoadConflictTable.at(
            connectionInfo.intersectionLanes.front().intersectionLaneRoadId);

    for(unsigned int i = 0; (i < intersectionInfo.externalRoadIds.size()); i++) {

        const RoadSegmentIdAndWhichEndType& otherRoadInfo = intersectionInfo.externalRoadIds[i];

        if (!roadIsConflictingWithConnection.at(otherRoadInfo.roadId)) {
            continue;
        }//if//

        const RoadSegment& otherRoad = GetRoadSegment(otherRoadInfo.roadId);
        const RoadSegmentState& otherRoadState = roadStates[otherRoadInfo.roadId];

        const double searchDistanceMeters =
            otherRoad.GetSpeedLimitMetersSec() * ConvertTimeToDoubleSecs(neededConflictFreeTime);


        if (otherRoadInfo.isAtFrontOfRoad) {
            for(unsigned int i = 0; (i < otherRoadState.GetNumberVehicles()); i++) {
                const Vehicle& aVehicle = otherRoadState.GetVehicle(i);

                const double distanceToIntersectionMeters =
                     aVehicle.GetCurrentPosition().positionInRoadMeters;

                if (distanceToIntersectionMeters > searchDistanceMeters) {
                    break;
                }//if//

                if ((!otherRoad.IsAForwardLane(aVehicle.GetCurrentPosition().laneNumber)) &&
                    (distanceToIntersectionMeters < searchDistanceMeters)) {

                    return (true);
                }//if//
            }//for//
        }
        else {
            for(int i = (otherRoadState.GetNumberVehicles()-1); (i >= 0); i--) {
                const Vehicle& aVehicle = otherRoadState.GetVehicle(static_cast<unsigned int>(i));

                const double distanceToIntersectionMeters =
                    (otherRoad.GetLengthMeters() -
                     aVehicle.GetCurrentPosition().positionInRoadMeters);


                if (distanceToIntersectionMeters > searchDistanceMeters) {
                    break;
                }//if//

                if ((otherRoad.IsAForwardLane(aVehicle.GetCurrentPosition().laneNumber)) &&
                    (distanceToIntersectionMeters < searchDistanceMeters)) {

                    return (true);
                }//if//
            }//for//
        }//if//
    }//for//

    return (false);

}//ThereIsAConflictingIntersectionCrossTraffic//



//--------------------------------------------------------------------------------------------------


inline
bool ThereIsEnoughSpaceInNextRoad(
    const double& vehicleGapMeters,
    const double& vehicleLengthMeters,
    const AutoRoadNetwork& roadNetwork,
    const vector<RoadSegmentState>& roadStates,
    const RoadSegmentIdType& currentRoadId,
    const unsigned int currentLaneNumber,
    const RoadSegmentIdType& nextRoadId)
{
    const RoadSegment& road = roadNetwork.GetRoadSegment(currentRoadId);
    const RoadSegment::ConnectionInfoType& connectionInfo = road.GetConnectionInfo(nextRoadId);

    const RoadSegment::ConnectionLaneInfoType& connectionLaneInfo =
        connectionInfo.GetLaneInfo(currentLaneNumber);

    if (connectionLaneInfo.intersectionLaneRoadId == InvalidRoadSegmentId) {
        // No turn lane exists (zero size intersection).
        return (true);
    }//if//

    const RoadSegmentState& intersectionLaneState = roadStates[connectionLaneInfo.intersectionLaneRoadId];

    double neededMeters = (vehicleGapMeters + vehicleLengthMeters);

    // For left (right-hand traffic system) turners in the intersection.

    for(unsigned int i = 0; (i < intersectionLaneState.GetNumberVehicles()); i++) {
        neededMeters += intersectionLaneState.GetVehicle(i).GetLengthMeters() + vehicleGapMeters;
    }//for//

    const RoadSegment& nextRoad = roadNetwork.GetRoadSegment(connectionInfo.nextRoadId);
    const RoadSegmentState& nextRoadState = roadStates[connectionInfo.nextRoadId];

    bool wasFound;
    unsigned int vehicleIndex;

    const bool isGoingForwardOnNextRoad = nextRoad.IsAForwardLane(connectionLaneInfo.nextRoadLaneNum);

    nextRoadState.GetFirstVehicleIndex(
        connectionLaneInfo.nextRoadLaneNum,
        isGoingForwardOnNextRoad,
        wasFound,
        vehicleIndex);

    if (!wasFound) {
        // Assuming non-tiny length roads (if tiny)
        return (neededMeters <= nextRoad.GetLengthMeters());
    }//if//

    const Vehicle& aVehicle = nextRoadState.GetVehicle(vehicleIndex);

    double availableSpaceMeters = 0.0;

    if (isGoingForwardOnNextRoad) {
        availableSpaceMeters = aVehicle.GetCurrentPosition().positionInRoadMeters;
    }
    else {
        availableSpaceMeters =
            (nextRoad.GetLengthMeters() - aVehicle.GetCurrentPosition().positionInRoadMeters);
    }//if//

    availableSpaceMeters -= (aVehicle.GetLengthMeters() / 2.0);

    return (availableSpaceMeters >= neededMeters);

}//ThereIsEnoughSpaceInNextRoad//



inline
double CalcBrakingDistanceMeters(
    const double& brakingDeacceleration,
    const double& startVelocity,
    const double& endVelocity)
{
    assert(brakingDeacceleration > 0.0);
    assert(startVelocity >= endVelocity);

    const double time = (startVelocity - endVelocity) / brakingDeacceleration;
    assert(time >= 0.0);

    return (time * ((startVelocity + endVelocity) / 2.0));
}


// Note "obsticleDistanceMeters" is "bumper to bumper" or "bumper to line" distance.
// (as opposed to vehicle center distance).

inline
void CalcLeadingVehicleOrObstacleDistanceAndVelocity(
    const TimeType& currentTime,
    const AutoRoadNetwork& roads,
    const vector<RoadSegmentState>& roadStates,
    const Vehicle& aVehicle,
    const double& brakingDeaccelerationMetersSec,
    const double& vehicleGapMeters,
    const double& searchDistanceCutoffMeters,
    double& obstacleDistanceMeters,
    double& obstacleVelocityMetersSec,
    bool& obstacleDoesNotNeedAVehicleGap,
    double& speedConstraintDistanceMeters,
    double& speedConstraintVelocityMetersSec)
{
    using std::max;

    obstacleDistanceMeters = DBL_MAX;
    obstacleVelocityMetersSec = 0.0;
    obstacleDoesNotNeedAVehicleGap = false;
    speedConstraintDistanceMeters = DBL_MAX;
    speedConstraintVelocityMetersSec = DBL_MAX;

    const PositionInRoadType& position = aVehicle.GetCurrentPosition();
    const RoadSegmentIdType roadSegmentId = aVehicle.GetCurrentRoadId();
    const RoadSegmentState& roadState = roadStates.at(roadSegmentId);
    const unsigned int vehicleIndex = roadState.GetVehicleIndex(aVehicle.GetVehicleId());
    const RoadSegment& roadSegment = roads.GetRoadSegment(roadSegmentId);
    const bool isGoingForward = roadSegment.IsAForwardLane(position.laneNumber);
    const bool currentlyInAnIntersection = roadSegment.GetIsAnIntersectionLanePseudoRoad();


    // Need to look ahead at least this far:

    const double currentStoppingDistance =
        CalcBrakingDistanceMeters(
            brakingDeaccelerationMetersSec,
            aVehicle.GetCurrentVelocity(),
            0.0);

    // Already at destination road special case:

    if (aVehicle.GetCurrentRoadId() == aVehicle.GetDestinationRoadId()) {

        // Check for stopping at destination (vehicle center).

        const double distanceLeftMeters =
            std::abs(
                aVehicle.GetDestinationPosition().positionInRoadMeters -
                aVehicle.GetCurrentPosition().positionInRoadMeters);

        if (distanceLeftMeters <= currentStoppingDistance) {

            // Front of car is past the destination.

            obstacleDistanceMeters = distanceLeftMeters;
            obstacleVelocityMetersSec = 0.0;
            obstacleDoesNotNeedAVehicleGap = true;
            return;

        }//if//
    }//if//

    bool leadingVehicleWasFound;
    unsigned int leadingVehicleIndex;

    roadState.GetNextVehicleIndex(
        vehicleIndex,
        position.laneNumber,
        isGoingForward,
        leadingVehicleWasFound,
        leadingVehicleIndex);

    if (leadingVehicleWasFound) {
        const Vehicle& leadingVehicle = roadState.GetVehicle(leadingVehicleIndex);

        const double centersDifferenceMeters =
            std::abs(
                position.positionInRoadMeters - leadingVehicle.GetCurrentPosition().positionInRoadMeters);

        const double bumperToBumperAdjustmentMeters =
            (aVehicle.GetHalfLengthMeters() + leadingVehicle.GetHalfLengthMeters());

        obstacleDistanceMeters = max(0.0, (centersDifferenceMeters - bumperToBumperAdjustmentMeters));
        obstacleVelocityMetersSec = leadingVehicle.GetCurrentVelocity();

        // Keep searching route for stops and speed limits.

    }//if//

    double totalDistanceSoFar = 0.0;
    if (isGoingForward) {
        totalDistanceSoFar = (roadSegment.GetLengthMeters() - position.positionInRoadMeters);
    }
    else {
        totalDistanceSoFar = position.positionInRoadMeters;
    }//if//


    // Search forward on route, looking for stops, speed limits and leading vehicle.
    // Support (anomalous) very short road segments.

    const deque<RoadSegmentIdType>& currentRoute = aVehicle.GetCurrentRoute();

    RoadSegmentIdType lastRoadId = roadSegmentId;
    unsigned int lastLaneNumber = position.laneNumber;

    for (unsigned int i = 1; (i < currentRoute.size()); i++) {

        if ((totalDistanceSoFar > searchDistanceCutoffMeters) ||
            ((leadingVehicleWasFound) && (totalDistanceSoFar > currentStoppingDistance))) {

            // Give up search.  Either maximum distance exceeded or found a leading vehicle
            // and any forced stops can be handled later.

            return;
        }//if//

        const RoadSegment& lastRoadSegment = roads.GetRoadSegment(lastRoadId);
        const RoadSegmentIdType nextRoadId = currentRoute[i];

        const RoadSegment::ConnectionInfoType& connectionInfo =
            lastRoadSegment.GetConnectionInfo(nextRoadId);

        if (!connectionInfo.IsOneOfTheLanes(lastLaneNumber)) {

            // Not in correct lane (lane change needed).

            obstacleDistanceMeters = totalDistanceSoFar - aVehicle.GetHalfLengthMeters();
            obstacleVelocityMetersSec = 0.0;
            obstacleDoesNotNeedAVehicleGap = true;
            return;
        }//if//

        const RoadSegment::ConnectionLaneInfoType& laneInfo = connectionInfo.GetLaneInfo(lastLaneNumber);

        if ((!lastRoadSegment.GetIsAnIntersectionLanePseudoRoad()) &&
            (laneInfo.intersectionLaneRoadId != InvalidRoadSegmentId)) {

            if ((speedConstraintDistanceMeters == DBL_MAX) &&
                (connectionInfo.speedLimitMetersSec < roadSegment.GetSpeedLimitMetersSec())) {

                speedConstraintDistanceMeters = totalDistanceSoFar;
                speedConstraintVelocityMetersSec = connectionInfo.speedLimitMetersSec;

            }//if//

            const TrafficLightColorType Green = TrafficLightColorType::Green;

            if ((connectionInfo.trafficLightPtr != nullptr) &&
                (connectionInfo.trafficLightPtr->GetCurrentColor(currentTime) != Green)) {

                obstacleDistanceMeters = max(0.0, (totalDistanceSoFar - aVehicle.GetHalfLengthMeters()));
                obstacleVelocityMetersSec = 0.0;
                obstacleDoesNotNeedAVehicleGap = true;
                return;
            }//if//

            if (!ThereIsEnoughSpaceInNextRoad(
                    vehicleGapMeters,
                    aVehicle.GetLengthMeters(),
                    roads,
                    roadStates,
                    lastRoadId,
                    lastLaneNumber,
                    nextRoadId)) {

                // Treat as a red light.

                obstacleDistanceMeters = max(0.0, (totalDistanceSoFar - aVehicle.GetHalfLengthMeters()));
                obstacleVelocityMetersSec = 0.0;
                obstacleDoesNotNeedAVehicleGap = true;
                return;
            }//if//

            if (!leadingVehicleWasFound) {

                // Check Intersection Lane for vehicles.

                const RoadSegmentState& laneState = roadStates.at(laneInfo.intersectionLaneRoadId);

                laneState.GetFirstVehicleIndex(
                    0,
                    true,
                    leadingVehicleWasFound,
                    leadingVehicleIndex);

                if (leadingVehicleWasFound) {
                    const Vehicle& leadingVehicle = roadState.GetVehicle(leadingVehicleIndex);

                    const double bumperToBumperAdjustmentMeters =
                        (aVehicle.GetHalfLengthMeters() + leadingVehicle.GetHalfLengthMeters());

                    obstacleDistanceMeters =
                        max(0.0,
                            ((totalDistanceSoFar +
                             leadingVehicle.GetCurrentPosition().positionInRoadMeters) -
                             bumperToBumperAdjustmentMeters));

                    obstacleVelocityMetersSec = leadingVehicle.GetCurrentVelocity();
                    return;

                }//if//
            }//if//

            totalDistanceSoFar +=
                roads.GetRoadSegment(laneInfo.intersectionLaneRoadId).GetLengthMeters();

            if ((totalDistanceSoFar > searchDistanceCutoffMeters) ||
                ((leadingVehicleWasFound) && (totalDistanceSoFar > currentStoppingDistance))) {

                // Give up search.

                return;
            }//if//
        }//if//


        // Going forward after intersection.

        const RoadSegment& nextRoadSegment = roads.GetRoadSegment(nextRoadId);
        const bool isGoingInForwardDirection = nextRoadSegment.IsAForwardLane(laneInfo.nextRoadLaneNum);

        if ((speedConstraintDistanceMeters == DBL_MAX) &&
            (nextRoadSegment.GetSpeedLimitMetersSec() < roadSegment.GetSpeedLimitMetersSec())) {

            speedConstraintDistanceMeters = totalDistanceSoFar;
            speedConstraintVelocityMetersSec = nextRoadSegment.GetSpeedLimitMetersSec();

        }//if//

        if (nextRoadId == aVehicle.GetDestinationRoadId()) {

            // Check for stopping at destination.

            double distanceLeftMeters = totalDistanceSoFar;
            if (isGoingInForwardDirection) {
                distanceLeftMeters += aVehicle.GetDestinationPosition().positionInRoadMeters;
            }
            else {
                distanceLeftMeters +=
                    (nextRoadSegment.GetLengthMeters() - aVehicle.GetDestinationPosition().positionInRoadMeters);
            }//if//

            if (distanceLeftMeters <= currentStoppingDistance) {

                // Front of car goes past destination point.

                obstacleDistanceMeters = distanceLeftMeters;
                obstacleVelocityMetersSec = 0.0;
                obstacleDoesNotNeedAVehicleGap = true;
                return;

            }//if//
        }//if//

        if (!leadingVehicleWasFound) {

            roadState.GetFirstVehicleIndex(
                laneInfo.nextRoadLaneNum,
                isGoingInForwardDirection,
                leadingVehicleWasFound,
                leadingVehicleIndex);

            if (leadingVehicleWasFound) {
                const Vehicle& leadingVehicle = roadState.GetVehicle(leadingVehicleIndex);

                const double bumperToBumperAdjustmentMeters =
                    (aVehicle.GetHalfLengthMeters() + leadingVehicle.GetHalfLengthMeters());

                if (isGoingInForwardDirection) {
                    obstacleDistanceMeters =
                        totalDistanceSoFar + leadingVehicle.GetCurrentPosition().positionInRoadMeters;
                }
                else {
                    obstacleDistanceMeters =
                        totalDistanceSoFar +
                        (nextRoadSegment.GetLengthMeters() -
                         leadingVehicle.GetCurrentPosition().positionInRoadMeters);
                }//if//

                obstacleDistanceMeters = (obstacleDistanceMeters - bumperToBumperAdjustmentMeters);
                assert(obstacleDistanceMeters >= 0.0);
                obstacleVelocityMetersSec = leadingVehicle.GetCurrentVelocity();
                return;

            }//if//
        }//if//

        totalDistanceSoFar += lastRoadSegment.GetLengthMeters();
        lastRoadId = nextRoadId;
        lastLaneNumber = laneInfo.nextRoadLaneNum;

    }//for//

}//CalcLeadingVehicleOrObstacleDistanceAndVelocity//




//-----------------------------------------------


const double CloseEnoughToDestinationDistanceMeters = 1.0;


inline
void UpdateVehiclePosition(
    const double& stepDurationSecs,
    const AutoRoadNetwork& roads,
    const double& accelerationMetersSec,
    const double& maxVelocityMetersSec,
    const int laneChangeDelta,
    Vehicle& aVehicle)
{
    using std::min;
    using std::max;

    if (aVehicle.GetCurrentRoadId() == aVehicle.GetDestinationRoadId()) {
        const double distanceFromDestinationMeters =
            std::abs(
                (aVehicle.GetCurrentPosition().positionInRoadMeters -
                 aVehicle.GetDestinationPosition().positionInRoadMeters));

        if (distanceFromDestinationMeters < CloseEnoughToDestinationDistanceMeters) {
            const RoadSegment& road = roads.GetRoadSegment(aVehicle.GetDestinationRoadId());

            TwoDVector xyPosition;
            double azimuthClockwiseRad;

            road.GetXyPositionAndAzimuth(
                aVehicle.GetDestinationPosition(),
                xyPosition,
                azimuthClockwiseRad);

            aVehicle.SetNextPosition(
                aVehicle.GetDestinationPosition(),
                xyPosition,
                azimuthClockwiseRad);

            aVehicle.SetNextVelocity(0.0);
            aVehicle.ClearCurrentRoute();

            // Making Vehicle disappear from model.

            aVehicle.SetNextRoadId(InvalidRoadSegmentId);
            return;
        }//if//
    }//if//

    const double currentVelocity = aVehicle.GetCurrentVelocity();
    const RoadSegment& road = roads.GetRoadSegment(aVehicle.GetCurrentRoadId());
    PositionInRoadType position = aVehicle.GetCurrentPosition();

    double nextVelocity;
    double distanceMeters;

    if (laneChangeDelta == 0) {
        nextVelocity =
            max(0.0,
                min(maxVelocityMetersSec, (currentVelocity + (accelerationMetersSec * stepDurationSecs))));

        distanceMeters =  stepDurationSecs * ((currentVelocity + nextVelocity) / 2.0);
    }
    else {
        // Stay at current velocity for lane change.

        nextVelocity = currentVelocity;
        distanceMeters = stepDurationSecs * currentVelocity;

        const bool isForwardLane = road.IsAForwardLane(position.laneNumber);
        position.laneNumber += laneChangeDelta;
        assert((isForwardLane == road.IsAForwardLane(position.laneNumber)) && "Bad lane change");

    }//if//

    aVehicle.SetNextVelocity(nextVelocity);

    if (road.IsAForwardLane(position.laneNumber)) {
        if (distanceMeters <= (road.GetLengthMeters() - position.positionInRoadMeters)) {
            position.positionInRoadMeters += distanceMeters;

            TwoDVector xyPosition;
            double azimuthClockwiseRad;
            road.GetXyPositionAndAzimuth(
                position,
                xyPosition,
                azimuthClockwiseRad);

            aVehicle.SetNextPosition(
                position,
                xyPosition,
                azimuthClockwiseRad);

            return;
        }//if//

        distanceMeters -= (road.GetLengthMeters() - position.positionInRoadMeters);
    }
    else {
        if (distanceMeters <= position.positionInRoadMeters) {
            position.positionInRoadMeters -= distanceMeters;

            TwoDVector xyPosition;
            double azimuthClockwiseRad;
            road.GetXyPositionAndAzimuth(
                position,
                xyPosition,
                azimuthClockwiseRad);

            aVehicle.SetNextPosition(
                position,
                xyPosition,
                azimuthClockwiseRad);

            return;
        }//if//

        distanceMeters -= position.positionInRoadMeters;

    }//if//

    // Moving to next road.

    assert(distanceMeters > 0.0);

    while (distanceMeters > 0.0) {
        RoadSegmentIdType nextRoadId;

        if (road.GetIsAnIntersectionLanePseudoRoad()) {

            // Leave Intersection.

            aVehicle.DeleteFrontOfCurrentRoute();
            nextRoadId = aVehicle.GetCurrentRoute().front();
        }
        else {
            // Enter Intersection, if it is exists.

            assert(aVehicle.GetCurrentRoute().size() > 1);

            const RoadSegment::ConnectionInfoType& connection =
                road.GetConnectionInfo(aVehicle.GetCurrentRoute()[1]);

            nextRoadId = connection.GetLaneInfo(position.laneNumber).intersectionLaneRoadId;
            if (nextRoadId == InvalidRoadSegmentId) {

                // Zero size intersection => Go to next road.

                aVehicle.DeleteFrontOfCurrentRoute();
                nextRoadId = aVehicle.GetCurrentRoute().front();
            }//if//

        }//if//

        const RoadSegment& nextRoad = roads.GetRoadSegment(nextRoadId);
        if (nextRoad.GetIsAnIntersectionLanePseudoRoad()) {
           position = PositionInRoadType(0.0, 0);

        }
        else {
            position =
                roads.GetClosestConnectingRoadPosition(
                    aVehicle.GetCurrentRoadId(),
                    position.laneNumber,
                    nextRoadId);
        }//if//

        if (position.positionInRoadMeters == 0.0) {
            position.positionInRoadMeters = min(distanceMeters, nextRoad.GetLengthMeters());
        }
        else {
            position.positionInRoadMeters = max((nextRoad.GetLengthMeters() - distanceMeters), 0.0);
        }//if//

        aVehicle.SetNextRoadId(nextRoadId);

        TwoDVector xyPosition;
        double azimuthClockwiseRad;
        nextRoad.GetXyPositionAndAzimuth(
            position,
            xyPosition,
            azimuthClockwiseRad);

        aVehicle.SetNextPosition(
            position,
            xyPosition,
            azimuthClockwiseRad);

        distanceMeters -= nextRoad.GetLengthMeters();
    }//while//

}//UpdateVehiclePosition//



inline
bool LaneChangeIsSafe(
    const double& minGapMeters,
    const double& timeHeadwaySecs,
    const double& vehicleVelocityMetersSec,
    const double& maxBrakingAccelerationMetersSec,
    const double& frontGapMeters,
    const double& frontVehicleVelocityMetersSec,
    const double& tailGapMeters,
    const double& tailVehicleVelocityMetersSec,
    const double& tailVehicleMaxBrakingAccelerationMetersSec)
{
    using std::max;

    if ((frontGapMeters < minGapMeters) || (tailGapMeters < minGapMeters)) {

        return false;
    }//if//

    if (tailGapMeters == DBL_MAX) {
        return true;
    }//if//

    const double maxVelocityMetersSec =
        max(vehicleVelocityMetersSec, max(frontVehicleVelocityMetersSec, tailVehicleVelocityMetersSec));

    const double tailVehicleAcceleration =
        CalculateIntelligentDriversModelAcceleration(
            maxVelocityMetersSec,
            tailVehicleMaxBrakingAccelerationMetersSec,
            tailVehicleMaxBrakingAccelerationMetersSec,
            minGapMeters,
            timeHeadwaySecs,
            tailVehicleVelocityMetersSec,
            tailGapMeters,
            vehicleVelocityMetersSec);

    if (tailVehicleAcceleration < -tailVehicleMaxBrakingAccelerationMetersSec) {
        return false;
    }//if//

    const double vehicleAcceleration =
        CalculateIntelligentDriversModelAcceleration(
            maxVelocityMetersSec,
            maxBrakingAccelerationMetersSec,
            maxBrakingAccelerationMetersSec,
            minGapMeters,
            timeHeadwaySecs,
            vehicleVelocityMetersSec,
            frontGapMeters,
            frontVehicleVelocityMetersSec);

    if (vehicleAcceleration < -maxBrakingAccelerationMetersSec) {
        return false;
    }//if//

    return true;

}//LaneChangeIsSafe//



inline
void AutoRoadNetwork::FindRoadAndLaneGoingBackwards(
    const RoadSegmentIdType roadId,
    const unsigned int laneNumber,
    bool& wasFound,
    RoadSegmentIdType& backwardsRoadId,
    bool& backwardsRoadIsConnectionAtFront,
    unsigned int& backwardsLaneNum) const
{
    wasFound = false;

    const RoadSegment& road = roadSegments.at(roadId);
    assert(!road.GetIsAnIntersectionLanePseudoRoad());
    const bool connectionIsAtFront = road.IsAForwardLane(laneNumber);
    const IntersectionIdType intersectionId = road.GetIntersectionId(connectionIsAtFront);
    const IntersectionInfoType& intersection = intersections[intersectionId];

    for(unsigned int i = 0; (i < intersection.externalRoadIds.size()); i++) {
        const RoadSegmentIdAndWhichEndType& otherRoadInfo = intersection.externalRoadIds[i];
        if (otherRoadInfo.roadId != roadId) {
            const RoadSegment& otherRoad = roadSegments[otherRoadInfo.roadId];

            if (otherRoad.ConnectionExists(roadId, otherRoadInfo.isAtFrontOfRoad)) {
                const RoadSegment::ConnectionInfoType& connection =
                    otherRoad.GetConnectionInfo(roadId, otherRoadInfo.isAtFrontOfRoad);

                if (connection.turnDirection == RoadTurnDirectionType::Straight) {
                    if (connection.IsOneOfTheDestinationLanes(laneNumber)) {
                        const RoadSegment::ConnectionLaneInfoType& laneInfo =
                            connection.GetLaneInfoForDestinationLane(laneNumber);

                        wasFound = true;
                        backwardsRoadId = otherRoadInfo.roadId;
                        backwardsRoadIsConnectionAtFront = otherRoadInfo.isAtFrontOfRoad;
                        backwardsLaneNum = laneInfo.outgoingLaneNum;
                    }//if//

                    break;
                }//if//
            }//if//
        }//if//
    }//for//

}//FindRoadAndLaneGoingBackwards//



inline
void MakeLaneChangeDecision(
    const TimeType& currentTime,
    const double& timeStepDurationSecs,
    const IntelligentDriverModelParameters& idmParams,
    const AutoRoadNetwork& roads,
    const vector<RoadSegmentState>& roadStates,
    const Vehicle& aVehicle,
    int& laneChangeDelta,
    double& forcedLaneChangeStopDistanceMeters)
{
    using std::max;

    laneChangeDelta = 0;
    forcedLaneChangeStopDistanceMeters = DBL_MAX;

    const deque<RoadSegmentIdType>& route = aVehicle.GetCurrentRoute();

    assert(!route.empty());

    const RoadSegmentIdType currentRoadId = aVehicle.GetCurrentRoadId();
    const RoadSegment& currentRoad = roads.GetRoadSegment(currentRoadId);
    const unsigned int currentLane = aVehicle.GetCurrentPosition().laneNumber;

    unsigned int nextLane = currentLane;
    unsigned int totalNumberLaneChangesNeeded = 0;

    if (currentRoadId == aVehicle.GetDestinationRoadId()) {
        assert(route.size() == 1);

        const unsigned int desiredLaneNum = aVehicle.GetDestinationPosition().laneNumber;
        if (currentLane == desiredLaneNum) {
            return;
        }//if//

        if (currentLane < desiredLaneNum) {
            nextLane = currentLane + 1;
            totalNumberLaneChangesNeeded = desiredLaneNum - currentLane;
        }
        else {
            nextLane = currentLane - 1;
            totalNumberLaneChangesNeeded = currentLane - desiredLaneNum;

        }//if//
    }
    else {

        if (currentRoad.GetIsAnIntersectionLanePseudoRoad()) {

            // No lane change allowed in intersection.

            return;
        }//if//

        assert(route.front() == currentRoadId);
        const RoadSegmentIdType nextRoadId = route[1];

        const RoadSegment::ConnectionInfoType& closestConnectionInfo =
            currentRoad.GetConnectionInfo(nextRoadId);

        vector<unsigned int> desiredLaneNumbers;

        if ((closestConnectionInfo.isAContinuationIntersection) && (route.size() > 2)) {

            // Not a full blown intersection (road expansion). Must search further.

            const RoadSegmentIdType afterNextRoadId = route[2];
            const RoadSegment& nextRoad = roads.GetRoadSegment(nextRoadId);
            const RoadSegment::ConnectionInfoType& afterNextConnectionInfo =
                nextRoad.GetConnectionInfo(afterNextRoadId);

            assert((!afterNextConnectionInfo.isAContinuationIntersection) &&
                   "No consecutive continuation intersections in a row (restriction)");

            // Find lane(s) that continue directly to the lane in the real intersection.

            for (unsigned int i = 0; (i < closestConnectionInfo.intersectionLanes.size()); i++) {
                if (afterNextConnectionInfo.IsOneOfTheLanes(
                    closestConnectionInfo.intersectionLanes[i].nextRoadLaneNum)) {

                    desiredLaneNumbers.push_back(
                        closestConnectionInfo.intersectionLanes[i].outgoingLaneNum);
                }//if//
            }//for//

            if (desiredLaneNumbers.empty()) {
                // Find closest lane(s) that continue directly to the lane in the real intersection.

                unsigned int minLaneDifference = UINT_MAX;
                unsigned int closestLaneIndex = 0;

                for (unsigned int i = 0; (i < closestConnectionInfo.intersectionLanes.size()); i++) {
                    for(unsigned int j = 0; (j < afterNextConnectionInfo.intersectionLanes.size()); j++) {
                        const unsigned int laneDifference = static_cast<unsigned int>(
                            std::abs(
                                static_cast<int>(
                                    closestConnectionInfo.intersectionLanes[i].nextRoadLaneNum) -
                                static_cast<int>(
                                    afterNextConnectionInfo.intersectionLanes[j].outgoingLaneNum)));

                        if (laneDifference < minLaneDifference) {
                            minLaneDifference = laneDifference;
                            closestLaneIndex = i;
                        }//if//
                    }//for//
                }//for//

                desiredLaneNumbers.push_back(
                    closestConnectionInfo.intersectionLanes[closestLaneIndex].outgoingLaneNum);

            }//if//
        }
        else {
            desiredLaneNumbers.resize(closestConnectionInfo.intersectionLanes.size());
            for(unsigned int i = 0; (i < closestConnectionInfo.intersectionLanes.size()); i++) {
                desiredLaneNumbers[i] = closestConnectionInfo.intersectionLanes[i].outgoingLaneNum;
            }//for//
        }//if//

        // Find closest Lane in connection to current lane.

        unsigned int closestLaneIndex = 0;
        if (desiredLaneNumbers.size() > 1) {
            unsigned int minLaneDifference = UINT_MAX;
            for(unsigned int i = 0; (i < desiredLaneNumbers.size()); i++) {
                const unsigned int laneDifference = static_cast<unsigned int>(
                    std::abs(
                        static_cast<int>(desiredLaneNumbers[i]) -
                        static_cast<int>(currentLane)));

                if (laneDifference < minLaneDifference) {
                    minLaneDifference = laneDifference;
                    closestLaneIndex = i;
                }//if//
            }//for//
        }//if//

        const unsigned int desiredLaneNum = desiredLaneNumbers[closestLaneIndex];

        if (desiredLaneNum > currentLane) {
            nextLane = currentLane + 1;
            totalNumberLaneChangesNeeded = (desiredLaneNum - currentLane);
        }
        else if (desiredLaneNum < currentLane) {
            nextLane = currentLane - 1;
            totalNumberLaneChangesNeeded = (currentLane - desiredLaneNum);
        }//if//
    }//if//


    if (nextLane != currentLane) {
        const double distanceToNextIntersectionMeters =
            currentRoad.CalcDistanceLeftMeters(aVehicle.GetCurrentPosition());

        forcedLaneChangeStopDistanceMeters =
            max(0.0,
                (distanceToNextIntersectionMeters -
                 (idmParams.forcedLaneChangeStopDistancePerLaneMeters * totalNumberLaneChangesNeeded)));
    }//if//


    // Optional Lane change logic here ...

    if (nextLane == currentLane) {
        return;
    }//if//

    const RoadSegmentState& roadState = roadStates.at(currentRoadId);

    const unsigned int vehicleIndex = roadState.GetVehicleIndex(aVehicle.GetVehicleId());

    const bool isGoingForward = currentRoad.IsAForwardLane(nextLane);

    bool nextVehicleWasFound;
    unsigned int nextVehicleIndex;

    roadState.GetNextVehicleIndex(
        vehicleIndex,
        nextLane,
        isGoingForward,
        nextVehicleWasFound,
        nextVehicleIndex);

    double frontSpaceMeters;
    double frontVelocityMetersSec;

    if (nextVehicleWasFound) {
        const Vehicle& frontVehicle = roadState.GetVehicle(nextVehicleIndex);

        frontVelocityMetersSec = frontVehicle.GetCurrentVelocity();

        frontSpaceMeters =
                std::abs(
                    (frontVehicle.GetCurrentPosition().positionInRoadMeters -
                     aVehicle.GetCurrentPosition().positionInRoadMeters));

        frontSpaceMeters -= (frontVehicle.GetLengthMeters() / 2.0);
    }
    else {
        frontVelocityMetersSec = currentRoad.GetSpeedLimitMetersSec();

        if (isGoingForward) {
            frontSpaceMeters =
                (currentRoad.GetLengthMeters() - aVehicle.GetCurrentPosition().positionInRoadMeters);
        }
        else {
            frontSpaceMeters = aVehicle.GetCurrentPosition().positionInRoadMeters;
        }//if//
    }//if//

    frontSpaceMeters -= (aVehicle.GetLengthMeters() / 2.0);

    if (frontSpaceMeters < 0.0) {
        frontSpaceMeters = 0.0;
    }//if//

    bool previousVehicleWasFound;
    unsigned int previousVehicleIndex;

    roadState.GetPreviousVehicleIndex(
        vehicleIndex,
        nextLane,
        isGoingForward,
        previousVehicleWasFound,
        previousVehicleIndex);

    double tailSpaceMeters = DBL_MAX;
    double tailVelocityMetersSec = 0.0;

    if (previousVehicleWasFound) {

        const Vehicle& tailVehicle = roadState.GetVehicle(previousVehicleIndex);
        tailVelocityMetersSec = tailVehicle.GetCurrentVelocity();

        tailSpaceMeters =
                std::abs(
                    (tailVehicle.GetCurrentPosition().positionInRoadMeters -
                     aVehicle.GetCurrentPosition().positionInRoadMeters));

        tailSpaceMeters -=
            ((tailVehicle.GetLengthMeters() + aVehicle.GetLengthMeters()) / 2.0);
    }
    else {
        double distanceSoFar;

        if (!isGoingForward) {
            distanceSoFar =
                (currentRoad.GetLengthMeters() - aVehicle.GetCurrentPosition().positionInRoadMeters);
        }
        else {
            distanceSoFar = aVehicle.GetCurrentPosition().positionInRoadMeters;
        }//if//

        const double maxLookbehindDistanceMeters = 200.0;

        if (distanceSoFar < maxLookbehindDistanceMeters) {
            bool roadWasFound;
            RoadSegmentIdType backwardsRoadId;
            bool backwardsRoadIsConnectedAtFront;
            unsigned int backwardsLaneNumber;

            roads.FindRoadAndLaneGoingBackwards(
                currentRoadId,
                nextLane,
                roadWasFound,
                backwardsRoadId,
                backwardsRoadIsConnectedAtFront,
                backwardsLaneNumber);

            if (roadWasFound) {
                const RoadSegment& backwardsRoad = roads.GetRoadSegment(backwardsRoadId);
                const RoadSegment::ConnectionInfoType& connection =
                    backwardsRoad.GetConnectionInfo(currentRoadId);
                const RoadSegmentIdType laneRoadId =
                    connection.GetLaneInfo(backwardsLaneNumber).intersectionLaneRoadId;

                bool wasFound = false;

                if (laneRoadId != InvalidRoadSegmentId) {
                    const RoadSegmentState& laneState = roadStates.at(laneRoadId);
                    const RoadSegment& lane = roads.GetRoadSegment(laneRoadId);

                    unsigned tailVehicleIndex;
                    laneState.GetFirstVehicleIndex(0, false, wasFound, tailVehicleIndex);

                    if (wasFound) {
                        const Vehicle& tailVehicle = laneState.GetVehicle(tailVehicleIndex);

                        tailVelocityMetersSec = tailVehicle.GetCurrentVelocity();

                        tailSpaceMeters =
                            distanceSoFar +
                            (lane.GetLengthMeters() -
                             tailVehicle.GetCurrentPosition().positionInRoadMeters);

                        tailSpaceMeters -=
                            ((tailVehicle.GetLengthMeters() + aVehicle.GetLengthMeters()) / 2.0);
                    }
                    else {
                        distanceSoFar += lane.GetLengthMeters();
                    }//if//
                }//if//

                if (!wasFound) {

                    // Intersection Lane does not exist (0 length) or it is empty of vehicles.
                    // Check road.

                    const RoadSegmentState& backwardsRoadState = roadStates[backwardsRoadId];
                    unsigned tailVehicleIndex;
                    backwardsRoadState.GetFirstVehicleIndex(
                        backwardsLaneNumber,
                        !backwardsRoadIsConnectedAtFront,
                        wasFound,
                        tailVehicleIndex);

                    if (wasFound) {
                        const Vehicle& tailVehicle = roadState.GetVehicle(previousVehicleIndex);
                        tailVelocityMetersSec = tailVehicle.GetCurrentVelocity();

                        tailSpaceMeters =
                                std::abs(
                                    (tailVehicle.GetCurrentPosition().positionInRoadMeters -
                                     aVehicle.GetCurrentPosition().positionInRoadMeters));

                        tailSpaceMeters -=
                            ((tailVehicle.GetLengthMeters() + aVehicle.GetLengthMeters()) / 2.0);
                    }//if//
                }//if//
            }//if//
        }//if//
    }//if//

    if (tailSpaceMeters < 0.0) {
        tailSpaceMeters = 0.0;
    }//if//

    if (LaneChangeIsSafe(
            idmParams.minGapMeters,
            idmParams.timeHeadwaySecs,
            aVehicle.GetCurrentVelocity(),
            idmParams.comfortableDeaccelerationMetersSec,
            frontSpaceMeters,
            frontVelocityMetersSec,
            tailSpaceMeters,
            tailVelocityMetersSec,
            idmParams.comfortableDeaccelerationMetersSec)) {

        laneChangeDelta = (static_cast<int>(nextLane) - static_cast<int>(currentLane));

    }//if//

}//MakeLaneChangeDecision//




inline
void MoveVehicle(
    const TimeType& currentTime,
    const double& timeStepDurationSecs,
    const IntelligentDriverModelParameters& idmParams,
    const AutoRoadNetwork& roads,
    const vector<RoadSegmentState>& roadStates,
    Vehicle& aVehicle)
{
    if ((!aVehicle.IsEnabled()) || (!aVehicle.HasARoute())) {
        return;
    }//if//

    int laneChangeDelta = 0;

    double forcedLaneChangeStopDistanceMeters = DBL_MAX;

    MakeLaneChangeDecision(
        currentTime,
        timeStepDurationSecs,
        idmParams,
        roads,
        roadStates,
        aVehicle,
        laneChangeDelta,
        forcedLaneChangeStopDistanceMeters);

    assert(std::abs(laneChangeDelta) <= 1);

    double accelerationMetersSec = 0.0;
    double maxVelocityMetersSec = 0.0;

    if (laneChangeDelta == 0) {

        double obstacleDistanceMeters;
        double obstacleVelocityMetersSec;
        bool obstacleDoesNotNeedAVehicleGap;
        double speedConstraintDistanceMeters;
        double speedConstraintVelocityMetersSec;

        CalcLeadingVehicleOrObstacleDistanceAndVelocity(
            currentTime, roads, roadStates,
            aVehicle,
            idmParams.comfortableDeaccelerationMetersSec,
            idmParams.minGapMeters,
            idmParams.searchDistanceCutoffMeters,
            obstacleDistanceMeters, obstacleVelocityMetersSec, obstacleDoesNotNeedAVehicleGap,
            speedConstraintDistanceMeters, speedConstraintVelocityMetersSec);

        const RoadSegment& aRoad = roads.GetRoadSegment(aVehicle.GetCurrentRoadId());
        maxVelocityMetersSec = aRoad.GetSpeedLimitMetersSec();

        if (forcedLaneChangeStopDistanceMeters != DBL_MAX) {
            if ((obstacleDistanceMeters == DBL_MAX) ||
                (forcedLaneChangeStopDistanceMeters < obstacleDistanceMeters) ||
                (forcedLaneChangeStopDistanceMeters <
                 (obstacleDistanceMeters +
                  CalcBrakingDistanceMeters(
                      idmParams.comfortableDeaccelerationMetersSec,
                      obstacleVelocityMetersSec,
                      0.0)))) {

                // Stop to make a forced (by route) lane change.

                obstacleDistanceMeters = forcedLaneChangeStopDistanceMeters;
                obstacleVelocityMetersSec = 0.0;
                obstacleDoesNotNeedAVehicleGap = true;
            }//if//
        }//if//

        if (obstacleDistanceMeters < DBL_MAX) {

            double minGapMeters = idmParams.minGapMeters;
            if (obstacleDoesNotNeedAVehicleGap) {
                minGapMeters = MinGapForTimestepErrorsMeters;
            }//if//

            accelerationMetersSec =
                CalculateIntelligentDriversModelAcceleration(
                    maxVelocityMetersSec,
                    idmParams.maxAccelerationMetersSec,
                    idmParams.comfortableDeaccelerationMetersSec,
                    minGapMeters,
                    idmParams.timeHeadwaySecs,
                    aVehicle.GetCurrentVelocity(),
                    obstacleDistanceMeters,
                    obstacleVelocityMetersSec);

        }
        else {
            if (speedConstraintDistanceMeters < DBL_MAX) {

                // Speed limit change ahead.  Calc possible braking.

                accelerationMetersSec =
                    CalculateSpeedLimitChangeAcceleration(
                        idmParams.comfortableDeaccelerationMetersSec,
                        aVehicle.GetCurrentVelocity(),
                        speedConstraintDistanceMeters,
                        speedConstraintVelocityMetersSec,
                        timeStepDurationSecs);
            }//if//

            if (accelerationMetersSec == 0.0) {
                if (aVehicle.GetCurrentVelocity() < maxVelocityMetersSec) {
                    accelerationMetersSec = idmParams.maxAccelerationMetersSec;
                }
                else {
                    assert((aVehicle.GetCurrentVelocity() - maxVelocityMetersSec) < DBL_EPSILON);
                }//if//
            }//if//
        }//if//
    }//if//

    UpdateVehiclePosition(
        timeStepDurationSecs,
        roads,
        accelerationMetersSec,
        maxVelocityMetersSec,
        laneChangeDelta,
        aVehicle);

}//MoveVehicle//



inline
void UpdateRoadStates(vector<RoadSegmentState>& roadStates)
{
    for(unsigned int i = 0; (i < roadStates.size()); i++) {
        roadStates[i].UpdateVehicleOrder();
    }//for//
}


inline
void MoveTheVehicles(
    const TimeType& currentTime,
    const TimeType& timeStepDuration,
    const double& timeStepDurationSecs,
    const IntelligentDriverModelParameters& idmParams,
    const AutoRoadNetwork& roads,
    vector<shared_ptr<Vehicle> > vehicles,
    vector<RoadSegmentState>& roadStates)
{
    // Calculate new positions:

    for(unsigned int i = 0; (i < vehicles.size()); i++) {
        MoveVehicle(
            currentTime,
            timeStepDurationSecs,
            idmParams,
            roads,
            roadStates,
            *vehicles[i]);
    }//for//

    for(unsigned int i = 0; (i < vehicles.size()); i++) {

        Vehicle& aVehicle = *vehicles[i];

        if (!aVehicle.IsEnabled() && (aVehicle.GetStartTime() <= (currentTime + timeStepDuration))) {

            roadStates.at(aVehicle.GetCurrentRoadId()).AddVehicle(vehicles[i]);

        }//if//

        if (aVehicle.GetCurrentRoadId() != aVehicle.GetNextRoadId()) {
            roadStates.at(aVehicle.GetCurrentRoadId()).DeleteVehicleFromRoad(aVehicle.GetVehicleId());
            if (aVehicle.GetNextRoadId() != InvalidRoadSegmentId) {
                roadStates.at(aVehicle.GetNextRoadId()).AddVehicle(vehicles[i]);
            }//if//
        }//if//

        aVehicle.UpdateToNextTimestep();

    }//for//

    UpdateRoadStates(roadStates);

}//MoveTheVehicles//



}//namespace//

#endif


