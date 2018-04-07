// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "multiagent_gis.h"
#include "multiagent_behavior.h"
#include "multiagent_publicvehicle.h"

namespace MultiAgent {

const double DEFAULT_VEHICLE_SPEED_MPS = 15;
const double DEFAULT_TRAIN_SPEED_MPS = 30;

const double MultiAgentGis::SCENSIM_VERSION = 1.5300;

const string MultiAgentGis::modelName = "Mas";

template <typename T> inline
void operator+=(deque<T>& left, const deque<T>& right)
{
    for(size_t i = 0; i < right.size(); i++) {
        left.push_back(right[i]);
    }
}

// vehicle functions -------------------------------------------------------

AgentRoad::AgentRoad(
    const shared_ptr<const Road>& initGisRoadPtr,
    const shared_ptr<RoadStat>& initRoadStatPtr,
    const vector<RoadIdType>& initGroupedRoadIds)
    :
    gisRoadPtr(initGisRoadPtr),
    length(initGisRoadPtr->GetArcDistanceMeters()),
    roadGranurality(std::max<double>(
            1.0,
            initGisRoadPtr->GetRoadWidthMeters() / MAX_NUMBER_ROAD_CROSSING_GRANURALITIES)),
    containsSomeVehicle(false),
    vehiclesPerDirection(NUMBER_ROAD_DIRECTIONS),
    needToOutputStatsAtThisTime(false),
    needToExecuteCrossingPedestrianUpdateAtThisTime(false),
    waitingPeopleAtStartIntersection(false),
    waitingPeopleAtEndIntersection(false),
    crossingPeopleExistanceAtStartIntersection(0),
    crossingPeopleExistanceAtEndIntersection(0),
    roadStatPtr(initRoadStatPtr),
    groupedRoadIds(initGroupedRoadIds)
{
    const double offset =
        gisRoadPtr->GetNumberOfStartToEndLanes() * gisRoadPtr->GetLaneWidthMeters();
    const size_t numberVertices = gisRoadPtr->NumberOfVertices();

    if (numberVertices >= 2) {
        Vertex p1;
        Vertex p2;
        Vertex normal;
        Vertex offsetPoint;

        p1 = gisRoadPtr->GetVertex(0);
        p2 = gisRoadPtr->GetVertex(1);
        normal = p1.NormalVector(p2);
        offsetPoint = normal * (offset/p1.DistanceTo(p2));

        startIntersectionAcrossBase = p1 + offsetPoint;

        p1 = gisRoadPtr->GetVertex(numberVertices - 1);
        p2 = gisRoadPtr->GetVertex(numberVertices - 2);
        normal = p1.NormalVector(p2);
        offsetPoint = normal * (offset/p1.DistanceTo(p2));

        endIntersectionAcrossBase = p1 + offsetPoint;
    }

    const size_t numberLanes = gisRoadPtr->GetNumberOfLanes();

    laneLengths.resize(numberLanes);

    for(size_t i = 0; i < numberLanes; i++) {
        deque<Vertex> laneVertices;

        gisRoadPtr->GetLaneVertices(i, false/*waypointFromAdditionalStartPosition*/, Vertex(), laneVertices);
        laneLengths[i] = CalculateArcDistance(laneVertices);
    }
}

void AgentRoad::FindFrontVehicle(
    const VehicleIter& origIter,
    const size_t laneNumber,
    bool& found,
    VehicleIter& iter) const
{
    const list<shared_ptr<Vehicle> >& vehicles =
        vehiclesPerDirection.at(gisRoadPtr->GetRoadDirection(laneNumber));

    if (origIter == vehicles.begin()) {
        found = false;
        return;
    }

    iter = origIter;
    do {
        iter--;

        if ((*iter)->IsOnLane(laneNumber)) {
            found = true;
            return;
        }

    } while (iter != vehicles.begin());

    found = false;
}

void AgentRoad::FindBackVehicle(
    const VehicleIter& origIter,
    const size_t laneNumber,
    bool& found,
    VehicleIter& iter) const
{
    const list<shared_ptr<Vehicle> >& vehicles =
        vehiclesPerDirection.at(gisRoadPtr->GetRoadDirection(laneNumber));

    if (origIter == vehicles.end()) {
        found = false;
        return;
    }

    iter = origIter;
    iter++;

    for(; iter != vehicles.end(); iter++) {
        if ((*iter)->IsOnLane(laneNumber)) {
            found = true;
            return;
        }
    }

    found = false;
}

void AgentRoad::FindLeaderVehicleIter(
    const size_t laneNumber,
    bool& found,
    VehicleIter& iter) const
{
    const list<shared_ptr<Vehicle> >& vehicles =
        vehiclesPerDirection.at(gisRoadPtr->GetRoadDirection(laneNumber));

    iter = vehicles.begin();

    for(; iter != vehicles.end(); iter++) {

        if ((*iter)->IsOnLane(laneNumber)) {
            found = true;
            return;
        }
    }

    found = false;
}

void AgentRoad::FindTailVehicleIter(
    const size_t laneNumber,
    bool& found,
    ReverseVehicleIter& iter,
    const double searchDistanceFromTail) const
{
    found = false;

    const list<shared_ptr<Vehicle> >& vehicles =
        vehiclesPerDirection.at(gisRoadPtr->GetRoadDirection(laneNumber));

    iter = vehicles.rbegin();

    for(; iter != vehicles.rend(); iter++) {

        const double distanceToIntersection = (*iter)->GetDistanceToIntersection();
        const double distanceFromTail = std::max<double>(0., length - distanceToIntersection);

        if (distanceFromTail > searchDistanceFromTail) {
            return;
        }

        if ((*iter)->IsOnLane(laneNumber)) {
            found = true;
            return;
        }
    }
}

bool AgentRoad::HasFrontVehicle(
    const Vehicle& vehicle,
    const size_t laneNumber) const
{
    if (gisRoadPtr->IsParking()) {
        return false;
    }

    bool found;

    const VehiclePositionKey& positionKey = vehicle.positionKey;

    if (positionKey.roadId == gisRoadPtr->GetRoadId() &&
        gisRoadPtr->GetRoadDirection(positionKey.laneNumber) == gisRoadPtr->GetRoadDirection(laneNumber)) {

        VehicleIter iter;

        (*this).FindFrontVehicle(
            positionKey.iter,
            laneNumber,
            found,
            iter);

    } else {
        ReverseVehicleIter iter;

        (*this).FindTailVehicleIter(laneNumber, found, iter);
    }

    return found;
}

bool AgentRoad::HasBackVehicle(
    const Vehicle& vehicle,
    const size_t laneNumber) const
{
    if (gisRoadPtr->IsParking()) {
        return false;
    }

    bool found;

    const VehiclePositionKey& positionKey = vehicle.positionKey;

    if (positionKey.roadId == gisRoadPtr->GetRoadId() &&
        gisRoadPtr->GetRoadDirection(positionKey.laneNumber) == gisRoadPtr->GetRoadDirection(laneNumber)) {

        VehicleIter iter;

        (*this).FindBackVehicle(
            positionKey.iter,
            laneNumber,
            found,
            iter);
    } else {
        VehicleIter iter;

        (*this).FindLeaderVehicleIter(laneNumber, found, iter);
    }

    return found;
}

bool AgentRoad::HasVehicle(const size_t laneNumber) const
{
    bool found;
    ReverseVehicleIter iter;

    (*this).FindTailVehicleIter(
        laneNumber,
        found,
        iter);

    return found;
}

const Vehicle& AgentRoad::GetFrontVehicle(
    const Vehicle& vehicle,
    const size_t laneNumber) const
{
    const VehiclePositionKey& positionKey = vehicle.positionKey;

    bool found;

    if (positionKey.roadId == gisRoadPtr->GetRoadId() &&
        gisRoadPtr->GetRoadDirection(positionKey.laneNumber) == gisRoadPtr->GetRoadDirection(laneNumber)) {

        VehicleIter iter;

        (*this).FindFrontVehicle(
            positionKey.iter,
            laneNumber,
            found,
            iter);

        assert(found);

        return *(*iter);

    } else {
        ReverseVehicleIter iter;

        (*this).FindTailVehicleIter(laneNumber, found, iter);

        assert(found);

        return *(*iter);
    }
}

const Vehicle& AgentRoad::GetBackVehicle(
    const Vehicle& vehicle,
    const size_t laneNumber) const
{
    const VehiclePositionKey& positionKey = vehicle.positionKey;

    bool found;

    if (positionKey.roadId == gisRoadPtr->GetRoadId() &&
        gisRoadPtr->GetRoadDirection(positionKey.laneNumber) == gisRoadPtr->GetRoadDirection(laneNumber)) {

        VehicleIter iter;

        (*this).FindBackVehicle(
            positionKey.iter,
            laneNumber,
            found,
            iter);

        assert(found);

        return *(*iter);

    } else {
        VehicleIter iter;

        (*this).FindLeaderVehicleIter(laneNumber, found, iter);

        assert(found);

        return *(*iter);
    }
}

const Vehicle& AgentRoad::GetTailVehicle(const size_t laneNumber) const
{
    bool found;
    ReverseVehicleIter iter;

    (*this).FindTailVehicleIter(
        laneNumber,
        found,
        iter);

    assert(found);

    return *(*iter);
}

bool AgentRoad::AllowedIncomingVehicle(
    const Vehicle& vehicle,
    const size_t laneNumber) const
{
    if (gisRoadPtr->IsParking()) {
        return true;
    }

    const list<shared_ptr<Vehicle> >& vehicles =
        vehiclesPerDirection.at(gisRoadPtr->GetRoadDirection(laneNumber));

    ReverseVehicleIter iter = vehicles.rbegin();

    const double minVelocityAtIntersection = 5;
    const double myVelocity =
        std::max(minVelocityAtIntersection, vehicle.GetVelocityMetersPerSec());

    const double enoughLength =
        vehicle.GetHalfBodyLength()*2 + vehicle.GetVehicleConstant().minVehicleGap;

    const double laneLength = laneLengths.at(laneNumber);

    for(; iter != vehicles.rend(); iter++) {
        const Vehicle& tailVehicle = *(*iter);

        const double distanceFromIntersection =
            std::max<double>(0., laneLength - tailVehicle.GetDistanceToIntersection());

        const double otherVelocity = tailVehicle.GetVelocityMetersPerSec();
        const double distanceToFront = vehicle.GetPosition().DistanceTo(tailVehicle.GetPosition());
        const double timeToTail = distanceToFront / myVelocity;

        if (distanceFromIntersection + timeToTail*otherVelocity >= enoughLength) {
            return true;
        }
        if (tailVehicle.IsOnLane(laneNumber)) {
            return false;
        }
    }

    return true;
}

void AgentIntersection::GetCrossingOrSideWalkRoadIds(
    const MultiAgentGis& theAgentGis,
    const RoadIdType& startRoadId,
    const RoadIdType& destRoadId,
    const bool isStartLaneLeft,
    bool& isClockwise,
    list<pair<RoadIdType, bool> >& crossingOrRoadsideWalks,
    deque<Vertex>& waypoints,
    HighQualityRandomNumberGenerator& aRandomNumberGenerator) const
{
    crossingOrRoadsideWalks.clear();
    waypoints.clear();

    if (startRoadId == destRoadId ||
        startRoadId == INVALID_VARIANT_ID ||
        !ref.ContainsRoadside(destRoadId)) {
        isClockwise = isStartLaneLeft;
        return;
    }

    typedef list<pair<double, RoadIdType> >::const_iterator IterType;
    typedef list<pair<double, RoadIdType> >::const_reverse_iterator ReverseIterType;

    deque<RoadIdType> clockwiseRoadIds;
    deque<RoadIdType> antiClockwiseRoadIds;

    bool found = false;
    double clockwiseRadians = 0;
    double antiClockwiseRadians = 0;
    double startRadians = 0;

    typedef Intersection::Roadside Roadside;

    const vector<Roadside>& antiClockwiseRoadsides = ref.GetAntiClockwiseRoadsides();
    const size_t destRoadsideNumber = ref.GetRoadsideNumber(destRoadId);
    const Roadside& destRoadside = antiClockwiseRoadsides.at(destRoadsideNumber);

    int numberOfClockwiseNonExtraPaths = 0;
    int numberOfAnticlockwiseNonExtraPaths = 0;

    for(IterType iter = sequencedRoadIdsOrderedByRadian.begin();
        iter != sequencedRoadIdsOrderedByRadian.end(); iter++) {

        const RoadIdType& roadId = (*iter).second;

        if (roadId == startRoadId) {
            found = true;
            startRadians = (*iter).first;

        } else if (found) {

            if (roadId == destRoadId) {
                if (startRadians < (*iter).second) {
                    startRadians += 2*PI;
                }
                antiClockwiseRadians = std::fabs(startRadians - (*iter).second);
                break;
            } else {
                if (!theAgentGis.GetAgentRoad(roadId).gisRoadPtr->IsExtraPath()) {
                    numberOfAnticlockwiseNonExtraPaths++;
                }
                antiClockwiseRoadIds.push_back(roadId);
            }
        }
    }

    found = false;

    for(ReverseIterType iter = sequencedRoadIdsOrderedByRadian.rbegin();
        iter != sequencedRoadIdsOrderedByRadian.rend(); iter++) {

        const RoadIdType& roadId = (*iter).second;

        if (roadId == startRoadId) {
            found = true;
            startRadians = (*iter).first;

        } else if (found) {

            if (roadId == destRoadId) {
                if (startRadians > (*iter).second) {
                    startRadians -= 2*PI;
                }
                clockwiseRadians = std::fabs((*iter).second - startRadians);
                break;
            } else {
                if (!theAgentGis.GetAgentRoad(roadId).gisRoadPtr->IsExtraPath()) {
                    numberOfClockwiseNonExtraPaths++;
                }
                clockwiseRoadIds.push_back(roadId);
            }
        }
    }

    const Road& startRoad = *theAgentGis.GetAgentRoad(startRoadId).gisRoadPtr;

    if (isStartLaneLeft) {
        antiClockwiseRoadIds.push_front(startRoadId);
        if (!startRoad.IsExtraPath()) {
            numberOfAnticlockwiseNonExtraPaths++;
        }
    } else {
        clockwiseRoadIds.push_front(startRoadId);
        if (!startRoad.IsExtraPath()) {
            numberOfClockwiseNonExtraPaths++;
        }
    }

    if (numberOfClockwiseNonExtraPaths == numberOfAnticlockwiseNonExtraPaths) {

        if (numberOfClockwiseNonExtraPaths == 1 &&
            clockwiseRadians == antiClockwiseRadians) {

            isClockwise = isStartLaneLeft;

        } else {

            // Small rotation is preferred.
            isClockwise = (aRandomNumberGenerator.GenerateRandomDouble() <= (antiClockwiseRadians / (clockwiseRadians + antiClockwiseRadians)));
        }

    } else {
        isClockwise = (numberOfClockwiseNonExtraPaths < numberOfAnticlockwiseNonExtraPaths);
    }

    const GisSubsystem& subsystem = theAgentGis.GetSubsystem();

    Intersection::RoadSideEdgeType crossingOriginEdgeType;
    Intersection::RoadSideEdgeType crossingOppositeEdgeType;

    int incrementCount;
    deque<RoadIdType> passRoadIds;

    const Vertex& intersectionPoint = ref.GetVertex();

    if (isClockwise) {

        crossingOriginEdgeType = Intersection::ROAD_SIDE_RIGHT_EDGE;
        crossingOppositeEdgeType = Intersection::ROAD_SIDE_LEFT_EDGE;
        incrementCount = -1;
        passRoadIds = clockwiseRoadIds;

    } else {

        crossingOriginEdgeType = Intersection::ROAD_SIDE_LEFT_EDGE;
        crossingOppositeEdgeType = Intersection::ROAD_SIDE_RIGHT_EDGE;
        incrementCount = 1;
        passRoadIds = antiClockwiseRoadIds;
    }

    passRoadIds.push_back(destRoadId);

    int startRoadsideNumber = static_cast<int>(ref.GetRoadsideNumber(startRoadId));
    Vertex lastCenterPointOnRoad;
    Vertex transferBaseEdge;
    bool isLastCenterPointInitialized = false;

    if (isClockwise == isStartLaneLeft) {

        if (!startRoad.IsExtraPath()) {
            const Roadside& roadside =
                antiClockwiseRoadsides.at(startRoadsideNumber % antiClockwiseRoadsides.size());

            lastCenterPointOnRoad = (roadside.edges[crossingOriginEdgeType] + roadside.edges[crossingOppositeEdgeType])*0.5;
            transferBaseEdge = roadside.edges[crossingOppositeEdgeType];
            isLastCenterPointInitialized = true;
        }

        startRoadsideNumber += incrementCount;
    }

    for(int i = 0; i < int(passRoadIds.size()); i++) {
        const RoadIdType& roadId = passRoadIds[i];
        const Road& road = *theAgentGis.GetAgentRoad(roadId).gisRoadPtr;

        if (road.IsExtraPath()) {
            continue;
        }

        int currentRoadsideNumber = startRoadsideNumber + (i*incrementCount);
        if (currentRoadsideNumber < 0) {
            currentRoadsideNumber += static_cast<int>(antiClockwiseRoadsides.size());
        }

        const Roadside& roadside =
            antiClockwiseRoadsides.at(currentRoadsideNumber % antiClockwiseRoadsides.size());

        const Vertex currentCenterPointOnRoad =
            (roadside.edges[crossingOriginEdgeType] + roadside.edges[crossingOppositeEdgeType])*0.5;

        const double nearDistanceMeters = 0.1;

        if (isLastCenterPointInitialized &&
            currentCenterPointOnRoad.DistanceTo(lastCenterPointOnRoad) > nearDistanceMeters &&
            currentCenterPointOnRoad.DistanceTo(intersectionPoint) > nearDistanceMeters &&
            lastCenterPointOnRoad.DistanceTo(intersectionPoint) > nearDistanceMeters &&
            transferBaseEdge.DistanceTo(roadside.edges[crossingOriginEdgeType]) > nearDistanceMeters) {

            const double offset = road.GetRoadWidthMeters()*0.5;
            vector<Vertex> srcWaypoints;

            srcWaypoints.push_back(lastCenterPointOnRoad);
            srcWaypoints.push_back(intersectionPoint);
            srcWaypoints.push_back(currentCenterPointOnRoad);

            deque<Vertex> roadSideWaypoints;

            subsystem.GetOffsetWaypoints(
                static_cast<double>(incrementCount) * offset, //adjust offset direction
                srcWaypoints,
                false/*waypointFromAdditionalStartPosition*/,
                Vertex()/*startPosition*/,
                false/*replaceFirstEntryWithStartPosition*/,
                road.IsBaseGroundLevel(),
                roadSideWaypoints);

            for(size_t j = 0; j < roadSideWaypoints.size(); j++) {
                waypoints.push_back(roadSideWaypoints[j]);
                crossingOrRoadsideWalks.push_back(make_pair(roadId, false));
            }

        } else {
            waypoints.push_back(roadside.edges[crossingOriginEdgeType]);
            crossingOrRoadsideWalks.push_back(make_pair(roadId, false));
        }

        if (i == static_cast<int>(passRoadIds.size()) - 1) {
            break;
        }

        waypoints.push_back(roadside.edges[crossingOppositeEdgeType]);
        crossingOrRoadsideWalks.push_back(make_pair(roadId, true));

        lastCenterPointOnRoad = currentCenterPointOnRoad;
        transferBaseEdge = roadside.edges[crossingOppositeEdgeType];
        isLastCenterPointInitialized = true;
    }
}

void AgentIntersection::CanEnterTheRoad(
    const TimeType& currentTime,
    const VehicleDriverBehavior& vehicleBehavior,
    const RoadTurnType& origTurn,
    const RoadIdType& incomingRoadId,
    const RoadIdType& outgoingRoadId,
    bool& canPassIntersection,
    TimeType& entranceEnabledTime) const
{
    entranceEnabledTime = currentTime;

    const VehicleConstant& vehicleConstant = vehicleBehavior.GetVehicle().GetVehicleConstant();
    const AgentResource resource = vehicleBehavior.Resouce();
    const double nearIntersectionDistance = 100.;

    const TrafficLightType trafficLight = ref.GetTrafficLight(currentTime, incomingRoadId);

    if (trafficLight == TRAFFIC_LIGHT_RED) {
        canPassIntersection = false;
        return;
    }

    for(size_t i = 0; i < incomingRoads.size(); i++) {
        const IncomingRoad& incomingRoad = incomingRoads[i];
        const AgentRoad& road = *incomingRoad.gisRoadPtr;
        const RoadIdType roadId = road.gisRoadPtr->GetRoadId();

        bool found;
        VehicleIter otherIter;

        road.FindLeaderVehicleIter(
            incomingRoad.laneNumber,
            found,
            otherIter);

        if (!found ||
            (*otherIter)->GetDistanceToIntersection() > nearIntersectionDistance ||
            (*otherIter)->GetDriverAgentId() == vehicleBehavior.GetAgentId()) {
            continue;
        }

        const Vehicle& otherVehicle = *(*otherIter);
        const RoadTurnType& otherTurn = otherVehicle.GetTurnType();
        const RoadTurnType& origToOtherTurn = ref.GetRoadTurnType(incomingRoadId, roadId);

        assert(otherVehicle.GetRoadId() == roadId);

        if (origToOtherTurn.direction == ROAD_TURN_BACK) {
            continue;
        }

        if (outgoingRoadId == roadId &&
            origTurn.direction == ROAD_TURN_STRAIGHT &&
            (otherTurn.direction == ROAD_TURN_STRAIGHT ||
             otherTurn.direction == ROAD_TURN_NONE)) {
            continue;
        }

        if (ref.GetTrafficLight(currentTime, roadId) == TRAFFIC_LIGHT_GREEN) {

            if (vehicleBehavior.YieldRoadTo(otherVehicle, vehicleConstant.passiveYieldTime)) {
                canPassIntersection = false;
                entranceEnabledTime =
                    currentTime + static_cast<TimeType>(vehicleConstant.yieldWaitingTime * SECOND);
                return;

            } else if ((*this).IsLowPriorityRoad(origTurn, otherTurn, origToOtherTurn)) {

                if (vehicleBehavior.YieldRoadTo(otherVehicle, vehicleConstant.activeYieldTime)) {
                    canPassIntersection = false;
                    entranceEnabledTime =
                        currentTime + static_cast<TimeType>(vehicleConstant.yieldWaitingTime * SECOND);
                    return;
                }
            }
        }
    }

    canPassIntersection = true;
}

Vertex AgentRoad::GetOthersideLaneNormalPosition(
    const VertexIdType& origVertexId,
    const Vertex& origPosition,
    const bool isOrigPositionLeft) const
{
    const size_t numberVertices = gisRoadPtr->NumberOfVertices();
    Vertex p1;
    Vertex p2;

    assert(numberVertices >= 2);

    if (origVertexId == gisRoadPtr->GetStartVertexId()) {
        p1 = gisRoadPtr->GetVertex(0);
        p2 = gisRoadPtr->GetVertex(1);
    } else {
        p1 = gisRoadPtr->GetVertex(numberVertices - 1);
        p2 = gisRoadPtr->GetVertex(numberVertices - 2);
    }

    const Vertex normal = p1.NormalVector(p2);
    const Vertex offsetPoint = normal * (gisRoadPtr->GetRoadWidthMeters()/p1.DistanceTo(p2));

    if (isOrigPositionLeft) {
        return origPosition + offsetPoint;
    } else {
        return origPosition - offsetPoint;
    }
}

bool AgentRoad::CrossingAPedestrian(const VertexIdType& outgoingVertexId) const
{
    if (outgoingVertexId == gisRoadPtr->GetStartVertexId()) {

        return crossingPeopleExistanceAtStartIntersection.any();

    } else {

        return crossingPeopleExistanceAtEndIntersection.any();
    }
}

bool AgentRoad::CrossingOrWaitingAPedestrian(const VertexIdType& outgoingVertexId) const
{
    if (outgoingVertexId == gisRoadPtr->GetStartVertexId()) {

        return (crossingPeopleExistanceAtStartIntersection.any() ||
                waitingPeopleAtStartIntersection);
    } else {

        return (crossingPeopleExistanceAtEndIntersection.any() ||
                waitingPeopleAtEndIntersection);
    }
}

bool AgentIntersection::IsLowPriorityRoad(
    const RoadTurnType& origTurnType,
    const RoadTurnType& otherTurnType,
    const RoadTurnType& origToOtherTurnType) const
{
    if (otherTurnType.totalRoadWidth > origTurnType.totalRoadWidth + 4) {

        return true;

    } else if (origTurnType.totalRoadWidth > otherTurnType.totalRoadWidth + 4) {

        return false;

    } else if (origTurnType.direction == ROAD_TURN_RIGHT) {

        if (otherTurnType.direction == ROAD_TURN_STRAIGHT) {

            return true;

        } else if (otherTurnType.direction == ROAD_TURN_LEFT) {

            if (origToOtherTurnType.direction == ROAD_TURN_RIGHT ||
                origToOtherTurnType.direction == ROAD_TURN_LEFT) {
                return false;
            } else {
                return true;
            }

        } else if (origToOtherTurnType.direction == ROAD_TURN_RIGHT) {

            if (origToOtherTurnType.direction == ROAD_TURN_LEFT) {
                return true;
            }
        }

    } else if (origToOtherTurnType.direction == ROAD_TURN_RIGHT) {

        if (otherTurnType.direction == ROAD_TURN_STRAIGHT &&
            origTurnType.direction == ROAD_TURN_LEFT) {
            return true;
        }

    } else if (otherTurnType.direction == ROAD_TURN_STRAIGHT &&
               origTurnType.direction != ROAD_TURN_STRAIGHT) {
        return true;
    }

    return false;
}

inline
Vertex MakeVertexWithPerpendicularOffset(
    const Vertex& mainVertex,
    const Vertex& secondaryVertex,
    const double& offset)
{
    const Vertex directionUnitVector = ((secondaryVertex - mainVertex).Normalized());
    const Vertex perpendicularVector = directionUnitVector.NormalVector();
    return (mainVertex + (offset * perpendicularVector));
}


//-----------------------------------------------------------------

AgentPeopleBuffer::AgentPeopleBuffer(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const MultiAgentGis& theMultiAgentGis,
    const set<EntranceIdType>& initEntranceIds,
    const shared_ptr<RealStatistic>& initCongestionStatPtr,
    const shared_ptr<RealStatistic>& initPopulationStatPtr)
    :
    needToOutputStatsAtThisTime(false),
    congestionTrace(0.),
    numberPeoples(0),
    congestionStatPtr(initCongestionStatPtr),
    populationStatPtr(initPopulationStatPtr)
{
    typedef set<EntranceIdType>::const_iterator IterType;

    for(IterType iter = initEntranceIds.begin(); (iter !=initEntranceIds.end()); ++iter) {
        (*this).AddEntranceQueue(theParameterDatabaseReader, theMultiAgentGis, *iter);
    }//for//
}




unique_ptr<PedQueuePositioner> AgentPeopleBuffer::CreateEntranceLineQueuePositioner(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GisSubsystem& theGisSubsystem,
    const EntranceIdType& entranceId,
    const GisObjectIdType gisObjectId) const
{
    const double minMainRoadLengthMeters = 10.0;

    RoadIdType mainRoadId;
    VertexIdType mainRoadVertexId;
    Vertex mainRoadVertex;

    theGisSubsystem.GetMainRoadAndVertexConnectedToEntrance(
        entranceId, minMainRoadLengthMeters,
        mainRoadId, mainRoadVertexId, mainRoadVertex);
    
    const Road& theRoad = theGisSubsystem.GetRoad(mainRoadId);

    const deque<Vertex>& vertices = theRoad.GetCompleteVertices();

    vector<Vertex> orderedVertices;

    if (vertices.front() == mainRoadVertex) {
        for(unsigned int i = 0; (i < vertices.size()); i++) {
            orderedVertices.push_back(vertices[i]);
        }//for//
    }
    else {
        for(int i = static_cast<int>(vertices.size() - 1); (i >= 0); i--) {
            orderedVertices.push_back(vertices[i]);
        }//for//
    }//if//

    // Move vertices to edge of road.  No official sidewalks implemented.

    const Vertex entranceVertex = theGisSubsystem.GetEntrance(entranceId).GetVertex();

    double roadEdgeOffsetMeters = theRoad.GetRoadWidthMeters() / 2.0;

    assert(orderedVertices.size() >= 2);

    const Vertex positiveOffsetVertex =
        MakeVertexWithPerpendicularOffset(
            orderedVertices[0],  orderedVertices[1], roadEdgeOffsetMeters);

    const Vertex negativeOffsetVertex =
        MakeVertexWithPerpendicularOffset(
            orderedVertices[0],  orderedVertices[1], -roadEdgeOffsetMeters);

    const double positiveOffsetDistance = entranceVertex.XYDistanceTo(positiveOffsetVertex);
    const double negativeOffsetDistance = entranceVertex.XYDistanceTo(negativeOffsetVertex);

    vector<Vertex> lineSegmentPoints(orderedVertices.size());

    if (positiveOffsetDistance < negativeOffsetDistance) {
        lineSegmentPoints[0] = positiveOffsetVertex;
    }
    else {
        // Change offset to negative.

        lineSegmentPoints[0] = negativeOffsetVertex;
        roadEdgeOffsetMeters = -roadEdgeOffsetMeters;
    }//if//

    for(unsigned int i = 1; (i < (orderedVertices.size() - 1)); i++) {
        lineSegmentPoints[i] =
            MakeVertexWithPerpendicularOffset(
                orderedVertices[i],  orderedVertices[i+1], roadEdgeOffsetMeters);
    }//for//

    // Backwards direction line => reverse the offset.

    lineSegmentPoints[orderedVertices.size() - 1] =
        MakeVertexWithPerpendicularOffset(
            orderedVertices[orderedVertices.size() - 1],
            orderedVertices[orderedVertices.size() - 2],
            -roadEdgeOffsetMeters);


    unsigned int peoplePerRow = 3;

    if (theParameterDatabaseReader.ParameterExists("gis-entrance-line-people-per-row", gisObjectId)) {
        peoplePerRow =
            theParameterDatabaseReader.ReadPositiveInt("gis-entrance-line-people-per-row", gisObjectId);
    }//if//

    double rowSeparationMeters = 1.0;
    if (theParameterDatabaseReader.ParameterExists("gis-entrance-line-row-separation-meters", gisObjectId)) {
        rowSeparationMeters =
            theParameterDatabaseReader.ReadDouble("gis-entrance-line-row-separation-meters", gisObjectId);
    }//if//

    double columnSeparationMeters = 1.0;
    if (theParameterDatabaseReader.ParameterExists("gis-entrance-line-column-separation-meters", gisObjectId)) {
        columnSeparationMeters =
            theParameterDatabaseReader.ReadDouble("gis-entrance-line-column-separation-meters", gisObjectId);
    }//if//

    return (
        unique_ptr<LinePedQueuePositioner>(
            new LinePedQueuePositioner(
                lineSegmentPoints,
                peoplePerRow,
                rowSeparationMeters,
                columnSeparationMeters)));

}//CreateEntranceLineQueuePositioner//



void AgentPeopleBuffer::AddEntranceQueue(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const MultiAgentGis& theMultiAgentGis,
    const EntranceIdType& entranceId)
{
    const GisObjectIdType gisObjectId =
        theMultiAgentGis.GetSubsystem().GetEntrance(entranceId).GetObjectId();

    string entranceQueueTypeName = "line";

    if (theParameterDatabaseReader.ParameterExists("gis-entrance-queue-type", gisObjectId)) {
        entranceQueueTypeName =
            theParameterDatabaseReader.ReadString("gis-entrance-queue-type", gisObjectId);

        ConvertStringToLowerCase(entranceQueueTypeName);

    }//if//

    unique_ptr<PedQueuePositioner> positionerPtr;

    if (entranceQueueTypeName == "line") {

        positionerPtr =
            CreateEntranceLineQueuePositioner(
                theParameterDatabaseReader,
                theMultiAgentGis.GetSubsystem(),
                entranceId,
                gisObjectId);
    }
    else {
        cerr << "Error in parameter: \"gis-entrance-queue-type\": unknown type" << endl;
        exit(1);
    }//if//

    const TimeType timestepDuration = theMultiAgentGis.GetTimestepDuration();

    double maxPeopleFlowRatePerSec = DBL_MAX;

    if (theParameterDatabaseReader.ParameterExists(
            "gis-entrance-queue-max-flow-rate-people-per-sec", gisObjectId)) {

        maxPeopleFlowRatePerSec =
            theParameterDatabaseReader.ReadDouble(
                "gis-entrance-queue-max-flow-rate-people-per-sec",
                gisObjectId);

        if (maxPeopleFlowRatePerSec < 0.0) {
            cerr << "Error in parameter: \"gis-entrance-queue-max-flow-rate-people-per-sec\": Flow rate must be larger than zero." << endl;
            exit(1);
        }//if//

    }//if//

    entranceQueue.AddEntrance(
        entranceId,
        positionerPtr,
        timestepDuration,
        maxPeopleFlowRatePerSec);

}//AddEntranceQueue//


void AgentPeopleBuffer::UpdateEntranceQueuesStatus(
    const TimeType& currentTime,
    bool& peopleChanged)
{
    peopleChanged = false;

    const int humcanCapacity = (*this).GetHumanCapacity();
    
    if (numberPeoples < humcanCapacity) {
        
        bool isActive;
        (*this).UpdateEntranceQueue(currentTime, isActive);
        
        if (isActive) {
            
            if (!nextPeopleStateInfo.peopleChanged) {
                nextPeopleStateInfo.peopleChanged = true;
                peopleChanged = true;                
            }//if//
        }//if//
    }//if//    
}//UpdateEntranceQueuesStatus//

void AgentPeopleBuffer::TransferPeopleToGisObject(
    const TimeType& currentTime,
    const GisPositionIdType& currentPositionId,
    MultiAgentGis& theAgentGis)
{
    bool somethingChanged = false;

    // Remove people who have given up.

    vector<AgentResource>& giveUpEntranceAgents = nextPeopleStateInfo.giveUpEntranceAgents;

    for(unsigned int i = 0; (i < giveUpEntranceAgents.size()); i++) {
        AgentResource& person = giveUpEntranceAgents[i];
        entranceQueue.RemovePerson(person);
        person.EndEntranceWaiting();
        person.SetDesiredNextPositionId(person.GetCurrentPositionId());
        somethingChanged = true;
    }//for//

    giveUpEntranceAgents.clear();

    // Allow waiting people to enter.

    const int humcanCapacity = (*this).GetHumanCapacity();
    
    while (numberPeoples < humcanCapacity) {

        const unsigned int remainingCapacity = (humcanCapacity - numberPeoples);

        bool wasRetrieved;
        AgentResource person;

        entranceQueue.GetPersonWhoCanEnterAtThisTime(
            currentTime,
            remainingCapacity,
            wasRetrieved,
            person);

        if (!wasRetrieved) {
            break;
        }//if//

        numberPeoples += person.NumberPeople();

        theAgentGis.RemoveAgentFromCurrentGisLocation(person);
        person.UpdateCurrentPositionIdToDesiredPositionId();
        assert(person.GetCurrentPositionId() == currentPositionId);
        person.AllowedEntrance();
        somethingChanged = true;

    }//while//

    // Moving incoming people to queue (or allow entry if possible).

    vector<AgentResource>& incomingPeopleAgents = nextPeopleStateInfo.incomingPeopleAgents;

    for(unsigned int i = 0; (i < incomingPeopleAgents.size()); i++) {
        AgentResource& person = incomingPeopleAgents[i];

        if (person.GetCurrentPositionId() == InvalidGisPositionId) {

            // Coming from nowhere.  Assumed at initialization/creation.
            // Bypassing limits and queues to get into location ("magically
            // appears").

            numberPeoples += person.NumberPeople();
            person.UpdateCurrentPositionIdToDesiredPositionId();
            person.AllowedEntrance();
            somethingChanged = true;
        }
        else {
            const EntranceIdType entranceId = 
                (*this).GetNearestEntranceId(person.Position());

            if (numberPeoples < humcanCapacity) {

                bool wasQueued;

                entranceQueue.TryToEnter(currentTime, person, entranceId, wasQueued);

                if (!wasQueued) {
                    numberPeoples += person.NumberPeople();
                    theAgentGis.RemoveAgentFromCurrentGisLocation(person);
                    person.UpdateCurrentPositionIdToDesiredPositionId();
                    person.AllowedEntrance();
                    somethingChanged = true;
                }//if//
            }
            else {
                entranceQueue.InsertIntoQueue(currentTime, person, entranceId);
            }//if//
        }//if//
    }//for//

    incomingPeopleAgents.clear();

    if (somethingChanged) {
        needToOutputStatsAtThisTime = true;
    }//if//
}//TransferPeopleToGisObject//

//-----------------------------------------------------------------

AgentBuilding::AgentBuilding(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const MultiAgentGis& theMultiAgentGis,
    const Building& initBuilding,
    const shared_ptr<RealStatistic>& initCongestionStatPtr,
    const shared_ptr<RealStatistic>& initPopulationStatPtr)
    :
    AgentPeopleBuffer(
        parameterDatabaseReader,
        theMultiAgentGis,
        initBuilding.GetEntranceIds(),
        initCongestionStatPtr,
        initPopulationStatPtr),
    ref(initBuilding),
    polygonSize(std::max<double>(1., ref.CalculateSize()))
{}

EntranceIdType AgentBuilding::GetNearestEntranceId(const Vertex& position)
{

    return ref.GetNearestEntranceId(position);

}//AGetNearestEntranceId//

int AgentBuilding::GetHumanCapacity()
{

    return ref.GetHumanCapacity();

}//GetHumanCapacity//

//-----------------------------------------------------------------

AgentPark::AgentPark(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const MultiAgentGis& theMultiAgentGis,
    const Park& initPark,
    const shared_ptr<RealStatistic>& initCongestionStatPtr,
    const shared_ptr<RealStatistic>& initPopulationStatPtr)
    :
    AgentPeopleBuffer(
        parameterDatabaseReader,
        theMultiAgentGis,
        initPark.GetEntranceIds(),
        initCongestionStatPtr,
        initPopulationStatPtr),
    ref(initPark),
    polygonSize(std::max<double>(1., ref.CalculateSize()))
{}

EntranceIdType AgentPark::GetNearestEntranceId(const Vertex& position)
{

    return ref.GetNearestEntranceId(position);

}//AGetNearestEntranceId//

int AgentPark::GetHumanCapacity()
{

    return ref.GetHumanCapacity();

}//GetHumanCapacity//

//-----------------------------------------------------------------

shared_ptr<Train> AgentStation::GetTrain(
    const LineIdType& lineId,
    const RouteIdType& routeId,
    const set<PublicVehicleIdType>& notAvailablePublicVehicleIds,
    const AgentResource& resource) const
{
    typedef list<shared_ptr<Train> >::const_iterator IterType;

    for(IterType iter = trainPtrs.begin();
        iter != trainPtrs.end(); iter++) {

        const Train& aTrain = *(*iter);

        if (aTrain.GetLineId() == lineId &&
            aTrain.GetRouteId() == routeId) {

            if ((notAvailablePublicVehicleIds.find(aTrain.GetPublicVehicleId()) == notAvailablePublicVehicleIds.end()) &&
                !aTrain.IsAlreadyRejectedAgent(resource.AgentId())) {
                return *iter;
            }
        }
    }

    return shared_ptr<Train>();
}

//-----------------------------------------------------------------

shared_ptr<Bus> AgentBusStop::GetBus(
    const PublicVehicleRoute& vehicleRoute,
    const set<PublicVehicleIdType>& notAvailablePublicVehicleIds,
    const AgentResource& resource) const
{
    typedef list<shared_ptr<Bus> >::const_iterator IterType;

    for(IterType iter = busPtrs.begin();
        iter != busPtrs.end(); iter++) {

        const Bus& bus = *(*iter);

        if (bus.GetLineId() == vehicleRoute.lineId &&
            bus.GetRouteId() == vehicleRoute.routeId) {

            if ((notAvailablePublicVehicleIds.find(bus.GetPublicVehicleId()) == notAvailablePublicVehicleIds.end()) &&
                !bus.IsAlreadyRejectedAgent(resource.AgentId())) {
                return (*iter);
            }
        }
    }

    return shared_ptr<Bus>();
}

shared_ptr<Bus> AgentBusStop::GetBus(
    const PublicVehicleRoute& vehicleRoute,
    const BusTicket& ticket,
    const AgentResource& resource) const
{
    typedef list<shared_ptr<Bus> >::const_iterator IterType;

    for(IterType iter = busPtrs.begin();
        iter != busPtrs.end(); iter++) {

        const Bus& bus = *(*iter);

        if (bus.GetLineId() == vehicleRoute.lineId &&
            bus.GetRouteId() == vehicleRoute.routeId) {

            if (ticket.lineId == vehicleRoute.lineId &&
                ticket.routeId == vehicleRoute.routeId &&
                ticket.reservedVehicleNumber == bus.GetVehicleNumber()) {

                if (!bus.IsAlreadyRejectedAgent(resource.AgentId())) {
                    return (*iter);
                }

            } else {

                if (!bus.IsAlreadyRejectedAgent(resource.AgentId())) {
                    return (*iter);
                }
            }
        }
    }

    return shared_ptr<Bus>();
}

bool AgentBusStop::HasLeft(
    const LineIdType& lineId,
    const RouteIdType& routeId,
    const VehicleNumberType& vehicleNumber) const
{
    typedef map<pair<LineIdType, RouteIdType>, set<VehicleNumberType> >::const_iterator IterType;

    IterType iter = leftVehicleNumbers.find(make_pair(lineId, routeId));

    if (iter == leftVehicleNumbers.end()) {
        return false;
    }

    const set<VehicleNumberType>& vehicleNumbers = (*iter).second;

    return (vehicleNumbers.find(vehicleNumber) != vehicleNumbers.end());
}

bool AgentStation::HasLeft(
    const LineIdType& lineId,
    const RouteIdType& routeId,
    const VehicleNumberType& vehicleNumber) const
{
    typedef map<pair<LineIdType, RouteIdType>, set<VehicleNumberType> >::const_iterator IterType;

    IterType iter = leftVehicleNumbers.find(make_pair(lineId, routeId));

    if (iter == leftVehicleNumbers.end()) {
        return false;
    }

    const set<VehicleNumberType>& vehicleNumbers = (*iter).second;

    return (vehicleNumbers.find(vehicleNumber) != vehicleNumbers.end());
}

shared_ptr<Taxi> AgentTaxiCenter::GetNearestWorkingTaxi(const Vertex& position) const
{
    typedef map<AgentIdType, shared_ptr<Taxi> >::const_iterator IterType;

    double minDistance = DBL_MAX;
    shared_ptr<Taxi> nearestTaxiPtr;

    for(IterType iter = workingTaxiPtrs.begin(); iter != workingTaxiPtrs.end(); iter++) {

        const shared_ptr<Taxi> taxiPtr = (*iter).second;
        const double distance = position.DistanceTo(taxiPtr->GetPosition());

        if (distance < minDistance) {
            minDistance = distance;
            nearestTaxiPtr = taxiPtr;
        }
    }

    return nearestTaxiPtr;
}

//-----------------------------------------------------------------

MultiAgentGis::MultiAgentGis(
    MultiAgentSimulator* initSimulatorPtr,
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngine>& simulationEnginePtr,
    const shared_ptr<GisSubsystem>& initGisSubsystemPtr)
    :
    simulatorPtr(initSimulatorPtr),
    theGisSubsystemPtr(initGisSubsystemPtr),
    navigationSystemUpdateInterval(initSimulatorPtr->TimeStep()),
    congestionNavigationSystemLastUpdatedTime(ZERO_TIME),
    SearchRoadRouteCallback(&MultiAgentGis::SearchRouteWithAStarAlgorithm)
{
    const vector<shared_ptr<const Road> > srcRoads = theGisSubsystemPtr->GetRoadPtrs();
    const vector<Intersection>& srcIntersections = theGisSubsystemPtr->GetIntersections();
    const vector<Building>& srcBuildings = theGisSubsystemPtr->GetBuildings();
    const vector<Park>& srcParks = theGisSubsystemPtr->GetParks();
    const vector<Poi>& srcPois = theGisSubsystemPtr->GetPois();
    const vector<Area>& srcAreas = theGisSubsystemPtr->GetAreas();
    const vector<RailRoadStation>& srcStations = theGisSubsystemPtr->GetStations();
    const vector<BusStop>& srcBusStops = theGisSubsystemPtr->GetBusStops();

    (*this).InitializeRoadMapping(parameterDatabaseReader, simulationEnginePtr, srcRoads);

    (*this).InitializeInterserctionMapping(parameterDatabaseReader, simulationEnginePtr, srcRoads, srcIntersections);

    (*this).InitializeBuildingMapping(parameterDatabaseReader, *this, simulationEnginePtr, srcBuildings);

    (*this).InitializeParkMapping(parameterDatabaseReader, *this, simulationEnginePtr, srcParks);
    
    (*this).InitializePoiMapping(parameterDatabaseReader, simulationEnginePtr, srcPois);

    (*this).InitializeAreaMapping(parameterDatabaseReader, simulationEnginePtr, srcAreas);

    (*this).InitializeStationMapping(parameterDatabaseReader, simulationEnginePtr, srcStations);

    (*this).InitializeBusStopMapping(parameterDatabaseReader, simulationEnginePtr, srcBusStops);

    (*this).InitializeOptimalConnection();

    (*this).InitializeIntersectionTurnResournce();

    (*this).InitializeRouteSearchPoints();

    if (parameterDatabaseReader.ParameterExists("multiagent-navigation-system-update-interval")) {
        navigationSystemUpdateInterval = std::max(
            navigationSystemUpdateInterval,
            parameterDatabaseReader.ReadTime("multiagent-navigation-system-update-interval"));
    }

}//MultiAgentGis()//




void MultiAgentGis::InitializeRoadMapping(
    const ParameterDatabaseReader& initParameterDatabaseReader,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const vector<shared_ptr<const Road> >& srcRoads)
{
    roadPtrs.resize(srcRoads.size());

    map<GisObjectIdType, pair<shared_ptr<AgentRoad::RoadStat>, vector<RoadIdType> > > roadStatPtrAndGroups;

    // road
    for(RoadIdType i = 0; i < RoadIdType(srcRoads.size()); i++) {
        const shared_ptr<const Road>& roadPtr = srcRoads[i];
        const Road& road = *roadPtr;
        const GisObjectIdType objectId = road.GetObjectId();

        typedef map<GisObjectIdType, pair<shared_ptr<AgentRoad::RoadStat>, vector<RoadIdType> > >::iterator IterType;

        IterType iter = roadStatPtrAndGroups.find(objectId);

        if (iter == roadStatPtrAndGroups.end()) {
            const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr =
                initSimulationEnginePtr->GetSimulationEngineInterface(
                    initParameterDatabaseReader, objectId);

            roadStatPtrAndGroups[objectId].first.reset(
                new AgentRoad::RoadStat(
                    simEngineInterfacePtr->CreateRealStat(modelName + "_Congestion")));
        }

        double roadSize = 1;
        if (!road.IsParking()) {
            roadSize = std::max<double>(1., CalculatePolygonSize(road.GetPolygon()));
        }

        pair<shared_ptr<AgentRoad::RoadStat>, vector<RoadIdType> >& roadStatPtrAndGroup = roadStatPtrAndGroups[objectId];

        roadStatPtrAndGroup.first->polygonSize += roadSize;
        roadStatPtrAndGroup.second.push_back(i);
    }

    for(size_t i = 0; i < srcRoads.size(); i++) {
        const shared_ptr<const Road>& roadPtr = srcRoads[i];
        const Road& road = *roadPtr;
        const GisObjectIdType objectId = road.GetObjectId();

        assert(roadStatPtrAndGroups.find(objectId) != roadStatPtrAndGroups.end());

        const pair<shared_ptr<AgentRoad::RoadStat>, vector<RoadIdType> >& roadStatPtrAndGroup =
            roadStatPtrAndGroups[objectId];

        roadPtrs[i].reset(
            new AgentRoad(
                roadPtr,
                roadStatPtrAndGroup.first,
                roadStatPtrAndGroup.second));
    }

}//InitializeRoadMapping//




void MultiAgentGis::InitializeInterserctionMapping(
    const ParameterDatabaseReader& initParameterDatabaseReader,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const vector<shared_ptr<const Road> >& srcRoads,
    const vector<Intersection>& srcIntersections)
{
    intersectionPtrs.resize(srcIntersections.size());

    // intersection
    for(IntersectionIdType i = 0; i < IntersectionIdType(srcIntersections.size()); i++) {
        const IntersectionIdType intersectionId = i;
        const Intersection& intersection = srcIntersections[intersectionId];
        const Vertex& basePoint = intersection.GetVertex();
        const vector<RoadIdType> roadIds = intersection.GetConnectedRoadIds();

        multimap<double, RoadIdType> roadIdPerRadians;

        for(size_t j = 0; j < roadIds.size(); j++) {
            const RoadIdType roadId = roadIds[j];
            const Road& road = *srcRoads[roadId];

            if (road.IsParking()) {
                continue;
            }

            const double directionRadians =
                (road.GetNeighborVertex(intersectionId) - basePoint).DirectionRadians();

            const vector<VertexIdType>& vertexIds = road.GetVertexIds();

            if (vertexIds.size() == 2 &&
                vertexIds.front() == vertexIds.back()) {
                continue;
            }

            roadIdPerRadians.insert(make_pair(directionRadians, roadId));
        }

        list<pair<double, RoadIdType> > sequencedRoadIdsOrderedByRadian;
        sequencedRoadIdsOrderedByRadian.insert(
            sequencedRoadIdsOrderedByRadian.end(),
            roadIdPerRadians.begin(), roadIdPerRadians.end());
        sequencedRoadIdsOrderedByRadian.insert(
            sequencedRoadIdsOrderedByRadian.end(),
            roadIdPerRadians.begin(), roadIdPerRadians.end());

        intersectionPtrs[i].reset(
            new AgentIntersection(
                srcIntersections[i],
                sequencedRoadIdsOrderedByRadian));
    }

}//InitializeInterserctionMapping//



void MultiAgentGis::InitializeBuildingMapping(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const MultiAgentGis& theMultiAgentGis,
    const shared_ptr<SimulationEngine>& simulationEnginePtr,
    const vector<Building>& srcBuildings)
{
    buildingPtrs.resize(srcBuildings.size());

    for(size_t i = 0; i < srcBuildings.size(); i++) {
        const Building& building = srcBuildings[i];
        const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr =
            simulationEnginePtr->GetSimulationEngineInterface(
                parameterDatabaseReader, building.GetObjectId());

        buildingPtrs[i].reset(
            new AgentBuilding(
                parameterDatabaseReader,
                theMultiAgentGis,
                building,
                simEngineInterfacePtr->CreateRealStat(modelName + "_Congestion"),
                simEngineInterfacePtr->CreateRealStat(modelName + "_Population")));
    }
}

void MultiAgentGis::InitializeParkMapping(
    const ParameterDatabaseReader& initParameterDatabaseReader,
    const MultiAgentGis& theMultiAgentGis,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const vector<Park>& srcParks)
{
    parkPtrs.resize(srcParks.size());

    for(size_t i = 0; i < srcParks.size(); i++) {
        const Park& park = srcParks[i];
        const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr =
            initSimulationEnginePtr->GetSimulationEngineInterface(
                initParameterDatabaseReader, park.GetObjectId());

        parkPtrs[i].reset(
            new AgentPark(
                initParameterDatabaseReader,
                theMultiAgentGis,
                park,
                simEngineInterfacePtr->CreateRealStat(modelName + "_Congestion"),
                simEngineInterfacePtr->CreateRealStat(modelName + "_Population")));
    }
}

void MultiAgentGis::InitializePoiMapping(
    const ParameterDatabaseReader& initParameterDatabaseReader,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const vector<Poi>& srcPois)
{
    poiPtrs.resize(srcPois.size());

    for(size_t i = 0; i < srcPois.size(); i++) {
        const Poi& poi = srcPois[i];
        const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr =
            initSimulationEnginePtr->GetSimulationEngineInterface(
                initParameterDatabaseReader, poi.GetObjectId());

        poiPtrs[i].reset(
            new AgentPoi(
                poi,
                simEngineInterfacePtr->CreateRealStat(modelName + "_Population")));
    }
}

void MultiAgentGis::InitializeAreaMapping(
    const ParameterDatabaseReader& initParameterDatabaseReader,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const vector<Area>& srcAreas)
{
    areaPtrs.resize(srcAreas.size());

    for(size_t i = 0; i < srcAreas.size(); i++) {
        const Area& area = srcAreas[i];

        areaPtrs[i].reset(new AgentArea(area));
    }
}

void MultiAgentGis::InitializeStationMapping(
    const ParameterDatabaseReader& initParameterDatabaseReader,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const vector<RailRoadStation>& srcStations)
{
    stationPtrs.resize(srcStations.size());

    for(size_t i = 0; i < srcStations.size(); i++) {
        const RailRoadStation& station = srcStations[i];
        const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr =
            initSimulationEnginePtr->GetSimulationEngineInterface(
                initParameterDatabaseReader, station.GetObjectId());

        stationPtrs[i].reset(
            new AgentStation(
                station,
                simEngineInterfacePtr->CreateRealStat(modelName + "_Congestion")));
    }
}

void MultiAgentGis::InitializeBusStopMapping(
    const ParameterDatabaseReader& initParameterDatabaseReader,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const vector<BusStop>& srcBusStops)
{
    busStopPtrs.resize(srcBusStops.size());

    for(size_t i = 0; i < srcBusStops.size(); i++) {
        const BusStop& busStop = srcBusStops[i];
        const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr =
            initSimulationEnginePtr->GetSimulationEngineInterface(
                initParameterDatabaseReader, busStop.GetObjectId());

        busStopPtrs[i].reset(
            new AgentBusStop(
                busStop,
                simEngineInterfacePtr->CreateRealStat(modelName + "_Congestion")));
    }

}

void MultiAgentGis::InitializeOptimalConnection()
{
    for(size_t i = 0; i < roadPtrs.size(); i++) {
        const shared_ptr<AgentRoad>& roadPtr = roadPtrs[i];
        AgentRoad& road = *roadPtr;

        const shared_ptr<AgentIntersection>& startIntersectionPtr =
            intersectionPtrs[road.gisRoadPtr->GetStartIntersectionId()];
        const shared_ptr<AgentIntersection>& endIntersectionPtr =
            intersectionPtrs[road.gisRoadPtr->GetEndIntersectionId()];

        road.startIntersectionPtr = startIntersectionPtr;
        road.endIntersectionPtr = endIntersectionPtr;

        const size_t numberStartToEndLanes = road.gisRoadPtr->GetNumberOfStartToEndLanes();
        const size_t numberEndToStartLanes = road.gisRoadPtr->GetNumberOfEndToStartLanes();

        for(size_t j = 0; j < numberStartToEndLanes; j++) {
            endIntersectionPtr->incomingRoads.push_back(
                AgentIntersection::IncomingRoad(roadPtr, j));
        }
        for(size_t j = 0; j < numberEndToStartLanes; j++) {
            startIntersectionPtr->incomingRoads.push_back(
                AgentIntersection::IncomingRoad(roadPtr, numberStartToEndLanes+j));
        }
    }
}

void MultiAgentGis::DisconnectGisConnection()
{
    for(size_t i = 0; i < roadPtrs.size(); i++) {
        AgentRoad& road = *roadPtrs[i];

        road.startIntersectionPtr.reset();
        road.endIntersectionPtr.reset();
    }

    for(size_t i = 0; i < intersectionPtrs.size(); i++) {
        AgentIntersection& intersection = *intersectionPtrs[i];

        intersection.incomingRoads.clear();
    }
}

void MultiAgentGis::InitializeIntersectionTurnResournce()
{
    map<IntersectionIdType, set<RoadIdType> > connectedRoadIds;

    for(size_t i = 0; i < intersectionPtrs.size(); i++) {
        AgentIntersection& intersection = *intersectionPtrs[i];

        const vector<RoadIdType> roadIds = intersection.ref.GetConnectedRoadIds();

        for(size_t j = 0; j < roadIds.size(); j++) {
            const RoadIdType roadId = roadIds[j];
            const Road& road = *roadPtrs[roadId]->gisRoadPtr;

            connectedRoadIds[road.GetStartIntersectionId()].insert(roadId);
            connectedRoadIds[road.GetEndIntersectionId()].insert(roadId);
        }
    }
}
//

void MultiAgentGis::SearchRoadRoute(
    const AgentResource& resource,
    const deque<VertexIdType>& startToDestVertexIds,
    const AgentBehaviorType& behavior,
    AgentRoute& roadRoute) const
{
    (*this).SearchRoadRoute(
        resource,
        startToDestVertexIds,
        behavior,
        AGENT_BEHAVIOR_ANY,
        ZERO_TIME,
        roadRoute);
}

void MultiAgentGis::SearchRoadRoute(
    const AgentResource& resource,
    const VertexIdType& startVertexId,
    const VertexIdType& destVertexId,
    const AgentBehaviorType& behavior,
    AgentRoute& roadRoute) const
{
    deque<VertexIdType> startToDestVertexIds;

    startToDestVertexIds.push_back(startVertexId);
    startToDestVertexIds.push_back(destVertexId);

    (*this).SearchRoadRoute(
        resource,
        startToDestVertexIds,
        behavior,
        AGENT_BEHAVIOR_ANY,
        ZERO_TIME,
        roadRoute);
}

void MultiAgentGis::SearchRoadRoute(
    const AgentResource& resource,
    const VertexIdType& startVertexId,
    const VertexIdType& destVertexId,
    const deque<VertexIdType>& shouldPassVertexIds,
    const AgentBehaviorType& behavior,
    AgentRoute& roadRoute) const
{
    deque<VertexIdType> startToDestVertexIds;

    startToDestVertexIds.push_back(startVertexId);
    startToDestVertexIds += shouldPassVertexIds;
    startToDestVertexIds.push_back(destVertexId);

    (*this).SearchRoadRoute(
        resource,
        startToDestVertexIds,
        behavior,
        AGENT_BEHAVIOR_ANY,
        ZERO_TIME,
        roadRoute);
}

//

void MultiAgentGis::SearchRoadRoute(
    const AgentResource& resource,
    const deque<VertexIdType>& startToDestVertexIds,
    const AgentBehaviorType& behavior,
    const AgentBehaviorType& preferenceBehavior,
    const TimeType& arrivalTimeMargin,
    AgentRoute& roadRoute) const
{
    assert(startToDestVertexIds.size() >= 2);

    const VertexIdType startVertexId = startToDestVertexIds.front();
    const VertexIdType destVertexId = startToDestVertexIds.back();

    assert(startVertexId != INVALID_VERTEX_ID);
    assert(destVertexId != INVALID_VERTEX_ID);

    if (startVertexId == destVertexId) {
        roadRoute = AgentRoute();
        return;
    }

    const RouteLey routeKey(startVertexId, destVertexId, behavior);

    if (noRoutes.find(routeKey) != noRoutes.end()) {
        roadRoute = AgentRoute();
        return;
    }

    for(size_t i = 0; i < startToDestVertexIds.size() - 1; i++) {
        const VertexIdType vertexId1 = startToDestVertexIds[i];
        const VertexIdType vertexId2 = startToDestVertexIds[i+1];

        (this->*(SearchRoadRouteCallback))(resource, vertexId1, vertexId2, behavior);

        const RouteSearchPoint& destPoint = routeSearchPoints[vertexId2];

        if (!destPoint.FoundRoute()) {
            roadRoute = AgentRoute();
            noRoutes.insert(RouteLey(vertexId1, vertexId2, behavior));
            //cout << "No route find " << startVertexId << " -> " << destVertexId << endl;
            return;
        }

        AgentRoute aRoadRoute;

        (*this).PickCalculatedRoute(vertexId1, vertexId2, aRoadRoute);

        roadRoute.totalCost += aRoadRoute.totalCost;
        roadRoute.roadRoutes += aRoadRoute.roadRoutes;
        //roadRoute.publicVehicleRoutes += roadRoute.publicVehicleRoutes;
    }

    roadRoute.behavior = behavior;

    roadRoute.totalCost.values[AGENT_ROUTE_COST_TRAVEL_TIME] += double(arrivalTimeMargin) / SECOND;

    if (preferenceBehavior != AGENT_BEHAVIOR_ANY &&
        behavior == preferenceBehavior) {

        roadRoute.totalCost.values[AGENT_ROUTE_COST_MODE] = 1;
    }
}

void MultiAgentGis::SearchRoadRoute(
    const AgentResource& resource,
    const VertexIdType& startVertexId,
    const VertexIdType& destVertexId,
    const AgentBehaviorType& behavior,
    const AgentBehaviorType& preferenceBehavior,
    const TimeType& arrivalTimeMargin,
    AgentRoute& roadRoute) const
{
    deque<VertexIdType> startToDestVertexIds;

    startToDestVertexIds.push_back(startVertexId);
    startToDestVertexIds.push_back(destVertexId);

    (*this).SearchRoadRoute(
        resource,
        startToDestVertexIds,
        behavior,
        preferenceBehavior,
        arrivalTimeMargin,
        roadRoute);
}

void MultiAgentGis::SearchRoadRoute(
    const AgentResource& resource,
    const VertexIdType& startVertexId,
    const VertexIdType& destVertexId,
    const deque<VertexIdType>& shouldPassVertexIds,
    const AgentBehaviorType& behavior,
    const AgentBehaviorType& preferenceBehavior,
    const TimeType& arrivalTimeMargin,
    AgentRoute& roadRoute) const
{
    deque<VertexIdType> startToDestVertexIds;

    startToDestVertexIds.push_back(startVertexId);
    startToDestVertexIds += shouldPassVertexIds;
    startToDestVertexIds.push_back(destVertexId);

    (*this).SearchRoadRoute(
        resource,
        startToDestVertexIds,
        behavior,
        preferenceBehavior,
        arrivalTimeMargin,
        roadRoute);
}


//

void MultiAgentGis::SearchRoadRoute(
    const AgentResource& resource,
    const deque<VertexIdType>& startToDestVertexIds,
    const TimeToSearchRoute& timeToSearchRoute,
    const AgentBehaviorType& behavior,
    const TimeType& currentTime,
    const TimeType& arrivalTimeMargin,
    const AgentBehaviorType& preferenceBehavior,
    AgentRouteList& roadRouteList) const
{
    shared_ptr<AgentRoute> routePtr(new AgentRoute());

    (*this).SearchRoadRoute(
        resource,
        startToDestVertexIds,
        behavior,
        preferenceBehavior,
        arrivalTimeMargin,
        *routePtr);

    const TimeType startTime = std::max(
        currentTime,
        timeToSearchRoute.CalculateDepartureTime(routePtr->totalCost.TravelTime()));

    routePtr->totalCost.values[AGENT_ROUTE_COST_ARRIVAL_TIME] =
        double(startTime + routePtr->totalCost.TravelTime())/SECOND;

    roadRouteList.PushRoute(routePtr);
}

void MultiAgentGis::SearchRoadRoute(
    const AgentResource& resource,
    const VertexIdType& startVertexId,
    const VertexIdType& destVertexId,
    const TimeToSearchRoute& timeToSearchRoute,
    const AgentBehaviorType& behavior,
    const TimeType& currentTime,
    const TimeType& arrivalTimeMargin,
    const AgentBehaviorType& preferenceBehavior,
    AgentRouteList& roadRouteList) const
{
    deque<VertexIdType> startToDestVertexIds;

    startToDestVertexIds.push_back(startVertexId);
    startToDestVertexIds.push_back(destVertexId);

    (*this).SearchRoadRoute(
        resource,
        startToDestVertexIds,
        timeToSearchRoute,
        behavior,
        currentTime,
        arrivalTimeMargin,
        preferenceBehavior,
        roadRouteList);
}

void MultiAgentGis::SearchRoadRoute(
    const AgentResource& resource,
    const VertexIdType& startVertexId,
    const VertexIdType& destVertexId,
    const deque<VertexIdType>& shouldPassVertexIds,
    const TimeToSearchRoute& timeToSearchRoute,
    const AgentBehaviorType& behavior,
    const TimeType& currentTime,
    const TimeType& arrivalTimeMargin,
    const AgentBehaviorType& preferenceBehavior,
    AgentRouteList& roadRouteList) const
{
    deque<VertexIdType> startToDestVertexIds;

    startToDestVertexIds.push_back(startVertexId);
    startToDestVertexIds += shouldPassVertexIds;
    startToDestVertexIds.push_back(destVertexId);

    (*this).SearchRoadRoute(
        resource,
        startToDestVertexIds,
        timeToSearchRoute,
        behavior,
        currentTime,
        arrivalTimeMargin,
        preferenceBehavior,
        roadRouteList);
}

//

void MultiAgentGis::CompleteInitialization(
    const ParameterDatabaseReader& initParameterDatabaseReader,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const AgentProfileAndTaskTable& theProfileAndTaskTable,
    const map<AgentIdType, Vertex>& taxiAgentLocations,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr)
{
    publicVehicleTablePtr = initPublicVehicleTablePtr;

    // allocate taxi

    vector<GisObjectType> prioritizedTaxiBaseSearchObjectTypes;
    map<GisPositionIdType, vector<AgentIdType> > taxiAgentIdsPerLocation;

    prioritizedTaxiBaseSearchObjectTypes.push_back(GIS_POI);
    prioritizedTaxiBaseSearchObjectTypes.push_back(GIS_RAILROAD_STATION);
    prioritizedTaxiBaseSearchObjectTypes.push_back(GIS_BUSSTOP);
    prioritizedTaxiBaseSearchObjectTypes.push_back(GIS_BUILDING);
    prioritizedTaxiBaseSearchObjectTypes.push_back(GIS_PARK);

    typedef map<AgentIdType, Vertex>::const_iterator AgentIter;

    const double defaultSearchDistance = 10.;

    for(AgentIter iter = taxiAgentLocations.begin();
        iter != taxiAgentLocations.end(); iter++) {

        const AgentIdType& agentId = (*iter).first;
        const Vertex& taxiPos = (*iter).second;
        const GisPositionIdType basePositionId =
            theGisSubsystemPtr->GetPositionId(taxiPos, prioritizedTaxiBaseSearchObjectTypes, defaultSearchDistance);

        if (!basePositionId.IsValid()) {
            cerr << "Error: Initial location for Taxi " << agentId << " is invalid." << endl
                 << "       Taxi must be placed on POI, Station, BusStop, Building or Park." << endl;
            exit(1);
        }

        taxiAgentIdsPerLocation[basePositionId].push_back(agentId);
    }

    typedef map<GisPositionIdType, vector<AgentIdType> >::const_iterator LocationIter;

    for(LocationIter iter = taxiAgentIdsPerLocation.begin();
        iter != taxiAgentIdsPerLocation.end(); iter++) {

        const GisPositionIdType& basePositionId = (*iter).first;
        const vector<AgentIdType>& agentIds = (*iter).second;
        const AgentTaxiCenterIdType taxiCenterId =
            static_cast<AgentTaxiCenterIdType>(taxiCenters.size());

        taxiCenters.push_back(AgentTaxiCenter(basePositionId));

        AgentTaxiCenter& taxiCenter = taxiCenters.back();

        for(size_t i = 0; i < agentIds.size(); i++) {
            const AgentIdType& agentId = agentIds[i];
            const AgentIter agentIter = taxiAgentLocations.find(agentId);

            assert(agentIter != taxiAgentLocations.end());

            const Vertex& taxiPos = (*agentIter).second;

            const VertexIdType taxiCenterVertexId =
                (*this).GetNearestVertexIdForTaxiCenter(
                    basePositionId,
                    taxiPos);

            const RoadIdType initRoadId =
                theGisSubsystemPtr->GetRoadId(taxiCenterVertexId);

            const size_t initLaneNumber =
                theGisSubsystemPtr->GetRoad(initRoadId).
                GetOutsideOutgoingLaneNumber(taxiCenterVertexId);

            const shared_ptr<Taxi> taxiPtr(
                new Taxi(
                    agentId,
                    taxiCenterId,
                    taxiCenterVertexId,
                    theGisSubsystemPtr->GetVertex(taxiCenterVertexId),
                    initRoadId,
                    initLaneNumber,
                    simulatorPtr));

            simulatorPtr->AddTaxi(taxiPtr);

            taxiCenter.idleTaxiPtrs.push_back(taxiPtr);
        }
    }

}//CompleteInitialization//

void MultiAgentGis::UpdateEntranceQueuesStatus()
{
    // Not efficient. Possibly keep active object list.

    const TimeType currentTime = simulatorPtr->CurrentTime();

    for (unsigned int i = 0; (i < buildingPtrs.size()); i++) {

        const BuildingIdType buildingId = static_cast<BuildingIdType>(i);
        bool peopleChanged;
        
        buildingPtrs[buildingId]->UpdateEntranceQueuesStatus(currentTime, peopleChanged);

        if (peopleChanged) {
            peopleChangedBuildingIds.push_back(buildingId);
        }//if//
    }//for//

    for (unsigned int i = 0; (i < parkPtrs.size()); i++) {

        const ParkIdType parkId = static_cast<ParkIdType>(i);
        bool peopleChanged;
        
        parkPtrs[parkId]->UpdateEntranceQueuesStatus(currentTime, peopleChanged);

        if (peopleChanged) {
            peopleChangedParkIds.push_back(parkId);
        }//if//
    }//for//

}//UpdateEntranceQueuesStatus//



void MultiAgentGis::MoveCalculatedNextStatesToCurrent()
{
    typedef list<RoadIdType>::const_iterator IterType;

    for(IterType roadIdIter = vehicleRoadIds.begin(); roadIdIter != vehicleRoadIds.end(); roadIdIter++) {

        const RoadIdType& roadId = *roadIdIter;
        AgentRoad& road = *roadPtrs.at(roadId);

        for(RoadDirectionType i = 0; i < RoadDirectionType(road.vehiclesPerDirection.size()); i++) {
            const RoadDirectionType direction(i);
            list<shared_ptr<Vehicle> >& vehicles = road.vehiclesPerDirection[direction];

            typedef list<shared_ptr<Vehicle> >::iterator VehicleIterType;

            for (VehicleIterType iter = vehicles.begin(); (iter != vehicles.end()); ++iter) {
                Vehicle& vehicle = *(*iter);
                vehicle.AdvanceStateToNextTimestep();
            }//while//
        }//for//
    }//for//

}//MoveCalculatedNextStatesToCurrent//




void MultiAgentGis::ExecuteActionsForCurrentTimestep()
{
    (*this).UpdateEntranceQueuesStatus();
    (*this).MoveCalculatedNextStatesToCurrent();
    (*this).SyncGisTopology();
    (*this).SyncVehicleAgentTopology();
    (*this).SyncPublicVehicleTopology(*publicVehicleTablePtr);
    (*this).SyncTaxiTopology();
    (*this).SyncCrossingPedestrianTopology();
    (*this).SyncGisStat();

}//ExecuteActionsForCurrentTimestep//


bool MultiAgentGis::CanReach(
    const AgentResource& resource,
    const VertexIdType& startVertexId,
    const VertexIdType& destVertexId,
    const AgentBehaviorType& behavior) const
{
    if (startVertexId == destVertexId) {
        return true;
    }

    const RouteLey routeKey(startVertexId, destVertexId, behavior);

    if (noRoutes.find(routeKey) != noRoutes.end()) {
        return false;
    }

    (this->*(SearchRoadRouteCallback))(
        resource,
        startVertexId,
        destVertexId,
        behavior);

    const RouteSearchPoint& destPoint =
        routeSearchPoints[destVertexId];

    if (!destPoint.FoundRoute()) {
        noRoutes.insert(routeKey);
        return false;
    }

    return true;
}

void MultiAgentGis::CreateVirtualRoute(
    const AgentResource& resource,
    const VertexIdType& startVertexId,
    const VertexIdType& destVertexId,
    const AgentBehaviorType& behavior,
    AgentRoute& roadRoute) const
{
    const RouteSearchPoint& destPoint = routeSearchPoints[destVertexId];

    vector<VertexIdType> startVertexIds;
    vector<VertexIdType> destVertexIds;

    startVertexIds.push_back(startVertexId);
    destVertexIds.push_back(destVertexId);

    const double maxExpandLength = 1000;
    double expandLength = 10;
    const Vertex& startPos = (*this).GetVertex(startVertexId);
    const Vertex& destPos = (*this).GetVertex(destVertexId);

    set<pair<VertexIdType, VertexIdType> > searchedPair;

    while (expandLength < maxExpandLength) {
        typedef vector<VertexIdType>::const_iterator IterType;

        vector<VertexIdType> vertexIds;

        theGisSubsystemPtr->GetVertexIds(Rectangle(startPos, expandLength), vertexIds);
        for(IterType iter = vertexIds.begin(); iter != vertexIds.end(); iter++) {
            bool canReach = false;
            for(size_t i = 0; (i < startVertexIds.size() && !canReach); i++) {
                canReach = (*this).CanReach(resource, startVertexIds[i], *iter, behavior);
            }
            if (!canReach) {
                startVertexIds.push_back(*iter);
            }
        }

        theGisSubsystemPtr->GetVertexIds(Rectangle(destPos, expandLength), vertexIds);
        for(IterType iter = vertexIds.begin(); iter != vertexIds.end(); iter++) {
            bool canReach = false;
            for(size_t i = 0; (i < destVertexIds.size() && !canReach); i++) {
                canReach = (*this).CanReach(resource, destVertexIds[i], *iter, behavior);
            }
            if (!canReach) {
                destVertexIds.push_back(*iter);
            }
        }

        for(size_t i = 0; i < startVertexIds.size(); i++) {
            const VertexIdType& virtualStartVertexId = startVertexIds[i];

            for(size_t j = 0; j < destVertexIds.size(); j++) {
                const VertexIdType& virtualDestVertexId = destVertexIds[j];

                const pair<VertexIdType, VertexIdType> routeKey(virtualStartVertexId, virtualDestVertexId);

                if (searchedPair.find(routeKey) != searchedPair.end()) {
                    continue;
                }

                (this->*(SearchRoadRouteCallback))(
                    resource,
                    virtualStartVertexId,
                    virtualDestVertexId,
                    behavior);

                if (destPoint.FoundRoute()) {
                    (*this).PickCalculatedRoute(virtualStartVertexId, virtualDestVertexId, roadRoute);

                    if (i != 0) {
                        roadRoute.roadRoutes.push_front(RoadRoute::CreateVirtualRoute(virtualStartVertexId));
                    }
                    if (virtualDestVertexId != destVertexId) {
                        roadRoute.roadRoutes.push_back(RoadRoute::CreateVirtualRoute(destVertexId));
                    }
                    return;
                }

                searchedPair.insert(routeKey);
            }
        }

        expandLength *= 2;
    }
}

void MultiAgentGis::InitializeRouteSearchPoints()
{
    const vector<GisVertex>& vertices = theGisSubsystemPtr->GetVertices();

    routeSearchPoints.resize(vertices.size());

    for(VertexIdType vertexId = 0; vertexId < VertexIdType(vertices.size()); vertexId++) {
        const GisVertex& gisVertex = vertices[vertexId];
        const map<GisObjectType, vector<VertexConnection> >& connections = gisVertex.connections;

        typedef map<GisObjectType, vector<VertexConnection> >::const_iterator IterType;

        RouteSearchPoint& searchPoint = routeSearchPoints[vertexId];
        searchPoint.vertexId = vertexId;

        IterType iter = connections.find(GIS_ROAD);

        if (iter != connections.end()) {
            const vector<VertexConnection>& connectedVertices = (*iter).second;
            const IntersectionIdType& intersectionId =
                theGisSubsystemPtr->GetIntersectionId(vertexId);

            for(size_t j = 0; j < connectedVertices.size(); j++) {
                const VertexConnection& connectedVertex = connectedVertices[j];

                searchPoint.links.push_back(
                    RoadConnection(
                        connectedVertex.vertexId,
                        connectedVertex.variantId,
                        intersectionId));
            }
        }
    }
}

void MultiAgentGis::InitializeRouteTable(
    const string& routeCachePath,
    const bool resetCache)
{
    routeTables = shared_array<fstream>(new fstream[NUMBER_ROUTE_ROAD_TOPOLOCIES]);

    for(size_t routeTableIndex = 0; routeTableIndex < NUMBER_ROUTE_ROAD_TOPOLOCIES; routeTableIndex++) {
        fstream& routeTable = routeTables[routeTableIndex];

        const string routeTableFileName =
            routeCachePath +
            modelName + "_routetable" +
            ConvertToString(routeTableIndex) + ".bin";

        if (resetCache || !fstream(routeTableFileName.c_str(), std::ios::in | std::ios::binary).good()) {
            routeTable.open(routeTableFileName.c_str(), std::ios::out | std::ios::binary);
            routeTable.close();

            if (MultiAgentSimulator::isDebugMode) {
                cout << "Create Route Cache:" << routeTableFileName << endl;
            }

            routeTable.open(routeTableFileName.c_str(), std::ios::in | std::ios::out | std::ios::binary);
            assert(routeTable.good());

            // make four bytes alighment

            assert(sizeof(SCENSIM_VERSION) == sizeof(double));
            assert(sizeof(routeTableIndex) == sizeof(int32_t));

            routeTable.write(reinterpret_cast<const char *>(&SCENSIM_VERSION), sizeof(double));
            routeTable.write(reinterpret_cast<const char *>(&routeTableIndex), sizeof(int32_t));

            RouteCache noRoute;

            for(size_t i = 0; i < routeSearchPoints.size(); i++) {
                for(size_t j = 0; j < routeSearchPoints.size(); j++) {
                    routeTable.write(
                        reinterpret_cast<const char *>(&noRoute),
                        sizeof(noRoute));
                }
            }
            routeTable << std::flush;

        } else {
            routeTable.open(routeTableFileName.c_str(), std::ios::in | std::ios::out | std::ios::binary);
            assert(routeTable.good());
        }
    }
}


class MultiAgentGis::RouteSearchQueue {
public:
    bool empty() const {
        while (!aQueue.empty() &&
               aQueue.top().ptr->totalCost.TravelTime() < aQueue.top().durationInQueuedTime) {
            aQueue.pop();
        }
        return aQueue.empty();
    }

    void push(const RouteSearchPoint* point) {
        aQueue.push(RouteSearchNode(point));
    }

    const RouteSearchPoint* pop() {
        assert(!(*this).empty());
        const RouteSearchPoint* point = aQueue.top().ptr;
        aQueue.pop();
        return point;
    }

private:

    class RouteSearchNode {
    public:
        RouteSearchNode()
            :
            ptr(),
            durationInQueuedTime(),
            expectedDurationInQueuedTime()
        {}

        RouteSearchNode(const RouteSearchPoint* const searchPoint)
            :
            ptr(searchPoint),
            durationInQueuedTime(ptr->totalCost.TravelTime()),
            expectedDurationInQueuedTime(durationInQueuedTime + ptr->expectedMinDurationToDest)
        {}

        bool operator<(const RouteSearchNode& right) const {
            return (expectedDurationInQueuedTime > right.expectedDurationInQueuedTime);
        }

    private:
        friend class RouteSearchQueue;

        const RouteSearchPoint* ptr;
        TimeType durationInQueuedTime;
        TimeType expectedDurationInQueuedTime;
    };

    mutable priority_queue_stable<RouteSearchNode> aQueue;
};

void MultiAgentGis::SearchRouteWithAStarAlgorithm(
    const AgentResource& resource,
    const VertexIdType& startVertexId,
    const VertexIdType& destVertexId,
    const AgentBehaviorType& behavior) const
{
    (*this).InitializeRoutePoints(startVertexId);

    RouteSearchQueue routeSearchQueue;
    routeSearchQueue.push(&routeSearchPoints[startVertexId]);

    TimeType minDurationToDest = INFINITE_TIME;
    const Vertex& destVertex = theGisSubsystemPtr->GetVertex(destVertexId);

    const AgentRoadRouteCost unreachableCost(
        INFINITE_TIME,
        DBL_MAX*0.5,
        DBL_MAX*0.5,
        INT_MAX/2,
        INT_MAX/2);

    while (!routeSearchQueue.empty()) {
        const RouteSearchPoint& aPoint = *routeSearchQueue.pop();
        const VertexIdType& lastVertexId = aPoint.vertexId;
        const vector<RoadConnection>& links = aPoint.links;

        if (aPoint.totalCost.TravelTime() + aPoint.expectedMinDurationToDest > minDurationToDest) {
            break;
        }

        for(size_t i = 0; i < links.size(); i++) {
            const RoadConnection& aLink = links[i];

            if (lastVertexId == aLink.vertexId) {
                continue;
            }

            if (!(*this).CanPassRoad(resource, aPoint.trackRoadId, behavior, lastVertexId, aLink)) {
                continue;
            }

            RouteSearchPoint& linkPoint = routeSearchPoints[aLink.vertexId];

            const AgentRoadRouteCost totalCost =
                aPoint.totalCost +
                (*this).CalculateCost(resource, behavior, lastVertexId, aLink.vertexId, aLink.roadId);

            if (linkPoint.expectedMinDurationToDest == ZERO_TIME) {
                const double speed = (*this).CalculateSpeedMetersPerSec(resource, behavior);
                const Vertex& linkVertex = theGisSubsystemPtr->GetVertex(aLink.vertexId);
                const double expectedMinDistanceToDest = destVertex.DistanceTo(linkVertex);

                linkPoint.expectedMinDistanceToDest = expectedMinDistanceToDest;
                linkPoint.expectedMinDurationToDest =
                    static_cast<TimeType>( (expectedMinDistanceToDest / speed) * SECOND);
            }

            if (totalCost.TravelTime() + linkPoint.expectedMinDurationToDest < minDurationToDest &&
                linkPoint.IsFastRoute(totalCost.TravelTime())) {

                linkPoint.SetBestRoute(lastVertexId, aLink.roadId, totalCost);
                assert(linkPoint.totalCost.travelDistance == totalCost.travelDistance);
                if (linkPoint.vertexId == destVertexId) {

                    minDurationToDest = totalCost.TravelTime();

                } else {

                    const vector<RoadConnection>& nextLinks = linkPoint.links;

                    if (nextLinks.size() <= 1) {
                        if (nextLinks.size() == 1) {
                            assert(nextLinks.front().vertexId == lastVertexId);
                        }
                        linkPoint.SetBestRoute(lastVertexId, aLink.roadId, unreachableCost);
                    } else {
                        routeSearchQueue.push(&linkPoint);
                    }
                }
            }
        }
    }
}

void MultiAgentGis::SearchRouteWithLearningRealTimeAStarAlgorithm(
    const AgentResource& resource,
    const VertexIdType& startVertexId,
    const VertexIdType& destVertexId,
    const AgentBehaviorType& behavior) const
{
    (*this).InitializeRoutePoints(startVertexId);

    RouteSearchQueue routeSearchQueue;
    routeSearchQueue.push(&routeSearchPoints[startVertexId]);

    VertexIdType lastVertexId = startVertexId;

    const Vertex& destVertex = theGisSubsystemPtr->GetVertex(destVertexId);
    set<VertexIdType> notSearchdVeretexIds;

    while (lastVertexId != destVertexId) {
        RouteSearchPoint& aPoint = routeSearchPoints[lastVertexId];
        const vector<RoadConnection>& links = aPoint.links;

        TimeType minLinkDurationToDest = INFINITE_TIME;
        VertexIdType nextVertexId = lastVertexId;

        assert(!links.empty());

        for(size_t i = 0; i < links.size(); i++) {
            const RoadConnection& aLink = links[i];

            if (lastVertexId == aLink.vertexId) {
                continue;
            }
            if (!(*this).CanPassRoad(resource, aPoint.trackRoadId, behavior, lastVertexId, aLink)) {
                continue;
            }

            RouteSearchPoint& linkPoint = routeSearchPoints[aLink.vertexId];

            if (!linkPoint.isSearchedVeretex) {
                notSearchdVeretexIds.insert(aLink.vertexId);
                linkPoint.isSearchedVeretex = true;
            }

            const AgentRoadRouteCost totalCost =
                aPoint.totalCost +
                (*this).CalculateCost(resource, behavior, lastVertexId, aLink.vertexId, aLink.roadId);

            if (linkPoint.expectedMinDurationToDest == ZERO_TIME) {
                const double speed = (*this).CalculateSpeedMetersPerSec(resource, behavior);
                const Vertex& linkVertex = theGisSubsystemPtr->GetVertex(aLink.vertexId);
                const double expectedMinDistanceToDest = destVertex.DistanceTo(linkVertex);

                linkPoint.expectedMinDistanceToDest = expectedMinDistanceToDest;
                linkPoint.expectedMinDurationToDest =
                    static_cast<TimeType>((expectedMinDistanceToDest / speed) * SECOND);
            }

            if (totalCost.TravelTime() + linkPoint.expectedMinDurationToDest < minLinkDurationToDest) {

                if (linkPoint.IsFastRoute(totalCost.TravelTime())) {
                    linkPoint.SetBestRoute(lastVertexId, aLink.roadId, totalCost);
                }

                minLinkDurationToDest = totalCost.TravelTime() + linkPoint.expectedMinDurationToDest;
                nextVertexId = aLink.vertexId;
            }
        }

        if (notSearchdVeretexIds.empty()) {
            return;
        }

        lastVertexId = nextVertexId;

        aPoint.expectedMinDurationToDest = std::max(
            aPoint.expectedMinDurationToDest, minLinkDurationToDest);

        notSearchdVeretexIds.erase(lastVertexId);
    }
}

double MultiAgentGis::CalculateSpeedMetersPerSec(
    const AgentResource& resource,
    const AgentBehaviorType& behavior) const
{
    if (behavior == AGENT_BEHAVIOR_PEDESTRIAN) {
        return resource.WalkSpeedMetersPerSec();
    } else if (behavior == AGENT_BEHAVIOR_BICYCLE) {
        return resource.BicycleSpeedMetersPerSec();
    } else {
        if ((behavior == AGENT_BEHAVIOR_TAXI) ||
            (behavior == AGENT_BEHAVIOR_TAXI_DRIVER)) {
            const AgentResource taxiAgentResource =
                simulatorPtr->GetMasterTaxiAgentResource();

            return taxiAgentResource.MaxVehicleSpeedMetersPerSec();

        } else {
            return resource.MaxVehicleSpeedMetersPerSec();
        }
    }
}

void MultiAgentGis::PickCalculatedRoute(
    const VertexIdType& startVertexId,
    const VertexIdType& destVertexId,
    AgentRoute& roadRoute) const
{
    const RouteSearchPoint& destPoint = routeSearchPoints[destVertexId];

    deque<RoadRoute>& roadRoutes = roadRoute.roadRoutes;
    roadRoutes.clear();

    assert(destPoint.FoundRoute());

    VertexIdType prevVertexId = destVertexId;

    do {
        const RouteSearchPoint& aPoint = routeSearchPoints[prevVertexId];

        roadRoutes.push_front(RoadRoute(aPoint.vertexId, aPoint.trackRoadId));
        prevVertexId = aPoint.trackVertexId;

    } while (prevVertexId != startVertexId);

    roadRoute.totalCost = destPoint.totalCost;
}

bool MultiAgentGis::CanPassRoad(
    const AgentResource& resource,
    const RoadIdType& lastRoadId,
    const AgentBehaviorType& behavior,
    const VertexIdType& startVertexId,
    const RoadConnection& aLink) const
{
    const Road& road = *roadPtrs[aLink.roadId]->gisRoadPtr;

    if (!road.IsEnabled()) {
        return false;
    }

    if (behavior == AGENT_BEHAVIOR_PEDESTRIAN ||
        behavior == AGENT_BEHAVIOR_BICYCLE) {

        return (road.PedestrianCanPass());

    } else {

        if (!intersectionPtrs[aLink.intersectionId]->ref.CanPassRoad(
                lastRoadId, aLink.roadId)) {
            return false;
        }

        if (road.IsStartVertex(startVertexId)) {

            if (road.GetNumberOfStartToEndLanes() == 0) {
                return false;
            }

        } else {

            if (road.GetNumberOfEndToStartLanes() == 0) {
                return false;
            }
        }

        if (behavior == AGENT_BEHAVIOR_BUS ||
            behavior == AGENT_BEHAVIOR_BUS_DRIVER) {

            return road.BusCanPass();

        } else if (behavior == AGENT_BEHAVIOR_VEHICLE ||
                   behavior == AGENT_BEHAVIOR_TAXI ||
                   behavior == AGENT_BEHAVIOR_TAXI_DRIVER) {
            return road.VehicleCanPass();
        }
    }

    return false;
}

void MultiAgentGis::InitializeRoutePoints(
    const VertexIdType& startVertexId) const
{
    for(size_t i = 0; i < routeSearchPoints.size(); i++) {
        routeSearchPoints[i].Initialize();
    }

    routeSearchPoints[startVertexId].totalCost.totalTravelTime = ZERO_TIME;
}

AgentRoadRouteCost MultiAgentGis::CalculateCost(
    const AgentResource& resource,
    const AgentBehaviorType& behavior,
    const VertexIdType& startVertexId,
    const VertexIdType& endVertexId,
    const RoadIdType& roadId) const
{
    const double minDistance = 0.00001; //1mm
    const double distance = std::max(minDistance, theGisSubsystemPtr->CalculateDistance(startVertexId, endVertexId));
    const AgentRoad& road = *roadPtrs.at(roadId);
    const size_t aLaneNumber = road.gisRoadPtr->GetIncomingLeftLaneNumber(startVertexId);

    double roadWidthWeight = resource.RoadWidthWeight();
    double roadWidthScale = road.gisRoadPtr->GetRoadWidthMeters() / theGisSubsystemPtr->GetRoadLayerPtr()->GetMaxRoadWidthMeters();

    if (roadWidthWeight < 0) {
        roadWidthScale = 1.0 - roadWidthScale;
        roadWidthWeight = - roadWidthWeight;
    }

    const double widthWeight = roadWidthScale*roadWidthWeight + (1.0 - roadWidthWeight);

    double speed;
    double gasolinCostPerMeters = 0;
    double density;

    if (behavior == AGENT_BEHAVIOR_PEDESTRIAN) {

        speed = resource.WalkSpeedMetersPerSec();
        density = road.CalculatePeopleSystemCongestion();

    } else if (behavior == AGENT_BEHAVIOR_BICYCLE) {

        speed = resource.BicycleSpeedMetersPerSec();
        density = road.CalculatePeopleSystemCongestion();

    } else {

        if (behavior == AGENT_BEHAVIOR_TAXI) {
            const AgentResource taxiAgentResource =
                simulatorPtr->GetMasterTaxiAgentResource();

            speed = taxiAgentResource.MaxVehicleSpeedMetersPerSec();
            gasolinCostPerMeters = taxiAgentResource.GasolinCostPerMeters();
            density = road.CalculateVehicleSystemCongestion(aLaneNumber);

        } else {
            speed = resource.MaxVehicleSpeedMetersPerSec();
            gasolinCostPerMeters = resource.GasolinCostPerMeters();
            density = road.CalculateVehicleSystemCongestion(aLaneNumber);
        }
    }

    const double minSpeedMeters = 1;
    const double maxSpeedMetersPerSec = std::max(speed * (1 - density) * widthWeight, minSpeedMeters);

    return AgentRoadRouteCost(
        static_cast<TimeType>((distance / maxSpeedMetersPerSec) * SECOND),
        distance,
        distance*gasolinCostPerMeters,
        road.GetNumberOfSystemPedestrians(),
        road.GetNumberOfSystemAverageVehicles(aLaneNumber));
}

VertexIdType MultiAgentGis::GetNearestVertexId(
    const GisPositionIdType& lastPositionId,
    const Vertex& position) const
{
    return theGisSubsystemPtr->GetNearestVertexId(lastPositionId, position);
}

const Vertex& MultiAgentGis::GetVertex(const VertexIdType& vertexId) const
{
    return theGisSubsystemPtr->GetVertex(vertexId);
}

void MultiAgentGis::AddVehicle(const shared_ptr<Vehicle>& vehiclePtr)
{
    newlyVehiclePtrs.push_back(vehiclePtr);
}


void MultiAgentGis::PedestrianOrDriverGiveUpEntrance(
    AgentResource& resource,
    const GisPositionIdType& desiredPositionId)
{
    if (!resource.WaitingAtEntrance()) {
        return;
    }

    switch (desiredPositionId.type) {
    case GIS_BUILDING: {
        AgentBuilding& building = *buildingPtrs[desiredPositionId.id];
        
        if (resource.GetBehaviorType() == AGENT_BEHAVIOR_VEHICLE) {
            AgentBuilding::NextVehicleStateInfoType& nextStateInfo =
                building.nextVehicleStateInfo;

            nextStateInfo.giveUpEntranceVehicleOwnerAgents.insert(resource);
            
            if (!nextStateInfo.vehiclesChanged) {
                vehiclesChangedBuildingIds.push_back(desiredPositionId.id);
                nextStateInfo.vehiclesChanged = true;
            }//if//
        }
        else {
            AgentBuilding::NextPeopleStateInfoType& nextStateInfo =
                building.nextPeopleStateInfo;
            
            nextStateInfo.giveUpEntranceAgents.push_back(resource);
            
            if (!nextStateInfo.peopleChanged) {
                peopleChangedBuildingIds.push_back(desiredPositionId.id);
                nextStateInfo.peopleChanged = true;
            }//if//
        }//if//

        break;
    }
    case GIS_PARK: {
        AgentPark& park = *parkPtrs[desiredPositionId.id];
        
        AgentBuilding::NextPeopleStateInfoType& nextStateInfo =
            park.nextPeopleStateInfo;

        nextStateInfo.giveUpEntranceAgents.push_back(resource);
        
        if (!nextStateInfo.peopleChanged) {
            peopleChangedBuildingIds.push_back(desiredPositionId.id);
            nextStateInfo.peopleChanged = true;
        }//if//

        break;
    }
    case GIS_POI: {
        AgentPoi::NextStateInfoType& poiNextState = poiPtrs[desiredPositionId.id]->nextStateInfo;

        poiNextState.giveUpEntranceAgents.push_back(resource);

        if (!poiNextState.peopleChanged) {
            peopleChangedPoiIds.push_back(desiredPositionId.id);
            poiNextState.peopleChanged = true;
        }
        break;
    }
    default:
        assert(desiredPositionId.type == GIS_BUILDING ||
               desiredPositionId.type == GIS_PARK ||
               desiredPositionId.type == GIS_POI);
        break;
    }
}

void MultiAgentGis::UpdatePeopleTranslationBetweenGisObjects(
    AgentResource& resource,
    const GisPositionIdType& currentPositionId,
    const GisPositionIdType& lastDesiredPositionId,
    const GisPositionIdType& desiredPositionId,
    const AgentBehaviorType& lastBehaviorType,
    const AgentBehaviorType& currentBehaviorType)
{
    assert(resource.GetCurrentPositionId() == currentPositionId);
    assert(resource.GetDesiredPositionId() == desiredPositionId);

    const bool positionChange =
        ((desiredPositionId != currentPositionId) &&
        (desiredPositionId != lastDesiredPositionId));

    const bool roadCapacityChange =
        (lastBehaviorType != currentBehaviorType &&
         (lastBehaviorType == AGENT_BEHAVIOR_PEDESTRIAN ||
          lastBehaviorType == AGENT_BEHAVIOR_BICYCLE ||
          currentBehaviorType == AGENT_BEHAVIOR_PEDESTRIAN ||
          currentBehaviorType == AGENT_BEHAVIOR_BICYCLE));

    assert(!positionChange || (currentPositionId == lastDesiredPositionId));

    if (positionChange) {
        // Position change notification

        resource.ArrivedAtGisPositionNotification();
    }

    if (!positionChange && !roadCapacityChange) {
        return;
    }

    if (currentPositionId.id != INVALID_VARIANT_ID) {
        switch (currentPositionId.type) {
        case GIS_ROAD: {
            AgentRoad::NextStateInfoType& roadNextState = roadPtrs[currentPositionId.id]->nextStateInfo;
            AgentRoad& road = *roadPtrs[currentPositionId.id];

            if (lastBehaviorType == AGENT_BEHAVIOR_PEDESTRIAN ||
                lastBehaviorType == AGENT_BEHAVIOR_BICYCLE) {
                roadNextState.numberLeavePeoples += resource.NumberPeople();
            }

            if (!roadNextState.peopleChanged) {
                peopleChangedRoadIds.push_back(currentPositionId.id);
                roadNextState.peopleChanged = true;
            }
            break;
        }
        case GIS_BUILDING: {
            AgentPeopleBuffer::NextPeopleStateInfoType& buildingNextStateInfo =
                buildingPtrs[currentPositionId.id]->nextPeopleStateInfo;

            buildingNextStateInfo.numberLeavePeoples += resource.NumberPeople();
            buildingNextStateInfo.outgoingAgents.push_back(make_pair(resource.ProfileType(), resource.AgentId()));
            assert(resource.AgentId() != 0);
            if (!buildingNextStateInfo.peopleChanged) {
                peopleChangedBuildingIds.push_back(currentPositionId.id);
                buildingNextStateInfo.peopleChanged = true;
            }
            break;
        }
        case GIS_PARK: {
            AgentPeopleBuffer::NextPeopleStateInfoType& parkNextStateInfo =
                parkPtrs[currentPositionId.id]->nextPeopleStateInfo;

            parkNextStateInfo.numberLeavePeoples += resource.NumberPeople();
            parkNextStateInfo.outgoingAgents.push_back(make_pair(resource.ProfileType(), resource.AgentId()));
            assert(resource.AgentId() != 0);
            if (!parkNextStateInfo.peopleChanged) {
                peopleChangedParkIds.push_back(currentPositionId.id);
                parkNextStateInfo.peopleChanged = true;
            }
            break;
        }
        case GIS_POI: {
            AgentPoi::NextStateInfoType& poiNextState = poiPtrs[currentPositionId.id]->nextStateInfo;

            poiNextState.numberLeavePeoples += resource.NumberPeople();

            if (!poiNextState.peopleChanged) {
                peopleChangedPoiIds.push_back(currentPositionId.id);
                poiNextState.peopleChanged = true;
            }
            break;
        }
        case GIS_BUSSTOP: {
            AgentBusStop::NextStateInfoType& busStopNextState = busStopPtrs[currentPositionId.id]->nextStateInfo;

            busStopNextState.numberLeavePeoples += resource.NumberPeople();

            if (!busStopNextState.peopleChanged) {
                peopleChangedBusStopIds.push_back(currentPositionId.id);
                busStopNextState.peopleChanged = true;
            }
            break;
        }
        case GIS_RAILROAD_STATION: {
            AgentStation::NextStateInfoType& stationNextState = stationPtrs[currentPositionId.id]->nextStateInfo;

            stationNextState.numberLeavePeoples += resource.NumberPeople();

            if (!stationNextState.peopleChanged) {
                peopleChangedStationIds.push_back(currentPositionId.id);
                stationNextState.peopleChanged = true;
            }
            break;
        }
        default:
            break;
        }
    }


    if (desiredPositionId.id == INVALID_VARIANT_ID) {
        peopleGoingToNowhere.push_back(resource);
    }
    else {
        switch (desiredPositionId.type) {
        case GIS_ROAD: {
            AgentRoad::NextStateInfoType& roadNextState = roadPtrs[desiredPositionId.id]->nextStateInfo;
            AgentRoad& road = *roadPtrs[desiredPositionId.id];

            roadNextState.incomingPeopleAgents.push_back(resource);

            if (currentBehaviorType == AGENT_BEHAVIOR_PEDESTRIAN ||
                currentBehaviorType == AGENT_BEHAVIOR_BICYCLE) {
                resource.WaitEntrance();
            }

            if (!roadNextState.peopleChanged) {
                peopleChangedRoadIds.push_back(desiredPositionId.id);
                roadNextState.peopleChanged = true;
            }
            break;
        }
        case GIS_BUILDING: {
            AgentPeopleBuffer::NextPeopleStateInfoType& buildingNextStateInfo =
                buildingPtrs[desiredPositionId.id]->nextPeopleStateInfo;

            buildingNextStateInfo.incomingPeopleAgents.push_back(resource);
            resource.WaitEntrance();

            if (!buildingNextStateInfo.peopleChanged) {
                peopleChangedBuildingIds.push_back(desiredPositionId.id);
                buildingNextStateInfo.peopleChanged = true;
            }
            break;
        }
        case GIS_PARK: {
            AgentPeopleBuffer::NextPeopleStateInfoType& parkNextState = 
                parkPtrs[desiredPositionId.id]->nextPeopleStateInfo;

            parkNextState.incomingPeopleAgents.push_back(resource);
            resource.WaitEntrance();

            if (!parkNextState.peopleChanged) {
                peopleChangedParkIds.push_back(desiredPositionId.id);
                parkNextState.peopleChanged = true;
            }
            break;
        }
        case GIS_POI: {
            AgentPoi::NextStateInfoType& poiNextState = poiPtrs[desiredPositionId.id]->nextStateInfo;

            poiNextState.incomingPeopleAgents.push_back(resource);
            resource.WaitEntrance();

            if (!poiNextState.peopleChanged) {
                peopleChangedPoiIds.push_back(desiredPositionId.id);
                poiNextState.peopleChanged = true;
            }
            break;
        }
        case GIS_BUSSTOP: {
            AgentBusStop::NextStateInfoType& busStopNextState = busStopPtrs[desiredPositionId.id]->nextStateInfo;

            busStopNextState.incomingPeopleAgents.push_back(resource);
            resource.WaitEntrance();

            if (!busStopNextState.peopleChanged) {
                peopleChangedBusStopIds.push_back(desiredPositionId.id);
                busStopNextState.peopleChanged = true;
            }
            break;
        }
        case GIS_RAILROAD_STATION: {
            AgentStation::NextStateInfoType& stationNextState = stationPtrs[desiredPositionId.id]->nextStateInfo;

            stationNextState.incomingPeopleAgents.push_back(resource);
            resource.WaitEntrance();

            if (!stationNextState.peopleChanged) {
                peopleChangedStationIds.push_back(desiredPositionId.id);
                stationNextState.peopleChanged = true;
            }
            break;
        }
        default:
            assert(false); abort();
            break;
        }
    }
}

void MultiAgentGis::TakeOwnershiopOfVehicleLastPositionUpdate(
    const AgentResource& resource,
    const shared_ptr<Vehicle>& vehiclePtr)
{
    lastPositionUpdateVehiclePtrs.push_back(vehiclePtr);
}

void MultiAgentGis::UpdateVehicleStatus(
    const AgentResource& resource,
    const shared_ptr<Vehicle>& vehiclePtr)
{
    statusUpdateVehicles.push_back(make_pair(vehiclePtr, resource));
}

void MultiAgentGis::RideOnPublicVehicle(
    const AgentResource& resource,
    const TimeType& waitTime,
    const shared_ptr<PublicVehicle>& publicVehiclePtr)
{
    addedPublicVehicleGuests.push_back(
        make_pair(TimeKey(-waitTime, resource.AgentId()), make_pair(publicVehiclePtr, resource)));
}

void MultiAgentGis::GetDownPublicVehicle(
    const AgentResource& resource,
    const shared_ptr<PublicVehicle>& publicVehiclePtr)
{
    removedPublicVehicleGuests.push_back(
        make_pair(publicVehiclePtr, resource));
}

struct SortVehiclesFromHead {
    bool operator()(
        const shared_ptr<Vehicle>& leftPtr,
        const shared_ptr<Vehicle>& rightPtr) const {
        return (leftPtr->GetDistanceToIntersection() < rightPtr->GetDistanceToIntersection());
    }
};

void MultiAgentGis::SyncVehicleAgentTopology()
{
    lastPositionUpdateVehiclePtrs.clear();

    list<shared_ptr<Vehicle> > transferVehicles;

    transferVehicles.splice(transferVehicles.end(), newlyVehiclePtrs);

    typedef list<RoadIdType>::const_iterator RoadIdIter;
    typedef list<shared_ptr<Vehicle> >::iterator VehicleIter;

    list<RoadIdType> newRoadIds;

    for(RoadIdIter roadIdIter = vehicleRoadIds.begin();
        roadIdIter != vehicleRoadIds.end(); roadIdIter++) {

        const RoadIdType& roadId = *roadIdIter;
        AgentRoad& road = *roadPtrs.at(roadId);

        road.containsSomeVehicle = false;

        for(RoadDirectionType i = 0; i < RoadDirectionType(road.vehiclesPerDirection.size()); i++) {
            const RoadDirectionType direction(i);
            list<shared_ptr<Vehicle> >& vehicles = road.vehiclesPerDirection[direction];

            vehicles.sort(SortVehiclesFromHead());

            VehicleIter iter = vehicles.begin();
            while (iter != vehicles.end()) {

                const Vehicle& vehicle = *(*iter);

                if (!vehicle.IsOnRoad()) {
                    iter = vehicles.erase(iter);
                } else if (!(vehicle.positionKey.roadId == roadId &&
                             vehicle.positionKey.roadId == vehicle.GetRoadId() &&
                             road.gisRoadPtr->GetRoadDirection(vehicle.GetLaneNumber()) == direction)) {
                    transferVehicles.push_back(*iter);
                    iter = vehicles.erase(iter);
                } else {
                    iter++;
                }
            }

            if (!vehicles.empty()) {
                road.containsSomeVehicle = true;
            }
        }

        if (road.containsSomeVehicle) {
            newRoadIds.push_back(roadId);
        }
    }

    for(VehicleIter iter = transferVehicles.begin(); iter != transferVehicles.end(); iter++) {
        Vehicle& vehicle = *(*iter);

        const RoadIdType currentRoadId = vehicle.GetRoadId();
        const size_t laneNumber = vehicle.GetLaneNumber();
        const double distanceToIntersection = vehicle.GetDistanceToIntersection();

        AgentRoad& road = *roadPtrs.at(currentRoadId);

        list<shared_ptr<Vehicle> >& vehicles =
            road.vehiclesPerDirection.at(road.gisRoadPtr->GetRoadDirection(laneNumber));

        VehicleIter lastIter = vehicles.end();

        if (!vehicles.empty()) {
            do {
                lastIter--;

                if ((*lastIter)->GetDistanceToIntersection() <= distanceToIntersection) {
                    lastIter++;
                    break;
                }
            } while (lastIter != vehicles.begin());
        }

        VehicleIter iterToList = vehicles.insert(lastIter, *iter);

        vehicle.positionKey.roadId = currentRoadId;
        vehicle.positionKey.laneNumber = laneNumber;
        vehicle.positionKey.iter = iterToList;
        vehicle.positionKey.isInitialized = true;

        if (!road.containsSomeVehicle) {
            road.containsSomeVehicle = true;
            newRoadIds.push_back(currentRoadId);
        }
    }

    vehicleRoadIds.clear();
    vehicleRoadIds.splice(vehicleRoadIds.end(), newRoadIds);

    for(size_t j = 0; j < statusUpdateVehicles.size(); j++) {
        const pair<shared_ptr<Vehicle>, AgentResource>& statusUpdateVehicle = statusUpdateVehicles[j];

        statusUpdateVehicle.first->UpdateVehicleConstant(
            statusUpdateVehicle.second.MakeVehicleConstant());
    }
    statusUpdateVehicles.clear();


    // GIS Stat depends on road status.
    //  - road[n].realInfo.lastNumberUpVehicles
    //  - road[n].realInfo.lastNumberDownVehicles
    (*this).SyncGisStat();
}




void MultiAgentGis::SetCrossingPeople(
    const RoadIdType& roadId,
    const AgentResource& resource)
{
    AgentRoad& road = *roadPtrs.at(roadId);

    AgentRoad::NextStateInfoType& roadNextState = road.nextStateInfo;

    const VertexIdType vertexId = resource.LastVertexId();
    const Vertex& position = resource.Position();

    if (vertexId == road.gisRoadPtr->GetStartVertexId()) {
        const int intersectionPosition =
            std::max(0, std::min(static_cast<int>(position.DistanceTo(road.startIntersectionAcrossBase)/road.roadGranurality), MAX_NUMBER_ROAD_CROSSING_GRANURALITIES - 1));

        roadNextState.crossingPeopleExistanceAtStartIntersection.set(intersectionPosition, true);

    } else {
        const int intersectionPosition =
            std::max(0, std::min(static_cast<int>(position.DistanceTo(road.endIntersectionAcrossBase)/road.roadGranurality), MAX_NUMBER_ROAD_CROSSING_GRANURALITIES - 1));

        roadNextState.crossingPeopleExistanceAtEndIntersection.set(intersectionPosition, true);
    }

    if (!roadNextState.crossingChanged) {
        crossingPedestrianChangeRoadIds.push_back(roadId);
        roadNextState.crossingChanged = true;
    }
}

void MultiAgentGis::SetWaitingPeople(
    const RoadIdType& roadId,
    const AgentResource& resource)
{
    AgentRoad& road = *roadPtrs.at(roadId);

    AgentRoad::NextStateInfoType& roadNextState = road.nextStateInfo;

    const VertexIdType vertexId = resource.LastVertexId();

    if (vertexId == road.gisRoadPtr->GetStartVertexId()) {
        roadNextState.waitingPeopleAtStartIntersection = true;
    } else {
        roadNextState.waitingPeopleAtEndIntersection = true;
    }

    if (!roadNextState.crossingChanged) {

        crossingPedestrianChangeRoadIds.push_back(roadId);
        roadNextState.crossingChanged = true;
    }
}

void MultiAgentGis::SetLeavePeople(
    const RoadIdType& roadId,
    const AgentResource& resource)
{
    AgentRoad& road = *roadPtrs.at(roadId);

    AgentRoad::NextStateInfoType& roadNextState = road.nextStateInfo;

    if (!roadNextState.crossingChanged) {
        crossingPedestrianChangeRoadIds.push_back(roadId);
        roadNextState.crossingChanged = true;
    }
}

void MultiAgentGis::SyncCrossingPedestrianTopology()
{
    vector<RoadIdType> uniqueRoadIds;

    for(size_t j = 0; j < crossingPedestrianChangeRoadIds.size(); j++) {
        const RoadIdType& roadId = crossingPedestrianChangeRoadIds[j];
        AgentRoad& road = *roadPtrs[roadId];

        if (road.needToExecuteCrossingPedestrianUpdateAtThisTime) {
            continue;
        }
        road.needToExecuteCrossingPedestrianUpdateAtThisTime = true;
        uniqueRoadIds.push_back(roadId);
    }

    // road
    //NotUsed: map<GisObjectIdType, RoadIdType> changedRoadObjectIds;

    for(size_t i = 0; i < uniqueRoadIds.size(); i++) {
        const RoadIdType& roadId = uniqueRoadIds[i];

        AgentRoad& road = *roadPtrs[roadId];

        road.waitingPeopleAtStartIntersection = false;
        road.waitingPeopleAtEndIntersection = false;
        road.crossingPeopleExistanceAtStartIntersection.reset();
        road.crossingPeopleExistanceAtEndIntersection.reset();

        AgentRoad::NextStateInfoType& roadNextState = road.nextStateInfo;

        road.waitingPeopleAtStartIntersection |=
            roadNextState.waitingPeopleAtStartIntersection;

        road.waitingPeopleAtEndIntersection |=
            roadNextState.waitingPeopleAtEndIntersection;

        road.crossingPeopleExistanceAtStartIntersection |=
            roadNextState.crossingPeopleExistanceAtStartIntersection;

        road.crossingPeopleExistanceAtEndIntersection |=
            roadNextState.crossingPeopleExistanceAtEndIntersection;

        roadNextState.crossingChanged = false;
        roadNextState.waitingPeopleAtStartIntersection = false;
        roadNextState.waitingPeopleAtEndIntersection = false;
        roadNextState.crossingPeopleExistanceAtStartIntersection.reset();
        roadNextState.crossingPeopleExistanceAtEndIntersection.reset();

        road.needToExecuteCrossingPedestrianUpdateAtThisTime = false;
    }
}



void MultiAgentGis::SyncPublicVehicleTopology(PublicVehicleTable& thePublicVehicleTable)
{
    (*this).SyncTrainTopology(thePublicVehicleTable);
    (*this).SyncBusTopology(thePublicVehicleTable);

    map<TimeKey, pair<shared_ptr<PublicVehicle>, AgentResource> > addedPublicVehicleGuestsMap;

    addedPublicVehicleGuestsMap.insert(
        addedPublicVehicleGuests.begin(),
        addedPublicVehicleGuests.end());

    for(size_t j = 0; j < removedPublicVehicleGuests.size(); j++) {
        const pair<shared_ptr<PublicVehicle>, AgentResource>& removedPublicVehicleGuest = removedPublicVehicleGuests[j];

        removedPublicVehicleGuest.first->Pop(removedPublicVehicleGuest.second);
    }

    addedPublicVehicleGuests.clear();
    removedPublicVehicleGuests.clear();

    const TimeType currentTime = simulatorPtr->CurrentTime();

    typedef map<TimeKey, pair<shared_ptr<PublicVehicle>, AgentResource> >::const_iterator IterType;

    for(IterType iter = addedPublicVehicleGuestsMap.begin();
        iter != addedPublicVehicleGuestsMap.end(); iter++) {

        const pair<shared_ptr<PublicVehicle>, AgentResource>& addPublicVehicleGuest = (*iter).second;

        PublicVehicle& publicVehicle = *addPublicVehicleGuest.first;
        const AgentResource& resource = addPublicVehicleGuest.second;
        const AgentIdType agentId = resource.AgentId();

        if (publicVehicle.IsAlreadyRejectedAgent(agentId)) {
            continue;
        }

        if (publicVehicle.CanTakeOn(resource)) {
            publicVehicle.Push(resource, currentTime);
        } else {
            publicVehicle.AddRejectedAgent(agentId);
        }
    }
}




void MultiAgentGis::AddTrain(const shared_ptr<Train>& trainPtr)
{
    trains.push_back(TrainEntry(trainPtr));
}

void MultiAgentGis::SyncTrainTopology(PublicVehicleTable& thePublicVehicleTable)
{
    typedef list<TrainEntry>::iterator IterType;

    IterType iter = trains.begin();

    while (iter != trains.end()) {
        TrainEntry& trainEntry = (*iter);
        Train& aTrain = *(trainEntry.trainPtr);

        const RailRoadStationIdType currentStationId = aTrain.GetStationId();

        // transfer train
        if (trainEntry.stationId != currentStationId) {
            if (trainEntry.stationId != INVALID_VARIANT_ID) {
                AgentStation& station = *stationPtrs.at(trainEntry.stationId);

                station.trainPtrs.erase(trainEntry.iterToTrain);

                if (aTrain.IsNeverVisitableStation(trainEntry.stationId)) {
                    station.leftVehicleNumbers[make_pair(aTrain.GetLineId(), aTrain.GetRouteId())].insert(aTrain.GetVehicleNumber());
                }
            }
            if (currentStationId != INVALID_VARIANT_ID) {
                list<shared_ptr<Train> >& trainPtrs =
                    stationPtrs.at(currentStationId)->trainPtrs;
                trainEntry.iterToTrain = trainPtrs.insert(
                    trainPtrs.end(), trainEntry.trainPtr);

                aTrain.ResetRejectedAgentIds();
            }

            trainEntry.stationId = currentStationId;
        }

        iter++;
    }
}

void MultiAgentGis::EnterToBusStop(
    const AgentResource& resource,
    const BusStopIdType& busStopId,
    const shared_ptr<Bus>& busPtr)
{
    enteredBusPtrs.push_back(make_pair(busStopId, busPtr));
}

void MultiAgentGis::SyncBusTopology(PublicVehicleTable& thePublicVehicleTable)
{
    const TimeType currentTime = simulatorPtr->CurrentTime();

    typedef list<StoppedBusEntry>::iterator IterType;

    IterType iter = stoppedBusEntry.begin();

    while(iter != stoppedBusEntry.end()) {

        const StoppedBusEntry& busEntry = (*iter);
        const Bus& bus = *busEntry.busPtr;

        if (bus.AtStop(currentTime)) {

            const TimeType delay = bus.GetCurrentDelay(currentTime);

            thePublicVehicleTable.SetStopDelay(
                bus.GetLineId(),
                bus.GetRouteId(),
                bus.GetStopNumber(),
                delay);

            thePublicVehicleTable.SetVehicleDelay(
                bus.GetLineId(),
                bus.GetRouteId(),
                bus.GetVehicleNumber(),
                delay);

            iter++;

        } else {
            AgentBusStop& busStop = *busStopPtrs.at(busEntry.busStopId);

            busStop.busPtrs.erase(busEntry.iterToBus);

            if (bus.IsNeverVisitableBusStop(busEntry.busStopId)) {
                busStop.leftVehicleNumbers[make_pair(bus.GetLineId(), bus.GetRouteId())].insert(bus.GetVehicleNumber());
            }

            iter = stoppedBusEntry.erase(iter);
        }
    }

    for(size_t j = 0; j < enteredBusPtrs.size(); j++) {
        const pair<BusStopIdType, shared_ptr<Bus> >& enteredBus = enteredBusPtrs[j];
        const BusStopIdType& busStopId = enteredBus.first;
        const shared_ptr<Bus>& busPtr = enteredBus.second;
        list<shared_ptr<Bus> >& busPtrs =
            busStopPtrs.at(busStopId)->busPtrs;

        stoppedBusEntry.push_back(
            StoppedBusEntry(busPtr, busStopId));

        stoppedBusEntry.back().iterToBus =
            busPtrs.insert(busPtrs.end(), busPtr);

        busPtr->ResetRejectedAgentIds();
    }

    enteredBusPtrs.clear();
}

void MultiAgentGis::SearchNearestTransportableTaxiCenter(
    const AgentResource& resource,
    bool& succeeded,
    AgentTaxiCenterIdType& transportableTaxiCenterId)
{
    const Vertex& position = resource.Position();

    const VertexIdType customerVertexId = resource.LastVertexId();
    const VertexIdType destinationVertexId = resource.DestVertexId();

    multimap<double, AgentTaxiCenterIdType> nearTaxiCenterIds;

    for(size_t i = 0; i < taxiCenters.size(); i++) {
        const AgentTaxiCenter& taxiCenter = taxiCenters[i];
        const double distance =
            position.DistanceTo(theGisSubsystemPtr->GetPositionVertex(taxiCenter.GetBasePositionId()));

        nearTaxiCenterIds.insert(make_pair(distance, static_cast<AgentTaxiCenterIdType>(i)));
    }

    typedef multimap<double, AgentTaxiCenterIdType>::const_iterator IterType;

    for(IterType iter = nearTaxiCenterIds.begin();
        iter != nearTaxiCenterIds.end(); iter++) {

        const AgentTaxiCenterIdType& taxiCenterId = (*iter).second;

        if ((*this).IsTransportableTaxiCenter(
                resource,
                position,
                taxiCenterId,
                customerVertexId,
                destinationVertexId)) {
            succeeded = true;
            transportableTaxiCenterId = taxiCenterId;
            return;
        }
    }

    succeeded = false;
}

bool MultiAgentGis::IsTransportableTaxiCenter(
    const AgentResource& resource,
    const Vertex& customerPosition,
    const AgentTaxiCenterIdType& taxiCenterId,
    const VertexIdType customerVertexId,
    const VertexIdType destinationVertexId) const
{
    const AgentTaxiCenter& taxiCenter = taxiCenters.at(taxiCenterId);
    const GisPositionIdType& basePositionId = taxiCenter.GetBasePositionId();

    const VertexIdType taxiCenterVertexId =
        (*this).GetNearestVertexIdForTaxiCenter(
            basePositionId,
            customerPosition);

    return ((*this).CanReach(resource, taxiCenterVertexId, customerVertexId, AGENT_BEHAVIOR_TAXI) &&
            (*this).CanReach(resource, customerVertexId, destinationVertexId, AGENT_BEHAVIOR_TAXI));
}

VertexIdType MultiAgentGis::GetNearestVertexIdForTaxiCenter(
    const GisPositionIdType& basePositionId,
    const Vertex& basePosition) const
{
    VertexIdType taxiCenterVertexId;

    switch (basePositionId.type) {
    case GIS_POI: taxiCenterVertexId = theGisSubsystemPtr->GetPoi(basePositionId.id).GetVertexId(); break;
    case GIS_RAILROAD_STATION: taxiCenterVertexId = theGisSubsystemPtr->GetStation(basePositionId.id).GetNearestEntranceVertexId(basePosition); break;
    case GIS_BUSSTOP: taxiCenterVertexId = theGisSubsystemPtr->GetBusStop(basePositionId.id).GetNearestEntranceVertexId(basePosition); break;
    case GIS_BUILDING: taxiCenterVertexId = theGisSubsystemPtr->GetBuilding(basePositionId.id).GetNearestEntranceVertexId(basePosition); break;
    case GIS_PARK: taxiCenterVertexId = theGisSubsystemPtr->GetPark(basePositionId.id).GetNearestEntranceVertexId(basePosition); break;
    default:
        assert(false); abort(); taxiCenterVertexId = 0;
        break;
    }

    return taxiCenterVertexId;
}

void MultiAgentGis::GetNearRouteSearchCandidateVertexIds(
    const AgentResource& resource,
    const Vertex& basePosition,
    const AgentBehaviorType& behaviorType,
    const GisPositionIdType& positionId,
    const VertexIdType& prioritizedDestVertexId,
    vector<VertexIdType>& vertexIds)
{
    const TimeType currentTime = resource.CurrentTime();
    const TimeType arrivalTimeMargin = ZERO_TIME;

    TimeToSearchRoute notUsedTimeToSearchRoute;

    vertexIds.clear();

    set<VertexIdType> reachableVertexIds;
    vector<VertexIdType> vertexIdCandisates;

    theGisSubsystemPtr->GetNearEntranceVertexIds(positionId, basePosition, vertexIdCandisates);

    if (prioritizedDestVertexId != INVALID_VERTEX_ID) {
        // move specified vertex to top
        for(size_t i = 0; i < vertexIdCandisates.size(); i++) {

            if (vertexIdCandisates[i] == prioritizedDestVertexId) {
                vertexIdCandisates.erase(vertexIdCandisates.begin() + i);
                vertexIdCandisates.insert(vertexIdCandisates.begin(), prioritizedDestVertexId);
                break;
            }
        }
    }

    // mark reachable vertex between vertices
    for(size_t i = 0; i < vertexIdCandisates.size(); i++) {
        const VertexIdType& vertexId1 = vertexIdCandisates[i];

        if (reachableVertexIds.find(vertexId1) != reachableVertexIds.end()) {
            continue;
        }

        for(size_t j = i+1; j < vertexIdCandisates.size(); j++) {
            const VertexIdType& vertexId2 = vertexIdCandisates[j];

            if (reachableVertexIds.find(vertexId2) != reachableVertexIds.end()) {
                continue;
            }

            AgentRouteList roadRoute;

            (*this).SearchRoadRoute(
                resource,
                vertexId1,
                vertexId2,
                notUsedTimeToSearchRoute,
                behaviorType,
                currentTime,
                arrivalTimeMargin,
                behaviorType,
                roadRoute);

            if (!roadRoute.IsEmpty()) {
                reachableVertexIds.insert(vertexId2);
            }
        }
    }

    // pick representative vertices.
    for(size_t i = 0; i < vertexIdCandisates.size(); i++) {
        const VertexIdType& vertexId = vertexIdCandisates[i];

        if (reachableVertexIds.find(vertexId) == reachableVertexIds.end()) {
            vertexIds.push_back(vertexId);
        }
    }
}

void MultiAgentGis::PlaceInitialVehicle(
    AgentResource& vehicleOwner,
    const BuildingIdType& buildingId)
{
    AgentBuilding::NextVehicleStateInfoType& buildingNextStateInfo =
        buildingPtrs[buildingId]->nextVehicleStateInfo;

    buildingNextStateInfo.initialVehicleOwnerAgents.push_back(vehicleOwner);

    if (!buildingNextStateInfo.vehiclesChanged) {
        buildingNextStateInfo.vehiclesChanged = true;
        vehiclesChangedBuildingIds.push_back(buildingId);
    }//if//
}

void MultiAgentGis::StartVehicleEntranceWait(
    AgentResource& vehicleOwner,
    const BuildingIdType& buildingId)
{
    AgentBuilding::NextVehicleStateInfoType& buildingNextStateInfo =
        buildingPtrs[buildingId]->nextVehicleStateInfo;
    
    buildingNextStateInfo.incomingVehicleOwnerAgents.push_back(vehicleOwner);
    vehicleOwner.WaitEntrance();

    if (!buildingNextStateInfo.vehiclesChanged) {
        buildingNextStateInfo.vehiclesChanged = true;
        vehiclesChangedBuildingIds.push_back(buildingId);
    }//if//
}

void MultiAgentGis::LeaveFromParking(
    AgentResource& vehicleOwner,
    const BuildingIdType& buildingId)
{
    AgentBuilding::NextVehicleStateInfoType& buildingNextStateInfo =
        buildingPtrs[buildingId]->nextVehicleStateInfo;
    
    buildingNextStateInfo.outgoingVehicleOwnerAgents.push_back(vehicleOwner);

    if (!buildingNextStateInfo.vehiclesChanged) {
        buildingNextStateInfo.vehiclesChanged = true;
        vehiclesChangedBuildingIds.push_back(buildingId);
    }//if//
}


void MultiAgentGis::CallTaxi(
    const AgentResource& resource,
    const AgentTaxiCenterIdType& requestTaxiCenterId,
    bool& succeeded,
    AgentTaxiCenterIdType& replyTaxiCenterId)
{
    if (requestTaxiCenterId == ANY_TAXICENTER_ID) {

        if (taxiCenters.empty()) {
            succeeded = false;
            return;
        }

        (*this).SearchNearestTransportableTaxiCenter(
            resource,
            succeeded,
            replyTaxiCenterId);

        if (!succeeded) {
            return;
        }

    } else {
        const Vertex& position = resource.Position();

        const VertexIdType customerVertexId = resource.LastVertexId();
        const VertexIdType destinationVertexId = resource.DestVertexId();

        if (!(*this).IsTransportableTaxiCenter(
                resource,
                position,
                requestTaxiCenterId,
                customerVertexId,
                destinationVertexId)) {
            succeeded = false;
            return;
        }

        replyTaxiCenterId = requestTaxiCenterId;
    }

    taxiReservations.insert(
        make_pair(
            resource.AgentId(),
            TaxiReservation(resource.CurrentTime(), resource, replyTaxiCenterId)));

    succeeded = true;
}

void MultiAgentGis::GiveupTaxiCall(
    const AgentResource& resource,
    const AgentTaxiCenterIdType& taxiCenterId)
{
    giveupTaxiCalls.push_back(make_pair(taxiCenterId, resource));
}

void MultiAgentGis::ReturnTaxiToStation(
    const AgentResource& resource,
    const shared_ptr<Taxi>& taxiPtr)
{
    returnTaxiPtrs.push_back(taxiPtr);
}

void MultiAgentGis::SyncTaxiTopology()
{
    set<AgentTaxiCenterIdType> taxiCenterIdsNeedToAssignTaxi;

    // cancel waiting reservations

    for(size_t j = 0; j < giveupTaxiCalls.size(); j++) {
        const pair<AgentTaxiCenterIdType, AgentResource>& giveupTaxiCall = giveupTaxiCalls[j];
        const AgentTaxiCenterIdType taxiCenterId = giveupTaxiCall.first;
        const AgentResource customerResource = giveupTaxiCall.second;

        AgentTaxiCenter& taxiCenter = taxiCenters[taxiCenterId];

        taxiCenter.reservedAgents.remove(customerResource);
    }

    giveupTaxiCalls.clear();

    // return idle taxi

    for(size_t j = 0; j < returnTaxiPtrs.size(); j++) {
        const shared_ptr<Taxi>& taxiPtr = returnTaxiPtrs[j];
        const AgentTaxiCenterIdType taxiCenterId = taxiPtr->GetHomeTaxiCenterId();
        const AgentIdType driverAgentId = taxiPtr->GetDriverAgentId();

        AgentTaxiCenter& taxiCenter = taxiCenters[taxiCenterId];

        taxiCenter.workingTaxiPtrs.erase(driverAgentId);
        taxiCenter.idleTaxiPtrs.push_back(taxiPtr);

        if (!taxiCenter.reservedAgents.empty()) {
            taxiCenterIdsNeedToAssignTaxi.insert(taxiCenterId);
        }
    }

    returnTaxiPtrs.clear();

    // merge taxi reservations

    typedef map<AgentIdType, TaxiReservation>::const_iterator ReservationIter;

    for(ReservationIter reservationIter = taxiReservations.begin();
        reservationIter != taxiReservations.end(); reservationIter++) {
        const TaxiReservation& reservation = (*reservationIter).second;
        const AgentTaxiCenterIdType& taxiCenterId = reservation.taxiCenterId;

        AgentTaxiCenter& taxiCenter = taxiCenters.at(taxiCenterId);

        taxiCenter.reservedAgents.push_back(reservation.resource);

        if (taxiCenter.HasIdleTaxi()) {
            taxiCenterIdsNeedToAssignTaxi.insert(taxiCenterId);
        }
    }

    taxiReservations.clear();

    // assign taxi

    typedef set<AgentTaxiCenterIdType>::const_iterator TaxiCenterIdIter;

    for(TaxiCenterIdIter iter = taxiCenterIdsNeedToAssignTaxi.begin();
        iter != taxiCenterIdsNeedToAssignTaxi.end(); iter++) {

        const AgentTaxiCenterIdType& taxiCenterId = (*iter);

        AgentTaxiCenter& taxiCenter = taxiCenters.at(taxiCenterId);
        list<AgentResource>& reservedAgents = taxiCenter.reservedAgents;
        list<shared_ptr<Taxi> >& idleTaxiPtrs = taxiCenter.idleTaxiPtrs;

        while (!reservedAgents.empty() && !idleTaxiPtrs.empty()) {

            AgentResource customerResource = reservedAgents.front();
            shared_ptr<Taxi> taxiPtr = idleTaxiPtrs.front();

            customerResource.AssignTaxi(taxiPtr);

            simulatorPtr->OutputTrace(
                "Assign Taxi " + ConvertToString(taxiPtr->GetDriverAgentId()) +
                " for " + ConvertToString(customerResource.AgentId()));

            reservedAgents.pop_front();
            idleTaxiPtrs.pop_front();

            taxiCenter.workingTaxiPtrs[taxiPtr->GetDriverAgentId()] = taxiPtr;
        }
    }
}

enum {
    ROAD_TRACE_NUMBER_UP_VEHICLES,
    ROAD_TRACE_NUMBER_DOWN_VEHICLES,
    ROAD_TRACE_NUMBER_PEDESTRIANS,

    NUMBER_ROAD_TRACES,
};




void MultiAgentGis::RemoveAgentFromCurrentGisLocation(AgentResource& agent)
{
    const GisPositionIdType positionId = agent.GetCurrentPositionId();

    if (positionId.IsInvalid()) {
        return;
    }

    switch (positionId.type) {
    case GIS_BUILDING: {
        const BuildingIdType& buildingId = positionId.id;
        AgentBuilding& building = *buildingPtrs[buildingId];

        building.numberPeoples -= agent.NumberPeople();
        building.needToOutputStatsAtThisTime = true;

        break;
    }

    case GIS_ROAD: {
        const RoadIdType& roadId = positionId.id;
        AgentRoad& road = *roadPtrs[roadId];

        if ((agent.GetLastTimestepBehaviorType() == AGENT_BEHAVIOR_PEDESTRIAN) ||
            (agent.GetLastTimestepBehaviorType() == AGENT_BEHAVIOR_BICYCLE)) {

            assert(road.realInfo.lastNumberPedestrians >= agent.NumberPeople());
            road.realInfo.lastNumberPedestrians -= agent.NumberPeople();
            road.needToOutputStatsAtThisTime = true;
        }

        break;
    }

    case GIS_PARK: {
        const ParkIdType& parkId = positionId.id;
        AgentPark& park = *parkPtrs[parkId];

        park.numberPeoples -= agent.NumberPeople();
        park.needToOutputStatsAtThisTime = true;

        break;
    }

    case GIS_POI: {
        const PoiIdType& poiId = positionId.id;
        AgentPoi& poi = *poiPtrs[poiId];

        poi.numberPeoples -= agent.NumberPeople();
        poi.needToOutputStatsAtThisTime = true;

        break;
    }

    case GIS_BUSSTOP: {
        const BusStopIdType& busStopId = positionId.id;
        AgentBusStop& busStop = *busStopPtrs[busStopId];

        busStop.numberPeoples -= agent.NumberPeople();
        busStop.needToOutputStatsAtThisTime = true;

        break;
    }

    case GIS_RAILROAD_STATION: {
        const RailRoadStationIdType& stationId = positionId.id;
        AgentStation& station = *stationPtrs[stationId];

        station.numberPeoples -= agent.NumberPeople();
        station.needToOutputStatsAtThisTime = true;

        break;
    }

    default:
        assert(false); abort();
        break;

    }//switch//

}//RemoveAgentFromCurrentGisLocation//


void MultiAgentGis::TransferPeopleToGisObject(
    const GisObjectType& anObjectType,
    const VariantIdType& objectId,
    bool& somethingChanged)
{
    const TimeType currentTime = simulatorPtr->CurrentTime();

    somethingChanged = false;

    switch (anObjectType) {
    case GIS_ROAD: {

        const RoadIdType& roadId = objectId;

        AgentRoad& road = *roadPtrs[roadId];

        AgentRoad::NavigationSystemInfo& realInfo = road.realInfo;

        realInfo.lastNumberUpVehicles = (int)(road.vehiclesPerDirection[ROAD_DIRECTION_UP].size());
        realInfo.lastNumberDownVehicles = (int)(road.vehiclesPerDirection[ROAD_DIRECTION_DOWN].size());

        list<AgentResource>& incomingPeopleAgents = road.nextStateInfo.incomingPeopleAgents;

        road.nextStateInfo.numberLeavePeoples = 0;
        road.nextStateInfo.peopleChanged = false;

        // Force enter to road to share route information(:no route cache) between pedestrians

        while ((!incomingPeopleAgents.empty()) &&
               (realInfo.lastNumberPedestrians + 1 <= road.gisRoadPtr->GetHumanCapacity())) {

            AgentResource& person = incomingPeopleAgents.front();

            // Don't count non-pedestrian/bicyclists.

            (*this).RemoveAgentFromCurrentGisLocation(person);
            person.UpdateCurrentPositionIdToDesiredPositionId();

            if ((person.GetBehaviorType() == AGENT_BEHAVIOR_PEDESTRIAN) ||
                (person.GetBehaviorType() == AGENT_BEHAVIOR_BICYCLE)) {

                realInfo.lastNumberPedestrians += person.NumberPeople();
                person.AllowedEntrance();
            }

            somethingChanged = true;
            incomingPeopleAgents.pop_front();

        }//while//

        if (somethingChanged) {
            road.needToOutputStatsAtThisTime = true;
        }//if//

        break;
    }

    case GIS_BUILDING: {

        const BuildingIdType& buildingId = objectId;

        buildingPtrs[buildingId]->TransferPeopleToGisObject(
            currentTime,
            GisPositionIdType(GIS_BUILDING, buildingId),
            *this);

        break;
    }

    case GIS_PARK: {

        const ParkIdType& parkId = objectId;

        parkPtrs[parkId]->TransferPeopleToGisObject(
            currentTime,
            GisPositionIdType(GIS_PARK, parkId),
            *this);
        
        break;
    }

    case GIS_POI: {

        const PoiIdType& poiId = objectId;

        AgentPoi& poi = *poiPtrs[poiId];

        list<AgentResource>& incomingPeopleAgents = poi.nextStateInfo.incomingPeopleAgents;

        poi.nextStateInfo.numberLeavePeoples = 0;
        poi.nextStateInfo.peopleChanged = false;

        list<AgentResource>& giveUpEntranceAgents = poi.nextStateInfo.giveUpEntranceAgents;

        typedef list<AgentResource>::iterator IterType;

        for(IterType iter = giveUpEntranceAgents.begin(); iter != giveUpEntranceAgents.end(); iter++) {
            AgentResource& resource = (*iter);

            resource.SetDesiredNextPositionId(resource.GetCurrentPositionId());
            incomingPeopleAgents.remove(resource);
            somethingChanged = true;
        }
        giveUpEntranceAgents.clear();

        while (!incomingPeopleAgents.empty() &&
               poi.numberPeoples + incomingPeopleAgents.front().NumberPeople() <= poi.ref.GetHumanCapacity()) {

            AgentResource& resource = incomingPeopleAgents.front();

            poi.numberPeoples += resource.NumberPeople();
            resource.UpdateCurrentPositionIdToDesiredPositionId();
            resource.AllowedEntrance();
            somethingChanged = true;

            incomingPeopleAgents.pop_front();
        }

        if (somethingChanged) {
            poi.needToOutputStatsAtThisTime = true;
        }//if//

        break;
    }

    case GIS_BUSSTOP: {

        const BusStopIdType& busStopId = objectId;

        AgentBusStop& busStop = *busStopPtrs[busStopId];

        list<AgentResource>& incomingPeopleAgents = busStop.nextStateInfo.incomingPeopleAgents;

        busStop.nextStateInfo.numberLeavePeoples = 0;
        busStop.nextStateInfo.peopleChanged = false;

        while (!incomingPeopleAgents.empty() &&
               busStop.numberPeoples + incomingPeopleAgents.front().NumberPeople() <= busStop.ref.GetHumanCapacity()) {

            AgentResource& resource = incomingPeopleAgents.front();

            busStop.numberPeoples += resource.NumberPeople();
            resource.UpdateCurrentPositionIdToDesiredPositionId();
            resource.AllowedEntrance();
            somethingChanged = true;

            incomingPeopleAgents.pop_front();
        }

        if (somethingChanged) {
            busStop.needToOutputStatsAtThisTime = true;
        }//if//


        break;
    }

    case GIS_RAILROAD_STATION: {

        const RailRoadStationIdType& stationId = objectId;

        AgentStation& station = *stationPtrs[stationId];

        list<AgentResource>& incomingPeopleAgents = station.nextStateInfo.incomingPeopleAgents;

        station.nextStateInfo.numberLeavePeoples = 0;
        station.nextStateInfo.peopleChanged = false;

        while (!incomingPeopleAgents.empty() &&
               station.numberPeoples + incomingPeopleAgents.front().NumberPeople() <= station.ref.GetHumanCapacity()) {

            AgentResource& resource = incomingPeopleAgents.front();

            station.numberPeoples += resource.NumberPeople();
            resource.UpdateCurrentPositionIdToDesiredPositionId();
            resource.AllowedEntrance();
            somethingChanged = true;
            incomingPeopleAgents.pop_front();
        }

        if (somethingChanged) {
            station.needToOutputStatsAtThisTime = true;
        }//if//

        break;
    }

    default:
        assert(false); abort();
        break;

    }//switch//

}//TransferPeopleToGisObject//


void MultiAgentGis::TransferEntranceWaitingVehicleDriver()
{
    // Building Vehicle count hack.
 
    for(size_t i = 0; i < vehiclesChangedBuildingIds.size(); i++) {
        const BuildingIdType& buildingId = vehiclesChangedBuildingIds[i];
        AgentBuilding& building = *buildingPtrs[buildingId];
        AgentBuilding::NextVehicleStateInfoType& nextStateInfo = building.nextVehicleStateInfo;

        nextStateInfo.vehiclesChanged = false;
        
        typedef list<AgentResource>::iterator IterType;
            
        list<AgentResource>& initialVehicleOwnerAgents =
            nextStateInfo.initialVehicleOwnerAgents;

        list<AgentResource>& incomingVehicleOwnerAgents =
            nextStateInfo.incomingVehicleOwnerAgents;

        list<AgentResource>& outgoingVehicleOwnerAgents =
            nextStateInfo.outgoingVehicleOwnerAgents;

        set<AgentResource>& giveUpEntranceVehicleOwnerAgents =
            nextStateInfo.giveUpEntranceVehicleOwnerAgents;

        // Remove give up vehicles from incoming list

        if (!giveUpEntranceVehicleOwnerAgents.empty()) {
            IterType iter = incomingVehicleOwnerAgents.begin();
            
            while (iter != incomingVehicleOwnerAgents.end()) {

                if (giveUpEntranceVehicleOwnerAgents.find(*iter) != giveUpEntranceVehicleOwnerAgents.end()) {

                    AgentResource& vehicleOwner = (*iter);
                    vehicleOwner.EndEntranceWaiting();

                    iter = incomingVehicleOwnerAgents.erase(iter);
                }
                else {
                    iter++;
                }//if//
            }//while//
        }//if//

        if (initialVehicleOwnerAgents.size() > static_cast<size_t>(building.ref.GetVehicleCapacity())) {
            cerr << "Warning: Vehicle Capacity Exceeded for building Id: "
                 << buildingId << " : " << initialVehicleOwnerAgents.size() << " / "
                 << building.ref.GetVehicleCapacity() << endl;
        }//if//

        // Force insert initial vehicle without checking capacity.

        for(IterType iter = initialVehicleOwnerAgents.begin();
            (iter != initialVehicleOwnerAgents.end()); iter++) {
            building.parkingVehicleOwnerAgents.insert(*iter);
        }//for//

        // Remove outgoing(or leaving) vehicle from parking

        for(IterType iter = outgoingVehicleOwnerAgents.begin();
            (iter != outgoingVehicleOwnerAgents.end()); iter++) {
            building.parkingVehicleOwnerAgents.erase(*iter);
        }//for//
        
        // Process incoming vehicle

        while (!incomingVehicleOwnerAgents.empty() &&
               building.parkingVehicleOwnerAgents.size() < static_cast<size_t>(building.ref.GetVehicleCapacity())) {

            AgentResource& resource = incomingVehicleOwnerAgents.front();

            assert(resource.GetBehaviorType() == AGENT_BEHAVIOR_VEHICLE);

            building.parkingVehicleOwnerAgents.insert(resource);
            resource.AllowedEntrance();
            incomingVehicleOwnerAgents.pop_front();
        }//while//

        initialVehicleOwnerAgents.clear();
        outgoingVehicleOwnerAgents.clear();
        giveUpEntranceVehicleOwnerAgents.clear();

    }//if//

    vehiclesChangedBuildingIds.clear();
}//TransferEntranceWaitingVehicleDriver//

//
// SyncGisStat: Combines and reconciles agent actions in previous timestep.
//

void MultiAgentGis::SyncGisStat()
{
    using std::min;

    const TimeType currentTime = simulatorPtr->CurrentTime();

    simulatorPtr->OutputAgentTraces();



    (*this).TransferEntranceWaitingVehicleDriver();

    //-------------------------------------------

    struct PeopleTransferListElemType {
        GisObjectType anObjectType;
        VariantIdType objectId;
        double orderMetric;

        PeopleTransferListElemType() { }

        PeopleTransferListElemType(
            const GisObjectType& initAnObjectType,
            const VariantIdType& initObjectId,
            const double& initOrderMetric)
        :
            anObjectType(initAnObjectType),
            objectId(initObjectId),
            orderMetric(initOrderMetric)
        {}

        // Reverse order (high to low sorting).
        bool operator<(const PeopleTransferListElemType& right)
            { return (orderMetric > right.orderMetric); }
    };

    vector<PeopleTransferListElemType> gisObjectsWithTransfers;

    for(size_t j = 0; j < peopleChangedRoadIds.size(); j++) {
        const RoadIdType& roadId = peopleChangedRoadIds[j];
        AgentRoad& road = *roadPtrs[roadId];
        AgentRoad::NavigationSystemInfo& realInfo = road.realInfo;

        road.nextStateInfo.peopleChanged = false;
        road.needToOutputStatsAtThisTime = false;

        const double orderMetric =
            min<double>(
                static_cast<double>(road.nextStateInfo.incomingPeopleAgents.size()),
                (road.gisRoadPtr->GetHumanCapacity() - realInfo.lastNumberPedestrians));

        gisObjectsWithTransfers.push_back(
            PeopleTransferListElemType(GIS_ROAD, roadId, orderMetric));

    }//for//


    for(size_t j = 0; j < peopleChangedBuildingIds.size(); j++) {
        const BuildingIdType& buildingId = peopleChangedBuildingIds[j];
        AgentBuilding& building = *buildingPtrs[buildingId];

        building.nextPeopleStateInfo.peopleChanged = false;
        building.needToOutputStatsAtThisTime = false;

        const double orderMetric =
            min<double>(
                static_cast<double>(building.nextPeopleStateInfo.incomingPeopleAgents.size()),
                (building.ref.GetHumanCapacity() - building.numberPeoples));

        gisObjectsWithTransfers.push_back(
            PeopleTransferListElemType(GIS_BUILDING, buildingId, orderMetric));
    }//for//


    for(size_t j = 0; j < peopleChangedParkIds.size(); j++) {
        const ParkIdType& parkId = peopleChangedParkIds[j];
        AgentPark& park = *parkPtrs[parkId];

        park.nextPeopleStateInfo.peopleChanged = false;
        park.needToOutputStatsAtThisTime = false;

        const double orderMetric =
            min<double>(
                static_cast<double>(park.nextPeopleStateInfo.incomingPeopleAgents.size()),
                (park.ref.GetHumanCapacity() - park.numberPeoples));

        gisObjectsWithTransfers.push_back(
            PeopleTransferListElemType(GIS_PARK, parkId, orderMetric));

    }//for//


    for(size_t j = 0; j < peopleChangedPoiIds.size(); j++) {
        const PoiIdType& poiId = peopleChangedPoiIds[j];
        AgentPoi& poi = *poiPtrs[poiId];

        poi.nextStateInfo.peopleChanged = false;
        poi.needToOutputStatsAtThisTime = false;

        const double orderMetric =
            min<double>(
                static_cast<double>(poi.nextStateInfo.incomingPeopleAgents.size()),
                (poi.ref.GetHumanCapacity() - poi.numberPeoples));

        gisObjectsWithTransfers.push_back(
            PeopleTransferListElemType(GIS_POI, poiId, orderMetric));

    }//for//


    for(size_t j = 0; j < peopleChangedBusStopIds.size(); j++) {
        const BusStopIdType& busStopId = peopleChangedBusStopIds[j];
        AgentBusStop& busStop = *busStopPtrs[busStopId];

        busStop.nextStateInfo.peopleChanged = false;
        busStop.needToOutputStatsAtThisTime = false;

        const double orderMetric =
            min<double>(
                static_cast<double>(busStop.nextStateInfo.incomingPeopleAgents.size()),
                (busStop.ref.GetHumanCapacity() - busStop.numberPeoples));

        gisObjectsWithTransfers.push_back(
            PeopleTransferListElemType(GIS_BUSSTOP, busStopId, orderMetric));

    }//for//

    for(size_t j = 0; j < peopleChangedStationIds.size(); j++) {
        const RailRoadStationIdType& stationId = peopleChangedStationIds[j];
        AgentStation& station = *stationPtrs[stationId];

        station.nextStateInfo.peopleChanged = false;
        station.needToOutputStatsAtThisTime = false;

        const double orderMetric =
            min<double>(
                static_cast<double>(station.nextStateInfo.incomingPeopleAgents.size()),
                (station.ref.GetHumanCapacity() - station.numberPeoples));

        gisObjectsWithTransfers.push_back(
            PeopleTransferListElemType(GIS_RAILROAD_STATION, stationId, orderMetric));

    }//for//

    //-------------------------------------------------------

    //Disable for comparing: std::sort(gisObjectsWithIncomingTransfers.begin(), gisObjectsWithIncomingTransfers.end());

    for (size_t i = 0; (i < peopleGoingToNowhere.size()); i++) {
        AgentResource& person = peopleGoingToNowhere[i];
        RemoveAgentFromCurrentGisLocation(person);
        person.UpdateCurrentPositionIdToDesiredPositionId();
    }//for//

    peopleGoingToNowhere.clear();


    bool isFinished = false;
    while (!isFinished) {
        isFinished = true;

        for(unsigned int i = 0; (i < gisObjectsWithTransfers.size()); i++) {

            bool somethingChanged;

            (*this).TransferPeopleToGisObject(
                gisObjectsWithTransfers[i].anObjectType,
                gisObjectsWithTransfers[i].objectId,
                /*out*/somethingChanged);

            if (somethingChanged) {
                isFinished = false;
            }//if//
        }//for//
    }//while//



    //-------------------------------------------------------
    // Output Stats:
    //

    //Jay Mimicking exact previous behavior.

    map<GisObjectIdType, RoadIdType> changedRoadObjectIds;

    for(size_t j = 0; j < peopleChangedRoadIds.size(); j++) {
        const RoadIdType& roadId = peopleChangedRoadIds[j];
        AgentRoad& road = *roadPtrs[roadId];
        changedRoadObjectIds.insert(make_pair(road.gisRoadPtr->GetObjectId(), roadId));
    }//for//

    typedef map<GisObjectIdType, RoadIdType>::const_iterator IterType;

    for(IterType iter = changedRoadObjectIds.begin();
        iter != changedRoadObjectIds.end(); iter++) {

        const GisObjectIdType& objectId = (*iter).first;
        const RoadIdType& roadId = (*iter).second;
        AgentRoad& road = *roadPtrs[roadId];

        if (true || road.needToOutputStatsAtThisTime) {
            road.needToOutputStatsAtThisTime = false;

            AgentRoad::NavigationSystemInfo& realInfo = road.realInfo;

            const GisObjectIdType& roadGisObjectId = road.gisRoadPtr->GetObjectId();
            const vector<RoadIdType>& groupedRoadIds = road.groupedRoadIds;

            int numberAgents = 0;

            for(size_t i = 0; i < groupedRoadIds.size(); i++) {
                const AgentRoad& groupedRoad = *roadPtrs.at(groupedRoadIds[i]);

                numberAgents +=
                    groupedRoad.realInfo.lastNumberUpVehicles +
                    groupedRoad.realInfo.lastNumberDownVehicles +
                    groupedRoad.realInfo.lastNumberPedestrians;
            }

            AgentRoad::RoadStat& roadStat = *road.roadStatPtr;

            const double congestion = numberAgents/roadStat.polygonSize;

            if (road.gisRoadPtr->MasTraceIsOn() && road.gisRoadPtr->IsEnabled()) {
                TraceValue<double>& congestionTrace = roadStat.congestionTrace;

                congestionTrace.SetValue(congestion);

                if (congestionTrace.HasChanged()) {
                    simulatorPtr->OutputTraceEvent(
                        roadGisObjectId, "Gis_Congestion", congestionTrace.GetValueAndUnsetChangeFlag());
                }
            }

            roadStat.congestionStatPtr->RecordStatValue(congestion);

        }//if//
    }//for//

    for(size_t j = 0; j < peopleChangedRoadIds.size(); j++) {
        const RoadIdType& roadId = peopleChangedRoadIds[j];
        AgentRoad& road = *roadPtrs[roadId];
        road.needToOutputStatsAtThisTime = false;
    }//for//


    for(size_t j = 0; j < peopleChangedBuildingIds.size(); j++) {
        const BuildingIdType& buildingId = peopleChangedBuildingIds[j];
        AgentBuilding& building = *buildingPtrs[buildingId];

        if (true || building.needToOutputStatsAtThisTime) {
            building.needToOutputStatsAtThisTime = false;

            const double congestion = building.numberPeoples/building.polygonSize;

            if (building.ref.MasTraceIsOn()) {

                TraceValue<double>& congestionTrace = building.congestionTrace;

                congestionTrace.SetValue(congestion);

                if (congestionTrace.HasChanged()) {
                    simulatorPtr->OutputTraceEvent(
                        building.ref.GetObjectId(), "Gis_Congestion", congestionTrace.GetValueAndUnsetChangeFlag());
                }//if//
            }//if//

            building.congestionStatPtr->RecordStatValue(congestion);
            building.populationStatPtr->RecordStatValue(building.numberPeoples);

        }//if//
    }//for//


    for(size_t j = 0; j < peopleChangedParkIds.size(); j++) {
        const ParkIdType& parkId = peopleChangedParkIds[j];
        AgentPark& park = *parkPtrs[parkId];

        if (true || park.needToOutputStatsAtThisTime) {
            park.needToOutputStatsAtThisTime = false;

            const double congestion = park.numberPeoples/park.polygonSize;

            if (park.ref.MasTraceIsOn()) {
                TraceValue<double>& congestionTrace = park.congestionTrace;

                congestionTrace.SetValue(congestion);

                if (congestionTrace.HasChanged()) {
                    simulatorPtr->OutputTraceEvent(
                        park.ref.GetObjectId(), "Gis_Congestion", congestionTrace.GetValueAndUnsetChangeFlag());
                }
            }

            park.congestionStatPtr->RecordStatValue(congestion);
            park.populationStatPtr->RecordStatValue(park.numberPeoples);

        }//if//
    }//for//


    for(size_t j = 0; j < peopleChangedPoiIds.size(); j++) {
        const PoiIdType& poiId = peopleChangedPoiIds[j];
        AgentPoi& poi = *poiPtrs[poiId];

        if (true || poi.needToOutputStatsAtThisTime) {
            poi.needToOutputStatsAtThisTime = false;

            poi.populationStatPtr->RecordStatValue(poi.numberPeoples);

        }//if//
    }//for//


    for(size_t j = 0; j < peopleChangedBusStopIds.size(); j++) {
        const BusStopIdType& busStopId = peopleChangedBusStopIds[j];
        AgentBusStop& busStop = *busStopPtrs[busStopId];

        if (true || busStop.needToOutputStatsAtThisTime) {
            busStop.needToOutputStatsAtThisTime = false;

            const double congestion = busStop.numberPeoples/busStop.polygonSize;

            if (busStop.ref.MasTraceIsOn()) {
                TraceValue<double>& congestionTrace = busStop.congestionTrace;

                congestionTrace.SetValue(congestion);

                if (congestionTrace.HasChanged()) {
                    simulatorPtr->OutputTraceEvent(
                        busStop.ref.GetObjectId(), "Gis_Congestion", congestionTrace.GetValueAndUnsetChangeFlag());
                }
            }

            busStop.congestionStatPtr->RecordStatValue(congestion);
        }//if//

    }//for//

    for(size_t j = 0; j < peopleChangedStationIds.size(); j++) {
        const RailRoadStationIdType& stationId = peopleChangedStationIds[j];
        AgentStation& station = *stationPtrs[stationId];

        if (true || station.needToOutputStatsAtThisTime) {
            station.needToOutputStatsAtThisTime = false;

            const double congestion = station.numberPeoples/station.polygonSize;

            if (station.ref.MasTraceIsOn()) {
                TraceValue<double>& congestionTrace = station.congestionTrace;

                congestionTrace.SetValue(congestion);

                if (congestionTrace.HasChanged()) {
                    simulatorPtr->OutputTraceEvent(
                        station.ref.GetObjectId(), "Gis_Congestion", congestionTrace.GetValueAndUnsetChangeFlag());
                }
            }

            station.congestionStatPtr->RecordStatValue(congestion);

        }//if//
    }//for//

    //----------------------------------------

    if (navigationSystemUpdateInterval != INFINITE_TIME) {
        bool updateNavigationInformation = false;

        while (currentTime >= congestionNavigationSystemLastUpdatedTime + navigationSystemUpdateInterval) {
            congestionNavigationSystemLastUpdatedTime += navigationSystemUpdateInterval;
            updateNavigationInformation = true;
        }

        if (updateNavigationInformation) {
            (*this).UpdateCongestionNavigationSystemInformation();
        }
    }

    // GIS enable/disable status update
    theGisSubsystemPtr->ExecuteEvents(currentTime);

    if (theGisSubsystemPtr->ChangedGisTopologyAtThisTime()) {
        noRoutes.clear();
    }

    (*this).peopleChangedRoadIds.clear();
    (*this).peopleChangedBuildingIds.clear();
    (*this).peopleChangedParkIds.clear();
    (*this).peopleChangedPoiIds.clear();
    (*this).peopleChangedBusStopIds.clear();
    (*this).peopleChangedStationIds.clear();

}//SyncGisStat//



void MultiAgentGis::UpdateCongestionNavigationSystemInformation()
{
    for(size_t i = 0; i < roadPtrs.size(); i++) {
        AgentRoad& road = *roadPtrs[i];

        road.systemInfo = road.realInfo;
    }

    for(size_t i = 0; i < busStopPtrs.size(); i++) {
        AgentBusStop& busStop = *busStopPtrs[i];

        busStop.naviInfo.lastNumberPeoples = busStop.numberPeoples;
    }

    for(size_t i = 0; i < stationPtrs.size(); i++) {
        AgentStation& station = *stationPtrs[i];

        station.naviInfo.lastNumberPeoples = station.numberPeoples;
    }
}

void MultiAgentGis::SyncGisTopology()
{
    simulatorPtr->SyncGisTopology();
}

}//namespace ScenSim
