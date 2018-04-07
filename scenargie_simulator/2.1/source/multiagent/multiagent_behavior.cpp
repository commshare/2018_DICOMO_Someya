// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "multiagent_behavior.h"
#include "multiagent_agentsim.h"
#include "multiagent_routesearch.h"

namespace MultiAgent {


static inline
Vertex CalculateNextVertex(const Vertex& nextPos, const Vertex& directionVector, const double remainingPathDistance)
{
    return nextPos - directionVector*remainingPathDistance;
}

//---------------------------------------------------------------------------

FreeWalkBehavior::FreeWalkBehavior(
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<AgentRoute>& initRoutePtr,
    const GisPositionIdType& initEndPositionId,
    const bool initEnterToPosition,
    const AgentResource& initResource)
    :
    AgentBehavior(initAgentGisPtr, initPublicVehicleTablePtr, initRoutePtr, initResource),
    remainingPathDistance(0),
    endPositionId(initEndPositionId),
    extraDestPoinId(resource.ExtraDestPoiId()),
    enterToPosition(initEnterToPosition)
{
    const Vertex& currentPos = resource.Position();

    if (enterToPosition) {
        const GisPositionIdType& extraDestPoiId = resource.ExtraDestPoiId();

        if (extraDestPoiId.IsValid()) {
            assert(extraDestPoiId.type == GIS_POI);
            nextPos = theAgentGisPtr->GetAgentPoi(extraDestPoiId.id).ref.GetVertex();

        } else {
            const GisPositionIdType& destPositionId = resource.DestPositionId();

            if (destPositionId.type == GIS_BUILDING) {
                nextPos = theAgentGisPtr->GetAgentBuilding(destPositionId.id).ref.
                    GetRandomPosition(resource.GetRandomNumberGenerator());
            } else if (destPositionId.type == GIS_PARK) {
                nextPos = theAgentGisPtr->GetAgentPark(destPositionId.id).ref.
                    GetRandomPosition(resource.GetRandomNumberGenerator());
            } else {
                assert(destPositionId.type == GIS_POI);

                nextPos = currentPos;
            }
        }

    } else {
        nextPos = theAgentGisPtr->GetVertex(resource.LastVertexId());
    }

    directionVector = (nextPos - currentPos).Normalized();
    remainingPathDistance = currentPos.DistanceTo(nextPos);

    resource.SetDirectionRadians(directionVector.DirectionRadians());
}



void FreeWalkBehavior::IncrementTimeStep(const TimeType& timeStep)
{
    const double walkSpeed = resource.WalkSpeedMetersPerSec();

    if (walkSpeed <= 0) {
        cerr << "Error: Walk speed is 0. Set WalkSpeed in AgentProfile." << endl;
        exit(1);
    }

    const double walkDistance = walkSpeed*(double(timeStep)/SECOND);

    remainingPathDistance = std::max<double>(0., remainingPathDistance - walkDistance);

    Vertex position = CalculateNextVertex(nextPos, directionVector, remainingPathDistance);
    position.z = theAgentGisPtr->GetSubsystem().GetGroundElevationMetersAt(position);

    resource.SetPosition(position);

    if ((*this).HasFinished()) {
        if (enterToPosition) {
            resource.ArrivedAtDestinationNotification();
        } else {
            resource.SetDesiredNextPositionId(endPositionId);
        }
        resource.SetExtraPoiId(extraDestPoinId);
    }
}



void FreeWalkBehavior::EndBehaviorAtViaPoint(const shared_ptr<AgentRoute>& nextRoutePtr)
{
    if (nextRoutePtr == nullptr) {
        return;
    }

    const Vertex& currentPos = resource.Position();

    enterToPosition = false;

    endPositionId = GisPositionIdType(GIS_ROAD, nextRoutePtr->GetFrontRoadId());
    extraDestPoinId = UNREACHABLE_POSITION_ID;

    nextPos = theAgentGisPtr->GetVertex(resource.LastVertexId());

    directionVector = (nextPos - currentPos).Normalized();
    remainingPathDistance = currentPos.DistanceTo(nextPos);

    resource.SetDirectionRadians(directionVector.DirectionRadians());
}



//-------------------------------------------------------------------
// Pedestrian
//-------------------------------------------------------------------

PedestrianBehavior::PedestrianBehavior(
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<AgentRoute>& initRoutePtr,
    const AgentResource& initResource)
    :
    AgentBehavior(initAgentGisPtr, initPublicVehicleTablePtr, initRoutePtr, initResource),
    state(STATE_WALK_ROADSIDE),
    nextPos(resource.Position()),
    roadId(INVALID_VARIANT_ID),
    nextVertexId(resource.LastVertexId()),
    remainingPathDistance(0),
    crossingDirectionIsClockwise(true),
    isCurrentLeftSide(true),
    isCrossingRoadAVehicleRoad(false),
    currentVertexNumber(0),
    currentRouteNumber(0),
    currentWalkStartTime(ZERO_TIME),
    pedestrianRoadWalkOffsetRatio(resource.GetRandomNumberGenerator().GenerateRandomDouble() - 0.5),
    endBehaviorAtViaPoint(false)
{
    (*this).ResetDestinationPosition();

    (*this).UpdateNextWalkPath();
}

PedestrianBehavior::PedestrianBehavior(
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<AgentRoute>& initRoutePtr,
    const Vertex& initDestPos,
    const AgentResource& initResource)
    :
    AgentBehavior(initAgentGisPtr, initPublicVehicleTablePtr, initRoutePtr, initResource),
    state(STATE_WALK_ROADSIDE),
    nextPos(resource.Position()),
    destPos(initDestPos),
    roadId(INVALID_VARIANT_ID),
    nextVertexId(resource.LastVertexId()),
    remainingPathDistance(0),
    crossingDirectionIsClockwise(true),
    isCurrentLeftSide(true),
    isCrossingRoadAVehicleRoad(false),
    currentVertexNumber(0),
    currentRouteNumber(0),
    currentWalkStartTime(ZERO_TIME),
    pedestrianRoadWalkOffsetRatio(resource.GetRandomNumberGenerator().GenerateRandomDouble() - 0.5),
    endBehaviorAtViaPoint(false)
{
    (*this).UpdateNextWalkPath();
}



void PedestrianBehavior::UpdateNextWalkPath()
{
    Vertex currentPos = resource.Position();
    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();

    if (crossingOrRoadsideWalks.empty()) {

        if (state == STATE_CROSSING_ROAD) {

            isCurrentLeftSide = crossingDirectionIsClockwise;
            theAgentGisPtr->SetLeavePeople(roadId, resource);

        } else if (currentVertexNumber == currentWaypoints.size() &&
                   currentRouteNumber < routePtr->roadRoutes.size()) {

            const RoadRoute& roadRoute = routePtr->roadRoutes[currentRouteNumber];
            const AgentRoad& agentRoad = theAgentGisPtr->GetAgentRoad(roadRoute.roadId);
            const double congestion = agentRoad.CalculatePeopleCongestion();
            const AgentIntersection& intersection =
                theAgentGisPtr->GetAgentIntersection(subsystem.GetIntersectionId(nextVertexId));

            resource.SetCongestion(congestion);
            resource.SetVertexId(nextVertexId);

            if (endBehaviorAtViaPoint) {
                (*this).ForceStopWalk();
                return;
            }

            if (!agentRoad.GetGisRoad().IsEnabled()) {
                resource.ArrivedAtDeadEndNotification();
                (*this).ForceStopWalk();
                resource.RecalculateRoute();
                return;
            }

            if (resource.IsPathQueryTriggerAvailable(congestion)) {
                (*this).ForceStopWalk();
                resource.RecalculateRoute();
                resource.SetLastPathQueryTriggerTime();
                return;
            }

            if (!resource.IsDisasterMode() &&
                !intersection.ref.PedestrianCanPass()) {
                intersection.GetCrossingOrSideWalkRoadIds(
                    *theAgentGisPtr,
                    roadId,
                    roadRoute.roadId,
                    isCurrentLeftSide,
                    crossingDirectionIsClockwise,
                    crossingOrRoadsideWalks,
                    currentWaypoints,
                    resource.GetRandomNumberGenerator());

                currentVertexNumber = 0;
            }
        }
    }

    if (!crossingOrRoadsideWalks.empty()) {
        state = STATE_CROSSING_ROAD;

        const pair<RoadIdType, bool>& crossingOrRoadsideWalk = crossingOrRoadsideWalks.front();

        roadId = crossingOrRoadsideWalk.first;
        resource.SetDesiredNextPositionId(GisPositionIdType(GIS_ROAD, roadId));

        assert(roadId != INVALID_VARIANT_ID);

        isCrossingRoadAVehicleRoad = false;

        // is crossing
        if (crossingOrRoadsideWalk.second) {
            const AgentRoad& agentRoad = theAgentGisPtr->GetAgentRoad(roadId);

            isCrossingRoadAVehicleRoad = agentRoad.GetGisRoad().VehicleCanPass();

            if (isCrossingRoadAVehicleRoad) {
                const AgentIntersection& intersection =
                    theAgentGisPtr->GetAgentIntersection(
                        subsystem.GetIntersectionId(resource.LastVertexId()));

                currentWalkStartTime =
                    intersection.ref.CalculateCrossingStartTime(
                        resource.CurrentTime(),
                        roadId,
                        crossingDirectionIsClockwise,
                        static_cast<TimeType>(agentRoad.GetGisRoad().GetRoadWidthMeters() / (*this).GetAgentSpeed()) * SECOND);
            }
        }

        crossingOrRoadsideWalks.pop_front();

    } else {

        const bool crossedRoad = (state == STATE_CROSSING_ROAD);

        state = STATE_WALK_ROADSIDE;

        if (currentVertexNumber == currentWaypoints.size() &&
            currentRouteNumber < routePtr->roadRoutes.size()) {

            const RoadRoute& roadRoute = routePtr->roadRoutes[currentRouteNumber];
            nextVertexId = roadRoute.nextVertexId;

            const AgentRoad& agentRoad = theAgentGisPtr->GetAgentRoad(roadRoute.roadId);

            if (resource.IsDisasterMode() || agentRoad.GetGisRoad().IsPedestrianRoad()) {
                const double roadWidth = agentRoad.GetGisRoad().GetRoadWidthMeters();
                agentRoad.GetGisRoad().GetPedestrianVertices(nextVertexId, resource.LastVertexId(), isCurrentLeftSide, roadWidth*pedestrianRoadWalkOffsetRatio, currentPos, currentWaypoints);

            } else {
                agentRoad.GetGisRoad().GetPedestrianVertices(nextVertexId, resource.LastVertexId(), isCurrentLeftSide, currentPos, currentWaypoints);
            }

            // adjust z to first position
            double adjustZ = currentPos.z;

            if (!currentWaypoints.empty()) {
                adjustZ = currentWaypoints.front().z;
            }
            // smooth correction to next point
            if (!currentWaypoints.empty()) {
                if (crossedRoad || agentRoad.GetGisRoad().IsExtraPath()) {
                    currentWaypoints.pop_front();
                }
            }

            // put adjusted position
            if (currentPos.z != adjustZ) {
                currentWaypoints.push_front(Vertex(currentPos.x, currentPos.y, adjustZ));
            }

            // smooth correction to next-next point
            if (!currentWaypoints.empty() &&
                currentRouteNumber + 1 < routePtr->roadRoutes.size()) {

                const RoadRoute& nextRoute = routePtr->roadRoutes[currentRouteNumber + 1];
                const AgentIntersection& intersection =
                    theAgentGisPtr->GetAgentIntersection(subsystem.GetIntersectionId(nextVertexId));

                if (!resource.IsDisasterMode() &&
                    !intersection.ref.PedestrianCanPass()) {
                    deque<Vertex> nextWaypoints;
                    bool isNextCrossingDirectionClockwise;
                    list<pair<RoadIdType, bool> > nextCrossingOrRoadsideWalks;

                    intersection.GetCrossingOrSideWalkRoadIds(
                        *theAgentGisPtr,
                        roadRoute.roadId,
                        nextRoute.roadId,
                        isCurrentLeftSide,
                        isNextCrossingDirectionClockwise,
                        nextCrossingOrRoadsideWalks,
                        nextWaypoints,
                        resource.GetRandomNumberGenerator());

                    if (!nextWaypoints.empty()) {
                        currentWaypoints.back() = nextWaypoints.front();
                    }
                }
            }

            currentRouteNumber++;
            currentVertexNumber = 0;

            roadId = roadRoute.roadId;

            resource.SetDesiredNextPositionId(GisPositionIdType(GIS_ROAD, roadId));
        }
    }

    if (currentVertexNumber < currentWaypoints.size()) {

        nextPos = currentWaypoints[currentVertexNumber];
        currentVertexNumber++;

    } else if (currentRouteNumber >= routePtr->roadRoutes.size()) {
        if (!routePtr->IsEmpty()) {
            resource.SetVertexId(routePtr->roadRoutes.back().nextVertexId);
        }
        nextPos = destPos;
    }

    directionVector = (nextPos - currentPos).Normalized();
    remainingPathDistance = currentPos.DistanceTo(nextPos);
    resource.SetDirectionRadians(directionVector.DirectionRadians());
}




void PedestrianBehavior::IncrementTimeStep(const TimeType& timeStep)
{
    TimeType remainingTime = timeStep;

    while (!(*this).HasFinished() && remainingTime > ZERO_TIME) {
        (*this).IncrementTimeStep(timeStep, remainingTime);
    }

    const Vertex position = CalculateNextVertex(nextPos, directionVector, remainingPathDistance);

    resource.SetPosition(position);
}




void PedestrianBehavior::IncrementTimeStep(const TimeType& timeStep, TimeType& remainingTime)
{
    const TimeType currentTime = resource.CurrentTime();

    remainingTime = std::max(ZERO_TIME, std::min(currentTime - currentWalkStartTime, timeStep));

    if (remainingTime == ZERO_TIME) {
        if (state == STATE_CROSSING_ROAD) {
            theAgentGisPtr->SetWaitingPeople(roadId, resource);
        }
        return;
    }

    double walkSpeedMetersPerSec = GetAgentSpeed();

    if (walkSpeedMetersPerSec <= 0) {
        cerr << "Error: Walk speed is 0. Set WalkSpeed in AgentProfile." << endl;
        exit(1);
    }

    if ((state == STATE_WALK_ROADSIDE) && (roadId != INVALID_VARIANT_ID)) {

        const AgentRoad& agentRoad = theAgentGisPtr->GetAgentRoad(roadId);
        const double density = agentRoad.CalculatePeopleCongestion();
        //const double density = agentRoad.CalculatePeopleCongestionWithout(resource);
        const double minPedestrianSpeedMetersPerSec = 1;
        walkSpeedMetersPerSec =
            std::max(walkSpeedMetersPerSec * (1 - density), minPedestrianSpeedMetersPerSec);
    }

    const double walkDistance = walkSpeedMetersPerSec*(double(remainingTime)/SECOND);

    if (remainingPathDistance <= walkDistance) {
        if (nextPos != destPos) {

            remainingTime += static_cast<TimeType>(
                ((walkDistance - remainingPathDistance) / walkSpeedMetersPerSec) * SECOND);

            (*this).UpdateNextWalkPath();
        } else {
            if (!routePtr->IsEmpty()) {
                resource.SetVertexId(routePtr->roadRoutes.back().nextVertexId);
            }

            currentRouteNumber = routePtr->roadRoutes.size();
            remainingPathDistance = 0;
            remainingTime = ZERO_TIME;
        }

    } else {
        remainingPathDistance -= walkDistance;
        remainingTime = ZERO_TIME;

        if ((state == STATE_CROSSING_ROAD) && (isCrossingRoadAVehicleRoad)) {
            theAgentGisPtr->SetCrossingPeople(roadId, resource);
        }
    }

    if (remainingPathDistance <= 0) {
        remainingTime = ZERO_TIME;
    }
}



void PedestrianBehavior::StartMovingToNewQueuePosition(const Vertex& newQueuePosition)
{
    (*this).state = STATE_QUEUED;
    (*this).queueingSavedCurrentPosition = resource.Position();
    (*this).queueingSavedNextPos = nextPos;
    (*this).nextPos = newQueuePosition;
    assert(nextPos != destPos);

}//StartMovingToNewQueuePosition//


void PedestrianBehavior::Dequeue()
{
    (*this).state = STATE_WALK_ROADSIDE;

    // "Teleport" to previously saved (entrance) position.

    (*this).resource.SetPosition(queueingSavedCurrentPosition);
    (*this).nextPos = queueingSavedNextPos;
}



void PedestrianBehavior::EndBehaviorAtViaPoint(const shared_ptr<AgentRoute>& nextRoutePtr)
{
    endBehaviorAtViaPoint = true;
}



VertexIdType PedestrianBehavior::GetViaPointVertexId() const
{
    if (state == STATE_WALK_ROADSIDE &&
        0 < currentRouteNumber && currentRouteNumber <= routePtr->roadRoutes.size()) {

        const RoadRoute& roadRoute = routePtr->roadRoutes[currentRouteNumber - 1];

        return roadRoute.nextVertexId;
    }

    return resource.LastVertexId();
}



bool PedestrianBehavior::IsAcceptableRouteChange(const AgentRoute& route) const
{
    return (route.behavior == (*this).GetBehaviorType());
}



void PedestrianBehavior::ChangeRoute(const shared_ptr<AgentRoute>& newRoutePtr)
{
    if ((state == STATE_WALK_ROADSIDE) &&
        (0 < currentRouteNumber && currentRouteNumber <= routePtr->roadRoutes.size())) {

        const RoadRoute lastRoadRoute = routePtr->roadRoutes[currentRouteNumber - 1];

        routePtr = newRoutePtr;
        routePtr->roadRoutes.push_front(lastRoadRoute);
        currentRouteNumber = 1;
    } else {
        routePtr = newRoutePtr;
        currentRouteNumber = 0;
    }

    endBehaviorAtViaPoint = false;

    (*this).ResetDestinationPosition();
}



void PedestrianBehavior::ResetDestinationPosition()
{
    assert(!routePtr->IsEmpty());

    destPos = theAgentGisPtr->GetVertex(routePtr->roadRoutes.back().nextVertexId);
}



void PedestrianBehavior::ForceStopWalk()
{
    remainingPathDistance = 0;

    if (endBehaviorAtViaPoint) {
        currentRouteNumber = routePtr->roadRoutes.size();
    }

    crossingOrRoadsideWalks.clear();
}




//-------------------------------------------------------------------
// Bicycle
//-------------------------------------------------------------------

BicycleBehavior::BicycleBehavior(
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<AgentRoute>& initRoutePtr,
    const AgentResource& initResource)
    :
    PedestrianBehavior(initAgentGisPtr, initPublicVehicleTablePtr, initRoutePtr, initResource)
{}

BicycleBehavior::BicycleBehavior(
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<AgentRoute>& initRoutePtr,
    const Vertex& initDestPos,
    const AgentResource& initResource)
    :
    PedestrianBehavior(initAgentGisPtr, initPublicVehicleTablePtr, initRoutePtr, initDestPos, initResource)
{}

//-----------------------------------------------------------------

const double VehicleDriverBehavior::enoughMinDistance = 0.01; // 1[cm]
const double VehicleDriverBehavior::laneChangeTimeSec = 5;
const double VehicleDriverBehavior::velocityMargin = 0;
const double VehicleDriverBehavior::laneChangeProhibitedZoneLength = 0;
const double VehicleDriverBehavior::forceLaneChangeDistance = 50;
const double VehicleDriverBehavior::slowDownDistanceBeforeCurve = 30;

VehicleDriverBehavior::VehicleDriverBehavior(
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<AgentRoute>& initRoutePtr,
    const AgentResource& initResource,
    const shared_ptr<Vehicle>& initVehiclePtr,
    const TimeType& initMinShortStopTime)
    :
    AgentBehavior(initAgentGisPtr, initPublicVehicleTablePtr, initRoutePtr, initResource),
    isOnVehicle(false),
    passingIntersection(false),
    vehiclePtr(initVehiclePtr),
    remainingDistanceToIntersection(0),
    remainingShortStopTime(ZERO_TIME),
    minShortStopTime(initMinShortStopTime),
    laneChangeEnableTime(ZERO_TIME),
    laneChangeDirection(0),
    velocity(0.0),
    currentLaneNumber(0),
    remainingPathDistance(DBL_MAX),
    currentWaypointRadians(0),
    viewdRouteNumber(0)
{
    vehiclePtr->SetVehicleConstant(resource.MakeVehicleConstant());

    (*this).GoToVehicle();
}

void VehicleDriverBehavior::GoToVehicle()
{
    pedestrianPtr.reset(
        new PedestrianBehavior(
            theAgentGisPtr,
            thePublicVehicleTablePtr,
            shared_ptr<AgentRoute>(new AgentRoute()),
            vehiclePtr->GetPosition(),
            resource));
}

void VehicleDriverBehavior::UpdateVisibleRouteIfNecessary()
{
    double visibleRouteDistance = 0;

    if (routes.empty()) {
        visibleRouteDistance += remainingDistanceToIntersection;
    }

    for(size_t i = 1; i < routes.size(); i++) {
        visibleRouteDistance +=
            theAgentGisPtr->GetAgentRoad(routes[i].roadId).GetRoadLength();
    }

    const double maxVisibleRouteDistance = 100;

    while ((visibleRouteDistance < maxVisibleRouteDistance) &&
           viewdRouteNumber < routePtr->roadRoutes.size()) {

        const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();
        VertexIdType lastVertexId;

        if (routes.empty()) {
            lastVertexId = resource.LastVertexId();
        } else {
            lastVertexId = routes.back().outgoingVertexId;
        }

        const RoadRoute& roadRoute = routePtr->GetRoadRoute(viewdRouteNumber);
        const AgentRoad& nextRoad = theAgentGisPtr->GetAgentRoad(roadRoute.roadId);

        routes.push_back(VehicleRoute());
        VehicleRoute& nextRoute = routes[routes.size() - 1];

        nextRoute.roadId = roadRoute.roadId;

        nextRoute.outgoingVertexId = roadRoute.nextVertexId;

        const bool addedFollowingRoute = (routes.size() == 2);

        if (addedFollowingRoute) {
            (*this).UpdateNextRoadWaypointIfNecessary();
        }

        if (routes.size() >= 2) {
            VehicleRoute& prevRoute = routes[routes.size() - 2];

            const Intersection& intersection =
                theAgentGisPtr->GetAgentIntersection(
                    subsystem.GetIntersectionId(lastVertexId)).ref;

            prevRoute.outgoingTurnType =
                intersection.GetRoadTurnType(prevRoute.roadId, nextRoute.roadId);
        }

        visibleRouteDistance += nextRoad.GetRoadLength();

        viewdRouteNumber++;
    }

    if (!routes.empty()) {
        vehiclePtr->viapointVertexId = routes.back().outgoingVertexId;
    }
}

void VehicleDriverBehavior::SetNextWaypoint()
{
    if (routes.empty()) {
        return;
    }

    assert(!routes.front().remainingWaypoints.empty());
    const Vertex lastPoint = routes.front().remainingWaypoints.front();

    VehicleRoute& currentRoute = routes.front();

    double extraPathOffset = 0.;

    const Road& currentRoad = theAgentGisPtr->GetAgentRoad(currentRoute.roadId).GetGisRoad();

    if (currentRoad.IsExtraPath()) {
        // force transition from extra(simple line) path
        if (routes.size() > 1) {
            extraPathOffset =
                theAgentGisPtr->GetAgentRoad(routes[1].roadId).GetGisRoad().GetRoadWidthMeters()*0.5;
        }
    }

    if (remainingDistanceToIntersection <= enoughMinDistance + extraPathOffset) {

        Vehicle& vehicle = *vehiclePtr;

        const VertexIdType arrivedVertexId = routes.front().outgoingVertexId;
        const RoadIdType prevRoadId = currentRoute.roadId;

        routes.pop_front();

        resource.SetVertexId(arrivedVertexId);
        vehiclePtr->SetNextVertexId(arrivedVertexId);

        if (!routes.empty()) {
            VehicleRoute& nextRoute = routes.front();
            const RoadIdType nextRoadId = nextRoute.roadId;
            const Road& prevRoad = theAgentGisPtr->GetAgentRoad(prevRoadId).GetGisRoad();

            if (!prevRoad.CanApproach(currentLaneNumber, routes.front().roadId)) {
                // force change lane fomr parking
                currentLaneNumber = prevRoad.GetApproachLaneNumber(nextRoadId);
            }

            const AgentRoad& nextAgentRoad = theAgentGisPtr->GetAgentRoad(nextRoadId);
            const TimeType noVehicleDelay = ZERO_TIME;

            currentLaneNumber = prevRoad.GetNextLaneNumber(currentLaneNumber, nextRoadId);

            const double congestion =
                nextAgentRoad.CalculateVehicleCongestion(currentLaneNumber);

            assert(!nextAgentRoad.GetGisRoad().IsBuildingParkingRoad());


            if ((GetBehaviorType() == AGENT_BEHAVIOR_VEHICLE) &&
                (prevRoad.IsBuildingParkingRoad())) {
                const BuildingIdType buildingId = prevRoad.GetBuildingId();

                theAgentGisPtr->LeaveFromParking(resource, buildingId);
            }//if//

            vehicle.SetNextLaneNumber(currentLaneNumber);
            vehicle.SetNextRoadId(nextRoute.roadId);
            vehicle.SetNextTurnType(nextRoute.outgoingTurnType);

            passingIntersection = true;

            remainingDistanceToIntersection =
                lastPoint.DistanceTo(nextRoute.remainingWaypoints.front()) +
                CalculateArcDistance(nextRoute.remainingWaypoints);

            resource.SetCongestion(congestion);
            resource.SetDesiredNextPositionId(GisPositionIdType(GIS_ROAD, nextRoute.roadId));

            (*this).ArrivedAtVertex(arrivedVertexId);

            if (!nextAgentRoad.GetGisRoad().IsEnabled()) {
                resource.ArrivedAtDeadEndNotification();
                (*this).ForceStopVehicle();
                resource.RecalculateRoute();
                return;
            }

            if (resource.IsPathQueryTriggerAvailable(congestion, noVehicleDelay)) {
                resource.RecalculateRoute();
                resource.SetLastPathQueryTriggerTime();
            }

            (*this).UpdateNextRoadWaypointIfNecessary();

        } else {
            (*this).ArrivedAtVertex(arrivedVertexId);
        }

    } else {
        currentRoute.remainingWaypoints.pop_front();
        passingIntersection = false;
    }

    if (routes.empty()) {
        remainingPathDistance = 0;
        return;
    }

    assert(!routes.front().remainingWaypoints.empty());
    const deque<Vertex>& currentWaypoints = routes.front().remainingWaypoints;

    if (currentWaypoints.empty()) {

        remainingPathDistance = enoughMinDistance;

    } else {
        const Vertex& nextPoint = currentWaypoints.front();

        if (lastPoint.DistanceTo(nextPoint) <= enoughMinDistance) {

            remainingPathDistance = enoughMinDistance;

        } else {
            const Vertex pathVector = nextPoint - lastPoint;

            directionVector = pathVector.Normalized();
            remainingPathDistance = pathVector.Distance();

            vehiclePtr->SetNextDirectionRadians(directionVector.DirectionRadians());
            resource.SetDirectionRadians(directionVector.DirectionRadians());

            if (currentWaypoints.size() > 1) {

                currentWaypointRadians = CalculateRadiansBetweenVector(
                    lastPoint, nextPoint, currentWaypoints[1]);

            } else if (routes.size() > 1) {
                assert(!routes[1].remainingWaypoints.empty());

                if (routes[1].remainingWaypoints.size() > 1) {
                    currentWaypointRadians = CalculateRadiansBetweenVector(
                        lastPoint,
                        routes[1].remainingWaypoints[0],
                        routes[1].remainingWaypoints[1]);
                } else {
                    const VertexIdType outgoingVertexId = routes.front().outgoingVertexId;

                    currentWaypointRadians = CalculateRadiansBetweenVector(
                        lastPoint,
                        theAgentGisPtr->GetVertex(outgoingVertexId),
                        routes[1].remainingWaypoints.front());
                }
            }

            assert(!routes.empty());
            remainingDistanceToIntersection =
                std::max(remainingDistanceToIntersection, remainingPathDistance);
        }
    }
}

void VehicleDriverBehavior::ArrivedAtVertex(const VertexIdType& vertexId)
{
    if (routes.empty()) {
        // arrived at the destination.

        (*this).ParkingVehicleIfPossible(vertexId, minShortStopTime);
    }
}

void VehicleDriverBehavior::ParkingVehicleIfPossible(
    const VertexIdType& vertexId,
    const TimeType& stopTime)
{
    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();

    if (subsystem.IsParkingVertex(vertexId)) {
        const RoadIdType roadId = subsystem.GetParkingRoadId(vertexId);
        const Road& aRoad = theAgentGisPtr->GetAgentRoad(roadId).GetGisRoad();

        vehiclePtr->SetNextRoadId(roadId);

        if ((GetBehaviorType() == AGENT_BEHAVIOR_VEHICLE) &&
            (aRoad.IsBuildingParkingRoad())) {

            const BuildingIdType buildingId = aRoad.GetBuildingId();

           theAgentGisPtr->StartVehicleEntranceWait(resource, buildingId);
        }//if//

        currentLaneNumber = 0;

        vehiclePtr->SetNextLaneNumber(currentLaneNumber);
        resource.SetDesiredNextPositionId(GIS_ROAD, roadId);
        remainingShortStopTime += stopTime;
    }
}

void VehicleDriverBehavior::IncrementTimeStep(const TimeType& timeStep)
{
    Vehicle& vehicle = *vehiclePtr;

    if (!isOnVehicle) {
        if (pedestrianPtr != nullptr) {
            pedestrianPtr->IncrementTimeStep(timeStep);

            if (pedestrianPtr->HasFinished()) {
                (*this).RideOnVehicle();
            }
        }

        return;
    }

    TimeType runTime = timeStep;

    if (remainingShortStopTime > ZERO_TIME) {

        if (remainingShortStopTime <= runTime) {
            runTime -= remainingShortStopTime;
            remainingShortStopTime = ZERO_TIME;

            (*this).RestartRun();

        } else {
            remainingShortStopTime -= runTime;
            runTime = ZERO_TIME;
        }
    }

    if (runTime > ZERO_TIME) {
        if ((*this).HasRemainingPath()) {
            (*this).RunVehicle(runTime);
        } else {
            isOnVehicle = false;
        }
    }

    resource.SetPosition(vehicle.GetNextPosition());
}



VehicleDriverBehavior::~VehicleDriverBehavior()
{
    Vehicle& vehicle = *vehiclePtr;
    theAgentGisPtr->TakeOwnershiopOfVehicleLastPositionUpdate(resource, vehiclePtr);
}


void VehicleDriverBehavior::UpdateVelocity(const TimeType& timeStep)
{
    double distanceToStop;
    double frontVehicleVelocity;
    double minDistanceToFrontVehicle;

    (*this).ViewFrontStatus(
        distanceToStop,
        frontVehicleVelocity,
        minDistanceToFrontVehicle);

    const double timeStepSec = double(timeStep)/SECOND;

    const double accel =
        (*this).CalculateIdmBasedAcceleration(
            timeStepSec,
            distanceToStop,
            frontVehicleVelocity);

    velocity = std::max<double>(0, velocity + accel*timeStepSec);

    vehiclePtr->SetNextVelocity(velocity);

    assert(velocity >= 0);
}

void VehicleDriverBehavior::ChangeLaneIfNecessary()
{
    if (routes.empty() || passingIntersection) {
        return;
    }

    const Vehicle& vehicle = *vehiclePtr;
    const VehicleRoute& currentRoute = routes.front();
    const Road& currentRoad = theAgentGisPtr->GetAgentRoad(currentRoute.roadId).GetGisRoad();

    const double possibleLaneChangeAcceleration =
        CalculateIntelligentDriversModelAcceleration(
            vehicle,
            remainingDistanceToIntersection - vehicle.GetHalfBodyLength(),
            DBL_MAX,
            vehicle.GetVehicleConstant().maxVehicleSpeed);

    if (possibleLaneChangeAcceleration < resource.MaxDeceleration()) {
        return;
    }

    if (routes.size() > 1) {
        const VehicleRoute& nextRoute = routes[1];
        const RoadIdType nextRoadId = nextRoute.roadId;

        if (!currentRoad.CanApproach(currentLaneNumber, nextRoadId) &&
            remainingDistanceToIntersection < forceLaneChangeDistance) {

            const size_t approachLaneNumber =
                currentRoad.GetNeighborLaneNumberToApproach(currentLaneNumber, nextRoadId);

            (*this).ChangeLane(approachLaneNumber);
            return;
        }
    }

    const double laneChangeRadians = PI / 10.;
    if (currentWaypointRadians > laneChangeRadians &&
        remainingPathDistance < slowDownDistanceBeforeCurve) {
        return;
    }

    const RoadDirectionType directionType = currentRoad.GetRoadDirection(currentLaneNumber);

    if (currentRoad.HasPassingLane(directionType, currentLaneNumber)) {
        const size_t rightLaneNumber =
            currentRoad.GetPassingLaneNumber(directionType, currentLaneNumber);

        if ((*this).CanChangeLane(rightLaneNumber)) {
            (*this).ChangeLane(rightLaneNumber);
        }
    }

    if (currentRoad.HasNonPassingLane(directionType, currentLaneNumber)) {
        const size_t leftLaneNumber =
            currentRoad.GetNonPassingLaneNumber(directionType, currentLaneNumber);

        if ((*this).CanChangeLane(leftLaneNumber)) {
            (*this).ChangeLane(leftLaneNumber);
        }
    }
}

bool VehicleDriverBehavior::CanChangeLane(const size_t nextLaneNumber) const
{
    if (laneChangeEnableTime > resource.CurrentTime() || passingIntersection) {
        return false;
    }

    assert(!routes.empty());

    const Vehicle& vehicle = *vehiclePtr;
    const VehicleRoute& currentRoute = routes.front();
    const AgentRoad& currentRoad = theAgentGisPtr->GetAgentRoad(currentRoute.roadId);

    if (routes.size() > 1) {
        const VehicleRoute& nextRoute = routes[1];

        if (!currentRoad.GetGisRoad().CanApproach(nextLaneNumber, nextRoute.roadId) &&
            remainingDistanceToIntersection < forceLaneChangeDistance) {
            return false;
        }
    }

    if (!(*this).HasEnoughGapAndSafetyCriterionToChangeLane(nextLaneNumber)) {
        return false;
    }

    double backVehicleDisadvantage = 0;
    double prevAccel;
    double nextAccel;

    if (currentRoad.HasFrontVehicle(vehicle, currentLaneNumber)) {
        const Vehicle& frontVehicle =
            currentRoad.GetFrontVehicle(vehicle, currentLaneNumber);

        prevAccel = CalculateIntelligentDriversModelAcceleration(vehicle, frontVehicle);
    } else {
        prevAccel = CalculateIntelligentDriversModelAcceleration(vehicle);
    }

    if (currentRoad.HasFrontVehicle(vehicle, nextLaneNumber)) {
        const Vehicle& frontVehicle =
            currentRoad.GetFrontVehicle(vehicle, nextLaneNumber);

        if (currentRoad.HasBackVehicle(vehicle, nextLaneNumber)) {
            const Vehicle& backVehicle =
                currentRoad.GetBackVehicle(vehicle, nextLaneNumber);

            const double nextBackAccel =
                CalculateIntelligentDriversModelAcceleration(backVehicle, vehicle);

            const double prevBackAccel =
                CalculateIntelligentDriversModelAcceleration(backVehicle, frontVehicle);

            backVehicleDisadvantage = std::max(0., nextBackAccel - prevBackAccel);
        }

        nextAccel = CalculateIntelligentDriversModelAcceleration(vehicle, frontVehicle);
    } else {
        nextAccel = CalculateIntelligentDriversModelAcceleration(vehicle);
    }

    const double ownAdvantage = nextAccel - prevAccel;

    const double accelerationThreshold =
        vehicle.GetVehicleConstant().laneChangeAccelerationThreshold;

    return ((ownAdvantage - backVehicleDisadvantage) > accelerationThreshold);
}

void VehicleDriverBehavior::ChangeLane(const size_t nextLaneNumber)
{
    assert(!routes.empty());

    VehicleRoute& currentRoute = routes.front();
    const AgentRoad& currentRoad = theAgentGisPtr->GetAgentRoad(currentRoute.roadId);
    const size_t lastLaneNumber = currentLaneNumber;

    if (lastLaneNumber == nextLaneNumber) {
        return;
    }

    currentLaneNumber = nextLaneNumber;

    vehiclePtr->SetNextLaneNumber(currentLaneNumber);
    vehiclePtr->SetNextChangedLaneNumber(lastLaneNumber);

    Vertex normalVector;

    if (currentRoad.GetGisRoad().GetRoadDirection(nextLaneNumber) == ROAD_DIRECTION_UP) {
        if (nextLaneNumber < lastLaneNumber) {
            laneChangeDirection = -1;
        } else {
            laneChangeDirection = 1;
        }
    } else {
        if (nextLaneNumber > lastLaneNumber) {
            laneChangeDirection = -1;
        } else {
            laneChangeDirection = 1;
        }
    }

    const Vertex& currentPos = resource.Position();

    deque<Vertex>& currentWaypoints = routes.front().remainingWaypoints;

    currentRoad.GetGisRoad().GetLaneVertices(
        nextLaneNumber, true/*waypointFromAdditionalStartPosition*/, currentPos, currentWaypoints);

    assert(!currentWaypoints.empty());
    Vertex nearestPosition;
    size_t vertexNumber;

    CalculatePointToArcNearestPosition(
        currentWaypoints,
        currentPos,
        nearestPosition,
        vertexNumber);

    assert(vertexNumber < currentWaypoints.size());
    for(size_t i = 0; i <= vertexNumber; i++) {
        currentWaypoints.pop_front();
    }
    currentWaypoints.push_front(nearestPosition);

    remainingDistanceToIntersection =
        CalculateArcDistance(currentWaypoints);

    (*this).UpdateNextRoadWaypointIfNecessary();

    (*this).SetNextWaypoint();

    laneChangeEnableTime =
        resource.CurrentTime() + static_cast<TimeType>(laneChangeTimeSec)*SECOND;
}

void VehicleDriverBehavior::UpdateNextRoadWaypointIfNecessary()
{
    assert(!routes.empty());

    VehicleRoute& currentRoute = routes.front();
    const Road& currentRoad = theAgentGisPtr->GetAgentRoad(currentRoute.roadId).GetGisRoad();

    if (routes.size() > 1) {
        VehicleRoute& nextRoute = routes[1];
        const RoadIdType nextRoadId = nextRoute.roadId;

        size_t approachLaneNumber = currentLaneNumber;

        if (!currentRoad.CanApproach(currentLaneNumber, nextRoadId)) {
            approachLaneNumber = currentRoad.GetApproachLaneNumber(nextRoadId);
        }

        const Road& nextRoad = theAgentGisPtr->GetAgentRoad(nextRoadId).GetGisRoad();

        const size_t nextLaneNumber =
            currentRoad.GetNextLaneNumber(approachLaneNumber, nextRoadId);

        const Vertex& currentPos = resource.Position();

        Vertex startPosition;

        if (routes[0].remainingWaypoints.empty()) {
            startPosition = currentPos;
        }
        else {
            startPosition = routes[0].remainingWaypoints.back();
        }

        nextRoad.GetLaneVertices(
            nextLaneNumber,
            true/*waypointFromAdditionalStartPosition*/,
            startPosition,
            nextRoute.remainingWaypoints);


        if (nextRoad.IsExtraPath()) {
            // discard intersection vertex
            if (nextRoute.remainingWaypoints.size() > 1) {
                nextRoute.remainingWaypoints.pop_front();
            }
        }

        assert(!nextRoute.remainingWaypoints.empty());
    }
}

bool VehicleDriverBehavior::HasEnoughGapAndSafetyCriterionToChangeLane(
    const size_t laneNumber) const
{
    const VehicleRoute& currentRoute = routes.front();
    const AgentRoad& currentRoad = theAgentGisPtr->GetAgentRoad(currentRoute.roadId);

    const Vehicle& vehicle = *vehiclePtr;

    if (currentRoad.HasFrontVehicle(vehicle, laneNumber)) {
        const Vehicle& frontVehicle = currentRoad.GetFrontVehicle(vehicle, laneNumber);
        const VehicleConstant& vehicleConstant = vehicle.GetVehicleConstant();

        const double distanceToFront =
            remainingDistanceToIntersection - frontVehicle.GetDistanceToIntersection() -
            (frontVehicle.GetHalfBodyLength() + vehicle.GetHalfBodyLength());

        if (distanceToFront <= vehicleConstant.minVehicleGap) {
            return false;
        }

        const double biasSave = vehicleConstant.saveAcceleration;

        if (CalculateIntelligentDriversModelAcceleration(vehicle, frontVehicle) < biasSave) {
            return false;
        }
    }

    if (currentRoad.HasBackVehicle(vehicle, laneNumber)) {
        const Vehicle& backVehicle = currentRoad.GetBackVehicle(vehicle, laneNumber);
        const VehicleConstant& vehicleConstant = backVehicle.GetVehicleConstant();

        const double distanceToFront =
            backVehicle.GetDistanceToIntersection() - remainingDistanceToIntersection -
            (vehicle.GetHalfBodyLength() + backVehicle.GetHalfBodyLength());

        if (distanceToFront <= vehicleConstant.minVehicleGap) {
            return false;
        }

        const double biasSaveSelf = vehicleConstant.saveAcceleration;

        if (CalculateIntelligentDriversModelAcceleration(backVehicle, vehicle) < biasSaveSelf) {
            return false;
        }
    }

    return true;
}

double VehicleDriverBehavior::GetRemainingLanechangeRate() const
{
    const double remainingTime =
        static_cast<double>(laneChangeEnableTime - resource.CurrentTime()) / SECOND;

    assert(remainingTime >= 0);

    return remainingTime / laneChangeTimeSec;
}

double VehicleDriverBehavior::CalculateIntelligentDriversModelAcceleration(
    const Vehicle& vehicle,
    const double distanceToFrontPoint,
    const double frontObstructionVelocity,
    const double maxVelocity)
{
    const double originalVelocity = vehicle.GetVelocityMetersPerSec();
    const VehicleConstant& vehicleConstant = vehicle.GetVehicleConstant();

    const double maxAccel = vehicleConstant.maxAcceleration;
    const double maxDecel = fabs(vehicleConstant.maxDecelaration);

    const double dv = originalVelocity - frontObstructionVelocity;
    const double s = distanceToFrontPoint;//actual gap
    const double s0 = vehicleConstant.minVehicleGap;
    const double s1 = vehicleConstant.velocityRatioGapDistance; //0m <= s1 <= 15m
    const double velocityRatio = originalVelocity/maxVelocity;
    const double T = vehicleConstant.timeHeadway;

    const double sstar =
        s0 + std::max(0., (s1*sqrt(velocityRatio) + originalVelocity*T + (originalVelocity*dv)/(2*sqrt(fabs(maxAccel*maxDecel)))));//desired minimum gap

    const double accelerationExponent = vehicleConstant.accelerationExponent;
    const double accel = maxAccel * (1 - pow(velocityRatio, accelerationExponent) - (sstar*sstar)/(s*s));

    double maxBrakingDecel = vehicleConstant.maxBrakingDecceleration;

    if (originalVelocity <= 0) {
        maxBrakingDecel = 0.;
    }

    return std::max(accel, maxBrakingDecel);
}

double VehicleDriverBehavior::CalculateIntelligentDriversModelAcceleration(
    const Vehicle& vehicle,
    const Vehicle& frontVehicle)
{
    const double distanceToFront =
        vehicle.GetDistanceToIntersection() - frontVehicle.GetDistanceToIntersection() -
        (frontVehicle.GetHalfBodyLength() + vehicle.GetHalfBodyLength());

    return CalculateIntelligentDriversModelAcceleration(
        vehicle,
        distanceToFront,
        frontVehicle.GetVelocityMetersPerSec(),
        vehicle.GetVehicleConstant().maxVehicleSpeed);
}

double VehicleDriverBehavior::CalculateIntelligentDriversModelAcceleration(
    const Vehicle& vehicle)
{
    return CalculateIntelligentDriversModelAcceleration(
        vehicle,
        DBL_MAX,
        DBL_MAX,
        vehicle.GetVehicleConstant().maxVehicleSpeed);
}

double VehicleDriverBehavior::CalculateFreeRunAcceleration(
    const double originalVelocity,
    const double maxVelocity)
{
    const double ta0 = 2.45; // [s]

    return (maxVelocity - originalVelocity)/ta0;
}

double VehicleDriverBehavior::CalculateDesiredVelocity(
    const AgentRoad& passingRoad,
    const Vehicle& vehicle,
    const double distanceToVertex,
    const double radiansToNextPoint)
{
    const VehicleConstant& vehicleConstant = vehicle.GetVehicleConstant();

    const double maxVelocity =
        std::min(passingRoad.GetGisRoad().GetSpeedLimitMetersPerSec(),
                 vehicleConstant.maxVehicleSpeed);

    if (maxVelocity <= 0.) {
        cerr << "Error: Maximum velocity is " << maxVelocity << endl
             << "  Vehicle velocity must be greater than 0." << endl;
        exit(1);
    }

    if (distanceToVertex > slowDownDistanceBeforeCurve) {
        return maxVelocity;
    }

    const double minVelocity = vehicleConstant.maxTurnSpeed;

    if (minVelocity <= 0.) {
        cerr << "Error: Minimum velocity is " << minVelocity << endl
             << "  Vehicle velocity must be greater than 0." << endl;
        exit(1);
    }

    if (maxVelocity < minVelocity) {
        return maxVelocity;
    }

    assert(radiansToNextPoint >= 0);
    assert(radiansToNextPoint <= PI);

    // Note: linear velocity limitation calculation.
    //       maybe exponent calculation is better.

    const double requiredVelocity =
        (maxVelocity-minVelocity)*((radiansToNextPoint)/PI) + minVelocity;

    return requiredVelocity;
}

double VehicleDriverBehavior::CalculateIdmBasedAcceleration(
    const double timeStepSec,
    const double distanceToStop,
    const double frontVehicleVelocity) const
{
    assert(!routes.empty());
    const VehicleRoute& currentRoute = routes.front();

    const double maxVelocity =
        CalculateDesiredVelocity(
            theAgentGisPtr->GetAgentRoad(currentRoute.roadId),
            *vehiclePtr,
            remainingPathDistance,
            currentWaypointRadians);

    if (maxVelocity <= 0) {
        cerr << "Error: Max vehicle speed is 0. Set VehicleSpeed in AgentProfile." << endl;
        exit(1);
    }

    return CalculateIntelligentDriversModelAcceleration(
        *vehiclePtr,
        distanceToStop,
        frontVehicleVelocity,
        maxVelocity);
}

void VehicleDriverBehavior::RunVehicle(const TimeType& runDuration)
{
    (*this).UpdateVelocity(runDuration);

    (*this).ChangeLaneIfNecessary();

    if (velocity == 0) {
        return;
    }

    double remainingTimeSec = double(runDuration)/SECOND;

    while ((*this).HasRemainingPath() && remainingTimeSec > 0) {

        const double runDistance = velocity*remainingTimeSec;

        vehiclePtr->runningDistance += runDistance;

        if (remainingPathDistance <= runDistance + enoughMinDistance) {
            remainingTimeSec = std::max(0.,remainingTimeSec - fabs(remainingPathDistance / velocity));

            remainingDistanceToIntersection -= remainingPathDistance;

            (*this).UpdateVisibleRouteIfNecessary();
            (*this).SetNextWaypoint();

            if (remainingShortStopTime > ZERO_TIME) {
                remainingShortStopTime =
                    std::max(minShortStopTime,
                             remainingShortStopTime - static_cast<TimeType>(remainingTimeSec*SECOND));
                break;
            }

        } else {
            remainingTimeSec = 0;

            remainingDistanceToIntersection -= runDistance;

            remainingPathDistance -= runDistance;

            (*this).UpdateVisibleRouteIfNecessary();
        }
    }

    if (!routes.empty()) {
        const VehicleRoute& currentRoute = routes.front();
        const Vertex& nextPos = currentRoute.remainingWaypoints.front();

        Vehicle& vehicle = *vehiclePtr;

        Vertex position = CalculateNextVertex(nextPos, directionVector, remainingPathDistance);

        if ((*this).IsChangingLane()) {
            const Vertex normalVector =
                directionVector.XYPoint().NormalVector().Normalized()*laneChangeDirection;

            const AgentRoad& currentRoad = theAgentGisPtr->GetAgentRoad(currentRoute.roadId);

            const double laneWidth = currentRoad.GetGisRoad().GetLaneWidthMeters();

            const double remainingLaneChangeRate = (*this).GetRemainingLanechangeRate();

            position += normalVector*laneWidth*remainingLaneChangeRate;
        } else {
            vehicle.SetNextChangedLaneNumber(Vehicle::NO_CHANGED_LANE_NUMBER);
        }

        vehicle.SetNextPosition(position);
        vehicle.SetNextDistanceToIntersection(remainingDistanceToIntersection);
    }
}



void VehicleDriverBehavior::AssignFirstRoute()
{
    Vehicle& vehicle = *vehiclePtr;

    viewdRouteNumber = 0;
    vehicle.viapointVertexId = resource.LastVertexId();

    if (routePtr->roadRoutes.empty()) {
        return;
    }

    const RoadRoute& roadRoute = routePtr->GetRoadRoute(viewdRouteNumber);
    const Road& road = theAgentGisPtr->GetAgentRoad(roadRoute.roadId).GetGisRoad();
    const Vertex& currentPos = resource.Position();

    currentLaneNumber =
        road.GetNearestOutgoingLaneNumber(resource.LastVertexId(), resource.Position());

    routes.clear();
    routes.push_back(VehicleRoute());
    VehicleRoute& route = routes.back();

    route.roadId = roadRoute.roadId;
    route.outgoingVertexId = roadRoute.nextVertexId;
    road.GetLaneVertices(currentLaneNumber, true/*waypointFromAdditionalStartPosition*/, currentPos, route.remainingWaypoints);

    assert(route.remainingWaypoints.size() > 1);

    // Adjust Bus Agent direction.

    if ((*this).GetBehaviorType() != AGENT_BEHAVIOR_BUS_DRIVER) {
        if (route.remainingWaypoints.front() != resource.Position()) {
            route.remainingWaypoints.push_front(resource.Position());
        }
    }

    remainingDistanceToIntersection = CalculateArcDistance(route.remainingWaypoints);

    assert(!road.IsBuildingParkingRoad());

    const Road& currentVehicleRoad = theAgentGisPtr->GetAgentRoad(vehicle.GetRoadId()).GetGisRoad();

    if ((GetBehaviorType() == AGENT_BEHAVIOR_VEHICLE) &&
        (currentVehicleRoad.IsBuildingParkingRoad())) {

        const BuildingIdType buildingId = currentVehicleRoad.GetBuildingId();

        theAgentGisPtr->LeaveFromParking(resource, buildingId);

    }//if//

    assert(!routes.front().remainingWaypoints.empty());
    vehicle.SetNextVelocity(velocity);
    vehicle.SetNextDistanceToIntersection(remainingDistanceToIntersection);
    vehicle.SetNextLaneNumber(currentLaneNumber);
    vehicle.SetNextRoadId(route.roadId);

    resource.SetDesiredNextPositionId(GIS_ROAD, route.roadId);

    viewdRouteNumber++;

    (*this).UpdateVisibleRouteIfNecessary();

    (*this).SetNextWaypoint();
}

void VehicleDriverBehavior::RideOnVehicle()
{
    pedestrianPtr.reset();

    isOnVehicle = true;

    vehiclePtr->runningDistance = 0;

    (*this).AssignFirstRoute();
}

void VehicleDriverBehavior::ViewFrontStatus(
    double& distanceToStop,
    double& frontVehicleVelocity,
    double& minDistanceToFrontVehicle)
{
    const Vehicle& vehicle = *vehiclePtr;

    assert(!routes.empty());
    const VehicleRoute& currentRoute = routes.front();
    const AgentRoad& currentRoad = theAgentGisPtr->GetAgentRoad(currentRoute.roadId);

    distanceToStop = DBL_MAX;
    frontVehicleVelocity = DBL_MAX;
    minDistanceToFrontVehicle = 1.38; // [m]

    if (currentRoad.HasFrontVehicle(*vehiclePtr, currentLaneNumber)) {
        const Vehicle& frontVehicle =
            currentRoad.GetFrontVehicle(*vehiclePtr, currentLaneNumber);

        //assert(frontVehicle.GetDistanceToIntersection() <= remainingDistanceToIntersection);

        if (frontVehicle.GetDistanceToIntersection() < remainingDistanceToIntersection) {

            const double distanceToFront =
                remainingDistanceToIntersection - frontVehicle.GetDistanceToIntersection();

            distanceToStop = std::min(
                distanceToStop,
                std::max(
                    0., distanceToFront - (frontVehicle.GetHalfBodyLength() + vehicle.GetHalfBodyLength()))); // for intermediate dest
            frontVehicleVelocity = frontVehicle.GetVelocityMetersPerSec();

        } else {
            distanceToStop = 0;
            frontVehicleVelocity = 0;
        }
        return;
    }

    const double remainingDistanceToStop = remainingDistanceToIntersection;

    const double nearStopDistance = 100;
    if (remainingDistanceToStop > nearStopDistance) {
        return;
    }

    // next intersection is end vertex
    if (routes.empty()) {
        frontVehicleVelocity = 0;
        minDistanceToFrontVehicle = 0;
        return;
    }

    distanceToStop = std::max(0., remainingDistanceToIntersection - vehicle.GetHalfBodyLength());

    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();
    const int numberVisibleTurns = 3;
    const TimeType currentTime = resource.CurrentTime();

    int numberVehicleRouteTurns = 0;
    bool foundStopPoint = false;

    size_t lastLaneNumber = currentLaneNumber;

    for(size_t i = 0;
        ((i < routes.size() - 1) &&
         (numberVehicleRouteTurns < numberVisibleTurns) &&
         (!foundStopPoint)); i++) {

        const VehicleRoute& route1 = routes[i];
        VehicleRoute& route2 = routes[i+1];

        const IntersectionIdType intersectionId =
            subsystem.GetIntersectionId(route1.outgoingVertexId);

        const AgentIntersection& intersection =
            theAgentGisPtr->GetAgentIntersection(intersectionId);

        const AgentRoad& lastRoad =
            theAgentGisPtr->GetAgentRoad(route1.roadId);

        const AgentRoad& nextRoad =
            theAgentGisPtr->GetAgentRoad(route2.roadId);

        if (currentTime < route2.entranceEnabledTime) {
            frontVehicleVelocity = 0;
            minDistanceToFrontVehicle = 0;
            foundStopPoint = true;
            break;
        }

        const bool isApproachLane =
            lastRoad.GetGisRoad().CanApproach(lastLaneNumber, route2.roadId);

        if (!isApproachLane) {
            foundStopPoint = true;
            break;
        }

        bool seeingAPedestrian = false;

        if (lastRoad.CrossingAPedestrian(route1.outgoingVertexId) ||
            nextRoad.CrossingAPedestrian(route1.outgoingVertexId)) {

            HighQualityRandomNumberGenerator& randomNumberGenerator = resource.GetRandomNumberGenerator();

            seeingAPedestrian =
                (randomNumberGenerator.GenerateRandomDouble() < resource.SeeingPedestrianProbability());
        }

        const size_t nextLaneNumer =
            lastRoad.GetGisRoad().GetNextLaneNumber(lastLaneNumber, route2.roadId);

        bool canPassIntersection;

        intersection.CanEnterTheRoad(
            currentTime,
            *this,
            route1.outgoingTurnType,
            route1.roadId,
            route2.roadId,
            canPassIntersection,
            route2.entranceEnabledTime);

        const bool nextRoadHasEnoughCapacity =
            nextRoad.AllowedIncomingVehicle(
                vehicle,
                nextLaneNumer);

        const bool isStopVertex =
            (*this).IsStopPoint(route1.outgoingVertexId);

        if (isStopVertex) {
            distanceToStop += vehicle.GetHalfBodyLength() + vehicle.GetVehicleConstant().minVehicleGap;
            frontVehicleVelocity = 0;
            minDistanceToFrontVehicle = 0;
            foundStopPoint = true;

        } else if (seeingAPedestrian ||
                   !canPassIntersection ||
                   !nextRoadHasEnoughCapacity) {

            // at least, stop short of intersection
            frontVehicleVelocity = 0;
            minDistanceToFrontVehicle = 0;
            foundStopPoint = true;

        } else if (nextRoad.HasVehicle(nextLaneNumer)) {

            const Vehicle& tailVehicle =
                nextRoad.GetTailVehicle(nextLaneNumer);

            const double intersectionToTailDistancfe =
                intersection.ref.GetVertex().DistanceTo(
                    tailVehicle.GetPosition());

            distanceToStop += intersectionToTailDistancfe;
            frontVehicleVelocity = tailVehicle.GetVelocityMetersPerSec();
            foundStopPoint = true;

        } else {
            distanceToStop += nextRoad.GetRoadLength();

            if (route1.outgoingTurnType.direction != ROAD_TURN_STRAIGHT) {
                numberVehicleRouteTurns++;
            }
        }

        lastLaneNumber = nextLaneNumer;
    }

    if (!foundStopPoint) {
        distanceToStop = DBL_MAX;
    }
}

bool VehicleDriverBehavior::YieldRoadTo(
    const Vehicle& otherVehicle,
    const double yieldDistanceTimeSec) const
{
    if (otherVehicle.GetVelocityMetersPerSec() == 0) {
        return false;
    }

    const Vehicle& vehicle = *vehiclePtr;
    const double distanceToIntersection = vehicle.GetDistanceToIntersection();
    const double minVelocityAtIntersection = 1;
    const double vehicleVelocity = std::max(minVelocityAtIntersection, vehicle.GetVelocityMetersPerSec());

    const double otherDistanceToIntersection = otherVehicle.GetDistanceToIntersection();
    const double otherVelocity = std::max(minVelocityAtIntersection, otherVehicle.GetVelocityMetersPerSec());

    double timeSecToIntersection = 0.0;
    double otherTimeSecToIntersection = 0.0;

    if (vehicleVelocity > 0) {
        if (distanceToIntersection >= vehicle.GetHalfBodyLength()) {
            timeSecToIntersection = (distanceToIntersection - vehicle.GetHalfBodyLength()) / vehicleVelocity;
        }
    }

    if (otherVelocity > 0) {
        if (otherDistanceToIntersection >= otherVehicle.GetHalfBodyLength()) {
            otherTimeSecToIntersection = (otherDistanceToIntersection - otherVehicle.GetHalfBodyLength()) / otherVelocity;
        }
    }

    if (timeSecToIntersection - otherTimeSecToIntersection > yieldDistanceTimeSec) {
        return true;
    }

    // Other vehicle will pass the intersection first.
    if (otherTimeSecToIntersection < timeSecToIntersection) {
        if (otherTimeSecToIntersection < vehicle.GetVehicleConstant().otherVehicleEntranceTime) {
            return true;
        }
    }

    return false;
}

void VehicleDriverBehavior::RestartRun()
{
    if (!routes.empty()) {
        Vehicle& vehicle = *vehiclePtr;

        if (!routes.empty()) {
            const VehicleRoute& nextRoute = routes.front();
            const AgentRoad& agentRoad = theAgentGisPtr->GetAgentRoad(nextRoute.roadId);
            const Road& road = agentRoad.GetGisRoad();

            currentLaneNumber =
                road.GetNearestOutgoingLaneNumber(resource.LastVertexId(), resource.Position());

            vehicle.SetNextLaneNumber(currentLaneNumber);
            vehicle.SetNextRoadId(nextRoute.roadId);
            vehicle.SetNextTurnType(nextRoute.outgoingTurnType);

            const double congestion =
                agentRoad.CalculateVehicleCongestion(currentLaneNumber);

            resource.SetCongestion(congestion);
            resource.SetDesiredNextPositionId(GIS_ROAD, nextRoute.roadId);
        }
    }
}

void VehicleDriverBehavior::EndBehaviorAtViaPoint(const shared_ptr<AgentRoute>& nextRoutePtr)
{
    routePtr.reset(new AgentRoute());

    viewdRouteNumber = 0;
}

VertexIdType VehicleDriverBehavior::GetViaPointVertexId() const
{
    if (!routes.empty()) {
        const VehicleRoute& currentRoute = routes.front();
        const double enoughSlowVelocity = 1;//m/s

        if (velocity < enoughSlowVelocity) {
            return currentRoute.outgoingVertexId;
        }

        return routes.back().outgoingVertexId;
    }

    return resource.LastVertexId();
}

bool VehicleDriverBehavior::IsAcceptableRouteChange(const AgentRoute& route) const
{
    return (route.behavior == AGENT_BEHAVIOR_VEHICLE);
}

void VehicleDriverBehavior::ChangeRoute(const shared_ptr<AgentRoute>& newRoutePtr)
{
    routePtr = newRoutePtr;

    viewdRouteNumber = 0;

    if (routes.empty()) {
        (*this).AssignFirstRoute();
    }
}

void VehicleDriverBehavior::ForceStopVehicle()
{
    routes.clear();

    passingIntersection = false;
    routes.clear();
    remainingDistanceToIntersection = 0;
    remainingShortStopTime = ZERO_TIME;

    remainingPathDistance = DBL_MAX;

    viewdRouteNumber = 0;
}

void VehicleDriverBehavior::TryInternalRouteChange(
    const VertexIdType& startVertexId,
    const VertexIdType& endVertexId,
    bool& foundRoute)
{
    if (startVertexId == endVertexId) {
        foundRoute = true;
        return;
    }

    shared_ptr<AgentRoute> newRoutePtr(new AgentRoute());

    assert(routePtr != nullptr);
    theAgentGisPtr->SearchRoadRoute(
        resource,
        startVertexId,
        endVertexId,
        (*this).GetBehaviorType(),
        *newRoutePtr);

    if (newRoutePtr->IsEmpty()) {

        foundRoute = false;

    } else {
        routePtr = newRoutePtr;
        viewdRouteNumber = 0;

        if (!routes.empty()) {
            // special case for slow vehicle speed

            if (routes.front().outgoingVertexId == startVertexId) {
                while (routes.size() >= 2) {
                    routes.pop_back();
                }
            }
        }

        if (routes.empty()) {
            (*this).AssignFirstRoute();
        }

        foundRoute = true;
    }
}

//-----------------------------------------------------------

BusDriverBehavior::BusDriverBehavior(
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const AgentResource& initResource,
    const shared_ptr<Vehicle>& initVehiclePtr,
    const shared_ptr<Bus>& initBusPtr,
    const TimeType& initMinShortStopTime)
    :
    VehicleDriverBehavior(
        initAgentGisPtr,
        initPublicVehicleTablePtr,
        initBusPtr->routePtr,
        initResource,
        initVehiclePtr,
        initMinShortStopTime),
    busPtr(initBusPtr)
{
    vehiclePtr->SetVehicleConstant(
        resource.MakeVehicleConstant(busPtr->GetVehicleFamily().lengthMeters));

    const BusStopIdType busStopId = busPtr->GetStartBusStopId();
    const TimeType currentTime = resource.CurrentTime();

    resource.SetCurrentPositionId(GIS_BUSSTOP, busStopId);

    theAgentGisPtr->EnterToBusStop(resource, busStopId, busPtr);

    remainingShortStopTime =
        std::max(minShortStopTime, busPtr->GetDepartureTime(currentTime) - currentTime);
}

BusDriverBehavior::~BusDriverBehavior()
{
    vehiclePtr->roadId = INVALID_VARIANT_ID;
    vehiclePtr->nextRoadId = vehiclePtr->roadId;
}

bool BusDriverBehavior::HasFinished() const
{
    return (!busPtr->HasAnAgent() &&
            busPtr->ArrivedAtTheLastBusStop(resource.CurrentTime()));
}

void BusDriverBehavior::IncrementTimeStep(const TimeType& timeStep)
{
    if (remainingShortStopTime > ZERO_TIME) {
        const TimeType currentTime = resource.CurrentTime();

        remainingShortStopTime =
            std::max(remainingShortStopTime, busPtr->GetDepartureTime(currentTime) - currentTime);
    }

    (*this).VehicleDriverBehavior::IncrementTimeStep(timeStep);
}

bool BusDriverBehavior::IsStopPoint(const VertexIdType& vertexId) const
{
    const BusStopIdType nextBusStopId = busPtr->GetNextBusStopId();

    return (theAgentGisPtr->GetAgentBusStop(nextBusStopId).ref.GetVertexId() == vertexId);
}

void BusDriverBehavior::ArrivedAtVertex(const VertexIdType& vertexId)
{
    if (busPtr->GetNextBusStopVertexId() == vertexId) {

        const BusStopIdType busStopId = busPtr->GetNextBusStopId();
        const TimeType currentTime = resource.CurrentTime();

        (*this).ParkingVehicleIfPossible(
            vertexId,
            std::max(minShortStopTime, busPtr->GetDepartureTime(currentTime) - currentTime));

        busPtr->ArrivedAtBusStop(currentTime);

        theAgentGisPtr->EnterToBusStop(resource, busStopId, busPtr);

    }//if//
}

void BusDriverBehavior::TryInternalRouteChange(
    const VertexIdType& startVertexId,
    const VertexIdType& endVertexId,
    bool& foundRoute)
{
    if (routePtr->IsEmpty()) {
        foundRoute = false;
        return;
    }

    const RoadLayer& roadLayer =
        *theAgentGisPtr->GetSubsystem().GetRoadLayerPtr();

    vector<VertexIdType> vertexIds;

    vertexIds.push_back(startVertexId);

    shared_ptr<AgentRoute> newBusRoutePtr(new AgentRoute());

    for(size_t i = busPtr->GetNextStopNumber(); i < busPtr->GetNumberOfStops(); i++) {
        vertexIds.push_back(roadLayer.GetBusStop(busPtr->GetBusStopId(i)).GetVertexId());
    }

    for(size_t i = 0; (i + 1) < vertexIds.size(); i++) {
        const VertexIdType& routeStartVertexId = vertexIds[i];
        const VertexIdType& routeEndVertexId = vertexIds[i+1];

        AgentRoute aRoute;

        theAgentGisPtr->SearchRoadRoute(
            resource,
            routeStartVertexId,
            routeEndVertexId,
            AGENT_BEHAVIOR_BUS,
            aRoute);

        if (aRoute.IsEmpty()) {
            foundRoute = false;
            return;
        }

        newBusRoutePtr->roadRoutes.insert(
            newBusRoutePtr->roadRoutes.end(),
            aRoute.roadRoutes.begin(),
            aRoute.roadRoutes.end());
    }

    routePtr = newBusRoutePtr;
    busPtr->routePtr = newBusRoutePtr;

    viewdRouteNumber = 0;

    if (routes.empty()) {
        (*this).AssignFirstRoute();
    }

    foundRoute = true;
}

VertexIdType BusDriverBehavior::GetFixedDestinationVertexId() const
{
    return routePtr->GetDestVertexId();
}

//-----------------------------------------------------------

TrainDriverBehavior::TrainDriverBehavior(
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const AgentResource& initResource,
    const shared_ptr<Train>& initTrainPtr)
    :
    AgentBehavior(initAgentGisPtr, initPublicVehicleTablePtr,  shared_ptr<AgentRoute>(new AgentRoute()), initResource),
    trainPtr(initTrainPtr)
{
}

TrainDriverBehavior::~TrainDriverBehavior()
{
}

bool TrainDriverBehavior::HasFinished() const
{
    const TimeType currentTime = resource.CurrentTime();
    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();
    const RailRoadLayer& railRoadLayer = *subsystem.GetRailRoadLayerPtr();

    return (railRoadLayer.IsRailRoadLineAvailable(trainPtr->GetRailRoadLineId()) &&
            currentTime >= trainPtr->GetGarbageTime());
}

void TrainDriverBehavior::IncrementTimeStep(const TimeType& timeStep)
{
    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();
    const RailRoadLayer& railRoadLayer = *subsystem.GetRailRoadLayerPtr();

    if (!railRoadLayer.IsRailRoadLineAvailable(trainPtr->GetRailRoadLineId())) {
        resource.SetPosition(resource.Position());
        return;
    }

    const TimeType currentTime = resource.CurrentTime();

    trainPtr->SetTimeTo(currentTime);

    resource.SetPosition(trainPtr->GetPosition());
}

//-----------------------------------------------------------

TaxiDriverBehavior::TaxiDriverBehavior(
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const AgentResource& initResource,
    const shared_ptr<Taxi>& initTaxiPtr,
    const TimeType& initMinShortStopTime)
    :
    VehicleDriverBehavior(
        initAgentGisPtr,
        initPublicVehicleTablePtr,
        shared_ptr<AgentRoute>(),
        initResource,
        initTaxiPtr,
        initMinShortStopTime),
    state(DRIVER_STATE_AT_HOME),
    taxiPtr(initTaxiPtr)
{
    vehiclePtr->SetVehicleConstant(resource.MakeVehicleConstant());
}

TaxiDriverBehavior::~TaxiDriverBehavior()
{
    vehiclePtr->SetNextRoadId(INVALID_VARIANT_ID);
}

bool TaxiDriverBehavior::HasNoCustomerAndReservationAtHome() const
{
    return (!taxiPtr->ContainsCustomer() &&
            !taxiPtr->HasReservation() &&
            resource.LastVertexId() == taxiPtr->GetHomeVertexId() &&
            state == DRIVER_STATE_AT_HOME);
}

bool TaxiDriverBehavior::IsStopPoint(const VertexIdType& vertexId) const
{
    return (vertexId == nextStopVertexId);
}

void TaxiDriverBehavior::RestartRun()
{
    if (nextStopVertexId == resource.LastVertexId()) {

        resource.OutputTrace("Departed");

        (*this).DecideRoute();

        (*this).AssignFirstRoute();

    } else {
        VehicleDriverBehavior::RestartRun();
    }
}

void TaxiDriverBehavior::IncrementTimeStep(const TimeType& timeStep)
{
    if ((*this).HasNoCustomerAndReservationAtHome()) {

        resource.SetPosition(resource.Position());
        return;
    }

    if (state == DRIVER_STATE_AT_HOME) {

        if (taxiPtr->HasReservation()) {
            (*this).DecideRoute();
        }

    } else if (state == DRIVER_STATE_TRANSPORT) {

        if (!taxiPtr->ContainsCustomer()) {

            (*this).DecideRoute();

        } else if (taxiPtr->HasCustomerRequestRoute()) {

            (*this).DecideRoute();
        }
    }

    (*this).VehicleDriverBehavior::IncrementTimeStep(timeStep);
}

void TaxiDriverBehavior::DecideRoute()
{
    const VertexIdType viapointVertexId = (*this).GetViaPointVertexId();

    shared_ptr<AgentRoute> newRoutePtr;

    if (taxiPtr->ContainsCustomer()) {
        state = DRIVER_STATE_TRANSPORT;

        assert(taxiPtr->HasCustomerRequestRoute());
        newRoutePtr = taxiPtr->TakeCustomerRequestRoute();

        nextStopVertexId = newRoutePtr->GetDestVertexId();

    } else {
        if (taxiPtr->HasReservation()) {

            state = DRIVER_STATE_GO_TO_CUSTOMER;
            nextStopVertexId = taxiPtr->GetReservedCustomer().LastVertexId();

        } else if (state == DRIVER_STATE_AT_HOME) {

            return;

        } else {
            state = DRIVER_STATE_GO_BACK_HOME;
            nextStopVertexId = taxiPtr->GetHomeVertexId();
        }

        newRoutePtr.reset(new AgentRoute());

        theAgentGisPtr->SearchRoadRoute(
            resource,
            viapointVertexId,
            nextStopVertexId,
            AGENT_BEHAVIOR_TAXI,
            *newRoutePtr);
    }

    if (state == DRIVER_STATE_TRANSPORT) {
        resource.OutputTrace("RouteToTransport " + newRoutePtr->ConvertToString());
    } else if (state == DRIVER_STATE_GO_TO_CUSTOMER) {
        resource.OutputTrace("RouteToCustomer " + newRoutePtr->ConvertToString());
    } else {
        resource.OutputTrace("RouteToHome " + newRoutePtr->ConvertToString());
    }

    (*this).ChangeRoute(newRoutePtr);

    if (nextStopVertexId == resource.LastVertexId()) {
        (*this).ArrivedAtVertex(nextStopVertexId);
    }
}

void TaxiDriverBehavior::ArrivedAtVertex(const VertexIdType& vertexId)
{
    if (nextStopVertexId == vertexId) {
        resource.OutputTrace("Arrived");

        if (state == DRIVER_STATE_GO_TO_CUSTOMER) {

            (*this).ParkingVehicleForNextCustomer();

        } else if (state == DRIVER_STATE_TRANSPORT) {

            taxiPtr->ArrivedAtTheDestinationForCurrentCustomer();

        } else {

            (*this).ParkingVehicleIfPossible(vertexId, minShortStopTime);

            if (state == DRIVER_STATE_GO_BACK_HOME) {
                theAgentGisPtr->ReturnTaxiToStation(resource, taxiPtr);

                state = DRIVER_STATE_AT_HOME;
            }
        }
    }
}

void TaxiDriverBehavior::ParkingVehicleForNextCustomer()
{
    if (taxiPtr->HasReservation()) {
        const GisPositionIdType& positionId = taxiPtr->GetCustomerPosition();
        assert(positionId.type == GIS_ROAD);
        vehiclePtr->SetNextRoadId(positionId.id);
        currentLaneNumber = 0;

        vehiclePtr->SetNextLaneNumber(currentLaneNumber);
        resource.SetDesiredNextPositionId(positionId);
        remainingShortStopTime += std::max(minShortStopTime, 10 * SECOND);

        taxiPtr->SetEnableCustomerPickupState();

    } else {

        (*this).ParkingVehicleIfPossible(nextStopVertexId, minShortStopTime);

    }
}

void TaxiDriverBehavior::TryInternalRouteChange(
    const VertexIdType& startVertexId,
    const VertexIdType& endVertexId,
    bool& foundRoute)
{
    VehicleDriverBehavior::TryInternalRouteChange(startVertexId, endVertexId, foundRoute);

    nextStopVertexId = endVertexId;
}

bool TaxiDriverBehavior::HasFixedDestinationVertex() const
{
    if (state == DRIVER_STATE_GO_TO_CUSTOMER ||
        state == DRIVER_STATE_GO_BACK_HOME ||
        state == DRIVER_STATE_AT_HOME) {
        return true;
    }

    return false;
}

VertexIdType TaxiDriverBehavior::GetFixedDestinationVertexId() const
{
    assert((*this).HasFixedDestinationVertex());

    return routePtr->GetDestVertexId();
}

//-----------------------------------------------------------

TaxiGuestBehavior::TaxiGuestBehavior(
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<AgentRoute>& initRoutePtr,
    const AgentResource& initResource)
    :
    AgentBehavior(initAgentGisPtr, initPublicVehicleTablePtr, initRoutePtr, initResource),
    tookOffTaxi(false),
    succeededTaxiCall(false),
    routeRecalculationHasExecuted(false),
    arrivalTime(resource.CurrentTime()),
    isOnTaxi(false),
    viaPointVertexId(INVALID_VERTEX_ID),
    taxiCenterId(INVALID_TAXICENTER_ID)
{
    // call taxi
    theAgentGisPtr->CallTaxi(resource, ANY_TAXICENTER_ID, succeededTaxiCall, taxiCenterId);


    assert(routePtr->IsRoad());

    const RoadIdType& roadId = routePtr->roadRoutes.front().roadId;

    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();
    const VertexIdType& vertexId = resource.LastVertexId();

     if (subsystem.IsParkingVertex(vertexId)) {
        resource.SetDesiredNextPositionId(GIS_ROAD, subsystem.GetParkingRoadId(vertexId));
     }
     else {
        resource.SetDesiredNextPositionId(GIS_ROAD, roadId);
     }
}

TaxiGuestBehavior::~TaxiGuestBehavior()
{
    (*this).TakeoffTaxi();
}

void TaxiGuestBehavior::TakeoffTaxi()
{
    if (tookOffTaxi || taxiPtr == nullptr) {
        return;
    }

    tookOffTaxi = true;

    taxiPtr->RemoveCustomerOrReservation(resource);

    resource.RemoveOwnerAgent();

    // Not set position id for entrance check when take off taxi.
    // Following behavior will set position id.
}

void TaxiGuestBehavior::IncrementTimeStep(const TimeType& timeStep)
{
    if (isOnTaxi) {

        const VertexIdType vertexId = taxiPtr->GetNextVertexId();

        resource.SetPosition(taxiPtr->GetNextPosition());
        resource.SetVertexId(vertexId);

        if (vertexId == viaPointVertexId ||
            taxiPtr->IsArrivedAtTheDestination()) {

            (*this).TakeoffTaxi();
        }

    } else {

        if (taxiPtr == nullptr) {

            if (!routeRecalculationHasExecuted &&
                (!succeededTaxiCall ||
                 resource.CurrentBehaviorSpentTime() > resource.TaxiCallWaitTime())) {

                routeRecalculationHasExecuted = true;

                if (succeededTaxiCall) {
                    theAgentGisPtr->GiveupTaxiCall(resource, taxiCenterId);
                }

                //set take off flag to discard(;finish) current behavior
                tookOffTaxi = true;

                // Recalculate other route so that the taxi center did not respond over waiting time.

                set<AgentBehaviorType> notAvailableBehaviorTypes;

                notAvailableBehaviorTypes.insert(AGENT_BEHAVIOR_TAXI);

                resource.RecalculateRouteWithNotAvailableBehaviorSpecification(
                    notAvailableBehaviorTypes);

            }

        } else if (taxiPtr->CanPickUp(resource)) {

            const VertexIdType vertexId = taxiPtr->GetNextVertexId();

            if (vertexId == viaPointVertexId) {

                // Route has changed
                (*this).GiveupTaxiWaiting();

            } else {
                taxiPtr->TakeOnReservedCustomer(resource, routePtr);

                // Set position id for entrance check
                // Add an entrance capacity to last road.

                resource.SetDesiredNextPositionId(GisPositionIdType(GIS_ROAD, INVALID_VARIANT_ID));

                isOnTaxi = true;
            }
        } else if (!isOnTaxi) {

            if (resource.LastVertexId() == viaPointVertexId) {

                (*this).GiveupTaxiWaiting();
            }

        }

        resource.SetPosition(resource.Position());
    }
}

void TaxiGuestBehavior::EndBehaviorAtViaPoint(const shared_ptr<AgentRoute>& nextRoutePtr)
{
    routePtr.reset(new AgentRoute());

    if (isOnTaxi) {
        taxiPtr->SetCustomerRequestRoute(routePtr);
    }

    viaPointVertexId = (*this).GetViaPointVertexId();
}

VertexIdType TaxiGuestBehavior::GetViaPointVertexId() const
{
    if (isOnTaxi) {
        return taxiPtr->GetViaPointVertexId();
    }

    return resource.LastVertexId();
}

void TaxiGuestBehavior::AssignTaxi(const shared_ptr<Taxi>& initTaxiPtr)
{
    if (taxiPtr == nullptr) {
        taxiPtr = initTaxiPtr;

        resource.SetOwnerAgent(taxiPtr->GetDriverAgentId());

        taxiPtr->AddReservedCustomer(resource);
    }
}

bool TaxiGuestBehavior::IsAcceptableRouteChange(const AgentRoute& route) const
{
    return (route.behavior == AGENT_BEHAVIOR_TAXI);
}

void TaxiGuestBehavior::ChangeRoute(const shared_ptr<AgentRoute>& newRoutePtr)
{
    routePtr = newRoutePtr;

    if (isOnTaxi) {
        taxiPtr->SetCustomerRequestRoute(newRoutePtr);
    }

    viaPointVertexId = INVALID_VERTEX_ID;
}

void TaxiGuestBehavior::TryInternalRouteChange(
    const VertexIdType& startVertexId,
    const VertexIdType& endVertexId,
    bool& foundRoute)
{
    assert(isOnTaxi);

    if (startVertexId == endVertexId) {
        foundRoute = true;
        return;
    }

    shared_ptr<AgentRoute> newRoutePtr(new AgentRoute());

    assert(routePtr != nullptr);
    theAgentGisPtr->SearchRoadRoute(
        resource,
        startVertexId,
        endVertexId,
        (*this).GetBehaviorType(),
        *newRoutePtr);

    if (newRoutePtr->IsEmpty()) {

        foundRoute = false;

    } else {

        foundRoute = true;

        (*this).ChangeRoute(newRoutePtr);
    }
}

bool TaxiGuestBehavior::IsInternalRouteChaneMode() const
{
    return isOnTaxi;
}

//-----------------------------------------------------------
// Train
//-----------------------------------------------------------

PublicVehicleBehavior::PublicVehicleBehavior(
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<AgentRoute>& initRoutePtr,
    const AgentResource& initResource)
    :
    AgentBehavior(initAgentGisPtr, initPublicVehicleTablePtr, initRoutePtr, initResource),
    destPositionInVehicleMeters(0),
    currentPositionInVehicleMeters(0),
    relativePositionRadians(0),
    currentStopNumber(0),
    arrivedAtRideOnStop(false),
    tryRiding(false),
    stopArrivalTime(INFINITE_TIME),
    vehicleScheduleArrivalTime(INFINITE_TIME),
    viaPointVertexId(INVALID_VERTEX_ID),
    anchorDirectionRadians(0)
{
    assert(!routePtr->IsTransferLine(0));
}

PublicVehicleBehavior::~PublicVehicleBehavior()
{
    (*this).GetDownVehicle();
}

void PublicVehicleBehavior::IncrementTimeStep(const TimeType& timeStep)
{
    resource.SetPosition(resource.Position());

    if (tryRiding) {

        tryRiding = false;

        if (vehiclePtr->Contains(resource)) {

            (*this).DecidePositionInVehicle();

            resource.SetOwnerAgent(vehiclePtr->GetDriverAgentId());

            // Set position id for entrance check
            // Add an entrance capacity to last road.

            resource.SetDesiredNextPositionId(GisPositionIdType(GIS_RAILROAD, INVALID_VARIANT_ID));

            return;

        } else {
            const PublicVehicleRoute& vehicleRoute =
                routePtr->publicVehicleRoutes.at(currentStopNumber);

            if (vehiclePtr->GetVehicleNumber() == vehicleRoute.vehicleNumber) {
                HighQualityRandomNumberGenerator& randomNumberGenerator = resource.GetRandomNumberGenerator();

                if (randomNumberGenerator.GenerateRandomDouble() < resource.MissedVehiclePathQueryProbability()) {
                    resource.RecalculateRoute(vehiclePtr->GetDepartureTime(resource.CurrentTime()) + SECOND);

                    vehiclePtr.reset();

                    resource.OutputTrace("Route Recalculation for reaching a vehicle limit.");
                    return;
                }
            }

            vehiclePtr.reset();
        }
    }

    if ((*this).IsOnPublicVehicle()) {

        (*this).IncrementTimeStepInVehicle(timeStep);

    } else if (arrivedAtRideOnStop) {

        if (viaPointVertexId != INVALID_VERTEX_ID) {
            (*this).ForceStopRiding();
            return;
        }

        const PublicVehicleRoute& vehicleRoute =
            routePtr->publicVehicleRoutes.at(currentStopNumber);

        const GisPositionIdType positionId =
            thePublicVehicleTablePtr->GetPositionId(vehicleRoute.origStopId);

        // First, set position id for entrance check.
        // Entrance status may block behavior execution.

        if (resource.GetCurrentPositionId() != positionId) {

            resource.SetDesiredNextPositionId(positionId);

        } else {

            assert(vehicleRoute.lineId != TRANSFER_LINE_ID);

            if (!thePublicVehicleTablePtr->IsLineAvailable(vehicleRoute.lineId)) {
                (*this).ForceStopRiding();
                resource.RecalculateRoute();
                resource.OutputTrace("Route Recalculation for nonavailable line");
                return;
            }

            const TimeType currentTime = resource.CurrentTime();

            const double congestion =
                thePublicVehicleTablePtr->CalculateCongestion(
                    vehicleRoute.origStopId);

            const TimeType vehicleDelay =
                std::max(ZERO_TIME, currentTime - vehicleScheduleArrivalTime);

            resource.SetCongestion(congestion);

            if (resource.IsPathQueryTriggerAvailable(congestion, vehicleDelay)) {

                resource.RecalculateRoute();
                resource.SetLastPathQueryTriggerTime();

                resource.OutputTrace("Route Recalculation at next step");

            } else if (currentTime > vehicleScheduleArrivalTime &&
                       (*this).MissedPublicVehicle()) {

                HighQualityRandomNumberGenerator& randomNumberGenerator = resource.GetRandomNumberGenerator();

                if (randomNumberGenerator.GenerateRandomDouble() < resource.MissedVehiclePathQueryProbability()) {
                    (*this).ForceStopRiding();
                    resource.RecalculateRoute();
                    resource.SetLastPathQueryTriggerTime();
                    resource.OutputTrace("Route Recalculation for missed a waiting vehicle.");
                } else {
                    (*this).RideOnVehicleIfPossible();
                }

            } else {
                assert(vehicleRoute.lineId != TRANSFER_LINE_ID);

                (*this).RideOnVehicleIfPossible();
            }
        }

    } else if (currentPositionInVehicleMeters > 0) {
        const double walkDistanceMeters = resource.WalkSpeedMetersPerSec()*double(timeStep)/SECOND;

        currentPositionInVehicleMeters = std::max(
            0., currentPositionInVehicleMeters - walkDistanceMeters);

        (*this).UpdateAnchorPosition();

    } else if (currentStopNumber < routePtr->publicVehicleRoutes.size()) {
        if (pedestrianPtr == nullptr) {

            const PublicVehicleRoute& route =
                routePtr->publicVehicleRoutes.at(currentStopNumber);

            if (route.lineId == TRANSFER_LINE_ID) {

                const PublicVehicleRoute& prevRoute =
                    routePtr->publicVehicleRoutes.at(currentStopNumber - 1);

                const PublicVehicleRoute& nextRoute =
                    routePtr->publicVehicleRoutes.at(currentStopNumber + 1);

                (*this).SetPedestrianBehaviorTowardNextStop(
                    routePtr->GetDownStopId(currentStopNumber),
                    thePublicVehicleTablePtr->GetDestVertexId(prevRoute),
                    thePublicVehicleTablePtr->GetOrigEntranceVertexId(nextRoute, resource.Position()));

            } else {

                (*this).SetPedestrianBehaviorTowardNextStop(
                    routePtr->GetStopId(currentStopNumber),
                    resource.LastVertexId(),
                    thePublicVehicleTablePtr->GetOrigEntranceVertexId(route, resource.Position()));
            }
        }
        pedestrianPtr->IncrementTimeStep(timeStep);

        if (pedestrianPtr->HasFinished()) {
            arrivedAtRideOnStop = true;
            pedestrianPtr.reset();

            if (routePtr->IsTransferLine(currentStopNumber)) {
                currentStopNumber++;
            }

            const TimeType currentTime = resource.CurrentTime();

            stopArrivalTime = currentTime;

            const PublicVehicleRoute& vehicleRoute =
                routePtr->publicVehicleRoutes.at(currentStopNumber);

            vehicleScheduleArrivalTime =
                thePublicVehicleTablePtr->GetScheduledArrivalTime(
                    vehicleRoute.lineId,
                    vehicleRoute.routeId,
                    vehicleRoute.vehicleNumber,
                    vehicleRoute.origStopId,
                    currentTime);

            if ((*this).MissedPublicVehicle()) {

                HighQualityRandomNumberGenerator& randomNumberGenerator = resource.GetRandomNumberGenerator();

                if (randomNumberGenerator.GenerateRandomDouble() < resource.MissedVehiclePathQueryProbability()) {
                    resource.RecalculateRoute();
                    resource.OutputTrace("Route Recalculation for missed a vehicle at arrived at the stop.");
                }
            }
        }
    }
}

void PublicVehicleBehavior::SetPedestrianBehaviorTowardNextStop(
    const StopIdType& stopId,
    const VertexIdType& startVertexId,
    const VertexIdType& destVertexId)
{
    const TimeType arrivalTimeMargin = 60*SECOND;

    shared_ptr<AgentRoute> roadRoutePtr(new AgentRoute());

    theAgentGisPtr->SearchRoadRoute(
        resource,
        startVertexId,
        destVertexId,
        AGENT_BEHAVIOR_PEDESTRIAN,
        AGENT_BEHAVIOR_PEDESTRIAN,
        arrivalTimeMargin,
        *roadRoutePtr);

    resource.OutputTrace("WalkToStop " + thePublicVehicleTablePtr->GetStopName(stopId) + " " + roadRoutePtr->ConvertToString());

    pedestrianPtr.reset(
        new PedestrianBehavior(
            theAgentGisPtr,
            thePublicVehicleTablePtr,
            roadRoutePtr,
            theAgentGisPtr->GetVertex(destVertexId),
            resource));
}

void PublicVehicleBehavior::RideOnVehicleIfPossible()
{
    const PublicVehicleRoute& vehicleRoute =
        routePtr->publicVehicleRoutes.at(currentStopNumber);

    if (thePublicVehicleTablePtr->IsStation(vehicleRoute.origStopId)) {
        const RailRoadStationIdType& stationId =
            thePublicVehicleTablePtr->GetStationId(vehicleRoute.origStopId);

        const AgentStation& station = theAgentGisPtr->GetAgentStation(stationId);

        vehiclePtr = station.GetTrain(vehicleRoute.lineId, vehicleRoute.routeId, getDownedVehicleIds, resource);

    } else {
        const BusStopIdType& busStopId =
            thePublicVehicleTablePtr->GetBusStopId(vehicleRoute.origStopId);

        const AgentBusStop& busStop = theAgentGisPtr->GetAgentBusStop(busStopId);

        vehiclePtr = busStop.GetBus(vehicleRoute, getDownedVehicleIds, resource);
    }

    if (vehiclePtr != nullptr) {
        theAgentGisPtr->RideOnPublicVehicle(resource, resource.CurrentTime() - stopArrivalTime, vehiclePtr);
        tryRiding = true;
    }

    resource.SetPosition(resource.Position());
}

void PublicVehicleBehavior::DecidePositionInVehicle()
{
    HighQualityRandomNumberGenerator& randomNumberGenerator =
        resource.GetRandomNumberGenerator();

    const PublicVehicle& vehicle = (*vehiclePtr);
    const VehicleFamily& vehicleFamily = vehicle.GetVehicleFamily();

    const double randomPositionInVehicleY =
        vehicleFamily.lengthMeters*randomNumberGenerator.GenerateRandomDouble() -
        vehicleFamily.lengthMeters/2.;

    const double randomPositionInVehicleX =
        vehicleFamily.widthMeters*randomNumberGenerator.GenerateRandomDouble() -
        vehicleFamily.widthMeters/2.;

    const Vertex randomPositionInVehicle(
        randomPositionInVehicleX,
        randomPositionInVehicleY);

    destPositionInVehicleMeters = randomPositionInVehicle.XYDistance();

    relativePositionRadians =
        randomPositionInVehicle.DirectionRadians();
}

void PublicVehicleBehavior::GetDownVehicle()
{
    if (vehiclePtr != nullptr) {
        const PublicVehicleRoute& vehicleRoute =
            routePtr->publicVehicleRoutes.at(currentStopNumber);

        resource.SetDesiredNextPositionId(vehiclePtr->GetViaPointPositionId(resource.CurrentTime()));

        if (vehiclePtr->Contains(resource)) {
            theAgentGisPtr->GetDownPublicVehicle(resource, vehiclePtr);
            getDownedVehicleIds.insert(vehiclePtr->GetPublicVehicleId());
        }

        vehiclePtr.reset();
        arrivedAtRideOnStop = false;

        currentStopNumber++;

        resource.RemoveOwnerAgent();
    }
}

void PublicVehicleBehavior::IncrementTimeStepInVehicle(const TimeType& timeStep)
{
    const double walkDistanceMeters = resource.WalkSpeedMetersPerSec()*double(timeStep)/SECOND;
    const StopIdType& getDownStopId = routePtr->GetDownStopId(currentStopNumber);

    anchorPosition = vehiclePtr->GetNextPosition();
    anchorDirectionRadians = vehiclePtr->GetNextDirectionRadians();

    bool moveInVehicle = false;

    if (vehiclePtr->AtStop(resource.CurrentTime())) {

        const TimeType currentTime = resource.CurrentTime();
        const VertexIdType vertexId = vehiclePtr->GetViaPointVertexId(currentTime);

        resource.SetVertexId(vertexId);

        if (vehiclePtr->GetStopId() == getDownStopId) {

            (*this).GetDownVehicle();

        } else if (vertexId == viaPointVertexId) {

            (*this).ForceStopRiding();

        } else {
            moveInVehicle = true;
        }
    } else {
        moveInVehicle = true;
    }

    if (moveInVehicle) {
        currentPositionInVehicleMeters = std::min(
            destPositionInVehicleMeters,
            currentPositionInVehicleMeters + walkDistanceMeters);
    }

    (*this).UpdateAnchorPosition();
}

void PublicVehicleBehavior::UpdateAnchorPosition()
{
    const double absPositionRadians =
        relativePositionRadians + (anchorDirectionRadians - PI/2.);

    const Vertex positionInVehicle(
        currentPositionInVehicleMeters * (std::cos(absPositionRadians)),
        currentPositionInVehicleMeters * (std::sin(absPositionRadians)));

    resource.SetPosition(anchorPosition + positionInVehicle);
}

bool PublicVehicleBehavior::HasFinished() const
{
    return (vehiclePtr == nullptr &&
            pedestrianPtr == nullptr &&
            currentStopNumber >= routePtr->publicVehicleRoutes.size() &&
            currentPositionInVehicleMeters == 0);
}

void PublicVehicleBehavior::ForceStopRiding()
{
    (*this).GetDownVehicle();

    arrivedAtRideOnStop = false;
    tryRiding = false;

    pedestrianPtr.reset();
    currentStopNumber = routePtr->publicVehicleRoutes.size();
}

bool PublicVehicleBehavior::IsOnPublicVehicle() const
{
    return (vehiclePtr != nullptr);
}

bool PublicVehicleBehavior::MissedPublicVehicle() const
{
    const PublicVehicleRoute& vehicleRoute =
        routePtr->publicVehicleRoutes.at(currentStopNumber);

    if (thePublicVehicleTablePtr->IsStation(vehicleRoute.origStopId)) {
        const RailRoadStationIdType& stationId =
            thePublicVehicleTablePtr->GetStationId(vehicleRoute.origStopId);

        const AgentStation& station = theAgentGisPtr->GetAgentStation(stationId);

        return station.HasLeft(vehicleRoute.lineId, vehicleRoute.routeId, vehicleRoute.vehicleNumber);

    } else {
        const BusStopIdType& busStopId =
            thePublicVehicleTablePtr->GetBusStopId(vehicleRoute.origStopId);

        const AgentBusStop& busStop = theAgentGisPtr->GetAgentBusStop(busStopId);

        return busStop.HasLeft(vehicleRoute.lineId, vehicleRoute.routeId, vehicleRoute.vehicleNumber);
    }
}

void PublicVehicleBehavior::EndBehaviorAtViaPoint(const shared_ptr<AgentRoute>& nextRoutePtr)
{
    viaPointVertexId = (*this).GetViaPointVertexId();
}

VertexIdType PublicVehicleBehavior::GetViaPointVertexId() const
{
    if ((*this).IsOnPublicVehicle()) {

        return vehiclePtr->GetViaPointVertexId(resource.CurrentTime());

    } else if (arrivedAtRideOnStop) {

        return resource.LastVertexId();

    } else if (pedestrianPtr != nullptr) {

        return pedestrianPtr->GetRoute().GetDestVertexId();
    }

    return resource.LastVertexId();
}

bool PublicVehicleBehavior::IsAcceptableRouteChange(const AgentRoute& route) const
{
    return (route.behavior == AGENT_BEHAVIOR_BUS ||
            route.behavior == AGENT_BEHAVIOR_TRAIN);
}

void PublicVehicleBehavior::ChangeRoute(const shared_ptr<AgentRoute>& newRoutePtr)
{
    routePtr = newRoutePtr;

    currentStopNumber = 0;
    arrivedAtRideOnStop = false;
    tryRiding = false;

    viaPointVertexId = INVALID_VERTEX_ID;
}

}//namespace ScenSim
