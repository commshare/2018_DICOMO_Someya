// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef MULTIAGENT_BEHAVIOR
#define MULTIAGENT_BEHAVIOR

#include "multiagent_agentsim.h"
#include "multiagent_publicvehicle.h"
#include "multiagent_gis.h"

namespace MultiAgent {

class AgentBehavior {
public:
    AgentBehavior(
        const shared_ptr<MultiAgentGis>& initAgentGisPtr,
        const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
        const shared_ptr<AgentRoute>& initRoutePtr,
        const AgentResource& initResource)
        :
        theAgentGisPtr(initAgentGisPtr),
        thePublicVehicleTablePtr(initPublicVehicleTablePtr),
        routePtr(initRoutePtr),
        resource(initResource)
    {}

    virtual ~AgentBehavior() {}

    virtual bool HasFinished() const = 0;
    virtual void IncrementTimeStep(const TimeType& timeStep) = 0;
    virtual AgentBehaviorType GetBehaviorType() const = 0;
    virtual string GetBehaviorName() const = 0;

    string MakePositionTraceString() const;

    const AgentRoute& GetRoute() const { return *routePtr; }

    virtual void TryInternalRouteChange(
        const VertexIdType& startVertexId,
        const VertexIdType& endVertexId,
        bool& foundRoute) { assert(false && "Not allowed internal route change"); foundRoute = false; }
    virtual void EndBehaviorAtViaPoint(const shared_ptr<AgentRoute>& nextRoutePtr) {}
    virtual VertexIdType GetViaPointVertexId() const { return resource.LastVertexId(); }

    virtual bool HasFixedDestinationVertex() const { return false; }
    virtual VertexIdType GetFixedDestinationVertexId() const { return INVALID_VERTEX_ID;/*no fixed vertex*/ }

    virtual bool IsAcceptableRouteChange(const AgentRoute& route) const { return false; }
    virtual void ChangeRoute(const shared_ptr<AgentRoute>& newRoutePtr) {}

    virtual void AssignTaxi(const shared_ptr<Taxi>& initTaxiPtr) {}

    virtual bool IsInternalRouteChaneMode() const { return false; }

    virtual bool IsStaticBehavior() const { return false; }

    virtual void StartMovingToNewQueuePosition(const Vertex& queuePosition) { assert(false); abort(); }
    virtual void Dequeue() { assert(false); abort(); }

protected:

    shared_ptr<MultiAgentGis> theAgentGisPtr;
    shared_ptr<PublicVehicleTable> thePublicVehicleTablePtr;
    shared_ptr<AgentRoute> routePtr;

    AgentResource resource;
};//AgentBehavior//



inline
string AgentBehavior::MakePositionTraceString() const
{
    ostringstream outStream;

    const Vertex& position = resource.DebugNextPosition();

    outStream << std::setw(10) << (*this).GetBehaviorName()
              << " (" << std::setw(10) << position.x
              << "," <<  std::setw(10) << position.y
              << "," <<  std::setw(10) << position.z << ")";

    return outStream.str();
}


//-----------------------------------------------------------------------------------------


class NoBehavior : public AgentBehavior {
public:
    NoBehavior(const AgentResource& initResource)
        :
        AgentBehavior(
            shared_ptr<MultiAgentGis>(),
            shared_ptr<PublicVehicleTable>(),
            shared_ptr<AgentRoute>(),
            initResource)
    {}

    virtual bool HasFinished() const { return true; }
    virtual void IncrementTimeStep(const TimeType& timeStep) {}
    virtual AgentBehaviorType GetBehaviorType() const { return AGENT_BEHAVIOR_NOTHING; }
    virtual string GetBehaviorName() const { return "NoBehavior"; }
};



class FreeWalkBehavior : public AgentBehavior {
public:
    FreeWalkBehavior(
        const shared_ptr<MultiAgentGis>& initAgentGisPtr,
        const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
        const shared_ptr<AgentRoute>& initRoutePtr,
        const GisPositionIdType& initEndPositionId,
        const bool initEnterToPosition,
        const AgentResource& initResource);

    virtual bool HasFinished() const { return (remainingPathDistance <= 0); }
    virtual void IncrementTimeStep(const TimeType& timeStep);
    virtual AgentBehaviorType GetBehaviorType() const { return AGENT_BEHAVIOR_FREEWALK; }
    virtual string GetBehaviorName() const { return "FreeWalk"; }

    virtual void EndBehaviorAtViaPoint(const shared_ptr<AgentRoute>& nextRoutePtr);

private:
    Vertex nextPos;
    Vertex directionVector;
    double remainingPathDistance;
    bool enterToPosition;

    //AgentRoute nextRoute;
    GisPositionIdType endPositionId;
    GisPositionIdType extraDestPoinId;
};



class PedestrianBehavior : public AgentBehavior {
public:
    PedestrianBehavior(
        const shared_ptr<MultiAgentGis>& initAgentGisPtr,
        const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
        const shared_ptr<AgentRoute>& initRoutePtr,
        const AgentResource& initResource);

    PedestrianBehavior(
        const shared_ptr<MultiAgentGis>& initAgentGisPtr,
        const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
        const shared_ptr<AgentRoute>& initRoutePtr,
        const Vertex& initDestPos,
        const AgentResource& initResource);

    virtual bool HasFinished() const override
        { return (remainingPathDistance <= 0 && currentRouteNumber >= routePtr->roadRoutes.size()); }
    virtual void IncrementTimeStep(const TimeType& timeStep) override;
    virtual AgentBehaviorType GetBehaviorType() const override { return AGENT_BEHAVIOR_PEDESTRIAN; }
    virtual string GetBehaviorName() const override { return "Pedestrian"; }

    virtual void EndBehaviorAtViaPoint(const shared_ptr<AgentRoute>& nextRoutePtr) override;
    virtual VertexIdType GetViaPointVertexId() const override;
    virtual bool IsAcceptableRouteChange(const AgentRoute& route) const override;
    virtual void ChangeRoute(const shared_ptr<AgentRoute>& newRoutePtr) override;

    virtual void StartMovingToNewQueuePosition(const Vertex& queuePosition) override;
    virtual void Dequeue() override;

protected:
    virtual double GetAgentSpeed() const { return resource.WalkSpeedMetersPerSec(); }

private:
    void IncrementTimeStep(const TimeType& timeStep, TimeType& remainingTime);

    void UpdateNextWalkPath();
    void ResetDestinationPosition();
    void ForceStopWalk();

    enum StateType {
        STATE_WALK_ROADSIDE,
        STATE_CROSSING_ROAD,
        STATE_QUEUED,
    };

    StateType state;

    Vertex nextPos;
    Vertex directionVector;
    Vertex destPos;

    Vertex queueingSavedCurrentPosition;
    Vertex queueingSavedNextPos;
    Vertex queuePosition;
    EntranceIdType queueEntranceId;
    unsigned int queuePositionIndex;

    RoadIdType roadId;
    VertexIdType nextVertexId;

    double remainingPathDistance;

    bool crossingDirectionIsClockwise;
    list<pair<RoadIdType, bool> > crossingOrRoadsideWalks;
    deque<Vertex> currentWaypoints;

    bool isCurrentLeftSide;
    bool isCrossingRoadAVehicleRoad;

    size_t currentVertexNumber;
    size_t currentRouteNumber;

    TimeType currentWalkStartTime;

    double pedestrianRoadWalkOffsetRatio;
    bool endBehaviorAtViaPoint;
};



class BicycleBehavior : public PedestrianBehavior {
public:
    BicycleBehavior(
        const shared_ptr<MultiAgentGis>& initAgentGisPtr,
        const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
        const shared_ptr<AgentRoute>& initRoutePtr,
        const AgentResource& initResource);

    BicycleBehavior(
        const shared_ptr<MultiAgentGis>& initAgentGisPtr,
        const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
        const shared_ptr<AgentRoute>& initRoutePtr,
        const Vertex& initDestPos,
        const AgentResource& initResource);

    virtual AgentBehaviorType GetBehaviorType() const { return AGENT_BEHAVIOR_BICYCLE; }
    virtual string GetBehaviorName() const { return "Bicycle"; }

protected:
    virtual double GetAgentSpeed() const { return resource.BicycleSpeedMetersPerSec(); }
};



//---------------------------------------------------------------------

class VehicleDriverBehavior : public AgentBehavior {
public:
    VehicleDriverBehavior(
        const shared_ptr<MultiAgentGis>& initAgentGisPtr,
        const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
        const shared_ptr<AgentRoute>& initRoutePtr,
        const AgentResource& initResource,
        const shared_ptr<Vehicle>& initVehiclePtr,
        const TimeType& minShortStopTime);

    ~VehicleDriverBehavior();

    virtual bool HasFinished() const { return ((remainingPathDistance <= 0) && !isOnVehicle); }
    virtual void IncrementTimeStep(const TimeType& timeStep);
    virtual AgentBehaviorType GetBehaviorType() const { return AGENT_BEHAVIOR_VEHICLE; }
    virtual string GetBehaviorName() const { return "Vehicle"; }

    AgentIdType GetAgentId() const { return resource.AgentId(); }
    AgentResource Resouce()  const { return resource; }

    bool YieldRoadTo(
        const Vehicle& otherVehicle,
        const double yieldDistanceTimeSec) const;

    const Vehicle& GetVehicle() const { return *vehiclePtr; }

    virtual VertexIdType GetViaPointVertexId() const;
    virtual void EndBehaviorAtViaPoint(const shared_ptr<AgentRoute>& nextRoutePtr);
    virtual bool IsAcceptableRouteChange(const AgentRoute& route) const;
    virtual void ChangeRoute(const shared_ptr<AgentRoute>& newRoutePtr);

    virtual void TryInternalRouteChange(
        const VertexIdType& startVertexId,
        const VertexIdType& endVertexId,
        bool& foundRoute);

    virtual bool IsInternalRouteChaneMode() const { return true; }

protected:
    void RunVehicle(const TimeType& runDuration);

    void RideOnVehicle();
    bool HasRemainingPath() const { return ((remainingPathDistance > 0) && !routes.empty()); }

    void UpdateVelocity(const TimeType& timeStep);

    void GoToVehicle();
    void UpdateVisibleRouteIfNecessary();
    void SetNextWaypoint();

    void ViewFrontStatus(
        double& distanceToStop,
        double& frontVehicleVelocity,
        double& minDistanceToFrontVehicle);

    void ChangeLaneIfNecessary();
    bool CanChangeLane(const size_t nextLaneNumber) const;
    void ChangeLane(const size_t nextLaneNumber);
    void UpdateNextRoadWaypointIfNecessary();

    bool HasEnoughGapAndSafetyCriterionToChangeLane(const size_t laneNubmer) const;
    bool IsChangingLane() const { return resource.CurrentTime() < laneChangeEnableTime; }
    double GetRemainingLanechangeRate() const;

    void ParkingVehicleIfPossible(
        const VertexIdType& vertexId,
        const TimeType& stopTime);

    virtual bool IsStopPoint(const VertexIdType& vertexId) const { return false; }
    virtual void ArrivedAtVertex(const VertexIdType& vertexId);
    virtual void RestartRun();

    void ForceStopVehicle();
    void AssignFirstRoute();

    // Velocity Unit: Meters/Second

    struct VehicleRoute {
        RoadIdType roadId;
        VertexIdType outgoingVertexId;
        deque<Vertex> remainingWaypoints;
        RoadTurnType outgoingTurnType;
        TimeType entranceEnabledTime;

        VehicleRoute()
            :
            roadId(INVALID_VARIANT_ID),
            entranceEnabledTime(ZERO_TIME)
        {}
    };

    bool isOnVehicle;
    bool passingIntersection;
    shared_ptr<PedestrianBehavior> pedestrianPtr;
    shared_ptr<Vehicle> vehiclePtr;

    deque<VehicleRoute> routes;
    double remainingDistanceToIntersection;
    TimeType remainingShortStopTime;
    const TimeType minShortStopTime;

    TimeType laneChangeEnableTime;
    double laneChangeDirection;

    Vertex directionVector;
    double velocity;
    size_t currentLaneNumber;

    double remainingPathDistance;
    double currentWaypointRadians;

    size_t viewdRouteNumber;

    static const double enoughMinDistance;

    static const double laneChangeTimeSec;
    static const double velocityMargin;
    static const double laneChangeProhibitedZoneLength;
    static const double forceLaneChangeDistance;
    static const double slowDownDistanceBeforeCurve;

    static double CalculateFreeRunAcceleration(
        const double originalVelocity,
        const double maxVelocity);

    static double CalculateIntelligentDriversModelAcceleration(
        const Vehicle& vehicle,
        const double distanceToFrontPoint,
        const double frontObstructionVelocityMeterPerSec,
        const double maxVelocityMeterPerSec);

    static double CalculateIntelligentDriversModelAcceleration(
        const Vehicle& vehicle,
        const Vehicle& frontVehicle);

    static double CalculateIntelligentDriversModelAcceleration(
        const Vehicle& vehicle);

    static double CalculateGeneralizedForceModelAcceleration(
        const Vehicle& vehicle,
        const double distanceToFrontObstruction,
        const double frontObstructionVelocity,
        const double minDistanceToFrontPoint,
        const double maxVelocity);

    static double CalculateDesiredVelocity(
        const AgentRoad& passingRoad,
        const Vehicle& vehicle,
        const double distanceToVertex,
        const double radiansToNextPoint);

    double CalculateIdmBasedAcceleration(
        const double timeStepSec,
        const double distanceToStop,
        const double frontVehicleVelocity) const;

    double CalculateGfmBasedAcceleration(
        const double timeStepSec,
        const double distanceToStop,
        const double frontVehicleVelocity,
        const double minDistanceToFrontVehicle) const;
};



class BusDriverBehavior : public VehicleDriverBehavior {
public:
    BusDriverBehavior(
        const shared_ptr<MultiAgentGis>& initAgentGisPtr,
        const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
        const AgentResource& initResource,
        const shared_ptr<Vehicle>& initVehiclePtr,
        const shared_ptr<Bus>& initBusPtr,
        const TimeType& initMinShortStopTime);

    ~BusDriverBehavior();

    const Bus& GetBus() const { return *busPtr; }

    virtual bool HasFinished() const;
    virtual void IncrementTimeStep(const TimeType& timeStep);
    virtual string GetBehaviorName() const { return "BusDriver"; }
    virtual AgentBehaviorType GetBehaviorType() const { return AGENT_BEHAVIOR_BUS_DRIVER; }

    virtual void TryInternalRouteChange(
        const VertexIdType& startVertexId,
        const VertexIdType& endVertexId,
        bool& foundRoute);

    virtual bool IsInternalRouteChaneMode() const { return true; }

    virtual bool HasFixedDestinationVertex() const { return true; }
    virtual VertexIdType GetFixedDestinationVertexId() const;

    virtual bool IsStaticBehavior() const { return true; }

protected:
    virtual bool IsStopPoint(const VertexIdType& vertexId) const;
    virtual void ArrivedAtVertex(const VertexIdType& vertexId);

private:
    shared_ptr<Bus> busPtr;
};



class TrainDriverBehavior : public AgentBehavior {
public:
    TrainDriverBehavior(
        const shared_ptr<MultiAgentGis>& initAgentGisPtr,
        const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
        const AgentResource& initResource,
        const shared_ptr<Train>& initTrainPtr);

    ~TrainDriverBehavior();

    const Train& GetTrain() const { return *trainPtr; }

    virtual bool HasFinished() const;
    virtual void IncrementTimeStep(const TimeType& timeStep);
    virtual string GetBehaviorName() const { return "TrainDriver"; }
    virtual AgentBehaviorType GetBehaviorType() const { return AGENT_BEHAVIOR_TRAIN_DRIVER; }

private:
    shared_ptr<Train> trainPtr;
};



class TaxiDriverBehavior : public VehicleDriverBehavior {
public:
    TaxiDriverBehavior(
        const shared_ptr<MultiAgentGis>& initAgentGisPtr,
        const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
        const AgentResource& initResource,
        const shared_ptr<Taxi>& initTaxiPtr,
        const TimeType& initMinShortStopTime);

    ~TaxiDriverBehavior();

    const Taxi& GetTaxi() const { return *taxiPtr; }

    virtual bool HasFinished() const { return false; }
    virtual void IncrementTimeStep(const TimeType& timeStep);
    virtual string GetBehaviorName() const { return "TaxiDriver"; }
    virtual AgentBehaviorType GetBehaviorType() const { return AGENT_BEHAVIOR_TAXI_DRIVER; }

    virtual bool IsInternalRouteChaneMode() const { return true; }

    virtual bool HasFixedDestinationVertex() const;

    virtual VertexIdType GetFixedDestinationVertexId() const;

    virtual void TryInternalRouteChange(
        const VertexIdType& startVertexId,
        const VertexIdType& endVertexId,
        bool& foundRoute);

    virtual bool IsStaticBehavior() const { return true; }

protected:

    virtual bool IsStopPoint(const VertexIdType& vertexId) const;
    virtual void ArrivedAtVertex(const VertexIdType& vertexId);
    virtual void RestartRun();

    void ParkingVehicleForNextCustomer();
    bool HasNoCustomerAndReservationAtHome() const;
    void LookRoadsideCustomer();
    void DecideRoute();

    enum DriverStateType {
        DRIVER_STATE_TRANSPORT,
        DRIVER_STATE_GO_TO_CUSTOMER,
        DRIVER_STATE_GO_BACK_HOME,
        DRIVER_STATE_AT_HOME,
    };

    DriverStateType state;
    VertexIdType nextStopVertexId;

    shared_ptr<Taxi> taxiPtr;
};

class TaxiGuestBehavior : public AgentBehavior {
public:
    TaxiGuestBehavior(
        const shared_ptr<MultiAgentGis>& initAgentGisPtr,
        const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
        const shared_ptr<AgentRoute>& initRoutePtr,
        const AgentResource& initResource);

    ~TaxiGuestBehavior();

    virtual bool HasFinished() const { return tookOffTaxi; }
    virtual void IncrementTimeStep(const TimeType& timeStep);
    virtual AgentBehaviorType GetBehaviorType() const { return AGENT_BEHAVIOR_TAXI; }
    virtual string GetBehaviorName() const { return "TaxiGuest"; }

    virtual void EndBehaviorAtViaPoint(const shared_ptr<AgentRoute>& nextRoutePtr);
    virtual VertexIdType GetViaPointVertexId() const;
    virtual bool IsAcceptableRouteChange(const AgentRoute& route) const;
    virtual void ChangeRoute(const shared_ptr<AgentRoute>& newRoutePtr);

    virtual void AssignTaxi(const shared_ptr<Taxi>& initTaxiPtr);

    virtual bool IsInternalRouteChaneMode() const;

    virtual void TryInternalRouteChange(
        const VertexIdType& startVertexId,
        const VertexIdType& endVertexId,
        bool& foundRoute);

private:
    void TakeoffTaxi();
    void GiveupTaxiWaiting() { (*this).TakeoffTaxi(); }

    shared_ptr<Taxi> taxiPtr;
    bool tookOffTaxi;

    bool succeededTaxiCall;
    bool routeRecalculationHasExecuted;

    TimeType arrivalTime;
    bool isOnTaxi;

    VertexIdType viaPointVertexId;

    AgentTaxiCenterIdType taxiCenterId;
};

//---------------------------------------------------------------------

class PublicVehicleBehavior : public AgentBehavior {
public:
    PublicVehicleBehavior(
        const shared_ptr<MultiAgentGis>& initAgentGisPtr,
        const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
        const shared_ptr<AgentRoute>& initRoutePtr,
        const AgentResource& initResource);

    ~PublicVehicleBehavior();

    virtual bool HasFinished() const;
    virtual void IncrementTimeStep(const TimeType& timeStep);
    virtual AgentBehaviorType GetBehaviorType() const { return AGENT_BEHAVIOR_BUS; }

    virtual string GetBehaviorName() const {
        if (vehiclePtr == nullptr) {
            return "PublicVehicleGuest";
        }
        return vehiclePtr->GetVehicleName() + "Guest";
    }

    virtual void EndBehaviorAtViaPoint(const shared_ptr<AgentRoute>& nextRoutePtr);
    virtual VertexIdType GetViaPointVertexId() const;
    virtual bool IsAcceptableRouteChange(const AgentRoute& route) const;
    virtual void ChangeRoute(const shared_ptr<AgentRoute>& newRoutePtr);

private:
    shared_ptr<PublicVehicle> vehiclePtr;
    shared_ptr<PedestrianBehavior> pedestrianPtr;


    double destPositionInVehicleMeters;
    double currentPositionInVehicleMeters;
    double relativePositionRadians;

    size_t currentStopNumber;
    bool arrivedAtRideOnStop;
    bool tryRiding;
    TimeType stopArrivalTime;
    TimeType vehicleScheduleArrivalTime;

    VertexIdType viaPointVertexId;

    Vertex anchorPosition;
    double anchorDirectionRadians;
    set<PublicVehicleIdType> getDownedVehicleIds;

    bool IsOnPublicVehicle() const;
    void RideOnVehicleIfPossible();
    void GetDownVehicle();
    void IncrementTimeStepInVehicle(const TimeType& timeStep);
    bool IsGetDownStop() const;

    void DecidePositionInVehicle();
    void SetPedestrianBehaviorTowardNextStop(
        const StopIdType& stopId,
        const VertexIdType& startVertexId,
        const VertexIdType& destVertexId);

    bool MissedPublicVehicle() const;
    void ForceStopRiding();
    void UpdateAnchorPosition();
};


}//namespace ScenSim

#endif
