// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef MULTIAGENT_PUBLICVEHICLE_H
#define MULTIAGENT_PUBLICVEHICLE_H

#include "multiagent_agentsim.h"
#include "multiagent_gis.h"

namespace MultiAgent {

using ScenSim::NoOverhead_weak_ptr;
using std::deque;

class PublicVehicleTable;

typedef size_t VehicleFamilyIdType;

struct VehicleFamily {
    string name;
    double widthMeters;
    double lengthMeters;
    size_t capacity;
    bool barrierFree;

    VehicleFamily()
        :
        widthMeters(0),
        lengthMeters(0),
        capacity(0),
        barrierFree(false)
    {}
};

//------------------------------------------------------------------------------
// Train
//------------------------------------------------------------------------------

struct WaypointType {
    TimeType time;
    Vertex position;

    WaypointType(
        const TimeType& initTime,
        const Vertex& initPosition)
        :
        time(initTime),
        position(initPosition)
    {}
};

class WaypointsType : public vector<WaypointType> {
public:
    virtual ~WaypointsType() {}

    void AssignConstantWaypoints(
        const TimeType& startTime,
        const TimeType& endTime,
        const vector<Vertex>& points);

    void operator+=(const WaypointsType& right);
};

struct VehicleStopInfo {

    StopIdType stopId;
    TimeType arrivalTime;
    TimeType departureTime;

    VehicleStopInfo()
        :
        stopId(INVALID_VARIANT_ID),
        arrivalTime(ZERO_TIME),
        departureTime(ZERO_TIME)
    {}

    VehicleStopInfo(
        const StopIdType& initStopId,
        const TimeType& initArrivalTime,
        const TimeType& initDepartureTime)
        :
        stopId(initStopId),
        arrivalTime(initArrivalTime),
        departureTime(initDepartureTime)
    {}

    bool IsPassStop() const { return (arrivalTime == departureTime); }
};

class Vehicle {
public:
    static const size_t NO_CHANGED_LANE_NUMBER;

    Vehicle(
        const AgentIdType& initDriverAgentId,
        const VertexIdType& initVertexId,
        const Vertex& initPosition,
        const RoadIdType& initRoadId,
        const size_t initLaneNumber,
        const MultiAgentSimulator* initSimulatorPtr);

    void AdvanceStateToNextTimestep();

    const Vertex& GetPosition() const { return position; }
    const Vertex& GetNextPosition() const { return nextPosition; }

    void SetNextPosition(const Vertex& newPosition) { nextPosition = newPosition; }

    double GetDirectionRadians() const { return directionRadians; }
    double GetNextDirectionRadians() const { return nextDirectionRadians; }

    void SetNextDirectionRadians(const double& newDirectionRadians)
        { nextDirectionRadians = newDirectionRadians; }

    double GetVelocityMetersPerSec() const { return velocity; }
    void SetNextVelocity(const double& newVelocity) { nextVelocity = newVelocity; }

    double GetDistanceToIntersection() const { return distanceToIntersection; }
    void SetNextDistanceToIntersection(const double& newDistanceToIntersection)
        { nextDistanceToIntersection = newDistanceToIntersection; }

    const RoadIdType& GetRoadId() const { return roadId; }
    void SetNextRoadId(const RoadIdType& newRoadId) { nextRoadId = newRoadId; }

    GisPositionIdType GetPositionId() const {
        return (GisPositionIdType(GIS_ROAD, roadId));
    }

    // const RoadIdType& GetPrevRoadId() const { return roadId.GetPrevValue(); }
    const RoadTurnType& GetTurnType() const { return turnType; }
    void SetNextTurnType(const RoadTurnType& newTurnType) { nextTurnType = newTurnType; }

    size_t GetLaneNumber() const { return laneNumber; }
    void SetNextLaneNumber(const size_t newLaneNumber) { nextLaneNumber = newLaneNumber; }

    bool IsChangingLane() const { return (changedLaneNumber != NO_CHANGED_LANE_NUMBER); }
    size_t GetChangedLaneNumber() const { return changedLaneNumber; }
    void SetNextChangedLaneNumber(const size_t newLaneNumber) { nextChangedLaneNumber = newLaneNumber; }


    bool IsOnRoad() const { return (roadId != INVALID_VARIANT_ID); }
    double GetHalfBodyLength() const { return vehicleConstant.vehicleHalfLength; }

    AgentIdType GetDriverAgentId() const { return driverAgentId; }

    VertexIdType GetVertexId() const { return vertexId; }
    VertexIdType GetNextVertexId() const { return nextVertexId; }
    void SetNextVertexId(const VertexIdType& newVertexId) { nextVertexId = newVertexId; }

    const VehicleConstant& GetVehicleConstant() const { return vehicleConstant; }

    void SetVehicleConstant(const VehicleConstant& initVehicleConstant);

    bool IsOnLane(const size_t laneNumber) const;

    VertexIdType GetViaPointVertexId() const { return viapointVertexId; }

    double GetRunningDistanceGasolinCost() const { return runningDistance*vehicleConstant.gasolinCost; }


protected:
    friend class VehicleDriverBehavior;
    friend class BusDriverBehavior;
    friend class TaxiDriverBehavior;

    const AgentIdType driverAgentId;
    VehicleConstant vehicleConstant;

    // "next" variables are for next timestep.

    VertexIdType vertexId;
    VertexIdType nextVertexId;

    Vertex position;
    Vertex nextPosition;

    size_t laneNumber;
    size_t nextLaneNumber;

    size_t changedLaneNumber;
    size_t nextChangedLaneNumber;

    double velocity;
    double nextVelocity;

    double distanceToIntersection;
    double nextDistanceToIntersection;

    RoadIdType roadId;
    RoadIdType nextRoadId;

    RoadTurnType turnType;
    RoadTurnType nextTurnType;

    double directionRadians;
    double nextDirectionRadians;

    VertexIdType viapointVertexId;
    double runningDistance;

    //-----------------------------------------------
    // Vehicle topology optimization key.

    friend class MultiAgentGis;
    friend class AgentRoad;

    VehiclePositionKey positionKey;

    void UpdateVehicleConstant(const VehicleConstant& initVehicleConstant) { vehicleConstant = initVehicleConstant; }
};

inline
void Vehicle::AdvanceStateToNextTimestep()
{
    (*this).vertexId = nextVertexId;
    (*this).position = nextPosition;
    (*this).laneNumber = nextLaneNumber;
    (*this).changedLaneNumber = nextChangedLaneNumber;
    (*this).velocity = nextVelocity;
    (*this).distanceToIntersection = nextDistanceToIntersection;
    (*this).roadId = nextRoadId;
    (*this).turnType = nextTurnType;
    (*this).directionRadians = nextDirectionRadians;

}//AdvanceStateToNextTimestep//


class Taxi : public Vehicle {
public:
    Taxi(
        const AgentIdType& initDriverAgentId,
        const AgentTaxiCenterIdType& initHomeTaxiCenterId,
        const VertexIdType& initVertexId,
        const Vertex& initPosition,
        const RoadIdType& initRoadId,
        const size_t initLaneNumber,
        const MultiAgentSimulator* initSimulatorPtr);

    void PopReservedCustomer() { reservedCustomers.pop_front(); }
    void PopCustomer() { customers.pop_front(); }
    void AddReservedCustomer(AgentResource& customerResource);

    bool ContainsCustomer() const { return !customers.empty(); }
    bool HasReservation() const { return !reservedCustomers.empty(); }

    bool IsTookOnCustomer(const AgentResource& customerResource) const;

    void TakeOn(
        const AgentIdType& agentId,
        const shared_ptr<AgentRoute>& requiredRoutePtr);

    bool Contains(const AgentResource& customerResource) const;

    bool AtHomeVertex() const { return (vertexId == homeVertexId); }
    const VertexIdType& GetHomeVertexId() const { return homeVertexId; }
    const AgentTaxiCenterIdType& GetHomeTaxiCenterId() const { return homeTaxiCenterId; }

    GisPositionIdType GetCustomerPosition() const;
    AgentResource GetReservedCustomer() const;

    bool CanTakeOn(const AgentIdType& agentId) const;

    void SetCustomerRequestRoute(const shared_ptr<AgentRoute>& initRequestRoutePtr);
    bool HasCustomerRequestRoute() const;
    shared_ptr<AgentRoute> TakeCustomerRequestRoute();

    void RemoveCustomerOrReservation(const AgentResource& customerResource);

    void TakeOnReservedCustomer(
        const AgentResource& customerResource,
        const shared_ptr<AgentRoute>& initRequestRoutePtr);

    void SetEnableCustomerPickupState() { pickupState = true; }
    bool CanPickUp(const AgentResource& customerResource) const;

    void ArrivedAtTheDestinationForCurrentCustomer() { arrivedAthTheDestination = true; }

    bool IsArrivedAtTheDestination() const { return arrivedAthTheDestination; }

private:
    friend class MultiAgentGis;

    VertexIdType homeVertexId;
    AgentTaxiCenterIdType homeTaxiCenterId;

    struct CustomerData {
        AgentResource resource;
        shared_ptr<AgentRoute> requestRoutePtr;

        CustomerData(
            const AgentResource& initResource,
            const shared_ptr<AgentRoute>& initRequestRoutePtr)
            :
            resource(initResource),
            requestRoutePtr(initRequestRoutePtr)
        {}
    };

    deque<CustomerData> customers;

    bool pickupState;
    deque<AgentResource> reservedCustomers;
    bool arrivedAthTheDestination;

    void RemoveCustomer(const AgentResource& customerResource);
    void RemoveReservation(const AgentResource& customerResource);
};

//-------------------------------------------------------------------------------

class PublicVehicle {
public:
    PublicVehicle(
        PublicVehicleTable* initPublicVehicleTablePtr,
        const shared_ptr<const deque<VehicleStopInfo> >& initStopsPtr,
        const PublicVehicleIdType& initPublicVehicleId,
        const LineIdType& initLineId,
        const RouteIdType& initRouteId,
        const VehicleNumberType& initVehicleNumber,
        const VehicleFamily& initVehicleFamily,
        const LineType& initLineType)
        :
        publicVehicleTablePtr(initPublicVehicleTablePtr),
        stopsPtr(initStopsPtr),
        publicVehicleId(initPublicVehicleId),
        lineId(initLineId),
        routeId(initRouteId),
        vehicleNumber(initVehicleNumber),
        vehicleFamily(initVehicleFamily),
        lineType(initLineType),
        numberRidingPeople(0),
        lastStopNumber(0),
        threadNumber(MASTER_THREAD_NUMBER)
    {}

    virtual ~PublicVehicle() {}

    const PublicVehicleIdType& GetPublicVehicleId() const { return  publicVehicleId; }
    const LineIdType& GetLineId() const { return lineId; }
    const RouteIdType& GetRouteId() const { return routeId; }
    const VehicleNumberType& GetVehicleNumber() const { return vehicleNumber; }
    const VehicleFamily& GetVehicleFamily() const { return vehicleFamily; }

    virtual const Vertex& GetPosition() const = 0;
    virtual const Vertex& GetNextPosition() const = 0;
    virtual double GetDirectionRadians() const = 0;
    virtual double GetNextDirectionRadians() const = 0;
    virtual VehicleType GetVehicleType() const = 0;

    virtual TimeType GetDepartureTime(const TimeType& currentTime) const = 0;
    virtual bool AtStop(const TimeType& currentTime) const = 0;
    virtual const StopIdType& GetStopId() const = 0;
    VertexIdType GetViaPointVertexId(const TimeType& currentTime) const;
    GisPositionIdType GetViaPointPositionId(const TimeType& currentTime) const;

    virtual string GetVehicleName() const = 0;

    bool Contains(const AgentResource& resource) const { return ridingAgents.find(resource) != ridingAgents.end(); }

    virtual bool CanTakeOn(const AgentResource& resouce) const { return !(*this).IsFull(); }

    bool IsFull() const { return (numberRidingPeople == vehicleFamily.capacity); }
    virtual void Push(
        const AgentResource& resource,
        const TimeType& currentTime) {

        assert(ridingAgents.find(resource) == ridingAgents.end());
        ridingAgents.insert(resource);
        numberRidingPeople += resource.NumberPeople();
    }

    virtual void Pop(
        const AgentResource& resource) {

        assert(ridingAgents.find(resource) != ridingAgents.end());
        ridingAgents.erase(resource);
        numberRidingPeople -= resource.NumberPeople();
    }

    bool HasAnAgent() const { return !ridingAgents.empty(); }

    void ResetRejectedAgentIds() { rejectedAgentIds.clear(); }
    void AddRejectedAgent(const AgentIdType& agentId) { rejectedAgentIds.insert(agentId); }
    bool IsAlreadyRejectedAgent(const AgentIdType& agentId) const { return (rejectedAgentIds.find(agentId) != rejectedAgentIds.end()); }

    virtual TimeType GetCurrentDelay(const TimeType& currentTime) const { return ZERO_TIME; }

    virtual AgentIdType GetDriverAgentId() const = 0;

    // for debugging

    bool IsNeverVisitableBusStop(const BusStopIdType& busStopId) const;
    bool IsNeverVisitableStation(const RailRoadStationIdType& stationId) const;

protected:
    PublicVehicleTable* publicVehicleTablePtr;
    shared_ptr<const deque<VehicleStopInfo> > stopsPtr;

    const PublicVehicleIdType publicVehicleId;
    const LineIdType lineId;
    const RouteIdType routeId;
    const VehicleNumberType vehicleNumber;
    const VehicleFamily vehicleFamily;
    const LineType lineType;

    set<AgentResource> ridingAgents;
    set<AgentIdType> rejectedAgentIds;
    size_t numberRidingPeople;

    StopNumberType lastStopNumber;

    size_t threadNumber;
};

struct StopIdPair {
    StopIdType stopId1;
    StopIdType stopId2;

    StopIdPair(
        const StopIdType& initStopId1,
        const StopIdType& initStopId2)
        :
        stopId1(std::min(initStopId1, initStopId2)),
        stopId2(std::max(initStopId1, initStopId2))
    {}

    bool operator<(const StopIdPair& right) const {
        return (stopId1 < stopId1 ||
                (stopId1 == right.stopId1 &&
                 stopId2 < right.stopId2));
    }
};

class Train : public PublicVehicle {
public:
    Train(
        PublicVehicleTable* initPublicVehicleTablePtr,
        const PublicVehicleIdType& initPublicVehicleId,
        const LineIdType& initLineId,
        const RoadIdType& initRoadId,
        const VehicleNumberType& initVehicleNumber,
        const VehicleFamily& initVehicleFamily,
        const LineType& initLineType,
        const shared_ptr<const deque<VehicleStopInfo> >& initStopsPtr,
        const shared_ptr<const vector<vector<Vertex> > >& initWaypointsPtr);

    PublicVehicleIdType GetPublicVehicleId() const { return publicVehicleId; }

    void SetTimeTo(const TimeType& time);

    virtual const Vertex& GetPosition() const { return currentPosition; }
    virtual const Vertex& GetNextPosition() const { return currentPosition; }
    virtual double GetDirectionRadians() const { return currentDirectionRadians; }
    virtual double GetNextDirectionRadians() const { return currentDirectionRadians; }

    virtual VehicleType GetVehicleType() const { return VEHICLE_TRAIN; }

    virtual TimeType GetDepartureTime(const TimeType& currentTime) const;

    virtual bool AtStop(const TimeType& currentTime) const;
    virtual const StopIdType& GetStopId() const;

    virtual string GetVehicleName() const { return "Train"; }

    RailRoadStationIdType GetStationId() const;

    bool GoToGarage() const;

    const RailRoadLineIdType& GetRailRoadLineId() const;

    TimeType GetGarbageTime() const { return stopsPtr->back().departureTime; }

    void SetDriverAgentId(const AgentIdType& agentId) { driverAgentId = agentId; }
    virtual AgentIdType GetDriverAgentId() const { return driverAgentId; }

private:
    const shared_ptr<const vector<vector<Vertex> > > waypointsPtr;

    AgentIdType driverAgentId;

    bool atStation;
    Vertex currentPosition;
    Vertex currentVector;
    double currentDirectionRadians;

    WaypointsType currentWaypoints;
    size_t lastWaypointNumber;
};

class Bus : public PublicVehicle {
public:
    Bus(
        PublicVehicleTable* initPublicVehicleTablePtr,
        const VehicleType& initVehicleType,
        const TimeType& initCurrentTime,
        const PublicVehicleIdType& initPublicVehicleId,
        const LineIdType& initLineId,
        const RouteIdType& initRouteId,
        const VehicleNumberType& initVehicleNumber,
        const VehicleFamily& initVehicleFamily,
        const LineType& initLineType,
        const shared_ptr<const deque<VehicleStopInfo> >& initStopsPtr,
        const shared_ptr<AgentRoute>& initRoutePtr,
        const RoadIdType& initStartRoadId,
        const set<AgentIdType>& initReservedAgentIds);

    PublicVehicleIdType GetPublicVehicleId() const { return publicVehicleId; }

    virtual const Vertex& GetPosition() const { return vehiclePtr->GetPosition(); }
    virtual const Vertex& GetNextPosition() const { return vehiclePtr->GetNextPosition(); }
    virtual double GetDirectionRadians() const { return vehiclePtr->GetDirectionRadians(); }
    virtual double GetNextDirectionRadians() const { return vehiclePtr->GetNextDirectionRadians(); }

    virtual VehicleType GetVehicleType() const {  return vehicleType; }

    virtual TimeType GetDepartureTime(const TimeType& currentTime) const;
    void ArrivedAtBusStop(const TimeType& currentTime);

    bool ArrivedAtTheLastBusStop(const TimeType& currentTime) const;
    virtual bool AtStop(const TimeType& currentTime) const;
    virtual const StopIdType& GetStopId() const;

    virtual string GetVehicleName() const { return "Bus"; }

    virtual bool CanTakeOn(const AgentResource& resource) const;

    virtual void Push(
        const AgentResource& resource,
        const TimeType& currentTime);

    virtual void Pop(const AgentResource& resource);

    BusStopIdType GetStartBusStopId() const;
    BusStopIdType GetBusStopId(const size_t i) const;
    BusStopIdType GetDestBusStopId() const;

    const RoadIdType& GetStartRoadId() const { return startRoadId; }

    const StopIdType& GetStartStopId() const { return stopsPtr->front().stopId; }
    const StopIdType& GetDestStopId() const { return stopsPtr->back().stopId; }

    BusStopIdType GetNextBusStopId() const;
    VertexIdType GetNextBusStopVertexId() const;

    const AgentRoute& GetRoute() const { return *(*currentRouteIter); }

    bool ContainsDestinationStopId(const StopIdType& stopId) const;

    void SetVehicle(const shared_ptr<Vehicle>& initVehiclePtr) {
        vehiclePtr = initVehiclePtr;
    }

    virtual TimeType GetCurrentDelay(const TimeType& currentTime) const;

    StopNumberType GetStopNumber() const { return lastStopNumber; }
    StopNumberType GetNextStopNumber() const { return lastStopNumber + 1; }
    size_t GetNumberOfStops() const { return stopsPtr->size(); }

    virtual AgentIdType GetDriverAgentId() const { return vehiclePtr->GetDriverAgentId(); }

private:
    friend class BusDriverBehavior;

    const VehicleType vehicleType;
    shared_ptr<AgentRoute> routePtr;
    set<AgentIdType> reservedAgentIds;

    shared_ptr<Vehicle> vehiclePtr;
    RoadIdType startRoadId;

    TimeType lastArrivedTime;

    typedef vector<shared_ptr<AgentRoute> >::const_iterator RouteIter;
    RouteIter currentRouteIter;

    set<StopIdType> destinationStopIds;
    size_t remainingNonReservedSeats;

    TimeType ridingEndTime;
};

//------------------------------------------------------------

class PublicVehicleTable {
public:
    PublicVehicleTable(
        MultiAgentSimulator* initSimulatorPtr,
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<MultiAgentGis>& agentGisPtr);

    void CompleteRouteAndVehiclewScheduleInitialization();

    void CompleteAllPublicVehicleInitialization();

    void CreatePublicVehicles(
        const TimeType& currentTime,
        vector<shared_ptr<Train> >& trainPtrs,
        vector<shared_ptr<Bus> >& busPtrs);

    void SearchPublicVehicleRoute(
        const AgentResource& resource,
        const StopIdType& startStopId,
        const StopIdType& endStopId,
        const TimeToSearchRoute& timeToSearchRoute,
        const AgentBehaviorType& preferenceBehavior,
        const set<AgentBehaviorType>& notAvailableBehavorTypes,
        const size_t numberMaxRoutes,
        const size_t numberMaxTransferTimes,
        vector<vector<shared_ptr<AgentRoute> > >& routeCandidatePtrsPerOrder);

    void FindStartAndStopIds(
        const Vertex& startVertex,
        const Vertex& destVertex,
        const double maxWalkDistance,
        const size_t numberMaxBusStopCandidatesForStopType,
        const bool searchFromNearStopPair,
        const set<AgentBehaviorType>& notAvailableBehavorTypes,
        vector<pair<StopIdType, StopIdType> >& startAndDestStopIds) const;

    Vertex GetStopVertex(const StopIdType& stopId) const;

    string GetStopName(const StopIdType& stopId) const;

    VertexIdType GetOrigVertexId(
        const PublicVehicleRoute& route) const;

    VertexIdType GetDestVertexId(
        const PublicVehicleRoute& route) const;

    VertexIdType GetOrigEntranceVertexId(
        const PublicVehicleRoute& route,
        const Vertex& position) const;

    VertexIdType GetLineVertexId(
        const StopIdType& stopId,
        const LineIdType& lineId,
        const RouteIdType& routeId) const;

    VertexIdType GetNearestEntranceVertexId(
        const StopIdType& stopId,
        const Vertex& position) const;

    const vector<VertexIdType>& GetEntranceVertexIds(const StopIdType& stopId) const;

    RailRoadStationIdType GetStationId(const StopIdType& stopId) const;
    BusStopIdType GetBusStopId(const StopIdType& stopId) const;

    bool IsStation(const StopIdType& stopId) const;
    bool IsBusStop(const StopIdType& stopId) const;

    GisPositionIdType GetPositionId(const StopIdType& stopId) const;

    double CalculateCongestion(const StopIdType& stopId) const;

    bool IsVertexOfStop(
        const StopIdType& stopId,
        const VertexIdType& vertexId)  const;

    TimeType GetStopDelay(
        const LineIdType& lineId,
        const RouteIdType& routeId,
        const StopNumberType& stopNumber) const;

    TimeType GetVehicleDelay(
        const LineIdType& lineId,
        const RouteIdType& routeId,
        const VehicleNumberType& vehicleNumber) const;

    TimeType GetScheduledArrivalTime(
        const LineIdType& lineId,
        const RouteIdType& routeId,
        const VehicleNumberType& vehicleNumber,
        const StopIdType& stopId,
        const TimeType& currentTime) const;

    void SetStopDelay(
        const LineIdType& lineId,
        const RouteIdType& routeId,
        const StopNumberType& stopNumber,
        const TimeType& delay);

    void SetVehicleDelay(
        const LineIdType& lineId,
        const RouteIdType& routeId,
        const VehicleNumberType& vehicleNumber,
        const TimeType& delay);

    bool ContainsLine(const string& lineName) const;
    LineIdType GetLineId(const string& lineName) const;
    StopIdType GetStopId(const LineIdType& lineId, const string& stopName) const;
    RouteIdType GetShortestRouteId(
        const LineIdType& lineId,
        const StopIdType& departureStopId,
        const StopIdType& arrivalStopId) const;

    bool IsLineAvailable(const LineIdType& lineId) const;

    string GetLineName(const LineIdType& lineId) const;

    const RailRoadLineIdType& GetRailRoadLineId(const LineIdType& lineId) const { return railRoadLineIdPerLine.at(lineId); }

    double CalculateGetDownPrice(
        const PublicVehicleRoute& route,
        const VertexIdType& getDownVertexId,
        const AgentResource& resource) const;

private:

    //typedef size_t StopIdType;

    //static const StopIdType INVALID_STOP_ID;

    void LoadVehicleScheduleTable(
        const string& scheduleFilePath,
        const TimeType& startTime,
        const TimeType& endTime);

    void AssignStops(
        const LineIdType& lineId,
        const RouteIdType& routeId,
        const vector<vector<IntersectionIdType> >& intersectionIdsPerRoute);

    void AddVehicleSchedule(
        const LineIdType& lineId,
        const RouteIdType& routeId,
        const VehicleNumberType& vehicleNumber,
        const deque<pair<TimeType, TimeType> >& timeLines,
        const string& parsingLine);

    void RegisterVehicleArrivalTimeAndDepartureTime(
        const LineIdType& lineId,
        const RouteIdType& routeId,
        const VehicleNumberType& vehicleNumber);

    void AssignLoopLineVehiclesIfNecessary(
        const LineIdType& lineId,
        const RouteIdType& routeId);

    void ScheduleVehicleCreationTime();

    PublicVehicleIdType GetNewPublicVehicleId();

    MultiAgentSimulator* simulatorPtr;
    NoOverhead_weak_ptr<MultiAgentGis> theAgentGisPtr;
    size_t numberThreads;

    size_t numberPublicVehicles;
    bool completedInitialization;
    double maxTravelDurationSec;
    double maxVehicleSpeedMetersPerSec;
    bool enableDynamicRouteSearch;

    struct VehicleSchedule {
        VehicleFamilyIdType familyId;
        shared_ptr<deque<VehicleStopInfo> > stopsPtr;
        TimeType currentDelay;

        size_t numberCycles;
        vector<pair<TimeType, TimeType> > timesPerStop;
        map<StopIdType, set<TimeType> > arrivalTimes;

        set<AgentIdType> reservedAgentIds;
        VehicleType vehicleType;

        LineType lineType;

        VehicleSchedule()
            :
            currentDelay(ZERO_TIME),
            numberCycles(0),
            vehicleType(VEHICLE_NONE),
            lineType(LINE_NORMAL)
        {}

        VehicleSchedule(const VehicleType& initVehicleType)
            :
            currentDelay(ZERO_TIME),
            numberCycles(0),
            vehicleType(initVehicleType),
            lineType(LINE_NORMAL)
        {}
    };

    struct RouteSchedule {
        vector<StopIdType> stopIds;
        vector<VehicleSchedule> vehicleSchedules;
        vector<TimeType> delays;

        shared_ptr<vector<vector<Vertex> > > trainWaypointsPtr;
        shared_ptr<AgentRoute> busRoutePtr;

        vector<pair<double, int> > prices;

        bool isLoopLine;
        TimeType loopIntervalBeweenVehicles;
        TimeType loopLineEndTime;

        TimeType loopIntervalForAVehicle;
        TimeType lapAroundTime;
        size_t loopCount;

        RouteSchedule() {
            trainWaypointsPtr.reset(new vector<vector<Vertex> >());
            busRoutePtr.reset(new AgentRoute());
        }

        RouteSchedule(
            const vector<pair<double, int> >& initPrices,
            const bool initIsLoopLine,
            const TimeType& initLoopIntervalBeweenVehicles,
            const TimeType& initLoopLineEndTime)
            :
            prices(initPrices),
            isLoopLine(initIsLoopLine),
            loopIntervalBeweenVehicles(initLoopIntervalBeweenVehicles),
            loopLineEndTime(initLoopLineEndTime),
            loopIntervalForAVehicle(INFINITE_TIME),
            lapAroundTime(ZERO_TIME),
            loopCount(1)
        {
            trainWaypointsPtr.reset(new vector<vector<Vertex> >());
            busRoutePtr.reset(new AgentRoute());
        }

        double CalculatePrice(const double distance, const AgentTicketType& ticketType) const {
            const pair<double, int>& pricePerDistance = prices.at(ticketType);
            return pricePerDistance.first * int(1. + distance/pricePerDistance.second);
        }
        bool IsLoopLine() const {
            assert(!stopIds.empty());
            return (stopIds.front() == stopIds.back());
        }
        bool IsAssigned() const { return !stopIds.empty(); }

        bool IsExpressRoute(
            const StopIdType& startStopId,
            const StopIdType& destStopId) const {

            return (startStopId == stopIds.front() &&
                    destStopId == stopIds.back());
        }
    };

    struct LineSchedule {
        VehicleType vehicleType;
        vector<RouteSchedule> routeSchedules;

        set<StopIdType> stopIds;

        bool IsTrain() const { return (vehicleType == VEHICLE_TRAIN); }
        bool IsBus() const { return (vehicleType == VEHICLE_BUS); }

        LineSchedule() : vehicleType(VEHICLE_NONE) {}
        LineSchedule(const VehicleType& initVehicleType)
            :
            vehicleType(initVehicleType)
        {}
    };

    struct ArrivalUnit {
        LineIdType lineId;
        RouteIdType routeId;
        VehicleNumberType vehicleNumber;
        StopNumberType stopNumber;

        LineType lineType;

        ArrivalUnit()
            :
            routeId(INVALID_ROUTE_ID),
            vehicleNumber(0),
            stopNumber(NO_STOP_NUMBER),
            lineType(LINE_NORMAL)
        {}

        ArrivalUnit(
            const LineIdType& initLineId,
            const RouteIdType& initRouteId = INVALID_ROUTE_ID,
            const VehicleNumberType& initVehicleNumber = 0,
            const StopNumberType& initStopNumber = NO_STOP_NUMBER,
            const LineType& initLineType = LINE_NORMAL)
            :
            lineId(initLineId),
            routeId(initRouteId),
            vehicleNumber(initVehicleNumber),
            stopNumber(initStopNumber),
            lineType(initLineType)
        {}

        bool IsTransferLine() const { return (lineId == TRANSFER_LINE_ID); }

        static const ArrivalUnit CreateTransferLineUnit() {
            return ArrivalUnit(TRANSFER_LINE_ID);
        }
    };

    struct LineStopInformation {
        vector<StopNumberType> stopNumberPerRoute;
    };

    struct StopLink {
        bool isEnabled;

        vector<LineIdType> lineIds;

        StopLink()
            :
            isEnabled(true)
        {}

        bool IsTransferLine() const { return lineIds.empty(); }
    };

    struct StopDetail {
        VehicleType vehicleType;
        VariantIdType stationOrBusStopId;

        map<LineIdType, LineStopInformation> lineStopInfo;

        map<pair<LineIdType, RouteIdType>, map<TimeType, ArrivalUnit> > arrivalVehiclesPerLineRoute;
        map<pair<LineIdType, RouteIdType>, map<TimeType, ArrivalUnit> > departureVehiclesPerLineRoute;

        vector<StopIdType> transferStopIds;

        const map<pair<LineIdType, RouteIdType>, map<TimeType, ArrivalUnit> >& GetArrivalOrDepartureVehicles(const bool isArrival) const {
            if (isArrival) {
                return arrivalVehiclesPerLineRoute;
            } else {
                return departureVehiclesPerLineRoute;
            }
        }

        bool IsStation() const { return (vehicleType == VEHICLE_TRAIN); }
        bool IsBusStop() const { return (vehicleType == VEHICLE_BUS); }

        StopDetail()
            :
            vehicleType(VEHICLE_NONE),
            stationOrBusStopId(INVALID_VARIANT_ID)
        {}

        bool IsSingleLineStop() const { return (lineStopInfo.size() == 1); }

        LineIdType GetLineId() const {
            assert((*this).IsSingleLineStop());
            return (*lineStopInfo.begin()).first;
        }
    };

    struct RouteSearchStop {
        double cost;

        bool needToRecalculateArrivalUnitCandidates;
        vector<pair<TimeType, ArrivalUnit> > arrivalUnitCandidates;

        double duration;
        StopIdType trackStopId;
        ArrivalUnit arrivalUnit;
        StopNumberType departureStopNumber;
        size_t numberTransferTime;
        set<LineIdType> ignoreLineIds;
        set<StopIdType> ignoreStopIds;
        double price;
        int transferTime;

        void Initialize() {
            cost = -1;
            needToRecalculateArrivalUnitCandidates = true;
            arrivalUnitCandidates.clear();
            duration = 0;
            departureStopNumber = NO_STOP_NUMBER;
            arrivalUnit.stopNumber = NO_STOP_NUMBER;
            numberTransferTime = 0;
            ignoreLineIds.clear();
            ignoreStopIds.clear();
            price = 0;
        }

        void SetBestRoute(
            const StopIdType& initTrackStopId,
            const double& initCost,
            const double& initDuration,
            const ArrivalUnit& initArrivalUnit,
            const StopNumberType& initDepartureStopNumber,
            const StopNumberType& initArrivalStopNumber) {
            trackStopId = initTrackStopId;
            cost = initCost;
            duration = initDuration;
            arrivalUnit = initArrivalUnit;
            arrivalUnit.stopNumber = initArrivalStopNumber;
            departureStopNumber = initDepartureStopNumber;
            needToRecalculateArrivalUnitCandidates = true;
        }

        bool IsBetterRoute(const double routeCost) const {
            return ((cost < 0) || (routeCost < cost));
        }

        bool FoundRoute() const { return  (cost > 0); }
    };

    LabelMap<LineIdType, LineSchedule> lineSchedules;
    LabelMap<VehicleFamilyIdType, VehicleFamily> vehicleFamilies;
    LabelMap<StopIdType, StopDetail> stopDetails;
    vector<RouteSearchStop> routeSearchStops;

    map<size_t, VehicleFamilyIdType> busFamilyIds;

    map<string, BusStopIdType> busStopIds;
    map<string, LineIdType> busLineIds;

    map<RailRoadStationIdType, StopIdType> stationIdToStopIdMap;
    map<BusStopIdType, StopIdType> busStopIdToStopIdMap;
    vector<RailRoadLineIdType> railRoadLineIdPerLine;
    vector<BusLineIdType> busLineIdPerLine;

    SpatialObjectMap stopIdMap;

    vector<double> walkSpeedsPerMobilityClass;

    struct VehicleCreationEntry {
        TimeType time;
        LineIdType lineId;
        RouteIdType routeId;
        VehicleNumberType vehicleNumber;

        PublicVehicleIdType publicVehicleId;

        VehicleCreationEntry()
            :
            time(),
            lineId(),
            routeId(),
            vehicleNumber(),
            publicVehicleId()
        {}

        VehicleCreationEntry(
            const TimeType& initTime,
            const LineIdType initLineId,
            const RouteIdType& initRouteId,
            const VehicleNumberType& initVehicleNumber,
            const PublicVehicleIdType& initPublicVehicleId)
            :
            time(initTime),
            lineId(initLineId),
            routeId(initRouteId),
            vehicleNumber(initVehicleNumber),
            publicVehicleId(initPublicVehicleId)
        {}

        bool operator<(const VehicleCreationEntry& left) const {
            return (time > left.time);
        }
    };

    struct RouteScheduleInput {
        LineIdType lineId;
        RouteIdType routeId;
        vector<vector<IntersectionIdType> > intersectionIdsPerRoute;

        RouteScheduleInput(
            const LineIdType& initLineId,
            const RouteIdType& initRouteId,
            const vector<vector<IntersectionIdType> >& initIntersectionIdsPerRoute)
            :
            lineId(initLineId),
            routeId(initRouteId),
            intersectionIdsPerRoute(initIntersectionIdsPerRoute)
        {}
    };

    struct VehicleScheduleInput {
        LineIdType lineId;
        RouteIdType routeId;
        VehicleNumberType vehicleNumber;
        deque<pair<TimeType, TimeType> > timeLines;
        string parsingLine;

        VehicleScheduleInput(
            const LineIdType& initLineId,
            const RouteIdType& initRouteId,
            const VehicleNumberType& initVehicleNumber,
            const deque<pair<TimeType, TimeType> >& initTimeLines,
            const string& initParsingLine)
            :
            lineId(initLineId),
            routeId(initRouteId),
            vehicleNumber(initVehicleNumber),
            timeLines(initTimeLines),
            parsingLine(initParsingLine)
        {}
    };

    priority_queue_stable<VehicleCreationEntry> vehicleCreationTable;
    vector<RouteScheduleInput> routeScheduleInputs;
    vector<VehicleScheduleInput> vehicleScheduleInputs;

    void InitializeRouteAndVehicles();

    void InitializeStopConnectionAndRouteSearchStops();

    void InitializeRouteSearchStop(
        const AgentResource& resource,
        const StopIdType& startStopId);

    void UpdateRouteCost(
        const AgentResource& resource,
        const AgentBehaviorType& preferenceBehavior,
        AgentRoute& routeCandidate) const;

    void SearchPublicVehicleRouteWithArrivalTime(
        const AgentResource& resource,
        const StopIdType& startStopId,
        const StopIdType& destStopId,
        const TimeToSearchRoute& timeToSearchRoute,
        const AgentBehaviorType& preferenceBehavior,
        const set<AgentBehaviorType>& notAvailableBehavorTypes,
        const size_t numberMaxRoutes,
        const size_t numberMaxTransferTimes,
        const vector<vector<shared_ptr<AgentRoute> > >& routeCandidatePtrsPerOrder,
        vector<multimap<double, shared_ptr<AgentRoute> > >& routePtrsPerOrder);

    void SearchPublicVehicleRouteWithDepartureTime(
        const AgentResource& resource,
        const StopIdType& startStopId,
        const StopIdType& destStopId,
        const TimeToSearchRoute& timeToSearchRoute,
        const AgentBehaviorType& preferenceBehavior,
        const set<AgentBehaviorType>& notAvailableBehavorTypes,
        const size_t numberMaxRoutes,
        const size_t numberMaxTransferTimes,
        const vector<vector<shared_ptr<AgentRoute> > >& routeCandidatePtrsPerOrder,
        vector<multimap<double, shared_ptr<AgentRoute> > >& routePtrsPerOrder);

    void FindBetterArrivalOrDepartureInfos(
        const AgentResource& resource,
        const StopIdType& stopId,
        const TimeType& earlyArrivalOrDepartureTime,
        const TimeType& arrivalOrDepartureTime,
        const TimeType& lateArrivalOrDepartureTime,
        const size_t numberMaxBetterVehiclesPerLine,
        const set<AgentBehaviorType>& notAvailableBehavorTypes,
        const bool isArrival,
        vector<pair<TimeType, ArrivalUnit> >& arrivalInfos) const;

    void FindBetterArrivalOrDepartureInfo(
        const map<TimeType, ArrivalUnit>& arrivalOrDepartureVehicles,
        const TimeType& earlyArrivalOrDepartureTime,
        const TimeType& arrivalOrDepartureTime,
        const TimeType& lateArrivalOrDepartureTime,
        bool& found,
        pair<TimeType, ArrivalUnit>& arrivalInfo) const;

    void GetBetterRouteFromDest(
        const AgentResource& resource,
        const StopIdType& startStopId,
        const StopIdType& destStopId,
        const TimeType& earlyDepartureTime,
        const TimeType& destArrivalTime,
        const ArrivalUnit& destArrivalUnit,
        const AgentRouteOrderType& order,
        const size_t numberMaxTransferTimes,
        const set<AgentBehaviorType>& notAvailableBehavorTypes,
        const double maxDuration,
        double& minDuration,
        AgentRoute& route);

    void GetBetterRouteFromStart(
        const AgentResource& resource,
        const StopIdType& startStopId,
        const StopIdType& destStopId,
        const TimeType& lateArrivalTime,
        const TimeType& startDepartureTime,
        const ArrivalUnit& startArrivalUnit,
        const AgentRouteOrderType& order,
        const size_t numberMaxTransferTimes,
        const set<AgentBehaviorType>& notAvailableBehavorTypes,
        const double maxDuration,
        double& minDuration,
        AgentRoute& route);

    void FindArrivalInfos(
        const AgentResource& resource,
        const StopIdType& stopId,
        const TimeType& earlyDepartureTime,
        const TimeType& arrivalTime,
        const set<AgentBehaviorType>& notAvailableBehavorTypes,
        const set<LineIdType>& ignoreLineIds,
        vector<pair<TimeType, ArrivalUnit> >& arrivalInfos) const;

    void FindDepartureInfos(
        const AgentResource& resource,
        const StopIdType& stopId,
        const TimeType& lateArrivalTime,
        const TimeType& departureTime,
        const set<AgentBehaviorType>& notAvailableBehavorTypes,
        const set<LineIdType>& ignoreLineIds,
        vector<pair<TimeType, ArrivalUnit> >& arrivalInfos) const;

    bool IsAvailableLink(
        const AgentResource& resource,
        const StopIdType& stopId,
        const StopIdType& linkedStopId,
        const ArrivalUnit& arrivalUnit) const;

    TimeType GetStopDelay(
        const ArrivalUnit& arrivalUnit) const;

    VehicleFamilyIdType GetSuitableVehicleFamilyId(const size_t numberReservations) const;

};

}//namespace ScenSim

#endif
