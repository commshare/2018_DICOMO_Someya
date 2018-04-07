// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "multiagent_publicvehicle.h"
#include "multiagent_gis.h"
#include "multiagent_extension.h"

namespace MultiAgent {

using ScenSim::ConvertToUShortInt;

static inline
double ConvertToDouble(
    const string& aString,
    const string& line)
{
    bool success;
    double aValue;

    ConvertStringToDouble(aString, aValue, success);

    if (!success) {
        cerr << "Error: failed to read double value "
             << aString << " at " << line << endl;
        exit(1);
    }

    return aValue;
}

static inline
int ConvertToInt(
    const string& aString,
    const string& line)
{
    bool success;
    int aValue;

    ConvertStringToInt(aString, aValue, success);

    if (!success) {
        cerr << "Error: failed to read int value "
             << aString << " at " << line << endl;
        exit(1);
    }

    return aValue;
}

static inline
TimeType ConvertToTime(
    const string& aString,
    const string& line)
{
    bool success;
    TimeType aTime;

    ConvertStringToTime(aString, aTime, success);

    if (!success) {
        cerr << "Error: failed to read time value "
             << aString << " at " << line << endl;
        exit(1);
    }

    return aTime;
}

static inline
void ConvertToTimeLines(
    const TimeType& startTime,
    deque<string>& timeLineStrings,
    deque<pair<TimeType, TimeType> >& timeLines)
{
    timeLines.clear();

    for(size_t i = 0; i < timeLineStrings.size(); i++) {
        const string& timeLineString = timeLineStrings[i];

        deque<string> arrivalTimeAndDepartureTime;
        TokenizeToTrimmedLowerString(timeLineString, "-", arrivalTimeAndDepartureTime);

        if (arrivalTimeAndDepartureTime.empty()) {
            cerr << "Error: time line format [Time]-[Time]" << endl;
            exit(1);
        }

        const TimeType arrivalTime =
            ConvertToTime(arrivalTimeAndDepartureTime[0], timeLineString) - startTime;

        if (arrivalTimeAndDepartureTime.size() == 1) {

            timeLines.push_back(make_pair(arrivalTime, arrivalTime));

        } else {

            const TimeType departureTime =
                ConvertToTime(arrivalTimeAndDepartureTime[1], timeLineString) - startTime;

            timeLines.push_back(make_pair(arrivalTime, departureTime));
        }
    }
}

static inline
bool IsAvailableVehicleType(
    const VehicleType& vehicleType,
    const set<AgentBehaviorType>& notAvailableBehavorTypes)
{
    if (notAvailableBehavorTypes.empty()) {
        return true;
    }

    if (vehicleType == VEHICLE_TRAIN) {
        if (notAvailableBehavorTypes.find(AGENT_BEHAVIOR_TRAIN) == notAvailableBehavorTypes.end()) {
            return true;
        }
    } else {
        assert(vehicleType == VEHICLE_BUS);

        if (notAvailableBehavorTypes.find(AGENT_BEHAVIOR_BUS) == notAvailableBehavorTypes.end()) {
            return true;
        }
    }

    return true;
}

void WaypointsType::AssignConstantWaypoints(
    const TimeType& startTime,
    const TimeType& endTime,
    const vector<Vertex>& points)
{
    (*this).clear();

    assert(!points.empty());
    assert(endTime >= startTime);

    const double totalDistance = CalculateArcDistance(points);
    const TimeType runningDuration = endTime - startTime;
    const double constantVelocity = totalDistance / runningDuration;

    TimeOffsetType stopTime = startTime;

    (*this).push_back(WaypointType(stopTime, points[0]));

    for(size_t pointId = 0; pointId < points.size() - 1; pointId++) {
        const double distance =
            points[pointId].DistanceTo(points[pointId + 1]);

        stopTime +=
            static_cast<TimeOffsetType>(distance / constantVelocity);

        (*this).push_back(WaypointType(stopTime, points[pointId + 1]));
    }
}

void WaypointsType::operator+=(const WaypointsType& right)
{
    for(size_t i = 0; i < right.size(); i++) {
        (*this).push_back(right[i]);
    }
}

//---------------------------------------------------------------------

VertexIdType PublicVehicle::GetViaPointVertexId(const TimeType& currentTime) const
{
    StopIdType stopId;

    if ((*this).AtStop(currentTime)) {
        stopId = stopsPtr->at(lastStopNumber).stopId;
    } else {
        stopId = stopsPtr->at(std::min(int(stopsPtr->size()) - 1, int(lastStopNumber) + 1)).stopId;
    }

    return publicVehicleTablePtr->GetLineVertexId(stopId, lineId, routeId);
}

GisPositionIdType PublicVehicle::GetViaPointPositionId(const TimeType& currentTime) const
{
    StopIdType stopId;

    if ((*this).AtStop(currentTime)) {
        stopId = stopsPtr->at(lastStopNumber).stopId;
    } else {
        stopId = stopsPtr->at(std::min(int(stopsPtr->size()) - 1, int(lastStopNumber) + 1)).stopId;
    }

    return publicVehicleTablePtr->GetPositionId(stopId);
}

bool PublicVehicle::IsNeverVisitableBusStop(const BusStopIdType& busStopId) const
{
    for(size_t i = lastStopNumber + 1; i < stopsPtr->size(); i++) {
        const StopIdType& stopId = stopsPtr->at(i).stopId;

        if (publicVehicleTablePtr->IsBusStop(stopId)) {

            if (publicVehicleTablePtr->GetBusStopId(stopId) == busStopId) {
                return false;
            }
        }
    }

    return true;
}

bool PublicVehicle::IsNeverVisitableStation(const RailRoadStationIdType& stationId) const
{
    for(size_t i = lastStopNumber + 1; i < stopsPtr->size(); i++) {
        const StopIdType& stopId = stopsPtr->at(i).stopId;

        if (publicVehicleTablePtr->IsStation(stopId)) {

            if (publicVehicleTablePtr->GetStationId(stopId) == stationId) {
                return false;
            }
        }
    }

    return true;
}

//---------------------------------------------------------------------

Train::Train(
    PublicVehicleTable* initPublicVehicleTablePtr,
    const PublicVehicleIdType& initPublicVehicleId,
    const LineIdType& initLineId,
    const RoadIdType& initRouteId,
    const VehicleNumberType& initVehicleNumber,
    const VehicleFamily& initVehicleFamily,
    const LineType& initLineType,
    const shared_ptr<const deque<VehicleStopInfo> >& initStopsPtr,
    const shared_ptr<const vector<vector<Vertex> > >& initWaypointsPtr)
    :
    PublicVehicle(initPublicVehicleTablePtr, initStopsPtr,initPublicVehicleId, initLineId, initRouteId, initVehicleNumber, initVehicleFamily, initLineType),
    waypointsPtr(initWaypointsPtr),
    driverAgentId(MASTER_ANY_AGENT_ID),
    atStation(false),
    currentDirectionRadians(0),
    lastWaypointNumber(0)
{
    const deque<VehicleStopInfo>& stops = *stopsPtr;

    assert(stops.size() > 1);

    const VehicleStopInfo& stop1 = stops[0];
    const VehicleStopInfo& stop2 = stops[1];

    currentWaypoints.AssignConstantWaypoints(
        stop1.departureTime, stop2.arrivalTime, waypointsPtr->at(0));

    assert(currentWaypoints.size() > 1);

    currentPosition = currentWaypoints[0].position;

    currentVector =
        (currentWaypoints[1].position - currentWaypoints[0].position) /
        double(currentWaypoints[1].time - currentWaypoints[0].time);

    currentDirectionRadians = currentVector.DirectionRadians();
}

void Train::SetTimeTo(const TimeType& time)
{
    const deque<VehicleStopInfo>& stops = *stopsPtr;

    bool advanceStop = false;
    bool advanceWaypoint = false;

    while (lastStopNumber + 1 < int(stops.size()) &&
           time > stops[lastStopNumber + 1].arrivalTime) {
        lastStopNumber++;
        advanceStop = true;
    }

    if (advanceStop && lastStopNumber < int(stops.size()) - 1) {
        const VehicleStopInfo& stop1 = stops[lastStopNumber];
        const VehicleStopInfo& stop2 = stops[lastStopNumber+1];

        currentWaypoints.AssignConstantWaypoints(
            stop1.departureTime,
            stop2.arrivalTime,
            waypointsPtr->at(lastStopNumber % waypointsPtr->size()));

        advanceWaypoint = true;
        lastWaypointNumber = 0;
    }

    const VehicleStopInfo& lastStopInfo = stops[lastStopNumber];

    if (lastStopNumber == int(stops.size()) - 1) {
        atStation = true;
        return;
    }

    atStation = (!lastStopInfo.IsPassStop() && time < lastStopInfo.departureTime);

    while (lastWaypointNumber + 1 < currentWaypoints.size() &&
           time > currentWaypoints[lastWaypointNumber + 1].time) {
        lastWaypointNumber++;
        advanceWaypoint = true;
    }
    if (advanceWaypoint && lastWaypointNumber + 1 < currentWaypoints.size()) {
        const WaypointType& lastWaypoint = currentWaypoints[lastWaypointNumber];
        const WaypointType& nextWaypoint = currentWaypoints[lastWaypointNumber+1];

        currentVector =
            (nextWaypoint.position - lastWaypoint.position) /
            double(nextWaypoint.time - lastWaypoint.time);

        if (!currentVector.IsNull()) {
            currentDirectionRadians = currentVector.DirectionRadians();
        }
    }

    const TimeType workDuration =
        std::max(ZERO_TIME, time - currentWaypoints[lastWaypointNumber].time);

    currentPosition = currentWaypoints[lastWaypointNumber].position +
        currentVector*double(workDuration);
}

TimeType Train::GetDepartureTime(const TimeType& currentTime) const
{
    const deque<VehicleStopInfo>& stops = *stopsPtr;

    if (lastStopNumber < int(stops.size())) {
        return stops[lastStopNumber].departureTime;
    }

    return INFINITE_TIME;
}

bool Train::AtStop(const TimeType& currentTime) const
{
    return atStation;
}

const StopIdType& Train::GetStopId() const
{
    if (atStation) {
        return stopsPtr->at(lastStopNumber).stopId;
    }

    return INVALID_VARIANT_ID;
}

RailRoadStationIdType Train::GetStationId() const
{
    const StopIdType stopId = (*this).GetStopId();

    if (stopId == INVALID_VARIANT_ID) {
        return INVALID_VARIANT_ID;
    }

    return publicVehicleTablePtr->GetStationId(stopId);
}

const RailRoadLineIdType& Train::GetRailRoadLineId() const
{
    return publicVehicleTablePtr->GetRailRoadLineId(lineId);
}

bool Train::GoToGarage() const
{
    return (lastWaypointNumber == currentWaypoints.size() - 1 && atStation);
}

//-------------------------------------------------------------------

 const size_t Vehicle::NO_CHANGED_LANE_NUMBER = BAD_SIZE_T;

Vehicle::Vehicle(
    const AgentIdType& initDriverAgentId,
    const VertexIdType& initVertexId,
    const Vertex& initPosition,
    const RoadIdType& initRoadId,
    const size_t initLaneNumber,
    const MultiAgentSimulator* initSimulatorPtr)
    :
    driverAgentId(initDriverAgentId),
    viapointVertexId(INVALID_VERTEX_ID),
    runningDistance(0),
    vertexId(initVertexId),
    nextVertexId(initVertexId),
    position(initPosition),
    nextPosition(initPosition),
    roadId(initRoadId),
    nextRoadId(initRoadId),
    laneNumber(initLaneNumber),
    nextLaneNumber(initLaneNumber),
    directionRadians(0),
    nextDirectionRadians(0),
    distanceToIntersection(DBL_MAX),
    nextDistanceToIntersection(DBL_MAX),
    changedLaneNumber(NO_CHANGED_LANE_NUMBER),
    nextChangedLaneNumber(NO_CHANGED_LANE_NUMBER),
    velocity(0.0),
    nextVelocity(0.0)
{
}

void Vehicle::SetVehicleConstant(const VehicleConstant& initVehicleConstant)
{
    assert(initVehicleConstant.vehicleHalfLength > 0 && "Vehicle body length MUST be greater than 0.");

    vehicleConstant = initVehicleConstant;
}

bool Vehicle::IsOnLane(const size_t targetLaneNumber) const
{
    if ((*this).IsOnRoad()) {

        if ((*this).GetLaneNumber() == targetLaneNumber) {

            return true;

        } else if ((*this).IsChangingLane() &&
                   (*this).GetChangedLaneNumber() == targetLaneNumber) {

            return true;
        }
    }

    return false;
}

Taxi::Taxi(
    const AgentIdType& initDriverAgentId,
    const AgentTaxiCenterIdType& initHomeTaxiCenterId,
    const VertexIdType& initVertexId,
    const Vertex& initPosition,
    const RoadIdType& initRoadId,
    const size_t initLaneNumber,
    const MultiAgentSimulator* initSimulatorPtr)
    :
    Vehicle(initDriverAgentId, initVertexId, initPosition, initRoadId, initLaneNumber, initSimulatorPtr),
    homeVertexId(initVertexId),
    homeTaxiCenterId(initHomeTaxiCenterId),
    pickupState(false),
    arrivedAthTheDestination(false)
{}

void Taxi::SetCustomerRequestRoute(const shared_ptr<AgentRoute>& initRequestRoutePtr)
{
    assert(!customers.empty());
    customers.front().requestRoutePtr = initRequestRoutePtr;
}

bool Taxi::HasCustomerRequestRoute() const
{
    if (customers.empty()) {
        return false;
    }

    return (customers.front().requestRoutePtr != nullptr);
}

shared_ptr<AgentRoute> Taxi::TakeCustomerRequestRoute()
{
    assert((*this).HasCustomerRequestRoute());
    assert(!customers.empty());
    shared_ptr<AgentRoute> routePtr = customers.front().requestRoutePtr;
    customers.front().requestRoutePtr.reset();
    return routePtr;
}

void Taxi::RemoveCustomerOrReservation(const AgentResource& customerResource)
{
    (*this).RemoveCustomer(customerResource);
    (*this).RemoveReservation(customerResource);
}


void Taxi::RemoveCustomer(const AgentResource& customerResource)
{
    typedef deque<CustomerData>::iterator IterType;

    IterType iter = customers.begin();

    while (iter != customers.end()) {

        if ((*iter).resource == customerResource) {
            iter = customers.erase(iter);
        } else {
            iter++;
        }
    }
}

void Taxi::RemoveReservation(const AgentResource& customerResource)
{
    if ((!reservedCustomers.empty()) && (reservedCustomers.front() == customerResource))
    {
        pickupState = false;
    }


    typedef deque<AgentResource>::iterator IterType;

    IterType foundIter =
        std::find(reservedCustomers.begin(),reservedCustomers.end(), customerResource);

    if (foundIter != reservedCustomers.end()) {
        reservedCustomers.erase(foundIter);
        assert(std::find(reservedCustomers.begin(),reservedCustomers.end(), customerResource) ==
               reservedCustomers.end());
    }//if//
}

void Taxi::TakeOnReservedCustomer(
    const AgentResource& customerResource,
    const shared_ptr<AgentRoute>& initRequestRoutePtr)
{
    arrivedAthTheDestination = false;

    (*this).RemoveReservation(customerResource);

    customers.push_back(
        CustomerData(
            customerResource,
            initRequestRoutePtr));

    // reset running distance

    runningDistance = 0;
}

void Taxi::AddReservedCustomer(AgentResource& customerResource)
{
    (*this).RemoveReservation(customerResource);

    reservedCustomers.push_back(customerResource);
}

bool Taxi::IsTookOnCustomer(const AgentResource& customerResource) const
{
    typedef deque<CustomerData>::const_iterator IterType;

    for(IterType iter = customers.begin(); iter != customers.end(); iter++) {

        if ((*iter).resource == customerResource) {
            return true;
        }
    }

    return false;
}

bool Taxi::CanPickUp(const AgentResource& customerResource) const
{
    assert(!reservedCustomers.empty());
    return (pickupState && reservedCustomers.front() == customerResource);
}

GisPositionIdType Taxi::GetCustomerPosition() const
{
    assert(!reservedCustomers.empty());
    const AgentResource& customer = reservedCustomers.front();
    if (customer.GetCurrentPositionId().type == GIS_ROAD) {
        return (customer.GetCurrentPositionId());
    }
    else {
        assert(customer.GetDesiredPositionId().type == GIS_ROAD);
        return (customer.GetDesiredPositionId());
    }//if//
}

AgentResource Taxi::GetReservedCustomer() const
{
    assert(!reservedCustomers.empty());
    return reservedCustomers.front();
}

bool Taxi::Contains(const AgentResource& customerResource) const
{
    typedef deque<CustomerData>::const_iterator IterType;

    for(IterType iter = customers.begin(); iter != customers.end(); iter++) {
        if ((*iter).resource == customerResource) {
            return true;
        }
    }

    return false;
}

bool Taxi::CanTakeOn(const AgentIdType&) const
{
    return customers.empty();
}

//-------------------------------------------------------------------------------------------

Bus::Bus(
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
    const set<AgentIdType>& initReservedAgentIds)
    :
    PublicVehicle(initPublicVehicleTablePtr, initStopsPtr, initPublicVehicleId, initLineId, initRouteId, initVehicleNumber, initVehicleFamily, initLineType),
    vehicleType(initVehicleType),
    routePtr(initRoutePtr),
    reservedAgentIds(initReservedAgentIds),
    startRoadId(initStartRoadId),
    lastArrivedTime(initCurrentTime),
    remainingNonReservedSeats(
        vehicleFamily.capacity - std::min(reservedAgentIds.size(), vehicleFamily.capacity)),
    ridingEndTime(ZERO_TIME)
{
    const deque<VehicleStopInfo>& stops = *stopsPtr;

    assert(!stops.empty());
}

TimeType Bus::GetDepartureTime(const TimeType& currentTime) const
{
    const VehicleStopInfo& stop = stopsPtr->at(lastStopNumber);
    const TimeType waitTime = stop.departureTime - stop.arrivalTime;

    return std::max(std::max(stop.departureTime, lastArrivedTime + waitTime), ridingEndTime);
}

bool Bus::ArrivedAtTheLastBusStop(const TimeType& currentTime) const
{
    const VehicleStopInfo& stop = stopsPtr->at(lastStopNumber);
    const TimeType waitTime = stop.departureTime - stop.arrivalTime;

    return (currentTime >= lastArrivedTime + waitTime &&
            lastStopNumber == StopNumberType(stopsPtr->size() - 1));
}

void Bus::ArrivedAtBusStop(const TimeType& time)
{
    destinationStopIds.erase(stopsPtr->at(lastStopNumber).stopId);

    lastArrivedTime = time;
    lastStopNumber++;
}

bool Bus::AtStop(const TimeType& currentTime) const
{
    const VehicleStopInfo& stop = stopsPtr->at(lastStopNumber);
    const TimeType waitTime = stop.departureTime - stop.arrivalTime;

    return (lastArrivedTime + waitTime > currentTime);
}

TimeType Bus::GetCurrentDelay(const TimeType& currentTime) const
{
    assert((*this).AtStop(currentTime));

    if (lastStopNumber >= StopNumberType(stopsPtr->size() - 1)) {
        return ZERO_TIME;
    }

    const VehicleStopInfo& stop = stopsPtr->at(lastStopNumber);

    if ((*this).AtStop(currentTime)) {
        return std::max(ZERO_TIME, currentTime - stop.departureTime);
    }

    return ZERO_TIME; //
}

const StopIdType& Bus::GetStopId() const
{
    return stopsPtr->at(lastStopNumber).stopId;
}

bool Bus::CanTakeOn(const AgentResource& resource) const
{
    if ((*this).IsFull()) {
        return false;
    }

    const AgentIdType agentId = resource.AgentId();

    if (reservedAgentIds.find(agentId) != reservedAgentIds.end()) {
        return true;
    }

    assert(resource.NumberPeople() >= 0);
    return (remainingNonReservedSeats >= static_cast<size_t>(resource.NumberPeople()));
}

void Bus::Push(
    const AgentResource& resource,
    const TimeType& currentTime)
{
    const TimeType ridingTime = resource.RidingTime();
    const AgentIdType agentId = resource.AgentId();

    if (reservedAgentIds.find(agentId) == reservedAgentIds.end()) {
        remainingNonReservedSeats -= resource.NumberPeople();
    }

    PublicVehicle::Push(resource, currentTime);

    ridingEndTime = std::max(currentTime, ridingEndTime) + ridingTime;
}

void Bus::Pop(const AgentResource& resource)
{
    const AgentIdType agentId = resource.AgentId();

    if (reservedAgentIds.find(agentId) == reservedAgentIds.end()) {
        remainingNonReservedSeats += resource.NumberPeople();
    }

    PublicVehicle::Pop(resource);
}

BusStopIdType Bus::GetStartBusStopId() const
{
    return publicVehicleTablePtr->GetBusStopId((*this).GetStartStopId());
}

BusStopIdType Bus::GetBusStopId(const size_t i) const
{
    return publicVehicleTablePtr->GetBusStopId(stopsPtr->at(i).stopId);
}

BusStopIdType Bus::GetDestBusStopId() const
{
    return publicVehicleTablePtr->GetBusStopId((*this).GetDestStopId());
}

BusStopIdType Bus::GetNextBusStopId() const
{
    assert(lastStopNumber + 1 < StopNumberType(stopsPtr->size()));

    const StopIdType nextStopId = stopsPtr->at(lastStopNumber + 1).stopId;

    return publicVehicleTablePtr->GetBusStopId(nextStopId);
}

VertexIdType Bus::GetNextBusStopVertexId() const
{
    assert(lastStopNumber + 1 < StopNumberType(stopsPtr->size()));

    const StopIdType nextStopId = stopsPtr->at(lastStopNumber + 1).stopId;

    return publicVehicleTablePtr->GetLineVertexId(nextStopId, lineId, routeId);
}

bool Bus::ContainsDestinationStopId(const StopIdType& stopId) const
{
    return (destinationStopIds.find(stopId) != destinationStopIds.end());
}

//--------------------------------------------------------------

PublicVehicleTable::PublicVehicleTable(
    MultiAgentSimulator* initSimulatorPtr,
    const ParameterDatabaseReader& initParameterDatabaseReader,
    const shared_ptr<MultiAgentGis>& initAgentGisPtr)
    :
    simulatorPtr(initSimulatorPtr),
    theAgentGisPtr(initAgentGisPtr),
    numberPublicVehicles(0),
    completedInitialization(false),
    maxTravelDurationSec(86400*2),
    maxVehicleSpeedMetersPerSec(300),
    enableDynamicRouteSearch(false)
{
    walkSpeedsPerMobilityClass.resize(GetNumberOfMobilityClasses());

    for(AgentMobilityClassType i = 0; i < AgentMobilityClassType(walkSpeedsPerMobilityClass.size()); i++) {
        walkSpeedsPerMobilityClass[i] = GetTypicalWalkSpeed(i);
    }

    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();
    const double spatialMeshUnit = 1; //1m

    stopIdMap.SetMesh(subsystem.GetEntireRect(), spatialMeshUnit);

    const string scheduleFilePath =
        initParameterDatabaseReader.ReadString(
            "gis-public-vehicle-file");

    const TimeType startTime =
        initParameterDatabaseReader.ReadTime(
            "multiagent-start-time");

    const TimeType endTime =
        initParameterDatabaseReader.ReadTime("simulation-time");

    (*this).LoadVehicleScheduleTable(scheduleFilePath, startTime, endTime);
}

void PublicVehicleTable::CompleteAllPublicVehicleInitialization()
{
    (*this).InitializeStopConnectionAndRouteSearchStops();

    (*this).ScheduleVehicleCreationTime();

    completedInitialization = true;
}

void PublicVehicleTable::CompleteRouteAndVehiclewScheduleInitialization()
{
    typedef vector<RouteScheduleInput>::const_iterator RouteIter;

    for(RouteIter iter = routeScheduleInputs.begin(); iter != routeScheduleInputs.end(); iter++) {
        const RouteScheduleInput& routeScheduleInput = (*iter);

        (*this).AssignStops(
            routeScheduleInput.lineId,
            routeScheduleInput.routeId,
            routeScheduleInput.intersectionIdsPerRoute);
    }

    routeScheduleInputs.clear();

    typedef vector<VehicleScheduleInput>::const_iterator VehicleIter;

    for(VehicleIter iter = vehicleScheduleInputs.begin(); iter != vehicleScheduleInputs.end(); iter++) {
        const VehicleScheduleInput& vehicleScheduleInput = (*iter);

        (*this).AddVehicleSchedule(
            vehicleScheduleInput.lineId,
            vehicleScheduleInput.routeId,
            vehicleScheduleInput.vehicleNumber,
            vehicleScheduleInput.timeLines,
            vehicleScheduleInput.parsingLine);
    }

    vehicleScheduleInputs.clear();
}

void PublicVehicleTable::LoadVehicleScheduleTable(
    const string& scheduleFilePath,
    const TimeType& startTime,
    const TimeType& endTime)
{
    ifstream inStream(scheduleFilePath.c_str());

    if (!inStream.good()) {
        cerr << "Error: Couldn't open Vehicle Schedule file " << scheduleFilePath << endl;
        exit(1);
    }

    LineIdType lineId = INVALID_LINE_ID;
    VehicleType vehicleType = VEHICLE_NONE;
    RouteIdType routeId = INVALID_ROUTE_ID;
    bool isLoopLine = false;
    TimeType loopInterval;
    TimeType loopLineEndTime;
    bool skipLine = false;

    vector<pair<double, int> > prices;
    vector<vector<IntersectionIdType> > intersectionIdsPerRoute;

    set<pair<LineIdType, RouteIdType> > assignedRouteIds;

    vector<RouteScheduleInput> routeScheduleInputsForLine;

    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();
    const RoadLayer& roadLayer = *subsystem.GetRoadLayerPtr();
    const RailRoadLayer& railRoadLayer = *subsystem.GetRailRoadLayerPtr();

    while(!inStream.eof()) {
        string aLine;
        getline(inStream, aLine);

        DeleteTrailingSpaces(aLine);

        if (IsAConfigFileCommentLine(aLine)) {
            continue;
        }

        deque<string> tokens;
        TokenizeToTrimmedLowerString(aLine, ",", tokens, false/*skipEmptyToken*/);

        if (tokens[0] == "family") {

            if (tokens.size() < 5) {
                cerr << "Error: family format name,width,length,capacity" << endl;
                exit(1);
            }

            const string& name = tokens[1];

            VehicleFamily& family = vehicleFamilies[name];
            family.name = name;
            family.widthMeters = ConvertToDouble(tokens[2], aLine);
            family.lengthMeters = ConvertToDouble(tokens[3], aLine);
            family.capacity = ConvertToInt(tokens[4], aLine);

            if (name.find("bus") != string::npos) {
                busFamilyIds[family.capacity] = vehicleFamilies.GetId(name);
            }

        } else if (tokens[0] == "line") {

            const string& lineName = tokens[1];

            if (lineSchedules.Contains(lineName)) {
                cerr << "Error: duplicated line name " << lineName << endl;
                exit(1);
            }

            lineId = lineSchedules.GetId(lineName);

            if (tokens[2] == "train") {
                vehicleType = VEHICLE_TRAIN;
            } else if (tokens[2] == "bus") {
                vehicleType = VEHICLE_BUS;
            } else {
                cerr << "Error: vehicle type " << tokens[2] << endl;
                exit(1);
            }

            isLoopLine = false;
            loopInterval = INFINITE_TIME;
            loopLineEndTime = endTime;
            skipLine = false;

            prices.resize(GetNumberOfTickets(), make_pair(0., 1));
            intersectionIdsPerRoute.clear();
            for(size_t i = 0; (i < routeScheduleInputsForLine.size()); i++) {
                routeScheduleInputs.push_back(routeScheduleInputsForLine[i]);
            }//for//
            routeScheduleInputsForLine.clear();

            lineSchedules[lineId] = LineSchedule(vehicleType);

            if (vehicleType == VEHICLE_TRAIN) {
                railRoadLineIdPerLine.resize(
                    std::max<size_t>(railRoadLineIdPerLine.size(), lineId + 1), INVALID_VARIANT_ID);
                railRoadLineIdPerLine[lineId] = railRoadLayer.GetRailRoadLineId(lineName);
            } else {
                busLineIdPerLine.resize(
                    std::max<size_t>(busLineIdPerLine.size(), lineId + 1), INVALID_VARIANT_ID);
                busLineIdPerLine[lineId] = roadLayer.GetBusLineId(lineName);
            }

        } else if (tokens[0] == "loopline") {

            if (tokens.size() < 4) {
                cerr << "Error: loopline format: tru/false, interval,endtime" << endl;
                exit(1);
            }

            if (tokens[1] == "true") {
                isLoopLine = true;
                loopInterval = ConvertToTime(tokens[2], aLine);
                loopLineEndTime = std::min(endTime, ConvertToTime(tokens[3], aLine));

                if (loopInterval != ZERO_TIME && loopInterval < 1 * SECOND) {
                    cerr << "Error: loop interval is too small. " << tokens[2] << endl;
                    exit(1);
                }

                if (loopLineEndTime != ZERO_TIME && loopLineEndTime < 1 * SECOND) {
                    cerr << "Error: loop end time is too small. " << tokens[3] << endl;
                    exit(1);
                }

                if (loopLineEndTime == ZERO_TIME) {
                    loopLineEndTime = endTime;
                }
            }

        } else if (skipLine) {

            continue;

        } else if (tokens[0] == "price") {

            tokens.pop_front();

            for(size_t i = 0; (i < tokens.size() && i < prices.size()); i++) {
                const pair<string, string> pricePerMeters = SeparateString(tokens[i],  "/");

                prices[i] = make_pair(
                    ConvertToDouble(pricePerMeters.first, aLine),
                    ConvertToInt(pricePerMeters.second, aLine));
            }

        } else if (tokens[0] == "stop") {

            tokens.pop_front();

            deque<VariantIdType> stationOrBusStopIds;

            if (vehicleType == VEHICLE_TRAIN) {
                const string& lineName = lineSchedules.GetLabel(lineId);

                if (!railRoadLayer.LineHasARoute(lineName)) {
                    skipLine = true;

                } else {

                    for(size_t i = 0; i < tokens.size(); i++) {
                        const string& stationName = tokens[i];

                        stationOrBusStopIds.push_back(
                            railRoadLayer.GetStationId(
                                lineSchedules.GetLabel(lineId),
                                stationName));
                    }

                    const RailRoadLineIdType& railroadLineId = railRoadLineIdPerLine.at(lineId);

                    if (railRoadLayer.ContainsRouteId(railroadLineId, stationOrBusStopIds)) {
                        routeId = railRoadLayer.GetRouteId(railroadLineId, stationOrBusStopIds);
                    }
                    else {
                        skipLine = true;
                    }
                }

            } else {

                for(size_t i = 0; i < tokens.size(); i++) {
                    stationOrBusStopIds.push_back(roadLayer.GetBusStopId(tokens[i]));
                }
                routeId = roadLayer.GetRouteId(busLineIdPerLine.at(lineId), stationOrBusStopIds);
            }

            if (skipLine) {
                continue;
            }

            if (assignedRouteIds.find(make_pair(lineId, routeId)) != assignedRouteIds.end()) {

                cerr << "Error: Already assigned stops for line " << lineSchedules.GetLabel(lineId) << endl;

                if (vehicleType == VEHICLE_TRAIN) {
                    const deque<RailRoadStationIdType>& stationIds = railRoadLayer.GetRouteStationIds(lineId, routeId);

                    cerr << lineSchedules.GetLabel(lineId) ;
                    for(size_t i = 0; i < stationIds.size(); i++) {
                        cerr << "," << railRoadLayer.GetStation(stationIds[i]).GetObjectName();
                    }
                    cerr << endl;

                } else {
                    const deque<BusStopIdType>& routeBusStopIds = roadLayer.GetRouteBusStopIds(lineId, routeId);

                    cerr << lineSchedules.GetLabel(lineId) ;
                    for(size_t i = 0; i < routeBusStopIds.size(); i++) {
                        cerr << "," << roadLayer.GetBusStop(routeBusStopIds[i]).GetObjectName();
                    }
                    cerr << endl;

                }

                exit(1);
            }

            assignedRouteIds.insert(make_pair(lineId, routeId));

            LineSchedule& lineSchedule = lineSchedules[lineId];

            while (routeId >= RouteIdType(lineSchedule.routeSchedules.size())) {
                lineSchedule.routeSchedules.push_back(RouteSchedule());
            }
            lineSchedule.routeSchedules[routeId] =
                RouteSchedule(
                    prices,
                    isLoopLine,
                    loopInterval,
                    loopLineEndTime);

            routeScheduleInputsForLine.push_back(
                RouteScheduleInput(
                    lineId,
                    routeId,
                    intersectionIdsPerRoute));

        } else if (tokens[0] == "route") {

            tokens.pop_front();

            if (vehicleType == VEHICLE_BUS) {
                for(size_t i = 0; i < tokens.size(); i++) {
                    deque<string> routeIntersectionNamess;
                    TokenizeToTrimmedLowerString(tokens[i], ":", routeIntersectionNamess);

                    intersectionIdsPerRoute.push_back(vector<IntersectionIdType>());
                    vector<IntersectionIdType>& intersectionIds = intersectionIdsPerRoute.back();

                    for(size_t j = 0; j < routeIntersectionNamess.size(); j++) {
                        const  GisPositionIdType positionId =
                            subsystem.GetPosition(routeIntersectionNamess[j], GIS_INTERSECTION);

                        if (!positionId.IsInvalid()) {
                            intersectionIds.push_back(positionId.id);
                        }
                    }
                }
            }

            if (!routeScheduleInputsForLine.empty()) {
                routeScheduleInputsForLine.back().intersectionIdsPerRoute = intersectionIdsPerRoute;
            }

        } else {

            assert(!tokens.empty());

            VehicleNumberType vehicleNumber;
            VehicleFamilyIdType familyId;

            if (vehicleType == VEHICLE_TRAIN) {
                vehicleNumber = ConvertToUShortInt(ConvertToInt(tokens[0], aLine));
                familyId = vehicleFamilies.GetId(tokens[1]);
                tokens.pop_front();
                tokens.pop_front();
            } else {
                vehicleNumber = (VehicleNumberType)(lineSchedules[lineId].routeSchedules.at(routeId).vehicleSchedules.size());
                familyId = vehicleFamilies.GetId(tokens[0]);
                tokens.pop_front();
            }

            deque<pair<TimeType, TimeType> > timeLines;
            ConvertToTimeLines(startTime, tokens, timeLines);

            if (!timeLines.empty() &&
                (ZERO_TIME <= timeLines.front().first && timeLines.front().first < endTime)) {

                RouteSchedule& routeSchedule = lineSchedules[lineId].routeSchedules.at(routeId);

                if (routeSchedule.isLoopLine) {
                    if (vehicleNumber > 0) {
                        cerr << "Error: Specify single schedule for loopline" << endl;
                        exit(1);
                    }
                }

                while (vehicleNumber >= VehicleNumberType(routeSchedule.vehicleSchedules.size())) {
                    routeSchedule.vehicleSchedules.push_back(VehicleSchedule());
                }
                VehicleSchedule& vehicleSchedule = routeSchedule.vehicleSchedules[vehicleNumber];

                if (vehicleSchedule.stopsPtr == nullptr) {
                    vehicleSchedule.familyId = familyId;
                    vehicleSchedule.stopsPtr.reset(new deque<VehicleStopInfo>());
                    vehicleSchedule.vehicleType = vehicleType;
                    vehicleSchedule.lineType = LINE_NORMAL;
                }

                vehicleScheduleInputs.push_back(
                    VehicleScheduleInput(
                        lineId,
                        routeId,
                        vehicleNumber,
                        timeLines,
                        aLine));
            }
        }
    }

    for(size_t i = 0; (i < routeScheduleInputsForLine.size()); i++) {
        routeScheduleInputs.push_back(routeScheduleInputsForLine[i]);
    }//for//
    routeScheduleInputsForLine.clear();
}

void PublicVehicleTable::AssignStops(
    const LineIdType& lineId,
    const RouteIdType& routeId,
    const vector<vector<IntersectionIdType> >& intersectionIdsPerRoute)
{
    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();
    const RailRoadLayer& railRoadLayer = *subsystem.GetRailRoadLayerPtr();
    const RoadLayer& roadLayer = *subsystem.GetRoadLayerPtr();

    LineSchedule& lineSchedule = lineSchedules[lineId];
    RouteSchedule& routeSchedule = lineSchedule.routeSchedules[routeId];

    if (routeSchedule.IsAssigned()) {
        cerr << "Error: Already assigned route shcedule." << endl;
        exit(1);
    }

    deque<VariantIdType> stationOrBusStopIds;
    if (lineSchedule.IsTrain()) {
        stationOrBusStopIds = railRoadLayer.GetRouteStationIds(railRoadLineIdPerLine.at(lineId), routeId);
    } else {
        stationOrBusStopIds = roadLayer.GetRouteBusStopIds(busLineIdPerLine.at(lineId), routeId);
    }

    for(VariantIdType i = 0; i < VariantIdType(stationOrBusStopIds.size()); i++) {
        const VariantIdType& stationOrBusStopId = stationOrBusStopIds[i];
        const string lineStopName =
            ConvertToString(lineSchedule.vehicleType) + "_" + ConvertToString(stationOrBusStopId);

        const StopIdType stopId = stopDetails.GetId(lineStopName);
        StopDetail& stopDetail = stopDetails[stopId];
        LineStopInformation& lineStopInfo = stopDetail.lineStopInfo[lineId];

        stopDetail.vehicleType = lineSchedule.vehicleType;
        stopDetail.stationOrBusStopId = stationOrBusStopId;

        routeSchedule.stopIds.push_back(stopId);
        lineSchedule.stopIds.insert(stopId);

        lineStopInfo.stopNumberPerRoute.resize(
            std::max<size_t>(lineStopInfo.stopNumberPerRoute.size(), routeId + 1),
            NO_STOP_NUMBER);

        lineStopInfo.stopNumberPerRoute[routeId] = i;
    }

    if (lineSchedule.IsTrain()) {

        railRoadLayer.GetLineVertices(
            railRoadLineIdPerLine.at(lineId),
            routeId,
            *routeSchedule.trainWaypointsPtr);

    } else {

        for(size_t i = 0; i < stationOrBusStopIds.size() - 1; i++) {

            deque<VertexIdType> passVertexIds;

            passVertexIds.push_back(
                roadLayer.GetBusStop(stationOrBusStopIds[i]).GetVertexId());

            if (i < intersectionIdsPerRoute.size()) {
                const vector<IntersectionIdType>& passIntersectionIds = intersectionIdsPerRoute[i];

                for(size_t j = 0; j < passIntersectionIds.size(); j++) {
                    passVertexIds.push_back(subsystem.GetIntersection(passIntersectionIds[j]).GetVertexId());
                }
            }

            passVertexIds.push_back(
                roadLayer.GetBusStop(stationOrBusStopIds[i+1]).GetVertexId());

            AgentRoute aRoute;

            theAgentGisPtr->SearchRoadRoute(
                simulatorPtr->GetMasterBusAgentResource(),
                passVertexIds,
                AGENT_BEHAVIOR_BUS,
                aRoute);

            if (aRoute.IsEmpty()) {
                routeSchedule.stopIds.clear();
                return;
            }

            routeSchedule.busRoutePtr->roadRoutes.insert(
                routeSchedule.busRoutePtr->roadRoutes.end(),
                aRoute.roadRoutes.begin(),
                aRoute.roadRoutes.end());
        }
    }

    for(size_t i = 0; i < routeSchedule.stopIds.size(); i++) {
        const StopIdType& stopId = routeSchedule.stopIds[i];
        const VertexIdType vertexId = (*this).GetLineVertexId(stopId, lineId, routeId);

        stopIdMap.InsertVertex(subsystem.GetVertex(vertexId), stopId);
    }

    routeSchedule.delays.resize(routeSchedule.stopIds.size(), ZERO_TIME);
}

void PublicVehicleTable::AddVehicleSchedule(
    const LineIdType& lineId,
    const RouteIdType& routeId,
    const VehicleNumberType& vehicleNumber,
    const deque<pair<TimeType, TimeType> >& timeLines,
    const string& parsingLine)
{
    RouteSchedule& routeSchedule = lineSchedules[lineId].routeSchedules.at(routeId);

    if (routeSchedule.stopIds.size() <= 1) {
        return;
    }

    if (routeSchedule.isLoopLine) {
        if (vehicleNumber != 0) {
            cerr << "Error: Specify single schedule for loopline" << endl;
            exit(1);
        }
    }

    while (vehicleNumber >= VehicleNumberType(routeSchedule.vehicleSchedules.size())) {
        routeSchedule.vehicleSchedules.push_back(VehicleSchedule());
    }
    VehicleSchedule& vehicleSchedule = routeSchedule.vehicleSchedules[vehicleNumber];

    const vector<StopIdType>& stopIds = routeSchedule.stopIds;
    deque<VehicleStopInfo>& stops = *vehicleSchedule.stopsPtr;

    if (!stops.empty()) {
        vehicleSchedule.numberCycles++;
    }

    const pair<LineIdType, RouteIdType> lineRouteId(lineId, routeId);

    assert(timeLines.size() == stopIds.size());
    for(size_t i = 0; i < timeLines.size(); i++) {
        const pair<TimeType, TimeType>& timeLine = timeLines[i];
        const TimeType arrivalTime = timeLine.first;
        const TimeType departureTime = timeLine.second;

        vehicleSchedule.timesPerStop.push_back(timeLine);

        if (i >= stopIds.size()) {
            cerr << "Warning: There is no stop to assign vehicle schedule:" << parsingLine << endl;
            return;
        }

        if (arrivalTime > departureTime) {
            cerr << "Error: Vehicle schedules must be time-lined:" << parsingLine << endl;
            exit(1);
        }

        const StopIdType& stopId = stopIds[i];

        vehicleSchedule.arrivalTimes[stopId].insert(arrivalTime);

        if (!stops.empty() && stops.back().stopId == stopId) {
            stops.back().departureTime = departureTime;
        } else {
            if (!stops.empty()) {
                const TimeType& lastDepartureTime = stops.back().departureTime;

                if (lastDepartureTime != INFINITE_TIME && arrivalTime < lastDepartureTime) {
                    cerr << "Error: Vehicle schedules must be time-lined:" << parsingLine << endl;
                    exit(1);
                }
                if (lastDepartureTime != INFINITE_TIME && arrivalTime == lastDepartureTime) {
                    cerr << "Error: Departure time and the following arrival time are the same value(" << ConvertTimeToStringSecs(arrivalTime) << "). (Traveling time is 0 between stops):" << parsingLine << endl;
                    exit(1);
                }
            }
            stops.push_back(VehicleStopInfo(stopId, arrivalTime, departureTime));
        }
    }

    (*this).RegisterVehicleArrivalTimeAndDepartureTime(lineId, routeId, vehicleNumber);

    (*this).AssignLoopLineVehiclesIfNecessary(lineId, routeId);
}


void PublicVehicleTable::RegisterVehicleArrivalTimeAndDepartureTime(
    const LineIdType& lineId,
    const RouteIdType& routeId,
    const VehicleNumberType& vehicleNumber)
{
    const RouteSchedule& routeSchedule = lineSchedules[lineId].routeSchedules.at(routeId);
    const vector<StopIdType>& stopIds = routeSchedule.stopIds;
    const VehicleSchedule& vehicleSchedule = routeSchedule.vehicleSchedules[vehicleNumber];
    const pair<LineIdType, RouteIdType> lineRouteId(lineId, routeId);
    const deque<VehicleStopInfo>& stops = (*vehicleSchedule.stopsPtr);

    assert(stops.size() == vehicleSchedule.timesPerStop.size());

    for(size_t i = 0; i < vehicleSchedule.timesPerStop.size(); i++) {
        StopIdType stopId;

        if (stops.size() == vehicleSchedule.timesPerStop.size()) {
            stopId = stops[i].stopId;
        } else {
            stopId = stopIds[i % stopIds.size()];
        }

        const pair<TimeType, TimeType>& timeLine = vehicleSchedule.timesPerStop[i];
        const TimeType& arrivalTime = timeLine.first;
        const TimeType& departureTime = timeLine.second;

        // pass this stop.
        if (arrivalTime == departureTime) {
            continue;
        }

        StopDetail& stopDetail = stopDetails[stopId];
        map<TimeType, ArrivalUnit>& arrivalVehicles = stopDetail.arrivalVehiclesPerLineRoute[lineRouteId];
        map<TimeType, ArrivalUnit>& departureVehicles = stopDetail.departureVehiclesPerLineRoute[lineRouteId];

        const ArrivalUnit arrivalUnit(
            lineId,
            routeId,
            vehicleNumber,
            (StopNumberType)(i + (vehicleSchedule.numberCycles*stopIds.size())), vehicleSchedule.lineType);

        // maybe overwritten for loopline

        arrivalVehicles.insert(make_pair(arrivalTime, arrivalUnit));

        const bool isLastStop =
            (i == stopIds.size() - 1);

        if (!isLastStop) {
            departureVehicles.insert(make_pair(departureTime, arrivalUnit));
        }
    }
}

void PublicVehicleTable::AssignLoopLineVehiclesIfNecessary(
    const LineIdType& lineId,
    const RouteIdType& routeId)
{
    RouteSchedule& routeSchedule = lineSchedules[lineId].routeSchedules.at(routeId);
    const vector<StopIdType>& stopIds = routeSchedule.stopIds;


    if (!routeSchedule.isLoopLine) {
        return;
    }

    assert(routeSchedule.vehicleSchedules.size() == 1);

    VehicleSchedule& firstVehicleSchedule = routeSchedule.vehicleSchedules[0];
    map<StopIdType, set<TimeType> >& arrivalTimesPerStop = firstVehicleSchedule.arrivalTimes;

    assert(!arrivalTimesPerStop.empty());

    typedef map<StopIdType, set<TimeType> >::const_iterator IterType;

    assert(!stopIds.empty());

    IterType firstIter = arrivalTimesPerStop.find(stopIds.front());

    assert(firstIter != arrivalTimesPerStop.end());

    const set<TimeType>& firstVehicleStartTimes = (*firstIter).second;

    assert(firstVehicleStartTimes.size() == 2);

    const TimeType& firstVehicleStartTime = (*firstVehicleStartTimes.begin());
    const TimeType& firstVehicleEndTime = (*firstVehicleStartTimes.rbegin());
    vector<pair<TimeType, TimeType> >& timesPerStop = firstVehicleSchedule.timesPerStop;

    assert(!timesPerStop.empty());

    const pair<TimeType, TimeType>& lastTimes = timesPerStop.back();

    routeSchedule.lapAroundTime = (firstVehicleEndTime - firstVehicleStartTime) + (lastTimes.second - lastTimes.first);

    routeSchedule.loopCount =
        static_cast<size_t>(std::ceil(double(routeSchedule.loopLineEndTime - firstVehicleEndTime) / routeSchedule.lapAroundTime) + 1);

    if (routeSchedule.loopCount <= 0) {
        cerr << "Error: Loop line end time " <<  ConvertTimeToDoubleSecs(routeSchedule.loopLineEndTime) << " is shorter than the first loop end time " << ConvertTimeToDoubleSecs(firstVehicleEndTime) << endl;
        exit(1);
    }

    if (routeSchedule.loopIntervalBeweenVehicles == ZERO_TIME) {
        routeSchedule.loopIntervalBeweenVehicles = routeSchedule.lapAroundTime;
    }

    routeSchedule.loopIntervalForAVehicle =
        static_cast<size_t>(std::ceil(double(routeSchedule.lapAroundTime) / routeSchedule.loopIntervalBeweenVehicles)) * routeSchedule.loopIntervalBeweenVehicles;

    const size_t numberVehicles = static_cast<size_t>(std::ceil(double(routeSchedule.lapAroundTime) / routeSchedule.loopIntervalBeweenVehicles));


    assert(routeSchedule.loopCount > 0);

    deque<VehicleStopInfo>& stops = (*firstVehicleSchedule.stopsPtr);

    assert(stops.size() == stopIds.size());
    assert(timesPerStop.size() == stopIds.size());

    // add extra route information

    const deque<RoadRoute> roadRoutes = routeSchedule.busRoutePtr->roadRoutes;

    for(size_t i = 1; i < routeSchedule.loopCount; i++) {

        const TimeType startTime = firstVehicleStartTime + routeSchedule.loopIntervalForAVehicle*i;

        assert(!timesPerStop.empty());

        pair<TimeType, TimeType>& currentLastTimes = timesPerStop.back();

        assert(currentLastTimes.first <= startTime);
        assert(currentLastTimes.second <= startTime);

        currentLastTimes.second = startTime;

        const size_t lastStartIndex =
            (timesPerStop.size() - stopIds.size());

        const TimeType offset =
            currentLastTimes.second - timesPerStop.at(lastStartIndex).second;

        assert(offset >= ZERO_TIME);
        assert(!stops.empty());

        stops.back().departureTime = startTime;

        for(size_t j = 1; j < stopIds.size(); j++) {
            const StopIdType& stopId = stopIds[j];

            timesPerStop.push_back(timesPerStop.at(lastStartIndex + j));

            pair<TimeType, TimeType>& times = timesPerStop.back();

            times.first += offset;
            times.second += offset;

            arrivalTimesPerStop[stopId].insert(times.first);

            stops.push_back(VehicleStopInfo(stopId, times.first, times.second));
        }

        for(size_t j = 0; j < roadRoutes.size(); j++) {
            routeSchedule.busRoutePtr->roadRoutes.push_back(roadRoutes[j]);
        }
    }

    // add offset vehicles

    if (numberVehicles > 1) {
        for(size_t i = 1; i < numberVehicles; i++) {
            const TimeType offset = routeSchedule.loopIntervalBeweenVehicles*i;

            // Retake front value so that vector implicit move front entry.

            VehicleSchedule offsetVehicleSchedule = routeSchedule.vehicleSchedules.front();

            const VehicleSchedule& baseVehicleSchedule = routeSchedule.vehicleSchedules.front();
            const vector<pair<TimeType, TimeType> >& baseTimesPerStop = baseVehicleSchedule.timesPerStop;

            vector<pair<TimeType, TimeType> >& offsetedTimesPerStop =
                offsetVehicleSchedule.timesPerStop;

            map<StopIdType, set<TimeType> >& offsetArrivalTimesPerStop =
                offsetVehicleSchedule.arrivalTimes;

            offsetVehicleSchedule.stopsPtr.reset(new deque<VehicleStopInfo>());

            deque<VehicleStopInfo>& offsetStops = (*offsetVehicleSchedule.stopsPtr);

            offsetedTimesPerStop.clear();
            offsetArrivalTimesPerStop.clear();

            for(size_t j = 0; j < baseTimesPerStop.size(); j++) {
                const StopIdType& stopId = stops.at(j).stopId;

                pair<TimeType, TimeType> times = baseTimesPerStop[j];

                times.first += offset;
                times.second += offset;

                offsetedTimesPerStop.push_back(times);

                offsetArrivalTimesPerStop[stopId].insert(times.first);

                offsetStops.push_back(VehicleStopInfo(stopId, times.first, times.second));

                if (stopId == stops.front().stopId &&
                    times.second >= routeSchedule.loopLineEndTime) {
                    break;
                }
            }

            if (offsetStops.size() > 1) {
                routeSchedule.vehicleSchedules.push_back(offsetVehicleSchedule);
            }
        }
    }

    for(VehicleNumberType i = 0; i < static_cast<VehicleNumberType>(routeSchedule.vehicleSchedules.size()); i++) {
        (*this).RegisterVehicleArrivalTimeAndDepartureTime(lineId, routeId, i);
    }
}

void PublicVehicleTable::ScheduleVehicleCreationTime()
{
    for(LineIdType lineId = 0; lineId < LineIdType(lineSchedules.size()); lineId++) {
        const LineSchedule& lineSchedule = lineSchedules[lineId];

        const vector<RouteSchedule>& routeSchedules = lineSchedule.routeSchedules;

        for(RouteIdType routeId = 0; routeId < RouteIdType(routeSchedules.size()); routeId++) {
            const RouteSchedule& routeSchedule = routeSchedules[routeId];
            const vector<VehicleSchedule>& vehicleSchedules = routeSchedule.vehicleSchedules;

            for(VehicleNumberType vehicleNumber = 0;
                vehicleNumber < VehicleNumberType(vehicleSchedules.size()); vehicleNumber++) {

                const VehicleSchedule& vehicleSchedule = vehicleSchedules[vehicleNumber];

                if (vehicleSchedule.stopsPtr == nullptr) {
                    continue;
                }

                assert(vehicleSchedule.stopsPtr != nullptr);
                const deque<VehicleStopInfo>& stops = *vehicleSchedule.stopsPtr;

                if (stops.empty()) {
                    // no route found.
                    continue;
                }

                const VehicleStopInfo& startStop = stops.front();

                const PublicVehicleIdType publicVehicleId =
                    (*this).GetNewPublicVehicleId();

                vehicleCreationTable.push(
                    VehicleCreationEntry(
                        startStop.arrivalTime,
                        lineId,
                        routeId,
                        vehicleNumber,
                        publicVehicleId));
            }
        }
    }
}

void PublicVehicleTable::CreatePublicVehicles(
    const TimeType& currentTime,
    vector<shared_ptr<Train> >& trainPtrs,
    vector<shared_ptr<Bus> >& busPtrs)
{
    trainPtrs.clear();
    busPtrs.clear();

    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();

    while (!vehicleCreationTable.empty() &&
           vehicleCreationTable.top().time <= currentTime) {
        const VehicleCreationEntry& vehicleEntry = vehicleCreationTable.top();

        const LineSchedule& lineSchedule = lineSchedules[vehicleEntry.lineId];
        const RouteSchedule& routeSchedule = lineSchedule.routeSchedules.at(vehicleEntry.routeId);
        const VehicleSchedule& vehicleSchedule = routeSchedule.vehicleSchedules.at(vehicleEntry.vehicleNumber);

        const VehicleFamily& vehicleFamily = vehicleFamilies[vehicleSchedule.familyId];
        const shared_ptr<deque<VehicleStopInfo> >& stopsPtr = vehicleSchedule.stopsPtr;

        if (!stopsPtr->empty()) {
            if (lineSchedule.vehicleType == VEHICLE_TRAIN) {
                trainPtrs.push_back(
                    shared_ptr<Train>(
                        new Train(
                            this,
                            vehicleEntry.publicVehicleId,
                            vehicleEntry.lineId,
                            vehicleEntry.routeId,
                            vehicleEntry.vehicleNumber,
                            vehicleFamily,
                            vehicleSchedule.lineType,
                            stopsPtr,
                            routeSchedule.trainWaypointsPtr)));

            } else if (lineSchedule.vehicleType == VEHICLE_BUS) {

                const BusStopIdType startBusStopId = (*this).GetBusStopId(stopsPtr->front().stopId);
                const VertexIdType vertexId = subsystem.GetBusStop(startBusStopId).GetVertexId();

                if (routeSchedule.busRoutePtr->roadRoutes.empty()) {
                    cerr << "No bus route" << endl;
                    exit(1);
                }

                RoadIdType startRoadId = routeSchedule.busRoutePtr->roadRoutes.front().roadId;

                if (subsystem.IsParkingVertex(vertexId)) {
                    startRoadId = subsystem.GetParkingRoadId(vertexId);
                }

                busPtrs.push_back(
                    shared_ptr<Bus>(
                        new Bus(
                            this,
                            vehicleSchedule.vehicleType,
                            currentTime,
                            vehicleEntry.publicVehicleId,
                            vehicleEntry.lineId,
                            vehicleEntry.routeId,
                            vehicleEntry.vehicleNumber,
                            vehicleFamily,
                            vehicleSchedule.lineType,
                            stopsPtr,
                            routeSchedule.busRoutePtr,
                            startRoadId,
                            vehicleSchedule.reservedAgentIds)));
            }
        }

        vehicleCreationTable.pop();
    }
}

PublicVehicleIdType PublicVehicleTable::GetNewPublicVehicleId()
{
    const PublicVehicleIdType newVehicleId = (PublicVehicleIdType)(numberPublicVehicles);

    numberPublicVehicles++;

    return newVehicleId;
}

void PublicVehicleTable::SearchPublicVehicleRoute(
    const AgentResource& resource,
    const StopIdType& startStopId,
    const StopIdType& destStopId,
    const TimeToSearchRoute& timeToSearchRoute,
    const AgentBehaviorType& preferenceBehavior,
    const set<AgentBehaviorType>& notAvailableBehavorTypes,
    const size_t numberMaxRoutes,
    const size_t numberMaxTransferTimes,
    vector<vector<shared_ptr<AgentRoute> > >& routeCandidatePtrsPerOrder)
{
    routeCandidatePtrsPerOrder.resize(NUMBER_AGENT_ROUTE_ORDERS);

    assert(completedInitialization);

    vector<multimap<double, shared_ptr<AgentRoute> > > routePtrsPerOrder(NUMBER_AGENT_ROUTE_ORDERS);

    if (timeToSearchRoute.specifiedArrivalTime) {

        (*this).SearchPublicVehicleRouteWithArrivalTime(
            resource,
            startStopId,
            destStopId,
            timeToSearchRoute,
            preferenceBehavior,
            notAvailableBehavorTypes,
            numberMaxRoutes,
            numberMaxTransferTimes,
            routeCandidatePtrsPerOrder,
            routePtrsPerOrder);

    } else {

        (*this).SearchPublicVehicleRouteWithDepartureTime(
            resource,
            startStopId,
            destStopId,
            timeToSearchRoute,
            preferenceBehavior,
            notAvailableBehavorTypes,
            numberMaxRoutes,
            numberMaxTransferTimes,
            routeCandidatePtrsPerOrder,
            routePtrsPerOrder);
    }

    for(size_t i = 0; i < routePtrsPerOrder.size(); i++) {
        const multimap<double, shared_ptr<AgentRoute> >& routePtrs = routePtrsPerOrder[i];
        vector<shared_ptr<AgentRoute> >& routeCandidatePtrs = routeCandidatePtrsPerOrder[i];

        typedef multimap<double, shared_ptr<AgentRoute> >::const_iterator IterType;

        IterType iter = routePtrs.begin();

        for(size_t numberRoutes = 0; (numberRoutes < numberMaxRoutes && iter != routePtrs.end()); numberRoutes++, iter++) {
            routeCandidatePtrs.push_back((*iter).second);
        }
    }
}

void PublicVehicleTable::SearchPublicVehicleRouteWithArrivalTime(
    const AgentResource& resource,
    const StopIdType& startStopId,
    const StopIdType& destStopId,
    const TimeToSearchRoute& timeToSearchRoute,
    const AgentBehaviorType& preferenceBehavior,
    const set<AgentBehaviorType>& notAvailableBehavorTypes,
    const size_t numberMaxRoutes,
    const size_t numberMaxTransferTimes,
    const vector<vector<shared_ptr<AgentRoute> > >& routeCandidatePtrsPerOrder,
    vector<multimap<double, shared_ptr<AgentRoute> > >& routePtrsPerOrder)
{
    const size_t numberMaxBetterVehiclesPerLine = 2;
    const bool isArrivalTime = true;

    vector<pair<TimeType, ArrivalUnit> > arrivalInfos;

    (*this).FindBetterArrivalOrDepartureInfos(
        resource,
        destStopId,
        timeToSearchRoute.earlyArrivalTime,
        timeToSearchRoute.arrivalTime,
        timeToSearchRoute.lateArrivalTime,
        numberMaxBetterVehiclesPerLine,
        notAvailableBehavorTypes,
        isArrivalTime,
        arrivalInfos);

    bool foundARoute = false;

    set<LineIdType> ignoredLineIds;

    for(size_t i = 0; i < arrivalInfos.size(); i++) {
        const pair<TimeType, ArrivalUnit>& arrivalInfo = arrivalInfos[i];
        const TimeType& arrivalTime = arrivalInfo.first;
        const ArrivalUnit& arrivalUnit = arrivalInfo.second;

        if (ignoredLineIds.find(arrivalUnit.lineId) != ignoredLineIds.end()) {
            continue;
        }

        vector<shared_ptr<AgentRoute> > routePtrPerOrder(NUMBER_AGENT_ROUTE_ORDERS);
        for(size_t j = 0; j < routePtrPerOrder.size(); j++) {
            routePtrPerOrder[j].reset(new AgentRoute(AGENT_BEHAVIOR_TRAIN));
        }

        double maxDuration = DBL_MAX;
        double transferRouteDuration;
        double priceRouteDuration = DBL_MAX;
        double routeDuration;

        (*this).GetBetterRouteFromDest(
            resource,
            startStopId,
            destStopId,
            timeToSearchRoute.earlyDepartureTime,
            arrivalTime,
            arrivalUnit,
            AGENT_ROUTE_ORDER_TRANSFER_TIME,
            numberMaxTransferTimes,
            notAvailableBehavorTypes,
            maxDuration,
            transferRouteDuration,
            *routePtrPerOrder[AGENT_ROUTE_ORDER_TRANSFER_TIME]);

        if (routePtrPerOrder[AGENT_ROUTE_ORDER_TRANSFER_TIME]->IsEmpty()) {
            ignoredLineIds.insert(arrivalUnit.lineId);
            continue;
        }
        foundARoute = true;

        (*this).GetBetterRouteFromDest(
            resource,
            startStopId,
            destStopId,
            timeToSearchRoute.earlyDepartureTime,
            arrivalTime,
            arrivalUnit,
            AGENT_ROUTE_ORDER_PRICE,
            numberMaxTransferTimes,
            notAvailableBehavorTypes,
            maxDuration,
            priceRouteDuration,
            *routePtrPerOrder[AGENT_ROUTE_ORDER_PRICE]);

        if (routePtrPerOrder[AGENT_ROUTE_ORDER_PRICE]->IsEmpty()) {
            *routePtrPerOrder[AGENT_ROUTE_ORDER_PRICE] = *routePtrPerOrder[AGENT_ROUTE_ORDER_TRANSFER_TIME];
        }

        (*this).GetBetterRouteFromDest(
            resource,
            startStopId,
            destStopId,
            timeToSearchRoute.earlyDepartureTime,
            arrivalTime,
            arrivalUnit,
            AGENT_ROUTE_ORDER_TIME,
            numberMaxTransferTimes,
            notAvailableBehavorTypes,
            std::min(transferRouteDuration, priceRouteDuration),
            routeDuration,
            *routePtrPerOrder[AGENT_ROUTE_ORDER_TIME]);

        if (routePtrPerOrder[AGENT_ROUTE_ORDER_TIME]->IsEmpty()) {
            if (transferRouteDuration < priceRouteDuration) {
                *routePtrPerOrder[AGENT_ROUTE_ORDER_TIME] = *routePtrPerOrder[AGENT_ROUTE_ORDER_TRANSFER_TIME];
            } else {
                *routePtrPerOrder[AGENT_ROUTE_ORDER_TIME] = *routePtrPerOrder[AGENT_ROUTE_ORDER_PRICE];
            }
        }

        for(AgentRouteOrderType order = 0; order < NUMBER_AGENT_ROUTE_ORDERS; order++) {
            shared_ptr<AgentRoute>& routePtr = routePtrPerOrder[order];

            (*this).UpdateRouteCost(resource, preferenceBehavior, *routePtr);

            const double cost = routePtr->totalCost.GetOrderCost(order);

            routePtrsPerOrder[order].insert(make_pair(cost, routePtr));
        }
    }

    if (!foundARoute) {
        (*this).SearchPublicVehicleRouteWithDepartureTime(
            resource,
            startStopId,
            destStopId,
            timeToSearchRoute,
            preferenceBehavior,
            notAvailableBehavorTypes,
            numberMaxRoutes,
            numberMaxTransferTimes,
            routeCandidatePtrsPerOrder,
            routePtrsPerOrder);
    }
}

void PublicVehicleTable::SearchPublicVehicleRouteWithDepartureTime(
    const AgentResource& resource,
    const StopIdType& startStopId,
    const StopIdType& destStopId,
    const TimeToSearchRoute& timeToSearchRoute,
    const AgentBehaviorType& preferenceBehavior,
    const set<AgentBehaviorType>& notAvailableBehavorTypes,
    const size_t numberMaxRoutes,
    const size_t numberMaxTransferTimes,
    const vector<vector<shared_ptr<AgentRoute> > >& routeCandidatePtrsPerOrder,
    vector<multimap<double, shared_ptr<AgentRoute> > >& routePtrsPerOrder)
{
    const size_t numberMaxBetterVehiclesPerLine = 2;
    const bool isArrivalTime = false;

    vector<pair<TimeType, ArrivalUnit> > arrivalInfos;

    (*this).FindBetterArrivalOrDepartureInfos(
        resource,
        startStopId,
        timeToSearchRoute.earlyDepartureTime,
        timeToSearchRoute.departureTime,
        timeToSearchRoute.lateDepartureTime,
        numberMaxBetterVehiclesPerLine,
        notAvailableBehavorTypes,
        isArrivalTime,
        arrivalInfos);

    set<LineIdType> ignoredLineIds;

    for(size_t i = 0; i < arrivalInfos.size(); i++) {
        const pair<TimeType, ArrivalUnit>& arrivalInfo = arrivalInfos[i];
        const TimeType& departureTime = arrivalInfo.first;
        const ArrivalUnit& arrivalUnit = arrivalInfo.second;

        if (ignoredLineIds.find(arrivalUnit.lineId) != ignoredLineIds.end()) {
            continue;
        }

        vector<shared_ptr<AgentRoute> > routePtrPerOrder(NUMBER_AGENT_ROUTE_ORDERS);
        for(size_t j = 0; j < routePtrPerOrder.size(); j++) {
            routePtrPerOrder[j].reset(new AgentRoute(AGENT_BEHAVIOR_TRAIN));
        }

        double maxDuration = DBL_MAX;
        double transferRouteDuration;
        double priceRouteDuration = DBL_MAX;
        double routeDuration;

        (*this).GetBetterRouteFromStart(
            resource,
            startStopId,
            destStopId,
            INFINITE_TIME,
            departureTime,
            arrivalUnit,
            AGENT_ROUTE_ORDER_TRANSFER_TIME,
            numberMaxTransferTimes,
            notAvailableBehavorTypes,
            maxDuration,
            transferRouteDuration,
            *routePtrPerOrder[AGENT_ROUTE_ORDER_TRANSFER_TIME]);

        if (routePtrPerOrder[AGENT_ROUTE_ORDER_TRANSFER_TIME]->IsEmpty()) {
            ignoredLineIds.insert(arrivalUnit.lineId);
            continue;
        }

        (*this).GetBetterRouteFromStart(
            resource,
            startStopId,
            destStopId,
            INFINITE_TIME,
            departureTime,
            arrivalUnit,
            AGENT_ROUTE_ORDER_PRICE,
            numberMaxTransferTimes,
            notAvailableBehavorTypes,
            maxDuration,
            priceRouteDuration,
            *routePtrPerOrder[AGENT_ROUTE_ORDER_PRICE]);

        if (routePtrPerOrder[AGENT_ROUTE_ORDER_PRICE]->IsEmpty()) {
            *routePtrPerOrder[AGENT_ROUTE_ORDER_PRICE] = *routePtrPerOrder[AGENT_ROUTE_ORDER_TRANSFER_TIME];
        }

        (*this).GetBetterRouteFromStart(
            resource,
            startStopId,
            destStopId,
            INFINITE_TIME,
            departureTime,
            arrivalUnit,
            AGENT_ROUTE_ORDER_TIME,
            numberMaxTransferTimes,
            notAvailableBehavorTypes,
            std::min(transferRouteDuration, priceRouteDuration),
            routeDuration,
            *routePtrPerOrder[AGENT_ROUTE_ORDER_TIME]);

        if (routePtrPerOrder[AGENT_ROUTE_ORDER_TIME]->IsEmpty()) {
            if (transferRouteDuration < priceRouteDuration) {
                *routePtrPerOrder[AGENT_ROUTE_ORDER_TIME] = *routePtrPerOrder[AGENT_ROUTE_ORDER_TRANSFER_TIME];
            } else {
                *routePtrPerOrder[AGENT_ROUTE_ORDER_TIME] = *routePtrPerOrder[AGENT_ROUTE_ORDER_PRICE];
            }
        }

        for(AgentRouteOrderType order = 0; order < NUMBER_AGENT_ROUTE_ORDERS; order++) {
            shared_ptr<AgentRoute>& routePtr = routePtrPerOrder[order];

            (*this).UpdateRouteCost(resource, preferenceBehavior, *routePtr);

            const double cost = routePtr->totalCost.GetOrderCost(order);
            routePtrsPerOrder[order].insert(make_pair(cost, routePtr));
        }
    }
}

void PublicVehicleTable::UpdateRouteCost(
    const AgentResource& resource,
    const AgentBehaviorType& preferenceBehavior,
    AgentRoute& routeCandidate) const
{
    deque<PublicVehicleRoute>& publicVehicleRoutes = routeCandidate.publicVehicleRoutes;

    if (publicVehicleRoutes.empty()) {
        return;
    }

    const TimeType departureTime = publicVehicleRoutes.front().departureTime;
    const TimeType arrivalTime = publicVehicleRoutes.back().arrivalTime;

    assert(/*orig*/departureTime < /*next*/arrivalTime);

    routeCandidate.totalCost.values[AGENT_ROUTE_COST_ARRIVAL_TIME] = double(arrivalTime) / SECOND;
    routeCandidate.totalCost.values[AGENT_ROUTE_COST_TRANSFER_TIME] = (double)(publicVehicleRoutes.size() - 1);


    for(size_t i = 0; i < publicVehicleRoutes.size() - 1; i++) {
        const PublicVehicleRoute& route1 = publicVehicleRoutes[i];
        const PublicVehicleRoute& route2 = publicVehicleRoutes[i+1];;

        assert(route2.departureTime - route1.departureTime >= 0);

        routeCandidate.totalCost.values[AGENT_ROUTE_COST_TRANSFER_DURATION] +=
            double(route2.departureTime - route1.arrivalTime) / SECOND;
    }

    routeCandidate.totalCost.values[AGENT_ROUTE_COST_TRAVEL_TIME] = double(arrivalTime - departureTime) / SECOND;

    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();
    bool isPreferenceBehavior = true;

    for(size_t i = 0; i < publicVehicleRoutes.size(); i++) {
        PublicVehicleRoute& route = publicVehicleRoutes[i];

        if (route.lineId == TRANSFER_LINE_ID) {
            continue;
        }

        const VertexIdType origVertexId = (*this).GetOrigVertexId(route);
        const VertexIdType nextVertexId = (*this).GetDestVertexId(route);
        const LineSchedule& lineSchedule = lineSchedules[route.lineId];
        const RouteSchedule& routeSchedule = lineSchedule.routeSchedules.at(route.routeId);
        const VehicleSchedule& vehicleSchedule = routeSchedule.vehicleSchedules.at(route.vehicleNumber);
        const double distance = subsystem.CalculateDistance(origVertexId, nextVertexId);

        const double price = routeSchedule.CalculatePrice(distance, resource.TicketType());

        route.price = price;

        routeCandidate.totalCost.values[AGENT_ROUTE_COST_VARIABILITY_TIME] += double((*this).GetStopDelay(route.lineId, route.routeId, route.stopNumber)) / SECOND;
        routeCandidate.totalCost.values[AGENT_ROUTE_COST_PRICE] += price;
        routeCandidate.totalCost.values[AGENT_ROUTE_COST_TRAVEL_DISTANCE] += distance;

        if (isPreferenceBehavior) {
            if (preferenceBehavior == AGENT_BEHAVIOR_TRAIN) {
                isPreferenceBehavior = stopDetails[route.origStopId].IsStation();
            } else if (preferenceBehavior == AGENT_BEHAVIOR_BUS) {
                isPreferenceBehavior = stopDetails[route.origStopId].IsBusStop();
            } else {
                isPreferenceBehavior = false;
            }
        }

    }

    if (isPreferenceBehavior) {
        routeCandidate.totalCost.values[AGENT_ROUTE_COST_MODE] = 1;
    }
}

double PublicVehicleTable::CalculateGetDownPrice(
    const PublicVehicleRoute& route,
    const VertexIdType& getDownVertexId,
    const AgentResource& resource) const
{
    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();
    const VertexIdType rideOnVertexId = (*this).GetOrigVertexId(route);
    const LineSchedule& lineSchedule = lineSchedules[route.lineId];
    const RouteSchedule& routeSchedule = lineSchedule.routeSchedules.at(route.routeId);
    const double distance = subsystem.CalculateDistance(rideOnVertexId, getDownVertexId);

    return routeSchedule.CalculatePrice(distance, resource.TicketType());
}

void PublicVehicleTable::FindArrivalInfos(
    const AgentResource& resource,
    const StopIdType& stopId,
    const TimeType& earlyDepartureTime,
    const TimeType& arrivalTime,
    const set<AgentBehaviorType>& notAvailableBehavorTypes,
    const set<LineIdType>& ignoreLineIds,
    vector<pair<TimeType, ArrivalUnit> >& arrivalInfos) const
{
    const StopDetail& stopDetail = stopDetails[stopId];
    const map<pair<LineIdType, RouteIdType>, map<TimeType, ArrivalUnit> >& arrivalVehiclesPerLineRoute =
        stopDetail.arrivalVehiclesPerLineRoute;

    arrivalInfos.clear();

    typedef map<pair<LineIdType, RouteIdType>, map<TimeType, ArrivalUnit> >::const_iterator RouteIter;

    for(RouteIter routeIter = arrivalVehiclesPerLineRoute.begin();
        routeIter != arrivalVehiclesPerLineRoute.end(); routeIter++) {

        typedef map<TimeType, ArrivalUnit>::const_iterator ArrivalIter;

        const pair<LineIdType, RouteIdType>& routeLineId = (*routeIter).first;

        if (ignoreLineIds.find(routeLineId.first) != ignoreLineIds.end()) {
            continue;
        }

        const LineSchedule& lineSchedule = lineSchedules[routeLineId.first];

        if (!IsAvailableVehicleType(lineSchedule.vehicleType, notAvailableBehavorTypes)) {
            continue;
        }

        bool found;
        pair<TimeType, ArrivalUnit> arrivalInfo;

        (*this).FindBetterArrivalOrDepartureInfo(
            (*routeIter).second,
            earlyDepartureTime,
            arrivalTime,
            arrivalTime,
            found,
            arrivalInfo);

        if (found) {
            arrivalInfos.push_back(arrivalInfo);
        }
    }
}

void PublicVehicleTable::FindDepartureInfos(
    const AgentResource& resource,
    const StopIdType& stopId,
    const TimeType& lateArrivalTime,
    const TimeType& departureTime,
    const set<AgentBehaviorType>& notAvailableBehavorTypes,
    const set<LineIdType>& ignoreLineIds,
    vector<pair<TimeType, ArrivalUnit> >& arrivalInfos) const
{
    const StopDetail& stopDetail = stopDetails[stopId];
    const map<pair<LineIdType, RouteIdType>, map<TimeType, ArrivalUnit> >& arrivalVehiclesPerLineRoute =
        stopDetail.departureVehiclesPerLineRoute;

    typedef multimap<TimeType, ArrivalUnit>::const_iterator ArrivalIter;

    arrivalInfos.clear();

    typedef map<pair<LineIdType, RouteIdType>, map<TimeType, ArrivalUnit> >::const_iterator RouteIter;

    for(RouteIter routeIter = arrivalVehiclesPerLineRoute.begin();
        routeIter != arrivalVehiclesPerLineRoute.end(); routeIter++) {

        typedef map<TimeType, ArrivalUnit>::const_iterator ArrivalIter;

        const pair<LineIdType, RouteIdType>& routeLineId = (*routeIter).first;

        if (ignoreLineIds.find(routeLineId.first) != ignoreLineIds.end()) {
            continue;
        }

        const LineSchedule& lineSchedule = lineSchedules[routeLineId.first];

        if (!IsAvailableVehicleType(lineSchedule.vehicleType, notAvailableBehavorTypes)) {
            continue;
        }

        bool found;
        pair<TimeType, ArrivalUnit> arrivalInfo;

        (*this).FindBetterArrivalOrDepartureInfo(
            (*routeIter).second,
            departureTime,
            departureTime,
            lateArrivalTime,
            found,
            arrivalInfo);

        if (found) {
            arrivalInfos.push_back(arrivalInfo);
        }
    }
}

void PublicVehicleTable::FindBetterArrivalOrDepartureInfos(
    const AgentResource& resource,
    const StopIdType& stopId,
    const TimeType& earlyArrivalOrDepartureTime,
    const TimeType& arrivalOrDepartureTime,
    const TimeType& lateArrivalOrDepartureTime,
    const size_t numberMaxBetterVehiclesPerLine,
    const set<AgentBehaviorType>& notAvailableBehavorTypes,
    const bool isArrival,
    vector<pair<TimeType, ArrivalUnit> >& arrivalInfos) const
{
    const StopDetail& stopDetail = stopDetails[stopId];
    const map<pair<LineIdType, RouteIdType>, map<TimeType, ArrivalUnit> >& arrivalOrDepartureVehiclesPerLineRoute =
        stopDetail.GetArrivalOrDepartureVehicles(isArrival);

    typedef map<pair<LineIdType, RouteIdType>, map<TimeType, ArrivalUnit> >::const_iterator RouteIter;

    multimap<TimeType, ArrivalUnit> earlyArrivals;
    multimap<TimeType, ArrivalUnit> lateArrivals;

    for(RouteIter routeIter = arrivalOrDepartureVehiclesPerLineRoute.begin();
        routeIter != arrivalOrDepartureVehiclesPerLineRoute.end(); routeIter++) {

        const pair<LineIdType, RouteIdType>& routeLineId = (*routeIter).first;
        const LineSchedule& lineSchedule = lineSchedules[routeLineId.first];

        if (!IsAvailableVehicleType(lineSchedule.vehicleType, notAvailableBehavorTypes)) {
            continue;
        }

        bool found;
        pair<TimeType, ArrivalUnit> arrivalInfo;

        (*this).FindBetterArrivalOrDepartureInfo(
            (*routeIter).second,
            earlyArrivalOrDepartureTime,
            arrivalOrDepartureTime,
            lateArrivalOrDepartureTime,
            found,
            arrivalInfo);

        if (found) {
            if (arrivalInfo.first <= arrivalOrDepartureTime) {
                arrivalInfo.first = arrivalOrDepartureTime - arrivalInfo.first;
                earlyArrivals.insert(arrivalInfo);
            } else {
                lateArrivals.insert(arrivalInfo);
            }
        }
    }

    arrivalInfos.clear();

    typedef multimap<TimeType, ArrivalUnit>::const_iterator ArrivalIter;
    typedef map<LineIdType, size_t>::iterator LineIter;

    map<LineIdType, size_t> numberRegisteredVehiclesPerLine;

    for(ArrivalIter iter = earlyArrivals.begin();
        iter != earlyArrivals.end(); iter++) {

        const ArrivalUnit& arrivalUnit = (*iter).second;
        const LineIdType& lineId = arrivalUnit.lineId;

        LineIter lineIter = numberRegisteredVehiclesPerLine.find(lineId);

        if (lineIter == numberRegisteredVehiclesPerLine.end()) {
            numberRegisteredVehiclesPerLine[lineId] = 0;
        } else {

            if ((*lineIter).second >= numberMaxBetterVehiclesPerLine) {
                continue;
            }
        }

        numberRegisteredVehiclesPerLine[lineId]++;

        arrivalInfos.push_back(*iter);
        arrivalInfos.back().first = arrivalOrDepartureTime - arrivalInfos.back().first;
    }

    for(ArrivalIter iter = lateArrivals.begin();
        iter != lateArrivals.end(); iter++) {

        const ArrivalUnit& arrivalUnit = (*iter).second;
        const LineIdType& lineId = arrivalUnit.lineId;

        LineIter lineIter = numberRegisteredVehiclesPerLine.find(lineId);

        if (lineIter == numberRegisteredVehiclesPerLine.end()) {
            numberRegisteredVehiclesPerLine[lineId] = 0;
        } else {

            if ((*lineIter).second >= numberMaxBetterVehiclesPerLine) {
                continue;
            }
        }

        numberRegisteredVehiclesPerLine[lineId]++;

        arrivalInfos.push_back(*iter);
    }
}

void PublicVehicleTable::FindBetterArrivalOrDepartureInfo(
    const map<TimeType, ArrivalUnit>& arrivalOrDepartureVehicles,
    const TimeType& earlyArrivalOrDepartureTime,
    const TimeType& arrivalOrDepartureTime,
    const TimeType& lateArrivalOrDepartureTime,
    bool& found,
    pair<TimeType, ArrivalUnit>& arrivalInfo) const
{
    assert(earlyArrivalOrDepartureTime <= arrivalOrDepartureTime);
    assert(arrivalOrDepartureTime <= lateArrivalOrDepartureTime);

    typedef map<TimeType, ArrivalUnit>::const_iterator IterType;

    IterType iter = arrivalOrDepartureVehicles.upper_bound(arrivalOrDepartureTime);

    while (iter != arrivalOrDepartureVehicles.begin()) {

        IterType prevIter = iter;
        prevIter--;

        TimeType arrivalTimeOrDepartureTimeWithDelay = (*prevIter).first;

        if (enableDynamicRouteSearch) {
            //Note: start stop vehicle delay is 0
            arrivalTimeOrDepartureTimeWithDelay += (*this).GetStopDelay((*prevIter).second);
        }

        if (arrivalTimeOrDepartureTimeWithDelay < earlyArrivalOrDepartureTime) {
            break;
        }

        if (arrivalTimeOrDepartureTimeWithDelay <= arrivalOrDepartureTime) {
            iter = prevIter;
            break;
        }

        iter = prevIter;
    }

    while (iter != arrivalOrDepartureVehicles.end()) {

        TimeType arrivalTimeOrDepartureTimeWithDelay = (*iter).first;

        if (enableDynamicRouteSearch) {
            arrivalTimeOrDepartureTimeWithDelay += (*this).GetStopDelay((*iter).second);
        }

        if (arrivalTimeOrDepartureTimeWithDelay >= lateArrivalOrDepartureTime) {
            break;
        }

        if (arrivalTimeOrDepartureTimeWithDelay >= earlyArrivalOrDepartureTime &&
            (*this).IsLineAvailable((*iter).second.lineId)) {

            arrivalInfo.first = arrivalTimeOrDepartureTimeWithDelay;
            arrivalInfo.second = (*iter).second;
            found = true;
            return;
        }

        iter++;
    }

    found = false;
}


void PublicVehicleTable::InitializeStopConnectionAndRouteSearchStops()
{
    const double maxTransferDistance = 200;

    set<set<StopIdType> > linkedStopIdsGroup;

    for(StopIdType stopId = 0; stopId < StopIdType(stopDetails.size()); stopId++) {
        StopDetail& stopDetail = stopDetails[stopId];

        const Vertex vertex = (*this).GetStopVertex(stopId);

        vector<StopIdType> stopIds;
        stopIdMap.GetGisObject(Rectangle(vertex, maxTransferDistance), stopIds);

        typedef vector<StopIdType>::const_iterator StopIter;

        set<StopIdType> linkedStopIds;

        for(StopIter iter = stopIds.begin(); iter != stopIds.end(); iter++) {
            const StopIdType& nearStopId = *iter;
            const Vertex nearVertex = (*this).GetStopVertex(nearStopId);

            if (nearStopId != stopId &&
                vertex.DistanceTo(nearVertex) <= maxTransferDistance) {

                linkedStopIds.insert(nearStopId);
            }
        }

        stopDetail.transferStopIds.assign(linkedStopIds.begin(), linkedStopIds.end());
        routeSearchStops.resize(stopDetails.size());
    }
}

class StopNode {
public:
    StopNode()
        :
        stopId(),
        cost(),
        expectedCost()
    {}

    StopNode(
        const StopIdType& initStopId,
        const double initCost,
        const double initExpectedCost)
        :
        stopId(initStopId),
        cost(initCost),
        expectedCost(initExpectedCost)
    {}

    bool operator<(const StopNode& right) const {
        return (expectedCost > right.expectedCost);
    }

    StopIdType stopId;
    double cost;
    double expectedCost;
};

void PublicVehicleTable::GetBetterRouteFromDest(
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
    AgentRoute& route)
{
    assert(startStopId != destStopId);

    (*this).InitializeRouteSearchStop(resource, destStopId);

    const Vertex& startVertex = (*this).GetStopVertex(startStopId);

    double minCost = DBL_MAX;
    priority_queue_stable<StopNode> stopNodes;

    routeSearchStops[destStopId].arrivalUnitCandidates.push_back(make_pair(destArrivalTime, destArrivalUnit));
    routeSearchStops[destStopId].needToRecalculateArrivalUnitCandidates = false;
    routeSearchStops[destStopId].ignoreStopIds.insert(destStopId);

    stopNodes.push(StopNode(destStopId, 0, 0));

    while (!stopNodes.empty()) {
        const StopNode stopNode = stopNodes.top();
        stopNodes.pop();

        RouteSearchStop& routeSearchStop = routeSearchStops[stopNode.stopId];

        if (stopNode.cost > routeSearchStop.cost) {
            continue;
        }

        const TimeType lateArrivalTime = destArrivalTime - static_cast<TimeType>(routeSearchStop.duration)*SECOND;

        if (routeSearchStop.needToRecalculateArrivalUnitCandidates) {
            (*this).FindArrivalInfos(
                resource,
                stopNode.stopId,
                earlyDepartureTime,
                std::max(lateArrivalTime, earlyDepartureTime),
                notAvailableBehavorTypes,
                routeSearchStop.ignoreLineIds,
                routeSearchStop.arrivalUnitCandidates);
        }

        const StopDetail& stopDetail = stopDetails[stopNode.stopId];
        const vector<pair<TimeType, ArrivalUnit> >& arrivalUnitCandidates = routeSearchStop.arrivalUnitCandidates;
        const Vertex& nextVertex = (*this).GetStopVertex(stopNode.stopId);
        const set<StopIdType>& ignoreStopIds = routeSearchStop.ignoreStopIds;

        set<StopIdType> linkedStopIds;
        set<StopIdType> updatedStopIds;

        for(size_t i = 0; i < arrivalUnitCandidates.size(); i++) {
            const pair<TimeType, ArrivalUnit>& arrivalUnitCandidate = arrivalUnitCandidates[i];
            const ArrivalUnit& arrivalUnit = arrivalUnitCandidate.second;
            const map<LineIdType, LineStopInformation>& lineStopInfo = stopDetail.lineStopInfo;

            typedef map<LineIdType, LineStopInformation>::const_iterator LineIter;

            LineIter lineIter = lineStopInfo.find(arrivalUnit.lineId);

            assert(lineIter != lineStopInfo.end());

            const RouteSchedule& routeSchedule =
                lineSchedules[arrivalUnit.lineId].
                routeSchedules.at(arrivalUnit.routeId);

            const vector<StopIdType>& stopIds = routeSchedule.stopIds;

            StopNumberType endStopNumber =
                std::max(0,  arrivalUnit.stopNumber - int(stopIds.size() - 1));

            for(StopNumberType j = arrivalUnit.stopNumber - 1; j >= endStopNumber; j--) {
                const StopIdType& prevStopId = stopIds.at(j % stopIds.size());
                linkedStopIds.insert(prevStopId);
            }
        }
        linkedStopIds.insert(stopDetail.transferStopIds.begin(), stopDetail.transferStopIds.end());

        for(size_t i = 0; i < arrivalUnitCandidates.size(); i++) {
            const pair<TimeType, ArrivalUnit>& arrivalUnitCandidate = arrivalUnitCandidates[i];
            const ArrivalUnit& arrivalUnit = arrivalUnitCandidate.second;
            const map<LineIdType, LineStopInformation>& lineStopInfo = stopDetail.lineStopInfo;

            typedef map<LineIdType, LineStopInformation>::const_iterator LineIter;

            LineIter lineIter = lineStopInfo.find(arrivalUnit.lineId);

            assert(lineIter != lineStopInfo.end());

            const RouteSchedule& routeSchedule =
                lineSchedules[arrivalUnit.lineId].
                routeSchedules.at(arrivalUnit.routeId);

            const VehicleSchedule& vehicleSchedule =
                routeSchedule.vehicleSchedules.at(arrivalUnit.vehicleNumber);

            const vector<pair<TimeType, TimeType> >& timesPerStop =
                vehicleSchedule.timesPerStop;

            const vector<TimeType>& delays = routeSchedule.delays;
            const vector<StopIdType>& stopIds = routeSchedule.stopIds;

            assert(!stopIds.empty());

            StopNumberType endStopNumber = static_cast<StopNumberType>(
                std::max(0,  arrivalUnit.stopNumber - int(stopIds.size() - 1)));

            for(StopNumberType j = arrivalUnit.stopNumber - 1; j >= endStopNumber; j--) {
                StopIdType prevStopId;

                if (routeSchedule.isLoopLine) {
                    prevStopId = vehicleSchedule.stopsPtr->at(j).stopId;
                } else {
                    prevStopId = stopIds.at(j % stopIds.size());
                }

                TimeType arrivalTime = timesPerStop.at(j).first;
                if (enableDynamicRouteSearch) {
                    if (routeSchedule.isLoopLine) {
                        arrivalTime += delays.at(j % (stopIds.size() - 1));
                    } else {
                        arrivalTime += delays.at(j % stopIds.size());
                    }
                }

                if (arrivalTime < earlyDepartureTime) {
                    break;
                }

                RouteSearchStop& prevRouteSearchStop = routeSearchStops[prevStopId];

                if (ignoreStopIds.find(prevStopId) != ignoreStopIds.end()) {
                    continue;
                }

                if (!(*this).IsAvailableLink(resource, prevStopId, stopNode.stopId, arrivalUnit)) {
                    continue;
                }

                const Vertex& prevVertex = (*this).GetStopVertex(prevStopId);
                const double price =
                    routeSchedule.CalculatePrice(prevVertex.DistanceTo(nextVertex), resource.TicketType());

                const double duration = static_cast<double>((destArrivalTime - arrivalTime)/SECOND);
                assert(duration < maxTravelDurationSec);

                double cost;
                double expectedMinCost;

                if (order == AGENT_ROUTE_ORDER_PRICE) {
                    cost = duration + maxTravelDurationSec*(price + routeSearchStop.price);
                    expectedMinCost = cost + maxTravelDurationSec;
                } else if (order == AGENT_ROUTE_ORDER_TIME) {
                    cost = duration;
                    expectedMinCost = cost + prevVertex.DistanceTo(startVertex)/maxVehicleSpeedMetersPerSec;
                } else {
                    assert(order == AGENT_ROUTE_ORDER_TRANSFER_TIME);
                    cost = duration + maxTravelDurationSec*(1 + routeSearchStop.numberTransferTime);
                    expectedMinCost = cost + maxTravelDurationSec;
                }

                if (expectedMinCost < minCost &&
                    duration < maxDuration &&
                    (routeSearchStop.numberTransferTime < numberMaxTransferTimes || prevStopId == startStopId) &&
                    prevRouteSearchStop.IsBetterRoute(cost)) {

                    prevRouteSearchStop.SetBestRoute(stopNode.stopId, cost, duration, arrivalUnit, j, arrivalUnit.stopNumber);

                    if (prevStopId == startStopId) {

                        minCost = cost;

                    } else {
                        prevRouteSearchStop.numberTransferTime = routeSearchStop.numberTransferTime + 1;
                        prevRouteSearchStop.price = price + routeSearchStop.price;

                        StopIdType lastStopId = stopNode.stopId;

                        while (lastStopId != destStopId) {
                            const RouteSearchStop& lastRouteSearchStop = routeSearchStops[lastStopId];
                            const ArrivalUnit& lastRouteArrivalUnit = lastRouteSearchStop.arrivalUnit;

                            prevRouteSearchStop.ignoreLineIds.insert(lastRouteArrivalUnit.lineId);
                            lastStopId = lastRouteSearchStop.trackStopId;

                            assert(lastStopId != stopNode.stopId);
                        }

                        updatedStopIds.insert(prevStopId);
                        stopNodes.push(StopNode(prevStopId, cost, expectedMinCost));
                    }
                }
            }
        }

        if (stopNode.stopId != destStopId &&
            routeSearchStop.numberTransferTime < numberMaxTransferTimes &&
            !routeSearchStop.arrivalUnit.IsTransferLine()) {

            const vector<StopIdType>& transferStopIds = stopDetail.transferStopIds;
            const double walkSpeed = walkSpeedsPerMobilityClass.at(resource.MobilityClass());
            const double marginSec = 10;

            for(size_t i = 0; i < transferStopIds.size(); i++) {
                const StopIdType& transferStopId = transferStopIds.at(i);
                const StopDetail& transferStopDetail = stopDetails[transferStopId];

                if (transferStopId == startStopId) {
                    continue;
                }
                if (ignoreStopIds.find(transferStopId) != ignoreStopIds.end()) {
                    continue;
                }
                if (!IsAvailableVehicleType(transferStopDetail.vehicleType, notAvailableBehavorTypes)) {
                    continue;
                }

                RouteSearchStop& transferRouteSearchStop = routeSearchStops[transferStopId];
                const Vertex& transferVertex = (*this).GetStopVertex(transferStopId);
                const double distance = transferVertex.DistanceTo(nextVertex);
                const double walkDuration = distance/walkSpeed + marginSec;
                const double duration = routeSearchStop.duration + walkDuration;

                double cost = routeSearchStop.cost + walkDuration;
                double expectedMinCost = cost + transferVertex.DistanceTo(startVertex)/maxVehicleSpeedMetersPerSec;

                if (expectedMinCost < minCost &&
                    duration < maxDuration &&
                    transferRouteSearchStop.IsBetterRoute(cost)) {

                    transferRouteSearchStop.SetBestRoute(
                        stopNode.stopId, cost, duration, ArrivalUnit::CreateTransferLineUnit(), NO_STOP_NUMBER, NO_STOP_NUMBER);

                    transferRouteSearchStop.numberTransferTime = routeSearchStop.numberTransferTime;
                    transferRouteSearchStop.price = routeSearchStop.price;

                    StopIdType lastStopId = stopNode.stopId;

                    while (lastStopId != destStopId) {
                        const RouteSearchStop& lastRouteSearchStop = routeSearchStops[lastStopId];
                        const ArrivalUnit& arrivalUnit = lastRouteSearchStop.arrivalUnit;

                        transferRouteSearchStop.ignoreLineIds.insert(arrivalUnit.lineId);
                        lastStopId = lastRouteSearchStop.trackStopId;

                        assert(lastStopId != stopNode.stopId);
                    }

                    updatedStopIds.insert(transferStopId);
                    stopNodes.push(StopNode(transferStopId, cost, expectedMinCost));
                }
            }
        }

        typedef set<StopIdType>::const_iterator IterType;

        set<StopIdType> newIgnoreStopIds;
        newIgnoreStopIds.insert(ignoreStopIds.begin(), ignoreStopIds.end());
        newIgnoreStopIds.insert(linkedStopIds.begin(), linkedStopIds.end());

        for(IterType iter = updatedStopIds.begin(); iter != updatedStopIds.end(); iter++) {
            routeSearchStops[(*iter)].ignoreStopIds = newIgnoreStopIds;
        }
    }

    if (routeSearchStops[startStopId].FoundRoute()) {
        deque<PublicVehicleRoute>& publiceVehicleRoutes = route.publicVehicleRoutes;

        publiceVehicleRoutes.clear();

        StopIdType prevStopId = startStopId;

        TimeType arrivalTime = ZERO_TIME;
        TimeType departureTime;

        minDuration = routeSearchStops[startStopId].duration;

        while (prevStopId != destStopId) {
            const StopIdType lastStopId = prevStopId;
            const RouteSearchStop& routeSearchStop = routeSearchStops[lastStopId];
            const ArrivalUnit& arrivalUnit = routeSearchStop.arrivalUnit;

            if (!arrivalUnit.IsTransferLine()) {
                const RouteSchedule& routeSchedule =
                    lineSchedules[arrivalUnit.lineId].
                    routeSchedules.at(arrivalUnit.routeId);

                const VehicleSchedule& vehicleSchedule =
                    routeSchedule.vehicleSchedules.at(arrivalUnit.vehicleNumber);

                TimeType delay = ZERO_TIME;
                if (enableDynamicRouteSearch) {
                    delay += (*this).GetStopDelay(arrivalUnit);
                }

                if (routeSearchStop.departureStopNumber != NO_STOP_NUMBER) {
                    arrivalTime = vehicleSchedule.timesPerStop.at(arrivalUnit.stopNumber).first + delay;
                }
                if (arrivalUnit.stopNumber != NO_STOP_NUMBER) {
                    departureTime = vehicleSchedule.timesPerStop.at(routeSearchStop.departureStopNumber).second + delay;
                }

            }

            publiceVehicleRoutes.push_back(
                PublicVehicleRoute(
                    arrivalUnit.lineId,
                    lastStopId,
                    routeSearchStop.trackStopId,
                    arrivalUnit.routeId,
                    arrivalUnit.vehicleNumber,
                    arrivalUnit.stopNumber,
                    departureTime,
                    arrivalTime));

            prevStopId = routeSearchStop.trackStopId;
        }

        for(size_t i = 0; i < publiceVehicleRoutes.size(); i++) {
            PublicVehicleRoute& aRoute = publiceVehicleRoutes[i];
            if (aRoute.IsTransferRoute()) {
                assert(i != 0 && i < publiceVehicleRoutes.size() - 1);
                aRoute.departureTime = publiceVehicleRoutes[i-1].arrivalTime;
                aRoute.arrivalTime = publiceVehicleRoutes[i+1].departureTime;
            }
        }

        publiceVehicleRoutes.back().arrivalTime = destArrivalTime;
    }
}


void PublicVehicleTable::GetBetterRouteFromStart(
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
    AgentRoute& route)
{
    assert(startStopId != destStopId);

    (*this).InitializeRouteSearchStop(resource, startStopId);

    const Vertex& destVertex = (*this).GetStopVertex(destStopId);

    double minCost = DBL_MAX;
    priority_queue_stable<StopNode> stopNodes;

    routeSearchStops[startStopId].arrivalUnitCandidates.push_back(make_pair(startDepartureTime, startArrivalUnit));
    routeSearchStops[startStopId].needToRecalculateArrivalUnitCandidates = false;
    routeSearchStops[startStopId].ignoreStopIds.insert(startStopId);

    stopNodes.push(StopNode(startStopId, 0, 0));

    while (!stopNodes.empty()) {
        const StopNode stopNode = stopNodes.top();
        stopNodes.pop();

        RouteSearchStop& routeSearchStop = routeSearchStops[stopNode.stopId];

        if (stopNode.cost > routeSearchStop.cost) {
            continue;
        }

        const TimeType earlyArrivalTime = startDepartureTime + static_cast<TimeType>(routeSearchStop.duration)*SECOND;

        if (routeSearchStop.needToRecalculateArrivalUnitCandidates) {
            (*this).FindDepartureInfos(
                resource,
                stopNode.stopId,
                lateArrivalTime,
                earlyArrivalTime,
                notAvailableBehavorTypes,
                routeSearchStop.ignoreLineIds,
                routeSearchStop.arrivalUnitCandidates);
        }

        const StopDetail& stopDetail = stopDetails[stopNode.stopId];
        const vector<pair<TimeType, ArrivalUnit> >& arrivalUnitCandidates = routeSearchStop.arrivalUnitCandidates;
        const Vertex& prevVertex = (*this).GetStopVertex(stopNode.stopId);
        const set<StopIdType>& ignoreStopIds = routeSearchStop.ignoreStopIds;

        set<StopIdType> linkedStopIds;
        set<StopIdType> updatedStopIds;

        for(size_t i = 0; i < arrivalUnitCandidates.size(); i++) {
            const pair<TimeType, ArrivalUnit>& arrivalUnitCandidate = arrivalUnitCandidates[i];
            const ArrivalUnit& arrivalUnit = arrivalUnitCandidate.second;
            const map<LineIdType, LineStopInformation>& lineStopInfo = stopDetail.lineStopInfo;
            assert(arrivalUnitCandidate.first >= earlyArrivalTime);
            typedef map<LineIdType, LineStopInformation>::const_iterator LineIter;

            LineIter lineIter = lineStopInfo.find(arrivalUnit.lineId);

            assert(lineIter != lineStopInfo.end());

            const RouteSchedule& routeSchedule =
                lineSchedules[arrivalUnit.lineId].
                routeSchedules.at(arrivalUnit.routeId);

            const VehicleSchedule& vehicleSchedule =
                routeSchedule.vehicleSchedules.at(arrivalUnit.vehicleNumber);

            const vector<StopIdType>& stopIds = routeSchedule.stopIds;

            const StopNumberType endStopNumber =
                (StopNumberType)(std::min(arrivalUnit.stopNumber + stopIds.size(),
                         stopIds.size()*(vehicleSchedule.numberCycles + 1)));

            for(StopNumberType j = arrivalUnit.stopNumber + 1; j < endStopNumber; j++) {
                const StopIdType& nextStopId = stopIds.at(j % stopIds.size());
                linkedStopIds.insert(nextStopId);
            }
        }
        linkedStopIds.insert(stopDetail.transferStopIds.begin(), stopDetail.transferStopIds.end());

        for(size_t i = 0; i < arrivalUnitCandidates.size(); i++) {
            const pair<TimeType, ArrivalUnit>& arrivalUnitCandidate = arrivalUnitCandidates[i];
            const ArrivalUnit& arrivalUnit = arrivalUnitCandidate.second;
            const map<LineIdType, LineStopInformation>& lineStopInfo = stopDetail.lineStopInfo;
            assert(arrivalUnitCandidate.first >= earlyArrivalTime);
            typedef map<LineIdType, LineStopInformation>::const_iterator LineIter;

            LineIter lineIter = lineStopInfo.find(arrivalUnit.lineId);

            assert(lineIter != lineStopInfo.end());

            const RouteSchedule& routeSchedule =
                lineSchedules[arrivalUnit.lineId].
                routeSchedules.at(arrivalUnit.routeId);

            const VehicleSchedule& vehicleSchedule =
                routeSchedule.vehicleSchedules.at(arrivalUnit.vehicleNumber);

            const vector<pair<TimeType, TimeType> >& timesPerStop =
                vehicleSchedule.timesPerStop;

            const vector<TimeType>& delays = routeSchedule.delays;
            const vector<StopIdType>& stopIds = routeSchedule.stopIds;

            StopNumberType endStopNumber;

            if (routeSchedule.isLoopLine) {
                assert(!stopIds.empty());

                endStopNumber = static_cast<StopNumberType>(
                    std::min(arrivalUnit.stopNumber + (stopIds.size() - 1),
                             vehicleSchedule.stopsPtr->size()));
            } else {
                endStopNumber = static_cast<StopNumberType>(
                    std::min(arrivalUnit.stopNumber + stopIds.size(),
                             stopIds.size()*(vehicleSchedule.numberCycles + 1)));

            }

            for(StopNumberType j = arrivalUnit.stopNumber + 1; j < endStopNumber; j++) {
                StopIdType nextStopId;

                if (routeSchedule.isLoopLine) {
                    nextStopId = vehicleSchedule.stopsPtr->at(j).stopId;
                } else {
                    nextStopId = stopIds.at(j % stopIds.size());
                }

                if (ignoreStopIds.find(nextStopId) != ignoreStopIds.end()) {
                    continue;
                }

                TimeType arrivalTime = timesPerStop.at(j).first;
                if (enableDynamicRouteSearch) {
                    if (routeSchedule.isLoopLine) {
                        arrivalTime += delays.at(j % (stopIds.size() - 1));
                    } else {
                        arrivalTime += delays.at(j % stopIds.size());
                    }
                }

                RouteSearchStop& nextRouteSearchStop = routeSearchStops[nextStopId];

                if (!(*this).IsAvailableLink(resource, stopNode.stopId, nextStopId, arrivalUnit)) {
                    continue;
                }

                const Vertex& nextVertex = (*this).GetStopVertex(nextStopId);
                const double price =
                    routeSchedule.CalculatePrice(prevVertex.DistanceTo(nextVertex), resource.TicketType());

                const double duration = static_cast<double>((arrivalTime - startDepartureTime)/SECOND);
                assert(duration < maxTravelDurationSec);
                assert(duration > routeSearchStop.duration);

                double cost;
                double expectedMinCost;

                if (order == AGENT_ROUTE_ORDER_PRICE) {
                    cost = duration + maxTravelDurationSec*(price + routeSearchStop.price);
                    expectedMinCost = cost + maxTravelDurationSec;
                } else if (order == AGENT_ROUTE_ORDER_TIME) {
                    cost = duration;
                    expectedMinCost = cost + nextVertex.DistanceTo(destVertex)/maxVehicleSpeedMetersPerSec;
                } else {
                    assert(order == AGENT_ROUTE_ORDER_TRANSFER_TIME);
                    cost = duration + maxTravelDurationSec*(1 + routeSearchStop.numberTransferTime);
                    expectedMinCost = cost + maxTravelDurationSec;
                }

                if (expectedMinCost < minCost &&
                    duration < maxDuration &&
                    (routeSearchStop.numberTransferTime < numberMaxTransferTimes || nextStopId == destStopId) &&
                    nextRouteSearchStop.IsBetterRoute(cost)) {

                    nextRouteSearchStop.SetBestRoute(
                        stopNode.stopId, cost, duration, arrivalUnit, arrivalUnit.stopNumber, j);

                    if (nextStopId == destStopId) {

                        minCost = cost;

                    } else {
                        nextRouteSearchStop.numberTransferTime = routeSearchStop.numberTransferTime + 1;
                        nextRouteSearchStop.price = price + routeSearchStop.price;
                        nextRouteSearchStop.ignoreLineIds.clear();

                        StopIdType lastStopId = stopNode.stopId;

                        while (lastStopId != startStopId) {
                            const RouteSearchStop& lastRouteSearchStop = routeSearchStops[lastStopId];
                            const ArrivalUnit& lastRouteArrivalUnit = lastRouteSearchStop.arrivalUnit;

                            nextRouteSearchStop.ignoreLineIds.insert(lastRouteArrivalUnit.lineId);
                            lastStopId = lastRouteSearchStop.trackStopId;

                            assert(lastStopId != stopNode.stopId);
                        }

                        updatedStopIds.insert(nextStopId);
                        stopNodes.push(StopNode(nextStopId, cost, expectedMinCost));
                    }
                }
            }
        }

        if (stopNode.stopId != startStopId &&
            routeSearchStop.numberTransferTime < numberMaxTransferTimes &&
            !routeSearchStop.arrivalUnit.IsTransferLine()) {

            const vector<StopIdType>& transferStopIds = stopDetail.transferStopIds;
            const double walkSpeed = walkSpeedsPerMobilityClass.at(resource.MobilityClass());
            const double marginSec = 10;

            for(size_t i = 0; i < transferStopIds.size(); i++) {
                const StopIdType& transferStopId = transferStopIds.at(i);
                const StopDetail& transferStopDetail = stopDetails[transferStopId];

                if (transferStopId == destStopId) {
                    continue;
                }
                if (ignoreStopIds.find(transferStopId) != ignoreStopIds.end()) {
                    continue;
                }
                if (!IsAvailableVehicleType(transferStopDetail.vehicleType, notAvailableBehavorTypes)) {
                    continue;
                }

                RouteSearchStop& transferRouteSearchStop = routeSearchStops[transferStopId];
                const Vertex& transferVertex = (*this).GetStopVertex(transferStopId);
                const double distance = prevVertex.DistanceTo(transferVertex);
                const double walkDuration = distance/walkSpeed + marginSec;
                const double duration = routeSearchStop.duration + walkDuration;

                double cost = routeSearchStop.cost + walkDuration;
                double expectedMinCost = cost + transferVertex.DistanceTo(destVertex)/maxVehicleSpeedMetersPerSec;

                if (expectedMinCost < minCost &&
                    duration < maxDuration &&
                    transferRouteSearchStop.IsBetterRoute(cost)) {

                    transferRouteSearchStop.SetBestRoute(
                        stopNode.stopId, cost, duration, ArrivalUnit::CreateTransferLineUnit(), NO_STOP_NUMBER, NO_STOP_NUMBER);

                    transferRouteSearchStop.ignoreLineIds.clear();

                    transferRouteSearchStop.numberTransferTime = routeSearchStop.numberTransferTime;
                    transferRouteSearchStop.price = routeSearchStop.price;

                    StopIdType lastStopId = stopNode.stopId;

                    while (lastStopId != startStopId) {
                        const RouteSearchStop& lastRouteSearchStop = routeSearchStops[lastStopId];
                        const ArrivalUnit& arrivalUnit = lastRouteSearchStop.arrivalUnit;

                        transferRouteSearchStop.ignoreLineIds.insert(arrivalUnit.lineId);
                        lastStopId = lastRouteSearchStop.trackStopId;

                        assert(lastStopId != stopNode.stopId);
                    }

                    updatedStopIds.insert(transferStopId);
                    stopNodes.push(StopNode(transferStopId, cost, expectedMinCost));
                }
            }
        }

        typedef set<StopIdType>::const_iterator IterType;

        set<StopIdType> newIgnoreStopIds;
        newIgnoreStopIds.insert(ignoreStopIds.begin(), ignoreStopIds.end());
        newIgnoreStopIds.insert(linkedStopIds.begin(), linkedStopIds.end());

        for(IterType iter = updatedStopIds.begin(); iter != updatedStopIds.end(); iter++) {
            routeSearchStops[(*iter)].ignoreStopIds = newIgnoreStopIds;
        }
    }

    if (routeSearchStops[destStopId].FoundRoute()) {
        deque<PublicVehicleRoute>& publiceVehicleRoutes = route.publicVehicleRoutes;

        publiceVehicleRoutes.clear();

        StopIdType prevStopId = destStopId;

        TimeType arrivalTime;
        TimeType departureTime;

        minDuration = routeSearchStops[destStopId].duration;

        while (prevStopId != startStopId) {
            const StopIdType lastStopId = prevStopId;
            const RouteSearchStop& routeSearchStop = routeSearchStops[lastStopId];
            const ArrivalUnit& arrivalUnit = routeSearchStop.arrivalUnit;

            if (!arrivalUnit.IsTransferLine()) {
                const RouteSchedule& routeSchedule =
                    lineSchedules[arrivalUnit.lineId].
                    routeSchedules.at(arrivalUnit.routeId);

                const VehicleSchedule& vehicleSchedule =
                    routeSchedule.vehicleSchedules.at(arrivalUnit.vehicleNumber);

                TimeType delay = ZERO_TIME;
                if (enableDynamicRouteSearch) {
                    delay += (*this).GetStopDelay(arrivalUnit);
                }

                if (routeSearchStop.departureStopNumber != NO_STOP_NUMBER) {
                    arrivalTime = vehicleSchedule.timesPerStop.at(arrivalUnit.stopNumber).first + delay;
                }
                if (arrivalUnit.stopNumber != NO_STOP_NUMBER) {
                    departureTime = vehicleSchedule.timesPerStop.at(routeSearchStop.departureStopNumber).second + delay;
                }

            }

            publiceVehicleRoutes.push_front(
                PublicVehicleRoute(
                    arrivalUnit.lineId,
                    routeSearchStop.trackStopId,
                    lastStopId,
                    arrivalUnit.routeId,
                    arrivalUnit.vehicleNumber,
                    arrivalUnit.stopNumber,
                    departureTime,
                    arrivalTime));

            prevStopId = routeSearchStop.trackStopId;
        }

        for(size_t i = 0; i < publiceVehicleRoutes.size(); i++) {
            PublicVehicleRoute& aRoute = publiceVehicleRoutes[i];
            if (aRoute.IsTransferRoute()) {
                assert(i != 0 && i < publiceVehicleRoutes.size() - 1);
                aRoute.departureTime = publiceVehicleRoutes[i-1].arrivalTime;
                aRoute.arrivalTime = publiceVehicleRoutes[i+1].departureTime;
            }
        }

        assert(startDepartureTime < publiceVehicleRoutes.front().arrivalTime);
        publiceVehicleRoutes.front().departureTime = startDepartureTime;
    }
}


bool PublicVehicleTable::IsAvailableLink(
    const AgentResource& resource,
    const StopIdType& stopId,
    const StopIdType& linkedStopId,
    const ArrivalUnit& arrivalUnit) const
{
    if (arrivalUnit.IsTransferLine()) {
        return true;
    }

    const AgentUserType& userType = resource.UserType();

    if (IsUserTypeADisabledPerson(userType)) {

        const VehicleFamilyIdType familyId =
            lineSchedules[arrivalUnit.lineId].routeSchedules.at(arrivalUnit.routeId).
            vehicleSchedules.at(arrivalUnit.vehicleNumber).familyId;

        return vehicleFamilies[familyId].barrierFree;
    }

    return true;
}

void PublicVehicleTable::InitializeRouteSearchStop(
    const AgentResource& resource,
    const StopIdType& startStopId)
{
    for(size_t i = 0; i < routeSearchStops.size(); i++) {
        routeSearchStops[i].Initialize();
    }
    routeSearchStops[startStopId].cost = 0;
}

void PublicVehicleTable::FindStartAndStopIds(
    const Vertex& startVertex,
    const Vertex& destVertex,
    const double maxWalkDistance,
    const size_t numberMaxBusStopCandidatesForStopType,
    const bool searchFromNearStopPair,
    const set<AgentBehaviorType>& notAvailableBehavorTypes,
    vector<pair<StopIdType, StopIdType> >& startAndDestStopIds) const
{
    startAndDestStopIds.clear();

    if (stopDetails.empty()) {
        return;
    }

    multimap<double, pair<double, StopIdType> > startStopIdsPerDistance;
    multimap<double, pair<double, StopIdType> > destStopIdsPerDistance;

    vector<VariantIdType> startStopNumbers;
    vector<VariantIdType> destStopNumbers;

    stopIdMap.GetGisObject(Rectangle(startVertex, maxWalkDistance), startStopNumbers);
    stopIdMap.GetGisObject(Rectangle(destVertex, maxWalkDistance), destStopNumbers);

    const bool stationIsAvailable =
        (notAvailableBehavorTypes.find(AGENT_BEHAVIOR_TRAIN) == notAvailableBehavorTypes.end());

    const bool busStopIsAvailable =
        (notAvailableBehavorTypes.find(AGENT_BEHAVIOR_BUS) == notAvailableBehavorTypes.end());

    typedef vector<StopIdType>::const_iterator StopIter;

    for(StopIter iter = startStopNumbers.begin(); iter != startStopNumbers.end(); iter++) {
        const StopIdType& stopId = *iter;
        const Vertex& stopVertex = (*this).GetStopVertex(stopId);
        const double distance = startVertex.DistanceTo(stopVertex);

        if (distance < maxWalkDistance) {
            startStopIdsPerDistance.insert(make_pair(distance, make_pair(destVertex.DistanceTo(stopVertex), stopId)));
        }
    }
    for(StopIter iter = destStopNumbers.begin(); iter != destStopNumbers.end(); iter++) {
        const StopIdType& stopId = *iter;
        const Vertex& stopVertex = (*this).GetStopVertex(stopId);
        const double distance = destVertex.DistanceTo(stopVertex);

        if (distance < maxWalkDistance) {
            destStopIdsPerDistance.insert(make_pair(distance, make_pair(startVertex.DistanceTo(stopVertex), stopId)));
        }
    }

    typedef multimap<double, pair<double, StopIdType> >::const_iterator DistanceIter;

    size_t numberStartStations = 0;
    size_t numberStartBusStops = 0;
    size_t numberDestStations = 0;
    size_t numberDestBusStops = 0;

    vector<pair<double, pair<double, StopIdType> > > startStopIdInfos;
    vector<pair<double, pair<double, StopIdType> > > destStopIdInfos;

    for(DistanceIter startIter = startStopIdsPerDistance.begin(); startIter != startStopIdsPerDistance.end(); startIter++) {
        const StopIdType& startStopId = (*startIter).second.second;

        if (stopDetails[startStopId].IsStation()) {
            if (stationIsAvailable &&
                numberStartStations < numberMaxBusStopCandidatesForStopType) {

                startStopIdInfos.push_back(make_pair((*startIter).first, (*startIter).second));
                numberStartStations++;
            }
        } else {
            if (busStopIsAvailable &&
                numberStartBusStops < numberMaxBusStopCandidatesForStopType) {

                startStopIdInfos.push_back(make_pair((*startIter).first, (*startIter).second));
                numberStartBusStops++;
            }
        }

        if (numberStartStations >= numberMaxBusStopCandidatesForStopType &&
            numberStartBusStops >= numberMaxBusStopCandidatesForStopType) {
            break;
        }
    }
    for(DistanceIter destIter = destStopIdsPerDistance.begin(); destIter != destStopIdsPerDistance.end(); destIter++) {
        const StopIdType& destStopId = (*destIter).second.second;

        if (stopDetails[destStopId].IsStation()) {
            if (stationIsAvailable &&
                numberDestStations < numberMaxBusStopCandidatesForStopType) {

                destStopIdInfos.push_back(make_pair((*destIter).first, (*destIter).second));
                numberDestStations++;
            }
        } else {
            if (busStopIsAvailable &&
                numberDestBusStops < numberMaxBusStopCandidatesForStopType) {

                destStopIdInfos.push_back(make_pair((*destIter).first, (*destIter).second));
                numberDestBusStops++;
            }
        }

        if (numberDestStations >= numberMaxBusStopCandidatesForStopType &&
            numberDestBusStops >= numberMaxBusStopCandidatesForStopType) {
            break;
        }
    }

    const double startToDestDistance = startVertex.DistanceTo(destVertex);

    for(size_t i = 0; i < startStopIdInfos.size(); i++) {
        const pair<double, pair<double, StopIdType> >& startStopIdInfo = startStopIdInfos[i];
        const double startToStartStopDistance = startStopIdInfo.first;
        const double startToDestStopDistance = startStopIdInfo.second.first;
        const StopIdType& startStopId = startStopIdInfo.second.second;

        for(size_t j = 0; j < destStopIdInfos.size(); j++) {
            const pair<double, pair<double, StopIdType> >& destStopIdInfo = destStopIdInfos[j];
            const double destToDestStopDistance = destStopIdInfo.first;
            const double destToStartStopDistance = destStopIdInfo.second.first;
            const StopIdType& destStopId = destStopIdInfo.second.second;

            if (startStopId != destStopId) {
                if (searchFromNearStopPair) {
                    if (startToStartStopDistance < destToStartStopDistance &&
                        destToDestStopDistance < startToDestStopDistance &&
                        (startToStartStopDistance + destToDestStopDistance) < startToDestDistance) {

                        startAndDestStopIds.push_back(make_pair(startStopId, destStopId));
                    }
                } else {
                    startAndDestStopIds.push_back(make_pair(startStopId, destStopId));
                }
            }
        }
    }
}

Vertex PublicVehicleTable::GetStopVertex(const StopIdType& stopId) const
{
    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();
    const StopDetail& stopDetail = stopDetails[stopId];

    if (stopDetail.IsStation()) {
        return subsystem.GetStation(stopDetail.stationOrBusStopId).GetVertex();
    }

    return subsystem.GetBusStop(stopDetail.stationOrBusStopId).GetVertex();
}

VertexIdType PublicVehicleTable::GetOrigVertexId(
    const PublicVehicleRoute& route) const
{
    return (*this).GetLineVertexId(route.origStopId, route.lineId, route.routeId);
}

VertexIdType PublicVehicleTable::GetDestVertexId(
    const PublicVehicleRoute& route) const
{
    return (*this).GetLineVertexId(route.nextStopId, route.lineId, route.routeId);
}

VertexIdType PublicVehicleTable::GetLineVertexId(
    const StopIdType& stopId,
    const LineIdType& lineId,
    const RouteIdType& routeId) const
{
    const StopDetail& stopDetail = stopDetails[stopId];

    assert(lineId != TRANSFER_LINE_ID);

    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();

    if (stopDetail.IsStation()) {
        return subsystem.GetStation(stopDetail.stationOrBusStopId).GetLineVertexId(
            railRoadLineIdPerLine.at(lineId), routeId);
    } else {

        return subsystem.GetBusStop(stopDetail.stationOrBusStopId).GetLineVertexId(
            busLineIdPerLine.at(lineId), routeId);
    }
}

VertexIdType PublicVehicleTable::GetOrigEntranceVertexId(
    const PublicVehicleRoute& route,
    const Vertex& position) const
{
    const StopDetail& stopDetail = stopDetails[route.origStopId];

    if (stopDetail.IsStation()) {

        return (*this).GetOrigVertexId(route);

    } else {

        return (*this).GetNearestEntranceVertexId(route.origStopId, position);
    }
}

bool PublicVehicleTable::IsVertexOfStop(
    const StopIdType& stopId,
    const VertexIdType& vertexId)  const
{
    const StopDetail& stopDetail = stopDetails[stopId];

    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();

    if (stopDetail.IsStation()) {
        return subsystem.GetStation(stopDetail.stationOrBusStopId).IsStationVertex(vertexId);
    } else {
        return subsystem.GetBusStop(stopDetail.stationOrBusStopId).IsBusStopVertex(vertexId);
    }

}

VertexIdType PublicVehicleTable::GetNearestEntranceVertexId(
    const StopIdType& stopId,
    const Vertex& position) const
{
    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();
    const StopDetail& stopDetail = stopDetails[stopId];

    if (stopDetail.IsStation()) {
        return subsystem.GetStation(stopDetail.stationOrBusStopId).
            GetNearestEntranceVertexId(position);
    }

    return subsystem.GetBusStop(stopDetail.stationOrBusStopId).GetNearestEntranceVertexId(position);
}

const vector<VertexIdType>& PublicVehicleTable::GetEntranceVertexIds(const StopIdType& stopId) const
{
    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();
    const StopDetail& stopDetail = stopDetails[stopId];

    if (stopDetail.IsStation()) {
        return subsystem.GetStation(stopDetail.stationOrBusStopId).GetVertexIds();
    }

    return subsystem.GetBusStop(stopDetail.stationOrBusStopId).GetVertexIds();
}

RailRoadStationIdType PublicVehicleTable::GetStationId(const StopIdType& stopId) const
{
    const StopDetail& stopDetail = stopDetails[stopId];

    assert(stopDetail.IsStation());

    return stopDetail.stationOrBusStopId;
}

BusStopIdType PublicVehicleTable::GetBusStopId(const StopIdType& stopId) const
{
    const StopDetail& stopDetail = stopDetails[stopId];

    assert(stopDetail.IsBusStop());

    return stopDetail.stationOrBusStopId;
}

bool PublicVehicleTable::IsStation(const StopIdType& stopId) const
{
    const StopDetail& stopDetail = stopDetails[stopId];

    return (stopDetail.IsStation());
}

bool PublicVehicleTable::IsBusStop(const StopIdType& stopId) const
{
    const StopDetail& stopDetail = stopDetails[stopId];

    return (stopDetail.IsBusStop());
}

GisPositionIdType PublicVehicleTable::GetPositionId(const StopIdType& stopId) const
{
    const StopDetail& stopDetail = stopDetails[stopId];

    if (stopDetail.IsStation()) {
        return GisPositionIdType(GIS_RAILROAD_STATION, stopDetail.stationOrBusStopId);
    } else {
        return GisPositionIdType(GIS_BUSSTOP, stopDetail.stationOrBusStopId);
    }
}

double PublicVehicleTable::CalculateCongestion(const StopIdType& stopId) const
{
    const StopDetail& stopDetail = stopDetails[stopId];

    if (stopDetail.IsStation()) {
        const AgentStation& station = theAgentGisPtr->GetAgentStation(stopDetail.stationOrBusStopId);

        return station.CalculatePeopleCongestion();

    } else {
        const AgentBusStop& busStop = theAgentGisPtr->GetAgentBusStop(stopDetail.stationOrBusStopId);

        return busStop.CalculatePeopleCongestion();
    }
}

string PublicVehicleTable::GetStopName(const StopIdType& stopId) const
{
    const StopDetail& stopDetail = stopDetails[stopId];

    if (stopDetail.IsStation()) {
        const AgentStation& station = theAgentGisPtr->GetAgentStation(stopDetail.stationOrBusStopId);

        return station.ref.GetObjectName();

    } else {
        const AgentBusStop& busStop = theAgentGisPtr->GetAgentBusStop(stopDetail.stationOrBusStopId);

        return busStop.ref.GetObjectName();
    }
}

TimeType PublicVehicleTable::GetStopDelay(
    const LineIdType& lineId,
    const RouteIdType& routeId,
    const StopNumberType& stopNumber) const
{
    const vector<TimeType>& delays = lineSchedules[lineId].routeSchedules.at(routeId).delays;

    if (stopNumber < StopNumberType(delays.size())) {
        return delays.at(stopNumber);
    }

    return ZERO_TIME;
}

TimeType PublicVehicleTable::GetVehicleDelay(
    const LineIdType& lineId,
    const RouteIdType& routeId,
    const VehicleNumberType& vehicleNumber) const
{
    return lineSchedules[lineId].routeSchedules.at(routeId).vehicleSchedules.at(vehicleNumber).currentDelay;
}

TimeType PublicVehicleTable::GetStopDelay(
    const ArrivalUnit& arrivalUnit) const
{
    return (*this).GetStopDelay(
        arrivalUnit.lineId, arrivalUnit.routeId, arrivalUnit.stopNumber);
}

TimeType PublicVehicleTable::GetScheduledArrivalTime(
    const LineIdType& lineId,
    const RouteIdType& routeId,
    const VehicleNumberType& vehicleNumber,
    const StopIdType& stopId,
    const TimeType& currentTime) const
{
    const map<StopIdType, set<TimeType> >& arrivalTimesPerStop =
        lineSchedules[lineId].routeSchedules.at(routeId).
        vehicleSchedules.at(vehicleNumber).arrivalTimes;

    typedef map<StopIdType, set<TimeType> >::const_iterator StopIter;

    StopIter stopIter = arrivalTimesPerStop.find(stopId);

    assert(stopIter != arrivalTimesPerStop.end());

    const TimeType delay = (*this).GetStopDelay(lineId, routeId, vehicleNumber);

    const set<TimeType>& arrivalTimes = (*stopIter).second;

    assert(!arrivalTimes.empty());

    typedef set<TimeType>::const_iterator TimeIter;

    TimeIter timeIter;

    if (arrivalTimes.size() == 1) {

        timeIter = arrivalTimes.begin();

    } else {

        const TimeType searchtime = currentTime - delay;

        timeIter = arrivalTimes.lower_bound(searchtime);

        if (timeIter != arrivalTimes.end()) {

            if ((*timeIter) < searchtime) {
                timeIter++;
            }
        }

        if (timeIter == arrivalTimes.end()) {
            timeIter--;
        }

        assert(timeIter != arrivalTimes.end());
    }

    return (*timeIter) + delay;
}

void PublicVehicleTable::SetStopDelay(
    const LineIdType& lineId,
    const RouteIdType& routeId,
    const StopNumberType& stopNumber,
    const TimeType& delay)
{
    vector<TimeType>& delays = lineSchedules[lineId].routeSchedules.at(routeId).delays;

    if (stopNumber < StopNumberType(delays.size())) {
        delays.at(stopNumber) = delay;
    }
}

void PublicVehicleTable::SetVehicleDelay(
    const LineIdType& lineId,
    const RouteIdType& routeId,
    const VehicleNumberType& vehicleNumber,
    const TimeType& delay)
{
    lineSchedules[lineId].routeSchedules.at(routeId).vehicleSchedules.at(vehicleNumber).currentDelay = delay;
}

bool PublicVehicleTable::ContainsLine(const string& lineName) const
{
    return lineSchedules.Contains(lineName);
}

LineIdType PublicVehicleTable::GetLineId(const string& lineName) const
{
    if (!lineSchedules.Contains(lineName)) {
        cerr << "Error: no line name " << lineName << endl;
        exit(1);
    }

    return lineSchedules.GetId(lineName);
}

string PublicVehicleTable::GetLineName(const LineIdType& lineId) const
{
    return lineSchedules.GetLabel(lineId);
}

StopIdType PublicVehicleTable::GetStopId(const LineIdType& lineId, const string& stopName) const
{
    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();
    const LineSchedule& lineSchedule = lineSchedules[lineId];

    GisPositionIdType positioniId;

    if (lineSchedule.vehicleType == VEHICLE_TRAIN) {
        positioniId = subsystem.GetPosition(stopName, GIS_RAILROAD_STATION);
    } else {
        positioniId = subsystem.GetPosition(stopName, GIS_BUSSTOP);
    }

    const string lineStopName =
            ConvertToString(lineSchedule.vehicleType) + "_" + ConvertToString(positioniId.id);

    if (!stopDetails.Contains(lineStopName)) {
        cerr << "Error : no stop " << stopName << endl;
        exit(1);
    }

    const StopDetail& stopDetail = stopDetails[lineStopName];
    const map<LineIdType, LineStopInformation>& lineStopInfo =
        stopDetail.lineStopInfo;

    if (lineStopInfo.find(lineId) == lineStopInfo.end()) {
        cerr << "Error : line " << lineSchedules.GetLabel(lineId) << " doesn't contain " << stopName << endl;
        exit(1);
    }

    return stopDetails.GetId(lineStopName);
}

RouteIdType PublicVehicleTable::GetShortestRouteId(
    const LineIdType& lineId,
    const StopIdType& departureStopId,
    const StopIdType& arrivalStopId) const
{
    RouteIdType shortestRouteId = INVALID_ROUTE_ID;
    TimeType minTravelDuration = INFINITE_TIME;

    const vector<RouteSchedule>& routeSchedules =
        lineSchedules[lineId].routeSchedules;

    for(RouteIdType routeId = 0; routeId < int(routeSchedules.size()); routeId++) {
        const RouteSchedule& routeSchedule = routeSchedules[routeId];
        const vector<StopIdType>& stopIds = routeSchedule.stopIds;

        size_t stopNumber = 0;
        size_t departureStopNumber = 0;
        size_t arrivalStopNumber = 0;

        for(; stopNumber < stopIds.size(); stopNumber++) {
            if (stopIds[stopNumber] == departureStopId) {
                departureStopNumber = stopNumber;
                break;
            }
        }
        for(; stopNumber < stopIds.size(); stopNumber++) {
            if (stopIds[stopNumber] == arrivalStopId) {
                arrivalStopNumber = stopNumber;
                break;
            }
        }

        if (departureStopNumber < arrivalStopNumber) {

            const vector<VehicleSchedule>& vehicleSchedules = routeSchedule.vehicleSchedules;

            if (vehicleSchedules.empty() &&
                shortestRouteId == INVALID_ROUTE_ID) {

                minTravelDuration = INFINITE_TIME;
                shortestRouteId = routeId;

            } else {
                for(size_t i = 0; i < vehicleSchedules.size(); i++) {
                    const vector<pair<TimeType, TimeType> >& timesPerStop = vehicleSchedules[i].timesPerStop;
                    const TimeType departureTime = timesPerStop.at(departureStopNumber).second;
                    const TimeType arrivalTime = timesPerStop.at(arrivalStopNumber).first;
                    const TimeType tranvelDuration = arrivalTime - departureTime;

                    if (tranvelDuration < minTravelDuration) {
                        minTravelDuration = tranvelDuration;
                        shortestRouteId = routeId;
                    }
                }
            }
        }
    }

    if (shortestRouteId == INVALID_ROUTE_ID) {
        cerr << "Line: [" << lineSchedules.GetLabel(lineId)
             << "] doesn't have a route from [" << (*this).GetStopName(departureStopId)
             << "] to [" << (*this).GetStopName(arrivalStopId)
             << "] to insert an extra bus." << endl;
        exit(1);
    }

    return shortestRouteId;
}


VehicleFamilyIdType PublicVehicleTable::GetSuitableVehicleFamilyId(const size_t numberReservations) const
{
    typedef map<size_t, VehicleFamilyIdType>::const_iterator IterType;

    if (busFamilyIds.empty()) {
        cerr << "Error: no vehicle family for bus" << endl;
        exit(1);
    }

    IterType iter = busFamilyIds.lower_bound(numberReservations);

    if (iter == busFamilyIds.end()) {
        iter--;
    }

    return (*iter).second;
}

bool PublicVehicleTable::IsLineAvailable(const LineIdType& lineId) const
{
    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();
    const LineSchedule& lineSchedule = lineSchedules[lineId];

    if (lineSchedule.vehicleType == VEHICLE_TRAIN) {
        return subsystem.GetRailRoadLayerPtr()->IsRailRoadLineAvailable(railRoadLineIdPerLine.at(lineId));
    }

    return true;
}

//-----------------------------------------------------------------------

}//namespace ScenSim
