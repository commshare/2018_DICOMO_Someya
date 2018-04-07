// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef MULTIAGENT_COMMON_H
#define MULTIAGENT_COMMON_H

#include "scensim_nodeid.h"
#include "scensim_engine.h"
#include "scensim_gis.h"
#include "boost/thread.hpp"
#include "scensim_netsim.h"
#include "scensim_application.h"
#include <deque>
#include <string>
#include <cstdlib>
#include <sstream>
#include <list>
#include <vector>
#include <map>
#include <queue>

namespace MultiAgent {

using std::cerr;
using std::cout;
using std::endl;
using std::shared_ptr;
using std::unique_ptr;
using std::move;
using std::string;
using std::ifstream;
using std::ofstream;
using std::ostringstream;

using std::list;
using std::vector;
using std::map;
using std::multimap;
using std::pair;
using std::queue;
using std::deque;
using std::priority_queue;
using ScenSim::priority_queue_stable;

using ScenSim::TimeType;
using ScenSim::SECOND;
using ScenSim::MILLI_SECOND;
using ScenSim::ZERO_TIME;
using ScenSim::INFINITE_TIME;

using ScenSim::RealStatistic;
using ScenSim::CounterStatistic;
using ScenSim::ObjectMobilityModel;
using ScenSim::ObjectMobilityPosition;

using ScenSim::NetworkAddress;
using ScenSim::NodeIdType;
using ScenSim::InterfaceOrInstanceIdType;
using ScenSim::nullInstanceId;

using ScenSim::HighQualityRandomNumberGenerator;
using ScenSim::RandomNumberGeneratorSeedType;
using ScenSim::HashInputsToMakeSeed;
using ScenSim::GisSubsystem;

using ScenSim::GisPositionIdType;
using ScenSim::InvalidGisPositionId;
using ScenSim::PointIdType;
using ScenSim::IntersectionIdType;
using ScenSim::PedestrianPathIdType;
using ScenSim::RoadIdType;
using ScenSim::BusStopIdType;
using ScenSim::RailRoadIdType;
using ScenSim::RailRoadIntersectionIdType;
using ScenSim::RailRoadStationIdType;
using ScenSim::AreaIdType;
using ScenSim::ParkIdType;
using ScenSim::WallIdType;
using ScenSim::BuildingIdType;
using ScenSim::PoiIdType;
using ScenSim::AreaIdType;
using ScenSim::EntranceIdType;
using ScenSim::TrafficLightIdType;
using ScenSim::GenericGisObjectIdType;
using ScenSim::VertexIdType;
using ScenSim::RailRoadLineIdType;
using ScenSim::BusLineIdType;
using ScenSim::RouteIdType;
using ScenSim::VariantIdType;
using ScenSim::GisObjectType;
using ScenSim::GisObjectIdType;

using ScenSim::INVALID_ROUTE_ID;
using ScenSim::INVALID_VERTEX_ID;
using ScenSim::INVALID_VARIANT_ID;

using ScenSim::GIS_AREA;
using ScenSim::GIS_POINT;
using ScenSim::GIS_ROAD;
using ScenSim::GIS_INTERSECTION;
using ScenSim::GIS_RAILROAD;
using ScenSim::GIS_RAILROAD_INTERSECTION;
using ScenSim::GIS_RAILROAD_STATION;
using ScenSim::GIS_PARK;
using ScenSim::GIS_WALL;
using ScenSim::GIS_BUILDING;
using ScenSim::GIS_PEDESTRIAN_PATH;
using ScenSim::GIS_BUSSTOP;
using ScenSim::GIS_TRAFFICLIGHT;
using ScenSim::GIS_ENTRANCE;
using ScenSim::GIS_VEHICLE_LINE;
using ScenSim::GIS_GENERIC_START;
using ScenSim::GIS_POI;

using ScenSim::VertexConnection;
using ScenSim::GisVertex;
using ScenSim::Vertex;
using ScenSim::Rectangle;

using ScenSim::GisObject;
using ScenSim::Point;
using ScenSim::Road;
using ScenSim::RoadTurnType;
using ScenSim::TrafficLightType;
using ScenSim::TrafficLight;
using ScenSim::Intersection;
using ScenSim::BusStop;
using ScenSim::RailRoad;
using ScenSim::RailRoadIntersection;
using ScenSim::RailRoadStation;
using ScenSim::Area;
using ScenSim::Park;
using ScenSim::Wall;
using ScenSim::Building;
using ScenSim::Poi;
using ScenSim::GenericGisObject;
using ScenSim::PoiLayer;
using ScenSim::RoadLayer;
using ScenSim::RailRoadLayer;
using ScenSim::AreaLayer;
using ScenSim::ParkLayer;
using ScenSim::BuildingLayer;

using ScenSim::RoadDirectionType;
using ScenSim::ROAD_DIRECTION_UP;
using ScenSim::ROAD_DIRECTION_DOWN;
using ScenSim::NUMBER_ROAD_DIRECTIONS;

using ScenSim::ROAD_TURN_STRAIGHT;
using ScenSim::ROAD_TURN_BACK;
using ScenSim::ROAD_TURN_RIGHT;
using ScenSim::ROAD_TURN_LEFT;
using ScenSim::ROAD_TURN_NONE;

using ScenSim::TRAFFIC_LIGHT_RED;
using ScenSim::TRAFFIC_LIGHT_GREEN;
using ScenSim::TRAFFIC_LIGHT_YELLOW;

using ScenSim::SpatialObjectMap;

using ScenSim::TraceSubsystem;
using ScenSim::StateTraceRecord;
using ScenSim::TraceMas;
using ScenSim::BAD_SIZE_T;
using ScenSim::PI;
using ScenSim::INVALID_NODEID;
using ScenSim::ANY_NODEID;
using ScenSim::InterfaceIdType;
using ScenSim::NetworkNode;
using ScenSim::GlobalNetworkingObjectBag;

using ScenSim::ParameterDatabaseReader;
using ScenSim::SimulationEngine;
using ScenSim::SimulationEngineInterface;


using ScenSim::MakeLowerCaseString;
using ScenSim::ConvertTimeToStringSecs;
using ScenSim::ConvertToString;
using ScenSim::TrimmedString;
using ScenSim::DeleteTrailingSpaces;
using ScenSim::IsAConfigFileCommentLine;
using ScenSim::ConvertStringToLowerCase;
using ScenSim::TokenizeToTrimmedLowerString;
using ScenSim::ConvertStringToInt;
using ScenSim::ConvertTimeToDoubleSecs;
using ScenSim::ConvertStringToDouble;
using ScenSim::ConvertStringToTime;
using ScenSim::ConvertAStringIntoAVectorOfStrings;
using ScenSim::App_ConvertStringToNodeIdOrAnyNodeId;

using ScenSim::InorderFileCache;
using ScenSim::TraceFileMobilityModel;

using ScenSim::PI;

class MultiAgentSimulator;
class MultiAgentGis;
class AgentRouteSearchSubsystem;

class AgentBehavior;
class VehicleDriverBehavior;
class BusDriverBehavior;
class TaxiDriverBehavior;
class PublicVehicleBehavior;

class AgentPoint;
class AgentRoad;
class AgentIntersection;
//class AgentBuilding;
class AgentBusStop;

class Train;
class Bus;
class Vehicle;
class Taxi;
class VehicleNode;
class PublicVehicle;
class AgentCommunicationNode;

class MultiAgentSimulator;

class AgentTask;
class AgentProfile;
class AgentTaskTable;
class PublicVehicleTable;

typedef int32_t AgentProfileType;
typedef uint32_t AgentIdType;
typedef int32_t PublicVehicleIdType;
typedef int32_t AgentTaxiCenterIdType;

const AgentIdType MASTER_ANY_AGENT_ID = 0;
const AgentIdType MASTER_BUS_AGENT_ID = static_cast<AgentIdType>(-1);
const AgentIdType MASTER_TAXI_AGENT_ID = static_cast<AgentIdType>(-2);

const AgentTaxiCenterIdType INVALID_TAXICENTER_ID = -1;
const AgentTaxiCenterIdType ANY_TAXICENTER_ID = -2;

typedef double AgentCharactorIdType;

const AgentProfileType INVALID_AGENT_TYPE = -1;
const AgentProfileType TRAIN_AGENT_TYPE = -2;
const AgentProfileType BUS_AGENT_TYPE = -3;

const double DEFAULT_SEEING_PEDESTRIAN_PROBABILITY = 1.0;
const double DEFAULT_VEHICLE_LENGTH_METERS = 5;
const double DEFAULT_MAX_VEHICLE_SPEED = 60 / 3.6;
const double DEFAULT_LANE_CHANGE_ACCELERATION_THRESHOLD = 0.2;
const double DEFAULT_MAX_ACCELERATION = 1.0;
const double DEFAULT_MAX_DECELERATION = -3.0;
const double DEFAULT_MIN_VEHICLE_GAP = 3.0;
const double DEFAULT_TIME_HEADWAY = 1.5;
const double DEFAULT_PASSIVE_YIELD_TIME = 3.0;
const double DEFAULT_ACTIVE_YIELD_TIME = -3.0;
const double DEFAULT_VELOCITY_RATIO_GAP_DISTANCE = 0.0;//s1 of IDM
const double DEFAULT_OTHER_VEHICLE_ENATRANCE_TIME = 1.0;
const double DEFAULT_YIELD_WAITING_TIME = 2.0;
const double DEFAULT_GASOLIN_COST = 0.0;
const double DEFAULT_ACCEPTABLE_WALK_DISTANCE_TO_VEHICLE = 1000.;
const double DEFAULT_ACCEPTABLE_WALK_DISTANCE_TO_STOP = 1000.;
const double DEFAULT_MIN_VEHICLE_ROUTE_DISTANCE = 0.;
const double DEFAULT_NUMBER_MAX_ROUTE_CANDIDATES = 3.;
const double DEFAULT_NUMBER_PEOPLE = 1.;
const double DEFAULT_ENTRANCE_WAIT_TIME = 3600.0;
const double DEFAULT_VEHICLE_ENTRANCE_WAIT_TIME = 1.0;
const double DEFAULT_TAXICALL_WAIT_TIME = 600.0;
const double DEFAULT_MAX_BRAKING_DECCELERATION = -20;
const double DEFAULT_ACCELERATION_EXPONENT = 4.;
const double DEFAULT_SAVE_DECELERATION = -12.;
const double DEFAULT_MAX_TURN_SPEED = 10. / 3.6;// to m/s

const size_t MASTER_THREAD_NUMBER = 0;
const size_t NUMBER_MAX_THREADS = sizeof(uint32_t)*8;

const GisPositionIdType UNREACHABLE_POSITION_ID = GisPositionIdType();

enum VehicleType {
    VEHICLE_NONE,

    VEHICLE_TRAIN,
    VEHICLE_BUS,
    VEHICLE_CAR,
    VEHICLE_TAXI,

    NUMBER_VEHICLE_TYPES,
};

typedef int16_t VehicleNumberType;
const VehicleNumberType NO_VEHICLE_NUMBER = -1;

enum AgentLocationType {
    AGENT_LOCATION_NONE,
    AGENT_LOCATION_POI,
    AGENT_LOCATION_NEAR_POI,
    AGENT_LOCATION_RANDOM_BUILDING,
    AGENT_LOCATION_RANDOM_PARK,
    AGENT_LOCATION_RANDOM_INTERSECTION,
    AGENT_LOCATION_RANDOM_POI,
    AGENT_LOCATION_HOME,
    AGNET_LOCATION_AREA,
    AGNET_LOCATION_POSITION_FILE,
};

typedef int8_t AgentBehaviorType;
enum {
    AGENT_BEHAVIOR_NOTHING = 0,

    AGENT_BEHAVIOR_WAIT = 1,
    AGENT_BEHAVIOR_FREEWALK = 2,
    AGENT_BEHAVIOR_PEDESTRIAN = 3,
    AGENT_BEHAVIOR_BICYCLE = 4,
    AGENT_BEHAVIOR_VEHICLE = 5,
    AGENT_BEHAVIOR_BUS = 6,
    AGENT_BEHAVIOR_TRAIN = 7,
    AGENT_BEHAVIOR_TAXI = 8,

    AGENT_BEHAVIOR_AIRPLANE = 10,

    AGENT_BEHAVIOR_BUS_DRIVER = 11,
    AGENT_BEHAVIOR_TAXI_DRIVER = 12,
    AGENT_BEHAVIOR_TRAIN_DRIVER = 13,

    NUMBER_AGENT_BEHAVIORS,

    AGENT_BEHAVIOR_ANY,
};

static inline
bool IsFreeWalkPosition(const GisObjectType& gisType) {
    return (gisType == GIS_BUILDING || gisType == GIS_PARK || gisType == GIS_POI);
}

typedef int8_t AgentUserType;
enum {
    AGENT_USER_TYPE_NONE = 0,
};

typedef int8_t AgentMobilityClassType;
enum {
    AGENT_MOBILTY_CLASS_NORMAL = 0,
};

typedef int8_t AgentTicketType;
enum {
    AGENT_TICKET_FULL_FARE = 0,
};

template <typename T>
class TraceValue {
public:
    TraceValue(const T& initValue = T())
        :
        value(initValue),
        hasChanged(true)
    {}

    const T& operator*() { return value; }

    const T& GetValueAndUnsetChangeFlag() {
        hasChanged = false;
        return value;
    }

    void UnsetChangeFlag() {
        hasChanged = false;
    }

    void SetValue(const T& initValue) {
        if (value != initValue) {
            value = initValue;
            hasChanged = true;
        }
    }

    bool HasChanged() const { return hasChanged; }

private:
    T value;
    bool hasChanged;
};

static inline
bool IsBehaviorOnRoad(const AgentBehaviorType& behavior)
{
    return (behavior == AGENT_BEHAVIOR_ANY ||
            behavior == AGENT_BEHAVIOR_PEDESTRIAN ||
            behavior == AGENT_BEHAVIOR_BICYCLE ||
            behavior == AGENT_BEHAVIOR_VEHICLE ||
            behavior == AGENT_BEHAVIOR_TAXI);
}

static inline
bool IsBehaviorAPublicVehicle(const AgentBehaviorType& behavior)
{
    return (behavior == AGENT_BEHAVIOR_ANY ||
            behavior == AGENT_BEHAVIOR_BUS ||
            behavior == AGENT_BEHAVIOR_TRAIN);
}

typedef int LineIdType;
const LineIdType INVALID_LINE_ID = -1;
const LineIdType TRANSFER_LINE_ID = -2;

enum LineType {
    LINE_NORMAL,
};

typedef VariantIdType StopIdType;
const StopIdType INVALID_STOP_ID = INVALID_VARIANT_ID;

typedef int StopNumberType;
const StopNumberType NO_STOP_NUMBER = -1;

struct PublicVehicleRoute {
    LineIdType lineId;
    StopIdType origStopId;
    StopIdType nextStopId;

    RouteIdType routeId;
    VehicleNumberType vehicleNumber;
    StopNumberType stopNumber;

    TimeType departureTime;
    TimeType arrivalTime;

    double price;

    PublicVehicleRoute(
        const LineIdType& initLineId,
        const StopIdType& initOrigStopId,
        const StopIdType& initNextStopId,
        const RouteIdType& initRouteId,
        const VehicleNumberType& initVehicleNumber,
        const StopNumberType& initStopNumber,
        const TimeType& initDepartureTime,
        const TimeType& initArrivalTime)
        :
        lineId(initLineId),
        origStopId(initOrigStopId),
        nextStopId(initNextStopId),
        routeId(initRouteId),
        vehicleNumber(initVehicleNumber),
        stopNumber(initStopNumber),
        departureTime(initDepartureTime),
        arrivalTime(initArrivalTime),
        price(0.)
    {}

    bool IsTransferRoute() const { return lineId == TRANSFER_LINE_ID; }
};

const size_t NUMBER_ROUTE_ROAD_TOPOLOCIES = 1;

struct TimeToSearchRoute {
    bool specifiedArrivalTime;
    bool specifiedDepartureTime;

    TimeType arrivalTime;
    TimeType earlyArrivalTime;
    TimeType lateArrivalTime;

    TimeType departureTime;
    TimeType earlyDepartureTime;
    TimeType lateDepartureTime;

    TimeToSearchRoute()
        :
        specifiedArrivalTime(false),
        specifiedDepartureTime(false),
        arrivalTime(INFINITE_TIME),
        earlyArrivalTime(ZERO_TIME),
        lateArrivalTime(INFINITE_TIME),
        departureTime(INFINITE_TIME),
        earlyDepartureTime(ZERO_TIME),
        lateDepartureTime(INFINITE_TIME)
    {}

    void OffsetArrivalTime(const TimeType& offset) {
        arrivalTime = OffsetTime(arrivalTime, offset);
        earlyArrivalTime = OffsetTime(earlyArrivalTime, offset);
        lateArrivalTime = OffsetTime(lateArrivalTime, offset);
    }

    void OffsetDepartureTime(const TimeType& offset) {
        departureTime = OffsetTime(departureTime, offset);
        earlyDepartureTime = OffsetTime(earlyDepartureTime, offset);
        lateDepartureTime = OffsetTime(lateDepartureTime, offset);
    }

    TimeType OffsetTime(const TimeType& time, const TimeType& offset) const {
        if (time == INFINITE_TIME) {
            return INFINITE_TIME;
        }

        return std::max(ZERO_TIME, std::min(INFINITE_TIME, time+offset));
    }

    TimeType CalculateDepartureTime(const TimeType& duration) const {

        if (arrivalTime < INFINITE_TIME) {
            return arrivalTime - duration;
        }
        if (departureTime < INFINITE_TIME) {
            return departureTime;
        }

        return ZERO_TIME;
    }

    void SetMinTime(const TimeType& time) {
        arrivalTime = std::max(time, arrivalTime);
        earlyArrivalTime = std::max(time, earlyArrivalTime);
        lateArrivalTime = std::max(time, lateArrivalTime);

        departureTime = std::max(time, departureTime);
        earlyDepartureTime = std::max(time, earlyDepartureTime);
        lateDepartureTime = std::max(time, lateDepartureTime);
    }

    bool IsToEarlyArrivalTime(const TimeType& time) const {
        return (specifiedArrivalTime && arrivalTime < earlyArrivalTime);
    }

    bool IsTooLateArrivalTime(const TimeType& time) const {
        return (specifiedArrivalTime && arrivalTime > lateArrivalTime);
    }

    bool IsTooEarlyDeparture(const TimeType& time) const {
        return (specifiedDepartureTime && time < earlyDepartureTime);
    }

    bool IsTooLateDeparture(const TimeType& time) const {
        return (specifiedDepartureTime && departureTime > lateDepartureTime);
    }
};

typedef int AgentRouteOrderType;
enum {
    AGENT_ROUTE_ORDER_PRICE = 0,
    AGENT_ROUTE_ORDER_TIME = 1,
    AGENT_ROUTE_ORDER_TRANSFER_TIME = 2,
    NUMBER_AGENT_ROUTE_ORDERS,
};

typedef TimeType TimeOffsetType;
const TimeType MULTIAGENT_MIN_TIME_STEP = 1 * MILLI_SECOND;

typedef int32_t AgentBehaviorIdType;
typedef int32_t RouteNumberType;
const RouteNumberType NO_ROUTE_NUMBER = -1;

const size_t NUMBER_TIMESTEP_SNAPSHOTS = 2;

typedef size_t AgentStatusIdType;
enum {
    AGENT_RESERVED_STATUS_AGE = 0,

    AGENT_RESERVED_STATUS_WALK_SPEED = 1,
    AGENT_RESERVED_STATUS_BICYCLE_SPEED = 2,
    AGENT_RESERVED_STATUS_MAX_VEHICLE_SPEED = 3,

    AGENT_RESERVED_STATUS_QUERY_TRIGGER_START = 4,
    AGENT_RESERVED_STATUS_LAST_DELAY_QUERY_TRIGGER = 4,
    AGENT_RESERVED_STATUS_NEXT_DELAY_QUERY_TRIGGER = 5,
    AGENT_RESERVED_STATUS_TRIP_DELAY_QUERY_TRIGGER = 6,
    AGENT_RESERVED_STATUS_VEHICLE_DELAY_QUERY_TRIGGER = 7,
    AGENT_RESERVED_STATUS_CONGESTION_QUERY_TRIGGER = 8,

    AGENT_RESERVED_STATUS_UTILITY1_QUERY_TRIGGER = 9,
    AGENT_RESERVED_STATUS_UTILITY2_QUERY_TRIGGER = 10,

    AGENT_RESERVED_STATUS_QUERY_TRIGGER_END = 10,

    AGENT_RESERVED_STATUS_MIN_PATH_QUERY_INTERVAL = 11,
    AGENT_RESERVED_STATUS_PATH_QUERY_PROBABILITY = 12,
    AGENT_RESERVED_STATUS_MISSED_VEHICLE_PATH_QUERY_PROBABILITY = 13,

    AGENT_RESERVED_STATUS_GASOLIN_COST_PER_METERS = 14,

    AGENT_RESERVED_STATUS_UTILITY1 = 15,
    AGENT_RESERVED_STATUS_UTILITY2 = 16,

    AGENT_RESERVED_STATUS_RIDING_TIME = 17,
    AGENT_RESERVED_STATUS_SEEING_PEDESTRIAN_PROBABILITY = 18,

    // Vehicle Status
    AGENT_RESERVED_STATUS_TIME_HEADWAY = 19,
    AGENT_RESERVED_STATUS_MIN_VEHICLE_GAP = 20,
    AGENT_RESERVED_STATUS_MAX_ACCELERATION = 21,
    AGENT_RESERVED_STATUS_MAX_DECELERATION = 22,
    AGENT_RESERVED_STATUS_MAX_BRAKING_DECCELERATION = 23,
    AGENT_RESERVED_STATUS_ACCELERATION_EXPONENT = 24,
    AGENT_RESERVED_STATUS_SAVE_DECELERATION = 25,
    AGENT_RESERVED_STATUS_MAX_TURN_SPEED = 26,

    AGENT_RESERVED_LANE_CHANGE_ACCELERATION_THRESHOLD = 27,
    AGENT_RESERVED_VELOCITY_RATIO_GAP_DISTANCE = 28,
    AGENT_RESERVED_OTHER_VEHICLE_ENATRANCE_TIME = 29,
    AGENT_RESERVED_PASSIVE_YIELD_TIME = 30,
    AGENT_RESERVED_ACTIVE_YIELD_TIME = 31,
    AGENT_RESERVED_YIELD_WAITING_TIME = 32,


    AGENT_RESERVED_STATUS_TOTAL_TRAVEL_DISTANCE = 33,
    AGENT_RESERVED_STATUS_TOTAL_TRAVEL_TIME = 34,

    AGENT_RESERVED_STATUS_ROAD_WIDTH_WEIGHT = 35,

    AGENT_RESERVED_STATUS_DISASTER = 36,

    AGENT_RESERVED_STATUS_ROUTE_RECALCULATION_TIME = 37,

    AGENT_RESERVED_ACCEPTABLE_WALK_DISTANCE_TO_CAR = 38,
    AGENT_RESERVED_ACCEPTABLE_WALK_DISTANCE_TO_STOP = 39,
    AGENT_RESERVED_MIN_VEHICLE_ROUTE_DISTANCE = 40,
    AGENT_RESERVED_NUMBER_MAX_ROUTE_CANDIDATES = 41,
    AGENT_RESERVED_NUMBER_PEOPLE = 42,
    AGENT_RESERVED_ENTRANCE_WAIT_TIME = 43,
    AGENT_RESERVED_VEHICLE_ENTRANCE_WAIT_TIME = 44,
    AGENT_RESERVED_TAXICALL_WAIT_TIME = 45,

    NUMBER_AGENT_STATUS_VALUES,
};

static const char* RESERVED_AGENT_STATUS_NAMES[] = {
    "age",
    "walkspeed",
    "bicyclespeed",
    "vehiclespeed",
    "recalcintervalforlastviapointdelay",
    "recalcintervalfornextviapointdelay",
    "recalcintervalfordestinationdelay",
    "recalcintervalforvehicledelay",
    "recalcthresholdforcongestion",
    "recalcthresholdforutility1",
    "recalcthresholdforutility2",
    "minrouterecalcinterval",
    "routerecalcprobability",
    "recalcprobwhenmissingavehicle",
    "fuelconsumption",
    "utility1",
    "utility2",
    "boardingdelay",
    "pedestrianrecognitionprobability",
    "timeheadway",
    "minvehiclegap",
    "maxacceleration",
    "maxdeceleration",
    "maxbrakingdecceleration",
    "accelerationexponent",
    "savedecceleration",
    "minturnspeed",
    "lanechangeaccelerationthreshold",
    "velocityratiogapdistance",
    "othervehicleentrancetime",
    "passiveyieldtime",
    "activeyieldtime",
    "yieldwaitingtime",
    "totaltraveldistance",
    "totaltraveltime",
    "roadwidthweight",
    "disaster",
    "routerecalculationtime",
    "walkabledistancetoprivatecar",
    "walkabledistancetobusstoporstation",
    "mindistancetousevehicle",
    "maxroutecandidates",
    "numberofpeople",
    "maxwaitingtimeatentrance",
    "maxwaitingtimeatvehicleentrance",
    "maxwaitingtimefortaxiassignment"
};

static inline
bool IsBusOrTaxiProfile(const AgentStatusIdType& statusId)
{
    return ((statusId == AGENT_RESERVED_STATUS_MAX_VEHICLE_SPEED) ||
            (statusId == AGENT_RESERVED_STATUS_CONGESTION_QUERY_TRIGGER) ||
            (statusId == AGENT_RESERVED_STATUS_MIN_PATH_QUERY_INTERVAL) ||
            (statusId == AGENT_RESERVED_STATUS_PATH_QUERY_PROBABILITY) ||
            (statusId == AGENT_RESERVED_STATUS_GASOLIN_COST_PER_METERS) ||
            (AGENT_RESERVED_STATUS_SEEING_PEDESTRIAN_PROBABILITY <= statusId &&
             statusId <= AGENT_RESERVED_YIELD_WAITING_TIME) ||
            (statusId == AGENT_RESERVED_STATUS_ROAD_WIDTH_WEIGHT) ||
            (statusId == AGENT_RESERVED_STATUS_ROUTE_RECALCULATION_TIME) ||
            (statusId == AGENT_RESERVED_NUMBER_MAX_ROUTE_CANDIDATES));
}

typedef size_t AgentSpecialStatusIdType;
enum {
    AGENT_RESERVED_SPECIAL_STATUS_PRIVATE_CAR_OWNERSHIP = 0,
    AGENT_RESERVED_SPECIAL_STATUS_BICYCLE_OWNERSHIP = 1,
    AGENT_RESERVED_SPECIAL_STATUS_WALK_SPEED_AT_TRANSFER = 2,
    AGENT_RESERVED_SPECIAL_STATUS_UTILITY1_FUNCTION = 3,
    AGENT_RESERVED_SPECIAL_STATUS_UTILITY2_FUNCTION = 4,
    AGENT_RESERVED_SPECIAL_STATUS_ROUTE_PRIORITY = 5,
};

static const char* RESERVED_SPECIAL_AGENT_STATUS_NAMES[] = {
    "privatecarownership",
    "bicycleownership",
    "walkspeedattransfer",
    "utility1function",
    "utility2function",
    "routepriority"
};

typedef int8_t AgentStatusChangeType;
enum {
    AGENT_STATUS_CHANGE_BASIC_TASK_START,
    AGENT_STATUS_CHANGE_BASIC_TASK_BEFORE_WAITING,
    AGENT_STATUS_CHANGE_BASIC_TASK_END,

    AGENT_STATUS_CHANGE_TASK_INTERRUPTION_START,
    AGENT_STATUS_CHANGE_TASK_INTERRUPTION_BEFORE_WAITING,
    AGENT_STATUS_CHANGE_TASK_INTERRUPTION_END,

    AGENT_STATUS_CHANGE_AT_SPECIFIC_TIME,
};

static inline
bool IsBasicTaskStatusChange(const AgentStatusChangeType& changeType)  {
    return (changeType == AGENT_STATUS_CHANGE_BASIC_TASK_START ||
            changeType == AGENT_STATUS_CHANGE_BASIC_TASK_BEFORE_WAITING ||
            changeType == AGENT_STATUS_CHANGE_BASIC_TASK_END);
}

static inline
bool IsInterrupionTaskStatusChange(const AgentStatusChangeType& changeType)  {
    return (changeType == AGENT_STATUS_CHANGE_TASK_INTERRUPTION_START ||
            changeType == AGENT_STATUS_CHANGE_TASK_INTERRUPTION_BEFORE_WAITING ||
            changeType == AGENT_STATUS_CHANGE_TASK_INTERRUPTION_END);
}

static inline
bool IsSpecificTimeStatusChange(const AgentStatusChangeType& changeType)  {
    return (changeType == AGENT_STATUS_CHANGE_AT_SPECIFIC_TIME);
}

typedef int8_t AgentBehaviorInterruptionType;
enum {
    AGENT_BEHAVIOR_INTERRUPTION_NONE,
    AGENT_BEHAVIOR_INTERRUPTION_REPLACE,
    AGENT_BEHAVIOR_INTERRUPTION_LATER,
};

static inline
AgentStatusChangeType GetStatusChangeType(
    const bool isJustStatusChange,
    const bool hasWaitingAction,
    const bool isBeforeSpecification,
    const bool isIntterrputBehavior,
    const bool isBeforeWaiting)
{
    if (isJustStatusChange) {
        return AGENT_STATUS_CHANGE_AT_SPECIFIC_TIME;
    }

    if (isIntterrputBehavior) {

        if (isBeforeSpecification) {

            return AGENT_STATUS_CHANGE_TASK_INTERRUPTION_START;

        } else if (hasWaitingAction && isBeforeWaiting) {

            return AGENT_STATUS_CHANGE_TASK_INTERRUPTION_BEFORE_WAITING ;
        }

        return AGENT_STATUS_CHANGE_TASK_INTERRUPTION_END;

    } else {

        if (isBeforeSpecification) {

            return AGENT_STATUS_CHANGE_BASIC_TASK_START;

        } else if (hasWaitingAction && isBeforeWaiting) {

            return AGENT_STATUS_CHANGE_BASIC_TASK_BEFORE_WAITING ;
        }

        return AGENT_STATUS_CHANGE_BASIC_TASK_END;
    }
}

typedef int8_t AgentTaskPurposeType;
enum {
    AGENT_TASK_PURPOSE_OTHER
};

struct AgentStatus {
    vector<double> values;
    Vertex currentPosition;
    Vertex nextPosition;

    AgentStatus() {}
};

struct RoadRoute {
    VertexIdType nextVertexId;
    RoadIdType roadId;

    RoadRoute()
        :
        nextVertexId(INVALID_VERTEX_ID),
        roadId(INVALID_VARIANT_ID)
    {}

    RoadRoute(
        const VertexIdType& initNextVertexId,
        const RoadIdType& initRoadId)
        :
        nextVertexId(initNextVertexId),
        roadId(initRoadId)
    {}

    static RoadRoute CreateVirtualRoute(const VertexIdType& initNextVertexId) {
        return RoadRoute(initNextVertexId, INVALID_VARIANT_ID);
    }

    bool IsVirtualRoute() const { return (roadId == INVALID_VARIANT_ID); }
};

struct VehicleConstant {
    double vehicleHalfLength;
    double laneChangeAccelerationThreshold;
    double maxAcceleration;
    double maxDecelaration;
    double maxBrakingDecceleration;
    double accelerationExponent;
    double saveAcceleration;
    double maxTurnSpeed;
    double maxVehicleSpeed;
    double minVehicleGap;
    double timeHeadway;
    double velocityRatioGapDistance;
    double otherVehicleEntranceTime;
    double passiveYieldTime;
    double activeYieldTime;
    double yieldWaitingTime;
    double gasolinCost;

    VehicleConstant()
        :
        vehicleHalfLength(DEFAULT_VEHICLE_LENGTH_METERS*0.5),
        laneChangeAccelerationThreshold(DEFAULT_LANE_CHANGE_ACCELERATION_THRESHOLD),
        maxAcceleration(DEFAULT_MAX_ACCELERATION),
        maxDecelaration(DEFAULT_MAX_DECELERATION),
        maxBrakingDecceleration(DEFAULT_MAX_BRAKING_DECCELERATION),
        accelerationExponent(DEFAULT_ACCELERATION_EXPONENT),
        saveAcceleration(DEFAULT_SAVE_DECELERATION),
        maxTurnSpeed(DEFAULT_MAX_TURN_SPEED),
        maxVehicleSpeed(DEFAULT_MAX_VEHICLE_SPEED),
        minVehicleGap(DEFAULT_MIN_VEHICLE_GAP),
        timeHeadway(DEFAULT_TIME_HEADWAY),
        velocityRatioGapDistance(DEFAULT_VELOCITY_RATIO_GAP_DISTANCE),
        otherVehicleEntranceTime(DEFAULT_OTHER_VEHICLE_ENATRANCE_TIME),
        passiveYieldTime(DEFAULT_PASSIVE_YIELD_TIME),
        activeYieldTime(DEFAULT_ACTIVE_YIELD_TIME),
        yieldWaitingTime(DEFAULT_YIELD_WAITING_TIME),
        gasolinCost(DEFAULT_GASOLIN_COST)
    {}

    VehicleConstant(
        const double initVehicleHalfLength,
        const double initLaneChangeAccelerationThreshold,
        const double initMaxAcceleration,
        const double initMaxDecelaration,
        const double initMaxBrakingDecceleration,
        const double initAccelerationExponent,
        const double initSaveAcceleration,
        const double initMaxTurnSpeed,
        const double initMaxVehicleSpeed,
        const double initMinVehicleGap,
        const double initTimeHeadway,
        const double initVelocityRatioGapDistance,
        const double initOtherVehicleEntranceTime,
        const double initPasiveYieldTime,
        const double initActiveYieldTime,
        const double initYieldWaitingTime,
        const double initGasolinCost)
        :
        vehicleHalfLength(initVehicleHalfLength),
        laneChangeAccelerationThreshold(initLaneChangeAccelerationThreshold),
        maxAcceleration(initMaxAcceleration),
        maxDecelaration(initMaxDecelaration),
        maxBrakingDecceleration(initMaxBrakingDecceleration),
        accelerationExponent(initAccelerationExponent),
        saveAcceleration(initSaveAcceleration),
        maxTurnSpeed(initMaxTurnSpeed),
        maxVehicleSpeed(initMaxVehicleSpeed),
        minVehicleGap(initMinVehicleGap),
        timeHeadway(initTimeHeadway),
        velocityRatioGapDistance(initVelocityRatioGapDistance),
        otherVehicleEntranceTime(initOtherVehicleEntranceTime),
        passiveYieldTime(initPasiveYieldTime),
        activeYieldTime(initActiveYieldTime),
        yieldWaitingTime(initYieldWaitingTime),
        gasolinCost(initGasolinCost)
    {}
};

static inline
bool IsVehicleStatus(const AgentStatusIdType& statusId)
{
    return (statusId == AGENT_RESERVED_STATUS_MAX_VEHICLE_SPEED ||
            statusId == AGENT_RESERVED_STATUS_GASOLIN_COST_PER_METERS ||
            (AGENT_RESERVED_STATUS_TIME_HEADWAY <= statusId && statusId <= AGENT_RESERVED_YIELD_WAITING_TIME));
}

struct BusTicket {
    LineIdType lineId;
    RouteIdType routeId;
    VehicleNumberType reservedVehicleNumber;

    StopIdType departureStopId;
    StopIdType arrivalStopId;

    TimeType departureTime;

    AgentIdType agentId;

    LineType lineType;

    BusTicket()
        :
        lineId(INVALID_LINE_ID),
        routeId(INVALID_ROUTE_ID),
        reservedVehicleNumber(NO_VEHICLE_NUMBER),
        departureStopId(INVALID_STOP_ID),
        arrivalStopId(INVALID_STOP_ID),
        departureTime(INFINITE_TIME),
        agentId(MASTER_ANY_AGENT_ID),
        lineType(LINE_NORMAL)
    {}

    BusTicket(
        const LineIdType& initLineId,
        const RouteIdType& initRouteId,
        const StopIdType& initDepartureStopId,
        const StopIdType& initArrivalStopId,
        const TimeType& initDepartureTime,
        const AgentIdType& initAgentId)
        :
        lineId(initLineId),
        routeId(initRouteId),
        reservedVehicleNumber(NO_VEHICLE_NUMBER),
        departureStopId(initDepartureStopId),
        arrivalStopId(initArrivalStopId),
        departureTime(initDepartureTime),
        agentId(initAgentId),
        lineType(LINE_NORMAL)
    {}

    bool IsAvailable() const { return (reservedVehicleNumber != NO_VEHICLE_NUMBER); }
};

typedef double AgentTravelMode;
const AgentTravelMode AGENT_TRAVEL_MODE_PREFERED = 10.;
const AgentTravelMode AGENT_TRAVEL_MODE_OTHREWISE = 0.;

typedef double AgentCongestionType;
const AgentCongestionType AGENT_CONGESTION_NO = 0.;
const AgentCongestionType AGENT_CONGESTION_FULL = 100.;
const AgentCongestionType AGENT_CONGESTION_MAX = 250.;

typedef uint8_t AgentRouteCostType;
enum {
    AGENT_ROUTE_COST_MODE = 0,
    AGENT_ROUTE_COST_ARRIVAL_TIME = 1,
    AGENT_ROUTE_COST_TRAVEL_TIME = 2,
    AGENT_ROUTE_COST_TRAVEL_DISTANCE = 3,
    AGENT_ROUTE_COST_VARIABILITY_TIME = 4,
    AGENT_ROUTE_COST_PRICE = 5,
    AGENT_ROUTE_COST_TRANSFER_TIME = 6,
    AGENT_ROUTE_COST_TRANSFER_DURATION = 7,
    AGENT_ROUTE_COST_PASSANGER_CONGESTION = 8,
    AGENT_ROUTE_COST_VEHICLE_CONGESTION = 9,

    NUMBER_AGENT_ROUTE_COSTS,
};

struct AgentRoadRouteCost {

    TimeType totalTravelTime;
    double travelDistance;
    double price;
    int numberPedestrians;
    int numberVehicles;

    const TimeType TravelTime() const {
        return totalTravelTime;
    }
    const double TravelDistance() const {
        return travelDistance;
    }
    const double Price() const {
        return price;
    }

    AgentRoadRouteCost()
        :
        totalTravelTime(ZERO_TIME),
        travelDistance(0),
        price(0),
        numberPedestrians(0),
        numberVehicles(0)
    {}

    AgentRoadRouteCost(
        const TimeType& initTotalTravelTime,
        const double& initTravelDistance,
        const double initPrice,
        const int initNumberPedestrians,
        const int initNumberVehicles)
        :
        totalTravelTime(initTotalTravelTime),
        travelDistance(initTravelDistance),
        price(initPrice),
        numberPedestrians(initNumberPedestrians),
        numberVehicles(initNumberVehicles)
    {}

    bool IsBetterThan(
        const AgentRoadRouteCost& cost,
        const AgentRouteOrderType& order) const {
        if (order == AGENT_ROUTE_ORDER_TIME) {
            return totalTravelTime < cost.totalTravelTime;
        } else if (order == AGENT_ROUTE_ORDER_PRICE) {
            return price < cost.price;
        }

        return false;
    }

    void operator+=(const AgentRoadRouteCost& right) {
        totalTravelTime += right.totalTravelTime;
        travelDistance += right.travelDistance;
        price += right.price;
        numberPedestrians += right.numberPedestrians;
        numberVehicles += right.numberVehicles;
    }

    AgentRoadRouteCost operator+(const AgentRoadRouteCost& right) const {
        return AgentRoadRouteCost(
            totalTravelTime + right.totalTravelTime,
            travelDistance + right.travelDistance,
            price + right.price,
            numberPedestrians + right.numberPedestrians,
            numberVehicles + right.numberVehicles);
    }
};

struct AgentRouteCost {
    double values[NUMBER_AGENT_ROUTE_COSTS];

    const TimeType ArrivalTime() const {
        return static_cast<TimeType>(values[AGENT_ROUTE_COST_ARRIVAL_TIME] * SECOND);
    }
    const TimeType TravelTime() const {
        return static_cast<TimeType>(values[AGENT_ROUTE_COST_TRAVEL_TIME] * SECOND);
    }
    const TimeType LatestTravelTime() const {
        return static_cast<TimeType>(
            (values[AGENT_ROUTE_COST_TRAVEL_TIME] + values[AGENT_ROUTE_COST_VARIABILITY_TIME]) * SECOND);
    }
    const double TravelDistance() const {
        return values[AGENT_ROUTE_COST_TRAVEL_DISTANCE];
    }
    const TimeType CalculateTravelDuration(const double speedMetersPerSec) {
        return static_cast<TimeType>((*this).TravelDistance() / speedMetersPerSec * SECOND);
    }

    const double Price() const {
        return values[AGENT_ROUTE_COST_PRICE];
    }

    AgentRouteCost() {
        for(size_t i = 0; i < NUMBER_AGENT_ROUTE_COSTS; i++) {
            values[i] = 0;
        }
    }

    AgentRouteCost(const AgentRoadRouteCost& roadCost) {

        for(size_t i = 0; i < NUMBER_AGENT_ROUTE_COSTS; i++) {
            values[i] = 0;
        }

        values[AGENT_ROUTE_COST_TRAVEL_TIME] = double(roadCost.totalTravelTime) / SECOND;
        values[AGENT_ROUTE_COST_TRAVEL_DISTANCE] = roadCost.travelDistance;
        values[AGENT_ROUTE_COST_PRICE] = roadCost.price;
        values[AGENT_ROUTE_COST_PASSANGER_CONGESTION] = roadCost.numberPedestrians;
        values[AGENT_ROUTE_COST_VEHICLE_CONGESTION] = roadCost.numberVehicles;
    }

    double GetOrderCost(const AgentRouteOrderType& order) const {
        if (order == AGENT_ROUTE_ORDER_TIME) {
            return values[AGENT_ROUTE_COST_TRAVEL_TIME];
        } else if (order == AGENT_ROUTE_ORDER_PRICE) {
            return values[AGENT_ROUTE_COST_PRICE];
        } else {
            return values[AGENT_ROUTE_COST_TRANSFER_TIME];
        }
    }

    bool IsEqual(
        const AgentRouteCost& cost,
        const AgentRouteOrderType& order) const {
        if (order == AGENT_ROUTE_ORDER_TIME) {
            return values[AGENT_ROUTE_COST_TRAVEL_TIME] == cost.values[AGENT_ROUTE_COST_TRAVEL_TIME];
        } else if (order == AGENT_ROUTE_ORDER_PRICE) {
            return values[AGENT_ROUTE_COST_PRICE] == cost.values[AGENT_ROUTE_COST_PRICE];
        } else {
            return values[AGENT_ROUTE_COST_TRANSFER_TIME] == cost.values[AGENT_ROUTE_COST_TRANSFER_TIME];
        }
    }
    bool IsBetterThan(
        const AgentRouteCost& cost,
        const AgentRouteOrderType& order) const {
        if (order == AGENT_ROUTE_ORDER_TIME) {
            return values[AGENT_ROUTE_COST_TRAVEL_TIME] < cost.values[AGENT_ROUTE_COST_TRAVEL_TIME];
        } else if (order == AGENT_ROUTE_ORDER_PRICE) {
            return values[AGENT_ROUTE_COST_PRICE] < cost.values[AGENT_ROUTE_COST_PRICE];
        } else {
            return values[AGENT_ROUTE_COST_TRANSFER_TIME] < cost.values[AGENT_ROUTE_COST_TRANSFER_TIME];
        }
    }

    bool operator==(const AgentRouteCost& right) const {
        for(size_t i = 0; i < NUMBER_AGENT_ROUTE_COSTS; i++) {
            if (values[i] != right.values[i]) {
                return false;
            }
        }
        return true;
    }

    void operator+=(const AgentRouteCost& right) {
        for(size_t i = 0; i < NUMBER_AGENT_ROUTE_COSTS; i++) {
            values[i] += right.values[i];
        }
    }

    AgentRouteCost operator+(const AgentRouteCost& right) const {
        AgentRouteCost cost;

        for(size_t i = 0; i < NUMBER_AGENT_ROUTE_COSTS; i++) {
            cost.values[i] = values[i] + right.values[i];
        }

        return cost;
    }
};

typedef uint8_t AgentHealthOrUtilityFactorType;
enum {
    AGENT_HEALTH_FACTOR_CONGESTION = 0,
    AGENT_UTILITY_FACTOR_LAST_DELAY = 1,
    AGENT_UTILITY_FACTOR_NEXT_DELAY = 2,
    AGENT_UTILITY_FACTOR_TRIP_DELAY = 3,
    AGENT_UTILITY_FACTOR_ARRIVAL_DELAY = 4,
    AGENT_UTILITY_FACTOR_UTILITY1_COUNTER = 5,
    AGENT_UTILITY_FACTOR_UTILITY2_COUNTER = 6,

    NUMBER_HEALTH_OR_UTILITY_FACTORS,
};

struct AgentHealthOrUtilityFactor {
    double values[NUMBER_HEALTH_OR_UTILITY_FACTORS];

    AgentHealthOrUtilityFactor() {
        for(size_t i = 0; i < NUMBER_HEALTH_OR_UTILITY_FACTORS; i++) {
            values[i] = 0;
        }
    }
};

struct AgentRoute {
    AgentBehaviorType behavior;
    AgentRouteCost totalCost;

    AgentRoute() : behavior(AGENT_BEHAVIOR_NOTHING) {}
    AgentRoute(const AgentBehaviorType& initBehavior) : behavior(initBehavior) {}

    bool IsRoad() const { return !roadRoutes.empty(); }
    bool IsPublicVehicle() const { return !publicVehicleRoutes.empty(); }
    bool IsEmpty() const { return
            ((behavior != AGENT_BEHAVIOR_FREEWALK) &&
             (roadRoutes.empty() && publicVehicleRoutes.empty())); }

    const VertexIdType GetNextRoadVertexId(const size_t i) const { return roadRoutes.at(i).nextVertexId; }
    const VertexIdType GetDestVertexId() const {
        assert((*this).IsRoad());
        return roadRoutes.back().nextVertexId;
    }
    const VertexIdType GetStartVertexId() const {
        assert((*this).IsRoad());
        return roadRoutes.front().nextVertexId;
    }

    const RoadIdType GetFrontRoadId() const { return roadRoutes.front().roadId; }
    const RoadIdType GetRoadId(const size_t i) const { return roadRoutes.at(i).roadId; }
    const RoadRoute& GetRoadRoute(const size_t i) const { return roadRoutes.at(i); }


    const StopIdType GetStartStopId() const { assert((*this).IsPublicVehicle()); return publicVehicleRoutes.front().origStopId; }
    const StopIdType GetDestStopId() const { assert((*this).IsPublicVehicle()); return publicVehicleRoutes.back().nextStopId; }
    const StopIdType GetStopId(const size_t i) const { return publicVehicleRoutes.at(i).origStopId; }
    const StopIdType GetDownStopId(const size_t i) const { return publicVehicleRoutes.at(i).nextStopId; }
    const StopIdType IsTransferLine(const size_t i) const { return (publicVehicleRoutes.at(i).lineId == TRANSFER_LINE_ID); }

    GisPositionIdType GetStartRoadPositionId() const {
        assert((*this).IsRoad());
        return GisPositionIdType(GIS_ROAD, roadRoutes.front().roadId);
    }

    void Clear() {
        behavior = AGENT_BEHAVIOR_NOTHING;
        totalCost = AgentRouteCost();
        roadRoutes.clear();
        publicVehicleRoutes.clear();
    }

    string ConvertToString() const {
        ostringstream outStream;

        if (behavior == AGENT_BEHAVIOR_FREEWALK) {
            return "FreeWalk";
        } else if ((*this).IsRoad()) {
            outStream << "Road (" << int(behavior) << ")";
            for(size_t i = 0; i < roadRoutes.size(); i++) {
                const RoadRoute& roadRoute = roadRoutes[i];
                outStream << "," << roadRoute.nextVertexId << "(r" << roadRoute.roadId << ")";
            }
        } else if ((*this).IsPublicVehicle()) {
            outStream << "Public";
            for(size_t i = 0; i < publicVehicleRoutes.size(); i++) {
                const PublicVehicleRoute& publicVehicleRoute = publicVehicleRoutes[i];

                outStream << "," << publicVehicleRoute.origStopId << "-" << publicVehicleRoute.nextStopId
                          << "(" << publicVehicleRoute.routeId << ")";
            }
        } else {
            outStream << "Empty";
        }

        return outStream.str();
    }

    TimeType DepartureTime() const {
        assert((*this).IsPublicVehicle());
        return publicVehicleRoutes.front().departureTime;
    }

    TimeType ArrivalTime() const {
        assert((*this).IsPublicVehicle());
        return publicVehicleRoutes.back().arrivalTime;
    }

    deque<RoadRoute> roadRoutes;
    deque<PublicVehicleRoute> publicVehicleRoutes;
};

struct AgentRouteList {
    TimeType startTime;
    AgentRouteCost totalCost;

    list<shared_ptr<AgentRoute> > routePtrs;

    AgentRouteList() : startTime(ZERO_TIME) {}

    void Clear() {
        totalCost = AgentRouteCost();
        routePtrs.clear();
    }

    bool IsEmpty() const {
        typedef list<shared_ptr<AgentRoute> >::const_iterator IterType;

        for(IterType iter = routePtrs.begin(); iter != routePtrs.end(); iter++) {
            if (!(*iter)->IsEmpty()) {
                return false;
            }
        }

        return true;
    }

    void PushRoute(const shared_ptr<AgentRoute>& aRoutePtr) {
        totalCost += aRoutePtr->totalCost;
        routePtrs.push_back(aRoutePtr);

        totalCost.values[AGENT_ROUTE_COST_ARRIVAL_TIME] =
            aRoutePtr->totalCost.values[AGENT_ROUTE_COST_ARRIVAL_TIME];
    }

    void PushRouteList(const AgentRouteList& routeList) {
        totalCost += routeList.totalCost;

        typedef list<shared_ptr<AgentRoute> >::const_iterator IterType;

        for(IterType iter = routeList.routePtrs.begin(); iter != routeList.routePtrs.end(); iter++) {
            routePtrs.push_back(*iter);
        }

        if (!routeList.routePtrs.empty()) {
            totalCost.values[AGENT_ROUTE_COST_ARRIVAL_TIME] =
                routeList.routePtrs.back()->totalCost.values[AGENT_ROUTE_COST_ARRIVAL_TIME];
        }
    }

    string ConvertToString() const {
        ostringstream outStream;

        if (routePtrs.empty()) {
            outStream << "empty route";
        } else {

            typedef list<shared_ptr<AgentRoute> >::const_iterator IterType;

            for(IterType iter = routePtrs.begin(); iter != routePtrs.end(); iter++) {
                assert(*iter != nullptr);
                outStream << " [" << (*iter)->ConvertToString() << " AT=" << double((*iter)->totalCost.ArrivalTime())/SECOND << "]";
            }
        }

        return outStream.str();
    }
};

struct DynamicApplicationIdType {
    string applicationModelName;
    string instanceName;

    DynamicApplicationIdType() {}

    DynamicApplicationIdType(
        const string& initApplicationModelName,
        const string& initInstanceName)
        :
        applicationModelName(initApplicationModelName),
        instanceName(initInstanceName)
    {}

    bool operator<(const DynamicApplicationIdType& right) const {
        return ((applicationModelName < right.applicationModelName) ||
                (applicationModelName == right.applicationModelName &&
                 instanceName < right.instanceName));
    }
};

struct DynamicApplicationDefinitionParameter {
    string applicationParameterName;
    string value;

    DynamicApplicationDefinitionParameter() {}

    DynamicApplicationDefinitionParameter(
        const string& initApplicationParameterName,
        const string& initValue)
        :
        applicationParameterName(initApplicationParameterName),
        value(initValue)
    {}
};

struct DynamicApplicationDefinition {
    vector<DynamicApplicationDefinitionParameter> parameters;
};

class AgentValueFormula;

struct AgentAdditionalStatusChange {
    vector<pair<AgentStatusIdType, AgentValueFormula> > statusChanges;
    map<DynamicApplicationIdType, DynamicApplicationDefinition> applicationSpecifications;
};

typedef int AgentLocationChoiceType;
enum {
    AGENT_LOCATION_CHOICE_RANDOM = 0,
    AGENT_LOCATION_CHOICE_NEAREST = 1,
};

}//namespace

#endif
