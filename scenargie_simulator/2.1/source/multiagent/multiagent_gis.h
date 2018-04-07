// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef MULTIAGENT_GIS_H
#define MULTIAGENT_GIS_H

#include "multiagent_agentsim.h"
#include "multiagent_queuepositioner.h"
#include <fstream>
#include "boost/shared_array.hpp"
#include <bitset>

namespace MultiAgent {

using std::fstream;
using boost::shared_array;
using std::bitset;

const int MAX_NUMBER_ROAD_CROSSING_GRANURALITIES = 64;

class VehiclePositionKey {
public:
    VehiclePositionKey()
        :
        isInitialized(false),
        roadId(INVALID_VARIANT_ID),
        laneNumber(BAD_SIZE_T)
    {}

    bool IsInitialized() const { return isInitialized; }

private:
    typedef list<shared_ptr<Vehicle> >::const_iterator VehicleIter;

    VehiclePositionKey(
        const VehiclePositionKey& initPositionKey)
        :
        isInitialized(initPositionKey.isInitialized),
        roadId(initPositionKey.roadId),
        laneNumber(initPositionKey.laneNumber),
        iter(initPositionKey.iter)
    {}

    friend class AgentRoad;
    friend class AgentIntersection;
    friend class MultiAgentGis;

    bool isInitialized;
    RoadIdType roadId;
    size_t laneNumber;
    VehicleIter iter;
};

struct RoadWaypoint {
    Vertex position;
    float turnRadians;

    RoadWaypoint(
        const Vertex& initPosition,
        const float initTurnRadians = 0)
        :
        position(initPosition),
        turnRadians(initTurnRadians)
    {}
};





class AgentRoad {
public:
    struct RoadStat {
        double polygonSize;
        shared_ptr<RealStatistic> congestionStatPtr;
        TraceValue<double> congestionTrace;

        RoadStat(
            const shared_ptr<RealStatistic>& initCongestionStatPtr)
            :
            polygonSize(0),
            congestionStatPtr(initCongestionStatPtr),
            congestionTrace(0.)
        {}
    };

    AgentRoad(
        const shared_ptr<const Road>& gisRoadPtr,
        const shared_ptr<RoadStat>& roadStatPtr,
        const vector<RoadIdType>& groupedRoadIds);

    bool HasFrontVehicle(
        const Vehicle& vehicle,
        const size_t laneNumber) const;

    bool HasBackVehicle(
        const Vehicle& vehicle,
        const size_t laneNumber) const;

    bool HasVehicle(const size_t laneNumber) const;

    const Vehicle& GetFrontVehicle(
        const Vehicle& vehicle,
        const size_t laneNumber) const;

    const Vehicle& GetBackVehicle(
        const Vehicle& vehicle,
        const size_t laneNumber) const;

    const Vehicle& GetTailVehicle(
        const size_t laneNumber) const;

    bool AllowedIncomingVehicle(
        const Vehicle& vehicle,
        const size_t laneNumber) const;

    Vertex GetOthersideLaneNormalPosition(
        const VertexIdType& origVertexId,
        const Vertex& origPosition,
        const bool isOrigPositionLeft) const;

    int GetNumberOfSystemPedestrians() const {
        return systemInfo.lastNumberPedestrians;
    }


    int GetNumberOfAverageVehicles(const size_t laneNumber) const {
        return (*this).GetNumberOfAverageVehicles(laneNumber, realInfo);
    }
    int GetNumberOfSystemAverageVehicles(const size_t laneNumber) const {
        return (*this).GetNumberOfAverageVehicles(laneNumber, systemInfo);
    }

    double CalculatePeopleCongestion() const {
        return (*this).CalculatePeopleCongestion(realInfo.lastNumberPedestrians);
    }
    double CalculatePeopleCongestionWithout(const AgentResource& resource) const {
        return (*this).CalculatePeopleCongestion(std::max(0, realInfo.lastNumberPedestrians - resource.NumberPeople()));
    }

    double CalculatePeopleSystemCongestion() const {
        return (*this).CalculatePeopleCongestion(systemInfo.lastNumberPedestrians);
    }

    double CalculateVehicleCongestion(const size_t laneNumber) const {
        return (*this).CalculateVehicleCongestion(
            laneNumber,
            (*this).GetNumberOfAverageVehicles(laneNumber));
    }
    double CalculateVehicleSystemCongestion(const size_t laneNumber) const {
        return (*this).CalculateVehicleCongestion(
            laneNumber,
            (*this).GetNumberOfSystemAverageVehicles(laneNumber));
    }

    bool CrossingAPedestrian(const VertexIdType& outgoingVertexId) const;
    bool CrossingOrWaitingAPedestrian(const VertexIdType& outgoingVertexId) const;

    double GetRoadLength() const { return length; }

    const Road& GetGisRoad() const { return (*gisRoadPtr); }

private:
    friend class AgentIntersection;
    friend class MultiAgentGis;

    shared_ptr<const Road> gisRoadPtr;

    shared_ptr<AgentIntersection> startIntersectionPtr;
    shared_ptr<AgentIntersection> endIntersectionPtr;

    typedef list<shared_ptr<Vehicle> >::const_iterator VehicleIter;
    typedef list<shared_ptr<Vehicle> >::const_reverse_iterator ReverseVehicleIter;

    double length;
    double roadGranurality;
    bool containsSomeVehicle;

    vector<list<shared_ptr<Vehicle> > > vehiclesPerDirection;
    vector<double> laneLengths;

    shared_ptr<RealStatistic> congestionStatPtr;

    Vertex startIntersectionAcrossBase;
    Vertex endIntersectionAcrossBase;

    struct NextStateInfoType {
        int numberLeavePeoples;
        bool peopleChanged;

        bool waitingPeopleAtStartIntersection;
        bool waitingPeopleAtEndIntersection;

        bitset<MAX_NUMBER_ROAD_CROSSING_GRANURALITIES> crossingPeopleExistanceAtStartIntersection;
        bitset<MAX_NUMBER_ROAD_CROSSING_GRANURALITIES> crossingPeopleExistanceAtEndIntersection;
        bool crossingChanged;

        list<AgentResource> incomingPeopleAgents;

        NextStateInfoType()
            :
            numberLeavePeoples(0),
            peopleChanged(false),
            waitingPeopleAtStartIntersection(false),
            waitingPeopleAtEndIntersection(false),
            crossingChanged(false)
        {}
    };

    NextStateInfoType nextStateInfo;

    bool needToOutputStatsAtThisTime;;
    bool needToExecuteCrossingPedestrianUpdateAtThisTime;

    // list<AgentResource> incomingPeopleAgents;

    bool waitingPeopleAtStartIntersection;
    bool waitingPeopleAtEndIntersection;
    bitset<MAX_NUMBER_ROAD_CROSSING_GRANURALITIES> crossingPeopleExistanceAtStartIntersection;
    bitset<MAX_NUMBER_ROAD_CROSSING_GRANURALITIES> crossingPeopleExistanceAtEndIntersection;

    shared_ptr<RoadStat> roadStatPtr;
    vector<RoadIdType> groupedRoadIds;

    struct NavigationSystemInfo {
        TimeType updatedTime;
        int lastNumberUpVehicles;
        int lastNumberDownVehicles;
        int lastNumberPedestrians;

        NavigationSystemInfo()
            :
            lastNumberUpVehicles(0),
            lastNumberDownVehicles(0),
            lastNumberPedestrians(0)
        {}
    };

    NavigationSystemInfo realInfo;
    NavigationSystemInfo systemInfo;

    //---------------------------------------------------------------------

    void FindFrontVehicle(
        const VehicleIter& origIter,
        const size_t laneNumber,
        bool& found,
        VehicleIter& iter) const;

    void FindBackVehicle(
        const VehicleIter& origIter,
        const size_t laneNumber,
        bool& found,
        VehicleIter& iter) const;

    void FindLeaderVehicleIter(
        const size_t laneNumber,
        bool& found,
        VehicleIter& iter) const;

    void FindTailVehicleIter(
        const size_t laneNumber,
        bool& found,
        ReverseVehicleIter& iter,
        const double searchDistanceFromTail = DBL_MAX) const;

    void SetStartIntersectionMargin(const double marginMeters);
    void SetEndIntersectionMargin(const double marginMeters);

    int GetNumberOfAverageVehicles(
        const size_t laneNumber,
        const NavigationSystemInfo& naviInfo) const {
        const size_t numberStartToEndLanes = gisRoadPtr->GetNumberOfStartToEndLanes();
        const size_t numberEndToStartLanes = gisRoadPtr->GetNumberOfEndToStartLanes();

        if (laneNumber < numberStartToEndLanes) {
            if (numberStartToEndLanes > 0) {
                return (int)(naviInfo.lastNumberUpVehicles / numberStartToEndLanes);
            }
        } else {
            if (numberEndToStartLanes > 0) {
                return (int)(naviInfo.lastNumberDownVehicles / numberEndToStartLanes);
            }
        }

        assert(false);
        return 0;
    }

    double CalculateVehicleCongestion(
        const size_t laneNumber,
        const int numberVehicles) const {
        const double basicVehicleLength = 5;
        const double numberMaxVehicles = length / basicVehicleLength;
        const double density = std::min(numberVehicles / numberMaxVehicles, 1.);
        return density;
    }

    double CalculatePeopleCongestion(const int lastNumberPedestrians) const {
        const double basicPedestrianLength = 1; //1m/pedestrian and two line.
        const double numberMaxPedestrians = length / basicPedestrianLength;
        const double density = std::min(lastNumberPedestrians / numberMaxPedestrians, 1.);
        return density;
    }

};//AgentRoad//


class AgentIntersection {
public:
    AgentIntersection(
        const Intersection& initIntersection,
        const list<pair<double, RoadIdType> >& initSequencedRoadIdsOrderedByRadian)
        :
        ref(initIntersection),
        signalSwitchingDuration(30*SECOND),
        sequencedRoadIdsOrderedByRadian(initSequencedRoadIdsOrderedByRadian)
    {}

    const Intersection& ref;

    void CanEnterTheRoad(
        const TimeType& currentTime,
        const VehicleDriverBehavior& vehicleBehavior,
        const RoadTurnType& origTurn,
        const RoadIdType& incomingRoadId,
        const RoadIdType& outgoingRoadId,
        bool& canPassIntersection,
        TimeType& entranceEnabledTime) const;

    void GetCrossingOrSideWalkRoadIds(
        const MultiAgentGis& theAgentGis,
        const RoadIdType& startRoadId,
        const RoadIdType& endRoadId,
        const bool isStartLaneLeft,
        bool& isClockwise,
        list<pair<RoadIdType, bool> >& crossingOrRoadsideWalks,
        deque<Vertex>& currentWaypoints,
        HighQualityRandomNumberGenerator& aRandomNumberGenerator) const;

private:
    friend class AgentRoad;
    friend class MultiAgentGis;

    typedef list<shared_ptr<Vehicle> >::const_iterator VehicleIter;

    struct IncomingRoad {
        shared_ptr<AgentRoad> gisRoadPtr;
        size_t laneNumber;

        IncomingRoad(
            const shared_ptr<AgentRoad>& initRoadPtr,
            const size_t initLaneNumber)
            :
            gisRoadPtr(initRoadPtr),
            laneNumber(initLaneNumber)
        {}
    };

    TimeType signalSwitchingDuration;

    map<pair<RoadIdType, RoadIdType>, RoadTurnType> roadTurnTypes;
    list<pair<double, RoadIdType> > sequencedRoadIdsOrderedByRadian;

    vector<IncomingRoad> incomingRoads;

    bool IsLowPriorityRoad(
        const RoadTurnType& srcTurnType,
        const RoadTurnType& otherTurnType,
        const RoadTurnType& srcToOtherTurnType) const;
};

class AgentPeopleBuffer {
public:
    AgentPeopleBuffer(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const MultiAgentGis& theMultiAgentGis,
        const set<EntranceIdType>& initEntranceIds,
        const shared_ptr<RealStatistic>& initCongestionStatPtr,
        const shared_ptr<RealStatistic>& initPopulationStatPtr);

    void UpdateEntranceQueue(
        const TimeType& currentTime,
        bool& isActive)
    {
        entranceQueue.DetermineIfHasPersonWhoCanEnterAtThisTime(currentTime, isActive);
    }
    
    void TransferPeopleToGisObject(
        const TimeType& currentTime,
        const GisPositionIdType& currentPositionId,
        MultiAgentGis& theAgentGis);

    void UpdateEntranceQueuesStatus(
        const TimeType& currentTime,
        bool& peopleChanged);

protected:
    friend class MultiAgentGis;

    struct NextPeopleStateInfoType {
        int numberLeavePeoples;
        bool peopleChanged;
        vector<AgentResource> incomingPeopleAgents;
        vector<pair<AgentProfileType, AgentIdType> > outgoingAgents;
        vector<AgentResource> giveUpEntranceAgents;

        NextPeopleStateInfoType() :
            numberLeavePeoples(0),
            peopleChanged(false)
        {}
    };

    NextPeopleStateInfoType nextPeopleStateInfo;

    GisObjectMultiEntranceQueue entranceQueue;

    bool needToOutputStatsAtThisTime;

    TraceValue<double> congestionTrace;

    int numberPeoples;
    set<AgentResource> parkingVehicleOwnerAgents;

    shared_ptr<RealStatistic> congestionStatPtr;
    shared_ptr<RealStatistic> populationStatPtr;

    unique_ptr<PedQueuePositioner> CreateEntranceLineQueuePositioner(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const GisSubsystem& theGisSubsystem,
        const EntranceIdType& entranceId,
        const GisObjectIdType gisObjectId) const;

    void AddEntranceQueue(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const MultiAgentGis& theMultiAgentGis,
        const EntranceIdType& entranceId);

    virtual EntranceIdType GetNearestEntranceId(const Vertex& position) = 0;
    virtual int GetHumanCapacity() = 0;

};//AgentPeopleBuffer//

class AgentBuilding : public AgentPeopleBuffer {
public:
    AgentBuilding(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const MultiAgentGis& theMultiAgentGis,
        const Building& initBuilding,
        const shared_ptr<RealStatistic>& initCongestionStatPtr,
        const shared_ptr<RealStatistic>& initPopulationStatPtr);

    const Building& ref;

protected:
    friend class MultiAgentGis;

    const double polygonSize;

    struct NextVehicleStateInfoType {
        bool vehiclesChanged;

        list<AgentResource> initialVehicleOwnerAgents;
        list<AgentResource> incomingVehicleOwnerAgents;
        list<AgentResource> outgoingVehicleOwnerAgents;
        set<AgentResource> giveUpEntranceVehicleOwnerAgents;

        NextVehicleStateInfoType()
            :
            vehiclesChanged(false)
        {}
    };

    NextVehicleStateInfoType nextVehicleStateInfo;
    
    virtual EntranceIdType GetNearestEntranceId(const Vertex& position) override;
    virtual int GetHumanCapacity() override;
};//AgentBuilding//



class AgentPark : public AgentPeopleBuffer {
public:
    AgentPark(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const MultiAgentGis& theMultiAgentGis,
        const Park& initPark,
        const shared_ptr<RealStatistic>& initCongestionStatPtr,
        const shared_ptr<RealStatistic>& initPopulationStatPtr);

    const Park& ref;

protected:
    friend class MultiAgentGis;
    
    const double polygonSize;

    virtual EntranceIdType GetNearestEntranceId(const Vertex& position) override;
    virtual int GetHumanCapacity() override;
};//AgentPark//

class AgentPoi {
public:
    AgentPoi(
        const Poi& initPoi,
        const shared_ptr<RealStatistic>& initPopulationStatPtr)
        :
        ref(initPoi),
        needToOutputStatsAtThisTime(false),
        numberPeoples(0),
        populationStatPtr(initPopulationStatPtr)
    {}

    const Poi& ref;

private:
    friend class MultiAgentGis;

    struct NextStateInfoType {
        int numberLeavePeoples;
        bool peopleChanged;
        list<AgentResource> incomingPeopleAgents;
        list<AgentResource> giveUpEntranceAgents;

        NextStateInfoType() : numberLeavePeoples(0), peopleChanged(false) {}
    };

    NextStateInfoType nextStateInfo;

    bool needToOutputStatsAtThisTime;

    int numberPeoples;

    shared_ptr<RealStatistic> populationStatPtr;
};

class AgentArea {
public:
    AgentArea(
        const Area& initArea)
        :
        ref(initArea),
        polygonSize(std::max<double>(1., ref.CalculateSize())),
        needToOutputStatsAtThisTime(false)
    {}

    const Area& ref;

private:
    friend class MultiAgentGis;

    const double polygonSize;

    struct NextStateInfoType {
        bool peopleChanged;
        list<AgentResource> incomingAgents;
        list<pair<AgentProfileType, AgentIdType> > outgoingAgents;

        NextStateInfoType() : peopleChanged(false) {}
    };

    NextStateInfoType nextStateInfo;

    bool needToOutputStatsAtThisTime;
};

class AgentLine {
public:
    AgentLine(
        const string& initLineName,
        const GisObjectIdType& initGisObjectId)
        :
        lineName(initLineName),
        gisObjectId(initGisObjectId),
        needToOutputStatsAtThisTime(false)
    {}

private:
    friend class MultiAgentGis;

    struct NextStateInfoType {
        bool peopleChanged;
        list<AgentResource> incomingAgents;
        list<pair<AgentProfileType, AgentIdType> > outgoingAgents;

        NextStateInfoType() : peopleChanged(false) {}
    };

    string lineName;

    GisObjectIdType gisObjectId;

    NextStateInfoType nextStateInfo;

    bool needToOutputStatsAtThisTime;
};

class AgentBusStop {
public:
    AgentBusStop(
        const BusStop& initBusStop,
        const shared_ptr<RealStatistic>& initCongestionStatPtr)
        :
        ref(initBusStop),
        polygonSize(1),
        needToOutputStatsAtThisTime(false),
        congestionTrace(0.),
        numberPeoples(0),
        congestionStatPtr(initCongestionStatPtr)
    {}

    const BusStop& ref;

    shared_ptr<Bus> GetBus(
        const PublicVehicleRoute& vehicleRoute,
        const set<PublicVehicleIdType>& notAvailablePublicVehicleIds,
        const AgentResource& resource) const;

    shared_ptr<Bus> GetBus(
        const PublicVehicleRoute& vehicleRoute,
        const BusTicket& ticket,
        const AgentResource& resource) const;

    double CalculatePeopleCongestion() const {
        const double basicStationCapcacity = 100;
        const double density = std::min(naviInfo.lastNumberPeoples / basicStationCapcacity, 1.);
        return density;
    }

    bool HasLeft(
        const LineIdType& lineId,
        const RouteIdType& routeId,
        const VehicleNumberType& vehicleNumber) const;

private:
    friend class MultiAgentGis;

    const double polygonSize;

    list<shared_ptr<Bus> > busPtrs;

    struct NextStateInfoType {
        int numberLeavePeoples;
        bool peopleChanged;
        list<AgentResource> incomingPeopleAgents;

        NextStateInfoType() : numberLeavePeoples(0), peopleChanged(false) {}
    };

    NextStateInfoType nextStateInfo;

    bool needToOutputStatsAtThisTime;

    TraceValue<double> congestionTrace;

    struct NavigationSystemInfo {
        int lastNumberPeoples;

        NavigationSystemInfo()
            :
            lastNumberPeoples(0)
        {}
    };

    int numberPeoples;
    NavigationSystemInfo naviInfo;

    shared_ptr<RealStatistic> congestionStatPtr;

    map<pair<LineIdType, RouteIdType>, set<VehicleNumberType> > leftVehicleNumbers;

};

class AgentStation {
public:
    AgentStation(
        const RailRoadStation& initRailRoadStation,
        const shared_ptr<RealStatistic>& initCongestionStatPtr)
        :
        ref(initRailRoadStation),
        polygonSize(std::max<double>(1., CalculatePolygonSize(ref.GetPolygon()))),
        needToOutputStatsAtThisTime(false),
        congestionTrace(0.),
        numberPeoples(0),
        congestionStatPtr(initCongestionStatPtr)
    {}

    const RailRoadStation& ref;

    shared_ptr<Train> GetTrain(
        const LineIdType& lineId,
        const RouteIdType& routeId,
        const set<PublicVehicleIdType>& notAvailablePublicVehicleIds,
        const AgentResource& resource) const;

    double CalculatePeopleCongestion() const {
        const double basicStationCapcacity = 100;
        const double density = std::min(naviInfo.lastNumberPeoples / basicStationCapcacity, 1.);
        return density;
    }

    bool HasLeft(
        const LineIdType& lineId,
        const RouteIdType& routeId,
        const VehicleNumberType& vehicleNumber) const;

private:
    friend class MultiAgentGis;

    const double polygonSize;

    list<shared_ptr<Train> > trainPtrs;

    struct NextStateInfoType {
        int numberLeavePeoples;
        bool peopleChanged;
        list<AgentResource> incomingPeopleAgents;

        NextStateInfoType() : numberLeavePeoples(0), peopleChanged(false) {}
    };

    NextStateInfoType nextStateInfo;

    bool needToOutputStatsAtThisTime;

    TraceValue<double> congestionTrace;

    struct NavigationSystemInfo {
        int lastNumberPeoples;

        NavigationSystemInfo()
            :
            lastNumberPeoples(0)
        {}
    };

    int numberPeoples;
    NavigationSystemInfo naviInfo;

    shared_ptr<RealStatistic> congestionStatPtr;

    map<pair<LineIdType, RouteIdType>, set<VehicleNumberType> > leftVehicleNumbers;
};

class AgentTaxiCenter {
public:
    AgentTaxiCenter(const GisPositionIdType& initBasePositionId)
        :
        basePositionId(initBasePositionId)
    {}

    bool HasIdleTaxi() const { return !idleTaxiPtrs.empty(); }

    shared_ptr<Taxi> GetNearestWorkingTaxi(const Vertex& position) const;

    const GisPositionIdType& GetBasePositionId() const { return basePositionId; }

private:
    friend class MultiAgentGis;

    GisPositionIdType basePositionId;

    list<shared_ptr<Taxi> > idleTaxiPtrs;
    map<AgentIdType, shared_ptr<Taxi> > workingTaxiPtrs;

    list<AgentResource> reservedAgents;
};

class MultiAgentGis {
public:
    static const string modelName;

    MultiAgentGis(
        MultiAgentSimulator* initSimulatorPtr,
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngine>& simulationEnginePtr,
        const shared_ptr<GisSubsystem>& theGisSubsystemPtr);

    void CompleteInitialization(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngine>& simulationEnginePtr,
        const AgentProfileAndTaskTable& theProfileAndTaskTable,
        const map<AgentIdType, Vertex>& taxiAgentLocations,
        const shared_ptr<PublicVehicleTable>& thePublicVehicleTablePtr);

    const TimeType GetTimestepDuration() const { return (simulatorPtr->TimeStep()); }

    void ExecuteActionsForCurrentTimestep();

    const GisSubsystem& GetSubsystem() const { return *theGisSubsystemPtr; }
    GisSubsystem& GetSubsystem() { return *theGisSubsystemPtr; }
    shared_ptr<GisSubsystem> GetSubsystemPtr() const { return theGisSubsystemPtr; }

    const AgentRoad& GetAgentRoad(const RoadIdType& roadId) const { return *roadPtrs.at(roadId); }
    const AgentIntersection& GetAgentIntersection(const IntersectionIdType& intersectionId) const
        { return *intersectionPtrs.at(intersectionId); }

    const AgentBuilding& GetAgentBuilding(const BuildingIdType& buildingId) const
        { return *buildingPtrs.at(buildingId); }

    const AgentPark& GetAgentPark(const ParkIdType& parkId) const { return *parkPtrs.at(parkId); }
    const AgentPoi& GetAgentPoi(const PoiIdType& poiId) const { return *poiPtrs.at(poiId); }
    const AgentArea& GetAgentArea(const AreaIdType& areaId) const { return *areaPtrs.at(areaId); }
    const AgentStation& GetAgentStation(const RailRoadStationIdType& stationId) const
        { return *stationPtrs.at(stationId); }

    const AgentBusStop& GetAgentBusStop(const BusStopIdType& busStopId) const
        { return *busStopPtrs.at(busStopId); }

    AgentStation& GetAgentStation(const RailRoadStationIdType& stationId) { return *stationPtrs.at(stationId); }

    bool CanReach(
        const AgentResource& resource,
        const VertexIdType& startVertexId,
        const VertexIdType& destVertexId,
        const AgentBehaviorType& behavior) const;

    //

    void SearchRoadRoute(
        const AgentResource& resource,
        const deque<VertexIdType>& startToDestVertexIds,
        const AgentBehaviorType& behavior,
        AgentRoute& roadRoute) const;

    void SearchRoadRoute(
        const AgentResource& resource,
        const VertexIdType& startVertexId,
        const VertexIdType& destVertexId,
        const AgentBehaviorType& behavior,
        AgentRoute& roadRoute) const;

    void SearchRoadRoute(
        const AgentResource& resource,
        const VertexIdType& startVertexId,
        const VertexIdType& destVertexId,
        const deque<VertexIdType>& shouldPassVertexIds,
        const AgentBehaviorType& behavior,
        AgentRoute& roadRoute) const;

    //

    void SearchRoadRoute(
        const AgentResource& resource,
        const deque<VertexIdType>& startToDestVertexIds,
        const AgentBehaviorType& behavior,
        const AgentBehaviorType& preferenceBehavior,
        const TimeType& arrivalTimeMargin,
        AgentRoute& roadRoute) const;

    void SearchRoadRoute(
        const AgentResource& resource,
        const VertexIdType& startVertexId,
        const VertexIdType& destVertexId,
        const AgentBehaviorType& behavior,
        const AgentBehaviorType& preferenceBehavior,
        const TimeType& arrivalTimeMargin,
        AgentRoute& roadRoute) const;

    void SearchRoadRoute(
        const AgentResource& resource,
        const VertexIdType& startVertexId,
        const VertexIdType& destVertexId,
        const deque<VertexIdType>& shouldPassVertexIds,
        const AgentBehaviorType& behavior,
        const AgentBehaviorType& preferenceBehavior,
        const TimeType& arrivalTimeMargin,
        AgentRoute& roadRoute) const;

    //

    void SearchRoadRoute(
        const AgentResource& resource,
        const deque<VertexIdType>& startToDestVertexIds,
        const TimeToSearchRoute& timeToSearchRoute,
        const AgentBehaviorType& behavior,
        const TimeType& currentTime,
        const TimeType& arrivalTimeMargin,
        const AgentBehaviorType& preferenceBehavior,
        AgentRouteList& roadRouteList) const;

    void SearchRoadRoute(
        const AgentResource& resource,
        const VertexIdType& startVertexId,
        const VertexIdType& destVertexId,
        const TimeToSearchRoute& timeToSearchRoute,
        const AgentBehaviorType& behavior,
        const TimeType& currentTime,
        const TimeType& arrivalTimeMargin,
        const AgentBehaviorType& preferenceBehavior,
        AgentRouteList& roadRouteList) const;

    void SearchRoadRoute(
        const AgentResource& resource,
        const VertexIdType& startVertexId,
        const VertexIdType& destVertexId,
        const deque<VertexIdType>& shouldPassVertexIds,
        const TimeToSearchRoute& timeToSearchRoute,
        const AgentBehaviorType& behavior,
        const TimeType& currentTime,
        const TimeType& arrivalTimeMargin,
        const AgentBehaviorType& preferenceBehavior,
        AgentRouteList& roadRouteList) const;

    void SearchBetterRoutesWithLinkCost(
        const AgentResource& resource,
        const VertexIdType& startVertexId,
        const VertexIdType& endVertexId,
        vector<AgentRoute>& routes) const;

    VertexIdType GetNearestVertexId(
        const GisPositionIdType& lastPositionId,
        const Vertex& position) const;

    const Vertex& GetVertex(const VertexIdType& vertexId) const;

    bool ThereAreSomeTaxiCenter() const { return !taxiCenters.empty(); }

    void CallTaxi(
        const AgentResource& resource,
        const AgentTaxiCenterIdType& requestTaxiCenterId,
        bool& succeeded,
        AgentTaxiCenterIdType& replyTaxiCenterId);

    void GiveupTaxiCall(
        const AgentResource& resource,
        const AgentTaxiCenterIdType& taxiCenterId);

    void AddVehicle(const shared_ptr<Vehicle>& vehiclePtr);

    void ReturnTaxiToStation(
        const AgentResource& resource,
        const shared_ptr<Taxi>& taxiPtr);

    void TakeOwnershiopOfVehicleLastPositionUpdate(
        const AgentResource& resource,
        const shared_ptr<Vehicle>& vehiclePtr);

    void UpdateVehicleStatus(
        const AgentResource& resource,
        const shared_ptr<Vehicle>& vehiclePtr);

    void RideOnPublicVehicle(
        const AgentResource& resource,
        const TimeType& waitTime,
        const shared_ptr<PublicVehicle>& publicVehiclePtr);

    void GetDownPublicVehicle(
        const AgentResource& resource,
        const shared_ptr<PublicVehicle>& publicVehiclePtr);

    void AddTrain(const shared_ptr<Train>& trainPtr);

    void EnterToBusStop(
        const AgentResource& resource,
        const BusStopIdType& busStopId,
        const shared_ptr<Bus>& busPtr);

    void SetCrossingPeople(
        const RoadIdType& roadId,
        const AgentResource& resource);

    void SetWaitingPeople(
        const RoadIdType& roadId,
        const AgentResource& resource);

    void SetLeavePeople(
        const RoadIdType& roadId,
        const AgentResource& resource);

    void DisconnectGisConnection();

    void PedestrianOrDriverGiveUpEntrance(
        AgentResource& resource,
        const GisPositionIdType& currentPositionId);

    void UpdatePeopleTranslationBetweenGisObjects(
        AgentResource& resource,
        const GisPositionIdType& currentPositionId,
        const GisPositionIdType& lastDesiredPositionId,
        const GisPositionIdType& desiredPostionId,
        const AgentBehaviorType& lastBehaviorType,
        const AgentBehaviorType& currentBehaviorType);

    void GetNearRouteSearchCandidateVertexIds(
        const AgentResource& resource,
        const Vertex& basePosition,
        const AgentBehaviorType& behaviorType,
        const GisPositionIdType& positionId,
        const VertexIdType& prioritizedDestVertexId,
        vector<VertexIdType>& vertexIds);


    // Use only for initialization
    void PlaceInitialVehicle(
        AgentResource& vehicleOwner,
        const BuildingIdType& buildingId);

    void StartVehicleEntranceWait(
        AgentResource& vehicleOwner,
        const BuildingIdType& buildingId);
    
    void LeaveFromParking(
        AgentResource& vehicleOwner,
        const BuildingIdType& buildingId);
    
    void TransferEntranceWaitingVehicleDriver();

    void RemoveAgentFromCurrentGisLocation(AgentResource& agent);

private:
    static const double SCENSIM_VERSION;

    MultiAgentSimulator* simulatorPtr;
    shared_ptr<GisSubsystem> theGisSubsystemPtr;
    shared_ptr<PublicVehicleTable> publicVehicleTablePtr;


    class RouteSearchQueue;

    typedef int32_t RouteSearchPointIdType;

    struct RoadConnection {
        VertexIdType vertexId;
        RoadIdType roadId;
        IntersectionIdType intersectionId;

        RoadConnection()
            :
            vertexId(INVALID_VERTEX_ID),
            roadId(INVALID_VARIANT_ID),
            intersectionId(INVALID_VERTEX_ID)
        {}

        RoadConnection(
            const VertexIdType& initVertexId,
            const RoadIdType& initRoadId,
            const IntersectionIdType& initIntersectionId)
            :
            vertexId(initVertexId),
            roadId(initRoadId),
            intersectionId(initIntersectionId)
        {}
    };

    struct RouteSearchPoint {
        VertexIdType vertexId;
        AgentRoadRouteCost totalCost;
        TimeType expectedMinDurationToDest;
        double expectedMinDistanceToDest;

        VertexIdType trackVertexId;
        RoadIdType trackRoadId;

        vector<RoadConnection> links;
        bool isSearchedVeretex; // for LRTA*

        void Initialize() {
            totalCost.totalTravelTime = -1;
            totalCost.travelDistance = 0;
            totalCost.price = 0;
            expectedMinDurationToDest = ZERO_TIME;
            expectedMinDistanceToDest = 0;
            isSearchedVeretex = false;
        }

        void SetBestRoute(
            const VertexIdType& initTrackVertexId,
            const RoadIdType& initTrackRoadId,
            const AgentRoadRouteCost& initTotalCost) {

            trackVertexId = initTrackVertexId;
            trackRoadId = initTrackRoadId;
            totalCost = initTotalCost;
        }

        bool IsFastRoute(const TimeType& duration) const {
            return ((totalCost.totalTravelTime < 0) ||
                    (duration < totalCost.totalTravelTime));
        }
        bool FoundRoute() const {
            return  (totalCost.totalTravelTime >= ZERO_TIME);
        }

        RouteSearchPoint()
            :
            vertexId(INVALID_VERTEX_ID),
            expectedMinDurationToDest(ZERO_TIME),
            expectedMinDistanceToDest(0),
            trackVertexId(INVALID_VERTEX_ID),
            trackRoadId(INVALID_VARIANT_ID)
        {}
    };

    struct RouteCache {
        RoadIdType roadId;
        float distance;

        RouteCache(
            const RoadIdType& initRoadId = INVALID_VARIANT_ID,
            const float initDistance = FLT_MAX)
            :
            roadId(initRoadId),
            distance(initDistance)
        {}

        void Clear() {
            roadId = INVALID_VARIANT_ID;
            distance = FLT_MAX;
        }

        bool IsAvailable() const {
            return (roadId != INVALID_VARIANT_ID);
        }
    };

    struct TaxiReservation {
        TimeType reservedTime;
        AgentResource resource;
        AgentTaxiCenterIdType taxiCenterId;

        TaxiReservation()
            :
            reservedTime(ZERO_TIME),
            taxiCenterId(INVALID_TAXICENTER_ID)
        {}

        TaxiReservation(
            const TimeType& initReservedTime,
            const AgentResource& initResource,
            const AgentTaxiCenterIdType& initTaxiCenterId)
            :
            reservedTime(initReservedTime),
            resource(initResource),
            taxiCenterId(initTaxiCenterId)
        {}
    };

    struct RouteLey {
        VertexIdType vertexId1;
        VertexIdType vertexId2;
        AgentBehaviorType behavior;

        RouteLey(
            const VertexIdType& initVertexId1,
            const VertexIdType& initVertexId2,
            const AgentBehaviorType& initBehavior)
            :
            vertexId1(initVertexId1),
            vertexId2(initVertexId2),
            behavior(initBehavior)
        {}

        bool operator<(const RouteLey& right) const {
            return ((vertexId1 < right.vertexId1) ||
                    (vertexId1 == right.vertexId1 &&
                     vertexId2 < right.vertexId2) ||
                    (vertexId1 == right.vertexId1 &&
                     vertexId2 == right.vertexId2 &&
                     behavior < right.behavior));
        }
    };

    struct TimeKey {
        TimeType time;
        AgentIdType agentId;

        TimeKey()
            :
            time(ZERO_TIME),
            agentId(MASTER_ANY_AGENT_ID)
        {}

        TimeKey(
            const TimeType& initTime,
            const AgentIdType& initAgentId)
            :
            time(initTime),
            agentId(initAgentId)
        {}

        bool operator<(const TimeKey& right) const {
            return ((time < right.time) ||
                    (time == right.time &&
                     agentId < right.agentId));
        }
    };

    mutable shared_array<fstream> routeTables;
    mutable vector<RouteSearchPoint> routeSearchPoints;
    mutable set<RouteLey> noRoutes;

    vector<pair<BusStopIdType, shared_ptr<Bus> > > enteredBusPtrs;

    vector<shared_ptr<Vehicle> > lastPositionUpdateVehiclePtrs;
    vector<pair<shared_ptr<Vehicle>, AgentResource> > statusUpdateVehicles;

    map<AgentIdType, TaxiReservation> taxiReservations;

    vector<shared_ptr<Taxi> > returnTaxiPtrs;
    vector<pair<AgentTaxiCenterIdType, AgentResource> > giveupTaxiCalls;

    vector<pair<TimeKey, pair<shared_ptr<PublicVehicle>, AgentResource> > > addedPublicVehicleGuests;
    vector<pair<shared_ptr<PublicVehicle>, AgentResource> > removedPublicVehicleGuests;

    vector<RoadIdType> crossingPedestrianChangeRoadIds;

    set<GisObjectIdType> peopleChangedRoadObjectIds;
    vector<RoadIdType> peopleChangedRoadIds;
    vector<BuildingIdType> peopleChangedBuildingIds;
    vector<BuildingIdType> vehiclesChangedBuildingIds;
    vector<ParkIdType> peopleChangedParkIds;
    vector<PoiIdType> peopleChangedPoiIds;
    vector<BusStopIdType> peopleChangedBusStopIds;
    vector<RailRoadStationIdType> peopleChangedStationIds;
    vector<AreaIdType> peopleChangedAreaIds;
    vector<LineIdType> peopleChangedLineIds;

    vector<AgentResource> peopleGoingToNowhere;

    typedef pair<TimeType, AgentResource> WaitAgentType;
    priority_queue_stable<WaitAgentType, false> taxiWaitAgents;

    list<shared_ptr<Vehicle> > newlyVehiclePtrs;
    vector<AgentIdType> deletedVehicleIds;
    vector<PublicVehicleIdType> deletedPublicVehicleIds;

    void RegisterRouteWay(
        const size_t threadNumber,
        const deque<VertexIdType>& vertexIds,
        const VertexIdType& destVertexId,
        const double minDistanceToDest) const;

    vector<shared_ptr<AgentRoad> > roadPtrs;
    vector<shared_ptr<AgentIntersection> > intersectionPtrs;
    vector<shared_ptr<AgentBuilding> > buildingPtrs;
    vector<shared_ptr<AgentPark> > parkPtrs;
    vector<shared_ptr<AgentPoi> > poiPtrs;
    vector<shared_ptr<AgentArea> > areaPtrs;
    vector<shared_ptr<AgentStation> > stationPtrs;
    vector<shared_ptr<AgentBusStop> > busStopPtrs;
    vector<shared_ptr<AgentLine> > linePtrs;
    vector<AgentTaxiCenter> taxiCenters;

    list<RoadIdType> vehicleRoadIds;
    set<RailRoadStationIdType> taxiHavingStationIds;

    map<AgentIdType, shared_ptr<PublicVehicle> > navigateVehiclePtrs;

    map<GisObjectIdType, vector<size_t> > numberVehicleAndPedestrians;
    TimeType navigationSystemUpdateInterval;
    TimeType congestionNavigationSystemLastUpdatedTime;

    struct TrainEntry {
        shared_ptr<Train> trainPtr;

        RailRoadStationIdType stationId;
        list<shared_ptr<Train> >::iterator iterToTrain;

        TrainEntry(const shared_ptr<Train>& initTrainPtr)
            :
            trainPtr(initTrainPtr),
            stationId(INVALID_VARIANT_ID)
        {}
    };

    list<TrainEntry> trains;

    struct StoppedBusEntry {
        shared_ptr<Bus> busPtr;
        BusStopIdType busStopId;

        list<shared_ptr<Bus> >::iterator iterToBus;

        StoppedBusEntry(
            const shared_ptr<Bus>& initBusPtr,
            const BusStopIdType& initBusStopId)
            :
            busPtr(initBusPtr),
            busStopId(initBusStopId)
        {}
    };

    list<StoppedBusEntry> stoppedBusEntry;

    void (MultiAgentGis::*SearchRoadRouteCallback)(
        const AgentResource&,
        const VertexIdType&,
        const VertexIdType&,
        const AgentBehaviorType&) const;

    void InitializeRoadMapping(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngine>& simulationEnginePtr,
        const vector<shared_ptr<const Road> >& srcRoadsPtr);

    void InitializeInterserctionMapping(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngine>& simulationEnginePtr,
        const vector<shared_ptr<const Road> >& srcRoads,
        const vector<Intersection>& srcIntersections);

    void InitializeBuildingMapping(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const MultiAgentGis& theMultiAgentGis,
        const shared_ptr<SimulationEngine>& simulationEnginePtr,
        const vector<Building>& srcBuildings);

    void InitializeParkMapping(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const MultiAgentGis& theMultiAgentGis,
        const shared_ptr<SimulationEngine>& simulationEnginePtr,
        const vector<Park>& srcParks);

    void InitializePoiMapping(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngine>& simulationEnginePtr,
        const vector<Poi>& srcPois);

    void InitializeAreaMapping(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngine>& simulationEnginePtr,
        const vector<Area>& srcAreas);

    void InitializeStationMapping(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngine>& simulationEnginePtr,
        const vector<RailRoadStation>& srcStations);

    void InitializeBusStopMapping(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngine>& simulationEnginePtr,
        const vector<BusStop>& srcBusStops);

    void InitializeOptimalConnection();
    void InitializeIntersectionTurnResournce();

    void InitializeRouteSearchPoints();
    void InitializeRouteLabelBin();
    void InitializeRouteTable(
        const string& routeCachePath,
        const bool resetCache);

    void SearchRouteWithAStarAlgorithm(
        const AgentResource& resource,
        const VertexIdType& startVertexId,
        const VertexIdType& endVertexId,
        const AgentBehaviorType& behavior) const;

    void SearchRouteWithLearningRealTimeAStarAlgorithm(
        const AgentResource& resource,
        const VertexIdType& startVertexId,
        const VertexIdType& destVertexId,
        const AgentBehaviorType& behavior) const;

    AgentRoute MakeNearbyVirtualPath(
        const VertexIdType& startVertexId,
        const VertexIdType& endVertexId) const;

    double GetRouteDistance(
        const size_t threadNumber,
        const VertexIdType& startVertexId,
        const VertexIdType& destVertexId) const;

    void CreateVirtualRoute(
        const AgentResource& resource,
        const VertexIdType& startVertexId,
        const VertexIdType& destVertexId,
        const AgentBehaviorType& behavior,
        AgentRoute& roadRoute) const;

    bool CanPassRoad(
        const AgentResource& resource,
        const RoadIdType& lastRoadId,
        const AgentBehaviorType& behavior,
        const VertexIdType& startVertexId,
        const RoadConnection& aLink) const;

    void InitializeRoutePoints(const VertexIdType& startVertexId) const;

    AgentRoadRouteCost CalculateCost(
        const AgentResource& resource,
        const AgentBehaviorType& behavior,
        const VertexIdType& startVertexId,
        const VertexIdType& endVertexId,
        const RoadIdType& roadId) const;

    double CalculateSpeedMetersPerSec(
        const AgentResource& resource,
        const AgentBehaviorType& behavior) const;

    void PickCalculatedRoute(
        const VertexIdType& startVertexId,
        const VertexIdType& destVertexId,
        AgentRoute& roadRoute) const;

    void TransferPeopleToGisObject(
        const GisObjectType& anObjectType,
        const VariantIdType& objectId,
        bool& somethingChanged);

    void UpdateEntranceQueuesStatus();
    void SyncVehicleAgentTopology();
    void SyncCrossingPedestrianTopology();
    void SyncTrainTopology(PublicVehicleTable& thePublicVehicleTable);
    void SyncPublicVehicleTopology(PublicVehicleTable& thePublicVehicleTable);
    void SyncBusTopology(PublicVehicleTable& thePublicVehicleTable);
    void SyncTaxiTopology();
    void SyncGisStat();
    void SyncGisTopology();
    void MoveCalculatedNextStatesToCurrent();

    void UpdateCongestionNavigationSystemInformation();

    VertexIdType GetNearestVertexIdForTaxiCenter(
        const GisPositionIdType& basePositionId,
        const Vertex& basePosition) const;

    bool IsTransportableTaxiCenter(
        const AgentResource& resource,
        const Vertex& customerPosition,
        const AgentTaxiCenterIdType& taxiCenterId,
        const VertexIdType customerVertexId,
        const VertexIdType destinationVertexId) const;

    void SearchNearestTransportableTaxiCenter(
        const AgentResource& resource,
        bool& succeeded,
        AgentTaxiCenterIdType& transportableTaxiCenterId);

};//MultiAgentGis//


}//namespace ScenSim

#endif
