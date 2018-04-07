// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "multiagent_routesearch.h"
#include "multiagent_gis.h"
#include "multiagent_publicvehicle.h"

namespace MultiAgent {

void AgentRouteSearchSubsystem::SearchRouteCandidates(
    const AgentResource& resource,
    const VertexIdType& startVertexId,
    const VertexIdType& destVertexId,
    const deque<VertexIdType>& shouldPassVertexIds,
    const TimeToSearchRoute& timeToSearchRoute,
    const AgentBehaviorType& preferenceBehavior,
    const set<AgentBehaviorType>& notAvailableBehavorTypes,
    vector<list<AgentRouteList> >& routeCandidatesPerOrder) const
{
    routeCandidatesPerOrder.clear();
    routeCandidatesPerOrder.resize(NUMBER_AGENT_ROUTE_ORDERS);

    map<pair<VertexIdType, VertexIdType>, AgentRoute > routeCache;
    vector<list<AgentRouteList> > publicVehicleCandidatesPerOrder;

    (*this).SearchPublicVehicleRoutes(
            resource,
            startVertexId,
            destVertexId,
            timeToSearchRoute,
            preferenceBehavior,
            notAvailableBehavorTypes,
            routeCache,
            publicVehicleCandidatesPerOrder);

    (*this).PushRouteIfBetterRoute(
        resource,
        publicVehicleCandidatesPerOrder,
        routeCandidatesPerOrder);

    const TimeType currentTime = resource.CurrentTime();
    const TimeType arrivalTimeMargin = 60*SECOND;

    list<AgentRouteList> roadRouteCandidates;

    if (notAvailableBehavorTypes.find(AGENT_BEHAVIOR_PEDESTRIAN) ==
        notAvailableBehavorTypes.end()) {

        roadRouteCandidates.push_back(AgentRouteList());
        theAgentGisPtr->SearchRoadRoute(
            resource,
            startVertexId,
            destVertexId,
            shouldPassVertexIds,
            timeToSearchRoute,
            AGENT_BEHAVIOR_PEDESTRIAN,
            currentTime,
            arrivalTimeMargin,
            preferenceBehavior,
            roadRouteCandidates.back());
    }


    if (notAvailableBehavorTypes.find(AGENT_BEHAVIOR_BICYCLE) ==
        notAvailableBehavorTypes.end()) {

        if (resource.CanUseBicycle()) {
            roadRouteCandidates.push_back(AgentRouteList());
            theAgentGisPtr->SearchRoadRoute(
                resource,
                startVertexId,
                destVertexId,
                shouldPassVertexIds,
                timeToSearchRoute,
                AGENT_BEHAVIOR_BICYCLE,
                currentTime,
                arrivalTimeMargin,
                preferenceBehavior,
                roadRouteCandidates.back());
        }
    }


    if (notAvailableBehavorTypes.find(AGENT_BEHAVIOR_TAXI) ==
        notAvailableBehavorTypes.end()) {

        if (theAgentGisPtr->ThereAreSomeTaxiCenter()) {
            roadRouteCandidates.push_back(AgentRouteList());
            theAgentGisPtr->SearchRoadRoute(
                resource,
                startVertexId,
                destVertexId,
                shouldPassVertexIds,
                timeToSearchRoute,
                AGENT_BEHAVIOR_TAXI,
                currentTime,
                arrivalTimeMargin,
                preferenceBehavior,
                roadRouteCandidates.back());

            // Discard the found route if too short travel
            if (roadRouteCandidates.back().totalCost.TravelDistance() < resource.MinVehicleRouteDistance()) {

                roadRouteCandidates.pop_back();
            }
        }
    }


    if (notAvailableBehavorTypes.find(AGENT_BEHAVIOR_VEHICLE) ==
        notAvailableBehavorTypes.end()) {

        if (resource.HasCar()) {
            const Vehicle& vehicle = resource.GetVehicle();
            const VertexIdType vehicleVertexId = vehicle.GetVertexId();

            AgentRouteList pedestrianRoute;
            bool canReachVehicle;

            if (vehicle.GetPosition().DistanceTo(resource.Position()) <= resource.AcceptableWalkDistanceToCar()) {

                if (startVertexId != vehicleVertexId) {
                    theAgentGisPtr->SearchRoadRoute(
                        resource,
                        startVertexId,
                        vehicleVertexId,
                        shouldPassVertexIds,
                        timeToSearchRoute,
                        AGENT_BEHAVIOR_PEDESTRIAN,
                        currentTime,
                        arrivalTimeMargin,
                        AGENT_BEHAVIOR_ANY,
                        pedestrianRoute);

                    canReachVehicle = (!pedestrianRoute.IsEmpty());
                } else {
                    canReachVehicle = true;
                }
            } else {
                canReachVehicle = false;
            }

            if (canReachVehicle) {
                AgentRouteList vehicleRoute;

                theAgentGisPtr->SearchRoadRoute(
                    resource,
                    vehicleVertexId,
                    destVertexId,
                    shouldPassVertexIds,
                    timeToSearchRoute,
                    AGENT_BEHAVIOR_VEHICLE,
                    pedestrianRoute.totalCost.ArrivalTime(),
                    ZERO_TIME,
                    preferenceBehavior,
                    vehicleRoute);

                if (!vehicleRoute.IsEmpty()) {

                    if (vehicleRoute.totalCost.TravelDistance() >= resource.MinVehicleRouteDistance()) {

                        roadRouteCandidates.push_back(AgentRouteList());

                        AgentRouteList& vehicleRouteList = roadRouteCandidates.back();

                        vehicleRouteList.PushRouteList(pedestrianRoute);
                        vehicleRouteList.PushRouteList(vehicleRoute);
                    }
                }
            }
        }
    }

    vector<list<AgentRouteList> > roadRouteCandidatesPerOrder(
        NUMBER_AGENT_ROUTE_ORDERS,
        roadRouteCandidates);

    (*this).PushRouteIfBetterRoute(
        resource,
        roadRouteCandidatesPerOrder,
        routeCandidatesPerOrder);

    bool foundARoute = false;

    for(size_t i = 0; (!foundARoute && i < routeCandidatesPerOrder.size()); i++) {
        if (!routeCandidatesPerOrder[i].empty()) {
            foundARoute = true;
        }
    }

    if (!foundARoute) {
        vector<list<AgentRouteList> > allPublicVehicleCandidatesPerOrder;

        (*this).SearchAllPublicVehicleRoutes(
            resource,
            startVertexId,
            destVertexId,
            timeToSearchRoute,
            preferenceBehavior,
            notAvailableBehavorTypes,
            routeCache,
            allPublicVehicleCandidatesPerOrder);

        (*this).PushRouteIfBetterRoute(
            resource,
            allPublicVehicleCandidatesPerOrder,
            routeCandidatesPerOrder);
    }
}

void AgentRouteSearchSubsystem::PushRouteIfBetterRoute(
    const AgentResource& resource,
    const vector<list<AgentRouteList> >& pushRouteCandidatesPerOrder,
    vector<list<AgentRouteList> >& routeCandidatesPerOrder) const
{
    const size_t numberMaxRoutes = std::max<size_t>(1, static_cast<size_t>(resource.NumberMaxRouteCandidates()));

    for(size_t i = 0; i < pushRouteCandidatesPerOrder.size(); i++) {

        if (i >= routeCandidatesPerOrder.size()) {
            return;
        }

        const AgentRouteOrderType routeOrder = AgentRouteOrderType(i);
        const list<AgentRouteList>& pushRoutes = pushRouteCandidatesPerOrder[i];

        list<AgentRouteList>& routes = routeCandidatesPerOrder[i];

        typedef list<AgentRouteList>::iterator IterType;
        typedef list<AgentRouteList>::const_iterator CostIterType;

        for(CostIterType pushIter = pushRoutes.begin();
            pushIter != pushRoutes.end(); pushIter++) {

            const AgentRouteList& pushRouteCandidate = (*pushIter);

            if (pushRouteCandidate.IsEmpty()) {
                continue;
            }

            IterType iter = routes.begin();
            bool isSameRoute = false;

            for(; iter != routes.end(); iter++) {
                const AgentRouteList& route = (*iter);

                if (pushRouteCandidate.totalCost.IsEqual(route.totalCost, routeOrder)) {
                    if (route.totalCost == pushRouteCandidate.totalCost) {
                        isSameRoute = true;
                        break;
                    }
                }

                if (pushRouteCandidate.totalCost.IsBetterThan(route.totalCost, routeOrder)) {
                    break;
                }
            }

            if (isSameRoute) {
                continue;
            }

            routes.insert(iter, pushRouteCandidate);

            if (routes.size() > numberMaxRoutes) {
                routes.pop_back();
            }
        }
    }
}

void AgentRouteSearchSubsystem::SearchPublicVehicleRoutes(
    const AgentResource& resource,
    const VertexIdType& startVertexId,
    const VertexIdType& destVertexId,
    const TimeToSearchRoute& timeToSearchRoute,
    const AgentBehaviorType& preferenceBehavior,
    const set<AgentBehaviorType>& notAvailableBehavorTypes,
    map<pair<VertexIdType, VertexIdType>, AgentRoute>& routeCache,
    vector<list<AgentRouteList> >& routeCandidatesPerOrder) const
{
    routeCandidatesPerOrder.clear();
    routeCandidatesPerOrder.resize(NUMBER_AGENT_ROUTE_ORDERS);

    const Vertex startVertex = theAgentGisPtr->GetVertex(startVertexId);
    const Vertex destVertex =theAgentGisPtr->GetVertex(destVertexId);

    const double maxWalkDistance = std::min(resource.AcceptableWalkDistanceToStop(), startVertex.DistanceTo(destVertex)/2.);
    const size_t numberMaxPublicVehicleRoutes = 2;
    const size_t numberMaxBusStopCandidatesForStopType = 3;
    const size_t numberMaxTransferTimes = 10;
    const bool searchFromNearStopPair = true;

    vector<pair<StopIdType, StopIdType> > startAndDestStopIds;

    thePublicVehicleTablePtr->FindStartAndStopIds(
        startVertex,
        destVertex,
        maxWalkDistance,
        numberMaxBusStopCandidatesForStopType,
        searchFromNearStopPair,
        notAvailableBehavorTypes,
        startAndDestStopIds);

    for(size_t i = 0; i < startAndDestStopIds.size(); i++) {
        const pair<StopIdType, StopIdType>& startAndDestStopId = startAndDestStopIds[i];
        const StopIdType& startStopId = startAndDestStopId.first;
        const StopIdType& destStopId = startAndDestStopId.second;

        assert(startStopId != destStopId);
        vector<list<AgentRouteList> > publicVehicleCandidatesPerOrder;

        (*this).SearchPublicVehicleRoute(
            resource,
            startVertexId,
            destVertexId,
            startStopId,
            destStopId,
            timeToSearchRoute,
            preferenceBehavior,
            notAvailableBehavorTypes,
            numberMaxPublicVehicleRoutes,
            numberMaxTransferTimes,
            routeCache,
            publicVehicleCandidatesPerOrder);

        (*this).PushRouteIfBetterRoute(
            resource,
            publicVehicleCandidatesPerOrder,
            routeCandidatesPerOrder);
    }
}

void AgentRouteSearchSubsystem::SearchAllPublicVehicleRoutes(
    const AgentResource& resource,
    const VertexIdType& startVertexId,
    const VertexIdType& destVertexId,
    const TimeToSearchRoute& timeToSearchRoute,
    const AgentBehaviorType& preferenceBehavior,
    const set<AgentBehaviorType>& notAvailableBehavorTypes,
    map<pair<VertexIdType, VertexIdType>, AgentRoute>& routeCache,
    vector<list<AgentRouteList> >& routeCandidatesPerOrder) const
{
    routeCandidatesPerOrder.clear();
    routeCandidatesPerOrder.resize(NUMBER_AGENT_ROUTE_ORDERS);

    const Vertex startVertex = theAgentGisPtr->GetVertex(startVertexId);
    const Vertex destVertex =theAgentGisPtr->GetVertex(destVertexId);
    const Rectangle& entireRect = theAgentGisPtr->GetSubsystem().GetEntireRect();

    const double maxWalkDistance = entireRect.GetDiagonalLength();
    const size_t numberMaxPublicVehicleRoutes = INT_MAX;
    const size_t numberMaxBusStopCandidatesForStopType = INT_MAX;
    const size_t numberMaxTransferTimes = INT_MAX;
    const bool searchFromNearStopPair = false;

    vector<pair<StopIdType, StopIdType> > startAndDestStopIds;

    thePublicVehicleTablePtr->FindStartAndStopIds(
        startVertex,
        destVertex,
        maxWalkDistance,
        numberMaxBusStopCandidatesForStopType,
        searchFromNearStopPair,
        notAvailableBehavorTypes,
        startAndDestStopIds);

    for(size_t i = 0; i < startAndDestStopIds.size(); i++) {
        const pair<StopIdType, StopIdType>& startAndDestStopId = startAndDestStopIds[i];
        const StopIdType& startStopId = startAndDestStopId.first;
        const StopIdType& destStopId = startAndDestStopId.second;

        assert(startStopId != destStopId);
        vector<list<AgentRouteList> > publicVehicleCandidatesPerOrder;

        (*this).SearchPublicVehicleRoute(
            resource,
            startVertexId,
            destVertexId,
            startStopId,
            destStopId,
            timeToSearchRoute,
            preferenceBehavior,
            notAvailableBehavorTypes,
            numberMaxPublicVehicleRoutes,
            numberMaxTransferTimes,
            routeCache,
            publicVehicleCandidatesPerOrder);

        (*this).PushRouteIfBetterRoute(
            resource,
            publicVehicleCandidatesPerOrder,
            routeCandidatesPerOrder);
    }
}

void AgentRouteSearchSubsystem::SearchPublicVehicleRoute(
    const AgentResource& resource,
    const VertexIdType& startVertexId,
    const VertexIdType& destVertexId,
    const StopIdType& startStopId,
    const StopIdType& destStopId,
    const TimeToSearchRoute& timeToSearchRoute,
    const AgentBehaviorType& preferenceBehavior,
    const set<AgentBehaviorType>& notAvailableBehavorTypes,
    const size_t numberMaxRoutes,
    const size_t numberMaxTransferTimes,
    map<pair<VertexIdType, VertexIdType>, AgentRoute>& routeCache,
    vector<list<AgentRouteList> >& routeCandidatesPerOrder) const
{
    routeCandidatesPerOrder.clear();
    routeCandidatesPerOrder.resize(NUMBER_AGENT_ROUTE_ORDERS);

    assert(startStopId != destStopId);

    const Vertex& startVertex = theAgentGisPtr->GetVertex(startVertexId);
    const Vertex& destVertex = theAgentGisPtr->GetVertex(destVertexId);

    VertexIdType startStopVertexId;

    if (thePublicVehicleTablePtr->IsVertexOfStop(startStopId, startVertexId)) {
        startStopVertexId = startVertexId;
    } else {
        startStopVertexId = thePublicVehicleTablePtr->GetNearestEntranceVertexId(startStopId, startVertex);
    }

    const VertexIdType& destStopVertexId = thePublicVehicleTablePtr->GetNearestEntranceVertexId(destStopId, destVertex);

    const TimeType walkTimeMargin = 30*SECOND;

    pair<VertexIdType, VertexIdType> startToStartStopId(startVertexId, startStopVertexId);
    pair<VertexIdType, VertexIdType> destStopTodestId(destStopVertexId, destVertexId);

    if (startVertexId != startStopVertexId) {

        if (routeCache.find(startToStartStopId) == routeCache.end()) {
            theAgentGisPtr->SearchRoadRoute(
                resource,
                startVertexId,
                startStopVertexId,
                AGENT_BEHAVIOR_PEDESTRIAN,
                AGENT_BEHAVIOR_ANY,
                walkTimeMargin,
                routeCache[startToStartStopId]);
        }

        if (routeCache[startToStartStopId].IsEmpty()) {
            return;
        }
    }

    if (destStopVertexId != destVertexId) {

        if (routeCache.find(destStopTodestId) == routeCache.end()) {
            theAgentGisPtr->SearchRoadRoute(
                resource,
                destStopVertexId,
                destVertexId,
                AGENT_BEHAVIOR_PEDESTRIAN,
                AGENT_BEHAVIOR_ANY,
                walkTimeMargin,
                routeCache[destStopTodestId]);
        }

        if (routeCache[destStopTodestId].IsEmpty()) {
            return;
        }
    }

    const AgentRoute& origStartRoute = routeCache[startToStartStopId];
    const AgentRoute& origDestRoute = routeCache[destStopTodestId];
    const TimeType lastWalkTime = origDestRoute.totalCost.TravelTime();
    const TimeType firstWalkTime = origStartRoute.totalCost.TravelTime();;
    const TimeType currentTime = resource.CurrentTime();

    TimeToSearchRoute timeToSearchRouteWithWalk(timeToSearchRoute);
    timeToSearchRouteWithWalk.OffsetArrivalTime(-lastWalkTime);
    timeToSearchRouteWithWalk.OffsetDepartureTime(firstWalkTime);

    vector<vector<shared_ptr<AgentRoute> > > routeCandidatePtrsPerOrder;

    thePublicVehicleTablePtr->SearchPublicVehicleRoute(
        resource,
        startStopId,
        destStopId,
        timeToSearchRouteWithWalk,
        preferenceBehavior,
        notAvailableBehavorTypes,
        numberMaxRoutes,
        numberMaxTransferTimes,
        routeCandidatePtrsPerOrder);

    routeCandidatesPerOrder.resize(routeCandidatePtrsPerOrder.size());

    typedef vector<shared_ptr<AgentRoute> >::const_iterator IterType;

    for(size_t i = 0; i < routeCandidatePtrsPerOrder.size(); i++) {
        const vector<shared_ptr<AgentRoute> >& publicVehicleRouteCandidatePtrs =
            routeCandidatePtrsPerOrder[i];

        list<AgentRouteList>& routeCandidates = routeCandidatesPerOrder[i];

        for(IterType iter = publicVehicleRouteCandidatePtrs.begin();
            iter != publicVehicleRouteCandidatePtrs.end(); iter++) {

            const shared_ptr<AgentRoute>& routePtr = (*iter);

            assert(!routePtr->publicVehicleRoutes.empty());
            assert(routePtr->publicVehicleRoutes.front().lineId != TRANSFER_LINE_ID);
            assert(routePtr->publicVehicleRoutes.back().lineId != TRANSFER_LINE_ID);
            assert(routePtr->GetStartStopId() == startStopId);
            assert(routePtr->GetDestStopId() == destStopId);

            const TimeType firstStopDepartureTime = routePtr->publicVehicleRoutes.front().departureTime;
            const TimeType lastStopArrivalTime = routePtr->publicVehicleRoutes.back().arrivalTime;

            TimeType startTime = firstStopDepartureTime;

            if (!origStartRoute.IsEmpty()) {
                startTime -= origStartRoute.totalCost.TravelTime();
            } else {
                startTime = currentTime; //at sttop
            }

            if (startTime < currentTime) {
                continue;
            }

            routeCandidates.push_back(AgentRouteList());
            AgentRouteList& routeList = routeCandidates.back();

            routeList.startTime = startTime;

            if (!origStartRoute.IsEmpty()) {
                shared_ptr<AgentRoute> startRoutePtr(new AgentRoute());

                *startRoutePtr = origStartRoute;
                startRoutePtr->totalCost.values[AGENT_ROUTE_COST_ARRIVAL_TIME] = double(firstStopDepartureTime)/SECOND;

                routeList.startTime -= startRoutePtr->totalCost.TravelTime();

                routeList.PushRoute(startRoutePtr);
            }

            routeList.PushRoute(routePtr);

            if (!origDestRoute.IsEmpty()) {
                shared_ptr<AgentRoute> destRoutePtr(new AgentRoute());

                *destRoutePtr = origDestRoute;
                destRoutePtr->totalCost.values[AGENT_ROUTE_COST_ARRIVAL_TIME] =
                    double(lastStopArrivalTime + destRoutePtr->totalCost.TravelTime())/SECOND;

                routeList.PushRoute(destRoutePtr);
            }
        }
    }
}

}//namespace ScenSim
