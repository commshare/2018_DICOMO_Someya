// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef MULTIAGENT_ROUTESEARCH_H
#define MULTIAGENT_ROUTESEARCH_H

#include "multiagent_agentsim.h"
#include "multiagent_gis.h"
#include "multiagent_publicvehicle.h"

namespace MultiAgent {

class AgentRouteSearchSubsystem {
public:
    AgentRouteSearchSubsystem(
        const shared_ptr<MultiAgentGis>& initAgentGisPtr,
        const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr)
        :
        theAgentGisPtr(initAgentGisPtr),
        thePublicVehicleTablePtr(initPublicVehicleTablePtr)
    {}

    void SearchRouteCandidates(
        const AgentResource& resource,
        const VertexIdType& startVertexId,
        const VertexIdType& destVertexId,
        const deque<VertexIdType>& shouldPassVertexIds,
        const TimeToSearchRoute& timeToSearchRoute,
        const AgentBehaviorType& preferenceBehavior,
        const set<AgentBehaviorType>& notAvailableBehavorTypes,
        vector<list<AgentRouteList> >& routeCandidatesPerOrder) const;
    
private:

    void SearchPublicVehicleRoutes(
        const AgentResource& resource,
        const VertexIdType& startVertexId,
        const VertexIdType& destVertexId,
        const TimeToSearchRoute& timeToSearchRoute,
        const AgentBehaviorType& preferenceBehavior,
        const set<AgentBehaviorType>& notAvailableBehavorTypes,
        map<pair<VertexIdType, VertexIdType>, AgentRoute>& routeCache,
        vector<list<AgentRouteList> >& routeCandidatesPerOrder) const;
    
    void SearchAllPublicVehicleRoutes(
        const AgentResource& resource,
        const VertexIdType& startVertexId,
        const VertexIdType& destVertexId,
        const TimeToSearchRoute& timeToSearchRoute,
        const AgentBehaviorType& preferenceBehavior,
        const set<AgentBehaviorType>& notAvailableBehavorTypes,
        map<pair<VertexIdType, VertexIdType>, AgentRoute>& routeCache,
        vector<list<AgentRouteList> >& routeCandidatesPerOrder) const;

    void SearchPublicVehicleRoute(
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
        vector<list<AgentRouteList> >& routeCandidatesPerOrder) const;
        
    void PushRouteIfBetterRoute(
        const AgentResource& resource,
        const vector<list<AgentRouteList> >& pushRouteCandidatesPerOrder,
        vector<list<AgentRouteList> >& routeCandidatesPerOrder) const;

    shared_ptr<MultiAgentGis> theAgentGisPtr;
    shared_ptr<PublicVehicleTable> thePublicVehicleTablePtr;
};

}//namespace ScenSim

#endif
