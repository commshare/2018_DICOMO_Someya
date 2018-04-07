// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef MUTIAGENT_NETWORKINTERFACE_H
#define MUTIAGENT_NETWORKINTERFACE_H

#include "multiagent_agentsim.h"

namespace MultiAgent {

//--------------------------------------------------------------------------------------------------
// Networking Interface.
// Inherit AgentNetworkNode for Network Node Interface

class AgentCommunicationNode : public NetworkNode {
public:
    AgentCommunicationNode(
        const ParameterDatabaseReader& initParameterDatabaseReader,
        const GlobalNetworkingObjectBag& initGlobalNetworkingObjectBag,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
        const NodeIdType& initNodeId,
        const RandomNumberGeneratorSeedType& initRunSeed)
        :
        NetworkNode(
            initParameterDatabaseReader,
            initGlobalNetworkingObjectBag,
            initSimulationEngineInterfacePtr,
            initNodeMobilityModelPtr,
            initNodeId,
            initRunSeed),
        nodeMobilityModelPtr(initNodeMobilityModelPtr)
    {}

    bool IsAttached() const { return nodeMobilityModelPtr != nullptr; }

    bool IsAvailable() const { return resource.IsAvailable(); }

    Vertex GetPosition() const { return resource.Position(); }

    string GetProfileName() const { return resource.GetProfileName(); }

    void SetDestination(
        const Vertex& position,
        const bool byCommunication = true) {
        resource.SetDestination(position, byCommunication);
    }

    void SetDestination(
        const GisPositionIdType& positionId,
        const bool byCommunication = true) {
        resource.SetDestination(positionId, resource.Position(), byCommunication);
    }

    void AddUnreachablePositions(
        const list<GisPositionIdType>& unreachablePositionIds,
        const bool byCommunication = true) {
        resource.AddUnreachablePositions(unreachablePositionIds, byCommunication);
    }

    void RecalculateRoute() { resource.RecalculateRoute(); }

    virtual void ArrivedAtDeadEndNotification() {}
    virtual void ArrivedAtVertexNotification(const VertexIdType& vertexId, const Vertex& postion) {}
    virtual void ArrivedAtDestinationNotification() {}
    virtual void EnteredToDestinationNotification() {}
    virtual void ArrivedAtGisPositionNotification(const GisPositionIdType& positionId) {}
    virtual void UnreachableDestinationNotification(const GisPositionIdType& positionId) {}

protected:
    shared_ptr<ObjectMobilityModel> nodeMobilityModelPtr;
    AgentResource resource;

private:
    virtual void Attach(const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr) { nodeMobilityModelPtr = initNodeMobilityModelPtr; }
    virtual void Detach() { nodeMobilityModelPtr.reset(); }

    friend class Agent;
    friend class MultiAgentSimulator;
};

} //namespace

#endif
