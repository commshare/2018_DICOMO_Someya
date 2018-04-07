// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "sim.h"

#pragma warning(disable:4355)

MultiSystemsSimulator::MultiSystemsSimulator(
    const shared_ptr<ParameterDatabaseReader>& initParameterDatabaseReaderPtr,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const RandomNumberGeneratorSeedType& initRunSeed,
    const bool initRunSequentially,
    const bool initIsScenarioSettingOutputMode,
    const string& initInputConfigFileName,
    const string& initOutputConfigFileName)
    :
    MultiAgentSimulator(
        initParameterDatabaseReaderPtr,
        initSimulationEnginePtr,
        initRunSeed,
        initRunSequentially,
        initIsScenarioSettingOutputMode,
        initInputConfigFileName,
        initOutputConfigFileName),
    channelModelSetPtr(
        new ChannelModelSet(
            this,
            initParameterDatabaseReaderPtr,
            initSimulationEnginePtr,
            theGisSubsystemPtr,
            initRunSeed))
{

    (*this).CompleteSimulatorConstruction();

}//MultiSystemsSimulator()//

#pragma warning(default:4355)

void MultiSystemsSimulator::CreateNewNode(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const shared_ptr<ObjectMobilityModel>& nodeMobilityModelPtr,
    const string& nodeTypeName)
{
    unsigned int partitionIndex = 0;

    if (theParameterDatabaseReader.ParameterExists("parallelization-partition-index", nodeId)) {
        partitionIndex =
            (theParameterDatabaseReader.ReadNonNegativeInt("parallelization-partition-index", nodeId) %
             theSimulationEnginePtr->GetNumberPartitionThreads());
    }//if//

    const shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr =
        theSimulationEnginePtr->GetSimulationEngineInterface(
            theParameterDatabaseReader, nodeId, partitionIndex);

    const shared_ptr<AttachedAntennaMobilityModel> attachedAntennaMobilityModelPtr(
        new AttachedAntennaMobilityModel(nodeMobilityModelPtr));

    shared_ptr<SimNode> aSimNodePtr(
        new SimNode(
            theParameterDatabaseReader,
            theGlobalNetworkingObjectBag,
            simulationEngineInterfacePtr,
            nodeId,
            runSeed,
            theGisSubsystemPtr,
            attachedAntennaMobilityModelPtr,
            channelModelSetPtr));

    (*this).AddCommunicationNode(aSimNodePtr);

    simNodePtrs[nodeId] = aSimNodePtr;

}//CreateNewNode//

void MultiSystemsSimulator::CompleteSimulatorConstruction()
{
    const ParameterDatabaseReader& theParameterDatabaseReader = (*theParameterDatabaseReaderPtr);

    set<NodeIdType> setOfNodeIds;
    theParameterDatabaseReader.MakeSetOfAllCommNodeIds(setOfNodeIds);

    typedef set<NodeIdType>::const_iterator IterType;

    for(IterType iter = setOfNodeIds.begin();
        iter != setOfNodeIds.end(); iter++) {

        const NodeIdType nodeId = (*iter);

        assert(nodeId <= MAX_COMMUNICATION_NODEID);

        unsigned int partitionIndex = 0;

        if (theParameterDatabaseReader.ParameterExists("parallelization-partition-index", nodeId)) {
            partitionIndex =
                (theParameterDatabaseReader.ReadNonNegativeInt("parallelization-partition-index", nodeId) %
                 theSimulationEnginePtr->GetNumberPartitionThreads());
        }//if//

        shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr(
            theSimulationEnginePtr->GetSimulationEngineInterface(
                theParameterDatabaseReader, nodeId, partitionIndex));

        shared_ptr<ObjectMobilityModel> nodeMobilityModelPtr =
            CreateAntennaMobilityModel(
                theParameterDatabaseReader,
                nodeId,
                ScenSim::nullInstanceId,
                mobilitySeed,
                mobilityFileCache,
                theGisSubsystemPtr);

        if ((*this).IsEqualToAgentId(nodeId)) {
        
            (*this).CreateCommunicationNodeAtWakeupTimeFor(nodeId);
        }
        else {
            
            const TimeType currentSimTime = simulationEngineInterfacePtr->CurrentTime();
            
            TimeType creationTime = ZERO_TIME;
            TimeType deletionTime = INFINITE_TIME;

            if (nodeMobilityModelPtr != nullptr) {
                creationTime = nodeMobilityModelPtr->GetCreationTime();
                deletionTime = nodeMobilityModelPtr->GetDeletionTime();
            }//if//

            if (creationTime <= currentSimTime) {

                (*this).CreateNewNode(theParameterDatabaseReader, nodeId, nodeMobilityModelPtr);

            }
            else if (creationTime < deletionTime &&
                     creationTime < INFINITE_TIME) {

                simulationEngineInterfacePtr->ScheduleEvent(
                    unique_ptr<SimulationEvent>(new NodeEnterEvent(this, nodeId, nodeMobilityModelPtr)), creationTime);
            }//if//

            if (creationTime < deletionTime &&
                deletionTime < INFINITE_TIME) {

                simulationEngineInterfacePtr->ScheduleEvent(
                    unique_ptr<SimulationEvent>(new NodeLeaveEvent(this, nodeId)), deletionTime);
            }//if//
        }//if//
    }//for//

    (*this).CheckTheNecessityOfMultiAgentSupport();

    (*this).SetupStatOutputFile();
    (*this).ExecuteTimestepBasedEvent();
}//CompleteSimulatorConstruction//

void MultiSystemsSimulator::DeleteNode(const NodeIdType& nodeId)
{
    NetworkSimulator::DeleteNode(nodeId);

    simNodePtrs.erase(nodeId);
}//DeleteNode//

unsigned int MultiSystemsSimulator::LookupInterfaceIndex(
    const NodeIdType& nodeId,
    const InterfaceIdType& interfaceName) const
{
    typedef map<NodeIdType, shared_ptr<SimNode> > ::const_iterator IterType;

    IterType iter = simNodePtrs.find(nodeId);
    
    if (iter == simNodePtrs.end() ||
        nodes.find(nodeId) == nodes.end()) {
        cerr << "Error: Network Node Id: " << nodeId << " Not Found." << endl;
        exit(1);
    }//if

    return (*iter).second->GetAntennaNumber(interfaceName);

}//LookupInterfaceIndex//
