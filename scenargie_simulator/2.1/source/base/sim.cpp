// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>

#include "scenargiesim.h"
#include "scensim_gui_interface.h"
#include "scensim_mobility.h"
#include "scensim_routing_support.h"
#include "aloha_mac.h"

using std::string;
using std::shared_ptr;
using std::map;
using std::cerr;
using std::endl;
using std::ofstream;

using ScenSim::NodeIdType;
using ScenSim::InterfaceOrInstanceIdType;
using ScenSim::ParameterDatabaseReader;
using ScenSim::SimulationEngine;
using ScenSim::SimulationEngineInterface;
using ScenSim::TimeType;

using ScenSim::NetworkSimulator;
using ScenSim::NetworkNode;
using ScenSim::InterfaceIdType;
using ScenSim::SimplePropagationModel;
using ScenSim::SimplePropagationModelForNode;
using ScenSim::SimplePropagationLossCalculationModel;
using ScenSim::RandomNumberGeneratorSeedType;
using ScenSim::AntennaModel;
using ScenSim::AttachedAntennaMobilityModel;
using ScenSim::ObjectMobilityModel;
using ScenSim::ObjectMobilityPosition;
using ScenSim::RandomNumberGeneratorSeedType;
using ScenSim::GlobalNetworkingObjectBag;
using ScenSim::GisSubsystem;
using ScenSim::PropagationInformationType;
using ScenSim::PropagationStatisticsType;
using ScenSim::MacLayer;
using ScenSim::MakeLowerCaseString;
using ScenSim::ConvertTimeToStringSecs;
using ScenSim::SquaredXYDistanceBetweenVertices;
using ScenSim::Vertex;
using ScenSim::GrabArgvForLicensing;
using ScenSim::MainFunctionArgvProcessingBasicParallelVersion1;
using ScenSim::GuiInterfacingSubsystem;
using ScenSim::SetupRoutingProtocol;

class SimNode;

typedef Aloha::AlohaPhy::PropFrameType FrameType;
typedef SimplePropagationModel<FrameType> PropagationModelType;
typedef SimplePropagationModelForNode<FrameType> PropagationInterfaceType;

class BasicNetworkSimulator : public NetworkSimulator {
public:
    BasicNetworkSimulator(
        const shared_ptr<ParameterDatabaseReader>& initParameterDatabaseReaderPtr,
        const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
        const RandomNumberGeneratorSeedType& initRunSeed,
        const bool initRunSequentially);

    ~BasicNetworkSimulator() { (*this).DeleteAllNodes(); }

    virtual void CreateNewNode(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& nodeId,
        const shared_ptr<ObjectMobilityModel>& nodeMobilityModelPtr,
        const string& nodeTypeName = "");

    shared_ptr<PropagationModelType> GetPropagationModel(
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId);

private:
    map<InterfaceOrInstanceIdType, shared_ptr<PropagationModelType> > propagationModelPtrs;

};//BasicNetworkSimulator//


inline
BasicNetworkSimulator::BasicNetworkSimulator(
    const shared_ptr<ParameterDatabaseReader>& initParameterDatabaseReaderPtr,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const RandomNumberGeneratorSeedType& initRunSeed,
    const bool initRunSequentially)
    :
    NetworkSimulator(
        initParameterDatabaseReaderPtr,
        initSimulationEnginePtr,
        initRunSeed,
        initRunSequentially)
{
    (*this).CompleteSimulatorConstruction();
}




class SimNode : public NetworkNode {
public:
    SimNode(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const GlobalNetworkingObjectBag& initGlobalNetworkingObjectBag,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeIdType& initNodeId,
        const RandomNumberGeneratorSeedType& initRunSeed,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
        BasicNetworkSimulator& simulator);

    ~SimNode() {}

    virtual const ObjectMobilityPosition GetCurrentLocation() const {
        const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
        ObjectMobilityPosition position;
        nodeMobilityModelPtr->GetPositionForTime(currentTime, position);
        return position;
    }

    virtual void CalculatePathlossToLocation(
        const PropagationInformationType& informationType,
        const unsigned int interfaceNumber,
        const double& positionXMeters,
        const double& positionYMeters,
        const double& positionZMeters,
        PropagationStatisticsType& propagationStatistics) const;

    virtual void CalculatePathlossToNode(
        const PropagationInformationType& informationType,
        const unsigned int interfaceIndex,
        const ObjectMobilityPosition& rxAntennaPosition,
        const ObjectMobilityModel::MobilityObjectIdType& rxObjectId,
        const AntennaModel& rxAntennaModel,
        PropagationStatisticsType& propagationStatistics) const;

    virtual bool HasAntenna(const InterfaceIdType& channelId) const {

        typedef  map<InterfaceIdType, unsigned int>::const_iterator IterType;
        IterType iter = interfaceNumberToChannel.find(channelId);

        if (iter == interfaceNumberToChannel.end()) {
            return false;
        }

        return (interfaces.at((*iter).second).antennaModelPtr != nullptr);
    }//HasAntenna//

    virtual shared_ptr<AntennaModel> GetAntennaModelPtr(const unsigned int interfaceIndex) const {
        return interfaces.at(interfaceIndex).antennaModelPtr;
    }

    virtual ObjectMobilityPosition GetAntennaLocation(const unsigned int interfaceIndex) const {

        const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

        ObjectMobilityPosition position;
        interfaces.at(interfaceIndex).antennaMobilityModelPtr->GetPositionForTime(
            currentTime, position);
        return (position);

    }//GetAntennaLocation//

    void DisconnectPropInterfaces();

    void SetupMacAndPhyInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const shared_ptr<PropagationModelType>& propagationModelPtr,
        const InterfaceIdType& interfaceId,
        const unsigned int interfaceNumber);

private:
    const shared_ptr<ObjectMobilityModel> nodeMobilityModelPtr;

    struct InterfaceType {
        shared_ptr<MacLayer> macPtr;
        shared_ptr<AntennaModel> antennaModelPtr;
        shared_ptr<ObjectMobilityModel> antennaMobilityModelPtr;
        shared_ptr<PropagationInterfaceType> propagationInterfacePtr;
    };

    map<InterfaceIdType, unsigned int> interfaceNumberToChannel;
    vector<InterfaceType> interfaces;
};



inline
SimNode::SimNode(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const GlobalNetworkingObjectBag& initGlobalNetworkingObjectBag,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeIdType& initNodeId,
    const RandomNumberGeneratorSeedType& initRunSeed,
    const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
    const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
    BasicNetworkSimulator& simulator)
    :
    NetworkNode(
        parameterDatabaseReader,
        initGlobalNetworkingObjectBag,
        initSimulationEngineInterfacePtr,
        initNodeMobilityModelPtr,
        initNodeId,
        initRunSeed),
    nodeMobilityModelPtr(initNodeMobilityModelPtr)
{
    const unsigned int numberInterfaces = networkLayerPtr->NumberOfInterfaces();

    for(unsigned int i = 0; i < numberInterfaces; i++) {

        const InterfaceIdType interfaceId = networkLayerPtr->GetInterfaceId(i);
        const shared_ptr<PropagationModelType> propagationModelPtr =
            simulator.GetPropagationModel(nodeId, interfaceId);

        (*this).SetupMacAndPhyInterface(
            parameterDatabaseReader,
            initGlobalNetworkingObjectBag,
            propagationModelPtr,
            interfaceId,
            i);
    }//for//

    // Add interface routing protocols from here. -------------------

    SetupRoutingProtocol(
        parameterDatabaseReader,
        nodeId,
        simulationEngineInterfacePtr,
        networkLayerPtr,
        nodeSeed);

    // Add (routing)/transport/application protocols from here. -----



    // --------------------------------------------------------------

    initGisSubsystemPtr->EnableMovingObjectIfNecessary(
        parameterDatabaseReader,
        nodeId,
        simulationEngineInterfacePtr->CurrentTime(),
        nodeMobilityModelPtr);
}

void SimNode::SetupMacAndPhyInterface(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
    const shared_ptr<PropagationModelType>& propagationModelPtr,
    const InterfaceIdType& interfaceId,
    const unsigned int interfaceNumber)
{
    interfaceNumberToChannel[interfaceId] = interfaceNumber;
    while (interfaces.size() <= interfaceNumber) {
        interfaces.push_back(InterfaceType());
    }

    if (InterfaceIsWired(theParameterDatabaseReader, nodeId, interfaceId) ||
        IsNoInterface(theParameterDatabaseReader, nodeId, interfaceId)) {
        // no mac protocol
        // Abstract Network has virtual connections between their network layers.
        return;
    }//if//

    // check aloha interface
    const string macProtocol = MakeLowerCaseString(
        theParameterDatabaseReader.ReadString("mac-protocol", nodeId, interfaceId));

    if (macProtocol.find("aloha") == string::npos) {
        cerr << "Error: bad mac-protocol format: " << macProtocol
             << " for node: " << nodeId << endl
             << "only \"aloha\" is available." << endl;
        exit(1);
    }//if//

    InterfaceType& interface = interfaces.at(interfaceNumber);

    // Antenna Model
    interface.antennaModelPtr =
        CreateAntennaModel(
            theParameterDatabaseReader,
            nodeId,
            interfaceId,
            *theGlobalNetworkingObjectBag.antennaPatternDatabasePtr);

    // Antenna Mobility
    if (AttachedAntennaMobilityModel::AntennaIsAttachedAntenna(
            theParameterDatabaseReader, nodeId, interfaceId)) {

        interface.antennaMobilityModelPtr.reset(
            new AttachedAntennaMobilityModel(
                theParameterDatabaseReader,
                nodeId,
                interfaceId,
                nodeMobilityModelPtr));
    }
    else {
        interface.antennaMobilityModelPtr = nodeMobilityModelPtr;
    }//if//

    // Propagation Interface
    interface.propagationInterfacePtr =
        propagationModelPtr->GetNewPropagationModelInterface(
            simulationEngineInterfacePtr,
            interface.antennaModelPtr,
            interface.antennaMobilityModelPtr,
            nodeId,
            interfaceId,
            interfaceNumber);

    // MAC protocol
    interface.macPtr =
        Aloha::AlohaFactory(
            theParameterDatabaseReader,
            simulationEngineInterfacePtr,
            interface.propagationInterfacePtr,
            nodeId,
            interfaceId,
            networkLayerPtr,
            nodeSeed);

    networkLayerPtr->SetInterfaceMacLayer(interfaceNumber, interface.macPtr);

}//SetupMacAndPhyInterface//

void SimNode::DisconnectPropInterfaces()
{
    for(unsigned int i = 0; i < interfaces.size(); i++) {
        interfaces[i].propagationInterfacePtr->DisconnectThisInterface();
    }//for//

    interfaces.clear();

}//DisconnectPropInterfaces//

void SimNode::CalculatePathlossToLocation(
    const PropagationInformationType& informationType,
    const unsigned int interfaceNumber,
    const double& positionXMeters,
    const double& positionYMeters,
    const double& positionZMeters,
    PropagationStatisticsType& propagationStatistics) const
{
    interfaces.at(interfaceNumber).
        propagationInterfacePtr->CalculatePathlossToLocation(
            informationType,
            positionXMeters,
            positionYMeters,
            positionZMeters,
            propagationStatistics);
}//CalculatePathlossToLocation//

void SimNode::CalculatePathlossToNode(
    const PropagationInformationType& informationType,
    const unsigned int interfaceNumber,
    const ObjectMobilityPosition& rxAntennaPosition,
    const ObjectMobilityModel::MobilityObjectIdType& rxObjectId,
    const AntennaModel& rxAntennaModel,
    PropagationStatisticsType& propagationStatistics) const
{
    const shared_ptr<PropagationInterfaceType> propagationInterfacePtr =
        interfaces.at(interfaceNumber).propagationInterfacePtr;

    const shared_ptr<PropagationModelType> propagationModelPtr =
        propagationInterfacePtr->GetPropagationModel();

        const vector<unsigned int>& channelNumberSet = propagationInterfacePtr->GetCurrentChannelNumberSet();

        unsigned int channelNumber;

        if (channelNumberSet.empty()) {
            channelNumber = propagationInterfacePtr->GetCurrentChannelNumber();
        }
        else {
            channelNumber = channelNumberSet.front();
        }//if//

    propagationModelPtr->CalculatePathlossToNode(
            informationType,
            propagationInterfacePtr->GetCurrentMobilityPosition(),
            nodeId,
            *(*this).GetAntennaModelPtr(interfaceNumber),
            rxAntennaPosition,
            rxObjectId,
            rxAntennaModel,
            channelNumber,
            propagationStatistics);

}//CalculatePathlossToNode//

shared_ptr<PropagationModelType> BasicNetworkSimulator::GetPropagationModel(
    const NodeIdType& nodeId,
    const InterfaceIdType& interfaceId)
{
    const ParameterDatabaseReader& theParameterDatabaseReader = (*theParameterDatabaseReaderPtr);

    if (InterfaceIsWired(theParameterDatabaseReader, nodeId, interfaceId) ||
        IsNoInterface(theParameterDatabaseReader, nodeId, interfaceId)) {
        return shared_ptr<PropagationModelType>();
    }//if//

    const string channelModel = GetChannelInstanceId(theParameterDatabaseReader, nodeId, interfaceId);

    if (propagationModelPtrs.find(channelModel) == propagationModelPtrs.end()) {

        shared_ptr<PropagationModelType> propagationModelPtr(
            new PropagationModelType(
                (*theParameterDatabaseReaderPtr),
                runSeed,
                theSimulationEnginePtr,
                theGisSubsystemPtr,
                channelModel));

        propagationModelPtrs[channelModel] = propagationModelPtr;

        (*this).AddPropagationCalculationTraceIfNecessary(
            channelModel,
            propagationModelPtr->GetPropagationCalculationModel());
    }//if//

    return propagationModelPtrs[channelModel];

}//GetPropagationModel//

void BasicNetworkSimulator::CreateNewNode(
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
    }

    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr =
        theSimulationEnginePtr->GetSimulationEngineInterface(
            theParameterDatabaseReader, nodeId, partitionIndex);

    shared_ptr<SimNode> aSimNodePtr(
        new SimNode(
            theParameterDatabaseReader,
            theGlobalNetworkingObjectBag,
            simulationEngineInterfacePtr,
            nodeId,
            runSeed,
            theGisSubsystemPtr,
            nodeMobilityModelPtr,
            *this));

    (*this).AddNode(aSimNodePtr);

}//CreateNewNode//


//
//-------------------- main --------------------
//

int main(int argc, char* argv[])
{
    GrabArgvForLicensing(argc, argv);

    string configFileName;
    bool isControlledByGui;
    unsigned int numberParallelThreads;
    bool runSequentially;
    bool seedIsSet;
    RandomNumberGeneratorSeedType runSeed;

    MainFunctionArgvProcessingBasicParallelVersion1(
        argc,
        argv,
        configFileName,
        isControlledByGui,
        numberParallelThreads,
        runSequentially,
        seedIsSet,
        runSeed);

    shared_ptr<ParameterDatabaseReader> theParameterDatabaseReaderPtr(
        new ParameterDatabaseReader(configFileName));
    ParameterDatabaseReader& theParameterDatabaseReader =
        *theParameterDatabaseReaderPtr;

    if (!seedIsSet) {
        if (!theParameterDatabaseReader.ParameterExists("seed")) {
            cerr << "Error: No seed parameter found." << endl;
            exit(1);
        }
        runSeed = theParameterDatabaseReader.ReadInt("seed");
    }

    shared_ptr<SimulationEngine> theSimulationEnginePtr(
        new SimulationEngine(
            theParameterDatabaseReader,
            runSequentially,
            numberParallelThreads));

    BasicNetworkSimulator theNetworkSimulator(
        theParameterDatabaseReaderPtr,
        theSimulationEnginePtr,
        runSeed,
        runSequentially);

    if (isControlledByGui) {

        GuiInterfacingSubsystem guiInterface(
            theSimulationEnginePtr,
            static_cast<unsigned short int>(
                theParameterDatabaseReader.ReadNonNegativeInt("gui-portnumber-sim")),
            static_cast<unsigned short int>(
                theParameterDatabaseReader.ReadNonNegativeInt("gui-portnumber-pausecommand")));

        while (true) {
            TimeType simulateUpToTime;

            guiInterface.GetAndExecuteGuiCommands(
                theParameterDatabaseReader,
                theNetworkSimulator,
                simulateUpToTime);

            if (theSimulationEnginePtr->SimulationIsDone()) {
                break;
            }//if//

            theNetworkSimulator.RunSimulationUntil(simulateUpToTime);

        }//while//

    } else {
        const TimeType endSimTime = theParameterDatabaseReader.ReadTime("simulation-time");

        theNetworkSimulator.RunSimulationUntil(endSimTime);

    }//if//

    return 0;

}//main//




