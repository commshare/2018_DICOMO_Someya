// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "multiagent_agentsim.h"
#include "multiagent_publicvehicle.h"
#include "multiagent_behavior.h"
#include "multiagent_routesearch.h"
#include "multiagent_gis.h"
#include "multiagent_extension.h"
#include "multiagent_networkinterface.h"
#include "multiagent_tracedefs.h"
#include "boost/filesystem.hpp"

namespace MultiAgent {

#ifdef MULTIAGENT_DEBUG
const bool MultiAgentSimulator::isDebugMode = true;
#else
const bool MultiAgentSimulator::isDebugMode = false;
#endif

//--------------------------------------------------------------------------------------------------

class AgentMobilityModel : public ObjectMobilityModel {
public:
    AgentMobilityModel(const AgentResource& initResource) : resource(initResource) {}

    virtual void GetUnadjustedPositionForTime(
        const TimeType& snapshotTime, ObjectMobilityPosition& position) override
    {
        position = resource.MobilityPositionForTime(snapshotTime);
    }
private:
    AgentResource resource;
};

class VehicleMobilityModel : public ObjectMobilityModel {
public:
    VehicleMobilityModel(const AgentIdType& initAgentId) : agentId(initAgentId) {}

    void SetPublicVehicle(const shared_ptr<PublicVehicle>& publicVehiclePtr) {
        vehicleAccessPtr.reset(new PublicVehicleAccess(publicVehiclePtr));
    }
    void SetVehicle(const shared_ptr<Vehicle>& vehiclePtr) {
        vehicleAccessPtr.reset(new VehicleAccess(vehiclePtr, VEHICLE_CAR));
    }
    void SetTaxi(const shared_ptr<Vehicle>& vehiclePtr) {
        vehicleAccessPtr.reset(new VehicleAccess(vehiclePtr, VEHICLE_TAXI));
    }

    VehicleType GetVehicleType() const { return vehicleAccessPtr->GetVehicleType(); }

    virtual void GetUnadjustedPositionForTime(
        const TimeType& snapshotTime,
        ObjectMobilityPosition& position) override
    {
        position = (*this).GetCurrentLocation(snapshotTime);
    }

    ObjectMobilityPosition GetCurrentLocation(const TimeType& currentTime) const {
        const Vertex currentPosition = vehicleAccessPtr->GetPosition();
        const double azimuthDegrees = vehicleAccessPtr->GetDirectionRadians()*(180./PI);

        return ObjectMobilityPosition(
            currentTime,
            currentTime,
            currentPosition.x,
            currentPosition.y,
            currentPosition.z,
            true/*theHeightContainsGroundHeightMeters*/,
            90 - azimuthDegrees,
            0.0, 0.0, 0.0, 0.0);
    }

private:
    class AbstractVehicleAccess {
    public:
        virtual ~AbstractVehicleAccess() {}
        virtual const Vertex& GetPosition() const = 0;
        virtual double GetDirectionRadians() const = 0;
        virtual VehicleType GetVehicleType() const = 0;
    };

    class PublicVehicleAccess : public AbstractVehicleAccess {
    public:
        PublicVehicleAccess(const shared_ptr<PublicVehicle>& initPublicVehiclePtr) : publicVehiclePtr(initPublicVehiclePtr) {}
        virtual const Vertex& GetPosition() const { return publicVehiclePtr->GetPosition(); }
        virtual double GetDirectionRadians() const { return publicVehiclePtr->GetDirectionRadians(); }
        virtual VehicleType GetVehicleType() const { return publicVehiclePtr->GetVehicleType(); }
    private:
        shared_ptr<PublicVehicle> publicVehiclePtr;
    };

    class VehicleAccess : public AbstractVehicleAccess {
    public:
        VehicleAccess(
            const shared_ptr<Vehicle>& initVehiclePtr,
            const VehicleType& initVehicleType)
            :
            vehiclePtr(initVehiclePtr),
            vehicleType(initVehicleType)
        {}

        virtual const Vertex& GetPosition() const { return vehiclePtr->GetPosition(); }
        virtual double GetDirectionRadians() const { return vehiclePtr->GetDirectionRadians(); }
        virtual VehicleType GetVehicleType() const { return vehicleType; }
    private:
        shared_ptr<Vehicle> vehiclePtr;
        VehicleType vehicleType;
    };

    AgentIdType agentId;
    shared_ptr<AbstractVehicleAccess> vehicleAccessPtr;
};

//-------------------------------------------------------------------------------------------------

class AgentNode : public NetworkNode {
public:
    AgentNode(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const shared_ptr<ScenSim::GisSubsystem>& theGisSubsystemPtr,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeIdType& theNodeId,
        const RandomNumberGeneratorSeedType& runSeed,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
        const AgentResource& initResource);

    virtual const ObjectMobilityPosition GetCurrentLocation() const {

        const ObjectMobilityPosition position = resource.MobilityPosition();

        if (position.X_PositionMeters() != lastMobilityPosition.X_PositionMeters() ||
            position.Y_PositionMeters() != lastMobilityPosition.Y_PositionMeters() ||
            position.HeightFromGroundMeters() != lastMobilityPosition.HeightFromGroundMeters() ||
            position.AttitudeAzimuthFromNorthClockwiseDegrees() != lastMobilityPosition.AttitudeAzimuthFromNorthClockwiseDegrees()) {
            lastMobilityPosition = position;
        }

        return lastMobilityPosition;
    }

private:
    AgentResource resource;
    mutable ObjectMobilityPosition lastMobilityPosition;
};

class VehicleNode : public AgentNode {
public:
    VehicleNode(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const shared_ptr<ScenSim::GisSubsystem>& theGisSubsystemPtr,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeIdType& theNodeId,
        const RandomNumberGeneratorSeedType& runSeed,
        const shared_ptr<VehicleMobilityModel>& initNodeMobilityModelPtr)
        :
        AgentNode(
            theParameterDatabaseReader,
            theGlobalNetworkingObjectBag,
            theGisSubsystemPtr,
            initSimulationEngineInterfacePtr,
            theNodeId,
            runSeed,
            initNodeMobilityModelPtr,
            AgentResource()),
        mobilityModelPtr(initNodeMobilityModelPtr)
    {}

    void SetPublicVehicle(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<MultiAgentGis>& theAgentGisPtr,
        const TimeType& currentTime,
        const shared_ptr<PublicVehicle>& publicVehiclePtr) {

        mobilityModelPtr->SetPublicVehicle(publicVehiclePtr);

        (*this).EnableMovingObjects(theParameterDatabaseReader, theAgentGisPtr, currentTime);

    }
    void SetVehicle(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<MultiAgentGis>& theAgentGisPtr,
        const TimeType& currentTime,
        const shared_ptr<Vehicle>& vehiclePtr) {

        mobilityModelPtr->SetVehicle(vehiclePtr);

        (*this).EnableMovingObjects(theParameterDatabaseReader, theAgentGisPtr, currentTime);
    }
    void SetTaxi(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<MultiAgentGis>& theAgentGisPtr,
        const TimeType& currentTime,
        const shared_ptr<Vehicle>& vehiclePtr) {

        mobilityModelPtr->SetTaxi(vehiclePtr);

        (*this).EnableMovingObjects(theParameterDatabaseReader, theAgentGisPtr, currentTime);
    }

    AgentIdType GetAgentId() const { return (*this).GetNodeId(); }

    VehicleType GetVehicleType() const { return mobilityModelPtr->GetVehicleType(); }

    virtual const ObjectMobilityPosition GetCurrentLocation() const {

        const ObjectMobilityPosition position = mobilityModelPtr->GetCurrentLocation(simulationEngineInterfacePtr->CurrentTime());

        if (position.X_PositionMeters() != lastMobilityPosition.X_PositionMeters() ||
            position.Y_PositionMeters() != lastMobilityPosition.Y_PositionMeters() ||
            position.HeightFromGroundMeters() != lastMobilityPosition.HeightFromGroundMeters() ||
            position.AttitudeAzimuthFromNorthClockwiseDegrees() != lastMobilityPosition.AttitudeAzimuthFromNorthClockwiseDegrees()) {
            lastMobilityPosition = position;
        }

        return lastMobilityPosition;
    }

private:
    void EnableMovingObjects(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<MultiAgentGis>& theAgentGisPtr,
        const TimeType& currentTime) {

        GisSubsystem& gisSubsystem = theAgentGisPtr->GetSubsystem();

        gisSubsystem.RemoveMovingObject(nodeId);
        gisSubsystem.EnableMovingObjectIfNecessary(
            theParameterDatabaseReader,
            nodeId,
            currentTime,
            mobilityModelPtr);
    }

    shared_ptr<VehicleMobilityModel> mobilityModelPtr;
    mutable ObjectMobilityPosition lastMobilityPosition;
};

inline
AgentNode::AgentNode(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
    const shared_ptr<ScenSim::GisSubsystem>& theGisSubsystemPtr,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeIdType& theNodeId,
    const RandomNumberGeneratorSeedType& runSeed,
    const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
    const AgentResource& initResource)
    :
    NetworkNode(
        theParameterDatabaseReader,
        theGlobalNetworkingObjectBag,
        initSimulationEngineInterfacePtr,
        initNodeMobilityModelPtr,
        theNodeId,
        runSeed),
    resource(initResource)
{}

//---------------------------------------------------------------------------------------------------------

bool AStringStartsWith(const string& aLine, const string& aString)
{
    return (aLine.find(aString) == 0);
}

static inline
bool IsNaturalNumber(const double aValue)
{
    double intPart;
    double floatPart;
    
    floatPart = modf(aValue, &intPart);

    if (intPart <= 0.0) {
        return false;
    }//if//

    return (fabs(floatPart) < DBL_EPSILON);    
}//IsNaturalNumber//

const string Agent::modelName = "Mas";

Agent::Agent(
    MultiAgentSimulator* initSimulatorPtr,
    const GlobalNetworkingObjectBag& initGlobalNetworkingObjectBag,
    const AgentIdType& initAgentId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const shared_ptr<AgentProfile>& initProfilePtr,
    const shared_ptr<AgentTaskTable>& initTaskTablePtr,
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<AgentRouteSearchSubsystem>& initRouteSearchSubsystemPtr)
    :
    simulatorPtr(initSimulatorPtr),
    theAgentGisPtr(initAgentGisPtr),
    thePublicVehicleTablePtr(initPublicVehicleTablePtr),
    theRouteSearchSubsystemPtr(initRouteSearchSubsystemPtr),
    agentId(initAgentId),
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    profilePtr(initProfilePtr),
    taskTablePtr(initTaskTablePtr),
    isDeletedAtTheEndOfTimeStep(false),
    hasBicycle(false),
    currentTaskStartTime(ZERO_TIME),
    currentTaskEndTime(ZERO_TIME),
    isTaskInitialized(false),
    currentTaskNumber(0),
    currentIsInterruptTask(false),
    currentInterruptTaskNumber(0),
    destVertexId(INVALID_VERTEX_ID),
    canChangeToOtherDestinationCandidate(false),
    currentRouteNumber(NO_ROUTE_NUMBER),
    lastVertexId(INVALID_VERTEX_ID),
    entranceWaitStartTime(INFINITE_TIME),
    directionRadians(0),
    currentBehaviorStartTime(ZERO_TIME),
    lastRouteCalculatedTime(ZERO_TIME),
    lastPathQueryTriggerTime(ZERO_TIME),
    recalculateRoute(false),
    recalculateRouteWithBehavior(AGENT_BEHAVIOR_ANY),
    lastDelay(ZERO_TIME),
    utility1CalculationCount(0),
    utility2CalculationCount(0),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(initSimulatorPtr->GetMobilitySeed(), initAgentId, SEED_HASHING)),
    aRandomNumberGeneratorForDestinationChoice(
        HashInputsToMakeSeed(initSimulatorPtr->GetMobilitySeed(), initAgentId, SEED_HASHING_FOR_DESTINATION_CHOICE)),
    parentAgentId(MASTER_ANY_AGENT_ID),
    utility1StatPtr(
        simEngineInterfacePtr->CreateRealStat(modelName + "_Utility1")),
    utility2StatPtr(
        simEngineInterfacePtr->CreateRealStat(modelName + "_Utility2")),
    travelDistanceStatPtr(
        simEngineInterfacePtr->CreateRealStat(modelName + "_TravelDistance")),
    travelTimeStatPtr(
        simEngineInterfacePtr->CreateRealStat(modelName + "_TravelTime")),
    numberNoRouteStatPtr(
        simEngineInterfacePtr->CreateCounterStat(modelName + "_NoRoutes")),
    numberRouteCalculateTimeStatPtr(
        simEngineInterfacePtr->CreateCounterStat(modelName + "_RouteCalculationTimes")),
    utility1Trace(0.),
    utility2Trace(0.),
    congestionTrace(0.),
    travelDistanceTrace(0.),
    travelTimeTrace(0.),
    destinationChangeTrace(0),
    lastTimestepBehaviorType(AGENT_BEHAVIOR_NOTHING)
{
    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();

    AgentResource resource(this);
    bool isSpecifiedPosition;
    Vertex position;
    
    HighQualityRandomNumberGenerator randGen(
        HashInputsToMakeSeed(
            simulatorPtr->GetMobilitySeed(),
            agentId,
            SEED_HASHING_FOR_SCENARIO_GENERATION_LOCATION_INIT));

    taskTablePtr->GetInitialLocationId(
        simulatorPtr->GetParameterDatabaseReader(),
        theAgentGisPtr,
        simulatorPtr->GetInitialLocationFileCachePtr(),
        homePositionId,
        isSpecifiedPosition,
        position,
        resource,
        randGen);

    currentPositionId = homePositionId;
    desiredNextPositionId = currentPositionId;

    RoadIdType roadId;

    if (currentPositionId.type == GIS_BUILDING) {
        const Building& building = subsystem.GetBuilding(currentPositionId.id);
        if (!isSpecifiedPosition) {
            position = building.GetRandomPosition(randGen);
        }//if//

        lastVertexId = building.GetNearestEntranceVertexId(position);
        roadId = building.GetNearestEntranceRoadId(position);

    } else if (currentPositionId.type == GIS_PARK) {

        const Park& park = subsystem.GetPark(currentPositionId.id);

        if (!isSpecifiedPosition) {
            position = park.GetRandomPosition(randGen);
        }//if//

        lastVertexId = park.GetNearestEntranceVertexId(position);
        roadId = park.GetNearestEntranceRoadId(position);
    } else {
        assert(currentPositionId.type == GIS_POI);

        const Poi& poi = subsystem.GetPoi(currentPositionId.id);

        if (!isSpecifiedPosition) {
            position = poi.GetVertex();
        }//if//
        
        lastVertexId = poi.GetVertexId();
        roadId = poi.GetNearestEntranceRoadId(position);
    }
    position.z = theAgentGisPtr->GetSubsystem().GetGroundElevationMetersAt(position);

    status.currentPosition = position;
    status.nextPosition = status.currentPosition;

    const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters = profilePtr->GetParameters();

    bool hasCar;

    (*this).InitializeStatusWith(parameters, true/*calculateCarAndBicycleProbability*/, hasCar);

    if (!IsNaturalNumber(status.values[AGENT_RESERVED_NUMBER_PEOPLE])) {
        cerr << "Error : \"NumberOfPeople\" must be a natural number in AgentProfile." << endl;
        exit(1);
    }//if..

    if (hasCar) {
        const Road& road = subsystem.GetRoad(roadId);
        const size_t poiLaneNumber = road.GetParkingLaneNumber();

        vehiclePtr.reset(
            new Vehicle(
                agentId,
                lastVertexId,
                subsystem.GetVertex(lastVertexId),
                roadId,
                poiLaneNumber,
                simulatorPtr));

        if (road.IsBuildingParkingRoad()) {
            const BuildingIdType buildingId = road.GetBuildingId();

            theAgentGisPtr->PlaceInitialVehicle(resource, buildingId);
        }//if//
    }//if//

    taskTablePtr->GetStatusChanges(resource, timeLineStatusChangeEvents);

    mobilityModelPtr.reset(new AgentMobilityModel(resource));

    const bool enableHumanTalkingInterface = false;

    if (enableHumanTalkingInterface) {
        humanInterfacePtr =
            initGlobalNetworkingObjectBag.sensingSubsystemInterfacePtr->CreateShapeSensingModel(
                initSimEngineInterfacePtr,
                agentId,
                "humantalk",
                HashInputsToMakeSeed(initSimulatorPtr->GetMobilitySeed(), initAgentId));

        humanInterfacePtr->SetDataReceiveHandler(
            shared_ptr<SensingModel::DataReceiveHandler>(
                new PhysicalDataReceiver(AgentResource(this))));


        shared_ptr<FanSensingShape> fanSensingShapePtr(new FanSensingShape());

        // assume talking interface for this instance.
        fanSensingShapePtr->SetCoverageDistanceMeters(100/*talking range 10m*/);
        fanSensingShapePtr->SetHorizontalCoverageFromBoresightDegrees(
            135/*-135 <= tallking range <= + 135*/);

        humanInterfacePtr->SetSensingShape(fanSensingShapePtr);
    }

    destinationChangeTrace.UnsetChangeFlag();
}

Agent::Agent(
    MultiAgentSimulator* initSimulatorPtr,
    const AgentIdType& initAgentId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const shared_ptr<AgentProfile>& initProfilePtr,
    const shared_ptr<AgentTaskTable>& initTaskTablePtr,
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<AgentRouteSearchSubsystem>& initRouteSearchSubsystemPtr,
    const shared_ptr<VehicleNode>& initVehicleNodePtr,
    const shared_ptr<Vehicle>& initVehiclePtr)
    :
    simulatorPtr(initSimulatorPtr),
    theAgentGisPtr(initAgentGisPtr),
    thePublicVehicleTablePtr(initPublicVehicleTablePtr),
    theRouteSearchSubsystemPtr(initRouteSearchSubsystemPtr),
    agentId(initAgentId),
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    profilePtr(initProfilePtr),
    taskTablePtr(initTaskTablePtr),
    isDeletedAtTheEndOfTimeStep(false),
    currentTaskStartTime(ZERO_TIME),
    currentTaskEndTime(INFINITE_TIME),
    isTaskInitialized(true),
    currentTaskNumber(0),
    currentIsInterruptTask(false),
    currentInterruptTaskNumber(0),
    destVertexId(INVALID_VERTEX_ID),
    canChangeToOtherDestinationCandidate(false),
    currentRouteNumber(0),
    lastVertexId(INVALID_VERTEX_ID),
    entranceWaitStartTime(INFINITE_TIME),
    directionRadians(0),
    currentBehaviorStartTime(ZERO_TIME),
    lastRouteCalculatedTime(ZERO_TIME),
    lastPathQueryTriggerTime(ZERO_TIME),
    recalculateRoute(false),
    recalculateRouteWithBehavior(AGENT_BEHAVIOR_ANY),
    lastDelay(ZERO_TIME),
    utility1CalculationCount(0),
    utility2CalculationCount(0),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(initSimulatorPtr->GetMobilitySeed(), initAgentId, SEED_HASHING)),
    aRandomNumberGeneratorForDestinationChoice(
        HashInputsToMakeSeed(initSimulatorPtr->GetMobilitySeed(), initAgentId, SEED_HASHING_FOR_DESTINATION_CHOICE)),
    parentAgentId(MASTER_ANY_AGENT_ID),
    vehicleNodePtr(initVehicleNodePtr),
    utility1Trace(0.),
    utility2Trace(0.),
    congestionTrace(0.),
    travelDistanceTrace(0.),
    travelTimeTrace(0.),
    destinationChangeTrace(0),
    lastTimestepBehaviorType(AGENT_BEHAVIOR_NOTHING)
{
    if (initVehiclePtr != nullptr) {
        currentPositionId = GisPositionIdType(GIS_ROAD, initVehiclePtr->GetRoadId());
        desiredNextPositionId = currentPositionId;
        lastVertexId = initVehiclePtr->GetVertexId();

        const Vertex position = theAgentGisPtr->GetVertex(lastVertexId);

        status.currentPosition = position;
        status.nextPosition = status.currentPosition;
    }

    const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters =
        profilePtr->GetParameters();

    bool notUsedCarFlag;

    (*this).InitializeStatusWith(parameters, false/*calculateCarAndBicycleProbability*/, notUsedCarFlag);

    AgentResource resource(this);

    mobilityModelPtr.reset(new AgentMobilityModel(resource));

    destinationChangeTrace.UnsetChangeFlag();
}


Agent::Agent(
    MultiAgentSimulator* initSimulatorPtr,
    const AgentIdType& initAgentId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const shared_ptr<AgentProfile>& initProfilePtr,
    const shared_ptr<AgentTaskTable>& initTaskTablePtr)
    :
    simulatorPtr(initSimulatorPtr),
    agentId(initAgentId),
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    profilePtr(initProfilePtr),
    taskTablePtr(initTaskTablePtr),
    isDeletedAtTheEndOfTimeStep(false),
    currentTaskStartTime(ZERO_TIME),
    currentTaskEndTime(ZERO_TIME),
    isTaskInitialized(false),
    currentTaskNumber(0),
    currentIsInterruptTask(false),
    currentInterruptTaskNumber(0),
    destVertexId(INVALID_VERTEX_ID),
    canChangeToOtherDestinationCandidate(false),
    currentRouteNumber(0),
    lastVertexId(INVALID_VERTEX_ID),
    entranceWaitStartTime(INFINITE_TIME),
    directionRadians(0),
    currentBehaviorStartTime(ZERO_TIME),
    lastRouteCalculatedTime(ZERO_TIME),
    lastPathQueryTriggerTime(ZERO_TIME),
    recalculateRoute(false),
    recalculateRouteWithBehavior(AGENT_BEHAVIOR_ANY),
    lastDelay(ZERO_TIME),
    utility1CalculationCount(0),
    utility2CalculationCount(0),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(initSimulatorPtr->GetMobilitySeed(), initAgentId, SEED_HASHING)),
    aRandomNumberGeneratorForDestinationChoice(
        HashInputsToMakeSeed(initSimulatorPtr->GetMobilitySeed(), initAgentId, SEED_HASHING_FOR_DESTINATION_CHOICE)),
    parentAgentId(MASTER_ANY_AGENT_ID),
    utility1Trace(0.),
    utility2Trace(0.),
    congestionTrace(0.),
    travelDistanceTrace(0.),
    travelTimeTrace(0.),
    destinationChangeTrace(0),
    lastTimestepBehaviorType(AGENT_BEHAVIOR_NOTHING)
{
    if (profilePtr != nullptr) {
        AgentResource resource(this);

        const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters =
            profilePtr->GetParameters();

        bool notUsedCarFlag;

        (*this).InitializeStatusWith(parameters, false/*calculateCarAndBicycleProbability*/, notUsedCarFlag);

        mobilityModelPtr.reset(new AgentMobilityModel(resource));
    }

    destinationChangeTrace.UnsetChangeFlag();
}

shared_ptr<Agent> Agent::CreateMasterAgent(
    MultiAgentSimulator* initSimulatorPtr,
    const AgentIdType& initAgentId,
    const shared_ptr<AgentProfile>& initProfilePtr,
    const shared_ptr<AgentTaskTable>& initTaskTablePtr)
{
    return shared_ptr<Agent>(
        new Agent(
            initSimulatorPtr,
            initAgentId,
            shared_ptr<SimulationEngineInterface>(),
            initProfilePtr,
            initTaskTablePtr));
}

shared_ptr<Agent> Agent::CreateTaxiDriverAgent(
    MultiAgentSimulator* initSimulatorPtr,
    const AgentIdType& initAgentId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const shared_ptr<AgentProfile>& initProfilePtr,
    const shared_ptr<AgentTaskTable>& initTaskTablePtr,
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<AgentRouteSearchSubsystem>& initRouteSearchSubsystemPtr,
    const shared_ptr<VehicleNode>& initVehicleNodePtr,
    const shared_ptr<Taxi>& initTaxiPtr)
{
    shared_ptr<Agent> agentPtr(
        new Agent(
            initSimulatorPtr,
            initAgentId,
            initSimEngineInterfacePtr,
            initProfilePtr,
            initTaskTablePtr,
            initAgentGisPtr,
            initPublicVehicleTablePtr,
            initRouteSearchSubsystemPtr,
            initVehicleNodePtr,
            initTaxiPtr));

    agentPtr->currentBehaviorPtr.reset(
        new TaxiDriverBehavior(
            initAgentGisPtr,
            initSimulatorPtr->GetPublicVehicleTable(),
            AgentResource(agentPtr),
            initTaxiPtr,
            initSimulatorPtr->TimeStep() + MULTIAGENT_MIN_TIME_STEP));

    return agentPtr;
}

shared_ptr<Agent> Agent::CreateBusDriverAgent(
    MultiAgentSimulator* initSimulatorPtr,
    const AgentIdType& initAgentId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const shared_ptr<AgentProfile>& initProfilePtr,
    const shared_ptr<AgentTaskTable>& initTaskTablePtr,
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<AgentRouteSearchSubsystem>& initRouteSearchSubsystemPtr,
    const shared_ptr<Vehicle>& initVehiclePtr,
    const shared_ptr<VehicleNode>& initVehicleNodePtr,
    const shared_ptr<Bus>& initBusPtr)
{
    shared_ptr<Agent> agentPtr(
        new Agent(
            initSimulatorPtr,
            initAgentId,
            initSimEngineInterfacePtr,
            initProfilePtr,
            initTaskTablePtr,
            initAgentGisPtr,
            initPublicVehicleTablePtr,
            initRouteSearchSubsystemPtr,
            initVehicleNodePtr,
            initVehiclePtr));

    agentPtr->currentBehaviorPtr.reset(
        new BusDriverBehavior(
            initAgentGisPtr,
            initSimulatorPtr->GetPublicVehicleTable(),
            AgentResource(agentPtr),
            initVehiclePtr,
            initBusPtr,
            initSimulatorPtr->TimeStep() + MULTIAGENT_MIN_TIME_STEP));

    agentPtr->isTaskInitialized = true;

    return agentPtr;
}

shared_ptr<Agent> Agent::CreateTrainDriverAgent(
    MultiAgentSimulator* initSimulatorPtr,
    const AgentIdType& initAgentId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const shared_ptr<AgentProfile>& initProfilePtr,
    const shared_ptr<AgentTaskTable>& initTaskTablePtr,
    const shared_ptr<MultiAgentGis>& initAgentGisPtr,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<AgentRouteSearchSubsystem>& initRouteSearchSubsystemPtr,
    const shared_ptr<VehicleNode>& initVehicleNodePtr,
    const shared_ptr<Train>& initTrainPtr)
{
    shared_ptr<Agent> agentPtr(
        new Agent(
            initSimulatorPtr,
            initAgentId,
            initSimEngineInterfacePtr,
            initProfilePtr,
            initTaskTablePtr,
            initAgentGisPtr,
            initPublicVehicleTablePtr,
            initRouteSearchSubsystemPtr,
            initVehicleNodePtr));

    agentPtr->status.currentPosition = initTrainPtr->GetPosition();
    agentPtr->status.nextPosition = agentPtr->status.currentPosition;

    agentPtr->currentBehaviorPtr.reset(
        new TrainDriverBehavior(
            initAgentGisPtr,
            initSimulatorPtr->GetPublicVehicleTable(),
            AgentResource(agentPtr),
            initTrainPtr));

    agentPtr->isTaskInitialized = true;

    return agentPtr;
}

Agent::~Agent()
{
    (*this).ReadyToDestruct();
}

void Agent::InitializeStatusWith(
    const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters,
    const bool calculateCarAndBicycleProbability,
    bool& hasCar)
{
    AgentResource resource(this);

    status.values.resize(parameters.Size());

    map<string, double> assignedValues;

    const bool isReservedMasterAgent =
        ((agentId == MASTER_ANY_AGENT_ID) ||
         (agentId == MASTER_BUS_AGENT_ID) ||
         (agentId == MASTER_TAXI_AGENT_ID));

    HighQualityRandomNumberGenerator randGen(
        HashInputsToMakeSeed(
            simulatorPtr->GetMobilitySeed(),
            agentId,
            SEED_HASHING_FOR_SCENARIO_GENERATION_STATUS_INIT));

    for(AgentStatusIdType statusId = 0; statusId < parameters.Size(); statusId++) {

        if (isReservedMasterAgent) {
            if (!IsBusOrTaxiProfile(statusId)) {
                continue;
            }
        }

        const double value = parameters[statusId].CalculateDouble(resource, randGen);

        status.values[statusId] = value;

        assignedValues[parameters.GetLabel(statusId)] = value;
    }

    hasCar = false;
    hasBicycle = false;

    if (calculateCarAndBicycleProbability) {
        const double hasCarPercentage = randGen.GenerateRandomDouble();
        const double hasBicyclePercentage = randGen.GenerateRandomDouble();

        hasCar = (profilePtr->GetHasCarRatio(resource, randGen) >= hasCarPercentage);
        (*this).hasBicycle = (profilePtr->GetHasBicycleRatio(resource, randGen) >= hasBicyclePercentage);
    }

    // Note: Call parent(Simulator) API.
    // Related functions of Simulator must be initialized.
    simulatorPtr->RecordAssignedProfileValuesToFile(
        agentId,
        profilePtr,
        assignedValues,
        hasCar,
        hasBicycle);
}

void Agent::ReadyToDestruct()
{
    // Clear behaviors.
    // Behaviors may refer an agent pointer in an indirect way.
    currentBehaviorPtr.reset();

    typedef set<shared_ptr<AgentCommunicationNode> >::iterator CommNodeIter;

    for(CommNodeIter iter = communicationNodePtrs.begin();
        iter != communicationNodePtrs.end(); iter++) {

        (*iter)->Detach();
    }

    communicationNodePtrs.clear();

    typedef list<shared_ptr<Agent> >::iterator AgentIter;

    for(AgentIter iter = childAgentPtrs.begin();
        iter != childAgentPtrs.end(); iter++) {

        (*iter)->ReadyToDestruct();
    }

    childAgentPtrs.clear();
}

TimeType Agent::CalculateWakeupTime()
{
    const AgentResource resource(this);

    return taskTablePtr->GetTask(0).GetStartTime(resource);
}

const AgentTask& Agent::CurrentTask() const
{
    if (taskTablePtr->IsEmpty()/*for Taxi*/) {
        assert(!childAgentPtrs.empty());

        if ((currentBehaviorPtr != nullptr) &&
            (currentBehaviorPtr->GetBehaviorType() == AGENT_BEHAVIOR_TAXI_DRIVER)) {

            // Use child(customer) task instead of own task.
            return childAgentPtrs.front()->CurrentTask();
        }
    }

    if (currentIsInterruptTask) {
        return taskTablePtr->GetInterruptTask(currentInterruptTaskNumber);
    }

    return taskTablePtr->GetTask(currentTaskNumber);
}

const GisPositionIdType& Agent::GetHomePositinId() const
{
    if ((currentBehaviorPtr != nullptr) &&
        (currentBehaviorPtr->GetBehaviorType() == AGENT_BEHAVIOR_TAXI_DRIVER)) {

        // Use child(customer) resource instead
        return AgentResource(childAgentPtrs.front().get()).HomePositionId();
    }

    return homePositionId;
}

bool Agent::ShouldChangeRouteInCurrentBehavior(const AgentBehaviorType& specifiedBehavior) const
{
    if (currentBehaviorPtr == nullptr) {
        return false;
    }

    const AgentBehaviorType currentBehaviorType = currentBehaviorPtr->GetBehaviorType();

    if (currentBehaviorPtr->IsInternalRouteChaneMode() &&
        notAvailableBehavorTypesForNextRouteCalculation.find(currentBehaviorType) == notAvailableBehavorTypesForNextRouteCalculation.end() &&
        (specifiedBehavior == AGENT_BEHAVIOR_ANY ||
         currentBehaviorType == specifiedBehavior ||
         currentBehaviorType == AGENT_BEHAVIOR_TAXI_DRIVER ||
         currentBehaviorType == AGENT_BEHAVIOR_BUS_DRIVER)) {
        return true;
    }

    return false;
}

void Agent::DecideRoute(const AgentBehaviorType& specifiedBehavior)
{
    AgentResource resource(this);

    const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters =
        profilePtr->GetParameters();

    lastRouteCalculatedTime = simulatorPtr->CurrentTime();

    status.values[AGENT_RESERVED_STATUS_ROUTE_RECALCULATION_TIME] =
        parameters[AGENT_RESERVED_STATUS_ROUTE_RECALCULATION_TIME].CalculateDouble(resource, resource.GetRandomNumberGenerator());

    bool foundRoute = false;
    bool mustNotResetLastsBehavior;

    do {

        const bool isInternalRouteChangeMode = (*this).ShouldChangeRouteInCurrentBehavior(specifiedBehavior);

        bool isDestinationControlledByBehavior = false;
        AgentBehaviorType behaviorToSearchVertex = AGENT_BEHAVIOR_PEDESTRIAN;
        vector<VertexIdType> destVertexIds;
        vector<VertexIdType> startVertexIds;

        if (isInternalRouteChangeMode) {
            assert(currentBehaviorPtr != nullptr);

            behaviorToSearchVertex = currentBehaviorPtr->GetBehaviorType();

            isDestinationControlledByBehavior =
                currentBehaviorPtr->HasFixedDestinationVertex();

            mustNotResetLastsBehavior = currentBehaviorPtr->IsStaticBehavior();
        } else {
            mustNotResetLastsBehavior = false;
        }

        if (isDestinationControlledByBehavior) {

            assert(currentBehaviorPtr != nullptr);
            destVertexIds.push_back(currentBehaviorPtr->GetFixedDestinationVertexId());

        } else {

            if (destPositionId == UNREACHABLE_POSITION_ID) {

                destVertexId = INVALID_VERTEX_ID;

                // CurrentTask() will return normal or interrupted task due to "currentIsInterruptTask" flag.
                const AgentTask& task = (*this).CurrentTask();

                GisPositionIdType initDestPositionId;
                bool isMultipleDestinations;

                task.GetDestinationId(
                    simulatorPtr->GetParameterDatabaseReader(),
                    theAgentGisPtr,
                    !currentIsInterruptTask/*ignoreLastPositionFromCandidate*/,
                    initDestPositionId,
                    isMultipleDestinations,
                    resource);

                if (initDestPositionId == UNREACHABLE_POSITION_ID) {
                    (*this).OutputTrace("There is no available destination for current task.");
                    break;
                }

                (*this).SetDestination(
                    initDestPositionId,
                    isMultipleDestinations,
                    false/*byCommunication*/);

                if ((destPositionId == desiredNextPositionId) ||
                    ((extraDestPoiId != UNREACHABLE_POSITION_ID &&
                      extraDestPoiId == desiredNextPositionId))) {
                    break;
                }

                // Assign pass intersections at first time
                if (task.HasPassVertexSpecification()) {
                    task.GetPassVertexIds(
                        simulatorPtr->GetParameterDatabaseReader(),
                        theAgentGisPtr,
                        shouldPassVertexIds,
                        resource);
                }
            }

            // return ids sorted by distance from current position.
            theAgentGisPtr->GetNearRouteSearchCandidateVertexIds(
                resource,
                resource.Position(),
                behaviorToSearchVertex,
                destPositionId,
                destVertexId/*prioritizedDestVertexId --> will be inserted to "destVertexIds" as the first entry*/,
                destVertexIds);
        }

        if (currentBehaviorPtr != nullptr) {

            // start next behavior from current behavior via point
            // whether the agent keeps the same behavior type or not.

            startVertexIds.push_back(currentBehaviorPtr->GetViaPointVertexId());

        } else {

            theAgentGisPtr->GetNearRouteSearchCandidateVertexIds(
                resource,
                theAgentGisPtr->GetSubsystem().GetPositionVertex(destPositionId),
                behaviorToSearchVertex,
                desiredNextPositionId,
                lastVertexId,
                startVertexIds);

            if (startVertexIds.empty()) {
                startVertexIds.push_back(lastVertexId);
            }
        }

        assert(!startVertexIds.empty());

        // search from front(prioritized) vertex
        for(size_t i = 0; (!foundRoute && i < destVertexIds.size()); i++) {
            const VertexIdType endVertexId = destVertexIds[i];

            for(size_t j = 0; (!foundRoute && j < startVertexIds.size()); j++) {
                const VertexIdType startVertexId = startVertexIds[j];

                if (isInternalRouteChangeMode) {

                    ostringstream outStream;
                    outStream << "InternalRouteChange v" << startVertexId << " to v" << endVertexId;
                    (*this).OutputTrace(outStream.str());

                    assert(currentBehaviorPtr != nullptr);
                    currentBehaviorPtr->TryInternalRouteChange(startVertexId, endVertexId, foundRoute);

                } else {
                    (*this).DecideRoute(startVertexId, endVertexId, specifiedBehavior, foundRoute);
                }

                if (foundRoute) {
                    if (currentBehaviorPtr == nullptr) {
                        lastVertexId = startVertexId;
                    }
                    destVertexId = endVertexId;
                }
            }
        }

        if (!foundRoute/*--> no route*/ &&
            !isDestinationControlledByBehavior) {

            assert(destPositionId != UNREACHABLE_POSITION_ID);

            unreachableDestinationIds.insert(destPositionId);
            if (extraDestPoiId != UNREACHABLE_POSITION_ID) {
                unreachableDestinationIds.insert(extraDestPoiId);
            }

            (*this).OutputTrace("No Route");

            if (canChangeToOtherDestinationCandidate) {
                (*this).OutputTrace("Destination is unreachable. Search other destination.");
            }

            destPositionId = UNREACHABLE_POSITION_ID;

            if (numberNoRouteStatPtr != nullptr) {
                numberNoRouteStatPtr->IncrementCounter();
            }
        }

    } while (!foundRoute && canChangeToOtherDestinationCandidate);


    if (destPositionId == UNREACHABLE_POSITION_ID) {

        if (mustNotResetLastsBehavior) {

            // nothing to do

        } else {

            currentRouteList.Clear();
            currentBehaviorPtr.reset();
            currentRouteNumber = NO_ROUTE_NUMBER;

            timeToSearchRoute.SetMinTime(resource.CurrentTime());
        }
    }

    (*this).UpdateUtility();
}



void Agent::DecideRoute(
    const VertexIdType& startVertexId,
    const VertexIdType endVertexId,
    const AgentBehaviorType& specifiedBehavior,
    bool& foundRoute)
{
    foundRoute = false;

    const AgentResource resource(this);

    shared_ptr<AgentRoute> lastRoutePtr;

    if (currentBehaviorPtr != nullptr) {
        lastRoutePtr = currentRouteList.routePtrs.front();
    }

    vector<list<AgentRouteList> > routeCandidatesPerOrder;

    typedef list<AgentRouteList>::const_iterator RouteIter;

    vector<pair<RouteIter, double> > routeIters;
    double totalRouteWeight = 0;
    bool hasBeaviorRoute = false;

    if (destPositionId != desiredNextPositionId &&
        startVertexId != endVertexId) {
        const AgentTask& task = (*this).CurrentTask();

        ostringstream outStream;
        outStream << "SearchRoute v" << startVertexId << " to v" << endVertexId;
        (*this).OutputTrace(outStream.str());

        const AgentBehaviorType preferedBehavior = task.GetPreferedBehavior();
        AgentBehaviorType behavior = task.GetBehavior();
        if (specifiedBehavior != AGENT_BEHAVIOR_ANY) {
            // overwrite behavior
            behavior = specifiedBehavior;
        }

        theRouteSearchSubsystemPtr->SearchRouteCandidates(
            resource,
            startVertexId,
            endVertexId,
            shouldPassVertexIds,
            timeToSearchRoute,
            preferedBehavior,
            notAvailableBehavorTypesForNextRouteCalculation,
            routeCandidatesPerOrder);

        assert(!routeCandidatesPerOrder.empty());

        if (behavior != AGENT_BEHAVIOR_ANY) {
            for(size_t i = 0; (!hasBeaviorRoute && i < routeCandidatesPerOrder.size()); i++) {

                const list<AgentRouteList>& routeCandidates = routeCandidatesPerOrder[i];

                for(RouteIter iter = routeCandidates.begin();
                    (!hasBeaviorRoute && iter != routeCandidates.end()); iter++) {

                    if ((*iter).totalCost.values[AGENT_ROUTE_COST_MODE] > 0) {
                        hasBeaviorRoute = true;
                    }
                }
            }
        }
    }

    for(size_t i = 0; i < routeCandidatesPerOrder.size(); i++) {
        const list<AgentRouteList>& routeCandidates = routeCandidatesPerOrder[i];

        for(RouteIter iter = routeCandidates.begin(); iter != routeCandidates.end(); iter++) {
            const AgentRouteList& aRoute = (*iter);

            if (aRoute.IsEmpty()) {
                continue;
            }
            if (hasBeaviorRoute && aRoute.totalCost.values[AGENT_ROUTE_COST_MODE] <= 0) {
                continue;
            }

            const double routeWeight = (*this).CalculateRouteWeight(aRoute);

            routeIters.push_back(make_pair(iter, routeWeight));
            totalRouteWeight += routeWeight;

            (*this).OutputTrace(
                "RouteCandidate " + ConvertToString(routeWeight) + ":" +
                aRoute.ConvertToString());
        }
    }

    if ((!routeIters.empty()) ||
        (startVertexId == endVertexId && destPositionId != desiredNextPositionId) ||
        (destPositionId == desiredNextPositionId &&
         extraDestPoiId != extraCurrentPoiId &&
         extraDestPoiId.IsValid())) {

        foundRoute = true;
        currentRouteNumber++;

        currentRouteList.Clear();
        timeToSearchRoute.SetMinTime(resource.CurrentTime());

        shared_ptr<AgentRoute> nextRoutePtr;

        if (!routeIters.empty()) {
            size_t routeId = 0;
            HighQualityRandomNumberGenerator& randomnumberGenerator = resource.GetRandomNumberGenerator();

            if (totalRouteWeight > 0) {
                const double decideRatio = randomnumberGenerator.GenerateRandomDouble();

                double totalRatio = 0;

                for(; routeId < routeIters.size() - 1; routeId++) {
                    totalRatio += (routeIters[routeId].second / totalRouteWeight);

                    if (decideRatio < totalRatio) {
                        break;
                    }
                }
            } else {
                routeId = randomnumberGenerator.GenerateRandomInt(0, (int32_t)(routeIters.size() - 1));
            }
            currentRouteList = *(routeIters[routeId].first);

            assert(!currentRouteList.routePtrs.empty());

            if (currentBehaviorPtr != nullptr &&

                currentBehaviorPtr->IsAcceptableRouteChange(*currentRouteList.routePtrs.front())) {
                currentBehaviorPtr->ChangeRoute(currentRouteList.routePtrs.front());

            } else {

                if (currentBehaviorPtr != nullptr) {

                    if (!currentRouteList.routePtrs.empty()) {
                        nextRoutePtr = currentRouteList.routePtrs.front();
                    }

                    currentRouteList.routePtrs.push_front(lastRoutePtr);
                    currentBehaviorPtr->EndBehaviorAtViaPoint(nextRoutePtr);

                } else {
                    if (!currentRouteList.IsEmpty()) {
                        if (IsFreeWalkPosition(desiredNextPositionId.type)) {
                            const TimeType preparetionTime = 30*SECOND;

                            currentRouteList.routePtrs.push_front(
                                shared_ptr<AgentRoute>(new AgentRoute(AGENT_BEHAVIOR_FREEWALK)));

                            currentRouteList.startTime -= preparetionTime;
                        }
                    }

                    currentTaskStartTime =
                        std::max(currentTaskStartTime, currentRouteList.startTime);
                }
            }
        } else {
            if (currentBehaviorPtr != nullptr) {
                currentRouteList.routePtrs.push_front(lastRoutePtr);
                currentBehaviorPtr->EndBehaviorAtViaPoint(nextRoutePtr);
            }
        }

        if (IsFreeWalkPosition(destPositionId.type)) {
            currentRouteList.routePtrs.push_back(
                shared_ptr<AgentRoute>(new AgentRoute(AGENT_BEHAVIOR_FREEWALK)));
        }

        (*this).OutputTrace(
            "Route:" + currentRouteList.ConvertToString());
    }
}

double Agent::CalculateRouteWeight(const AgentRouteList& aRoute)
{
    const AgentResource resource(this);

    const double totalWeight =
        profilePtr->CalculateRouteUtility(
            resource,
            resource.GetRandomNumberGenerator(),
            AGENT_ROUTE_COST_MODE,
            healthOrUtilityFactor,
            aRoute.totalCost);

    return std::max(0., totalWeight);
}

void Agent::SetDestination(
    const GisPositionIdType& initDestPositionId,
    const bool initCanChangeToOtherDestinationCandidate,
    const bool byCommunication)
{
    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();

    GisPositionIdType newDestPositionId = initDestPositionId;
    GisPositionIdType newExtraDestPoiId = UNREACHABLE_POSITION_ID;

    if (newDestPositionId.type == GIS_POI) {
        const Poi& poi = subsystem.GetPoi(newDestPositionId.id);

        if (poi.IsAPartOfObject()) {
            newDestPositionId = poi.GetParentGisPositionId();

            assert(newDestPositionId.type == GIS_BUILDING ||
                   newDestPositionId.type == GIS_PARK);

            newExtraDestPoiId = initDestPositionId;
        }
    }

    if (destPositionId != newDestPositionId &&
        newDestPositionId != UNREACHABLE_POSITION_ID) {
        destinationChangeTrace.SetValue(*destinationChangeTrace + 1);

        if (byCommunication) {
            destinationChangeByCommunicationTrace.SetValue(*destinationChangeByCommunicationTrace + 1);
        }

        const string name = subsystem.GetGisObject(initDestPositionId).GetObjectName();

        ostringstream outStream;
        outStream << "SetDestination " << name;

        (*this).OutputTrace(outStream.str());
    }

    shouldPassVertexIds.clear();
    destPositionId = newDestPositionId;
    extraDestPoiId = newExtraDestPoiId;

    canChangeToOtherDestinationCandidate = initCanChangeToOtherDestinationCandidate;
    recalculateRoute = true;
}

void Agent::SetCurrentDestinationToUnreachablePosition()
{
    AgentResource resource(this);

    resource.UnreachableDestinationNotification();

    list<GisPositionIdType> unreachablePositionIds;

    unreachablePositionIds.push_back(destPositionId);

    if (extraDestPoiId != UNREACHABLE_POSITION_ID) {
        unreachablePositionIds.push_back(extraDestPoiId);
    }

    (*this).AddUnreachablePositions(unreachablePositionIds, false/*byCommunication*/);
}

void Agent::AddUnreachablePositions(
    const list<GisPositionIdType>& unreachablePositionIds,
    const bool byCommunication)
{
    AgentResource resource(this);

    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();

    typedef list<GisPositionIdType>::const_iterator IterType;

    for(IterType iter = unreachablePositionIds.begin(); iter != unreachablePositionIds.end(); iter++) {
        const GisPositionIdType& positionId = (*iter);

        // agent is in a position
        if (positionId == desiredNextPositionId && !resource.WaitingAtEntrance()) {
            continue;
        }

        unreachableDestinationIds.insert(positionId);
    }

    if (unreachableDestinationIds.find(destPositionId) != unreachableDestinationIds.end()) {

        if ((*this).WaitingAtDestinationEntrace()) {
                
            theAgentGisPtr->PedestrianOrDriverGiveUpEntrance(resource, destPositionId);
        }

        if (byCommunication) {
            destinationChangeByCommunicationTrace.SetValue(*destinationChangeByCommunicationTrace + 1);
        }
        destPositionId = UNREACHABLE_POSITION_ID;
        recalculateRoute = true;
    }
}

void Agent::ChangeToSpecificDestination(
    const GisPositionIdType& initDestPositionId,
    const VertexIdType& initDestVertexId,
    const bool byCommunication)
{
    AgentResource resource(this);

    if ((*this).WaitingAtDestinationEntrace()) {
        theAgentGisPtr->PedestrianOrDriverGiveUpEntrance(resource, desiredNextPositionId);
    }

    (*this).SetDestination(initDestPositionId, false/*isMultipleDestinations*/, byCommunication);

    const TimeType currentTime = simulatorPtr->CurrentTime();

    currentTaskStartTime = currentTime;
    destVertexId = initDestVertexId;

    (*this).RecalculateRoute(currentTime);
}

void Agent::MoveNextStatesToCurrent()
{
    status.currentPosition = status.nextPosition;
}



void Agent::IncrementTime()
{
    const TimeType currentTime = simulatorPtr->CurrentTime();

    AgentResource resource(this);

    (*this).lastTimestepBehaviorType = resource.GetBehaviorType();

    if (isDeletedAtTheEndOfTimeStep) {
        return;
    }

    // initialization (called only once at first time)
    if (!isTaskInitialized) {
        isTaskInitialized = true;
        (*this).AssignCurrentTask();
    }

    (*this).MoveNextStatesToCurrent();

    const GisPositionIdType lastCurrentPositionId = currentPositionId;
    const GisPositionIdType lastDesiredNextPositionId = desiredNextPositionId;

    // status(profile variable or task) changes
    (*this).ApplyStatusChangesAndInstantiateApplications();

    // route (re)calculation
    if (resource.ExceededRouteRecalculationTime()) {
        resource.RecalculateRoute();
    }
    if (recalculateRoute) {
        (*this).DecideRoute(recalculateRouteWithBehavior);

        recalculateRoute = false;
        notAvailableBehavorTypesForNextRouteCalculation.clear();

        if (numberRouteCalculateTimeStatPtr != nullptr) {
            numberRouteCalculateTimeStatPtr->IncrementCounter();
        }

        recalculateRoute = false;
        recalculateRouteWithBehavior = AGENT_BEHAVIOR_ANY;
    }

    // execute behavior
    if (currentTaskStartTime <= currentTime) {
        bool assignedNewBehavior;

        (*this).RecalculateBehaviorIfNecessary(assignedNewBehavior);

        if (assignedNewBehavior) {
            //Increment from next time step.
            resource.SetPosition(resource.Position());
        } else {
            (*this).IncrementCurrentBehaviorTime();
        }

    } else {
        resource.SetPosition(resource.Position());
    }

    // next behavior preparation
    if ((*this).CurrentTaskHasFinished()) {
        (*this).OutputTrace("Finished Task");

        (*this).GoToNextBehaviorIfPossible();

    // Give up current destination and go to other destination.
    } else if ((*this).OtherDestinationSeemsToBeBetter()) {
        
        assert(resource.WaitingAtEntrance());
        assert((resource.ExceededWaitEntranceTime()) ||
               (resource.ExceededWaitVehicleEntranceTime()));

        (*this).OutputTrace("Destination reached a limit capacity. Set other destination");

        (*this).SetCurrentDestinationToUnreachablePosition();
    }

    //assert((lastDesiredNextPositionId == desiredNextPositionId) ||
    //       (currentPositionId == lastDesiredNextPositionId) ||
    //       (currentPositionId == desiredNextPositionId));

    // update translation state

    theAgentGisPtr->UpdatePeopleTranslationBetweenGisObjects(
        resource,
        currentPositionId,
        lastDesiredNextPositionId,
        desiredNextPositionId,
        lastTimestepBehaviorType,
        resource.GetBehaviorType());

    // statistics/traces
    const Vertex& currentPos = (*this).GetCurrentPosition();
    const Vertex& nextPos = (*this).GetNextPosition();

    if (utility1StatPtr != nullptr) {
        utility1StatPtr->RecordStatValue(resource.Utility1());
    }
    if (utility2StatPtr != nullptr) {
        utility2StatPtr->RecordStatValue(resource.Utility2());
    }

    const double distanceMeters = currentPos.DistanceTo(nextPos);
    status.values[AGENT_RESERVED_STATUS_TOTAL_TRAVEL_DISTANCE] += currentPos.DistanceTo(nextPos);

    if (distanceMeters >= MinStepDistanceToCountInStatsMeters) {

        status.values[AGENT_RESERVED_STATUS_TOTAL_TRAVEL_TIME] +=
            double(simulatorPtr->TimeStep()) / SECOND;

        if (travelDistanceStatPtr != nullptr) {
            const double traveDistance = resource.TotalTravelDistance();
            travelDistanceStatPtr->RecordStatValue(traveDistance);
            travelDistanceTrace.SetValue(traveDistance);
        }
        if (travelTimeStatPtr != nullptr) {
            const double travelTime = resource.TotalTravelTime();
            travelTimeStatPtr->RecordStatValue(travelTime);
            travelTimeTrace.SetValue(travelTime);
        }
    }

    // chldren calculation
    typedef list<shared_ptr<Agent> >::const_iterator IterType;

    for(IterType iter = childAgentPtrs.begin(); iter != childAgentPtrs.end(); iter++) {
        assert(agentId != (*iter)->GetAgentId());
        (*iter)->IncrementTime();
    }
}




void Agent::GoToNextBehaviorIfPossible()
{
    const TimeType currentTime = simulatorPtr->CurrentTime();

    AgentResource resource(this);

    if (currentIsInterruptTask) {

        timeLineStatusChangeEvents.push(
            AgentStatusChangeEvent(
                currentTime,
                currentInterruptTaskNumber,
                AGENT_STATUS_CHANGE_TASK_INTERRUPTION_END));

        if ((*this).CurrentTask().GetInterruptionType() == AGENT_BEHAVIOR_INTERRUPTION_LATER) {
            // back to last (interrupted) normal task.
            if (currentTaskNumber > 0) {
                currentTaskNumber--;
            }
        }

        currentIsInterruptTask = false;

    } else {
        timeLineStatusChangeEvents.push(
            AgentStatusChangeEvent(
                currentTime,
                currentTaskNumber,
                AGENT_STATUS_CHANGE_BASIC_TASK_END));
    }


    do {
        currentTaskNumber++;

        if ((*this).HasCurrentTask()) {
            if ((*this).CurrentTask().SatisfyCondition(resource)) {
                (*this).AssignCurrentTask();
                break;
            }
        }

    } while ((*this).HasCurrentTask());

    if (!(*this).HasCurrentTask()) {
        (*this).OutputTrace("Completed All Task");
        isDeletedAtTheEndOfTimeStep = true;
        desiredNextPositionId = GisPositionIdType();
    }
}

void Agent::AssignCurrentTask()
{
    assert(!currentIsInterruptTask);

    const AgentTask& task = (*this).CurrentTask();

    AgentResource resource(this);

    currentTaskEndTime = task.GetEndTime(resource);
    currentTaskStartTime = task.GetStartTime(resource);

    const TimeType earlyStartTime =
        std::max(resource.CurrentTime(), currentTaskStartTime);

    task.GetTimeLine(resource, earlyStartTime, timeToSearchRoute);

    timeLineStatusChangeEvents.push(
        AgentStatusChangeEvent(
            earlyStartTime,
            currentTaskNumber,
            AGENT_STATUS_CHANGE_BASIC_TASK_START));

    destPositionId = UNREACHABLE_POSITION_ID;

    (*this).RecalculateRoute(simulatorPtr->CurrentTime());
}

void Agent::AssignInterruptTask()
{
    assert(currentIsInterruptTask);

    AgentResource resource(this);

    if ((*this).WaitingAtDestinationEntrace()) {
        theAgentGisPtr->PedestrianOrDriverGiveUpEntrance(resource, desiredNextPositionId);
    }

    const AgentTask& task = (*this).CurrentTask();

    currentTaskEndTime = task.GetEndTime(resource);
    currentTaskStartTime = task.GetStartTime(resource);

    assert(currentTaskStartTime == simulatorPtr->CurrentTime());

    task.GetTimeLine(resource, currentTaskStartTime, timeToSearchRoute);

    destPositionId = UNREACHABLE_POSITION_ID;

    (*this).RecalculateRoute(simulatorPtr->CurrentTime());
}

void Agent::ApplyStatusChangesAndInstantiateApplications()
{
    const TimeType currentTime = simulatorPtr->CurrentTime();

    while (!timeLineStatusChangeEvents.empty() &&
           timeLineStatusChangeEvents.top().time <= currentTime) {

        const AgentStatusChangeEvent& statusChangeEvent = timeLineStatusChangeEvents.top();
        const AgentResource resource(this);
        const AgentStatusChangeType& changeType = statusChangeEvent.statusChangeType;

        if (IsBasicTaskStatusChange(changeType)) {

            if (statusChangeEvent.statusChangeNumber < taskTablePtr->GetNumberOfTasks()) {
                const AgentTask& task = taskTablePtr->GetTask(statusChangeEvent.statusChangeNumber);

                if (task.HasStatusChange(changeType)) {
                    (*this).ApplyAdditionalStatusChanges(task.GetAdditionaStatusChange(changeType));
                }
            }

        } else if (IsInterrupionTaskStatusChange(changeType)) {

            if (statusChangeEvent.statusChangeNumber < taskTablePtr->GetNumberOfInterruptTasks()) {
                const AgentTask& task = taskTablePtr->GetInterruptTask(statusChangeEvent.statusChangeNumber);

                if (changeType == AGENT_STATUS_CHANGE_TASK_INTERRUPTION_START) {

                    if (task.SatisfyCondition(resource)) {
                        (*this).OutputTrace("InterruptCurrentAction");

                        currentIsInterruptTask = true;
                        currentInterruptTaskNumber = statusChangeEvent.statusChangeNumber;

                        if (task.HasStatusChange(changeType)) {
                            (*this).ApplyAdditionalStatusChanges(task.GetAdditionaStatusChange(changeType));
                        }

                        (*this).AssignInterruptTask();
                    }
                } else {

                    if (task.HasStatusChange(changeType)) {
                        (*this).ApplyAdditionalStatusChanges(task.GetAdditionaStatusChange(changeType));
                    }
                }
            }

        } else {
            assert(IsSpecificTimeStatusChange(changeType));

            const AgentTask& task = taskTablePtr->GetStatusChange(statusChangeEvent.statusChangeNumber);

            if (task.SatisfyCondition(resource)) {
                (*this).ApplyAdditionalStatusChanges(task.GetAdditionaStatusChange(changeType));
            }
        }

        timeLineStatusChangeEvents.pop();
    }
}

void Agent::ApplyAdditionalStatusChanges(const AgentAdditionalStatusChange& additionalStatusChange)
{
    (*this).ApplyStatusChanges(additionalStatusChange.statusChanges);
    (*this).InstantiateApplications(additionalStatusChange.applicationSpecifications);
}

void Agent::ApplyStatusChanges(const vector<pair<AgentStatusIdType, AgentValueFormula> >& statusChanges)
{
    if (statusChanges.empty()) {
        return;
    }

    const AgentResource resource(this);

    bool needToUpdateVehicleStatus = false;

    ostringstream outStream;
    outStream << "ChangeStatus: ";

    const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters = profilePtr->GetParameters();

    for(size_t i = 0; i < statusChanges.size(); i++) {
        const pair<AgentStatusIdType, AgentValueFormula>& statusChange = statusChanges[i];
        const AgentStatusIdType& statusId = statusChange.first;
        const AgentValueFormula& valueFormula = statusChange.second;

        status.values[statusId] = valueFormula.CalculateDouble(resource, resource.GetRandomNumberGenerator());

        if (i > 0) {
            outStream << ", ";
        }
        outStream << parameters.GetLabel(statusId) << " = " << status.values[statusId];

        if (IsVehicleStatus(statusId)) {
            needToUpdateVehicleStatus = true;
        }
    }

    (*this).OutputTrace(outStream.str());

    if (needToUpdateVehicleStatus && vehiclePtr != nullptr) {
        theAgentGisPtr->UpdateVehicleStatus(resource, vehiclePtr);
    }
}

void Agent::InstantiateApplications(
    const map<DynamicApplicationIdType, DynamicApplicationDefinition>& applicationSpecifications)
{
    if (applicationSpecifications.empty()) {
        return;
    }

    typedef map<DynamicApplicationIdType, DynamicApplicationDefinition>::const_iterator IterType;

    const AgentResource resource(this);
    const NodeIdType nodeId = agentId;
    const TimeType currentTime = simulatorPtr->CurrentTime();

    for(IterType iter = applicationSpecifications.begin();
        iter != applicationSpecifications.end(); iter++) {

        const DynamicApplicationIdType& dynamicApplicationId = (*iter).first;
        const DynamicApplicationDefinition& applicationDefinition = (*iter).second;
        const vector<DynamicApplicationDefinitionParameter>& parameters = applicationDefinition.parameters;
        const string& instanceName = dynamicApplicationId.instanceName;

        InterfaceOrInstanceIdType instanceId;

        if (!instanceName.empty()) {
            instanceId = instanceName + "-" + ConvertToString(nodeId);
        } else {
            instanceId = "dynamicapp" + ConvertTimeToStringSecs(currentTime) + "s-" + ConvertToString(nodeId);
        }

        ostringstream scopeStream;

        scopeStream << "[" << nodeId << ";" << instanceId << "]";

        const string scope = scopeStream.str();

        vector<string> parameterLines;
        set<NodeIdType> targetNodeIds;

        targetNodeIds.insert(nodeId);

        for(size_t i = 0; i < parameters.size(); i++) {
            const DynamicApplicationDefinitionParameter& parameter = parameters[i];

            if (parameter.applicationParameterName.find("-destination") != string::npos) {
                const NodeIdType destinationNodeId = App_ConvertStringToNodeIdOrAnyNodeId(parameter.value);

                targetNodeIds.insert(destinationNodeId);
            }

            const string& parameterName = parameter.applicationParameterName;
            string parameterNameWithSpace = parameter.applicationParameterName + " ";
            string value = parameter.value;

            // Search with white space end indicator to avoid finding "-start-time-xxx" and "-end-time-xxx".

            if ((parameterNameWithSpace.find("-start-time ") != string::npos) ||
                (parameterNameWithSpace.find("-end-time ") != string::npos)) {

                TimeType timeValue;
                bool success;

                ConvertStringToTime(value, timeValue, success);

                if (!success) {
                    cerr << "Error: Application Behavior, bad Time parameter value for: " << parameterName;
                    cerr << "  Value = " << value << endl;
                    exit(1);
                }

                timeValue += currentTime;

                value = ConvertTimeToStringSecs(timeValue);
            }

            const string parameterLine = scope + " " + parameterName + " = " + value;

            parameterLines.push_back(parameterLine);

        }

        ostringstream outStream;
        outStream << "GenerateApplication: ";

        for(size_t i = 0; i < parameterLines.size(); i++) {
            if (i > 0) {
                outStream << ", ";
            }
            outStream << parameterLines[i];
        }

        (*this).OutputTrace(outStream.str());

        simulatorPtr->CreateApplicationForNode(
            resource,
            nodeId,
            instanceId,
            parameterLines,
            targetNodeIds);
    }
}

void Agent::RecalculateBehaviorIfNecessary(bool& assignedNewBehavior)
{
    AgentResource resource(this);

    if (currentBehaviorPtr != nullptr || resource.WaitingAtEntrance()) {
        assignedNewBehavior = false;
        return;
    }

    if (currentRouteList.IsEmpty()) {
        assignedNewBehavior = false;
        return;
    }

    assignedNewBehavior = true;

    (*this).OutputTrace("Assign Behavior for " + currentRouteList.ConvertToString());

    const shared_ptr<AgentRoute> routePtr = currentRouteList.routePtrs.front();
    const TimeType currentTime = resource.CurrentTime();

    currentBehaviorStartTime = currentTime;
    currentBehaviorCost = AgentRouteCost();

    const shared_ptr<PublicVehicleTable> thePublicVehicleTableConstPtr = simulatorPtr->GetPublicVehicleTable();

    switch (routePtr->behavior) {
    case AGENT_BEHAVIOR_FREEWALK: {

        GisPositionIdType endPositionId = destPositionId;
        bool enterToPosition;

        if (currentRouteList.routePtrs.size() > 1) {
            enterToPosition = false;// -> leave from position

            list<shared_ptr<AgentRoute> >::const_iterator iter =
                currentRouteList.routePtrs.begin();

            iter++;

            if ((*iter)->IsRoad()) {
                endPositionId = (*iter)->GetStartRoadPositionId();
            } else {
                endPositionId = thePublicVehicleTableConstPtr->GetPositionId((*iter)->GetStartStopId());
            }

        } else {
            enterToPosition = true;

            assert(endPositionId.type == GIS_BUILDING ||
                   endPositionId.type == GIS_PARK ||
                   endPositionId.type == GIS_POI);

            resource.SetDesiredNextPositionId(endPositionId);
        }

        currentBehaviorPtr.reset(
            new FreeWalkBehavior(theAgentGisPtr, thePublicVehicleTableConstPtr, routePtr, endPositionId, enterToPosition, resource));
        break;
    }

    case AGENT_BEHAVIOR_VEHICLE:
        currentBehaviorPtr.reset(
            new VehicleDriverBehavior(
                theAgentGisPtr, thePublicVehicleTableConstPtr, routePtr, resource, vehiclePtr, simulatorPtr->TimeStep() + MULTIAGENT_MIN_TIME_STEP));
        break;

    case AGENT_BEHAVIOR_BUS:
    case AGENT_BEHAVIOR_TRAIN:
        currentBehaviorPtr.reset(
            new PublicVehicleBehavior(
                theAgentGisPtr, thePublicVehicleTableConstPtr, routePtr, resource));
        break;

    case AGENT_BEHAVIOR_TAXI:
        currentBehaviorPtr.reset(
            new TaxiGuestBehavior(theAgentGisPtr, thePublicVehicleTableConstPtr, routePtr, resource));
        break;

    case AGENT_BEHAVIOR_BICYCLE:
        currentBehaviorPtr.reset(
            new BicycleBehavior(theAgentGisPtr, thePublicVehicleTableConstPtr, routePtr, resource));
        break;

    case AGENT_BEHAVIOR_PEDESTRIAN:
        currentBehaviorPtr.reset(
            new PedestrianBehavior(theAgentGisPtr, thePublicVehicleTableConstPtr, routePtr, resource));
        break;
    default:
        assert(false); abort(); break;
    }

    if (currentBehaviorPtr != nullptr) {
        (*this).OutputTrace("New Behavior:" + currentBehaviorPtr->GetBehaviorName());
    }

    (*this).UpdateUtility();
}

void Agent::IncrementCurrentBehaviorTime()
{
    AgentResource resource(this);

    if (currentBehaviorPtr == nullptr || resource.WaitingAtEntrance()) {
        resource.SetPosition(resource.Position());
        return;
    }

    currentBehaviorPtr->IncrementTimeStep(simulatorPtr->TimeStep());

    if (MultiAgentSimulator::isDebugMode) {
        (*this).OutputTrace(currentBehaviorPtr->MakePositionTraceString());
    }//if//

    if (!recalculateRoute && currentBehaviorPtr->HasFinished()) {

        lastDelay = resource.NextDelay();
        (*this).UpdateUtility();

        currentBehaviorPtr.reset();

        (*this).OutputTrace("End Behavior");

        if (!currentRouteList.IsEmpty()) {

            currentRouteList.routePtrs.pop_front();
        }

        if (currentRouteList.routePtrs.empty() && (*this).HasCurrentTask()) {
            const TimeType currentTime = resource.CurrentTime();
            const TimeType waitTime = (*this).CurrentTask().GetWaitTime(resource);
            const TimeType waitEndTime = currentTime + waitTime;

            if (waitTime > ZERO_TIME) {
                if (currentIsInterruptTask) {
                    timeLineStatusChangeEvents.push(
                        AgentStatusChangeEvent(
                            currentTime,
                            currentInterruptTaskNumber,
                            AGENT_STATUS_CHANGE_TASK_INTERRUPTION_BEFORE_WAITING));
                } else {
                    timeLineStatusChangeEvents.push(
                        AgentStatusChangeEvent(
                            currentTime,
                            currentTaskNumber,
                            AGENT_STATUS_CHANGE_BASIC_TASK_BEFORE_WAITING));
                }
            }

            if (currentTaskEndTime < INFINITE_TIME) {
                currentTaskEndTime = std::max(currentTaskEndTime, waitEndTime);
            } else {
                currentTaskEndTime = waitEndTime;
            }
        }
    }

    //(*this).UpdateHealthFactor();
}

void Agent::UpdateHealthFactor()
{
    AgentResource resource(this);

    healthOrUtilityFactor.values[AGENT_UTILITY_FACTOR_LAST_DELAY] = double(resource.LastDelay()) / SECOND;
    healthOrUtilityFactor.values[AGENT_UTILITY_FACTOR_NEXT_DELAY] = double(resource.NextDelay()) / SECOND;
    healthOrUtilityFactor.values[AGENT_UTILITY_FACTOR_TRIP_DELAY] = double(resource.TripDelay()) / SECOND;
    healthOrUtilityFactor.values[AGENT_UTILITY_FACTOR_ARRIVAL_DELAY] = double(resource.ArrivalDelay()) / SECOND;
}

void Agent::UpdateUtility()
{
    (*this).UpdateHealthFactor();

    const AgentResource resource(this);

    utility1CalculationCount++;
    utility2CalculationCount++;

    status.values[AGENT_RESERVED_STATUS_UTILITY1] =
        profilePtr->CalculateUtility1(resource, resource.GetRandomNumberGenerator(), healthOrUtilityFactor, currentRouteList.totalCost);
    status.values[AGENT_RESERVED_STATUS_UTILITY2] =
        profilePtr->CalculateUtility2(resource, resource.GetRandomNumberGenerator(), healthOrUtilityFactor, currentRouteList.totalCost);
    healthOrUtilityFactor.values[AGENT_UTILITY_FACTOR_UTILITY1_COUNTER] = utility1CalculationCount;
    healthOrUtilityFactor.values[AGENT_UTILITY_FACTOR_UTILITY2_COUNTER] = utility2CalculationCount;

    const double utility1 = resource.Utility1();
    const double utility2 = resource.Utility2();

    utility1Trace.SetValue(utility1);
    utility2Trace.SetValue(utility2);

    (*this).OutputTrace("Utility = " + ConvertToString(utility1) + " / " +  ConvertToString(utility2));
}

bool Agent::CurrentTaskHasFinished() const
{
    const TimeType currentTime = simulatorPtr->CurrentTime();

    if (currentTime < currentTaskStartTime) {
        return false;
    }

    if (currentBehaviorPtr != nullptr) {
        return false;
    }

    if (!currentRouteList.IsEmpty()) {
        return false;
    }

    // 1. Dynamically generated agents in gis object which has capacity may wait entrance.
    // 2. Agents are going to skip current task if there is no route and try to leave(;be deleted) from simulation at the end of the last task.
    // Guarantee the agent entrance before the agent deletion.

    if ((*this).WaitingAtEntrance()) {
        return false;
    }

    if (currentTaskEndTime < INFINITE_TIME) {
        return (currentTime >= currentTaskEndTime);
    }

    return true;
}

bool Agent::HasCurrentTask() const
{
    return (currentTaskNumber < taskTablePtr->GetNumberOfTasks());
}

void Agent::AddCommunicationNode(const shared_ptr<AgentCommunicationNode>& communicationNodePtr)
{
    communicationNodePtr->resource = AgentResource(this); //connect resource

    communicationNodePtrs.insert(communicationNodePtr);

    communicationNodePtr->Attach(mobilityModelPtr);
}

void Agent::DeleteCommunicationNode(const shared_ptr<AgentCommunicationNode>& communicationNodePtr)
{
    assert(communicationNodePtrs.find(communicationNodePtr) != communicationNodePtrs.end());

    communicationNodePtrs.erase(communicationNodePtr);

    communicationNodePtr->Detach();
}

void Agent::RecalculateRoute(
    const TimeType& recalculateStartTime,
    const AgentBehaviorType& initRecalculateRouteWithBehavior)
{
    timeToSearchRoute.SetMinTime(recalculateStartTime);

    recalculateRoute = true;
    recalculateRouteWithBehavior = initRecalculateRouteWithBehavior;
}

void Agent::SetVertexId(const VertexIdType& vertexId)
{
    if (lastVertexId != vertexId) {

        if (!shouldPassVertexIds.empty()) {
            if (shouldPassVertexIds.front() == vertexId) {
                shouldPassVertexIds.pop_front();
            }
        }

        if (!communicationNodePtrs.empty()) {
            const Vertex position = theAgentGisPtr->GetVertex(vertexId);

            typedef set<shared_ptr<AgentCommunicationNode> >::const_iterator IterType;

            for(IterType iter = communicationNodePtrs.begin();
                iter != communicationNodePtrs.end(); iter++) {
                (*iter)->ArrivedAtVertexNotification(vertexId, position);
            }
        }
    }

    lastVertexId = vertexId;
}

bool Agent::WaitingAtDestinationEntrace() const
{
    if (!(*this).WaitingAtEntrance()) {
        return false;
    }//if//

    // In vehicle behavior, entrance wait occurs
    // only at the destiantion position.
    
    return ((/*jay*/desiredNextPositionId == destPositionId) ||

            ((currentBehaviorPtr != nullptr) &&
             (currentBehaviorPtr->GetBehaviorType() == AGENT_BEHAVIOR_VEHICLE)));
}

bool Agent::WaitingAtEntrance() const
{
    return (entranceWaitStartTime != INFINITE_TIME);
}

bool Agent::OtherDestinationSeemsToBeBetter()
{
    if (!canChangeToOtherDestinationCandidate) {
        return false;
    }

    if ((*this).WaitingAtDestinationEntrace()) {
  
        const AgentResource resource(this);
        
        if ((currentBehaviorPtr != nullptr) &&
            (currentBehaviorPtr->GetBehaviorType() == AGENT_BEHAVIOR_VEHICLE)) {

            if (resource.ExceededWaitVehicleEntranceTime()) {
                return true;
            }//if//
        }
        else {
                        
            if (resource.ExceededWaitEntranceTime()) {
                return true;
            }
        }
    }
    
    return false;
}

const Vertex& Agent::GetCurrentPosition() const
{
    return (status.currentPosition);
}

const Vertex& Agent::GetNextPosition() const
{
    return (status.nextPosition);
}

AgentProfileType AgentResource::ProfileType() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->profilePtr->GetProfileType();
}

TimeType AgentResource::CurrentTime() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->simulatorPtr->CurrentTime();
}

TimeType AgentResource::CurrentBehaviorSpentTime() const
{
    (*this).CheckAgentAvailability();

    return ((*this).CurrentTime() - agentPtr->currentBehaviorStartTime);
}

AgentMobilityClassType AgentResource::MobilityClass() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->profilePtr->GetMobilityClass();
}

AgentTicketType AgentResource::TicketType() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->profilePtr->GetTicketType();
}

AgentUserType AgentResource::UserType() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->profilePtr->GetUserType();
}

const Vertex& AgentResource::Position() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->GetCurrentPosition();
}

string AgentResource::GetProfileName() const
{
    if (agentPtr == nullptr) {
        return string();
    }

    return agentPtr->profilePtr->GetProfileName();
}

const Vertex& AgentResource::DebugNextPosition() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->GetNextPosition();
}

ObjectMobilityPosition AgentResource::MobilityPosition() const
{
    (*this).CheckAgentAvailability();

    const Vertex& position = (*this).Position();
    const TimeType currentTime = (*this).CurrentTime();

    return ObjectMobilityPosition(
        currentTime,
        currentTime,
        position.x,
        position.y,
        position.z,
        true/*theHeightContainsGroundHeightMeters*/,
        90 - agentPtr->directionRadians*(180./PI),
        0.0, 0.0, 0.0, 0.0);
}

ObjectMobilityPosition AgentResource::MobilityPositionForTime(const TimeType& time) const
{
    (*this).CheckAgentAvailability();

    const Vertex& position = (*this).Position();
    const TimeType currentTime = (*this).CurrentTime();

    return ObjectMobilityPosition(
        currentTime,
        currentTime,
        position.x,
        position.y,
        position.z,
        true/*theHeightContainsGroundHeightMeters*/,
        (90 - agentPtr->directionRadians*(180./PI)),
        0.0, 0.0, 0.0, 0.0);
}

AgentBehaviorType AgentResource::GetBehaviorType() const
{
    (*this).CheckAgentAvailability();

    if (agentPtr->currentBehaviorPtr == nullptr) {
        return AGENT_BEHAVIOR_NOTHING;
    }

    return agentPtr->currentBehaviorPtr->GetBehaviorType();
}

bool AgentResource::WaitingAtEntrance() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->WaitingAtEntrance();
}

bool AgentResource::ExceededWaitEntranceTime() const
{
    assert((*this).WaitingAtEntrance());
    (*this).CheckAgentAvailability();

    // Entrance wait check should be executed after at least a time step.

    const TimeType waitTime = std::max(
        agentPtr->simulatorPtr->TimeStep(),
        (*this).EntranceWaitTime());

    return ((*this).CurrentTime() >= (agentPtr->entranceWaitStartTime + waitTime));
}

bool AgentResource::ExceededWaitVehicleEntranceTime() const
{
    assert((*this).WaitingAtEntrance());
    (*this).CheckAgentAvailability();

    // Entrance wait check should be executed after at least a time step.

    const TimeType waitTime = std::max(
        agentPtr->simulatorPtr->TimeStep(),
        (*this).VehicleEntranceWaitTime());
    
    return ((*this).CurrentTime() >= (agentPtr->entranceWaitStartTime + waitTime));
}

const GisPositionIdType& AgentResource::DestPositionId() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->destPositionId;
}

const GisPositionIdType& AgentResource::ExtraDestPoiId() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->extraDestPoiId;
}

const GisPositionIdType& AgentResource::HomePositionId() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->GetHomePositinId();
}

RouteNumberType  AgentResource::CurrentRouteNumber() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->currentRouteNumber;
}

const AgentRoute& AgentResource::CurrentRoute() const
{
    (*this).CheckAgentAvailability();

    assert(!agentPtr->currentRouteList.routePtrs.empty());
    return *agentPtr->currentRouteList.routePtrs.front();
}

StopIdType AgentResource::GetNextRouteStopId() const
{
    (*this).CheckAgentAvailability();
    assert(agentPtr->currentRouteList.routePtrs.size() > 1);

    typedef list<shared_ptr<AgentRoute> >::const_iterator IterType;

    IterType iter = agentPtr->currentRouteList.routePtrs.begin();
    iter++;

    return (*iter)->GetStartStopId();
}

const set<GisPositionIdType>& AgentResource::UnreachableDestinationIds() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->unreachableDestinationIds;
}

const AgentTaskPurposeType& AgentResource::CurrentPurpose() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->CurrentTask().GetPurpose();
}

HighQualityRandomNumberGenerator& AgentResource::GetRandomNumberGenerator() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->aRandomNumberGenerator;
}

HighQualityRandomNumberGenerator& AgentResource::GetRandomNumberGeneratorForDestinationChoice() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->aRandomNumberGeneratorForDestinationChoice;
}

TimeType AgentResource::LastDelay() const
{
    (*this).CheckAgentAvailability();

    return agentPtr->lastDelay;
}

TimeType AgentResource::NextDelay() const
{
    (*this).CheckAgentAvailability();

    if (agentPtr->currentBehaviorPtr == nullptr) {
        return ZERO_TIME;
    }
    if (agentPtr->currentBehaviorPtr->GetBehaviorType() == AGENT_BEHAVIOR_FREEWALK) {
        return ZERO_TIME;
    }

    const AgentRouteCost& routeCost = agentPtr->currentBehaviorPtr->GetRoute().totalCost;
    const TimeType currentTime = (*this).CurrentTime();

    return std::max(ZERO_TIME, currentTime - routeCost.ArrivalTime());
}

TimeType AgentResource::TripDelay() const
{
    (*this).CheckAgentAvailability();

    if (agentPtr->currentBehaviorPtr == nullptr) {
        return ZERO_TIME;
    }

    if (!agentPtr->timeToSearchRoute.specifiedArrivalTime) {
        return ZERO_TIME;
    }

    const AgentRouteCost& routeCost = agentPtr->currentRouteList.totalCost;

    return std::max(ZERO_TIME, routeCost.ArrivalTime() - agentPtr->timeToSearchRoute.arrivalTime);
}

TimeType AgentResource::ArrivalDelay() const
{
    (*this).CheckAgentAvailability();

    if (agentPtr->currentBehaviorPtr == nullptr) {
        return ZERO_TIME;
    }

    if (!agentPtr->timeToSearchRoute.specifiedArrivalTime) {
        return ZERO_TIME;
    }

    const TimeType currentTime = (*this).CurrentTime();

    return std::max(ZERO_TIME, currentTime - agentPtr->timeToSearchRoute.arrivalTime);
}

void AgentResource::SetPosition(const Vertex& position)
{
    (*this).CheckAgentAvailability();

    (*this).Status().nextPosition = position;
}

void AgentResource::SetVertexId(const VertexIdType& vertexId)
{
    (*this).CheckAgentAvailability();

    agentPtr->SetVertexId(vertexId);
}

void AgentResource::SetCongestion(const double value)
{
    (*this).CheckAgentAvailability();

    agentPtr->healthOrUtilityFactor.values[AGENT_HEALTH_FACTOR_CONGESTION] = value;

    agentPtr->congestionTrace.SetValue((*this).Congestion());
}

bool AgentResource::ExceededRouteRecalculationTime() const
{
    (*this).CheckAgentAvailability();

    const TimeType recalculationTime = (*this).RouteRecalculationTime();

    if (recalculationTime <= ZERO_TIME) {
        return false;
    }

    return ((*this).CurrentTime() > agentPtr->lastRouteCalculatedTime + recalculationTime);
}

void AgentResource::SetLastPathQueryTriggerTime()
{
    (*this).CheckAgentAvailability();

    agentPtr->lastPathQueryTriggerTime = (*this).CurrentTime();
}

bool AgentResource::IsPathQueryTriggerAvailable(
    const double congestion,
    const TimeType& vehicleDelay) const
{
    (*this).CheckAgentAvailability();

    const double pathQueryProbability = (*this).PathQueryProbability();

    if (pathQueryProbability <= 0) {
        return false;
    }

    const TimeType minPathQueryDuration = (*this).MinPathQueryInterval();

    if ((*this).CurrentTime() <= agentPtr->lastPathQueryTriggerTime + minPathQueryDuration) {
        return false;
    }

    HighQualityRandomNumberGenerator& randomnumberGenerator = (*this).GetRandomNumberGenerator();

    const double aValue = randomnumberGenerator.GenerateRandomDouble();

    if (aValue > pathQueryProbability) {
        return false;
    }

    const bool isAvailable =
        ((*this).LastDelay() >= (*this).LastDelayQueryTrigger()*SECOND ||
         (*this).NextDelay() >= (*this).NextDelayQueryTrigger()*SECOND ||
         (*this).TripDelay() >= (*this).TripDelayQueryTrigger()*SECOND ||
         vehicleDelay >= (*this).VehicleDelayQueryTrigger()*SECOND ||
         congestion >= (*this).CongestionQueryTrigger() ||
         (*this).Utility1() >= (*this).Utility1QueryTrigger() ||
         (*this).Utility2() >= (*this).Utility2QueryTrigger());

    ostringstream outStream;
    outStream << "PathQuery = " << isAvailable
              << ", LastDelay = " << (*this).LastDelay() / SECOND
              << ", NextDelay = " << (*this).NextDelay() / SECOND
              << ", TripDelay = " << (*this).TripDelay() / SECOND
              << ", ArrivalDelay = " << (*this).ArrivalDelay() / SECOND
              << ", VehicleDelay = " << vehicleDelay / SECOND
              << ", Congestion = " << congestion
              << ", Utility = " << (*this).Utility1() << " / " << (*this).Utility2();

    (*this).OutputTrace(outStream.str());

    return isAvailable;
}

void AgentResource::AssignTaxi(const shared_ptr<Taxi>& initTaxiPtr)
{
    (*this).CheckAgentAvailability();

    if (agentPtr->currentBehaviorPtr != nullptr) {
        agentPtr->currentBehaviorPtr->AssignTaxi(initTaxiPtr);
    }
}

void AgentResource::RecalculateRoute(const TimeType& recalculateStartTime)
{
    (*this).CheckAgentAvailability();

    agentPtr->RecalculateRoute(recalculateStartTime);
}

void AgentResource::RecalculateRoute()
{
    (*this).CheckAgentAvailability();

    agentPtr->RecalculateRoute((*this).CurrentTime());
}

void AgentResource::RecalculateRouteWithBehavior(const AgentBehaviorType& behavior)
{
    (*this).CheckAgentAvailability();

    agentPtr->RecalculateRoute((*this).CurrentTime(), behavior);
}

void AgentResource::RecalculateRouteWithNotAvailableBehaviorSpecification(
    const set<AgentBehaviorType>& notAvailableBehaviorTypes)
{
    (*this).CheckAgentAvailability();

    agentPtr->notAvailableBehavorTypesForNextRouteCalculation = notAvailableBehaviorTypes;
    agentPtr->RecalculateRoute((*this).CurrentTime());
}

void AgentResource::ArrivedAtDeadEndNotification()
{
    (*this).CheckAgentAvailability();

    const set<shared_ptr<AgentCommunicationNode> >& communicationNodePtrs = agentPtr->communicationNodePtrs;

    typedef set<shared_ptr<AgentCommunicationNode> >::const_iterator IterType;

    for(IterType iter = communicationNodePtrs.begin();
        iter != communicationNodePtrs.end(); iter++) {

        (*iter)->ArrivedAtDeadEndNotification();
    }
}

void AgentResource::ArrivedAtDestinationNotification()
{
    (*this).CheckAgentAvailability();

    const set<shared_ptr<AgentCommunicationNode> >& communicationNodePtrs = agentPtr->communicationNodePtrs;

    typedef set<shared_ptr<AgentCommunicationNode> >::const_iterator IterType;

    for(IterType iter = communicationNodePtrs.begin();
        iter != communicationNodePtrs.end(); iter++) {

        (*iter)->ArrivedAtDestinationNotification();
    }
}

void AgentResource::EnteredToDestinationNotification()
{
    (*this).CheckAgentAvailability();

    const set<shared_ptr<AgentCommunicationNode> >& communicationNodePtrs = agentPtr->communicationNodePtrs;

    typedef set<shared_ptr<AgentCommunicationNode> >::const_iterator IterType;

    for(IterType iter = communicationNodePtrs.begin();
        iter != communicationNodePtrs.end(); iter++) {

        (*iter)->EnteredToDestinationNotification();
    }
}

void AgentResource::ArrivedAtGisPositionNotification()
{
    (*this).CheckAgentAvailability();

    const set<shared_ptr<AgentCommunicationNode> >& communicationNodePtrs = agentPtr->communicationNodePtrs;
    const GisPositionIdType currentPositionId = agentPtr->currentPositionId;

    typedef set<shared_ptr<AgentCommunicationNode> >::const_iterator IterType;

    for(IterType iter = communicationNodePtrs.begin();
        iter != communicationNodePtrs.end(); iter++) {

        (*iter)->ArrivedAtGisPositionNotification(currentPositionId);
    }
}

void AgentResource::ReceivePhysicalData(SensingSharedInfoType& broadcastData)
{
    assert((*this).IsAvailable());

    // receiving process
}

void AgentResource::UnreachableDestinationNotification()
{
    (*this).CheckAgentAvailability();

    const set<shared_ptr<AgentCommunicationNode> >& communicationNodePtrs = agentPtr->communicationNodePtrs;
    const GisPositionIdType& destPositionId = agentPtr->destPositionId;

    if (destPositionId == UNREACHABLE_POSITION_ID) {
        return;
    }

    typedef set<shared_ptr<AgentCommunicationNode> >::const_iterator IterType;

    for(IterType iter = communicationNodePtrs.begin();
        iter != communicationNodePtrs.end(); iter++) {

        (*iter)->UnreachableDestinationNotification(destPositionId);
    }
}

void AgentResource::SetDestination(const Vertex& position, const bool byCommunication)
{
    (*this).CheckAgentAvailability();

    const GisSubsystem& subsystem = agentPtr->theAgentGisPtr->GetSubsystem();

    vector<GisObjectType> prioritizedDestinationObjectTypes;
    prioritizedDestinationObjectTypes.push_back(GIS_POI);
    prioritizedDestinationObjectTypes.push_back(GIS_BUILDING);
    prioritizedDestinationObjectTypes.push_back(GIS_PARK);

    const GisPositionIdType positionId =
        subsystem.GetPositionId(position, prioritizedDestinationObjectTypes);

    if (positionId.IsInvalid()) {
        return;
    }

    (*this).SetDestination(positionId, position, byCommunication);
}

void AgentResource::SetDestination(
    const GisPositionIdType& positionId,
    const Vertex& position,
    const bool byCommunication)
{
    (*this).CheckAgentAvailability();

    const GisSubsystem& subsystem = agentPtr->theAgentGisPtr->GetSubsystem();
    const VertexIdType destVertexId = subsystem.GetNearestVertexId(positionId, position);

    agentPtr->ChangeToSpecificDestination(positionId, destVertexId, byCommunication);
}

void AgentResource::AddUnreachablePositions(
    const list<GisPositionIdType>& unreachablePositionIds,
    const bool byCommunication)
{
    (*this).CheckAgentAvailability();

    agentPtr->AddUnreachablePositions(unreachablePositionIds, byCommunication);
}


void AgentResource::UpdateCurrentPositionIdToDesiredPositionId()
{
    agentPtr->currentPositionId = agentPtr->desiredNextPositionId;
}


void AgentResource::SetCurrentPositionId(const GisPositionIdType& positionId)
{
    (*this).CheckAgentAvailability();

    agentPtr->currentPositionId = positionId;

    // Make sure desired position is equal to current position when
    // current position is set, i.e. override current desired position.

    agentPtr->desiredNextPositionId = agentPtr->currentPositionId;
}


void AgentResource::SetDesiredNextPositionId(const GisPositionIdType& positionId)
{
    (*this).CheckAgentAvailability();
    agentPtr->desiredNextPositionId = positionId;
}


void AgentResource::SetExtraPoiId(const GisPositionIdType& positionId)
{
    (*this).CheckAgentAvailability();

    agentPtr->extraCurrentPoiId = positionId;
}
void AgentResource::SetCurrentPositionId(const GisObjectType& objectType, const VariantIdType& variantId)
{
    (*this).CheckAgentAvailability();

    (*this).SetCurrentPositionId(GisPositionIdType(objectType, variantId));
}


void AgentResource::SetDesiredNextPositionId(const GisObjectType& objectType, const VariantIdType& variantId)
{
    (*this).CheckAgentAvailability();

    (*this).SetDesiredNextPositionId(GisPositionIdType(objectType, variantId));
}

void AgentResource::SetDirectionRadians(const double directionRadians)
{
    (*this).CheckAgentAvailability();

    agentPtr->directionRadians = directionRadians;
}

void AgentResource::SetOwnerAgent(
    const AgentIdType& ownerAgentId)
{
    (*this).CheckAgentAvailability();

    agentPtr->simulatorPtr->SetOwnerAgent(*this, ownerAgentId);
}

void AgentResource::RemoveOwnerAgent()
{
    (*this).CheckAgentAvailability();

    if (agentPtr->HasParent()) {
        (*this).SetOwnerAgent();
    } else {
        agentPtr->simulatorPtr->RemoveOwnerAgentChange(*this);
    }
}

void AgentResource::WaitEntrance()
{
    if ((*this).WaitingAtEntrance()) {
        return;
    }

    (*this).CheckAgentAvailability();

    agentPtr->entranceWaitStartTime = (*this).CurrentTime();
}

void AgentResource::AllowedEntrance()
{
    if (!WaitingAtEntrance()) {
        return;
    }

    (*this).EndEntranceWaiting();

    if ((*this).GetBehaviorType() == AGENT_BEHAVIOR_VEHICLE) {
        
        // Driver agent enterd to parking.
        // After parking a car, the agent will go to destination position with walking.

    }
    else {
        assert(GetCurrentPositionId() == GetDesiredPositionId());
        
        if (GetCurrentPositionId() == DestPositionId()) {
            (*this).EnteredToDestinationNotification();
        }
    }
}

void AgentResource::EndEntranceWaiting()
{
    (*this).CheckAgentAvailability();

    agentPtr->entranceWaitStartTime = INFINITE_TIME;

}//ResetEntranceState//

//----------------------------------------------------------------------------------

enum AgentParameterValueType {
    AGENT_PARAMETER_VALUE_INT,
    AGENT_PARAMETER_VALUE_DOUBLE,
    AGENT_PARAMETER_VALUE_STRING,
};

class NoFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return values.front();
    }
};
class PlusFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return values[0] + values[1];
    }
};
class MinusFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return values[0] - values[1];
    }
};
class DivFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        if (values[1] == 0) {
            cerr << "Error: occured 0 division. Check agent profile forumula." << endl;
            exit(1);
        }
        return values[0] / values[1];
    }
};
class MultiFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return values[0] * values[1];
    }
};
class ModFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        if (values[1] == 0) {
            cerr << "Error: occured 0 mod. Check agent profile forumula." << endl;
            exit(1);
        }
        return int(values[0]) % int(values[1]);
    }
};
class EplusFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return values[0] * std::pow(10, values[1]);
    }
};
class EminusFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return values[0] * std::pow(10, -values[1]);
    }
};
class Log10Formula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {

        return log10(values[0]);
    }
};
class LogNFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return log(values[0]);
    }
};
class PowFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return pow(values[0], values[1]);
    }
};
class MinFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return std::min(values[0], values[1]);
    }
};
class MaxFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return std::max(values[0], values[1]);
    }
};
class SqrtFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return (sqrt(values[0]));
    }
};
class SinFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return (std::sin(values[0]));
    }
};
class CosFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return (std::cos(values[0]));
    }
};
class TanFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return (std::tan(values[0]));
    }
};
class AbsFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return fabs(values[0]);
    }
};
class CeilFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return ceil(values[0]);
    }
};
class FloorFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return floor(values[0]);
    }
};
class PiFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return PI;
    }
};
class ExpFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return (exp(values[0]));
    }
};
class UniFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return double(randGen.GenerateRandomInt(
                          static_cast<int>(values[0]),
                          static_cast<int>(values[1])));
    }
};
class UnidFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        const double largeValue = std::max(values[0], values[1]);
        const double samllValue = std::min(values[0], values[1]);

        if (largeValue == samllValue) {
            return samllValue;
        }

        return ((largeValue - samllValue) * randGen.GenerateRandomDouble() + samllValue);
    }
};
class NormalFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {

        // central limit theorem

        double total = 0;

        for (int i = 0; i < 12; i++) {
            total += randGen.GenerateRandomDouble();
        }

        return (values[1] * (total - 6.0) + values[0]);
    }
};
class ExpDistributionFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {
        return (- values[0] * log(1 - randGen.GenerateRandomDouble()));
    }
};
class PoissonFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {

        int i = 0;
        double aValue = exp(values[0])*randGen.GenerateRandomDouble();

        for(; aValue > 1.0; i++) {
            aValue *= randGen.GenerateRandomDouble();
        }

        return static_cast<double>(i);
    }
};
class ErlangFormula : public AgentValueFormula::Formula {
public:
    virtual double operator()(
        const vector<double>& values,
        const AgentResource& resource,
        HighQualityRandomNumberGenerator& randGen) const {

        const int phase = static_cast<int>(values[1]);
        double aValue = 1.0;

        for(int i = 0; i < phase; i++) {
            aValue *= (1 - randGen.GenerateRandomDouble());
        }

        return (- values[0] / static_cast<double>(phase) * log(aValue));
    }
};

const shared_ptr<AgentValueFormula::Formula> AgentValueFormula::formulaPtrs[] = {
    shared_ptr<AgentValueFormula::Formula>(new NoFormula()),
    shared_ptr<AgentValueFormula::Formula>(new PlusFormula()),
    shared_ptr<AgentValueFormula::Formula>(new MinusFormula()),
    shared_ptr<AgentValueFormula::Formula>(new DivFormula()),
    shared_ptr<AgentValueFormula::Formula>(new MultiFormula()),
    shared_ptr<AgentValueFormula::Formula>(new ModFormula()),
    shared_ptr<AgentValueFormula::Formula>(new EplusFormula()),
    shared_ptr<AgentValueFormula::Formula>(new EminusFormula()),
    shared_ptr<AgentValueFormula::Formula>(new Log10Formula()),
    shared_ptr<AgentValueFormula::Formula>(new LogNFormula()),
    shared_ptr<AgentValueFormula::Formula>(new PowFormula()),
    shared_ptr<AgentValueFormula::Formula>(new MinFormula()),
    shared_ptr<AgentValueFormula::Formula>(new MaxFormula()),
    shared_ptr<AgentValueFormula::Formula>(new SqrtFormula()),
    shared_ptr<AgentValueFormula::Formula>(new SinFormula()),
    shared_ptr<AgentValueFormula::Formula>(new CosFormula()),
    shared_ptr<AgentValueFormula::Formula>(new TanFormula()),
    shared_ptr<AgentValueFormula::Formula>(new AbsFormula()),
    shared_ptr<AgentValueFormula::Formula>(new CeilFormula()),
    shared_ptr<AgentValueFormula::Formula>(new FloorFormula()),
    shared_ptr<AgentValueFormula::Formula>(new PiFormula()),
    shared_ptr<AgentValueFormula::Formula>(new ExpFormula()),
    shared_ptr<AgentValueFormula::Formula>(new UniFormula()),
    shared_ptr<AgentValueFormula::Formula>(new UnidFormula()),
    shared_ptr<AgentValueFormula::Formula>(new NormalFormula()),
    shared_ptr<AgentValueFormula::Formula>(new ExpDistributionFormula()),
    shared_ptr<AgentValueFormula::Formula>(new PoissonFormula()),
    shared_ptr<AgentValueFormula::Formula>(new ErlangFormula()),
};

static inline
bool CanRemoveOutmostArc(const string& aString)
{
    if (aString.empty() ||
        !(aString[0] == '(' && aString[aString.length() - 1] == ')')) {
        return false;
    }

    size_t numberStartArcs = 1;

    for(size_t i = 1; i < aString.size() - 1; i++) {

        if (aString[i] == '(') {
            numberStartArcs++;
        } else if (aString[i] == ')') {
            numberStartArcs--;

            if (numberStartArcs == 0) {
                return false;
            }
        }
    }

    return (numberStartArcs == 1);
}

static inline
bool IsFunctionString(const string& aString)
{
    if (aString.empty() || !isalpha(aString[0])) {
        return false;
    }

    size_t functionNameEndPos = aString.find_first_of("+-*/%^(.");

    if (functionNameEndPos != string::npos &&
        aString[functionNameEndPos] == '(') {

        const string functionContents = aString.substr(functionNameEndPos);

        return CanRemoveOutmostArc(functionContents);
    }

    return false;
}

static inline
bool IsVariableString(const string& aString)
{
    if (aString.empty() ||
        !(isalpha(aString[0]) || aString[0] == '_'))  {
        return false;
    }

    return (aString.find_first_of("+-*/%^(.") == string::npos);
}

static inline
void ConvertTimeStringToDoubleSec(
    const string& aString, double& value, bool& success)
{
    deque<string> timeStrings;
    TokenizeToTrimmedLowerStringWithArc(aString, ":", timeStrings);

    int hour = 0;
    int minute = 0;
    int second = 0;

    ConvertStringToInt(timeStrings[0], hour, success);
    if (!success) {
        return;
    }

    ConvertStringToInt(timeStrings[1], minute, success);
    if (!success) {
        return;
    }

    if (timeStrings.size() >= 3) {
        ConvertStringToInt(timeStrings[2], second, success);
        if (!success) {
            return;
        }
    }

    success = true;
    value = 60*60*hour + 60*minute + second;
}

static inline
bool IsEnumValue(const string& aString)
{
    if (IsFunctionString(aString)) {
        return false;
    }

    bool success;
    double aValue;

    ConvertStringToDouble(aString, aValue, success);

    return !success;
}

static inline
void TokenizeToConditionOrTaskString(
    const string& aString,
    vector<string>& tokens)
{
    tokens.clear();

    size_t lastToneStartPos = 0;
    size_t currentPos = 0;

    while (currentPos < aString.size()) {
        const size_t arcPos = aString.find('(', lastToneStartPos);

        size_t numberArcs = 0;

        for(currentPos = arcPos; currentPos < aString.size(); currentPos++) {
            const char aChar = aString[currentPos];

            if (aChar == '(') {
                numberArcs++;
            } else if (aChar == ')') {
                numberArcs--;
            }

            if (numberArcs == 0) {
                tokens.push_back(aString.substr(lastToneStartPos, currentPos - lastToneStartPos + 1));
                lastToneStartPos = currentPos + 1;
                break;
            }
        }
    }
}

static inline
string NonspacedString(const string& aString)
{
    string nonspacedString;

    for(size_t i = 0; i < aString.size(); i++) {
        if (aString[i] != ' ') {
            nonspacedString.push_back(aString[i]);
        }
    }

    return nonspacedString;
}

static inline
string NonspacedRawFormulaString(const string& aString)
{
    string nonspacedString = NonspacedString(aString);

    size_t numberStartArcs = 0;
    size_t numberEndArcs = 0;

    for(size_t i = 0; i < nonspacedString.size(); i++) {
        const char aChar = nonspacedString[i];

        if (aChar == '(') {
            numberStartArcs++;
        } else  if (aChar == ')') {
            numberEndArcs++;
        }
    }

    if (numberStartArcs != numberEndArcs) {
        cerr << "Error: lack of arcs in formula " << aString << endl;
        exit(1);
    }

    while (CanRemoveOutmostArc(nonspacedString)) {

        nonspacedString = NonspacedString(nonspacedString.substr(1, nonspacedString.length() - 2));
    }

    return nonspacedString;
}

AgentValueFormula::AgentValueFormula(
    const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters,
    const double simStartTimeSec,
    const string& aString,
    map<string, AgentCharactorIdType>& charactorIds,
    const double initDefaultValue)
    :
    inputFormulaString(aString),
    defaultValue(initDefaultValue)
{
    if (!aString.empty()) {

        string nonEqualString = NonspacedRawFormulaString(aString);

        if (nonEqualString[0] == '=') {
            nonEqualString = nonEqualString.substr(1);
        }

        if (nonEqualString != "-") {
            (*this).AddFormulaUnitRecursively(
                parameters, simStartTimeSec, nonEqualString, charactorIds);

            assert(formulaUnits.size() > 0);
            (*this).PrecalculateFormulaUnit(0);
        }
    }
}

AgentValueFormula::AgentValueFormula(
    const string& aString,
    const double initDefaultValue)
    :
    inputFormulaString(aString),
    defaultValue(initDefaultValue)
{
    LabelMap<AgentStatusIdType, AgentValueFormula> noParameters;
    const double simStartTimeSec = 0;
    map<string, AgentCharactorIdType> noCharactorIds;

    if (!aString.empty()) {

        string nonEqualString = NonspacedRawFormulaString(aString);

        if (nonEqualString[0] == '=') {
            nonEqualString = nonEqualString.substr(1);
        }

        if (nonEqualString != "-") {
            (*this).AddFormulaUnitRecursively(
                noParameters, simStartTimeSec, nonEqualString, noCharactorIds);

            assert(formulaUnits.size() > 0);
            (*this).PrecalculateFormulaUnit(0);
        }
    }
}

void AgentValueFormula::PrecalculateFormulaUnit(const FormulaUnitIdType& unitId)
{
    const AgentResource emptyResource(nullptr);
    HighQualityRandomNumberGenerator notUsedRandGen(0);

    FormulaUnit& formulaUnit = (*this).GetFormulaUnit(unitId);

    if (formulaUnit.operation != FORMULA_OPERATION_NONE) {
        vector<pair<FormulaUnitIdType, double> >& values = formulaUnit.values;

        for(size_t i = 0; i < values.size(); i++) {
            pair<FormulaUnitIdType, double>& aValue = values[i];
            const FormulaUnitIdType valueUnitId = aValue.first;

            if (valueUnitId != NO_FORMULA_UNIT_ID) {
                assert(valueUnitId != unitId);

                const FormulaUnit& valueFormulaUnit =
                    (*this).GetFormulaUnit(valueUnitId);

                if (MayBeConstValue(valueFormulaUnit.operation)) {
                    (*this).PrecalculateFormulaUnit(valueUnitId);
                }
                if (valueFormulaUnit.CompletedAllCalculation()) {
                    aValue.second = (*this).Calculate(valueUnitId, emptyResource, notUsedRandGen);
                    aValue.first = NO_FORMULA_UNIT_ID;
                }
            }
        }
    }
}

AgentValueFormula::FormulaUnit& AgentValueFormula::GetFormulaUnit(const FormulaUnitIdType& unitId)
{
    assert(unitId != NO_FORMULA_UNIT_ID);
    assert(unitId < formulaUnits.size());
    return formulaUnits[unitId];
}

const AgentValueFormula::FormulaUnit& AgentValueFormula::GetFormulaUnit(const FormulaUnitIdType& unitId) const
{
    assert(unitId != NO_FORMULA_UNIT_ID);
    assert(unitId < formulaUnits.size());
    return formulaUnits[unitId];
}

bool AgentValueFormula::FormulaUnit::CompletedAllCalculation() const
{
    if (operation >= FORMULA_OPERATION_DISTRIBUTION_START) {
        return false;
    }

    for(size_t i = 0; i < values.size(); i++) {
        if (values[i].first != NO_FORMULA_UNIT_ID) {
            return false;
        }
    }

    return true;
}

TimeType AgentValueFormula::CalculateTime(
    const AgentResource& resource,
    HighQualityRandomNumberGenerator& randGen,
    const bool calculateMaxValue) const
{
    if ((*this).IsNull()) {
        if (calculateMaxValue) {
            return INFINITE_TIME;
        }
        return resource.CurrentTime();
    }

    return static_cast<TimeType>((*this).CalculateDouble(resource, randGen, calculateMaxValue) * SECOND);
}

double AgentValueFormula::CalculateDouble(
    const AgentResource& resource,
    HighQualityRandomNumberGenerator& randGen,
    const bool calculateMaxValue) const
{
    if ((*this).IsNull()) {
        return defaultValue;
    }

    return (*this).Calculate(0, resource, randGen, calculateMaxValue);
}

double AgentValueFormula::CalculateUtility(
    const AgentResource& resource,
    HighQualityRandomNumberGenerator& randGen,
    const AgentHealthOrUtilityFactor& healthOrUtilityFactor,
    const AgentRouteCost& cost) const
{
    if ((*this).IsNull()) {
        return defaultValue;
    }

    return (*this).CalculateUtility(0, resource, randGen, healthOrUtilityFactor, cost);
}

double AgentValueFormula::Calculate(
    const FormulaUnitIdType& unitId,
    const AgentResource& resource,
    HighQualityRandomNumberGenerator& randGen,
    const bool calculateMaxValue) const
{
    assert(unitId < formulaUnits.size());
    assert(unitId != NO_FORMULA_UNIT_ID);

    const FormulaUnit& formulaUnit = (*this).GetFormulaUnit(unitId);
    const vector<pair<FormulaUnitIdType, double> >& values = formulaUnit.values;

    vector<double> doubleValues(values.size());

    for(size_t i = 0; i < values.size(); i++) {
        const pair<FormulaUnitIdType, double>& value = values[i];
        if (value.first == NO_FORMULA_UNIT_ID) {
            doubleValues[i] = value.second;
        } else {
            assert(unitId != value.first);
            doubleValues[i] = (*this).Calculate(value.first, resource, randGen, calculateMaxValue);
        }
    }

    if (IsAgentStatus(formulaUnit.operation)) {
        return resource.Value(ConvertTofAgentStatusId(formulaUnit.operation));
    }

    if (calculateMaxValue) {
        if (formulaUnit.operation == FORMULA_OPERATION_UNI ||
            formulaUnit.operation == FORMULA_OPERATION_UNID) {
            return std::max(doubleValues[0], doubleValues[1]);
        } else if (formulaUnit.operation == FORMULA_OPERATION_NORMAL) {
            return doubleValues[1] * 6.0 + doubleValues[0];
        } else if (formulaUnit.operation == FORMULA_OPERATION_EXP_DISTRIBUTION ||
                   formulaUnit.operation == FORMULA_OPERATION_POISSON ||
                   formulaUnit.operation == FORMULA_OPERATION_ERLANG) {
            return DBL_MAX;
        }
    }

    if (formulaUnit.operation >= NUMBER_FORMULA_TYPES) {
        cerr << "Error: Predefined reserved parameter is available only in UtilityFunction/RoutePriority" << endl;
        exit(1);
    }

    return (*formulaPtrs[formulaUnit.operation])(doubleValues, resource, randGen);
}

double AgentValueFormula::CalculateUtility(
    const FormulaUnitIdType& unitId,
    const AgentResource& resource,
    HighQualityRandomNumberGenerator& randGen,
    const AgentHealthOrUtilityFactor& healthOrUtilityFactor,
    const AgentRouteCost& cost) const
{
    assert(unitId < formulaUnits.size());
    assert(unitId != NO_FORMULA_UNIT_ID);

    const FormulaUnit& formulaUnit = (*this).GetFormulaUnit(unitId);
    const vector<pair<FormulaUnitIdType, double> >& values = formulaUnit.values;

    vector<double> doubleValues(values.size());

    for(size_t i = 0; i < values.size(); i++) {
        const pair<FormulaUnitIdType, double>& value = values[i];
        if (value.first == NO_FORMULA_UNIT_ID) {
            doubleValues[i] = value.second;
        } else {
            assert(unitId != value.first);
            doubleValues[i] = (*this).CalculateUtility(value.first, resource, randGen, healthOrUtilityFactor, cost);
        }
    }

    if (IsRouteCost(formulaUnit.operation)) {
        return cost.values[ConvertToRouteCostId(formulaUnit.operation)];
    }

    if (IsHealthOrUtilityCost(formulaUnit.operation)) {
        return healthOrUtilityFactor.values[
            ConvertToHealthOrUtilityFactorId(formulaUnit.operation)];
    }

    if (IsAgentStatus(formulaUnit.operation)) {
        return resource.Value(ConvertTofAgentStatusId(formulaUnit.operation));
    }

    assert(formulaUnit.operation < NUMBER_FORMULA_TYPES);
    return (*formulaPtrs[formulaUnit.operation])(doubleValues, resource, randGen);
}

void AgentValueFormula::ResolveOperation(
    const string& aString,
    string& leftValue,
    string& rightValue,
    FormulaOperationType& operation)
{
    assert(!aString.empty());

    const string nonspacedString = NonspacedRawFormulaString(aString); //remove outmost "()"

    assert(!IsFunctionString(nonspacedString));
    int currentPos;

    int numberRemainingFirstArcs = 0;

    bool found = false;
    for(currentPos = (int)(nonspacedString.size() - 1); currentPos >= 0; currentPos--) {
        const char aChar = nonspacedString[currentPos];

        if (aChar == '(') {
            numberRemainingFirstArcs--;
        } else  if (aChar == ')') {
            numberRemainingFirstArcs++;
        }

        if (numberRemainingFirstArcs == 0 &&
            ((aChar == '+')  || (aChar == '-'))) {

            if (currentPos == 0 ||
                (currentPos > 0 && nonspacedString[currentPos - 1] != 'e')) {
                found  = true;
            }
            break;
        }
    }

    if (!found) {
        for(currentPos = (int)(nonspacedString.size() - 1); currentPos >= 0; currentPos--) {
            const char aChar = nonspacedString[currentPos];

            if (aChar == '(') {
                numberRemainingFirstArcs--;
            } else  if (aChar == ')') {
                numberRemainingFirstArcs++;
            }

            if (numberRemainingFirstArcs == 0 &&
                ((aChar == '/') || (aChar == '*') || (aChar == '%') || (aChar == '^') || (aChar == '+') || (aChar == '-'))) {
                break;
            }
        }
    }

    if (currentPos < 0) {//>= int(nonspacedString.size())) {
        leftValue = aString;
        operation = FORMULA_OPERATION_NONE;
        return;
    }

    leftValue = nonspacedString.substr(0, currentPos);

    assert(currentPos < int(nonspacedString.size()));

    const char aChar = nonspacedString[currentPos];

    if (aChar == '+') {

        if (currentPos > 0 && nonspacedString[currentPos - 1] == 'e') {

            operation = FORMULA_OPERATION_E_PLUS;
            leftValue = nonspacedString.substr(0, currentPos - 1);

        } else {
            operation = FORMULA_OPERATION_PLUS;
        }

    } else if (aChar == '-') {

        if (currentPos > 0 && nonspacedString[currentPos - 1] == 'e') {

            operation = FORMULA_OPERATION_E_MINUS;
            leftValue = nonspacedString.substr(0, currentPos - 1);

        } else {
            operation = FORMULA_OPERATION_MINUS;
        }

    } else if (aChar == '/') {
        operation = FORMULA_OPERATION_DIV;
    } else if (aChar == '*') {
        operation = FORMULA_OPERATION_MULTI;
    } else if (aChar == '%') {
        operation = FORMULA_OPERATION_MOD;
    } else if (aChar == '^') {
        operation = FORMULA_OPERATION_POW;
    } else {
        cerr << "Error: invalid operation '" << aChar << "' at " << aString << endl;
        exit(1);
    }

    rightValue = nonspacedString.substr(currentPos + 1);
}

AgentValueFormula::FormulaUnitIdType AgentValueFormula::AddFormulaUnitRecursively(
    const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters,
    const double simStartTimeSec,
    const string& aString,
    map<string, AgentCharactorIdType>& charactorIds)
{
    const string nonspacedString = NonspacedRawFormulaString(aString);

    if (IsVariableString(nonspacedString)) {

        return (*this).AddVariableUnit(parameters, nonspacedString);

    } else if (IsFunctionString(nonspacedString)) {

        return (*this).AddFunctionUnitRecursively(
            parameters, simStartTimeSec, nonspacedString, charactorIds);

    } else {

        const FormulaUnitIdType unitId = formulaUnits.size();
        formulaUnits.push_back(FormulaUnit(unitId));

        string leftValue;
        string rightValue;

        FormulaOperationType operation;

        (*this).ResolveOperation(nonspacedString, leftValue, rightValue, operation);

        (*this).GetFormulaUnit(unitId).operation = operation;

        assert(operation < NUMBER_FORMULA_TYPES);

        if (operation == FORMULA_OPERATION_NONE) {
            bool success;
            double aValue;

            if (aString.find(":") != string::npos) {
                double timeSec;
                ConvertTimeStringToDoubleSec(aString, timeSec, success);

                if (timeSec < simStartTimeSec) {
                    cerr << "Error: invalid time" << aString << endl;
                    exit(1);
                }
                aValue = timeSec - simStartTimeSec;
            } else {
                ConvertStringToDouble(aString, aValue, success);
            }

            // Static cast workaround on Linux for Microsoft violation of C++ standard.

            (*this).GetFormulaUnit(unitId).values.push_back(
                make_pair(static_cast<FormulaUnitIdType>(NO_FORMULA_UNIT_ID), aValue));
        } else {
            FormulaUnitIdType leftUnitId = NO_FORMULA_UNIT_ID;
            double noValue = 0;

            if (!leftValue.empty()) {
                leftUnitId =
                    (*this).AddFormulaUnitRecursively(
                        parameters, simStartTimeSec, leftValue, charactorIds);
            }

            const FormulaUnitIdType rightUnitId =
                (*this).AddFormulaUnitRecursively(
                    parameters, simStartTimeSec, rightValue, charactorIds);

            assert(unitId != leftUnitId);
            assert(unitId != rightUnitId);

            (*this).GetFormulaUnit(unitId).values.push_back(make_pair(leftUnitId, noValue));
            (*this).GetFormulaUnit(unitId).values.push_back(make_pair(rightUnitId, noValue));
        }

        return unitId;
    }
}

AgentValueFormula::FormulaOperationType AgentValueFormula::GetFormulaOperation(
    const string& functionString,
    const size_t numberArguments) const
{
    if (numberArguments < 1) {
        cerr << "Error: Function definition needs one or more arguments" << endl;
        exit(1);
    }

    if (functionString == "log10") {
        return FORMULA_OPERATION_LOG10;
    } else if (functionString == "logn" ||
               functionString == "log" ||
               functionString == "logarithm") {
        return FORMULA_OPERATION_LOGN;
    } else if (functionString == "pow" || functionString == "power") {
        return FORMULA_OPERATION_POW;
    } else if (functionString == "min" || functionString == "minimum") {
        return FORMULA_OPERATION_MIN;
    } else if (functionString == "max" || functionString == "maximum") {
        return FORMULA_OPERATION_MAX;
    } else if (functionString == "sqrt" || functionString == "squareroot") {
        return FORMULA_OPERATION_SQRT;
    } else if (functionString == "sin" || functionString == "sine") {
        return FORMULA_OPERATION_SIN;
    } else if (functionString == "cos" || functionString == "cosine") {
        return FORMULA_OPERATION_COS;
    } else if (functionString == "tan" || functionString == "tangent") {
        return FORMULA_OPERATION_TAN;
    } else if (functionString == "abs" || functionString == "absolute") {
        return FORMULA_OPERATION_ABS;
    } else if (functionString == "ceil" || functionString == "ceiling") {
        return FORMULA_OPERATION_CEIL;
    } else if (functionString == "floor") {
        return FORMULA_OPERATION_FLOOR;
    } else if (functionString == "pi") {
        return FORMULA_OPERATION_PI;
    } else if (functionString == "exp" || functionString == "exponential") {
        return FORMULA_OPERATION_EXP;
    } else if (functionString == "uni" || functionString == "uniform") {
        if (numberArguments < 2) {
            cerr << "Error: Uniform distribution needs 2 arguments (min, max)." << endl;
            exit(1);
        }
        return FORMULA_OPERATION_UNI;
    } else if (functionString == "unid" || functionString == "uniformd") {
        if (numberArguments < 2) {
            cerr << "Error: Uniform distribution needs 2 arguments (min, max)." << endl;
            exit(1);
        }
        return FORMULA_OPERATION_UNID;
    } else if (functionString == "normal") {
        if (numberArguments < 2) {
            cerr << "Error: Earlnag distribution needs 2 arguments (average, deviation)." << endl;
            exit(1);
        }
        return FORMULA_OPERATION_NORMAL;
    } else if (functionString == "expdist" || functionString == "exponentialdistribution") {
        return FORMULA_OPERATION_EXP_DISTRIBUTION;
    } else if (functionString == "poisson") {
        return FORMULA_OPERATION_POISSON;
    } else if (functionString == "erlang") {
        if (numberArguments < 2) {
            cerr << "Error: Earlnag distribution needs 2 arguments (lambda, phase)." << endl;
            exit(1);
        }
        return FORMULA_OPERATION_ERLANG;
    }

    return FORMULA_OPERATION_NONE;
}

AgentValueFormula::FormulaUnitIdType AgentValueFormula::AddVariableUnit(
    const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters,
    const string& variableString)
{
    AgentValueFormula::FormulaOperationType operation;

    if (variableString == "_movebypreferedmobilitymeans") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_MODE;

    } else if (variableString == "_arrivaltime") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_ARRIVAL_TIME;

    } else if (variableString == "_totaltraveltime") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_TRAVEL_TIME;

    } else if (variableString == "_totaltraveldistance") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_TRAVEL_DISTANCE;

    } else if (variableString == "_totalpublictransportationdelay") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_VARIABILITY_TIME;

    } else if (variableString == "_price") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_PRICE;

    } else if (variableString == "_totaltransfercount") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_TRANSFER_TIME;

    } else if (variableString == "_totaltransferduration") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_TRANSFER_DURATION;

    } else if (variableString == "_numberofpeopleonroute") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_PASSANGER_CONGESTION;

    } else if (variableString == "_numberofvehiclesonroute") {

        operation = ROUTE_COST_START + AGENT_ROUTE_COST_VEHICLE_CONGESTION;

    } else if (variableString == "_segmentcongestion") {

        operation = HEALTH_OR_UTILITY_FACTOR_START + AGENT_HEALTH_FACTOR_CONGESTION;

    } else if (variableString == "_delayforexpectedarrivaltime") {

        operation = HEALTH_OR_UTILITY_FACTOR_START + AGENT_UTILITY_FACTOR_TRIP_DELAY;
        assert(IsHealthOrUtilityCost(operation));

    } else if (variableString == "_delayforspecfiedarrivaltime") {

        operation = HEALTH_OR_UTILITY_FACTOR_START + AGENT_UTILITY_FACTOR_ARRIVAL_DELAY;
        assert(IsHealthOrUtilityCost(operation));

    } else if (variableString == "_delayforlastviappoint") {

        operation = HEALTH_OR_UTILITY_FACTOR_START + AGENT_UTILITY_FACTOR_LAST_DELAY;
        assert(IsHealthOrUtilityCost(operation));

    } else if (variableString == "_delayfornextviapoint") {

        operation = HEALTH_OR_UTILITY_FACTOR_START + AGENT_UTILITY_FACTOR_NEXT_DELAY;
        assert(IsHealthOrUtilityCost(operation));

    } else if (variableString == "_utility1updatecount") {

        operation = HEALTH_OR_UTILITY_FACTOR_START + AGENT_UTILITY_FACTOR_UTILITY1_COUNTER;
        assert(IsHealthOrUtilityCost(operation));

    } else if (variableString == "_utility2updatecount") {

        operation = HEALTH_OR_UTILITY_FACTOR_START + AGENT_UTILITY_FACTOR_UTILITY2_COUNTER;
        assert(IsHealthOrUtilityCost(operation));

    } else {

        if (!parameters.Contains(variableString)) {
            cerr << "Error: invalid variable " << variableString << endl;
            exit(1);
        }

        operation = ConvertTofFormulaOperationType(parameters.GetId(variableString));
        assert((IsAgentStatus(operation)));
    }

    const FormulaUnitIdType unitId = formulaUnits.size();
    formulaUnits.push_back(FormulaUnit(unitId, operation));

    return unitId;
}

AgentValueFormula::FormulaUnitIdType AgentValueFormula::AddFunctionUnitRecursively(
    const LabelMap<AgentStatusIdType, AgentValueFormula>& parameters,
    const double simStartTimeSec,
    const string& aString,
    map<string, AgentCharactorIdType>& charactorIds)
{
    assert(IsFunctionString(aString));

    const FormulaUnitIdType unitId = formulaUnits.size();
    formulaUnits.push_back(FormulaUnit(unitId));

    vector<string> arguments;
    size_t argumentStartPos = 0;

    const size_t arcPos = aString.find_first_of('(');
    const string functionString = aString.substr(0, arcPos);

    argumentStartPos = arcPos + 1;

    size_t currentPos = arcPos;
    int arcNumber = 0;

    while (arcNumber != -1) {
        currentPos = aString.find_first_of("(),", currentPos + 1);
        assert(currentPos != string::npos);

        if (aString[currentPos] == '(') {
            arcNumber++;
        } else if (aString[currentPos] == ')') {
            arcNumber--;
        } else if (arcNumber == 0 && aString[currentPos] == ',') {
            arguments.push_back(
                aString.substr(argumentStartPos, currentPos - argumentStartPos));
            argumentStartPos = currentPos + 1;
        }
    }

    arguments.push_back(aString.substr(argumentStartPos, aString.size() - argumentStartPos - 1));

    const FormulaOperationType operation = GetFormulaOperation(functionString, arguments.size());

    assert(operation < NUMBER_FORMULA_TYPES);

    (*this).GetFormulaUnit(unitId).operation = operation;

    for(size_t i = 0; i < arguments.size(); i++) {
        const string& argument = arguments[i];

        FormulaUnitIdType argUnitId;
        double statusValue = 0;

        argUnitId = (*this).AddFormulaUnitRecursively(
            parameters, simStartTimeSec, argument, charactorIds);

        assert(unitId != argUnitId);
        (*this).GetFormulaUnit(unitId).values.push_back(make_pair(argUnitId, statusValue));
    }

    return unitId;
}

//----------------------------------------------------------------------------------------

static inline
AgentLocationType GetLocationType(const string& locationName)
{
    if (locationName == "randombuilding") {

        return AGENT_LOCATION_RANDOM_BUILDING;

    } else if (locationName == "randompark") {

        return AGENT_LOCATION_RANDOM_PARK;

    } else if (locationName == "randompoi") {

        return AGENT_LOCATION_RANDOM_POI;

    } else if (locationName == "randomintersection") {

        return AGENT_LOCATION_RANDOM_INTERSECTION;

    } else if (locationName == "initiallocation") {

        return AGENT_LOCATION_HOME;

    } else if (locationName == "presentlocation") {

        return AGNET_LOCATION_POSITION_FILE;

    } else if (locationName == "none" || locationName.empty()) {

        return AGENT_LOCATION_NONE;

    } else {

        return AGENT_LOCATION_POI;
    }
}

static inline
void GetLocationsById(
    const GisSubsystem& theGisSubsystem,
    const AgentResource& resource,
    HighQualityRandomNumberGenerator& randGen,
    const string& idString,
    const set<GisPositionIdType>& ignoreLocationIds,
    vector<GisPositionIdType>& locationCandidatetPositionIds)
{
    locationCandidatetPositionIds.clear();

    map<string, AgentCharactorIdType> notUsed;

    const int gisObjectId = AgentValueFormula(
        LabelMap<AgentStatusIdType, AgentValueFormula>(),
        0,
        idString,
        notUsed).CalculateInt(resource, randGen);

    const GisPositionIdType positionId =
        theGisSubsystem.GetPositionId(gisObjectId);

    if (ignoreLocationIds.find(positionId) == ignoreLocationIds.end()) {

        if (positionId.type == GIS_AREA) {

            theGisSubsystem.GetBuildingPositionIdsInArea(
                positionId.id, ignoreLocationIds, locationCandidatetPositionIds);

        } else if (positionId.type == GIS_BUILDING ||
                   positionId.type == GIS_PARK ||
                   positionId.type == GIS_PARK) {

            locationCandidatetPositionIds.push_back(positionId);

        } else {
            cerr << "Error: location id MUST be a building, park, POI or area id. id:" << idString << endl;
            exit(1);
        }
    }
}

static inline
void GetLocationsByName(
    const GisSubsystem& theGisSubsystem,
    const map<string, vector<GisPositionIdType> >& locationGroups,
    const string& locationName,
    const set<GisPositionIdType>& ignoreLocationIds,
    const bool searchIntersection,
    vector<GisPositionIdType>& locationCandidatetPositionIds)
{
    locationCandidatetPositionIds.clear();

    typedef map<string, vector<GisPositionIdType> >::const_iterator IterType;

    IterType iter = locationGroups.find(locationName);

    if (iter != locationGroups.end()) {

        const vector<GisPositionIdType>& positionIds = (*iter).second;

        for(size_t i = 0; i < positionIds.size(); i++) {
            const GisPositionIdType& positionId = positionIds[i];

            if (ignoreLocationIds.find(positionId) == ignoreLocationIds.end()) {

                if (positionId.type == GIS_AREA) {

                    vector<GisPositionIdType> buildingPositionIds;

                    theGisSubsystem.GetBuildingPositionIdsInArea(
                        positionId.id, ignoreLocationIds, buildingPositionIds);

                    for(size_t j = 0; j < buildingPositionIds.size(); j++) {
                        const GisPositionIdType& buildingPositionId = buildingPositionIds[j];

                        if (ignoreLocationIds.find(buildingPositionId) == ignoreLocationIds.end()) {
                            locationCandidatetPositionIds.push_back(buildingPositionId);
                        }
                    }

                } else {
                    locationCandidatetPositionIds.push_back(positionId);
                }
            }
        }

    } else {

        const GisPositionIdType positionId =
            theGisSubsystem.GetPosition(locationName);

        if (ignoreLocationIds.find(positionId) == ignoreLocationIds.end()) {

            if (searchIntersection) {

                if (positionId.type == GIS_INTERSECTION) {
                    locationCandidatetPositionIds.push_back(positionId);

                } else {
                    cerr << "Error: pass location MUST be an intersection name. name:" << locationName << endl;
                    exit(1);
                }

            } else {

                if (positionId.type == GIS_AREA) {

                    theGisSubsystem.GetBuildingPositionIdsInArea(
                        positionId.id, ignoreLocationIds, locationCandidatetPositionIds);

                } else if (positionId.type == GIS_BUILDING ||
                           positionId.type == GIS_PARK ||
                           positionId.type == GIS_POI) {

                    locationCandidatetPositionIds.push_back(positionId);

                } else {
                    cerr << "Error: location MUST be a building, park, POI or area name. name:" << locationName << endl;
                    exit(1);
                }
            }
        }
    }
}

TimeType AgentTask::GetStartTime(const AgentResource& resource) const
{
    if (startTime.IsNull()) {
        return ZERO_TIME;
    }

    return startTime.CalculateTime(resource, resource.GetRandomNumberGenerator());
}

bool AgentTask::SatisfyCondition(const AgentResource& resource) const
{
    for(size_t i = 0; i < conditionCheckers.size(); i++) {
        if (!conditionCheckers[i].Check(resource)) {
            return false;
        }
    }

    return true;
}

void AgentTask::GetDestinationId(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<MultiAgentGis>& theAgentGisPtr,
    const bool ignoreLastPositionFromCandidate,
    GisPositionIdType& positionId,
    bool& isMultipleDestinations,
    AgentResource& resource) const
{
    bool isSpecifiedPosition;
    Vertex notUsedPosition;

    profileAndTaskTablePtr->GetLocationId(
        theParameterDatabaseReader,
        theAgentGisPtr,
        destination,
        shared_ptr<InorderFileCache>(),
        ignoreLastPositionFromCandidate,
        false/*searchIntersection*/,
        positionId,
        isMultipleDestinations,
        isSpecifiedPosition,
        notUsedPosition,
        resource,
        resource.GetRandomNumberGeneratorForDestinationChoice());

    if (isSpecifiedPosition) {
        cerr << "Location specification is available only for initialization location." << endl;
        exit(1);
    }//if//

}

void AgentTask::GetPassVertexIds(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<MultiAgentGis>& theAgentGisPtr,
    deque<VertexIdType>& passVertexIds,
    AgentResource& resource) const
{
    passVertexIds.clear();

    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();

    for(size_t i = 0; i < passIntersectionLocationInfos.size(); i++) {

        GisPositionIdType positionId;
        bool isMultipleDestinations;

        bool isSpecifiedPosition;
        Vertex notUsedPosition;

        profileAndTaskTablePtr->GetLocationId(
            theParameterDatabaseReader,
            theAgentGisPtr,
            passIntersectionLocationInfos[i],
            shared_ptr<InorderFileCache>(),
            true/*ignoreLastPositionFromCandidate*/,
            true/*searchIntersection*/,
            positionId,
            isMultipleDestinations,
            isSpecifiedPosition,
            notUsedPosition,
            resource,
            resource.GetRandomNumberGenerator());

        if (isSpecifiedPosition) {
            cerr << "Location specification is available only for initialization location." << endl;
            exit(1);
        }//if//

        if (positionId.type != GIS_INTERSECTION) {
            cerr << "Specify valid intersection name for pass intersection names" << endl;
            exit(1);
        }

        passVertexIds.push_back(
            subsystem.GetIntersection(positionId.id).GetVertexId());
    }
}

void AgentTaskTable::GetInitialLocationId(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<MultiAgentGis>& theAgentGisPtr,
    const shared_ptr<InorderFileCache>& initialLocationFileCachePtr,
    GisPositionIdType& positionId,
    bool& isSpecifiedPosition,
    Vertex& position,
    AgentResource& resource,
    HighQualityRandomNumberGenerator& randGen) const
{
    bool isMultiplePositionsNotUsed;

    (*this).GetLocationId(
        theParameterDatabaseReader,
        theAgentGisPtr,
        initialLocation,
        initialLocationFileCachePtr,
        positionId,
        isMultiplePositionsNotUsed,
        isSpecifiedPosition,
        position,
        resource,
        randGen);
}

void AgentTaskTable::GetLocationId(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<MultiAgentGis>& theAgentGisPtr,
    const LocationInfo& locationInfo,
    const shared_ptr<InorderFileCache>& initialLocationFileCachePtr,
    GisPositionIdType& positionId,
    bool& isMultiplePositions,
    bool& isSpecifiedPosition,
    Vertex& position,
    AgentResource& resource,
    HighQualityRandomNumberGenerator& randGen) const
{
    profileAndTaskTablePtr->GetLocationId(
        theParameterDatabaseReader,
        theAgentGisPtr,
        locationInfo,
        initialLocationFileCachePtr,
        true/*ignoreLastPositionFromCandidate*/,
        false/*searchIntersection*/,
        positionId,
        isMultiplePositions,
        isSpecifiedPosition,
        position,
        resource,
        randGen);
}

void AgentProfileAndTaskTable::GetLocationId(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<MultiAgentGis>& theAgentGisPtr,
    const LocationInfo& locationInfo,
    const shared_ptr<InorderFileCache>& initialLocationFileCachePtr,
    const bool ignoreLastPositionFromCandidate,
    const bool searchIntersection,
    GisPositionIdType& positionId,
    bool& isMultiplePositions,
    bool& isSpecifiedPosition,
    Vertex& position,
    AgentResource& resource,
    HighQualityRandomNumberGenerator& randGen) const
{
    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();
    const AgentLocationType locationType = GetLocationType(locationInfo.locationName);

    const GisPositionIdType currentPositionId = resource.GetCurrentPositionId();

    set<GisPositionIdType> ignoredDestinationIds = resource.UnreachableDestinationIds();

    if (ignoreLastPositionFromCandidate) {
        ignoredDestinationIds.insert(currentPositionId);
    }

    isSpecifiedPosition = false;

    vector<GisPositionIdType> locationCandidateIds;

    if (locationType == AGENT_LOCATION_NONE) {

        locationCandidateIds.push_back(currentPositionId);

    } else if (locationType == AGENT_LOCATION_RANDOM_BUILDING) {

        bool found;
        GisPositionIdType locationId;

        subsystem.GetARandomPosition(
            GIS_BUILDING,
            ignoredDestinationIds,
            randGen,
            found,
            locationId);

        if (found) {
            locationCandidateIds.push_back(locationId);
        } else {
            cerr << "There is no building for random building specification." << endl;
            exit(1);
        }

    } else if (locationType == AGENT_LOCATION_RANDOM_PARK) {

        bool found;
        GisPositionIdType locationId;

        subsystem.GetARandomPosition(
            GIS_PARK,
            ignoredDestinationIds,
            randGen,
            found,
            locationId);

        if (found) {
            locationCandidateIds.push_back(locationId);
        } else {
            cerr << "There is no park for random park specification." << endl;
            exit(1);
        }

    } else if (locationType == AGENT_LOCATION_RANDOM_POI) {

        bool found;
        GisPositionIdType locationId;

        subsystem.GetARandomPosition(
            GIS_POI,
            ignoredDestinationIds,
            randGen,
            found,
            locationId);

        if (found) {
            locationCandidateIds.push_back(locationId);
        } else {
            cerr << "There is no POI for random POI specification." << endl;
            exit(1);
        }

    } else if (locationType == AGENT_LOCATION_RANDOM_INTERSECTION) {

        bool found;
        GisPositionIdType locationId;

        subsystem.GetARandomPosition(
            GIS_INTERSECTION,
            ignoredDestinationIds,
            randGen,
            found,
            locationId);

        if (found) {
            locationCandidateIds.push_back(locationId);
        } else {
            cerr << "There is no intersection for random intersection specification." << endl;
            exit(1);
        }

    } else  if (locationType == AGENT_LOCATION_HOME) {

        const GisPositionIdType homePositionId = resource.HomePositionId();

        if (homePositionId == GisPositionIdType()) {
            cerr << "There is no InitialLocation position." << endl;
            exit(1);
        }

        locationCandidateIds.push_back(homePositionId);

    } else  if (locationType == AGNET_LOCATION_POSITION_FILE) {

        const AgentIdType agentId = resource.AgentId();
        const double distanceGranularityMeters = 0;
        const string positionFileName =
            theParameterDatabaseReader.ReadString(
                "mobility-init-positions-file",
                agentId);
        
        TraceFileMobilityModel traceMobility(
            theParameterDatabaseReader,
            agentId,
            nullInstanceId,
            *initialLocationFileCachePtr,
            positionFileName,
            agentId,
            distanceGranularityMeters,
            theAgentGisPtr->GetSubsystemPtr());
        
        ObjectMobilityPosition initialPosition;
        
        traceMobility.GetUnadjustedPositionForTime(ZERO_TIME, initialPosition);

        position = Vertex(
            initialPosition.X_PositionMeters(),
            initialPosition.Y_PositionMeters(),
            initialPosition.HeightFromGroundMeters());
        
        vector<GisObjectType> initialGisObjectTypes;

        initialGisObjectTypes.push_back(GIS_BUILDING);
        initialGisObjectTypes.push_back(GIS_PARK);
        initialGisObjectTypes.push_back(GIS_POI);
        
        const GisPositionIdType initialPositionId =
            subsystem.GetPositionIdWithZValue(
                position,
                initialGisObjectTypes);

        if (!initialPositionId.IsValid()) {
            cerr << "There is no GIS object for initialization. (" << position.x << "," << position.y << ") agent " << agentId << endl;
            exit(1);
        }
        
        locationCandidateIds.push_back(initialPositionId);
        isSpecifiedPosition = true;

    } else {

        if (locationInfo.isId) {

            GetLocationsById(
                subsystem,
                resource,
                randGen,
                locationInfo.locationName,
                ignoredDestinationIds,
                locationCandidateIds);

        } else {

            if (thePublicVehicleTablePtr->ContainsLine(locationInfo.locationName)) {
                locationCandidateIds.push_back(
                    GisPositionIdType(
                        GIS_VEHICLE_LINE,
                        thePublicVehicleTablePtr->GetLineId(locationInfo.locationName)));
            } else {

                GetLocationsByName(
                    subsystem,
                    locationGroups,
                    locationInfo.locationName,
                    ignoredDestinationIds,
                    searchIntersection,
                    locationCandidateIds);
            }
        }
    }

    if (locationCandidateIds.empty()) {
        positionId = UNREACHABLE_POSITION_ID;
        isMultiplePositions = false;
        return;
    }

    if (locationCandidateIds.size() == 1) {
        if (ignoredDestinationIds.find(locationCandidateIds.front()) != ignoredDestinationIds.end()) {
            positionId = UNREACHABLE_POSITION_ID;
        } else {
            positionId = locationCandidateIds.front();
        }

        isMultiplePositions = false;
        return;
    }

    isMultiplePositions = true;

    if (locationInfo.locationChoiceType == AGENT_LOCATION_CHOICE_NEAREST) {

        Vertex basePosition;

        if ((locationInfo.nearestChoiceBaseLocationName.empty()) ||
            (locationInfo.nearestChoiceBaseLocationName == "agentlocation")) {

            basePosition = resource.Position();
        }
        else {

            LocationInfo baseLocationInfo;
            GisPositionIdType baesPositionId;
            bool notUsedIsBaseMultiplePositions;

            (*this).GetLocationId(
                theParameterDatabaseReader,
                theAgentGisPtr,
                LocationInfo(false/*isId*/, AGENT_LOCATION_CHOICE_RANDOM, locationInfo.nearestChoiceBaseLocationName),
                shared_ptr<InorderFileCache>(),
                false/*ignoreLastPositionFromCandidate*/,
                false/*searchIntersection*/,
                baesPositionId,
                notUsedIsBaseMultiplePositions,
                isSpecifiedPosition,
                position,
                resource,
                randGen);
            
            if (isSpecifiedPosition) {
                cerr << "Location specification is available only for \"PresentLication\"." << endl;
                exit(1);
            }//if//

            if (baesPositionId.IsValid()) {
                basePosition = subsystem.GetPositionVertex(baesPositionId);
            }
            else {
                basePosition = resource.Position();
            }//if//

        }//if/

        double minDistance = DBL_MAX;

        for(size_t i = 0; i < locationCandidateIds.size(); i++) {

            const Vertex gisObjectPosition = subsystem.GetPositionVertex(locationCandidateIds[i]);

            double distance = basePosition.DistanceTo(gisObjectPosition);

            if (distance < minDistance) {
                minDistance = distance;
                positionId = locationCandidateIds[i];
            }
        }

    } else if (locationInfo.locationChoiceType == AGENT_LOCATION_CHOICE_RANDOM) {

        positionId = locationCandidateIds[
            randGen.GenerateRandomInt(
                0, (int32_t)(locationCandidateIds.size() - 1))];
    }
}

TimeType AgentTask::GetEndTime(const AgentResource& resource) const
{
    if (endTime.IsNull()) {
        return INFINITE_TIME;
    }

    return endTime.CalculateTime(resource, resource.GetRandomNumberGenerator());
}

TimeType AgentTask::GetWaitTime(const AgentResource& resource) const
{
    if (waitTime.IsNull()) {
        return ZERO_TIME;
    }

    return waitTime.CalculateTime(resource, resource.GetRandomNumberGenerator());
}

void AgentTask::GetTimeLine(
    const AgentResource& resource,
    const TimeType& earlyStartTime,
    TimeToSearchRoute& timeToSearchRoute) const
{
    timeToSearchRoute.specifiedArrivalTime = !arrivalTime.IsNull();
    timeToSearchRoute.specifiedDepartureTime = !departureTime.IsNull();

    timeToSearchRoute.departureTime = earlyStartTime;
    timeToSearchRoute.earlyDepartureTime = earlyStartTime;
    timeToSearchRoute.lateDepartureTime = INFINITE_TIME;

    if (!timeToSearchRoute.specifiedArrivalTime &&
        !timeToSearchRoute.specifiedDepartureTime) {

        timeToSearchRoute.earlyArrivalTime = ZERO_TIME;
        timeToSearchRoute.arrivalTime = INFINITE_TIME;
        timeToSearchRoute.lateArrivalTime = INFINITE_TIME;
        return;
    }

    HighQualityRandomNumberGenerator& randGen = resource.GetRandomNumberGenerator();

    if (!arrivalTime.IsNull()) {
        timeToSearchRoute.arrivalTime =
            std::max(earlyStartTime, arrivalTime.CalculateTime(resource, randGen));
    }
    if (!earlyArrivalTime.IsNull()) {
        timeToSearchRoute.earlyArrivalTime =
            std::max(earlyStartTime, earlyArrivalTime.CalculateTime(resource, randGen));
    }
    if (!lateArrivalTime.IsNull()) {
        timeToSearchRoute.lateArrivalTime =
            std::max(earlyStartTime, lateArrivalTime.CalculateTime(resource, randGen));
    }
    if (!departureTime.IsNull()) {
        timeToSearchRoute.departureTime =
            std::max(earlyStartTime, departureTime.CalculateTime(resource, randGen));
    }
    if (!earlyDepartureTime.IsNull()) {
        timeToSearchRoute.earlyDepartureTime =
            std::max(earlyStartTime, earlyDepartureTime.CalculateTime(resource, randGen));
    }
    if (!lateDepartureTime.IsNull()) {
        timeToSearchRoute.lateDepartureTime =
            std::max(earlyStartTime, lateDepartureTime.CalculateTime(resource, randGen));
    }

    timeToSearchRoute.earlyArrivalTime =
        std::min(timeToSearchRoute.earlyArrivalTime, timeToSearchRoute.arrivalTime);

    timeToSearchRoute.lateArrivalTime =
        std::max(timeToSearchRoute.lateArrivalTime, timeToSearchRoute.arrivalTime);

    timeToSearchRoute.earlyDepartureTime =
        std::min(timeToSearchRoute.earlyDepartureTime, timeToSearchRoute.departureTime);

    timeToSearchRoute.lateDepartureTime =
        std::max(timeToSearchRoute.lateDepartureTime, timeToSearchRoute.departureTime);
}

void AgentTaskTable::GetStatusChanges(
    const AgentResource& resource,
    priority_queue_stable<AgentStatusChangeEvent>& timeLineStatusChangeEvents) const
{
    while (!timeLineStatusChangeEvents.empty()) {
        timeLineStatusChangeEvents.pop();
    }

    for(size_t i = 0; i < statusChangePtrs.size(); i++) {
        const AgentTask& statusChangeTask = *(statusChangePtrs[i]);
        const TimeType changeTime = statusChangeTask.GetStartTime(resource);

        timeLineStatusChangeEvents.push(
            AgentStatusChangeEvent(
                changeTime,
                i,
                AGENT_STATUS_CHANGE_AT_SPECIFIC_TIME));
    }

    for(size_t i = 0; i < interruptTaskPtrs.size(); i++) {
        const AgentTask& statusChangeTask = *(interruptTaskPtrs[i]);
        const TimeType changeTime = statusChangeTask.GetStartTime(resource);

        timeLineStatusChangeEvents.push(
            AgentStatusChangeEvent(
                changeTime,
                i,
                AGENT_STATUS_CHANGE_TASK_INTERRUPTION_START));
    }
}

//----------------------------------------------------------------------------------------

AgentConditionChecker::AgentConditionChecker(
    const LabelMap<AgentStatusIdType, AgentValueFormula>& initParameters,
    const ConditionParameterType& initParameterType,
    const double initSimStartTimeSec,
    const string& initString,
    map<string, AgentCharactorIdType>& initCharactorIds)
    :
    parameterType(initParameterType)
{
    assert(initString.size() > 2);

    const string checkString = initString.substr(0, 2);
    size_t checkStringLength = 2;

    if (checkString == "==") {
        Checker = &AgentConditionChecker::Check1;
    } else if (checkString == "!=") {
        Checker = &AgentConditionChecker::Check2;
    } else if (checkString == "<=") {
        Checker = &AgentConditionChecker::Check4;
    } else if (checkString == ">=") {
        Checker = &AgentConditionChecker::Check6;
    } else if (checkString[0] == '<') {
        Checker = &AgentConditionChecker::Check3;
        checkStringLength = 1;
    } else if (checkString[0] == '>') {
        Checker = &AgentConditionChecker::Check5;
        checkStringLength = 1;
    } else {
        Checker = &AgentConditionChecker::Check1;
        checkStringLength = 0;
    }

    formula = AgentValueFormula(
        initParameters,
        initSimStartTimeSec,
        initString.substr(checkStringLength),
        initCharactorIds);
}

bool AgentConditionChecker::Check(const AgentResource& resource) const
{
    if (parameterType == CONDITION_PARAMETER_TIME) {
        return (this->*(Checker))(double(resource.CurrentTime()/SECOND), formula.CalculateDouble(resource, resource.GetRandomNumberGenerator()));
    } else {
        return (this->*(Checker))(resource.Value(ConvertToStatusId(parameterType)), formula.CalculateDouble(resource, resource.GetRandomNumberGenerator()));
    }
}

AgentProfile::AgentProfile(
    const AgentProfileType& initProfileType,
    const string& initProfileName)
    :
    profileName(initProfileName),
    profileType(initProfileType),
    userType(AGENT_USER_TYPE_NONE),
    mobilityClass(AGENT_MOBILTY_CLASS_NORMAL),
    ticketType(AGENT_TICKET_FULL_FARE),
    routeCostFormulas(NUMBER_AGENT_BEHAVIORS)
{
    for(AgentStatusIdType statusId = 0; statusId < NUMBER_AGENT_STATUS_VALUES; statusId++) {

        double defaultValue = 0;

        if (AGENT_RESERVED_STATUS_QUERY_TRIGGER_START <= statusId &&
            statusId <= AGENT_RESERVED_STATUS_QUERY_TRIGGER_END) {
            defaultValue = DBL_MAX;
        }
        parameters[RESERVED_AGENT_STATUS_NAMES[statusId]] = AgentValueFormula(defaultValue);
    }

    parameters[AGENT_RESERVED_STATUS_SEEING_PEDESTRIAN_PROBABILITY] = DEFAULT_SEEING_PEDESTRIAN_PROBABILITY;
    parameters[AGENT_RESERVED_STATUS_MAX_VEHICLE_SPEED] = DEFAULT_MAX_VEHICLE_SPEED;
    parameters[AGENT_RESERVED_LANE_CHANGE_ACCELERATION_THRESHOLD] = DEFAULT_LANE_CHANGE_ACCELERATION_THRESHOLD;
    parameters[AGENT_RESERVED_STATUS_TIME_HEADWAY] = DEFAULT_TIME_HEADWAY;
    parameters[AGENT_RESERVED_STATUS_MIN_VEHICLE_GAP] = DEFAULT_MIN_VEHICLE_GAP;
    parameters[AGENT_RESERVED_STATUS_MAX_ACCELERATION] = DEFAULT_MAX_ACCELERATION;
    parameters[AGENT_RESERVED_STATUS_MAX_DECELERATION] = DEFAULT_MAX_DECELERATION;
    parameters[AGENT_RESERVED_VELOCITY_RATIO_GAP_DISTANCE] = DEFAULT_VELOCITY_RATIO_GAP_DISTANCE;
    parameters[AGENT_RESERVED_OTHER_VEHICLE_ENATRANCE_TIME] = DEFAULT_OTHER_VEHICLE_ENATRANCE_TIME;
    parameters[AGENT_RESERVED_PASSIVE_YIELD_TIME] = DEFAULT_PASSIVE_YIELD_TIME;
    parameters[AGENT_RESERVED_ACTIVE_YIELD_TIME] = DEFAULT_ACTIVE_YIELD_TIME;
    parameters[AGENT_RESERVED_YIELD_WAITING_TIME] = DEFAULT_YIELD_WAITING_TIME;
    parameters[AGENT_RESERVED_ACCEPTABLE_WALK_DISTANCE_TO_CAR] = DEFAULT_ACCEPTABLE_WALK_DISTANCE_TO_VEHICLE;
    parameters[AGENT_RESERVED_ACCEPTABLE_WALK_DISTANCE_TO_STOP] = DEFAULT_ACCEPTABLE_WALK_DISTANCE_TO_STOP;
    parameters[AGENT_RESERVED_MIN_VEHICLE_ROUTE_DISTANCE] = DEFAULT_MIN_VEHICLE_ROUTE_DISTANCE;
    parameters[AGENT_RESERVED_NUMBER_MAX_ROUTE_CANDIDATES] = DEFAULT_NUMBER_MAX_ROUTE_CANDIDATES;
    parameters[AGENT_RESERVED_NUMBER_PEOPLE] = DEFAULT_NUMBER_PEOPLE;
    parameters[AGENT_RESERVED_ENTRANCE_WAIT_TIME] = DEFAULT_ENTRANCE_WAIT_TIME;
    parameters[AGENT_RESERVED_VEHICLE_ENTRANCE_WAIT_TIME] = DEFAULT_VEHICLE_ENTRANCE_WAIT_TIME;
    parameters[AGENT_RESERVED_TAXICALL_WAIT_TIME] = DEFAULT_TAXICALL_WAIT_TIME;
    parameters[AGENT_RESERVED_STATUS_MAX_BRAKING_DECCELERATION] = DEFAULT_MAX_BRAKING_DECCELERATION;
    parameters[AGENT_RESERVED_STATUS_ACCELERATION_EXPONENT] = DEFAULT_ACCELERATION_EXPONENT;
    parameters[AGENT_RESERVED_STATUS_SAVE_DECELERATION] = DEFAULT_SAVE_DECELERATION;
    parameters[AGENT_RESERVED_STATUS_MAX_TURN_SPEED] = DEFAULT_MAX_TURN_SPEED;
}

static inline
void MakeSetOfAllAgentIds(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    set<AgentIdType>& agentIds)
{
    set<AgentIdType> profileAgentIds;
    set<AgentIdType> behaviorAgentIds;

    theParameterDatabaseReader.MakeSetOfAllNodeIdsWithParameter("multiagent-profile-type", profileAgentIds);
    theParameterDatabaseReader.MakeSetOfAllNodeIdsWithParameter("multiagent-behavior-type", behaviorAgentIds);

    agentIds.insert(profileAgentIds.begin(), profileAgentIds.end());
    agentIds.insert(behaviorAgentIds.begin(), behaviorAgentIds.end());
}

//--------------------------------------------------------------------

const string AGENT_TYPE_TAXI("taxi");
const string AGENT_TYPE_BUS("bus");

const string MultiAgentSimulator::modelName = "Mas";

#pragma warning(disable:4355)

MultiAgentSimulator::MultiAgentSimulator(
    const shared_ptr<ParameterDatabaseReader>& initParameterDatabaseReaderPtr,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const RandomNumberGeneratorSeedType& initRunSeed,
    const bool initRunSequentially,
    const bool initIsScenarioSettingOutputMode,
    const string& initInputConfigFileName,
    const string& initOutputConfigFileName)
    :
    NetworkSimulator(
        initParameterDatabaseReaderPtr,
        initSimulationEnginePtr,
        initRunSeed,
        initRunSequentially),
    currentSnapshotId(0),
    currentTime(ZERO_TIME),
    timeStep(
        theParameterDatabaseReaderPtr->ReadTime(
            "time-step-event-synchronization-step")),
    isSimulationDone(false),
    isScenarioSettingOutputMode(initIsScenarioSettingOutputMode),
    inputConfigFileName(initInputConfigFileName),
    outputConfigFileName(initOutputConfigFileName),
    profileValueOutputSubsystem(*theParameterDatabaseReaderPtr),
    reservedVehicleNodePtrs(NUMBER_VEHICLE_TYPES),
    masterAnyAgentPtr(
        Agent::CreateMasterAgent(
            this,
            MASTER_ANY_AGENT_ID,
            shared_ptr<AgentProfile>(new AgentProfile(INVALID_AGENT_TYPE)),
            shared_ptr<AgentTaskTable>(new AgentTaskTable(nullptr)))),
    theSimulationRunInterfacePtr(
        theSimulationEnginePtr->GetSimulationEngineInterface(
            *initParameterDatabaseReaderPtr, ScenSim::ANY_NODEID)),
    theAgentGisPtr(
        new MultiAgentGis(
            this,
            *theParameterDatabaseReaderPtr,
            initSimulationEnginePtr,
            theGisSubsystemPtr)),
    initialLocationFileCachePtr(new InorderFileCache())
{
    const ParameterDatabaseReader& theParameterDatabaseReader = *initParameterDatabaseReaderPtr;

    publicVehicleTablePtr.reset(
        new PublicVehicleTable(
            this,
            theParameterDatabaseReader,
            theAgentGisPtr));

    theRouteSearchSubsystemPtr.reset(
        new AgentRouteSearchSubsystem(
            theAgentGisPtr,
            publicVehicleTablePtr));

    profileAndTaskTablePtr.reset(
        new AgentProfileAndTaskTable(
            *theParameterDatabaseReaderPtr,
            publicVehicleTablePtr,
            theGisSubsystemPtr,
            masterAnyAgentPtr));

    masterBusAgentPtr =
        Agent::CreateMasterAgent(
            this,
            MASTER_BUS_AGENT_ID,
            profileAndTaskTablePtr->GetProfile(AGENT_TYPE_BUS),
            profileAndTaskTablePtr->GetEmptyTaskTable());

    masterTaxiAgentPtr =
        Agent::CreateMasterAgent(
            this,
            MASTER_TAXI_AGENT_ID,
            profileAndTaskTablePtr->GetProfile(AGENT_TYPE_TAXI),
            profileAndTaskTablePtr->GetEmptyTaskTable());

    publicVehicleTablePtr->CompleteRouteAndVehiclewScheduleInitialization();

    if (!theParameterDatabaseReader.ReadBool("gis-road-set-intersection-margin")) {
        cerr << "Error in MultiAgent Extension Module: Set gis-road-set-intersection-margin = true" << endl;
        exit(1);
    }

    MakeSetOfAllAgentIds(*theParameterDatabaseReaderPtr, entireAgentIds);

    profileAndTaskTablePtr->CompleteInitialize(
        *theParameterDatabaseReaderPtr,
        theGisSubsystemPtr,
        masterAnyAgentPtr,
        entireAgentIds);

    if (timeStep < MULTIAGENT_MIN_TIME_STEP) {
        cerr << "Error:  Multi-Agent min time step is 1 ms" << endl;
        exit(1);
    }

    set<AgentIdType> agentIds;
    vector<shared_ptr<BusTicket> > busReservationPtrs;
    vector<shared_ptr<Agent> > allHumanAgentPtrs;

    MakeSetOfAllAgentIds(*theParameterDatabaseReaderPtr, agentIds);

    typedef set<NodeIdType>::const_iterator IterType;

    map<AgentIdType, Vertex> taxiAgentLocations;

    for(IterType iter = agentIds.begin(); iter != agentIds.end(); iter++) {
        const AgentIdType& agentId = (*iter);

        string profileName = MakeLowerCaseString(
            theParameterDatabaseReaderPtr->ReadString(
                "multiagent-profile-type", agentId));

        string taskName = MakeLowerCaseString(
            theParameterDatabaseReaderPtr->ReadString(
                "multiagent-behavior-type", agentId));

        if (AStringStartsWith(profileName, "taxi")) {

            const double distanceGranularityMeters = 0;
            const string positionFileName =
                theParameterDatabaseReaderPtr->ReadString(
                    "mobility-init-positions-file",
                    agentId);

            InorderFileCache mobilityFileCacheForTaxi;

            TraceFileMobilityModel traceMobilityForTaxi(
                *theParameterDatabaseReaderPtr,
                agentId,
                nullInstanceId,
                mobilityFileCacheForTaxi,
                positionFileName,
                agentId,
                distanceGranularityMeters,
                theGisSubsystemPtr);

            ObjectMobilityPosition taxiMobilityPosition;

            traceMobilityForTaxi.GetUnadjustedPositionForTime(ZERO_TIME, taxiMobilityPosition);

            Vertex& taxiPos = taxiAgentLocations[agentId];

            taxiPos.x = taxiMobilityPosition.X_PositionMeters();
            taxiPos.y = taxiMobilityPosition.Y_PositionMeters();
            
        } else if (profileName == "train" ||
                   profileName == "bus" ||
                   profileName == "vehicle" ||
                   profileName == "privatecar") {

            VehicleType vehicleType = VEHICLE_CAR;

            if (profileName == "train") {
                vehicleType = VEHICLE_TRAIN;
            } else if (profileName == "bus") {
                vehicleType = VEHICLE_BUS;
            } else if (profileName == "vehicle" || profileName == "privatecar") {
                vehicleType = VEHICLE_CAR;
            }

            (*this).ReserveVehicleNode(agentId, vehicleType);

        } else {

            string origProfileName = profileName;
            size_t idExchangePos = profileName.find("$n");

            if (idExchangePos != string::npos) {
                profileName =
                    profileName.substr(0, idExchangePos) +
                    ConvertToString(agentId) +
                    profileName.substr(idExchangePos + 2);
            }
            else if (profileName == "{simulation_node_id}") {

                profileName = ConvertToString(agentId);
            }//if//

            const shared_ptr<Agent> agentPtr(
                new Agent(
                    this,
                    theGlobalNetworkingObjectBag,
                    agentId,
                    (*this).GetSimEngineInterfacePtr(agentId),
                    profileAndTaskTablePtr->GetProfile(profileName),
                    profileAndTaskTablePtr->GetTaskTable(taskName, origProfileName),
                    theAgentGisPtr,
                    publicVehicleTablePtr,
                    theRouteSearchSubsystemPtr));

            if (agentPtr->HasCar()) {
                newlyAddedVehiclePtrs.push_back(agentPtr->GetVehicle());
            }

            const AgentResource resource(agentPtr);
            const double lastDelayQueryTrigger = resource.LastDelayQueryTrigger();
            const double nextDelayQueryTrigger = resource.NextDelayQueryTrigger();
            const double tripDelayQueryTrigger = resource.TripDelayQueryTrigger();
            const double vehicleDelayQueryTrigger = resource.VehicleDelayQueryTrigger();
            const double congestionQueryTrigger = resource.CongestionQueryTrigger();
            const double utility1QueryTrigger = resource.Utility1QueryTrigger();
            const double utility2QueryTrigger = resource.Utility2QueryTrigger();

            if (lastDelayQueryTrigger <= 0) {
                cerr << "Error: Set RecalcIntervalForLastViaPointDelay to non-zero(> 0) value in AgentProfile" << endl;
                exit(1);
            }
            if (nextDelayQueryTrigger <= 0) {
                cerr << "Error: Set RecalcIntervalForNextViaPointDelay to non-zero(> 0) value in AgentProfile" << endl;
                exit(1);
            }
            if (tripDelayQueryTrigger <= 0) {
                cerr << "Error: Set RecalcIntervalForDestinationDelay to non-zero(> 0) value in AgentProfile" << endl;
                exit(1);
            }
            if (vehicleDelayQueryTrigger <= 0) {
                cerr << "Error: Set RecalcIntervalForVehicleDelay to non-zero(> 0) value in AgentProfile" << endl;
                exit(1);
            }
            if (congestionQueryTrigger <= 0) {
                cerr << "Error: Set RecalcThresholdForCongestion to non-zero(> 0) value in AgentProfile" << endl;
                exit(1);
            }
            if (utility1QueryTrigger <= 0) {
                cerr << "Error: Set RecalcThresholdForUtility1 to non-zero(> 0) value in AgentProfile" << endl;
                exit(1);
            }
            if (utility2QueryTrigger <= 0) {
                cerr << "Error: Set RecalcThresholdForUtility2 to non-zero(> 0) value in AgentProfile" << endl;
                exit(1);
            }

            const TimeType wakeupTime = agentPtr->CalculateWakeupTime();

            agentWakeupQueue.push(AgentWakeupEntry(wakeupTime, agentPtr));

            wakeupTimes[agentId] = wakeupTime;

            allHumanAgentPtrs.push_back(agentPtr);
        }
    }
    
    //theGisSubsystemPtr->AddGisChangeEventHandler(
    //    modelName,
    //    shared_ptr<MultiAgentGisChangeEventHandler>( new MultiAgentGisChangeEventHandler(this)));

    publicVehicleTablePtr->CompleteAllPublicVehicleInitialization();

    theAgentGisPtr->CompleteInitialization(
        *theParameterDatabaseReaderPtr,
        initSimulationEnginePtr,
        *profileAndTaskTablePtr,
        taxiAgentLocations,
        publicVehicleTablePtr);

    // synchronizer -----------------------------------------

    // Application constructor must not output trace because of trace output confliction between threads.
    // If an application output trace in constructor, call SyncApplicationCreation() without ApplicationCreationSynchronizer

    (*this).SetupStatOutputFile();


    if (isScenarioSettingOutputMode) {

        (*this).OutputScenarioSettingToConfig(
            *theParameterDatabaseReaderPtr,
            agentIds,
            allHumanAgentPtrs,
            newlyAddedVehiclePtrs);
    }//if//

}//MultiAgentSimulator()//

#pragma warning(default:4355)

MultiAgentSimulator::~MultiAgentSimulator()
{
    isSimulationDone = true;
    theAgentGisPtr->DisconnectGisConnection();

    (*this).DeleteAllNodes();

    reservedVehicleNodePtrs.clear();
    theAgentGisPtr.reset();
    masterAnyAgentPtr.reset();
    publicVehicleTablePtr.reset();

    masterBusAgentPtr.reset();
    masterTaxiAgentPtr.reset();

    theRouteSearchSubsystemPtr.reset();
    theSimulationRunInterfacePtr.reset();

    entireAgentIds.clear();

    while(!agentWakeupQueue.empty()) {
        agentWakeupQueue.pop();
    }

    newlyAddedAgentIds.clear();

    communicationNodePtrsWaitingAgentCreation.clear();
    synchronizedNodePtrs.clear();

    timeIncrementThreadPtr.reset();

    while(!deleteNodeIds.empty()) {
        deleteNodeIds.pop();
    }

    typedef list<shared_ptr<Agent> >::iterator AgentIter;

    for(AgentIter iter = agentPtrs.begin();
        (iter != agentPtrs.end()); iter++) {
        (*iter)->ReadyToDestruct();
    }//for//
    agentPtrs.clear();

    while (!tookSynchronizerPtrs.empty()) {
        tookSynchronizerPtrs.pop();
    }

    iterToAgentList.clear();
    newlyAddedVehiclePtrs.clear();
    ownerChangeEvents.clear();
}

struct ReplaceLineInfo {
    string value;
    string scopeString;
    
    ReplaceLineInfo() {}

    explicit ReplaceLineInfo(
        const string& initValue,
        const string& initScopeString = string()/*=> global*/)
        :
        value(initValue),
        scopeString(initScopeString)
    {}
};

static inline
void OutputNewConfigFileWithReplaceLine(
    const string& inputConfigFileName,
    const string& outputConfigFileName,
    const map<string, ReplaceLineInfo>& replaceLineMap)
{
    // Remove corresponding lines (simple replacement)

    ifstream inStream(inputConfigFileName.c_str());

    if (!inStream.good()) {
        cerr << "Error: Couldn't open input config file " << inputConfigFileName << endl;
        exit(1);
    }//if//

    string tmpOutputFileName = outputConfigFileName + ".tmp";

    ofstream outStream(tmpOutputFileName.c_str(), std::ios::out);

    if (!outStream.good()) {
        cerr << "Error: Couldn't open output config file " << tmpOutputFileName << endl;
        exit(1);
    }//if//

    while(!inStream.eof()) {
        string aLine;
        getline(inStream, aLine);

        DeleteTrailingSpaces(aLine);

        if (!IsAConfigFileCommentLine(aLine)) {
            size_t parameterNameStartPos = aLine.find_first_not_of(' ');
            
            // No checking the file format.
            // (already checked by "ParameterDatabaseReader")

            if (aLine.at(parameterNameStartPos) == '[') {
                parameterNameStartPos = aLine.find(']', parameterNameStartPos) + 1;
                parameterNameStartPos = aLine.find_first_not_of(' ', parameterNameStartPos);
            }//if//

            const size_t parameterNameEndPos =
                aLine.find_first_of(" ", parameterNameStartPos);
            
            string parameterNAme =
                aLine.substr(parameterNameStartPos, parameterNameEndPos - parameterNameStartPos);
            
            ConvertStringToLowerCase(parameterNAme);

            if (replaceLineMap.find(parameterNAme) != replaceLineMap.end()) {
                continue;
            }//if//
        }//if//

        outStream << aLine << endl;
    }//while//

    outStream << endl;
    outStream << "#Simulated Scenario Setting" << endl;

    // Additional line

    typedef map<string, ReplaceLineInfo>::const_iterator IterType;

    for(IterType iter = replaceLineMap.begin();
        (iter != replaceLineMap.end()); iter++) {

        const ReplaceLineInfo& lineInfo = (*iter).second;

        if (!lineInfo.scopeString.empty()) {
            outStream << "[" << lineInfo.scopeString << "] ";
        }//if//

        outStream << (*iter).first << " = " << lineInfo.value << endl;
    }//for//

    inStream.close();
    outStream.close();

    boost::system::error_code errorCode;
    
    boost::filesystem::rename(tmpOutputFileName, outputConfigFileName, errorCode);

    if (errorCode != boost::system::errc::success) {
        cerr << "Error: Failed to output " << outputConfigFileName << endl;
        exit(1);
    }//if//
    
}//OutputNewConfigFileWithReplaceLine//    
    
static inline
string ConvertToScopeString(const set<AgentIdType>& agentIds)
{
    typedef set<AgentIdType>::const_iterator IterType;

    const AgentIdType invalidAgentId = static_cast<AgentIdType>(-1);

    AgentIdType scopeStartAgentId = invalidAgentId;
    AgentIdType scopeEndAgentId = invalidAgentId;
    
    vector<pair<AgentIdType, AgentIdType> > agentIdPairs;

    for(IterType iter = agentIds.begin();
        (iter != agentIds.end()); iter++) {
        
        const AgentIdType& agentId = (*iter);
        
        if (agentId == (scopeEndAgentId + 1)) {

            scopeEndAgentId = agentId;
        }
        else {

            if (scopeStartAgentId != invalidAgentId) {
                agentIdPairs.push_back(make_pair(scopeStartAgentId, scopeEndAgentId));
            }//if//

            scopeStartAgentId = agentId;
            scopeEndAgentId = agentId;

        }//if//
    }//for//

    if (scopeStartAgentId != invalidAgentId) {
        agentIdPairs.push_back(make_pair(scopeStartAgentId, scopeEndAgentId));
    }//if//
    
    ostringstream outStream;
    
    for(size_t i = 0; i < agentIdPairs.size(); i++) {
        const pair<AgentIdType, AgentIdType>& agentIdPair = agentIdPairs[i];

        if (i > 0) {
            outStream << ", ";
        }//if//

        if (agentIdPair.first == agentIdPair.second) {
            outStream << agentIdPair.first;
        }
        else {
            outStream << agentIdPair.first << "-" << agentIdPair.second;
        }//if//
    }//for//

    return outStream.str();

}//ConvertToScopeString//

void MultiAgentSimulator::OutputScenarioSettingToConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const set<AgentIdType>& agentIds,
    const vector<shared_ptr<Agent> >& allHumanAgentPtrs,
    const list<shared_ptr<Vehicle> >& allPrivateCarPtrs)
{
    // Output current scenario settings
 
    const string scenarioName = boost::filesystem::basename(outputConfigFileName);
   
    const string initPosFileName = scenarioName + ".pos";

    (*this).OutputAllHumanAgentInitialLocationToPosFile(
        theParameterDatabaseReader,
        agentIds,
        allHumanAgentPtrs,
        allPrivateCarPtrs,
        initPosFileName);
    
    const string scenarioPrefix = scenarioName + "_";
    const string behaviorFileName = theParameterDatabaseReader.ReadString("multiagent-behavior-file");

    const boost::filesystem::path origConfigPath(inputConfigFileName);
    const boost::filesystem::path destConfigPath(outputConfigFileName);

    string destShapeDirName;
    string destBehaviorFileName;

    if (origConfigPath == destConfigPath) {
        destShapeDirName = theParameterDatabaseReader.ReadString("gis-object-file-path");
        destBehaviorFileName = behaviorFileName;
    }
    else {
        destShapeDirName = scenarioPrefix + "shapes/";
        destBehaviorFileName = scenarioPrefix + "AgentBehaviors.txt";
    }//if//

    vector<string> outputShapeFileNames;
    
    (*this).OverwriteGisShapeFile(
        theParameterDatabaseReader,
        destShapeDirName,
        outputShapeFileNames);
    
    (*this).OverwriteBehaviorFileWithPresentInitialLocation(
        behaviorFileName,
        destBehaviorFileName);
    
    // Output config
    
    map<string, ReplaceLineInfo> replaceLineMap;
    
    const string scopeString = ConvertToScopeString(agentIds);

    replaceLineMap["mobility-init-positions-file"] = ReplaceLineInfo(initPosFileName, scopeString);
    replaceLineMap["gis-object-file-path"] = ReplaceLineInfo(destShapeDirName);
    replaceLineMap["multiagent-behavior-file"] = ReplaceLineInfo(destBehaviorFileName);
    
    ostringstream shapeLineStream;
    
    for(size_t i = 0; i < outputShapeFileNames.size(); i++) {
        shapeLineStream << " " << outputShapeFileNames[i];
    }//for//
    
    replaceLineMap["gis-object-files"] = ReplaceLineInfo(shapeLineStream.str());
    
    OutputNewConfigFileWithReplaceLine(
        inputConfigFileName,
        outputConfigFileName,
        replaceLineMap);
    
}//OutputScenarioSettingToConfig//

void MultiAgentSimulator::OutputAllHumanAgentInitialLocationToPosFile(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const set<AgentIdType>& agentIds,
    const vector<shared_ptr<Agent> >& allHumanAgentPtrs,
    const list<shared_ptr<Vehicle> >& allPrivateCarPtrs,
    const string& outputFileName)
{
    typedef set<AgentIdType>::const_iterator AgentIdIter;

    map<AgentIdType, ObjectMobilityPosition> mobilityPositions;

    InorderFileCache mobilityFileCacheForTaxi;
    
    for(AgentIdIter iter = agentIds.begin(); iter != agentIds.end(); iter++) {
        const AgentIdType& agentId = (*iter);
        
        const string positionFileName =
            theParameterDatabaseReaderPtr->ReadString(
                "mobility-init-positions-file", agentId);
        
        TraceFileMobilityModel traceMobilityForTaxi(
            *theParameterDatabaseReaderPtr,
            agentId,
            nullInstanceId,
            mobilityFileCacheForTaxi,
            positionFileName,
            agentId,
            0/*distanceGranularityMeters*/,
            theGisSubsystemPtr);
        
        ObjectMobilityPosition mobilityPosition;

        traceMobilityForTaxi.GetUnadjustedPositionForTime(ZERO_TIME, mobilityPosition);
        
        mobilityPositions[agentId] = mobilityPosition;
    }//for//
    
    // Overwrite initial position for human agent

    for(size_t i = 0; i < allHumanAgentPtrs.size(); i++) {
        const shared_ptr<Agent>& humanAgentPtr = allHumanAgentPtrs[i];
        const AgentResource resource(humanAgentPtr);

        mobilityPositions[humanAgentPtr->GetAgentId()] =
            resource.MobilityPosition();
    }//for//

    // Assign vehicle(private car) with the same algorithm as the simulation

    queue<shared_ptr<VehicleNode> > vehicleNodePtrs =
        reservedVehicleNodePtrs.at(VEHICLE_CAR); //deep copy

    typedef list<shared_ptr<Vehicle> >::const_iterator VehicleIter;

    for(VehicleIter iter = allPrivateCarPtrs.begin(); iter != allPrivateCarPtrs.end(); iter++) {
        const shared_ptr<Vehicle>& vehiclePtr = (*iter);

        if (vehicleNodePtrs.empty()) {
            cerr << "Error: not enough vehicle node" << endl;
            exit(1);
        }//if//

        shared_ptr<VehicleNode> vehicleNodePtr = vehicleNodePtrs.front();

        vehicleNodePtr->SetVehicle(
            theParameterDatabaseReader,
            theAgentGisPtr,
            (*this).CurrentTime(),
            vehiclePtr);

        mobilityPositions[vehicleNodePtr->GetAgentId()] =
            vehicleNodePtr->GetCurrentLocation();
        
        vehicleNodePtrs.pop();
    }//for//


    ofstream outStream(outputFileName.c_str(), std::ios::out);

    if (!outStream.good()) {
        cerr << "Error: Couldn't open position file " << outputFileName << endl;
        exit(1);
    }//if//
    
    typedef map<AgentIdType, ObjectMobilityPosition>::const_iterator PositionIter;
    
    outStream.precision(30);

    for(PositionIter iter = mobilityPositions.begin();
        (iter != mobilityPositions.end()); iter++) {
        
        using std::setw;

        const AgentIdType agentId = (*iter).first;
        const ObjectMobilityPosition& mobilityPosition = (*iter).second;

        outStream
            << agentId << ' '
            << "0 "
            << setw(11) << mobilityPosition.X_PositionMeters() << ' '
            << setw(11) << mobilityPosition.Y_PositionMeters() << ' '
            << mobilityPosition.HeightFromGroundMeters() << ' '
            << mobilityPosition.AttitudeAzimuthFromNorthClockwiseDegrees() << ' '
            << mobilityPosition.AttitudeElevationFromHorizonDegrees() << endl;
        
    }//for//    
    
}//OutputAllHumanAgentInitialLocationToPosFile//
    
void MultiAgentSimulator::OverwriteGisShapeFile(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& destShapeDirName,
    vector<string>& outputShapeFileNames)
{
    const string origShapeDirName =
        theParameterDatabaseReader.ReadString("gis-object-file-path");

    const boost::filesystem::path origShapeDirPath(origShapeDirName);
    const boost::filesystem::path destShapeDirPath(destShapeDirName);

    if (origShapeDirPath == destShapeDirPath) {

        //overwrite original shape
    }
    else {
        using boost::filesystem::directory_iterator;

        boost::system::error_code errorCode;

        boost::filesystem::create_directories(destShapeDirPath, errorCode);

        if (errorCode != boost::system::errc::success) {
            cerr << "Error: Failed to create directory " << destShapeDirName << endl;
            exit(1);
        }//if//

        directory_iterator endIter;

        for(directory_iterator dirIter(origShapeDirName);
            (dirIter != endIter); dirIter++) {

            const boost::filesystem::path& filePath = *dirIter;

            if (!boost::filesystem::is_directory(filePath)) {
                boost::filesystem::copy_file(filePath, destShapeDirPath / filePath.filename(), errorCode);

                if (errorCode != boost::system::errc::success) {
                    cerr << "Error: Failed coping " << filePath << endl;
                    exit(1);
                }//if//
            }//if//
        }//for//
    }//if//

    const string origShapeFileNames =
        theParameterDatabaseReader.ReadString("gis-object-files");

    bool success;
    vector<string> aVector;

    ConvertAStringIntoAVectorOfStrings(origShapeFileNames, success, aVector);

    if (!success) {
        cerr << "Error: Failed to convert to string vector " << origShapeFileNames << endl;
        exit(1);
    }//if//

    set<string> shapeFileNameSet;

    for(size_t i = 0; i < aVector.size(); i++) {
        shapeFileNameSet.insert(aVector[i]);
    }//for//

    // Overwrite entrance and road shape file.

    shapeFileNameSet.insert("entrance.shp");
    theAgentGisPtr->GetSubsystem().OvtputCurrentEntranceShapeFile(destShapeDirName);

    //Road import/export feature
    //future// shapeFileNameSet.insert("road.shp");
    //future// shapeFileNameSet.insert("intersection.shp");
    //future// theAgentGisPtr->GetSubsystem().OvtputCurrentRoadShapeFile(destShapeDirName);
    //future// theAgentGisPtr->GetSubsystem().OvtputCurrentIntersectionShapeFile(destShapeDirName);

    outputShapeFileNames.clear();

    typedef set<string>::const_iterator IterType;

    for(IterType iter = shapeFileNameSet.begin();
        (iter != shapeFileNameSet.end()); iter++) {
        outputShapeFileNames.push_back(*iter);
    }//for//

}//OverwriteGisShapeFile//

    
void MultiAgentSimulator::OverwriteBehaviorFileWithPresentInitialLocation(
    const string& origBehaviorFileName,
    const string& destBehaviorFileName)
{
    // Remove corresponding lines (simple replacement)

    ifstream inStream(origBehaviorFileName.c_str());

    if (!inStream.good()) {
        cerr << "Error: Couldn't open input config file " << origBehaviorFileName << endl;
        exit(1);
    }//if//

    string tmpOutputFileName = destBehaviorFileName + ".tmp";

    ofstream outStream(tmpOutputFileName.c_str(), std::ios::out);

    if (!outStream.good()) {
        cerr << "Error: Couldn't open output config file " << tmpOutputFileName << endl;
        exit(1);
    }//if//

    const string initialLocationString = "InitialLocation";

    while(!inStream.eof()) {
        string aLine;
        getline(inStream, aLine);

        DeleteTrailingSpaces(aLine);

        if (!IsAConfigFileCommentLine(aLine)) {

            const size_t initialLocationPos = aLine.find(initialLocationString);
            
            if (initialLocationPos != string::npos) {
                const size_t equalPos = aLine.find_first_not_of(" ", initialLocationPos + initialLocationString.length());
                
                if ((equalPos != string::npos) &&
                    (aLine[equalPos] == '=')) {

                    const size_t locationStartPos = aLine.find_first_not_of("= ", equalPos);
                    
                    if (locationStartPos != string::npos) {
                        const size_t locationEndPos = aLine.find_first_of(", ", locationStartPos);
                    
                        outStream
                            << aLine.substr(0, equalPos)
                            << "= PresentLocation";

                        if (locationEndPos != string::npos) {
                            outStream << aLine.substr(locationEndPos);
                        }//if//

                        outStream << endl;
                        continue;

                    }//if//

                }//if//

            }//if//

        }//if//

        outStream << aLine << endl;
    }//while//

    inStream.close();
    outStream.close();

    boost::system::error_code errorCode;
    
    boost::filesystem::rename(tmpOutputFileName, destBehaviorFileName, errorCode);

    if (errorCode != boost::system::errc::success) {
        cerr << "Error: Failed to output " << destBehaviorFileName << endl;
        exit(1);
    }//if//

}//OverwriteBehaviorFileWithPresentInitialLocation//

MultiAgentSimulator::ProfileValueOutputSubsystem::ProfileValueOutputSubsystem(
    const ParameterDatabaseReader& initParameterDatabaseReader)
{
    if (initParameterDatabaseReader.ParameterExists("multiagent-profile-value-output-file")) {
        const string profileValueOutputFileName =
            initParameterDatabaseReader.ReadString("multiagent-profile-value-output-file");

        const string agentProfileFilePath =
            initParameterDatabaseReader.ReadString("multiagent-profile-file");

        if (agentProfileFilePath == profileValueOutputFileName) {
            cerr << "Error: Set \"multiagent-profile-value-output-file\" value other than \"multiagent-profile-file\" value." << endl;
            exit(1);
        }

        profileValueOutputFile.open(profileValueOutputFileName.c_str());
        profileValueOutputFile.precision(30);

        if (!profileValueOutputFile.is_open()) {
            cerr << "Error: Could not open profile value output file " << profileValueOutputFileName << endl;
            exit(1);
        }
    }
}


void MultiAgentSimulator::RunSimulationUntil(const TimeType& snapshotTime)
{
    while (currentTime <= snapshotTime) {
        theSimulationRunInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new AdvanceTimeStepEvent(this)), currentTime);

        theSimulationEnginePtr->RunSimulationSequentially(currentTime);
    }
}

void MultiAgentSimulator::AdvanceTimeStep()
{
    if (isDebugMode) {
        cout << "SimTime = " << ConvertTimeToDoubleSecs(currentTime) << "[s]" << endl;
    }

    newlyAddedAgentIds.clear();

    (*this).AddVehicle();

    while (!agentWakeupQueue.empty() &&
           agentWakeupQueue.top().time <= currentTime) {

        (*this).AddAgent(agentWakeupQueue.top().agentPtr);

        agentWakeupQueue.pop();
    }

    (*this).ChangeAgentOwnerIfNecessary();

    (*this).IncrementTimeStep();

    currentTime += timeStep;
}

void MultiAgentSimulator::AddVehicle()
{
    vector<shared_ptr<Train> > trainPtrs;
    vector<shared_ptr<Bus> > busPtrs;

    publicVehicleTablePtr->CreatePublicVehicles(currentTime, trainPtrs, busPtrs);

    (*this).AddTrain(trainPtrs);

    (*this).AddBus(busPtrs);

    (*this).AddCar();
}

void MultiAgentSimulator::AddTrain(const vector<shared_ptr<Train> >& trainPtrs)
{
    if (trainPtrs.empty()) {
        return;
    }

    queue<shared_ptr<VehicleNode> >& trainNodePtrs = reservedVehicleNodePtrs.at(VEHICLE_TRAIN);

    for(size_t i = 0; i < trainPtrs.size(); i++) {
        const shared_ptr<Train>& trainPtr = trainPtrs[i];

        if (trainNodePtrs.size() < trainPtrs.size()) {
            cerr << "Error: not enough train node" << endl;
            exit(1);
        }

        assert(!trainNodePtrs.empty());
        const shared_ptr<VehicleNode> trainNodePtr = trainNodePtrs.front();
        const AgentIdType agentId = trainNodePtr->GetAgentId();

        shared_ptr<Agent> trainDriverPtr = Agent::CreateTrainDriverAgent(
            this,
            agentId,
            (*this).GetSimEngineInterfacePtr(agentId),
            profileAndTaskTablePtr->GetProfile(AGENT_TYPE_BUS),
            profileAndTaskTablePtr->GetEmptyTaskTable(),
            theAgentGisPtr,
            publicVehicleTablePtr,
            theRouteSearchSubsystemPtr,
            trainNodePtr,
            trainPtr);

        trainPtr->SetDriverAgentId(agentId);

        (*this).AddNode(trainNodePtr);
        (*this).AddAgentWithoutNodeGeneration(trainDriverPtr);

        trainNodePtr->SetPublicVehicle(
            *theParameterDatabaseReaderPtr,
            theAgentGisPtr,
            (*this).CurrentTime(),
            trainPtr);

        theAgentGisPtr->AddTrain(trainPtr);

        trainNodePtrs.pop();

        (*this).OutputTrace("Create Train " + ConvertToString(trainNodePtr->GetAgentId()));
    }
}

void MultiAgentSimulator::AddBus(const vector<shared_ptr<Bus> >& busPtrs)
{
    if (busPtrs.empty()) {
        return;
    }


    const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();

    for(size_t i = 0; i < busPtrs.size(); i++) {
        const shared_ptr<Bus>& busPtr = busPtrs[i];

        queue<shared_ptr<VehicleNode> >& busNodePtrs =
            reservedVehicleNodePtrs.at(busPtr->GetVehicleType());

        if (busNodePtrs.empty()) {
            cerr << "Error: not enough bus node" << endl;
            exit(1);
        }

        const shared_ptr<VehicleNode> busNodePtr = busNodePtrs.front();
        const AgentIdType agentId = busNodePtr->GetAgentId();
        const RoadIdType roadId = busPtr->GetStartRoadId();
        const BusStopIdType startBusStopId = busPtr->GetStartBusStopId();
        const VertexIdType vertexId = subsystem.GetBusStop(startBusStopId).GetVertexId();
        const size_t laneNumber = subsystem.GetRoad(roadId).GetOutsideOutgoingLaneNumber(vertexId);

        shared_ptr<Vehicle> vehiclePtr(
            new Vehicle(
                agentId,
                vertexId,
                theAgentGisPtr->GetVertex(vertexId),
                roadId,
                laneNumber,
                this));

        shared_ptr<Agent> busDriverPtr = Agent::CreateBusDriverAgent(
            this,
            agentId,
            (*this).GetSimEngineInterfacePtr(agentId),
            profileAndTaskTablePtr->GetProfile(AGENT_TYPE_BUS),
            profileAndTaskTablePtr->GetEmptyTaskTable(),
            theAgentGisPtr,
            publicVehicleTablePtr,
            theRouteSearchSubsystemPtr,
            vehiclePtr,
            busNodePtr,
            busPtr);

        (*this).AddNode(busNodePtr);
        (*this).AddAgentWithoutNodeGeneration(busDriverPtr);

        busPtr->SetVehicle(vehiclePtr);

        busNodePtr->SetPublicVehicle(
            *theParameterDatabaseReaderPtr,
            theAgentGisPtr,
            (*this).CurrentTime(),
            busPtr);

        theAgentGisPtr->AddVehicle(vehiclePtr);

        busNodePtrs.pop();

        (*this).OutputTrace("Create Bus " + ConvertToString(agentId));
    }
}

void MultiAgentSimulator::ProfileValueOutputSubsystem::RecordAssignedProfileValuesToFile(
    const AgentIdType& agentId,
    const shared_ptr<AgentProfile>& profilePtr,
    const map<string, double>& assignedValues,
    const bool hasCar,
    const bool hasBicycle)
{
    if (!profileValueOutputFile.is_open()) {
        return;
    }

    if (agentId == MASTER_ANY_AGENT_ID) {
        // Skip master agent profile
        return;
    }

    typedef map<string, double>::const_iterator IterType;

    if (agentId == MASTER_BUS_AGENT_ID) {

        profileValueOutputFile << "ProfileType:Bus" << endl;

    } else if (agentId == MASTER_TAXI_AGENT_ID) {

        profileValueOutputFile << "ProfileType:Taxi" << endl;

    } else {
        profileValueOutputFile << "ProfileType:" << agentId << endl;
    }

    for(IterType iter = assignedValues.begin();
        iter != assignedValues.end(); iter++) {

        const string& profileName = (*iter).first;
        const double value = (*iter).second;

        profileValueOutputFile << profileName << " = " << value << endl;
    }

    if ((agentId != MASTER_BUS_AGENT_ID) &&
        (agentId != MASTER_TAXI_AGENT_ID)) {

        double privateCarOwnership = 0.0;

        if (hasCar) {
            privateCarOwnership = 1.0;
        }

        profileValueOutputFile << RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_PRIVATE_CAR_OWNERSHIP] << " = " << privateCarOwnership << endl;

        double bicycleOwnership = 0.0;

        if (hasBicycle) {
            bicycleOwnership = 1.0;
        }

        profileValueOutputFile << RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_BICYCLE_OWNERSHIP] << " = " << bicycleOwnership << endl;

        const AgentMobilityClassType mobilityClass = profilePtr->GetMobilityClass();

        profileValueOutputFile << RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_WALK_SPEED_AT_TRANSFER] << " = " << GetAgentMobilityClassName(mobilityClass) << endl;

        const AgentValueFormula& utilityFormula1 = profilePtr->GetUtilityFormula1();
        const AgentValueFormula& utilityFormula2 = profilePtr->GetUtilityFormula2();

        if (!utilityFormula1.IsNull()) {
            profileValueOutputFile << RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_UTILITY1_FUNCTION] << " = " << utilityFormula1.GetInputFormulaString() << endl;
        }
        if (!utilityFormula2.IsNull()) {
            profileValueOutputFile << RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_UTILITY2_FUNCTION] << " = " << utilityFormula2.GetInputFormulaString() << endl;
        }
    }

    profileValueOutputFile << RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_ROUTE_PRIORITY] << " = " << profilePtr->GetPrimaryRouteCostFormula().GetInputFormulaString() << endl;

    profileValueOutputFile << endl;
}

void MultiAgentSimulator::AddCar()
{
    queue<shared_ptr<VehicleNode> >& vehicleNodePtrs = reservedVehicleNodePtrs.at(VEHICLE_CAR);

    list<shared_ptr<Vehicle> >& vehiclePtrs = newlyAddedVehiclePtrs;

    typedef list<shared_ptr<Vehicle> >::const_iterator IterType;

    for(IterType iter = vehiclePtrs.begin(); iter != vehiclePtrs.end(); iter++) {
        const shared_ptr<Vehicle>& vehiclePtr = (*iter);

        if (vehicleNodePtrs.empty()) {
            cerr << "Error: not enough vehicle node" << endl;
            exit(1);
        }

        shared_ptr<VehicleNode> vehicleNodePtr = vehicleNodePtrs.front();

        vehicleNodePtr->SetVehicle(
            *theParameterDatabaseReaderPtr,
            theAgentGisPtr,
            (*this).CurrentTime(),
            vehiclePtr);

        (*this).AddNode(vehicleNodePtr);

        vehicleNodePtrs.pop();

        theAgentGisPtr->AddVehicle(vehiclePtr);
    }
    vehiclePtrs.clear();
}



void MultiAgentSimulator::AddTaxi(const shared_ptr<Taxi>& taxiPtr)
{
    const AgentIdType agentId = taxiPtr->GetDriverAgentId();

    const shared_ptr<VehicleNode> vehicleNodePtr =
        (*this).CreateVehicleNode(agentId, VEHICLE_TAXI);

    const shared_ptr<Agent> taxiDriverPtr =
        Agent::CreateTaxiDriverAgent(
            this,
            agentId,
            (*this).GetSimEngineInterfacePtr(agentId),
            profileAndTaskTablePtr->GetProfile(AGENT_TYPE_TAXI),
            profileAndTaskTablePtr->GetEmptyTaskTable(),
            theAgentGisPtr,
            publicVehicleTablePtr,
            theRouteSearchSubsystemPtr,
            vehicleNodePtr,
            taxiPtr);

    (*this).AddNode(vehicleNodePtr);
    (*this).AddAgentWithoutNodeGeneration(taxiDriverPtr);

    vehicleNodePtr->SetTaxi(
        *theParameterDatabaseReaderPtr,
        theAgentGisPtr,
        (*this).CurrentTime(),
        taxiPtr);

    theAgentGisPtr->AddVehicle(taxiPtr);
}


void MultiAgentSimulator::AddCommunicationNode(const shared_ptr<AgentCommunicationNode>& aNodePtr)
{
    const AgentIdType agentId = aNodePtr->GetNodeId();

    if (entireAgentIds.find(agentId) == entireAgentIds.end()) {

        (*this).AddNode(aNodePtr);

    } else {

        synchronizedNodePtrs[agentId] = aNodePtr;

        (*this).AttachCommunicationNode(agentId, aNodePtr);
    }
}

bool MultiAgentSimulator::IsEqualToAgentId(const NodeIdType& nodeId) const
{
    return (entireAgentIds.find(nodeId) != entireAgentIds.end());
}

TimeType MultiAgentSimulator::GetWakeupTime(const NodeIdType& nodeId) const
{
    assert((*this).IsEqualToAgentId(nodeId));

    typedef map<NodeIdType, TimeType>::const_iterator IterType;

    IterType iter = wakeupTimes.find(nodeId);

    assert(iter != wakeupTimes.end());

    return (*iter).second;
}

void MultiAgentSimulator::AddAgent(
    const shared_ptr<Agent>& agentPtr,
    const bool withNodeGeneration)
{
    AgentResource resource(agentPtr);
    const AgentIdType agentId = resource.AgentId();

    // multiagent

    iterToAgentList[agentId] = agentPtrs.insert(agentPtrs.end(), agentPtr);

    newlyAddedAgentIds.insert(agentId);

    // sim
    if (withNodeGeneration) {
        shared_ptr<NetworkNode> simNodePtr;

        if (createCommunicationNodeAtWakeupTimeFor.find(agentId) != createCommunicationNodeAtWakeupTimeFor.end()) {
            (*this).CreateNewNode(*theParameterDatabaseReaderPtr, agentId, agentPtr->GetMobilityModelPtr());
        }

        if (synchronizedNodePtrs.find(agentId) != synchronizedNodePtrs.end()) {

            simNodePtr = synchronizedNodePtrs[agentId];
            synchronizedNodePtrs.erase(agentId);

        } else {
            simNodePtr.reset(
                new AgentNode(
                    *theParameterDatabaseReaderPtr,
                    theGlobalNetworkingObjectBag,
                    theGisSubsystemPtr,
                    agentPtr->GetSimEngineInterfacePtr(),
                    agentId,
                    runSeed,
                    agentPtr->GetMobilityModelPtr(),
                    resource));
        }

        (*this).AddNode(simNodePtr);
    }

    const GisPositionIdType firstPositionId = resource.GetCurrentPositionId();
    resource.SetCurrentPositionId(InvalidGisPositionId);
    resource.SetDesiredNextPositionId(firstPositionId);

    theAgentGisPtr->UpdatePeopleTranslationBetweenGisObjects(
        resource,
        resource.GetCurrentPositionId(),
        resource.GetCurrentPositionId(),
        resource.GetDesiredPositionId(),
        AGENT_BEHAVIOR_NOTHING,
        AGENT_BEHAVIOR_NOTHING);

    resource.ArrivedAtGisPositionNotification();

    typedef map<AgentIdType, list<shared_ptr<AgentCommunicationNode> > >::iterator CommunicationNodesIter;

    CommunicationNodesIter communicationNodesIter = communicationNodePtrsWaitingAgentCreation.find(agentId);

    if (communicationNodesIter != communicationNodePtrsWaitingAgentCreation.end()) {
        list<shared_ptr<AgentCommunicationNode> >& communicationNodePtrs = (*communicationNodesIter).second;

        typedef list<shared_ptr<AgentCommunicationNode> >::iterator NodeIter;

        for(NodeIter nodeIter = communicationNodePtrs.begin();
            nodeIter != communicationNodePtrs.end(); nodeIter++) {

            agentPtr->AddCommunicationNode(*nodeIter);
        }

        communicationNodePtrsWaitingAgentCreation.erase(agentId);
    }


    (*this).OutputTrace("Add Agent " + ConvertToString(agentId));
}

shared_ptr<SimulationEngineInterface> MultiAgentSimulator::GetSimEngineInterfacePtr(const AgentIdType& agentId)
{
    unsigned int partitionIndex = 0;
    if (theParameterDatabaseReaderPtr->ParameterExists(
            "parallelization-partition-index", agentId)) {
        partitionIndex =
            (theParameterDatabaseReaderPtr->ReadNonNegativeInt(
                "parallelization-partition-index", agentId) %
             theSimulationEnginePtr->GetNumberPartitionThreads());
    }

    return theSimulationEnginePtr->GetSimulationEngineInterface(
        *theParameterDatabaseReaderPtr, agentId, partitionIndex);
}

shared_ptr<VehicleNode> MultiAgentSimulator::CreateVehicleNode(
    const AgentIdType& agentId,
    const VehicleType& vehicleType)
{
    unsigned int partitionIndex = 0;

    if (theParameterDatabaseReaderPtr->ParameterExists(
            "parallelization-partition-index", agentId)) {
        partitionIndex =
            (theParameterDatabaseReaderPtr->ReadNonNegativeInt(
                "parallelization-partition-index", agentId) %
             theSimulationEnginePtr->GetNumberPartitionThreads());
    }

    const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr =
        theSimulationEnginePtr->GetSimulationEngineInterface(
            *theParameterDatabaseReaderPtr, agentId, partitionIndex);

    shared_ptr<VehicleMobilityModel> mobilityModelPtr(new VehicleMobilityModel(agentId));

    shared_ptr<VehicleNode> vehicleNodePtr(
        new VehicleNode(
            *theParameterDatabaseReaderPtr,
            theGlobalNetworkingObjectBag,
            theGisSubsystemPtr,
            simEngineInterfacePtr,
            agentId,
            runSeed,
            mobilityModelPtr));

    return vehicleNodePtr;
}

void MultiAgentSimulator::ReserveVehicleNode(
    const AgentIdType& agentId,
    const VehicleType& vehicleType)
{
    const shared_ptr<VehicleNode> vehicleNodePtr =
        (*this).CreateVehicleNode(agentId, vehicleType);

    reservedVehicleNodePtrs.at(vehicleType).push(vehicleNodePtr);
}

void MultiAgentSimulator::SetOwnerAgent(
    const AgentResource& resource,
    const AgentIdType& ownerAgentId)
{
    assert(resource.AgentId() != ownerAgentId);

    ownerChangeEvents[resource.AgentId()] = OwnerChangeEvent(resource.AgentId(), ownerAgentId);
}

void MultiAgentSimulator::RemoveOwnerAgentChange(const AgentResource& resource)
{
    ownerChangeEvents.erase(resource.AgentId());
}

void MultiAgentSimulator::ChangeAgentOwnerIfNecessary()
{
    list<pair<AgentIdType, shared_ptr<Agent> > > freeAgentPtrs;

    typedef map<AgentIdType, OwnerChangeEvent>::const_iterator IterType;

    for(IterType iter = ownerChangeEvents.begin();
        iter != ownerChangeEvents.end(); iter++) {

        const OwnerChangeEvent& ownerChangeEvent = (*iter).second;

        assert(iterToAgentList.find(ownerChangeEvent.agentId) != iterToAgentList.end());

        typedef list<shared_ptr<Agent> >::iterator AgentIter;

        AgentIter childIter = iterToAgentList[ownerChangeEvent.agentId];

        const shared_ptr<Agent> childAgentPtr = (*childIter);

        if (ownerChangeEvent.ownerId == MASTER_ANY_AGENT_ID) {
            assert(childAgentPtr->HasParent());

            AgentIter parentIter = iterToAgentList[childAgentPtr->parentAgentId];
            const shared_ptr<Agent> parentAgentPtr = (*parentIter);

            parentAgentPtr->childAgentPtrs.erase(childIter);

        } else {
            assert(!childAgentPtr->HasParent());

            agentPtrs.erase(childIter);
        }

        freeAgentPtrs.push_back(make_pair(ownerChangeEvent.ownerId, childAgentPtr));

        iterToAgentList.erase(ownerChangeEvent.agentId);
    }

    ownerChangeEvents.clear();

    typedef list<pair<AgentIdType, shared_ptr<Agent> > >::const_iterator IterType2;

    for(IterType2 iter = freeAgentPtrs.begin(); iter != freeAgentPtrs.end(); iter++) {
        const AgentIdType& ownerAgentId = (*iter).first;
        const shared_ptr<Agent>& agentPtr = (*iter).second;
        const AgentIdType agentId = agentPtr->GetAgentId();

        agentPtr->parentAgentId = ownerAgentId;

        if (ownerAgentId == MASTER_ANY_AGENT_ID) {

            iterToAgentList[agentId] = agentPtrs.insert(agentPtrs.end(), agentPtr);
        }
        else {
            list<shared_ptr<Agent> >::iterator listIter = iterToAgentList[ownerAgentId];
            shared_ptr<Agent> ownerAgentPtr = (*listIter);
            list<shared_ptr<Agent> >& childAgentPtrs = ownerAgentPtr->childAgentPtrs;

            iterToAgentList[agentId] = childAgentPtrs.insert(childAgentPtrs.end(), agentPtr);
        }
    }
}

void MultiAgentSimulator::DeleteInactiveAgents()
{
    while (!deleteNodeIds.empty()) {
        const AgentIdType agentId = deleteNodeIds.front();

        typedef list<shared_ptr<Agent> >::iterator AgentIter;

        if (isDebugMode) {
            cout << "delete agent " << agentId << endl;
        }

        AgentIter agentIter = iterToAgentList[agentId];
        shared_ptr<Agent> agentPtr = (*agentIter);

        if (!agentPtr->HasParent()) {
            agentPtrs.erase(agentIter);
        } else {

            if (iterToAgentList.find(agentPtr->parentAgentId) != iterToAgentList.end()) {
                AgentIter parentIter = iterToAgentList[agentPtr->parentAgentId];
                shared_ptr<Agent> parentAgentPtr = (*parentIter);

                parentAgentPtr->childAgentPtrs.erase(agentIter);
            }
        }

        assert(agentPtr->childAgentPtrs.empty());

        iterToAgentList.erase(agentId);

        deleteNodeIds.pop();

        (*this).DeleteNode(agentId);

        if (agentPtr->vehicleNodePtr !=nullptr) {
            shared_ptr<VehicleNode> vehicleNodePtr = agentPtr->vehicleNodePtr;

            reservedVehicleNodePtrs.at(vehicleNodePtr->GetVehicleType()).push(vehicleNodePtr);
        }
    }
}


void MultiAgentSimulator::IncrementTimeStep()
{
    typedef list<shared_ptr<Agent> >::const_iterator AgentIter;


    if (isSimulationDone) {
        return;
    }

    theAgentGisPtr->ExecuteActionsForCurrentTimestep();
    (*this).SyncApplicationCreation();

    // Delaying deletion until synchronization of all agents has been done.

    (*this).DeleteInactiveAgents();

    // main routine
    for(AgentIter iter = agentPtrs.begin(); iter != agentPtrs.end(); iter++) {

        Agent& agent = *(*iter);

        agent.IncrementTime();

        if (agent.IsDeletedAfterEndOfTimeStep()) {

            const AgentIdType& agentId = agent.GetAgentId();
            if (newlyAddedAgentIds.find(agentId) == newlyAddedAgentIds.end()) {
                deleteNodeIds.push(agentId);
            }
        }
    }

    // increment at the end of time step for GUI step compatibility.
    currentSnapshotId = (currentSnapshotId + 1) % NUMBER_TIMESTEP_SNAPSHOTS;
}



void MultiAgentSimulator::RerouteAllAgents()
{
    typedef list<shared_ptr<Agent> >::const_iterator AgentIter;

    for(AgentIter iter = agentPtrs.begin(); iter != agentPtrs.end(); iter++) {
        Agent& agent = *(*iter);

        agent.RecalculateRoute(currentTime);
    }
}

static const GisObjectIdType MINIMUM_GIS_OBJECT_ID = 100000000;

void Agent::OutputTraceEvent()
{
    if (simEngineInterfacePtr != nullptr &&
        simEngineInterfacePtr->TraceIsOn(TraceMas)) {

        if (utility1Trace.HasChanged()) {
            simulatorPtr->OutputTraceEvent(agentId, "Utility1", utility1Trace.GetValueAndUnsetChangeFlag());
        }
        if (utility2Trace.HasChanged()) {
            simulatorPtr->OutputTraceEvent(agentId, "Utility2" , utility2Trace.GetValueAndUnsetChangeFlag());
        }
        if (travelDistanceTrace.HasChanged()) {
            simulatorPtr->OutputTraceEvent(agentId, "TravelDistance" , travelDistanceTrace.GetValueAndUnsetChangeFlag());
        }
        if (travelTimeTrace.HasChanged()) {
            simulatorPtr->OutputTraceEvent(agentId, "TravelTime" , travelTimeTrace.GetValueAndUnsetChangeFlag());
        }
        if (destinationChangeTrace.HasChanged() &&
            destPositionId != UNREACHABLE_POSITION_ID) {
            simulatorPtr->OutputTraceEvent(agentId, "DestinationChangeCount" , static_cast<double>(destinationChangeTrace.GetValueAndUnsetChangeFlag()));

            const GisSubsystem& subsystem = theAgentGisPtr->GetSubsystem();
            const string name = subsystem.GetGisObject(destPositionId).GetObjectName();

            simulatorPtr->OutputStringTraceEvent(agentId, "Destination" , name);
        }
        if (destinationChangeByCommunicationTrace.HasChanged() &&
            destPositionId != UNREACHABLE_POSITION_ID) {

            simulatorPtr->OutputTraceEvent(agentId, "DestinationChangeCountByCommunication" , static_cast<double>(destinationChangeByCommunicationTrace.GetValueAndUnsetChangeFlag()));
        }
    }
}


void MultiAgentSimulator::OutputTraceEvent(
    const NodeIdType& gisObjectOrAgentId,
    const string& eventName,
    const double value) const
{
    TraceSubsystem& traceSubsystem = theSimulationEnginePtr->GetTraceSubsystem();

    if (traceSubsystem.BinaryOutputIsOn()) {

        const MultiAgentTraceRecord record(value);

        traceSubsystem.OutputTraceInBinary(
            currentTime,
            gisObjectOrAgentId,
            modelName,
            "",
            eventName,
            reinterpret_cast<const unsigned char* >(&record),
            sizeof(record),
            MASTER_THREAD_NUMBER);

    } else {

        ostringstream outStream;
        outStream << "V= " << value;

        traceSubsystem.OutputTrace(
            currentTime,
            gisObjectOrAgentId,
            modelName,
            "",
            eventName,
            outStream.str(),
            MASTER_THREAD_NUMBER);
    }
}

void MultiAgentSimulator::OutputStringTraceEvent(
    const NodeIdType& gisObjectOrAgentId,
    const string& eventName,
    const string& value) const
{
    TraceSubsystem& traceSubsystem = theSimulationEnginePtr->GetTraceSubsystem();

    if (traceSubsystem.BinaryOutputIsOn()) {
        StateTraceRecord record;

        const size_t nameSize = std::min(value.size(), sizeof(record.stateId) - 1);

        for(size_t i = 0; i < nameSize; i++) {
            record.stateId[i] = value[i];
        }
        record.stateId[nameSize] = '\0';

        traceSubsystem.OutputTraceInBinary(
            currentTime,
            gisObjectOrAgentId,
            modelName,
            "",
            eventName,
            reinterpret_cast<const unsigned char* >(&record),
            sizeof(record),
            MASTER_THREAD_NUMBER);

    } else {

        ostringstream outStream;
        outStream << "V= " << value;

        traceSubsystem.OutputTrace(
            currentTime,
            gisObjectOrAgentId,
            modelName,
            "",
            eventName,
            outStream.str(),
            MASTER_THREAD_NUMBER);
    }
}

void MultiAgentSimulator::AttachCommunicationNode(
    const AgentIdType& agentId,
    const shared_ptr<AgentCommunicationNode>& communicationNodePtr)
{
    bool found = false;

    typedef map<AgentIdType, list<shared_ptr<Agent> >::iterator>::const_iterator IterType;

    IterType iter = iterToAgentList.find(agentId);

    if (iter != iterToAgentList.end()) {
        (*(*iter).second)->AddCommunicationNode(communicationNodePtr);
        found = true;
    }

    if (!found) {
        communicationNodePtrsWaitingAgentCreation[agentId].push_back(communicationNodePtr);
    }
}



void MultiAgentSimulator::DetachCommunicationNode(
    const AgentIdType& agentId,
    const shared_ptr<AgentCommunicationNode>& communicationNodePtr)
{
    bool found = false;

    typedef map<AgentIdType, list<shared_ptr<Agent> >::iterator>::const_iterator IterType;

    IterType iter = iterToAgentList.find(agentId);

    if (iter != iterToAgentList.end()) {
        (*(*iter).second)->DeleteCommunicationNode(communicationNodePtr);
        found = true;
    }

    if (!found) {
        communicationNodePtrsWaitingAgentCreation[agentId].remove(communicationNodePtr);
    }
}

void MultiAgentSimulator::OutputAgentTraces()
{
    (*this).OutputTraceForAllNodePositions(ZERO_TIME);

    typedef list<shared_ptr<Agent> >::const_iterator AgentIter;

    for(AgentIter iter = agentPtrs.begin(); iter != agentPtrs.end(); iter++) {
        (*iter)->OutputTraceEvent();
    }
}

void MultiAgentSimulator::CreateApplicationForNode(
    const AgentResource& resource,
    const NodeIdType& sourceNodeId,
    const InterfaceOrInstanceIdType& instanceId,
    const vector<string>& parameterLines,
    const set<NodeIdType>& targetNodeIds)
{
    dynamicApplicationDatas.push_back(
        DynamicApplucationData(
            sourceNodeId,
            instanceId,
            parameterLines,
            targetNodeIds));
}

void MultiAgentSimulator::SyncApplicationCreation()
{
    for(size_t j = 0; j < dynamicApplicationDatas.size(); j++) {
        const DynamicApplucationData& dynamicApplicationData = dynamicApplicationDatas[j];

        (*this).GenerateDynamicApplication(dynamicApplicationData);
    }

    dynamicApplicationDatas.clear();
}

void MultiAgentSimulator::GenerateDynamicApplication(const DynamicApplucationData& dynamicApplicationData)
{
    const NodeIdType& sourceNodeId = dynamicApplicationData.sourceNodeId;
    const InterfaceOrInstanceIdType& instanceId = dynamicApplicationData.instanceId;
    const vector<string>& parameterLines = dynamicApplicationData.parameterLines;
    const set<NodeIdType>& targetNodeIds = dynamicApplicationData.targetNodeIds;

    assert(!parameterLines.empty());

    for(size_t i = 0; i < parameterLines.size(); i++) {
        const string& parameterLine = parameterLines[i];

        bool foundAnError;

        theParameterDatabaseReaderPtr->AddNewDefinitionToDatabase(parameterLine, foundAnError);

        if (foundAnError) {
            cerr << "Error: Failed to add dynamic application line " << parameterLine << endl;
            exit(1);
        }
    }

    using std::dynamic_pointer_cast;

    if (targetNodeIds.find(ANY_NODEID) != targetNodeIds.end()) {

        typedef map<NodeIdType, shared_ptr<NetworkNode> >::const_iterator IterType;

        for(IterType iter = nodes.begin(); iter != nodes.end(); iter++) {

            shared_ptr<NetworkNode> aNodePtr = (*iter).second;

            if (aNodePtr != nullptr) {
                 aNodePtr->CreateDynamicApplication(
                    *theParameterDatabaseReaderPtr,
                    theGlobalNetworkingObjectBag,
                    sourceNodeId,
                    instanceId);
            }
        }

    } else {

        typedef set<NodeIdType>::const_iterator IterType;

        for(IterType iter = targetNodeIds.begin(); iter != targetNodeIds.end(); iter++) {
            const NodeIdType& nodeId = (*iter);

            if (nodes.find(nodeId) == nodes.end()) {
                cerr << "Error: Network Node Id: " << nodeId << " Not Found." << endl;
                exit(1);
            }

            const shared_ptr<NetworkNode> networkNodePtr = nodes[nodeId];

            if (networkNodePtr == nullptr) {
                cerr << "Error: Network Node Id: " << nodeId << " is not NetworkNode." << endl;
                exit(1);
            }

            networkNodePtr->CreateDynamicApplication(
                *theParameterDatabaseReaderPtr,
                theGlobalNetworkingObjectBag,
                sourceNodeId,
                instanceId);
        }
    }
}

//---------------------------------------------------------------------------------

#pragma warning(disable:4355)

AgentProfileAndTaskTable::AgentProfileAndTaskTable(
    const ParameterDatabaseReader& initParameterDatabaseReader,
    const shared_ptr<PublicVehicleTable>& initPublicVehicleTablePtr,
    const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
    const AgentResource& masterResource)
    :
    thePublicVehicleTablePtr(initPublicVehicleTablePtr),
    emptyTaskTablePtr(new AgentTaskTable(this))
{
    const string agentProfileFilePath =
        initParameterDatabaseReader.ReadString(
            "multiagent-profile-file");

    const TimeType startTime =
        initParameterDatabaseReader.ReadTime(
            "multiagent-start-time");

    const double startTimeSec = static_cast<double>(startTime / SECOND);

    (*this).LoadProfile(
        *initGisSubsystemPtr, masterResource, agentProfileFilePath, startTimeSec);
}

#pragma warning(default:4355)

void AgentProfileAndTaskTable::CompleteInitialize(
    const ParameterDatabaseReader& initParameterDatabaseReader,
    const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
    const AgentResource& masterResource,
    const set<AgentIdType>& entireAgentIds)
{
    const string agentScheduleFilePath =
        initParameterDatabaseReader.ReadString(
            "multiagent-behavior-file");

    const TimeType startTime =
        initParameterDatabaseReader.ReadTime(
            "multiagent-start-time");

    const double startTimeSec = static_cast<double>(startTime / SECOND);

    map<string, set<string> > availableTaskTables;

    typedef set<AgentIdType>::const_iterator IterType;

    for(IterType iter = entireAgentIds.begin(); iter != entireAgentIds.end(); iter++) {

        const AgentIdType& agentId = (*iter);

        const string profileName = MakeLowerCaseString(
            initParameterDatabaseReader.ReadString(
                "multiagent-profile-type", agentId));

        const string taskTableName = MakeLowerCaseString(
            initParameterDatabaseReader.ReadString(
                "multiagent-behavior-type", agentId));

        if (AStringStartsWith(profileName, "taxi") ||
            profileName == "train" ||
            profileName == "bus" ||
            profileName == "vehicle" ||
            profileName == "privatecar") {
            // skip.
        } else {
            availableTaskTables[taskTableName].insert(profileName);
        }
    }

    (*this).LoadTaskTable(
        *initGisSubsystemPtr, masterResource, agentScheduleFilePath, startTimeSec, availableTaskTables);
}

shared_ptr<AgentProfile> AgentProfileAndTaskTable::GetProfile(const string& profileName) const
{
    if (!profilePtrs.Contains(profileName)) {
        cerr << "Error: invalid profile name " << profileName << endl;
        exit(1);
    }

    return profilePtrs[profileName];
}

shared_ptr<AgentTaskTable> AgentProfileAndTaskTable::GetTaskTable(
        const string& taskTableName,
        const string& profileName) const
{
    if (!profilePtrs.Contains(profileName)) {
        cerr << "Error: invalid profile name " << profileName << " for task " << taskTableName << endl;
        exit(1);
    }

    const AgentProfileType profileType = profilePtrs.GetId(profileName);

    typedef map<pair<string, AgentProfileType>, shared_ptr<AgentTaskTable> >::const_iterator IterType;

    IterType iter = taskTablePtrs.find(make_pair(taskTableName, profileType));

    if (iter == taskTablePtrs.end()) {
        cerr << "Specify available Behavior for Agent:" << taskTableName <<  " Profile:" << profileName << endl;
        exit(1);
    }

    return (*iter).second;
}

bool AgentProfileAndTaskTable::ContainsTask(
    const string& taskTableName,
    const string& profileName) const
{
    if (!profilePtrs.Contains(profileName)) {
        return false;
    }

    const AgentProfileType profileType = profilePtrs.GetId(profileName);

    typedef map<pair<string, AgentProfileType>, shared_ptr<AgentTaskTable> >::const_iterator IterType;

    IterType iter = taskTablePtrs.find(make_pair(taskTableName, profileType));

    return (iter != taskTablePtrs.end());
}

vector<string> AgentProfileAndTaskTable::GetProfileTypeNames() const
{
    vector<string> profileTypeNames;

    for(size_t i = 0; i < profilePtrs.Size(); i++) {
        profileTypeNames.push_back(profilePtrs[AgentProfileType(i)]->GetProfileName());
    }

    return profileTypeNames;
}

void AgentProfileAndTaskTable::LoadProfile(
    const GisSubsystem& theGisSubsystem,
    const AgentResource& masterResource,
    const string& profileFilePath,
    const double startTimeSec)
{
    ifstream inStream(profileFilePath.c_str());

    if (!inStream.good()) {
        cerr << "Error: Couldn't open Profile file " << profileFilePath << endl;
        exit(1);
    }

    AgentProfileType profileType = INVALID_AGENT_TYPE;
    set<string> specifiedparameterNames;

    while(!inStream.eof()) {
        string aLine;
        getline(inStream, aLine);

        DeleteTrailingSpaces(aLine);

        if (IsAConfigFileCommentLine(aLine)) {
            continue;
        }

        if (AStringStartsWith(MakeLowerCaseString(aLine), "profiletype")) {
            const string profileName = SeparateString(aLine,  ":").second;

            if (profileName.empty()) {
                cerr << "Error can't read agent line " << aLine << endl;
                exit(1);
            }

            profileType = profilePtrs.GetId(MakeLowerCaseString(profileName));

            profilePtrs[profileType].reset(new AgentProfile(profileType, profileName));
            specifiedparameterNames.clear();
            continue;
        }

        ConvertStringToLowerCase(aLine);

        if (profileType == INVALID_AGENT_TYPE) {
            cerr << "Error:Define Profile in AgenrProfile file." << endl;
            exit(1);
        }

        deque<string> parameterNameAndValue;
        TokenizeToTrimmedLowerString(aLine, "=", parameterNameAndValue);

        if (parameterNameAndValue.size() != 2) {
            cerr << "Error:Paremter definition for " << profilePtrs.GetLabel(profileType) << " :" << aLine << endl;
            exit(1);
        }

        const string& parameterName = parameterNameAndValue[0];
        const string& value = parameterNameAndValue[1];

        if (specifiedparameterNames.find(parameterName) != specifiedparameterNames.end()) {
            cerr << "Error: Profile \"" << parameterName << "\" is duplicated in \"" <<  profilePtrs.GetLabel(profileType) << "\"" << endl;
            exit(1);
        }//if//

        specifiedparameterNames.insert(parameterName);

        if (parameterName == RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_PRIVATE_CAR_OWNERSHIP]) {
            AgentProfile& profile = *profilePtrs[profileType];

            profile.hasCar =
                AgentValueFormula(profile.parameters, startTimeSec, value, charactorIds);

        } else if (parameterName == RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_BICYCLE_OWNERSHIP]) {
            AgentProfile& profile = *profilePtrs[profileType];

            profile.hasBicycle =
                AgentValueFormula(profile.parameters, startTimeSec, value, charactorIds);

        } else if (parameterName == RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_WALK_SPEED_AT_TRANSFER]) {

            const string& mobilityType = value;
            AgentProfile& profile = *profilePtrs[profileType];

            profile.mobilityClass = GetAgentMobilityClass(mobilityType);

        } else if (parameterName == RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_UTILITY1_FUNCTION] || parameterName == "utilityfunction"/*obsolete*/) {
            AgentProfile& profile = *profilePtrs[profileType];

            profile.utilityFormula1 =
                AgentValueFormula(profile.parameters, startTimeSec, value, charactorIds);

        } else if (parameterName == RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_UTILITY2_FUNCTION]) {
            AgentProfile& profile = *profilePtrs[profileType];

            profile.utilityFormula2 =
                AgentValueFormula(profile.parameters, startTimeSec, value, charactorIds);

        } else if (parameterName == RESERVED_SPECIAL_AGENT_STATUS_NAMES[AGENT_RESERVED_SPECIAL_STATUS_ROUTE_PRIORITY]) {
            AgentProfile& profile = *profilePtrs[profileType];

            const AgentValueFormula aFormula(profile.parameters, startTimeSec, value, charactorIds);

            if (parameterName == "routepriority-pedestrian" ||
                parameterName == "routepriority-walk") {

                profile.routeCostFormulas[AGENT_BEHAVIOR_PEDESTRIAN] = aFormula;

            } else if (parameterName == "routepriority-bicycle") {

                profile.routeCostFormulas[AGENT_BEHAVIOR_BICYCLE] = aFormula;

            } else if (parameterName == "routepriority-privatecar") {

                profile.routeCostFormulas[ AGENT_BEHAVIOR_VEHICLE] = aFormula;

            } else if (parameterName == "routepriority-train") {

                profile.routeCostFormulas[AGENT_BEHAVIOR_TRAIN] = aFormula;

            } else if (parameterName == "routepriority-bus") {

                profile.routeCostFormulas[AGENT_BEHAVIOR_BUS] = aFormula;

            } else if (parameterName == "routepriority-taxi") {

                profile.routeCostFormulas[AGENT_BEHAVIOR_TAXI] = aFormula;

            } else {

                for(size_t i = 0; i < profile.routeCostFormulas.size(); i++) {
                    if (profile.routeCostFormulas[i].IsNull()) {
                        profile.routeCostFormulas[i] = aFormula;
                    }
                }
            }

        } else {
            AgentProfile& profile = *profilePtrs[profileType];

            double defaultValue = 0;

            const AgentStatusIdType statusId = profile.parameters.GetId(parameterName);

            if (AGENT_RESERVED_STATUS_QUERY_TRIGGER_START <= statusId &&
                statusId <= AGENT_RESERVED_STATUS_QUERY_TRIGGER_END ) {
                defaultValue = DBL_MAX;
            }

            profile.parameters[parameterName] =
                AgentValueFormula(profile.parameters, startTimeSec, value, charactorIds, defaultValue);
        }
    }
}

static inline
bool IsEqualString(const string& aString)
{
    const size_t equalPos = aString.find("=");

    if (equalPos != string::npos) {

        const size_t conditionPos = aString.find_first_of("=<>!", equalPos + 1);

        if (conditionPos == equalPos + 1) {
            return false;
        }

        return true;
    }

    return false;
}

void AgentProfileAndTaskTable::LoadTaskTable(
    const GisSubsystem& theGisSubsystem,
    const AgentResource& masterResource,
    const string& behaviorFilePath,
    const double startTimeSec,
    const map<string, set<string> >& availableTaskTables)
{
    ifstream inStream(behaviorFilePath.c_str());

    if (!inStream.good()) {
        cerr << "Error: Couldn't open Behavior file " << behaviorFilePath << endl;
        exit(1);
    }

    //AgentTaskTableType taskTableType = INVALID_TASK_TABLE_TYPE;
    string taskTableName;
    vector<string> profileNames;
    vector<pair<AgentProfileType, shared_ptr<AgentTaskTable> > > defaultProfileTaskTablePtrs;

    while(!inStream.eof()) {
        string aLine;
        getline(inStream, aLine);

        DeleteTrailingSpaces(aLine);

        if (IsAConfigFileCommentLine(aLine)) {
            continue;
        }
        ConvertStringToLowerCase(aLine);

        if (AStringStartsWith(aLine, "locationgroup")) {

            const pair<string, string>& groupNameAndlocationNames =
                SeparateString(aLine.substr(string("locationgroup").length()),  "=");

            deque<string> locationNames;
            TokenizeToTrimmedLowerString(groupNameAndlocationNames.second, ",", locationNames);

            vector<GisPositionIdType>& positionIds =
                locationGroups[groupNameAndlocationNames.first];

            for(size_t i = 0; i < locationNames.size(); i++) {
                const GisPositionIdType positionId = theGisSubsystem.GetPosition(locationNames[i]);

                if (positionId.type == GIS_BUILDING ||
                    positionId.type == GIS_PARK ||
                    positionId.type == GIS_POI ||
                    positionId.type == GIS_AREA) {
                    positionIds.push_back(positionId);
                }
                else {
                    cerr << "Error: Group location can contain building, park, POI or area names only. name:" << theGisSubsystem.GetGisObject(positionId).GetObjectName() << endl;
                    exit(1);
                }
            }

        } else if (AStringStartsWith(aLine, "locationidgroup")) {

            const pair<string, string>& groupNameAndlocationNames =
                SeparateString(aLine.substr(string("locationidgroup").length()),  "=");

            deque<string> locationNames;
            TokenizeToTrimmedLowerString(groupNameAndlocationNames.second, ",", locationNames);

            vector<GisPositionIdType>& positionIds =
                locationGroups[groupNameAndlocationNames.first];

            for(size_t i = 0; i < locationNames.size(); i++) {
                bool success;
                int aValue;
                ConvertStringToInt(locationNames[i], aValue, success);

                if (!success) {
                    cerr << "Wrong format string for int value" << locationNames[i] << endl;
                    exit(1);
                }

                positionIds.push_back(
                    theGisSubsystem.GetPositionId(GisObjectIdType(aValue)));
            }

        } else if (AStringStartsWith(aLine, "behaviortype")) {
            const string behaviorName = SeparateString(aLine,  ":").second;

            if (behaviorName.empty()) {
                cerr << "Error can't read agent line " << aLine << endl;
                exit(1);
            }

            typedef map<string, set<string> >::const_iterator TaskTableIter;

            TaskTableIter taskTableIter = availableTaskTables.find(behaviorName);

            if (taskTableIter != availableTaskTables.end()) {
                taskTableName = behaviorName;

                typedef set<string>::const_iterator ProfileNameIter;

                const set<string>& availableProfileNames = (*taskTableIter).second;

                defaultProfileTaskTablePtrs.clear();

                for(ProfileNameIter nameIter = availableProfileNames.begin();
                    nameIter != availableProfileNames.end(); nameIter++) {

                    const string& profileName = (*nameIter);

                    if ((!profilePtrs.Contains(profileName)) &&
                        (profileName != "{simulation_node_id}")) {
                        cerr << "Error: invalid profile name " << profileName << endl;
                        exit(1);
                    }

                    const AgentProfileType profileType = profilePtrs.GetId(profileName);

                    defaultProfileTaskTablePtrs.push_back(
                        make_pair(profileType,
                                  shared_ptr<AgentTaskTable>(new AgentTaskTable(this))));

                    taskTablePtrs[make_pair(behaviorName, profileType)] =
                        defaultProfileTaskTablePtrs.back().second;
                }

            } else {
                taskTableName.clear();
            }

        } else if (!taskTableName.empty()) {

            (*this).AddTask(
                theGisSubsystem,
                taskTableName,
                masterResource,
                aLine,
                startTimeSec,
                defaultProfileTaskTablePtrs,
                locationGroups);
        }
    }
}

static inline
void SeparateToConditionAndActionString(
    const string& taskLine,
    string& conditionString,
    string& actionString)
{
    size_t numberRemainingArcs = 0;
    size_t currentPos = 0;
    size_t conditionStartPos = 1;
    size_t conditionEndPos = 0;

    do {
        currentPos = taskLine.find_first_of("[]", currentPos);

        if (currentPos != string::npos) {
            if (taskLine[currentPos] == '[') {
                if (numberRemainingArcs == 0) {
                    conditionStartPos = currentPos + 1;
                }
                numberRemainingArcs++;
            } else {
                if (numberRemainingArcs == 0) {
                    cerr << "Error: lack of \"[\":" << taskLine << endl;
                    exit(1);
                }
                if (numberRemainingArcs == 1) {
                    conditionEndPos = currentPos - 1;
                }
                numberRemainingArcs--;
            }
            currentPos++;
        }

    } while (numberRemainingArcs != 0 && currentPos != string::npos);

    if (conditionEndPos >= conditionStartPos) {
        conditionString = taskLine.substr(conditionStartPos, conditionEndPos - conditionStartPos + 1);
        actionString = taskLine.substr(conditionEndPos + 2);
    } else {
        conditionString = "";
        actionString = taskLine;
    }
}

static inline
void ResolveApplicationParameterSpecification(
    const string& parameteLine,
    string& instanceName,
    string& modelName,
    vector<DynamicApplicationDefinitionParameter>& parameters)
{
    parameters.clear();

    string applicationSpecificaitonLine;

    SeparateToConditionAndActionString(parameteLine, instanceName, applicationSpecificaitonLine);

    deque<string> parameterTokens;
    TokenizeToTrimmedLowerStringWithArc(applicationSpecificaitonLine, ",", parameterTokens);

    for(size_t i = 0; i < parameterTokens.size(); i++) {
        const pair<string, string>& parameterAndValue =
            SeparateString(parameterTokens[i],  "=");

        parameters.push_back(
            DynamicApplicationDefinitionParameter(
                parameterAndValue.first,
                parameterAndValue.second));
    }

    if (!parameters.empty()) {
        const pair<string, string>& parameterAndValue =
            SeparateString(parameters.front().applicationParameterName,  "-");

        modelName = parameterAndValue.first;
    } else {
        modelName = "";
    }
}

static inline
void ParseActionLine(const string& actionLine, deque<string>& actionStrings)
{
    actionStrings.clear();

    size_t equalPos = 0;
    size_t delimPos = 0;

    // Quote is replace by white space according to parsing.
    string simplifiedActionLine  = actionLine;

    while (true) {
        const size_t actionStringStartPos = delimPos;

        equalPos = simplifiedActionLine.find_first_of('=', delimPos);

        if (equalPos == string::npos) {
            break;
        }

        bool valueIsOutOfQuote = true;
        int numberRemainingArcs = 0;

        delimPos = equalPos + 1;

        while (true) {

            delimPos = simplifiedActionLine.find_first_of(",\"()", delimPos);

            if (delimPos == string::npos) {
                break;
            }

            if (simplifiedActionLine[delimPos] == '(') {
                numberRemainingArcs++;
                delimPos++;
                continue;
            }

            if (simplifiedActionLine[delimPos] == ')') {
                numberRemainingArcs--;
                delimPos++;
                continue;
            }

            if (valueIsOutOfQuote &&
                numberRemainingArcs == 0 &&
                simplifiedActionLine[delimPos] == ',') {

                actionStrings.push_back(
                    simplifiedActionLine.substr(
                        actionStringStartPos,
                        delimPos - actionStringStartPos));

                delimPos++;
                break;
            }

            if (simplifiedActionLine[delimPos] == '"') {
                valueIsOutOfQuote = !valueIsOutOfQuote;
                simplifiedActionLine[delimPos] = ' '; //replace
            }

            delimPos++;
        }

        if (numberRemainingArcs != 0) {
            cerr << "Error: Mismatched number of function arcs in " << actionLine << endl;
            exit(1);
        }

        if (delimPos == string::npos) {

            if (!valueIsOutOfQuote) {
                cerr << "Error: invalid behavior line: [" << actionLine << "] Check number of \"" << endl;
                exit(1);
            }

            // found a specification(;equal) but no separator --> push last action
            actionStrings.push_back(
                simplifiedActionLine.substr(actionStringStartPos));

            break;
        }
    }

    for(size_t i = 0; i < actionStrings.size(); i++) {
        ConvertStringToLowerCase(actionStrings[i]);
    }
}

void AgentProfileAndTaskTable::AddTask(
    const GisSubsystem& theGisSubsystem,
    const string& taskTableName,
    const AgentResource& masterResource,
    const string& taskLine,
    const double startTimeSec,
    const vector<pair<AgentProfileType, shared_ptr<AgentTaskTable> > >& defaultProfileTaskTablePtrs,
    const map<string, vector<GisPositionIdType> >& locationGroups_NotUsed)
{
    string conditionLine;
    string actionLine;

    SeparateToConditionAndActionString(taskLine, conditionLine, actionLine);

    deque<string> conditionStrings;
    TokenizeToTrimmedLowerStringWithArc(conditionLine, ",", conditionStrings);

    deque<string> actionStrings;
    ParseActionLine(actionLine, actionStrings);

    vector<pair<AgentProfileType, shared_ptr<AgentTaskTable> > > availableProfileTaskTablePtrs;
    bool foundProfileTypeSpecification = false;

    for(size_t i = 0; (!foundProfileTypeSpecification && i < conditionStrings.size()); i++) {

        const string& conditionString = conditionStrings[i];

        if (IsEqualString(conditionString)) {
            const pair<string, string>& parameterAndValue =
                SeparateString(conditionString,  "=");

            if (parameterAndValue.first == "profiletype") {
                foundProfileTypeSpecification = true;

                deque<string> profileNames;
                TokenizeToTrimmedLowerStringWithArc(parameterAndValue.second, ";", profileNames);

                for(size_t j = 0; j < profileNames.size(); j++) {

                    const string& profileName = profileNames[j];

                    if (!profilePtrs.Contains(profileName)) {
                        cerr << "Error: invalid profile name " << profileName << " at " << conditionLine << endl;
                        exit(1);
                    }

                    const AgentProfileType profileType = profilePtrs.GetId(profileName);

                    availableProfileTaskTablePtrs.push_back(
                        make_pair(profileType,
                                  taskTablePtrs[make_pair(taskTableName, profileType)]));
                }
            }
        }
    }
    if (!foundProfileTypeSpecification) {
        availableProfileTaskTablePtrs = defaultProfileTaskTablePtrs;
    }

    for(size_t i = 0; i < availableProfileTaskTablePtrs.size(); i++) {
        const pair<AgentProfileType, shared_ptr<AgentTaskTable> >& profileTaskTablePtr =
                   availableProfileTaskTablePtrs[i];

        const AgentProfileType& profileType = profileTaskTablePtr.first;
        const AgentProfile& profile = *(profilePtrs[profileType]);

        AgentTaskTable& taskTable = *(profileTaskTablePtr.second);

        shared_ptr<AgentTask> taskPtr(new AgentTask(this));
        AgentTask& task = *taskPtr;

        for(size_t j = 0; j < conditionStrings.size(); j++) {
            const string& conditionString = conditionStrings[j];

            if (j == 0 && conditionString.find_first_of("=<>!") == string::npos) {

                task.startTime = AgentValueFormula(
                    profile.parameters, startTimeSec, conditionString, charactorIds);

            } else {
                const size_t conditionPos = conditionString.find_first_of("=<>!");

                if (!(conditionPos > 0 && conditionPos != string::npos) ){
                    cerr << "Error can't read condition line " << taskLine << endl;
                    exit(1);
                }

                const string parameterName = TrimmedString(conditionString.substr(0, conditionPos - 1));

                if (parameterName == "profiletype") {
                    continue;
                }

                const string value = conditionString.substr(conditionPos);

                if (!profile.parameters.Contains(parameterName)) {
                    cerr << "Error: Profile Type " << profilePtrs.GetLabel(profileType)
                         << " doesn't have status parameter " << parameterName << endl;
                    exit(1);
                }

                const AgentStatusIdType statusId =
                    profile.parameters.GetId(parameterName);

                const AgentConditionChecker::ConditionParameterType parameterType =
                    AgentConditionChecker::ConvertToParameterType(statusId);

                task.conditionCheckers.push_back(
                    AgentConditionChecker(
                        profile.parameters,
                        parameterType,
                        startTimeSec,
                        value,
                        charactorIds));
            }
        }

        vector<pair<string, string> > parameterAndValues;

        bool isJustStatusChange = true;
        bool hasWaitingAction = false;
        bool isInterrputionBehavior = false;

        for(size_t j = 0; j < actionStrings.size(); j++) {
            const string& actionString = actionStrings[j];

            parameterAndValues.push_back(SeparateString(actionString,  "="));

            const string& parameterName = parameterAndValues.back().first;

            if (parameterName == "initiallocationid" ||
                parameterName == "initiallocation" ||
                parameterName == "movetodestinationid" ||
                parameterName == "movetodestination" ||
                parameterName == "waituntil" ||
                parameterName == "wait") {

                isJustStatusChange = false;

            } else if (parameterName == "interruptcurrentaction") {

                const string& value = parameterAndValues.back().second;

                if (value == "terminatenow" || value == "resumeafterinterruption") {
                    isInterrputionBehavior = true;
                }

            } else if (parameterName == "wait") {
                hasWaitingAction = true;
            }
        }

        bool isBeforeSpecification = true;
        bool isBeforeWaiting = true;

        bool specifiedDestinationChoiceType = false;

        for(size_t j = 0; j < parameterAndValues.size(); j++) {
            const pair<string, string>& parameterAndValue = parameterAndValues[j];
            const string& parameterName = parameterAndValue.first;
            const string& value = parameterAndValue.second;

            if (parameterName == "initiallocationid" ||
                parameterName == "initiallocation") {

                if (!taskTable.initialLocation.locationName.empty()) {
                    cerr << "Error: duplicated location definition for " << taskLine << endl;
                    exit(1);
                }
                if (value == "initiallocation") {
                    cerr << "Error: \"InitialLocation\" is a reserved destination string." << endl;
                    exit(1);
                }

                if (parameterName == "initiallocationid") {
                    taskTable.initialLocation.isId = true;
                }
                taskTable.initialLocation.locationName = value;

            } else if (parameterName == "neardestinationid" ||
                       parameterName == "neardestination") {

                cerr << "Error: Use \"Destination=" << value << "、DestinationChoiceType=Nearest\" specification instead of \"" << parameterName << "=" << value << "\"" << endl;
                    exit(1);

            } else if (parameterName == "movetodestinationid" ||
                       parameterName == "movetodestination") {

                if (!task.destination.locationName.empty()) {
                    cerr << "Error: duplicated destination definition for " << taskLine << endl;
                    exit(1);
                }

                task.destination.locationName = value;

                isBeforeSpecification = false;

            } else if (parameterName == "destinationchoicetype") {

                if (value == "nearest") {
                    task.destination.locationChoiceType = AGENT_LOCATION_CHOICE_NEAREST;
                } else if (value == "random") {
                    task.destination.locationChoiceType = AGENT_LOCATION_CHOICE_RANDOM;
                } else {
                    cerr << "Error: invalid destination choice type " << value << endl;
                    exit(1);
                }
                specifiedDestinationChoiceType = true;

            } else if (parameterName == "destinationchoicebaselocation") {

                if (!task.destination.nearestChoiceBaseLocationName.empty()) {
                    cerr << "Error: duplicated estination choice base location definition for " << taskLine << endl;
                    exit(1);
                }

                task.destination.nearestChoiceBaseLocationName = value;

            } else if (parameterName == "departuretime") {

                task.departureTime = AgentValueFormula(
                    profile.parameters, startTimeSec, value, charactorIds);

            } else if (parameterName == "earliestdeparturetime") {

                task.earlyDepartureTime = AgentValueFormula(
                    profile.parameters, startTimeSec, value, charactorIds);

            } else if (parameterName == "latestdeparturetime") {

                task.lateDepartureTime = AgentValueFormula(
                    profile.parameters, startTimeSec, value, charactorIds);

            } else if (parameterName == "arrivaltime") {

                task.arrivalTime = AgentValueFormula(
                    profile.parameters, startTimeSec, value, charactorIds);

            } else if (parameterName == "earliestarrivaltime") {

                task.earlyArrivalTime = AgentValueFormula(
                    profile.parameters, startTimeSec, value, charactorIds);

            } else if (parameterName == "latestarrivaltime") {

                task.lateArrivalTime = AgentValueFormula(
                    profile.parameters, startTimeSec, value, charactorIds);

            } else if (parameterName == "waituntil") {

                task.endTime = AgentValueFormula(
                    profile.parameters, startTimeSec, value, charactorIds);

                isBeforeSpecification = false;

            } else if (parameterName == "wait") {

                task.waitTime = AgentValueFormula(
                    profile.parameters, 0, value, charactorIds);

                isBeforeWaiting = false;

            } else if (parameterName == "preferedmobilitymeans") {

                if (value == "pedestrian" || value == "walk") {
                    task.preferedBehavior = AGENT_BEHAVIOR_PEDESTRIAN;
                } else if (value == "bicycle") {
                    task.preferedBehavior = AGENT_BEHAVIOR_BICYCLE;
                } else if (value == "vehicle" || value == "car" || value == "privatecar") {
                    task.preferedBehavior = AGENT_BEHAVIOR_VEHICLE;
                } else if (value == "taxi") {
                    task.preferedBehavior = AGENT_BEHAVIOR_TAXI;
                } else if (value == "bus") {
                    task.preferedBehavior = AGENT_BEHAVIOR_BUS;
                } else if (value == "train") {
                    task.preferedBehavior = AGENT_BEHAVIOR_TRAIN;
                } else {
                    cerr << "Error: invalid PreferedMobility<eans " << value << endl;
                    exit(1);
                }

            } else if (parameterName == "mobilitymeans") {

                if (value == "pedestrian" || value == "walk") {
                    task.behavior = AGENT_BEHAVIOR_PEDESTRIAN;
                } else if (value == "bicycle") {
                    task.behavior = AGENT_BEHAVIOR_BICYCLE;
                } else if (value == "vehicle" || value == "car" || value == "privatecar") {
                    task.behavior = AGENT_BEHAVIOR_VEHICLE;
                } else if (value == "taxi") {
                    task.behavior = AGENT_BEHAVIOR_TAXI;
                } else if (value == "bus") {
                    task.behavior = AGENT_BEHAVIOR_BUS;
                } else if (value == "train") {
                    task.behavior = AGENT_BEHAVIOR_TRAIN;
                } else {
                    cerr << "Error: invalid Mobility<eans " << value << endl;
                    exit(1);
                }

            } else if (parameterName == "reservation") {

                if (!IsFunctionString(value)) {
                    cerr << "Error: invalid reservation  " << value << endl;
                    exit(1);
                }

                const size_t arcPos = value.find_first_of('(');
                const string functionString = value.substr(0, arcPos);

                cerr << "Error: invalid reservation function " << functionString << endl;
                exit(1);

            } else if (parameterName == "intersectionstogothrough") {

                deque<string> passIntersectionNames;
                TokenizeToTrimmedLowerString(value, ":", passIntersectionNames);

                for(size_t k = 0; k < passIntersectionNames.size(); k++) {
                    task.passIntersectionLocationInfos.push_back(
                        LocationInfo(
                            false/*isId*/,
                            AGENT_LOCATION_CHOICE_RANDOM,
                            passIntersectionNames[k]));
                }

            } else if (parameterName == "interruptcurrentaction") {

                if (value == "terminatenow") {
                    task.interruptionType = AGENT_BEHAVIOR_INTERRUPTION_REPLACE;
                } else if (value == "resumeafterinterruption") {
                    task.interruptionType = AGENT_BEHAVIOR_INTERRUPTION_LATER;
                } else {
                    task.interruptionType = AGENT_BEHAVIOR_INTERRUPTION_NONE;
                }

            } else if (parameterName == "generateapplication") {

                string applicationModelName;
                string instanceName;
                vector<DynamicApplicationDefinitionParameter> applicationParameters;

                ResolveApplicationParameterSpecification(
                    value,
                    instanceName,
                    applicationModelName,
                    applicationParameters);

                const AgentStatusChangeType changeType = GetStatusChangeType(
                    isJustStatusChange,
                    hasWaitingAction,
                    isBeforeSpecification,
                    isInterrputionBehavior,
                    isBeforeWaiting);

                const DynamicApplicationIdType dynamicApplicationId(
                    applicationModelName,
                    instanceName);

                task.additionalStatusChanges[changeType].applicationSpecifications[dynamicApplicationId].parameters = applicationParameters;

            } else {
                //assert(profile.parameters.Contains(parameterName));

                if (!profile.parameters.Contains(parameterName)) {
                    cerr << "Warning: Couldn't find an agent profile parameter: " << parameterName << endl
                         << "   A profile change behavior [" << actionStrings[j] << "] is skippped." << endl;
                    continue;
                }

                const AgentStatusChangeType changeType = GetStatusChangeType(
                    isJustStatusChange,
                    hasWaitingAction,
                    isBeforeSpecification,
                    isInterrputionBehavior,
                    isBeforeWaiting);

                const AgentStatusIdType statusId =
                    profile.parameters.GetId(parameterName);

                task.additionalStatusChanges[changeType].statusChanges.push_back(
                    make_pair(statusId, AgentValueFormula(profile.parameters, startTimeSec, value, charactorIds)));
            }
        }

        if (isInterrputionBehavior) {
            if (isJustStatusChange) {
                cerr << "Error: Specify destination or wait for InterruptCurrentAction " << actionLine << endl;
                exit(1);
            }
            taskTable.interruptTaskPtrs.push_back(taskPtr);

        } else {

            if (isJustStatusChange) {
                taskTable.statusChangePtrs.push_back(taskPtr);
            } else {
                taskTable.taskPtrs.push_back(taskPtr);
            }
        }

        if (taskTable.initialLocation.locationName.empty()) {
            cerr << "Error: Set initial location for Behavior Type " << taskTableName << " Profile Type " << profilePtrs.GetLabel(profileType) << endl;
            exit(1);
        }
    }
}

AgentValueFormula AgentProfileAndTaskTable::MakeValueFormula(
    const AgentProfileType& profileType,
    const string& formula)
{
    const AgentProfile& profile = *profilePtrs[profileType];

    const double defaultValue = 0;
    const double startTimeSec = 0;

    return AgentValueFormula(profile.parameters, startTimeSec, formula, charactorIds, defaultValue);
}


}//namespace
