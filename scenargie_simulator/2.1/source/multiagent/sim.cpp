// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "scenargiesim.h"
#include "multiagent_agentsim.h"
#include "scensim_gui_interface.h"

using std::cerr;
using std::endl;

using std::shared_ptr;

using ScenSim::GrabArgvForLicensing;
using ScenSim::MainFunctionArgvProcessingMultiSystemsParallelVersion1;
using ScenSim::GuiInterfacingSubsystem;
using ScenSim::RandomNumberGeneratorSeedType;
using ScenSim::ParameterDatabaseReader;
using ScenSim::SimulationEngine;
using ScenSim::TimeType;
using ScenSim::ConvertToUShortInt;

using MultiAgent::MultiAgentSimulator;

int main(int argc, char* argv[])
{
    GrabArgvForLicensing(argc, argv);

    string configFileName;
    bool isControlledByGui;
    unsigned int numberParallelThreads;
    bool runSequentially;
    bool isSeedSet;
    RandomNumberGeneratorSeedType runSeed;
    bool isScenarioSettingOutputMode;
    string outputConfigFileName;

    MainFunctionArgvProcessingMultiSystemsParallelVersion1(
        argc,
        argv,
        configFileName,
        isControlledByGui,
        numberParallelThreads,
        runSequentially,
        isSeedSet,
        runSeed,
        isScenarioSettingOutputMode,
        outputConfigFileName);

    shared_ptr<ParameterDatabaseReader> theParameterDatabaseReaderPtr(
        new ParameterDatabaseReader(configFileName));
    ParameterDatabaseReader& theParameterDatabaseReader =
        *theParameterDatabaseReaderPtr;

    if (!isSeedSet) {
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

    MultiAgentSimulator theMultiAgentSimulator(
        theParameterDatabaseReaderPtr,
        theSimulationEnginePtr,
        runSeed,
        runSequentially,
        isScenarioSettingOutputMode,
        configFileName,
        outputConfigFileName);

    // Currently the output supports ZERO_TIME setting only.
    if (isScenarioSettingOutputMode) {
        return 0;
    }//if//

    if (isControlledByGui) {

        GuiInterfacingSubsystem guiInterface(
            theSimulationEnginePtr,
            ConvertToUShortInt(
                theParameterDatabaseReader.ReadNonNegativeInt("gui-portnumber-sim"),
                "Error in parameter gui-portnumber-sim: port number is too large."),
            ConvertToUShortInt(
                theParameterDatabaseReader.ReadNonNegativeInt("gui-portnumber-pausecommand"),
                "Error in parameter gui-portnumber-pausecommand: port number is too large."));

        while (true) {
            TimeType simulateUpToTime;

            guiInterface.GetAndExecuteGuiCommands(
                theParameterDatabaseReader,
                theMultiAgentSimulator,
                simulateUpToTime);

            if (theSimulationEnginePtr->SimulationIsDone()) {
                break;
            }//if//

            theMultiAgentSimulator.RunSimulationUntil(simulateUpToTime);

        }//while//

    } else {
        const TimeType endSimTime = theParameterDatabaseReader.ReadTime("simulation-time");

        theMultiAgentSimulator.RunSimulationUntil(endSimTime);

    }//if//

    return 0;
}
