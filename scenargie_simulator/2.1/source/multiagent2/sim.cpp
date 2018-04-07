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
#include "multiagent2_sim.h"
#include "scensim_gui_interface.h"

using ScenSim::SimulationEngine;
using ScenSim::TimeType;
using ScenSim::ZERO_TIME;
using ScenSim::SECOND;
using ScenSim::ParameterDatabaseReader;
using ScenSim::RandomNumberGeneratorSeedType;
using ScenSim::ConvertToUShortInt;

using ScenSim::GrabArgvForLicensing;
using ScenSim::MainFunctionArgvProcessingBasicParallelVersion1;

using std::shared_ptr;
using std::cerr;
using std::endl;


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

    MultiAgent2::MultiAgentSim2d
        theSimulator(theParameterDatabaseReaderPtr, theSimulationEnginePtr, runSeed);


    if (isControlledByGui) {

        ScenSim::GuiInterfacingSubsystem guiInterface(
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
                theSimulator,
                simulateUpToTime);

            if (theSimulationEnginePtr->SimulationIsDone()) {
                break;
            }//if//

            theSimulator.RunSimulationUntil(simulateUpToTime);

        }//while//

    } else {
        const TimeType endSimTime = theParameterDatabaseReader.ReadTime("simulation-time");

        theSimulator.RunSimulationUntil(endSimTime);

    }//if//

    return 0;

}//main//

