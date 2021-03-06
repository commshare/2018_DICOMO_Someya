// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11_EXTENSION_NOTAVAILABLE_H
#define DOT11_EXTENSION_NOTAVAILABLE_H

#ifndef DOT11_EXTENSION_CHOOSER_H
#error "Include dot11_extension_chooser.h (indirect include only)"
#endif

#include "scensim_proploss.h"
#include "scensim_prop.h"
#include "scensim_mimo_channel.h"
#include "scensim_freqsel_channel.h"
#include "scensim_gis.h"
#include "scensim_mac.h"
#include "scensim_bercurves.h"

namespace Dot11 {

using std::cerr;
using std::endl;
using std::string;
using std::shared_ptr;
using ScenSim::SimplePropagationModelForNode;
using ScenSim::MimoChannelModel;
using ScenSim::MimoChannelModelInterface;
using ScenSim::FrequencySelectiveFadingModel;
using ScenSim::FrequencySelectiveFadingModelInterface;
using ScenSim::SimulationEngineInterface;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeIdType;
using ScenSim::InterfaceIdType;
using ScenSim::InterfaceOrInstanceIdType;
using ScenSim::RandomNumberGeneratorSeedType;
using ScenSim::NetworkLayer;
using ScenSim::BitOrBlockErrorRateCurveDatabase;
using ScenSim::MacLayer;

class Dot11Phy {
public:
    struct PropFrameType {};

    Dot11Phy(
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<SimplePropagationModelForNode<PropFrameType> >& propModelInterfacePtr,
        const shared_ptr<BitOrBlockErrorRateCurveDatabase>& berCurveDatabasePtr,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const RandomNumberGeneratorSeedType& nodeSeed)
    {
        cerr << "Error: DOT11 module is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables DOT11 Module." << endl;
        exit(1);
   }
};//Dot11Phy//

class Dot11Mac : public MacLayer {
public:
    typedef Dot11Phy::PropFrameType PropFrameType;

    static shared_ptr<Dot11Mac> Create(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<SimplePropagationModelForNode<PropFrameType> >& propModelInterfacePtr,
        const shared_ptr<MimoChannelModelInterface>& mimoChannelModelInterfacePtr,
        const shared_ptr<FrequencySelectiveFadingModelInterface>& frequencySelectiveFadingModelInterfacePtr,
        const shared_ptr<BitOrBlockErrorRateCurveDatabase>& berCurveDatabasePtr,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<NetworkLayer>& networkLayerPtr,
        const RandomNumberGeneratorSeedType& nodeSeed)
    {
        cerr << "Error: DOT11 module is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables DOT11 Module." << endl;
        exit(1);
    }

    static shared_ptr<Dot11Mac> Create(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<SimplePropagationModelForNode<Dot11Phy::PropFrameType> >& propModelInterfacePtr,
        const shared_ptr<BitOrBlockErrorRateCurveDatabase>& berCurveDatabasePtr,
        const NodeIdType& initNodeId,
        const InterfaceIdType& initInterfaceId,
        const unsigned int initInterfaceIndex,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
        const RandomNumberGeneratorSeedType& nodeSeed)
    {
        cerr << "Error: DOT11 module is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables DOT11 Module." << endl;
        exit(1);
    }

    void NetworkLayerQueueChangeNotification() {}
    void DisconnectFromOtherLayers() {}

};//Dot11Mac//

static inline
void ChooseAndCreateChannelModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceOrInstanceIdType& propagationModeInstanceId,
    const unsigned int baseChannelNumber,
    const unsigned int numberChannels,
    const vector<double>& channelCarrierFrequenciesMhz,
    const vector<double>& channelBandwidthsMhz,
    const RandomNumberGeneratorSeedType& runSeed,
    shared_ptr<MimoChannelModel>&,
    shared_ptr<FrequencySelectiveFadingModel>&)
{
    cerr << "Error: DOT11 module is not available." << endl;
    cerr << "Please confirm that the executable (simulator) enables DOT11 Module." << endl;
    exit(1);
}//ChooseAndCreateChannelModel//

}//namespace//


#endif
