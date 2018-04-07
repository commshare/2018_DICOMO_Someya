// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11_TXPOWERCONTROL_H
#define DOT11_TXPOWERCONTROL_H

#include "scensim_engine.h"
#include "scensim_netsim.h"
#include "dot11_common.h"

namespace Dot11{

using std::shared_ptr;
using std::cerr;
using std::endl;

using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeIdType;
using ScenSim::InterfaceIdType;
using ScenSim::SimulationEngineInterface;
using ScenSim::MakeLowerCaseString;


class AdaptiveTxPowerController {
public:
    AdaptiveTxPowerController(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId);

    virtual double CurrentTransmitPowerDbm(const MacAddressType& macAddress) const
        { return transmitPowerDbm; }

    virtual ~AdaptiveTxPowerController() { }

    bool TxPowerIsSpecifiedByPhyLayer() const { return txPowerIsSpecifiedByPhyLayer; }

protected:

    double transmitPowerDbm;
    bool txPowerIsSpecifiedByPhyLayer;

};//AdaptiveTxPowerController//


inline
AdaptiveTxPowerController::AdaptiveTxPowerController(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const InterfaceIdType& interfaceId)
    :
    txPowerIsSpecifiedByPhyLayer(true)
{
    string powerSpecifiedBy = "phylayer";

    if (theParameterDatabaseReader.ParameterExists("dot11-tx-power-specified-by", nodeId, interfaceId)) {
        powerSpecifiedBy = MakeLowerCaseString(
            theParameterDatabaseReader.ReadString("dot11-tx-power-specified-by", nodeId, interfaceId));
    }//if//

    if (powerSpecifiedBy == "phylayer") {

        txPowerIsSpecifiedByPhyLayer = true;

        transmitPowerDbm =
            theParameterDatabaseReader.ReadDouble("dot11-tx-power-dbm", nodeId, interfaceId);
    }
    else if (powerSpecifiedBy == "upperlayer") {

        txPowerIsSpecifiedByPhyLayer = false;

        transmitPowerDbm =
            theParameterDatabaseReader.ReadDouble("dot11-default-tx-power-dbm-when-not-specified", nodeId, interfaceId);                
    }
    else {
        cerr << "Error: Unknown dot11-tx-power-specified-by = " << powerSpecifiedBy << endl;
        exit(1);
    }//if//

}//AdaptiveTxPowerController//


//--------------------------------------------------------------------------------------------------

inline
shared_ptr<AdaptiveTxPowerController> CreateAdaptiveTxPowerController(
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const InterfaceIdType& interfaceId)
{

    return shared_ptr<AdaptiveTxPowerController>(
        new AdaptiveTxPowerController(
            theParameterDatabaseReader,
            nodeId,
            interfaceId));

    //string adaptiveTxPowerControlType =
    //    theParameterDatabaseReader.ReadString(
    //        "dot11-adaptive-tx-power-control-type", nodeId, interfaceId);

}//CreateAdaptiveTxPowerController//


}//namespace

#endif
