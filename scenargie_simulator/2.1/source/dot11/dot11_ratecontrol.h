// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11_RATECONTROL_H
#define DOT11_RATECONTROL_H

#include "scensim_engine.h"
#include "scensim_netsim.h"
#include "dot11_common.h"
#include "dot11_phy.h"

#include <map>
#include <string>
#include <vector>


namespace Dot11 {

using std::map;
using std::set;
using std::istringstream;
using std::cerr;
using std::endl;
using std::shared_ptr;
using std::enable_shared_from_this;

using ScenSim::EventRescheduleTicket;
using ScenSim::ConvertAStringSequenceOfANumericTypeIntoAVector;
using ScenSim::ConvertStringToLowerCase;
using ScenSim::ConvertAStringSequenceOfRealValuedPairsIntoAMap;
using ScenSim::ConvertAStringIntoAVectorOfStrings;

using ScenSim::CalcNodeId;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeIdType;
using ScenSim::InterfaceIdType;
using ScenSim::SimulationEngineInterface;
using ScenSim::TimeType;
using ScenSim::SimulationEvent;



enum AckDatarateOptionType {
    ackRateSameAsData,
    ackRateLowest,
};


inline
AckDatarateOptionType ReadAckDatarateOption(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const InterfaceIdType& interfaceId)
{
    if (theParameterDatabaseReader.ParameterExists(
        parameterNamePrefix + "ack-datarate-selection-type", nodeId, interfaceId)) {

        const string ackDatarateSelectionString =
            MakeLowerCaseString(
                theParameterDatabaseReader.ReadString(
                    parameterNamePrefix + "ack-datarate-selection-type", nodeId, interfaceId));

        if (ackDatarateSelectionString == "sameasdata") {
            return (ackRateSameAsData);
        }
        else if (ackDatarateSelectionString == "lowest") {
            return (ackRateLowest);
        }
        else {
            cerr << "Error: Unkown dot11-ack-datarate-selection-type = " << ackDatarateSelectionString << endl;
            exit(1);
        }//if//
    }//if//

    return (ackRateLowest);

}//ReadAckDatarateOption//



class AdaptiveRateController {
public:
    virtual ~AdaptiveRateController() { }

    virtual void SetHighThroughputModeForLink(const MacAddressType& macAddress) = 0;

    virtual unsigned int GetMinChannelBandwidthMhz() const = 0;
    virtual unsigned int GetMaxChannelBandwidthMhz() const = 0;

    virtual ModulationAndCodingSchemesType GetLowestModulationAndCoding() const = 0;

    virtual void GetDataRateInfoForDataFrameToStation(
        const MacAddressType& macAddress,
        TransmissionParametersType& txParameters) const = 0;

    virtual void GetDataRateInfoForAckFrame(
        const MacAddressType& macAddress,
        const TransmissionParametersType& receivedFrameTxParameters,
        TransmissionParametersType& ackTxParameters) const = 0;

    virtual void GetDataRateInfoForManagementFrameToStation(
        const MacAddressType& macAddress,
        TransmissionParametersType& txParameters) const = 0;

    virtual void GetDataRateInfoForBeaconFrame(
        TransmissionParametersType& txParameters) const = 0;

    virtual void GetDataRateInfoForDatarateSpecifiedFrame(
        const DatarateBitsPerSecType datarateBps,
        TransmissionParametersType& txParameters) const = 0;
    
    void SetModulationAndCondingMapForDatarateSpecifiedFrame(
        const map<ModulationAndCodingSchemeKey, ModulationAndCodingSchemesType>& initModulrationAndCodingSchemeMap) {
        modulrationAndCodingSchemeMap = initModulrationAndCodingSchemeMap;
    }

    virtual void NotifyAckReceived(const MacAddressType& macAddress) { }
    virtual void NotifyAckFailed(const MacAddressType& macAddress) { }

    virtual void ReceiveIncomingFrameSinrValue(
        const MacAddressType& sourceMacAddress,
        const double& measuredSinrValue) { }

protected:
    map<ModulationAndCodingSchemeKey, ModulationAndCodingSchemesType> modulrationAndCodingSchemeMap;

};//AdaptiveRateController//



//--------------------------------------------------------------------------------------------------


class StaticRateController : public AdaptiveRateController {
public:
    StaticRateController(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const unsigned int baseChannelBandwidthMhz);

    virtual ~StaticRateController() { }

    virtual void SetHighThroughputModeForLink(const MacAddressType& macAddress) override
        { assert(false && "TBD"); }

    virtual ModulationAndCodingSchemesType GetLowestModulationAndCoding() const override {
        return (lowestModulationAndCoding);
    }

    virtual unsigned int GetMinChannelBandwidthMhz() const override
        { return (minChannelBandwidthMhz); }

    virtual unsigned int GetMaxChannelBandwidthMhz() const override
        { return (maxChannelBandwidthMhz); }

    virtual void GetDataRateInfoForDataFrameToStation(
        const MacAddressType& macAddress,
        TransmissionParametersType& txParameters) const override;

    virtual void GetDataRateInfoForAckFrame(
        const MacAddressType& macAddress,
        const TransmissionParametersType& receivedFrameTxParameters,
        TransmissionParametersType& ackTxParameters) const override;

    virtual void GetDataRateInfoForManagementFrameToStation(
        const MacAddressType& macAddress,
        TransmissionParametersType& txParameters) const override;

    virtual void GetDataRateInfoForBeaconFrame(
        TransmissionParametersType& txParameters) const override;

    virtual void GetDataRateInfoForDatarateSpecifiedFrame(
        const DatarateBitsPerSecType datarateBps,
        TransmissionParametersType& txParameters) const override;

private:
    unsigned int minChannelBandwidthMhz;
    unsigned int maxChannelBandwidthMhz;

    bool highThroughputModeIsOn;

    AckDatarateOptionType ackDatarateOption;

    ModulationAndCodingSchemesType lowestModulationAndCoding;

    ModulationAndCodingSchemesType defaultModulationAndCoding;
    ModulationAndCodingSchemesType modAndCodingForManagementFrames;
    ModulationAndCodingSchemesType modAndCodingForBroadcast;

    bool perLinkDatarate;

    map<NodeIdType, ModulationAndCodingSchemesType> datarateTable;

    void ParseDatarateTable(
        const string& datarateTableString, bool& success);

};//StaticRateController//




inline
StaticRateController::StaticRateController(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const InterfaceIdType& interfaceId,
    const unsigned int baseChannelBandwidthMhz)
    :
    perLinkDatarate(false),
    minChannelBandwidthMhz(baseChannelBandwidthMhz),
    maxChannelBandwidthMhz(baseChannelBandwidthMhz)
{
    using std::min;

    ackDatarateOption = ReadAckDatarateOption(theParameterDatabaseReader, nodeId, interfaceId);

    if (theParameterDatabaseReader.ParameterExists("dot11-max-channel-bandwidth-mhz", nodeId, interfaceId)) {
        maxChannelBandwidthMhz =
           theParameterDatabaseReader.ReadNonNegativeInt("dot11-max-channel-bandwidth-mhz", nodeId, interfaceId);
    }//if//


    highThroughputModeIsOn = false;

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "enable-high-throughput-mode"),nodeId, interfaceId)) {

        highThroughputModeIsOn =
            theParameterDatabaseReader.ReadBool(
                (parameterNamePrefix + "enable-high-throughput-mode"), nodeId, interfaceId);
    }//if//


    //default
    defaultModulationAndCoding =
        ConvertNameToModulationAndCodingScheme(
            theParameterDatabaseReader.ReadString("dot11-modulation-and-coding", nodeId, interfaceId),
            "dot11-modulation-and-coding");

    //management
    if (theParameterDatabaseReader.ParameterExists(
        "dot11-modulation-and-coding-for-management-frames", nodeId, interfaceId)) {

        modAndCodingForManagementFrames =
            ConvertNameToModulationAndCodingScheme(
                theParameterDatabaseReader.ReadString(
                    "dot11-modulation-and-coding-for-management-frames", nodeId, interfaceId),
                    "dot11-modulation-and-coding-for-management-frames");
    }
    else {
        //default datarate
        modAndCodingForManagementFrames = defaultModulationAndCoding;
    }//if//

    //broadcast
    if (theParameterDatabaseReader.ParameterExists(
        "dot11-modulation-and-coding-for-broadcast", nodeId, interfaceId)) {

        modAndCodingForBroadcast =
            ConvertNameToModulationAndCodingScheme(
                theParameterDatabaseReader.ReadString(
                    "dot11-modulation-and-coding-for-broadcast", nodeId, interfaceId),
                "dot11-modulation-and-coding-for-broadcast");
    }
    else {
        //default datarate
        modAndCodingForBroadcast = defaultModulationAndCoding;

    }//if//

    //lowest

    lowestModulationAndCoding =
        min(defaultModulationAndCoding,
            min(modAndCodingForManagementFrames, modAndCodingForBroadcast));

    //per link datarate
    if (theParameterDatabaseReader.ParameterExists("dot11-modulation-and-coding-table", nodeId, interfaceId)) {

        const string datarateTableString =
            theParameterDatabaseReader.ReadString("dot11-modulation-and-coding-table", nodeId, interfaceId);

        bool success = false;
        ParseDatarateTable(datarateTableString, success);

        if (!success) {
            cerr << "Invalid value: \"dot11-modulation-and-coding-table\": " << datarateTableString << endl;
            exit(1);
         }//if//

        perLinkDatarate = true;

    }//if//

}//StaticRateController//


inline
void StaticRateController::ParseDatarateTable(const string& datarateTableString, bool& success)
{
    using std::min;

    istringstream datarateTableStringStream(datarateTableString);

    while (!datarateTableStringStream.eof()) {

        string aString;
        datarateTableStringStream >> aString;

        const size_t endOfTargetNodesTerminatorPos = aString.find_first_of(':');

        if (endOfTargetNodesTerminatorPos == string::npos) {
            success = false;
            return;
        }//if//

        const string targetNodesString =
            aString.substr(0, endOfTargetNodesTerminatorPos);

        const string targetModAndCodingString =
            aString.substr(endOfTargetNodesTerminatorPos + 1, aString.length() - endOfTargetNodesTerminatorPos);


        ModulationAndCodingSchemesType targetModAndCoding =
            ConvertNameToModulationAndCodingScheme(
                targetModAndCodingString, "dot11-modulation-and-coding-table");

        size_t currentPosition = 0;
        set<NodeIdType> targetNodeIds;
        while (currentPosition < targetNodesString.length()) {

            size_t endOfSubfieldTerminatorPos = targetNodesString.find_first_of(",-", currentPosition);

            if (endOfSubfieldTerminatorPos == string::npos) {
                endOfSubfieldTerminatorPos = targetNodesString.length();
            }

            if ((endOfSubfieldTerminatorPos == targetNodesString.length())
                || (targetNodesString.at(endOfSubfieldTerminatorPos) == ',')) {

                const NodeIdType extractedNumber =
                    atoi(targetNodesString.substr(currentPosition, (endOfSubfieldTerminatorPos - currentPosition)).c_str());

                targetNodeIds.insert(extractedNumber);
            }
            else if (targetNodesString.at(endOfSubfieldTerminatorPos) == '-') {

                //This is a range.

                const size_t dashPosition = endOfSubfieldTerminatorPos;

                endOfSubfieldTerminatorPos = targetNodesString.find(',', dashPosition);

                if (endOfSubfieldTerminatorPos == string::npos) {
                    endOfSubfieldTerminatorPos = targetNodesString.length();
                }

                const NodeIdType lowerBoundNumber =
                    atoi(targetNodesString.substr(currentPosition, (dashPosition - currentPosition)).c_str());

                const NodeIdType upperBoundNumber =
                    atoi(targetNodesString.substr((dashPosition + 1), (endOfSubfieldTerminatorPos - dashPosition - 1)).c_str());

                if (lowerBoundNumber > upperBoundNumber) {
                    success = false;
                    return;
                }//if//

                for(NodeIdType nodeId = lowerBoundNumber; (nodeId <= upperBoundNumber); nodeId++) {
                    targetNodeIds.insert(nodeId);
                }//for//

            }//if//

            currentPosition = endOfSubfieldTerminatorPos + 1;

        }//while//


        //set datarate per node
        for(set<NodeIdType>::const_iterator iter = targetNodeIds.begin();
            iter != targetNodeIds.end(); ++iter) {

            const NodeIdType targetNodeId = *iter;

            (*this).datarateTable[targetNodeId] = targetModAndCoding;

            (*this).lowestModulationAndCoding = min(lowestModulationAndCoding, targetModAndCoding);

        }//for//

    }//while//

    success = true;

}//ParseDatarateTable//


inline
void StaticRateController::GetDataRateInfoForDataFrameToStation(
    const MacAddressType& macAddress,
    TransmissionParametersType& txParameters) const
{
    if (macAddress.IsABroadcastOrAMulticastAddress()) {

        //broadcast
        txParameters.modulationAndCodingScheme = modAndCodingForBroadcast;
    }
    else {
        //unicast
        if (perLinkDatarate) {

            const NodeIdType targetNodeId = CalcNodeId(macAddress.ConvertToGenericMacAddress());

            map<NodeIdType,ModulationAndCodingSchemesType>::const_iterator iter =
                datarateTable.find(targetNodeId);

            if (iter == datarateTable.end()) {
                txParameters.modulationAndCodingScheme = defaultModulationAndCoding;
            }
            else {
                txParameters.modulationAndCodingScheme = iter->second;
            }//if//

        }
        else {

            txParameters.modulationAndCodingScheme = defaultModulationAndCoding;

        }//if//
    }//if//

    txParameters.channelBandwidthMhz = minChannelBandwidthMhz;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = highThroughputModeIsOn;

}//GetDataRateInformationForStation//



inline
void StaticRateController::GetDataRateInfoForAckFrame(
    const MacAddressType& macAddress,
    const TransmissionParametersType& receivedFrameTxParameters,
    TransmissionParametersType& ackTxParameters) const
{
    ackTxParameters.modulationAndCodingScheme = modAndCodingForManagementFrames;
    if (ackDatarateOption == ackRateSameAsData) {
        ackTxParameters.modulationAndCodingScheme =
            receivedFrameTxParameters.modulationAndCodingScheme;
    }//if//

    ackTxParameters.channelBandwidthMhz = minChannelBandwidthMhz;
    ackTxParameters.numberSpatialStreams = 1;
    ackTxParameters.isHighThroughputFrame = highThroughputModeIsOn;

}//GetDataRateInfoForAckFrame//


inline
void StaticRateController::GetDataRateInfoForManagementFrameToStation(
    const MacAddressType& macAddress,
    TransmissionParametersType& txParameters) const
{
    txParameters.modulationAndCodingScheme = modAndCodingForManagementFrames;
    txParameters.channelBandwidthMhz = minChannelBandwidthMhz;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = highThroughputModeIsOn;

}//GetDataRateInfoForManagementFrameToStation//


inline
void StaticRateController::GetDataRateInfoForBeaconFrame(
    TransmissionParametersType& txParameters) const
{
    txParameters.modulationAndCodingScheme = modAndCodingForManagementFrames;
    txParameters.channelBandwidthMhz = minChannelBandwidthMhz;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = false;
}


inline
void StaticRateController::GetDataRateInfoForDatarateSpecifiedFrame(
    const DatarateBitsPerSecType datarateBps,
    TransmissionParametersType& txParameters) const
{
    const ModulationAndCodingSchemeKey mcsKey(
        datarateBps, minChannelBandwidthMhz, highThroughputModeIsOn);

    typedef map<ModulationAndCodingSchemeKey, ModulationAndCodingSchemesType>::const_iterator IterType;

    IterType iter = modulrationAndCodingSchemeMap.find(mcsKey);

    if (iter == modulrationAndCodingSchemeMap.end()) {
        cerr << "Error: No available Modulation and Coding Scheme for " << endl
             << "Datarate:" << datarateBps << ", Bandwidth:" << minChannelBandwidthMhz << ", highThroughputModeIsOn:" << highThroughputModeIsOn << endl;
        exit(1);
    }
    
    txParameters.modulationAndCodingScheme = (*iter).second;
    txParameters.channelBandwidthMhz = minChannelBandwidthMhz;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = highThroughputModeIsOn;
}//GetDataRateInfoForDatarateSpecifiedFrame//



//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------


// Abbr: Automatic Rate Fallback = ARF

class AdaptiveRateControllerWithArf : public AdaptiveRateController {
public:
    AdaptiveRateControllerWithArf(
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const unsigned int baseChannelBandwidthMhz);

    virtual void SetHighThroughputModeForLink(const MacAddressType& macAddress) override
        { assert(false && "TBD"); }

    virtual ModulationAndCodingSchemesType GetLowestModulationAndCoding() const override {
        return (lowestModulationAndCoding);
    }

    virtual unsigned int GetMinChannelBandwidthMhz() const override
        { return (minChannelBandwidthMhz); }

    virtual unsigned int GetMaxChannelBandwidthMhz() const override
        { return (maxChannelBandwidthMhz); }

    virtual void GetDataRateInfoForDataFrameToStation(
        const MacAddressType& macAddress,
        TransmissionParametersType& txParameters) const override;

    virtual void GetDataRateInfoForAckFrame(
        const MacAddressType& macAddress,
        const TransmissionParametersType& receivedFrameTxParameters,
        TransmissionParametersType& ackTxParameters) const override;

    virtual void GetDataRateInfoForManagementFrameToStation(
        const MacAddressType& macAddress,
        TransmissionParametersType& txParameters) const override;

    virtual void GetDataRateInfoForBeaconFrame(
        TransmissionParametersType& txParameters) const override;

    virtual void GetDataRateInfoForDatarateSpecifiedFrame(
        const DatarateBitsPerSecType datarateBps,
        TransmissionParametersType& txParameters) const override;

    virtual void NotifyAckReceived(const MacAddressType& macAddress) override;
    virtual void NotifyAckFailed(const MacAddressType& macAddress) override;

    virtual ~AdaptiveRateControllerWithArf() { }

private:
    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;

    unsigned int minChannelBandwidthMhz;
    unsigned int maxChannelBandwidthMhz;

    bool highThroughputModeIsOn;

    ModulationAndCodingSchemesType lowestModulationAndCoding;
    ModulationAndCodingSchemesType modAndCodingForManagementFrames;
    ModulationAndCodingSchemesType modAndCodingForBroadcast;
    vector<ModulationAndCodingSchemesType> modulationAndCodingSchemes;

    AckDatarateOptionType ackDatarateOption;

    TimeType rateUpgradeTimerDuration;
    int ackInSuccessCountThreshold;
    int ackInFailureCountThreshold;
    int ackInFailureCountThresholdOfNewRateState;

    class AckCounterEntry : public enable_shared_from_this<AckCounterEntry> {
    public:
        AckCounterEntry(
            const AdaptiveRateControllerWithArf* initArfPtr,
            const unsigned int initModAndCodingIndex)
            :
            arfPtr(initArfPtr),
            consecutiveUnicastPacketsSentStat(0),
            consecutiveDroppedPacketsStat(0),
            currentModulationAndCodingIndex(initModAndCodingIndex),
            isStableState(true),
            highThroughputModeIsEnabled(false)
        {}

        void IncrementUnicastSentCount();
        void IncrementUnicastDroppedCount();

        bool CanIncreaseDatarate() const;
        bool ShouldDecreaseDatarate() const;

        void IncreaseDatarate();
        void DecreaseDatarate();

        void CancelRateUpgradeEvent();
        void ScheduleRateUpgradeEvent();
        void ExecuteRateUpgradeEvent();

        ModulationAndCodingSchemesType GetCurrentModulationAndCoding() const
            { return (arfPtr->modulationAndCodingSchemes.at(currentModulationAndCodingIndex)); }

    private:
        const AdaptiveRateControllerWithArf* arfPtr;

        int consecutiveUnicastPacketsSentStat;
        int consecutiveDroppedPacketsStat;
        unsigned int currentModulationAndCodingIndex;
        bool isStableState;
        EventRescheduleTicket rateUpgradeEventTicket;
        bool highThroughputModeIsEnabled;

    };//AckCounterEntry//

    mutable map<MacAddressType, shared_ptr<AckCounterEntry> > macAddressAndDatarateMap;

    class RateUpgradeEvent : public SimulationEvent {
    public:
        RateUpgradeEvent(const shared_ptr<AckCounterEntry>& initAckCounterEnvtryPtr)
            : ackCounterEnvtryPtr(initAckCounterEnvtryPtr) {}

        void ExecuteEvent() {
            ackCounterEnvtryPtr->ExecuteRateUpgradeEvent();
        }
    private:
        const shared_ptr<AckCounterEntry> ackCounterEnvtryPtr;
    };//RateUpgradeEvent//

    AckCounterEntry& GetAckCounterEntry(const MacAddressType& macAddress) const;

};//AdaptiveRateControllerWithArf//


inline
AdaptiveRateControllerWithArf::AdaptiveRateControllerWithArf(
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const InterfaceIdType& interfaceId,
    const unsigned int baseChannelBandwidthMhz)
    :
    simEngineInterfacePtr(initSimulationEngineInterfacePtr),
    minChannelBandwidthMhz(baseChannelBandwidthMhz),
    maxChannelBandwidthMhz(baseChannelBandwidthMhz)
{
    ackDatarateOption = ReadAckDatarateOption(theParameterDatabaseReader, nodeId, interfaceId);

    rateUpgradeTimerDuration =
        theParameterDatabaseReader.ReadTime("dot11-arf-timer-duration", nodeId, interfaceId);

    ackInSuccessCountThreshold =
        theParameterDatabaseReader.ReadInt("dot11-arf-ack-in-success-count", nodeId, interfaceId);

    ackInFailureCountThreshold =
        theParameterDatabaseReader.ReadInt("dot11-arf-ack-in-failure-count", nodeId, interfaceId);

    ackInFailureCountThresholdOfNewRateState =
        theParameterDatabaseReader.ReadInt("dot11-arf-ack-in-failure-count-of-new-rate-state", nodeId, interfaceId);

    lowestModulationAndCoding = minModulationAndCodingScheme;

    highThroughputModeIsOn = false;

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "enable-high-throughput-mode"),nodeId, interfaceId)) {

        highThroughputModeIsOn =
            theParameterDatabaseReader.ReadBool(
                (parameterNamePrefix + "enable-high-throughput-mode"), nodeId, interfaceId);
    }//if//

    //unicast
    const string modAndCodingListString =
        theParameterDatabaseReader.ReadString("dot11-modulation-and-coding-list", nodeId, interfaceId);

    vector<string> modAndCodingNameVector;
    bool isSuccess;
    ConvertAStringIntoAVectorOfStrings(modAndCodingListString, isSuccess, modAndCodingNameVector);

    if ((!isSuccess) || (modAndCodingNameVector.empty())) {
        cerr << "Error in \"dot11-modulation-and-coding-list\" parameter was invalid." << endl;
        exit(1);
    }//if//

    modulationAndCodingSchemes.resize(modAndCodingNameVector.size());

    for(unsigned int i = 0; (i < modAndCodingNameVector.size()); i++) {
        bool success;
        GetModulationAndCodingSchemeFromName(
            modAndCodingNameVector[i],
            success,
            modulationAndCodingSchemes[i]);

        if (!success) {
            cerr << "Error in \"dot11-modulation-and-coding-list\" value: "
                 << modAndCodingNameVector[i] << " was invalid." << endl;
            exit(1);
        }//if//

        if ((i != 0) && (modulationAndCodingSchemes[i-1] >= modulationAndCodingSchemes[i])) {
            cerr << "Error in \"dot11-modulation-and-coding-list\". Schemes must be in "
                 << "increasing speed order." << endl;
            exit(1);
        }//if//
    }//for//

    //management
    if (theParameterDatabaseReader.ParameterExists(
        "dot11-modulation-and-coding-for-management-frames", nodeId, interfaceId)) {

        modAndCodingForManagementFrames =
            ConvertNameToModulationAndCodingScheme(
                theParameterDatabaseReader.ReadString(
                    "dot11-modulation-and-coding-for-management-frames", nodeId, interfaceId),
                    "dot11-modulation-and-coding-for-management-frames");
    }
    else {
        modAndCodingForManagementFrames = lowestModulationAndCoding;
    }//if//

    //broadcast
    if (theParameterDatabaseReader.ParameterExists(
        "dot11-modulation-and-coding-for-broadcast", nodeId, interfaceId)) {

        modAndCodingForBroadcast =
            ConvertNameToModulationAndCodingScheme(
                theParameterDatabaseReader.ReadString(
                    "dot11-modulation-and-coding-for-broadcast", nodeId, interfaceId),
                "dot11-modulation-and-coding-for-broadcast");
    }
    else {
        modAndCodingForBroadcast = lowestModulationAndCoding;
    }//if//

}//AdaptiveRateControllerWithArf//



inline
void AdaptiveRateControllerWithArf::GetDataRateInfoForDataFrameToStation(
        const MacAddressType& macAddress,
        TransmissionParametersType& txParameters) const
{
    if (macAddress.IsABroadcastOrAMulticastAddress()) {
        txParameters.modulationAndCodingScheme = modAndCodingForBroadcast;
    }
    else {
        txParameters.modulationAndCodingScheme =
            GetAckCounterEntry(macAddress).GetCurrentModulationAndCoding();
    }//if//

    txParameters.channelBandwidthMhz = minChannelBandwidthMhz;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = highThroughputModeIsOn;

}//GetDataRateInformationForStation//


inline
void AdaptiveRateControllerWithArf::GetDataRateInfoForAckFrame(
    const MacAddressType& macAddress,
    const TransmissionParametersType& receivedFrameTxParameters,
    TransmissionParametersType& ackTxParameters) const
{
    ackTxParameters.modulationAndCodingScheme = modAndCodingForManagementFrames;
    if (ackDatarateOption == ackRateSameAsData) {
        ackTxParameters.modulationAndCodingScheme =
            receivedFrameTxParameters.modulationAndCodingScheme;
    }//if//

    ackTxParameters.channelBandwidthMhz = minChannelBandwidthMhz;
    ackTxParameters.numberSpatialStreams = 1;
    ackTxParameters.isHighThroughputFrame = highThroughputModeIsOn;

}//GetDataRateInfoForAckFrame//



inline
void AdaptiveRateControllerWithArf::GetDataRateInfoForManagementFrameToStation(
    const MacAddressType& macAddress,
    TransmissionParametersType& txParameters) const
{
    txParameters.modulationAndCodingScheme = modAndCodingForManagementFrames;
    txParameters.channelBandwidthMhz = minChannelBandwidthMhz;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = highThroughputModeIsOn;

}//GetDataRateInfoForManagementFrameToStation//


inline
void AdaptiveRateControllerWithArf::GetDataRateInfoForBeaconFrame(
    TransmissionParametersType& txParameters) const
{
    txParameters.modulationAndCodingScheme = modAndCodingForManagementFrames;
    txParameters.channelBandwidthMhz = minChannelBandwidthMhz;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = false;
}


inline
void AdaptiveRateControllerWithArf::GetDataRateInfoForDatarateSpecifiedFrame(
    const DatarateBitsPerSecType datarateBps,
    TransmissionParametersType& txParameters) const
{
    const ModulationAndCodingSchemeKey mcsKey(
        datarateBps, minChannelBandwidthMhz, highThroughputModeIsOn);

    typedef map<ModulationAndCodingSchemeKey, ModulationAndCodingSchemesType>::const_iterator IterType;

    IterType iter = modulrationAndCodingSchemeMap.find(mcsKey);

    if (iter == modulrationAndCodingSchemeMap.end()) {
        cerr << "Error: No available Modulation and Coding Scheme for " << endl
             << "Datarate:" << datarateBps << ", Bandwidth:" << minChannelBandwidthMhz << ", highThroughputModeIsOn:" << highThroughputModeIsOn << endl;
        exit(1);
    }
    
    txParameters.modulationAndCodingScheme = (*iter).second;
    txParameters.channelBandwidthMhz = minChannelBandwidthMhz;
    txParameters.numberSpatialStreams = 1;
    txParameters.isHighThroughputFrame = highThroughputModeIsOn;
}//GetDataRateInfoForDatarateSpecifiedFrame//


inline
AdaptiveRateControllerWithArf::AckCounterEntry&
AdaptiveRateControllerWithArf::GetAckCounterEntry(const MacAddressType& macAddress) const
{
    typedef map<MacAddressType, shared_ptr<AckCounterEntry> >::const_iterator IterType;

    IterType iter = macAddressAndDatarateMap.find(macAddress);

    if (iter != macAddressAndDatarateMap.end()) {
        return *(*iter).second;
    }//if//

    const unsigned int highestModulationIndex = static_cast<unsigned int>(modulationAndCodingSchemes.size() - 1);

    shared_ptr<AckCounterEntry> newEntryPtr(new AckCounterEntry(this, highestModulationIndex));

    macAddressAndDatarateMap[macAddress] = newEntryPtr;

    return *newEntryPtr;

}//GetAckCounterEntryPtr//


inline
void AdaptiveRateControllerWithArf::NotifyAckReceived(const MacAddressType& macAddress)
{
    AckCounterEntry& ackCounterEntry = (*this).GetAckCounterEntry(macAddress);

    ackCounterEntry.IncrementUnicastSentCount();

    if (ackCounterEntry.CanIncreaseDatarate()) {
        ackCounterEntry.IncreaseDatarate();
        ackCounterEntry.CancelRateUpgradeEvent();
    }//if//

}//NotifyAckReceived//


inline
void AdaptiveRateControllerWithArf::NotifyAckFailed(const MacAddressType& macAddress)
{
    AckCounterEntry& ackCounterEntry = (*this).GetAckCounterEntry(macAddress);

    ackCounterEntry.IncrementUnicastDroppedCount();

    if (ackCounterEntry.ShouldDecreaseDatarate()) {
        ackCounterEntry.DecreaseDatarate();
        ackCounterEntry.ScheduleRateUpgradeEvent();
    }//if//

}//NotifyAckFailed//


inline
void AdaptiveRateControllerWithArf::AckCounterEntry::IncrementUnicastSentCount()
{
    consecutiveDroppedPacketsStat = 0;

    consecutiveUnicastPacketsSentStat = std::min(
        consecutiveUnicastPacketsSentStat + 1,
        arfPtr->ackInSuccessCountThreshold);

    isStableState = true;

}//IncrementUnicastSentCount//


inline
void AdaptiveRateControllerWithArf::AckCounterEntry::IncrementUnicastDroppedCount()
{
    consecutiveUnicastPacketsSentStat = 0;

    const int maxAckInFailureCountThreshold = std::max(
        arfPtr->ackInFailureCountThreshold,
        arfPtr->ackInFailureCountThresholdOfNewRateState);

    consecutiveDroppedPacketsStat = std::min(
        consecutiveDroppedPacketsStat + 1,
        maxAckInFailureCountThreshold);

}//IncrementUnicastDroppedCount//


inline
bool AdaptiveRateControllerWithArf::AckCounterEntry::CanIncreaseDatarate() const
{
    //The number of success is larger than threshold//
    return (consecutiveUnicastPacketsSentStat >= arfPtr->ackInSuccessCountThreshold);

}//CanIncreaseDatarate//


inline
bool AdaptiveRateControllerWithArf::AckCounterEntry::ShouldDecreaseDatarate() const
{
    //The number of failure is larger than threshold, or starting connection of new rate is failure//
    return
        ((currentModulationAndCodingIndex != 0) &&
         ((consecutiveDroppedPacketsStat >= arfPtr->ackInFailureCountThreshold) ||
         ((!isStableState) &&
          (consecutiveDroppedPacketsStat >= arfPtr->ackInFailureCountThresholdOfNewRateState))));

}//ShouldDecreaseDatarate//


inline
void AdaptiveRateControllerWithArf::AckCounterEntry::IncreaseDatarate()
{
    if (currentModulationAndCodingIndex != (arfPtr->modulationAndCodingSchemes.size() - 1)) {
        currentModulationAndCodingIndex++;
        isStableState = false;
    }//if//

    consecutiveDroppedPacketsStat = 0;
    consecutiveUnicastPacketsSentStat = 0;

}//IncreaseDatarate//


inline
void AdaptiveRateControllerWithArf::AckCounterEntry::DecreaseDatarate()
{
    if (currentModulationAndCodingIndex != 0) {
        currentModulationAndCodingIndex--;
    }//if//

    consecutiveDroppedPacketsStat = 0;
    consecutiveUnicastPacketsSentStat = 0;

}//DecreaseDatarate//


inline
void AdaptiveRateControllerWithArf::AckCounterEntry::CancelRateUpgradeEvent()
{
    if (!rateUpgradeEventTicket.IsNull()) {
        arfPtr->simEngineInterfacePtr->CancelEvent(rateUpgradeEventTicket);
    }//if//

}//CancelRateUpgradeEvent//


inline
void AdaptiveRateControllerWithArf::AckCounterEntry::ScheduleRateUpgradeEvent()
{

    const TimeType rateUpgradeTime =
        (*arfPtr).simEngineInterfacePtr->CurrentTime() + arfPtr->rateUpgradeTimerDuration;

    if (rateUpgradeEventTicket.IsNull()) {
        const shared_ptr<RateUpgradeEvent> rateUpgradeEventPtr(
            new RateUpgradeEvent((*this).shared_from_this()));

        (*arfPtr).simEngineInterfacePtr->ScheduleEvent(
            rateUpgradeEventPtr,
            rateUpgradeTime,
            rateUpgradeEventTicket);
    }
    else {
        (*arfPtr).simEngineInterfacePtr->RescheduleEvent(
            rateUpgradeEventTicket,
            rateUpgradeTime);
    }//if//

}//ScheduleRateUpgradeEvent//


inline
void AdaptiveRateControllerWithArf::AckCounterEntry::ExecuteRateUpgradeEvent()
{
    (*this).IncreaseDatarate();
    rateUpgradeEventTicket.Clear();

}//AdaptiveRateControllerWithArf//

//--------------------------------------------------------------------------------------------------

inline
shared_ptr<AdaptiveRateController> CreateAdaptiveRateController(
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const InterfaceIdType& interfaceId,
    const unsigned int baseChannelBandwidthMhz)
{
    if (!theParameterDatabaseReader.ParameterExists(
            "dot11-adaptive-rate-control-type", nodeId, interfaceId)) {

        return (shared_ptr<AdaptiveRateController>(
            new StaticRateController(
                theParameterDatabaseReader,
                nodeId,
                interfaceId,
                baseChannelBandwidthMhz)));
    }
    else {
        string adaptiveRateControlType =
            theParameterDatabaseReader.ReadString(
                "dot11-adaptive-rate-control-type", nodeId, interfaceId);

        ConvertStringToLowerCase(adaptiveRateControlType);

        if (adaptiveRateControlType == "arf") {
            return (shared_ptr<AdaptiveRateController>(
                new AdaptiveRateControllerWithArf(
                    simulationEngineInterfacePtr,
                    theParameterDatabaseReader,
                    nodeId,
                    interfaceId,
                    baseChannelBandwidthMhz)));
        }
        else if ((adaptiveRateControlType == "static") || (adaptiveRateControlType == "fixed"/*will be deleted*/)) {
            return (shared_ptr<AdaptiveRateController>(
                new StaticRateController(
                    theParameterDatabaseReader,
                    nodeId,
                    interfaceId,
                    baseChannelBandwidthMhz)));
        }
        else {
            cerr << "Error: dot11-adaptive-rate-control-type parameter is not valid: "
                 << adaptiveRateControlType << endl;
            exit(1);
            return shared_ptr<AdaptiveRateController>();
        }//if//
    }//if//

}//CreateAdaptiveRateController//


}//namespace

#endif
