// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef T109_PHY_H
#define T109_PHY_H

#include <sstream>

#include "scensim_engine.h"
#include "scensim_netsim.h"
#include "scensim_prop.h"
#include "scensim_bercurves.h"

#include "dot11_phy.h"

#include "t109_tracedefs.h"

namespace T109 {

using std::shared_ptr;
using std::unique_ptr;
using std::move;
using std::cerr;
using std::endl;
using std::make_pair;
using std::list;
using std::ostringstream;
using std::deque;
using std::pair;
using std::cout;
using std::map;
using std::array;

using ScenSim::TimeType;
using ScenSim::INFINITE_TIME;
using ScenSim::ZERO_TIME;
using ScenSim::SECOND;
using ScenSim::ConvertTimeToStringSecs;
using ScenSim::SimulationEngineInterface;
using ScenSim::SimulationEvent;
using ScenSim::SimplePropagationModelForNode;
using ScenSim::BitOrBlockErrorRateCurveDatabase;
using ScenSim::BitErrorRateCurve;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeIdType;
using ScenSim::InterfaceIdType;
using ScenSim::Packet;
using ScenSim::PacketIdType;
using ScenSim::IntegralPowerType;
using ScenSim::CounterStatistic;
using ScenSim::RealStatistic;
using ScenSim::RandomNumberGenerator;
using ScenSim::RandomNumberGeneratorSeedType;
using ScenSim::HashInputsToMakeSeed;
using ScenSim::CalculateThermalNoisePowerWatts;
using ScenSim::ConvertToNonDb;
using ScenSim::ConvertToDb;
using ScenSim::TracePhy;
using ScenSim::TracePhyInterference;
using ScenSim::ConvertDoubleSecsToTime;
using ScenSim::NetworkLayer;
using ScenSim::SimpleMacPacketHandler;
using ScenSim::NetworkAddress;
using ScenSim::PacketPriorityType;
using ScenSim::GenericMacAddressType;
using ScenSim::ObjectMobilityModel;
using ScenSim::CounterStatistic;
using ScenSim::MacLayer;
using ScenSim::EventRescheduleTicket;
using ScenSim::ConvertStringToLowerCase;
using ScenSim::ConvertStringToInt;
using ScenSim::ConvertToString;
using ScenSim::ConvertStringToLowerCase;
using ScenSim::ConvertTimeToStringSecs;
using ScenSim::ZERO_TIME;
using ScenSim::INFINITE_TIME;
using ScenSim::NANO_SECOND;
using ScenSim::SECOND;
using ScenSim::MILLI_SECOND;
using ScenSim::ApplicationIdType;
using ScenSim::ObjectMobilityPosition;
using ScenSim::ExtrinsicPacketInformation;
using ScenSim::ExtrinsicPacketInfoIdType;
using ScenSim::SixByteMacAddressType;
using ScenSim::BAD_SIZE_T;
using ScenSim::ANY_NODEID;

using ScenSim::MakeLowerCaseString;
using ScenSim::TokenizeToTrimmedLowerString;
using ScenSim::TraceMac;
using ScenSim::ConvertStringToTime;
using ScenSim::ConvertTimeToDoubleSecs;
using ScenSim::MICRO_SECOND;
using ScenSim::Application;
using ScenSim::TraceApplication;
using ScenSim::ApplicationSendTraceRecord;
using ScenSim::ApplicationReceiveTraceRecord;
using ScenSim::APPLICATION_SEND_TRACE_RECORD_BYTES;
using ScenSim::APPLICATION_RECEIVE_TRACE_RECORD_BYTES;

class T109MacInterfaceForPhy;
class T109Mac;
class T109Phy;

typedef unsigned int DatarateBitsPerSecType;

//--------------------------------------------------------------------------------------------------

enum ModulationAndCodingSchemesType {
    McsBpsk1Over2,
    McsBpsk3Over4,
    McsQpsk1Over2,
    McsQpsk3Over4,
    Mcs16Qam1Over2,
    Mcs16Qam3Over4,
    McsInvalid,
};

const unsigned int NumberModAndCodingSchemes = 6;

const double t109ChannelBandwidthMhz = 10.;
const unsigned int numOfdmSubcarriers = 64;
const unsigned int numOfdmDataSubcarriers = 48;
const unsigned int numUsedOfdmSubcarriers = 52;
const double normalGuardIntervalOfdmSymbolDurationFactor = 0.8;

inline
double CalcPilotSubcarriersPowerAdjustmentFactor()
{
    return (static_cast<double>(numOfdmDataSubcarriers) / numUsedOfdmSubcarriers);

}//CalcPilotSubcarriersPowerAdjustmentFactor//

inline
double CalcSignalPowerAdjustmentFactor()
{
    return
        (CalcPilotSubcarriersPowerAdjustmentFactor() *
         normalGuardIntervalOfdmSymbolDurationFactor);

}//CalcSignalPowerAdjustmentFactor//


// Factor adjusts "full bandwidth" noise power down to "used bandwidth" noise.

inline
double CalcThermalNoiseAdjustmentFactor()
{
    return (static_cast<double>(numUsedOfdmSubcarriers) / numOfdmSubcarriers);

}//CalcThermalNoiseAdjustmentFactor//

inline
string GetModulationAndCodingName(const ModulationAndCodingSchemesType modulationAndCoding)
{
    static const array<string, NumberModAndCodingSchemes> mcsName =
        {"BPSK_0.5", "BPSK_0.75", "QPSK_0.5", "QPSK_0.75", "16QAM_0.5", "16QAM_0.75" };
    return (mcsName.at(static_cast<size_t>(modulationAndCoding)));
}

inline
unsigned int GetBitsPerSymbolForModulation(const ModulationAndCodingSchemesType& modulationAndCoding)
{
    static const array<unsigned int, NumberModAndCodingSchemes> bitsPerSymbol =
        {1, 1, 2, 2, 4, 4};
    return (bitsPerSymbol.at(static_cast<size_t>(modulationAndCoding)));
}//GetBitsPerSymbolForModulation//

inline
double GetCodingRate(const ModulationAndCodingSchemesType& modulationAndCoding)
{
    static const array<double, NumberModAndCodingSchemes> codingRates =
        {0.5, 0.75, 0.5, 0.75, 0.5, 0.75};

    return (codingRates.at(static_cast<size_t>(modulationAndCoding)));
}//GetCodingRate//

inline
double GetFractionalBitsPerSymbol(const ModulationAndCodingSchemesType& modulationAndCoding)
{
    return (GetBitsPerSymbolForModulation(modulationAndCoding) * GetCodingRate(modulationAndCoding));
}//GetFractionalBitsPerSymbol//

inline
int CalcNumberOfBitsPerOfdmSymbol(const ModulationAndCodingSchemesType& modulationAndCodingScheme)
{
    return (
        static_cast<unsigned int>(
            numOfdmDataSubcarriers *
            GetFractionalBitsPerSymbol(modulationAndCodingScheme)));

}//GetNumberOfBitsPerOfdmSymbol//

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

class T109Phy {
public:
    static const string modelName;

    static const int fcsFieldLengthBytes = 4; // FCS
    static const int phyFrameDataPaddingBits = 16 + 6; // Service and Tail

    struct PropFrameType {
        DatarateBitsPerSecType datarateBitsPerSecond;
        ModulationAndCodingSchemesType modulationAndCodingScheme;

        unique_ptr<ScenSim::Packet> macFramePtr;

        PropFrameType()
            :
            datarateBitsPerSecond(0),
            modulationAndCodingScheme(McsInvalid)
        {}
    };

    typedef SimplePropagationModelForNode<PropFrameType>::IncomingSignal IncomingSignal;

    T109Phy(
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<SimplePropagationModelForNode<PropFrameType> >& propModelInterfacePtr,
        const shared_ptr<BitOrBlockErrorRateCurveDatabase>& berCurveDatabasePtr,
        T109Mac* macLayerPtr,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const RandomNumberGeneratorSeedType& nodeSeed);


    ~T109Phy() {
        propagationModelInterfacePtr->DisconnectThisInterface();
    }

    bool IsReceivingAFrame() const { return (phyState == PHY_RECEIVING); }

    bool IsTransmittingAFrame() const {
        return ((phyState == PHY_TX_STARTING) || (phyState == PHY_TRANSMITTING));
    }

    bool ChannelIsClear() const { return (!currentlySensingBusyMedium); }

    void TransmitFrame(
        unique_ptr<Packet>& packetPtr,
        const DatarateBitsPerSecType& datarateBitsPerSecond,
        const double& transmitPowerDbm,
        const TimeType& delayUntilAirborne);

    TimeType CalculateFrameTransmitDuration(
        const size_t frameLengthBytes,
        const DatarateBitsPerSecType& datarateBitsPerSecond,
        const ModulationAndCodingSchemesType& modulationAndCodingScheme) const;

    TimeType CalculateFrameTransmitDuration(
        const size_t frameLengthBytes,
        const DatarateBitsPerSecType& datarateBitsPerSecond) const;

    TimeType GetSlotDuration() const { return aSlotTimeDuration; }
    TimeType GetShortInterframeSpaceDuration() const { return aShortInterframeSpaceDuration; }
    TimeType GetRxTxTurnaroundTime() const { return aRxTxTurnaroundTime; }

    unsigned int GetChannelCount() const { return propagationModelInterfacePtr->GetChannelCount(); }
    unsigned int GetCurrentChannelNumber() const { return propagationModelInterfacePtr->GetCurrentChannelNumber(); }

    void SwitchToChannelNumber(const unsigned int channelNumber);

private:
    double GetRssiOfLastFrameDbm() const { return lastReceivedPacketRssiDbm; }

    unsigned int GetNumberOfReceivedFrames() const { return numberOfReceivedFrames; }
    unsigned int GetNumberOfFramesWithErrors() const { return numberOfFramesWithErrors; }
    unsigned int GetNumberOfSignalCaptures() const { return numberOfSignalCaptures; }

    TimeType GetTotalIdleChannelTime() const { return totalIdleChannelTime; }
    TimeType GetTotalBusyChannelTime() const { return totalBusyChannelTime; }
    TimeType GetTotalTransmissionTime() const { return totalTransmissionTime; }

    static const int SEED_HASH = 23788567;

    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;

    shared_ptr<SimplePropagationModelForNode<PropFrameType> > propagationModelInterfacePtr;

    shared_ptr<BitOrBlockErrorRateCurveDatabase> berCurveDatabasePtr;

    T109Mac* macLayerPtr;

    // If (unsynchronized) noise energy is over this threshold, then the station
    // is sensing a "busy medium"
    // Note: these variables should always be the same, but in different units.

    double energyDetectionPowerThresholdDbm;
    IntegralPowerType energyDetectionPowerThreshold;

    // If the "locked on" signal energy is over this threshold, then the station
    // is sensing a "busy medium" and will try to receive the frame.
    // Note in OFDM, the radio (as defined in the standard) can detect much lower energy
    // levels when it is locked onto the signal than when it is not (100X=20dB more sensitive).

    double preambleDetectionPowerThresholdDbm;

    // Delays

    TimeType aSlotTimeDuration;
    TimeType aShortInterframeSpaceDuration;
    TimeType aRxTxTurnaroundTime;
    TimeType aPreambleLengthDuration;
    TimeType aPhysicalLayerCpHeaderLengthDuration; // aka aPLCPHeaderLength
    DatarateBitsPerSecType aPhysicalLayerCpHeaderRateBitsPerSec;

    // Standard radio parameter for how much noise the radio circuitry adds.

    double radioNoiseFigureDb;

    double channelBandwidthMhz;

    IntegralPowerType thermalNoisePower;

    // New signal must be at least this dB over the signal currently being received to preempt.

    double signalCaptureRatioThresholdDb;

    // Used for error messages:

    NodeIdType nodeId;
    InterfaceIdType interfaceId;

    // Model State variabbles

    enum PhyStateType { PHY_SCANNING, PHY_RECEIVING, PHY_TX_STARTING, PHY_TRANSMITTING };

    PhyStateType phyState;

    string phyProtocolString;

    // int currentChannelNumber;

    shared_ptr<PropFrameType> currentPropagatedFramePtr;
    DatarateBitsPerSecType outgoingTransmissionDatarateBitsPerSecond;
    double outgoingTransmissionPowerDbm;

    DatarateBitsPerSecType currentSignalDatarateBitsPerSecond;

    // Must be equivalent (but in dBm vs Milliwatts)
    double currentSignalPowerDbm;
    double currentAdjustedSignalPowerMilliwatts;

    int currentFrameSizeBytes;

    map<DatarateBitsPerSecType, ModulationAndCodingSchemesType> mcsForDatarateMap;
    shared_ptr<BitErrorRateCurve> bitErrorCurveForPhyHeaderPtr;
    shared_ptr<BitErrorRateCurve> currentBitErrorRateCurvePtr;

    bool currentPacketHasAnError;

    IntegralPowerType currentInterferencePower;
    TimeType lastInterferenceSignalChangeTime;

    bool currentlySensingBusyMedium;


    TimeType currentIncomingSignalStartTime;
    NodeIdType currentIncomingSignalSourceNodeId;

    PacketIdType currentLockedOnFramePacketId;

    RandomNumberGenerator aRandomNumberGenerator;


    //For Stats

    shared_ptr<CounterStatistic> transmittedFramesStatPtr;

    double lastReceivedPacketRssiDbm;
    shared_ptr<RealStatistic> receivedFrameRssiDbmStatPtr;
    shared_ptr<RealStatistic> receivedFrameSinrDbStatPtr;

    int numberOfReceivedFrames;
    shared_ptr<CounterStatistic> receivedFramesStatPtr;

    int numberOfFramesWithErrors;
    shared_ptr<CounterStatistic> framesWithErrorsStatPtr;

    int numberOfSignalCaptures;
    shared_ptr<CounterStatistic> signalCaptureStatPtr;

    TimeType lastChannelStateTransitionTime;
    TimeType totalIdleChannelTime;
    TimeType totalBusyChannelTime;
    TimeType totalTransmissionTime;


    shared_ptr<CounterStatistic> signalsDuringTransmissionStatPtr;
    shared_ptr<CounterStatistic> weakSignalsStatPtr;
    shared_ptr<CounterStatistic> interferingSignalsStatPtr;

    int tracePrecisionDigitsForDbm;

    //-----------------------------------------------------
    class SignalArrivalHandler: public SimplePropagationModelForNode<PropFrameType>::SignalHandler {
    public:
        SignalArrivalHandler(T109Phy* initPhyPtr) : phyPtr(initPhyPtr) { }
        void ProcessSignal(const IncomingSignal& aSignal) { phyPtr->ProcessSignalArrivalFromChannel(aSignal); }
    private:
        T109Phy* phyPtr;
    };//SignalArrivalHandler//

    class SignalEndHandler: public SimplePropagationModelForNode<PropFrameType>::SignalHandler {
    public:
        SignalEndHandler(T109Phy* initPhyPtr) : phyPtr(initPhyPtr) { }
        void ProcessSignal(const IncomingSignal& aSignal) { phyPtr->ProcessSignalEndFromChannel(aSignal); }
    private:
        T109Phy* phyPtr;
    };//SignalEndHandler//


    class TransmissionTimerEvent: public SimulationEvent {
    public:
        TransmissionTimerEvent(T109Phy* initPhyPtr) : phyPtr(initPhyPtr) { }
        void ExecuteEvent() { phyPtr->StartOrEndTransmission(); }
    private:
        T109Phy* phyPtr;
    };//EndOfTransmissionEvent//


    shared_ptr<SimulationEvent> transmissionTimerEventPtr;

    //-----------------------------------------------------

    void PerformBernoulliTrialBitErrorCalculation(
        const BitErrorRateCurve& bitErrorRateCurve,
        const double& signalToNoiseAndInterferenceRatio,
        const TimeType& duration,
        bool& foundAnError);

    void UpdatePacketReceptionCalculation();

    void StartTransmission();
    void EndCurrentTransmission();
    void StartOrEndTransmission();

    void StartReceivingThisSignal(const IncomingSignal& aSignal);
    void ProcessNewSignal(const IncomingSignal& aSignal);

    void AddSignalPowerToInterferenceLevel(const double& signalPowerMw);
    void AddSignalPowerToInterferenceLevelAndDoStats(
        const double& signalPowerDbm,
        const double& adjustedSignalPowerMw,
        const NodeIdType& signalSourceNodeId,
        const PacketIdType& signalPacketId,
        const int frameSizeBytes);

    void AddSignalToInterferenceLevel(const IncomingSignal& aSignal);
    void SubtractSignalFromInterferenceLevel(const IncomingSignal& aSignal);
    void SubtractSignalFromInterferenceLevelAndNotifyMacIfMediumIsClear(const IncomingSignal& aSignal);

    bool IsCurrentlyReceivingThisSignal(const IncomingSignal& aSignal) const;

    void ProcessEndOfTheSignalCurrentlyBeingReceived(const IncomingSignal&  aSignal);

    void ProcessSignalArrivalFromChannel(const IncomingSignal& aSignal);
    void ProcessSignalEndFromChannel(const IncomingSignal& aSignal);

    void OutputTraceAndStatsForAddSignalToInterferenceLevel(
        const double& signalPowerDbm,
        const double& adjustedSignalPowerDbm,
        const NodeIdType& signalSourceNodeId,
        const PacketIdType& signalPacketId,
        const int frameSizeBytes) const;

    void OutputTraceForSubtractSignalFromInterferenceLevel(const IncomingSignal& aSignal) const;

    void OutputTraceAndStatsForTxStart(
        const Packet& aPacket,
        const double& txPowerDbm,
        const DatarateBitsPerSecType& dataRateBitsSec,
        const TimeType& duration) const;

    void OutputTraceAndStatsForRxStart(const Packet& aPacket);

    void OutputTraceAndStatsForRxEnd(const IncomingSignal& aSignal, const bool& rxIsEndedByCapture);

    void ProcessStatsForTxStartingStateTransition();
    void ProcessStatsForEndCurrentTransmission();
    void ProcessStatsForTransitionToBusyChannel();
    void ProcessStatsForTransitionToIdleChannel();

    // Parallelism Stuff:

    unsigned int eotIndex;

};//T109Phy//



//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

inline
T109Phy::T109Phy(
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<SimplePropagationModelForNode<PropFrameType> >& initPropModelInterfacePtr,
    const shared_ptr<BitOrBlockErrorRateCurveDatabase>& initBerCurveDatabasePtr,
    T109Mac* initMacLayerPtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& initNodeId,
    const InterfaceIdType& initInterfaceId,
    const RandomNumberGeneratorSeedType& nodeSeed)
    :
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    propagationModelInterfacePtr(initPropModelInterfacePtr),
    berCurveDatabasePtr(initBerCurveDatabasePtr),
    macLayerPtr(initMacLayerPtr),
    nodeId(initNodeId),
    interfaceId(initInterfaceId),
    phyState(PHY_SCANNING),
    outgoingTransmissionDatarateBitsPerSecond(0),
    outgoingTransmissionPowerDbm(0.0),
    currentlySensingBusyMedium(false),
    lastInterferenceSignalChangeTime(ZERO_TIME),
    currentIncomingSignalStartTime(ZERO_TIME),
    currentSignalDatarateBitsPerSecond(0),
    currentSignalPowerDbm(-DBL_MAX),
    currentAdjustedSignalPowerMilliwatts(0.0),
    currentFrameSizeBytes(0),
    currentPropagatedFramePtr(new PropFrameType()),
    currentPacketHasAnError(false),
    currentIncomingSignalSourceNodeId(0),
    aRandomNumberGenerator(HashInputsToMakeSeed(nodeSeed, initInterfaceId, SEED_HASH)),
    numberOfReceivedFrames(0),
    numberOfFramesWithErrors(0),
    numberOfSignalCaptures(0),
    transmittedFramesStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_FramesTransmitted"))),
    receivedFrameRssiDbmStatPtr(
        simulationEngineInterfacePtr->CreateRealStatWithDbConversion(
            (modelName + '_' + interfaceId + "_ReceivedFrameRssiDbm"))),
    receivedFrameSinrDbStatPtr(
        simulationEngineInterfacePtr->CreateRealStatWithDbConversion(
            (modelName + '_' + interfaceId + "_ReceivedFrameSinrDb"))),
    receivedFramesStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_FramesReceived"))),
    framesWithErrorsStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_FramesWithErrors"))),
    signalCaptureStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_SignalsCaptured"))),
    signalsDuringTransmissionStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_SignalsDuringTransmission"))),
    weakSignalsStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_TooWeakToReceiveSignals"))),
    interferingSignalsStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_InterferingSignals"))),
    tracePrecisionDigitsForDbm(8)
{
    mcsForDatarateMap[3000000] = McsBpsk1Over2;
    mcsForDatarateMap[4500000] = McsBpsk3Over4;
    mcsForDatarateMap[6000000] = McsQpsk1Over2;
    mcsForDatarateMap[9000000] = McsQpsk3Over4;
    mcsForDatarateMap[12000000] = Mcs16Qam1Over2;
    mcsForDatarateMap[18000000] = Mcs16Qam3Over4;

    phyProtocolString =
        theParameterDatabaseReader.ReadString(
            "t109-phy-protocol",
            nodeId,
            interfaceId);

    channelBandwidthMhz = propagationModelInterfacePtr->GetChannelBandwidthMhz();

    if (channelBandwidthMhz != t109ChannelBandwidthMhz) {
        cerr << "Error: Channel bandwidth must be 10MHz for T109" << endl;
        exit(1);        
    }

    radioNoiseFigureDb =
        theParameterDatabaseReader.ReadDouble(
            "t109-radio-noise-figure-db",
            nodeId,
            interfaceId);

    thermalNoisePower = IntegralPowerType(
        CalculateThermalNoisePowerWatts(radioNoiseFigureDb, channelBandwidthMhz) * CalcThermalNoiseAdjustmentFactor() * 1000.0);


    energyDetectionPowerThresholdDbm =
        theParameterDatabaseReader.ReadDouble(
            "t109-energy-detection-power-threshold-dbm",
            nodeId,
            interfaceId);

    energyDetectionPowerThreshold =
        IntegralPowerType(ConvertToNonDb(energyDetectionPowerThresholdDbm));


    preambleDetectionPowerThresholdDbm =
        theParameterDatabaseReader.ReadDouble(
            "t109-preamble-detection-power-threshold-dbm",
            nodeId,
            interfaceId);


    signalCaptureRatioThresholdDb =
        theParameterDatabaseReader.ReadDouble(
            "t109-signal-capture-ratio-threshold-db",
            nodeId,
            interfaceId);

    aSlotTimeDuration =
        theParameterDatabaseReader.ReadTime(
            "t109-slot-time",
            nodeId,
            interfaceId);

    aShortInterframeSpaceDuration =
        theParameterDatabaseReader.ReadTime(
            "t109-sifs-time",
            nodeId,
            interfaceId);

    aRxTxTurnaroundTime =
        theParameterDatabaseReader.ReadTime(
            "t109-rx-tx-turnaround-time",
            nodeId,
            interfaceId);

    aPreambleLengthDuration =
        theParameterDatabaseReader.ReadTime(
            "t109-preamble-length-duration",
            nodeId,
            interfaceId);

    aPhysicalLayerCpHeaderLengthDuration =
        theParameterDatabaseReader.ReadTime(
            "t109-plcp-header-length-duration",
            nodeId,
            interfaceId);

    aPhysicalLayerCpHeaderRateBitsPerSec =
        theParameterDatabaseReader.ReadInt(
            "t109-plcp-header-rate-bits-sec",
            nodeId,
            interfaceId);

    if (mcsForDatarateMap.find(aPhysicalLayerCpHeaderRateBitsPerSec) == mcsForDatarateMap.end()) {
        cerr << "Phy header datarate " << aPhysicalLayerCpHeaderRateBitsPerSec << "bps not supported" << endl
             << "  Available datarate is 3000000, 4500000, 6000000, 9000000, 12000000 or 18000000bps" << endl;
        exit(1);
    }//if//

    const ModulationAndCodingSchemesType modulationAndCoding = 
        mcsForDatarateMap[aPhysicalLayerCpHeaderRateBitsPerSec];

    bitErrorCurveForPhyHeaderPtr =
        berCurveDatabasePtr->GetBerCurve(
            phyProtocolString,
            GetModulationAndCodingName(modulationAndCoding));

    transmissionTimerEventPtr = shared_ptr<SimulationEvent>(new TransmissionTimerEvent(this));
    propagationModelInterfacePtr->RegisterSignalHandler(
        unique_ptr<SimplePropagationModelForNode<PropFrameType>::SignalHandler>(
            new SignalArrivalHandler(this)));
    propagationModelInterfacePtr->RegisterSignalEndHandler(
        unique_ptr<SimplePropagationModelForNode<PropFrameType>::SignalHandler>(
            new SignalEndHandler(this)));

    // Paralelism Stuff:

    (*this).eotIndex = (*this).simulationEngineInterfacePtr->AllocateEarliestOutputTimeIndex();
    simulationEngineInterfacePtr->SetAnEarliestOutputTimeForThisNode(INFINITE_TIME, eotIndex);

}//T109Phy()//



inline
TimeType T109Phy::CalculateFrameTransmitDuration(
    const size_t frameLengthBytes,
    const DatarateBitsPerSecType& datarateBitsPerSecond) const
{
    typedef map<DatarateBitsPerSecType, ModulationAndCodingSchemesType>::const_iterator IterType;

    IterType iter = mcsForDatarateMap.find(datarateBitsPerSecond);

    if (iter == mcsForDatarateMap.end()) {
        cerr << "Phy datarate " << datarateBitsPerSecond << "bps not supported" << endl
             << "  Available datarate is 3000000, 4500000, 6000000, 9000000, 12000000 or 18000000bps" << endl;
        exit(1);
    }//if//

    return (*this).CalculateFrameTransmitDuration(frameLengthBytes, datarateBitsPerSecond, (*iter).second);

}//CalculateFrameTransmitDuration//

inline
TimeType T109Phy::CalculateFrameTransmitDuration(
    const size_t frameLengthBytes,
    const DatarateBitsPerSecType& datarateBitsPerSecond,
    const ModulationAndCodingSchemesType& modulationAndCodingScheme) const
{
    const int numberFrameBits = (static_cast<int>(frameLengthBytes + fcsFieldLengthBytes) * 8) + phyFrameDataPaddingBits;
    const int numberOfOfdmBitsPerSymbol = CalcNumberOfBitsPerOfdmSymbol(modulationAndCodingScheme);
    const int numberOfOfdmSymbols = (numberFrameBits + numberOfOfdmBitsPerSymbol - 1) / numberOfOfdmBitsPerSymbol;

    return
        (aPreambleLengthDuration +
         aPhysicalLayerCpHeaderLengthDuration +
         static_cast<TimeType>(SECOND * (static_cast<double>(numberOfOfdmSymbols * numberOfOfdmBitsPerSymbol) / datarateBitsPerSecond)));

}//CalculateFrameTransmitDuration//


inline
void T109Phy::ProcessStatsForTxStartingStateTransition()
{
    TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
    if (currentlySensingBusyMedium) {
        (*this).totalBusyChannelTime += (currentTime - lastChannelStateTransitionTime);
    }
    else {
        (*this).totalIdleChannelTime += (currentTime - lastChannelStateTransitionTime);
    }//if//

    (*this).lastChannelStateTransitionTime = currentTime;

}//ProcessStatsForTxStartingStateTransition//



inline
void T109Phy::TransmitFrame(
    unique_ptr<Packet>& packetPtr,
    const DatarateBitsPerSecType& datarateBitsPerSecond,
    const double& transmitPowerDbm,
    const TimeType& delayUntilAirborne)
{
    assert((phyState != PHY_TX_STARTING) && (phyState != PHY_TRANSMITTING));

    if (phyState == PHY_RECEIVING) {
        //switch RECEIVING to TX_STARTING

        signalsDuringTransmissionStatPtr->IncrementCounter();

        const double adjustedSignalPowerMw =
            (ConvertToNonDb(currentSignalPowerDbm) *
             CalcPilotSubcarriersPowerAdjustmentFactor());

        (*this).AddSignalPowerToInterferenceLevelAndDoStats(
            currentSignalPowerDbm,
            adjustedSignalPowerMw,
            currentIncomingSignalSourceNodeId,
            currentLockedOnFramePacketId,
            currentFrameSizeBytes);

        (*this).currentSignalPowerDbm = -DBL_MAX;
        (*this).currentAdjustedSignalPowerMilliwatts = 0.0;
        (*this).currentFrameSizeBytes = 0;
        (*this).currentIncomingSignalSourceNodeId = 0;
        (*this).currentLockedOnFramePacketId = PacketIdType::nullPacketId;

    }//if//

    (*this).ProcessStatsForTxStartingStateTransition();

    phyState = PHY_TX_STARTING;

    currentPropagatedFramePtr->datarateBitsPerSecond = datarateBitsPerSecond;

    typedef map<DatarateBitsPerSecType, ModulationAndCodingSchemesType>::const_iterator IterType;

    IterType iter = mcsForDatarateMap.find(datarateBitsPerSecond);

    if (iter == mcsForDatarateMap.end()) {
        cerr << "Phy datarate " << datarateBitsPerSecond << "bps not supported" << endl
             << "  Available datarate is 3000000, 4500000, 6000000, 9000000, 12000000 or 18000000bps" << endl;
        exit(1);
    }//if//

    currentPropagatedFramePtr->modulationAndCodingScheme = (*iter).second;

    currentPropagatedFramePtr->macFramePtr = move(packetPtr);

    outgoingTransmissionDatarateBitsPerSecond = datarateBitsPerSecond;
    outgoingTransmissionPowerDbm = transmitPowerDbm;

    TimeType startTransmissionTime = simulationEngineInterfacePtr->CurrentTime() + delayUntilAirborne;
    simulationEngineInterfacePtr->ScheduleEvent(transmissionTimerEventPtr, startTransmissionTime);

    (*this).ProcessStatsForTxStartingStateTransition();

    // Parallelism Stuff: Transmission is imminent.

    simulationEngineInterfacePtr->SetAnEarliestOutputTimeForThisNode(
        (simulationEngineInterfacePtr->CurrentTime() + delayUntilAirborne),
        eotIndex);

}//TransmitFrame//


inline
void T109Phy::StartTransmission()
{
    assert(phyState == PHY_TX_STARTING);
    phyState = PHY_TRANSMITTING;

    // The medium is busy because we are transmitting on it!
    (*this).currentlySensingBusyMedium = true;

    const TimeType duration =
        CalculateFrameTransmitDuration(
            currentPropagatedFramePtr->macFramePtr->LengthBytes(),
            currentPropagatedFramePtr->datarateBitsPerSecond,
            currentPropagatedFramePtr->modulationAndCodingScheme);

    OutputTraceAndStatsForTxStart(
        *currentPropagatedFramePtr->macFramePtr,
        outgoingTransmissionPowerDbm,
        currentPropagatedFramePtr->datarateBitsPerSecond,
        duration);

    propagationModelInterfacePtr->TransmitSignal(
        outgoingTransmissionPowerDbm, duration, currentPropagatedFramePtr);

    TimeType endOfTransmissionTime = simulationEngineInterfacePtr->CurrentTime() + duration;

    simulationEngineInterfacePtr->ScheduleEvent(transmissionTimerEventPtr, endOfTransmissionTime);

    // Parallelism: Delete stale Earliest Output Time.

    (*this).simulationEngineInterfacePtr->SetAnEarliestOutputTimeForThisNode(INFINITE_TIME, eotIndex);

}//StartTransmission//



inline
void T109Phy::StartOrEndTransmission()
{
    if (phyState == PHY_TX_STARTING) {
        (*this).StartTransmission();
    }
    else if (phyState == PHY_TRANSMITTING) {
        (*this).EndCurrentTransmission();
    }
    else {
        assert(false); abort();
    }//if//
}

inline
void T109Phy::PerformBernoulliTrialBitErrorCalculation(
    const BitErrorRateCurve& bitErrorRateCurve,
    const double& signalToNoiseAndInterferenceRatio,
    const TimeType& duration,
    bool& foundAnError)
{
    const int numberBits = int(((double(duration) / SECOND) * currentSignalDatarateBitsPerSecond) + 0.5);

    if (numberBits == 0) {
        foundAnError = false;
    }
    else {
        const double bitErrorRate = bitErrorRateCurve.CalculateBitErrorRate(signalToNoiseAndInterferenceRatio);

        const double probabilityOfZeroErrors = pow((1.0-bitErrorRate), numberBits);
        const double randomNumber = (*this).aRandomNumberGenerator.GenerateRandomDouble();

        foundAnError = (randomNumber > probabilityOfZeroErrors);

    }//if//

}//PerformBernoulliTrialBitErrorCalculation//


inline
void T109Phy::UpdatePacketReceptionCalculation()
{
    using std::min;
    using std::max;

    if (!currentPacketHasAnError) {

        const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
        const TimeType startHeaderTime = (currentIncomingSignalStartTime + aPreambleLengthDuration);
        const TimeType startDataTime = startHeaderTime + aPhysicalLayerCpHeaderLengthDuration;

        if (currentTime <= startHeaderTime) {
            // Still in preamble, nothing to do.
            return;
        }//if//

        const IntegralPowerType totalNoiseAndInterferencePower =
            (thermalNoisePower + currentInterferencePower);

        const double signalToNoiseAndInterferenceRatio =
            (currentAdjustedSignalPowerMilliwatts / totalNoiseAndInterferencePower.ConvertToMilliwatts());

        assert(currentTime > startHeaderTime);

        if (lastInterferenceSignalChangeTime < startDataTime) {

            const TimeType timeInHeader =
                min(currentTime, startDataTime) -
                max(lastInterferenceSignalChangeTime, startHeaderTime);

            (*this).PerformBernoulliTrialBitErrorCalculation(
                *bitErrorCurveForPhyHeaderPtr,
                signalToNoiseAndInterferenceRatio,
                timeInHeader,
                currentPacketHasAnError);

            if (currentPacketHasAnError) {
                return;
            }//if//

        }//if//

        if (currentTime > startDataTime) {
            const TimeType timeInDataPart =
                currentTime -
                max(lastInterferenceSignalChangeTime, startDataTime);

            (*this).PerformBernoulliTrialBitErrorCalculation(
                *currentBitErrorRateCurvePtr,
                signalToNoiseAndInterferenceRatio,
                timeInDataPart,
                currentPacketHasAnError);
        }//if//

    }//if//

}//UpdatePacketReceptionCalculation//



inline
void T109Phy::AddSignalPowerToInterferenceLevel(const double& adjustedSignalPowerMw)
{
    if (phyState == PHY_RECEIVING) {
        (*this).UpdatePacketReceptionCalculation();
    }//if//

    (*this).lastInterferenceSignalChangeTime = simulationEngineInterfacePtr->CurrentTime();
    (*this).currentInterferencePower += adjustedSignalPowerMw;
}


inline
void T109Phy::AddSignalPowerToInterferenceLevelAndDoStats(
    const double& signalPowerDbm,
    const double& adjustedSignalPowerMw,
    const NodeIdType& signalSourceNodeId,
    const PacketIdType& signalPacketId,
    const int frameSizeBytes)
{
    (*this).AddSignalPowerToInterferenceLevel(adjustedSignalPowerMw);

    OutputTraceAndStatsForAddSignalToInterferenceLevel(
        signalPowerDbm,
        ConvertToDb(adjustedSignalPowerMw),
        signalSourceNodeId,
        signalPacketId,
        frameSizeBytes);
}


inline
void T109Phy::AddSignalToInterferenceLevel(const IncomingSignal& aSignal)
{
    PacketIdType packetId;
    if (aSignal.HasACompleteFrame()) {
        packetId = aSignal.GetFrame().macFramePtr->GetPacketId();
    }//if//

    // Assuming pilots don't contribute to interference.

    const double signalPowerDbm = aSignal.GetReceivedPowerDbm();

    const double adjustedSignalPowerMw =
        (ConvertToNonDb(signalPowerDbm) *
         CalcPilotSubcarriersPowerAdjustmentFactor());

    AddSignalPowerToInterferenceLevelAndDoStats(
        signalPowerDbm,
        adjustedSignalPowerMw,
        aSignal.GetSourceNodeId(),
        packetId,
        aSignal.GetFrame().macFramePtr->LengthBytes());
}


inline
void T109Phy::SubtractSignalFromInterferenceLevel(const IncomingSignal& aSignal)
{
    if (phyState == PHY_RECEIVING) {
        (*this).UpdatePacketReceptionCalculation();
    }//if//

    (*this).lastInterferenceSignalChangeTime = simulationEngineInterfacePtr->CurrentTime();
    
    const double adjustedSignalPowerMw =
        (ConvertToNonDb(aSignal.GetReceivedPowerDbm()) *
         CalcPilotSubcarriersPowerAdjustmentFactor());

    (*this).currentInterferencePower -= adjustedSignalPowerMw;

    OutputTraceForSubtractSignalFromInterferenceLevel(aSignal);

}//SubtractSignalFromInterferenceLevel//


inline
void T109Phy::StartReceivingThisSignal(const IncomingSignal& aSignal)
{
    (*this).currentIncomingSignalStartTime = simulationEngineInterfacePtr->CurrentTime();
    (*this).currentIncomingSignalSourceNodeId = aSignal.GetSourceNodeId();
    (*this).currentSignalDatarateBitsPerSecond = aSignal.GetFrame().datarateBitsPerSecond;
    
    (*this).currentBitErrorRateCurvePtr =
        berCurveDatabasePtr->GetBerCurve(
            phyProtocolString,
            GetModulationAndCodingName(aSignal.GetFrame().modulationAndCodingScheme));
    
    (*this).currentPacketHasAnError = false;
    (*this).currentSignalPowerDbm = aSignal.GetReceivedPowerDbm();

    (*this).currentAdjustedSignalPowerMilliwatts =
        (ConvertToNonDb(currentSignalPowerDbm) *
         CalcSignalPowerAdjustmentFactor());

    (*this).currentFrameSizeBytes = aSignal.GetFrame().macFramePtr->LengthBytes();

    // Start interference calculation.
    (*this).lastInterferenceSignalChangeTime = simulationEngineInterfacePtr->CurrentTime();

    (*this).currentLockedOnFramePacketId = aSignal.GetFrame().macFramePtr->GetPacketId();

    OutputTraceAndStatsForRxStart(*aSignal.GetFrame().macFramePtr);

}//StartReceivingThisSignal//




inline
void T109Phy::ProcessSignalArrivalFromChannel(const IncomingSignal& aSignal)
{
    switch (phyState) {
    case PHY_SCANNING:
    case PHY_RECEIVING:
        (*this).ProcessNewSignal(aSignal);

        break;
    case PHY_TRANSMITTING:
    case PHY_TX_STARTING:
        (*this).AddSignalToInterferenceLevel(aSignal);

        break;
    default:
        assert(false); abort(); break;
    }//switch//

}//ProcessSignalArrivalFromChannel//


inline
bool T109Phy::IsCurrentlyReceivingThisSignal(const IncomingSignal& aSignal) const
{
    assert(phyState == PHY_RECEIVING);
    return (aSignal.GetSourceNodeId() == currentIncomingSignalSourceNodeId);
}



inline
void T109Phy::ProcessSignalEndFromChannel(const IncomingSignal& aSignal)
{
    switch (phyState) {
    case PHY_RECEIVING:
        if (IsCurrentlyReceivingThisSignal(aSignal)) {
            (*this).ProcessEndOfTheSignalCurrentlyBeingReceived(aSignal);
        }
        else {
            (*this).SubtractSignalFromInterferenceLevelAndNotifyMacIfMediumIsClear(aSignal);
        }//if//

        break;

    case PHY_SCANNING:
    case PHY_TRANSMITTING:
    case PHY_TX_STARTING:

        (*this).SubtractSignalFromInterferenceLevelAndNotifyMacIfMediumIsClear(aSignal);

        break;

    default:
        assert(false); abort(); break;
    }//switch//

}//ProcessSignalEndFromChannel//


inline
void T109Phy::SwitchToChannelNumber(const unsigned int channelNumber)
{
    const unsigned int currentChannel = GetCurrentChannelNumber();
    if (currentChannel == channelNumber) {
        return;
    }

    (*this).currentSignalPowerDbm = -DBL_MAX;
    (*this).currentAdjustedSignalPowerMilliwatts = 0.0;
    (*this).currentFrameSizeBytes = 0;
    (*this).currentIncomingSignalSourceNodeId = 0;
    (*this).currentLockedOnFramePacketId = PacketIdType::nullPacketId;
    (*this).currentInterferencePower = IntegralPowerType(0.0);
    (*this).currentlySensingBusyMedium = false;

    switch (phyState) {
    case PHY_TRANSMITTING:
    case PHY_TX_STARTING:

        assert(false && "Should be prevented from occuring"); abort();
        break;

    case PHY_RECEIVING:
    case PHY_SCANNING:

        phyState = PHY_SCANNING;

        propagationModelInterfacePtr->SwitchToChannelNumber(channelNumber);

        break;

    default:
        assert(false); abort(); break;
    }//switch//

}//SwitchToChannelNumber//


inline
void T109Phy::OutputTraceAndStatsForAddSignalToInterferenceLevel(
    const double& signalPowerDbm,
    const double& adjustedSignalPowerDbm,
    const NodeIdType& signalSourceNodeId,
    const PacketIdType& signalPacketId,
    const int frameSizeBytes) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TracePhyInterference)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            T109NoiseStartTraceRecord traceData;

            const double currentInterferenceAndNoisePowerDbm =
                ConvertToDb((thermalNoisePower + currentInterferencePower).ConvertToMilliwatts());

            traceData.sourceNodeId = signalSourceNodeId;
            traceData.rxPower = adjustedSignalPowerDbm;
            traceData.interferenceAndNoisePower = currentInterferenceAndNoisePowerDbm;

            assert(sizeof(traceData) == T109_NOISE_START_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "NoiseStart", traceData);

        }
        else {
            const double currentInterferenceAndNoisePowerDbm =
                ConvertToDb((thermalNoisePower + currentInterferencePower).ConvertToMilliwatts());

            ostringstream msgStream;
            msgStream.precision(tracePrecisionDigitsForDbm);

            msgStream << "SrcN= " << signalSourceNodeId
                      << " RxPow= " << adjustedSignalPowerDbm
                      << " I&NPow= " << currentInterferenceAndNoisePowerDbm;

            simulationEngineInterfacePtr->OutputTrace(modelName, interfaceId, "NoiseStart", msgStream.str());
        }//if//
    }//if//

    interferingSignalsStatPtr->IncrementCounter();

    if ((weakSignalsStatPtr->IsEnabled()) && (signalPowerDbm < preambleDetectionPowerThresholdDbm)) {

        weakSignalsStatPtr->IncrementCounter();
    }//if//


    if ((signalsDuringTransmissionStatPtr->IsEnabled()) &&
        ((phyState == PHY_TX_STARTING) || (phyState == PHY_TRANSMITTING)) &&
        (signalPowerDbm >= preambleDetectionPowerThresholdDbm)) {

        // Count all signals that the PHY could lock on to staring during a transmission.

        signalsDuringTransmissionStatPtr->IncrementCounter();

    }//if//


}//OutputTraceForAddSignalToInterferenceLevel//

inline
void T109Phy::OutputTraceForSubtractSignalFromInterferenceLevel(const IncomingSignal& aSignal) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TracePhyInterference)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            T109NoiseEndTraceRecord traceData;

            const double currentInterferenceAndNoisePowerDbm =
                ConvertToDb((thermalNoisePower + currentInterferencePower).ConvertToMilliwatts());

            traceData.rxPower = aSignal.GetReceivedPowerDbm();
            traceData.interferenceAndNoisePower = currentInterferenceAndNoisePowerDbm;

            assert(sizeof(traceData) == T109_NOISE_END_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "NoiseEnd", traceData);

        }
        else {

            const double currentInterferenceAndNoisePowerDbm =
                ConvertToDb((thermalNoisePower + currentInterferencePower).ConvertToMilliwatts());

            ostringstream msgStream;
            msgStream.precision(tracePrecisionDigitsForDbm);

            msgStream << "RxPow= " << aSignal.GetReceivedPowerDbm()
                      << " I&NPow= " << currentInterferenceAndNoisePowerDbm;

            simulationEngineInterfacePtr->OutputTrace(modelName, interfaceId, "NoiseEnd", msgStream.str());
        }//if//
    }//if//

}//OutputTraceForSubtractSignalFromInterferenceLevel//

inline
void T109Phy::OutputTraceAndStatsForTxStart(
    const Packet& aPacket,
    const double& txPowerDbm,
    const DatarateBitsPerSecType& dataRateBitsSec,
    const TimeType& duration) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TracePhy)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            T109TxStartTraceRecord traceData;
            const PacketIdType& packetId = aPacket.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            traceData.txPower = txPowerDbm;
            traceData.dataRate = dataRateBitsSec;
            traceData.duration = duration;

            assert(sizeof(traceData) == T109_TX_START_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "TxStart", traceData);

        }
        else {
            ostringstream msgStream;
            msgStream.precision(tracePrecisionDigitsForDbm);

            msgStream << "PktId= " << aPacket.GetPacketId()
                      << " TxPow= " << txPowerDbm
                      << " Rate= " << dataRateBitsSec
                      << " Dur= " << ConvertTimeToStringSecs(duration);

            simulationEngineInterfacePtr->OutputTrace(modelName, interfaceId, "TxStart", msgStream.str());
        }//if//

    }//if//

    transmittedFramesStatPtr->IncrementCounter();

}//OutputTraceForTxStart//

inline
void T109Phy::OutputTraceAndStatsForRxStart(const Packet& aPacket)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TracePhy)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            T109RxStartTraceRecord traceData;
            const PacketIdType& packetId = aPacket.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            traceData.rxPower = currentSignalPowerDbm;

            assert(sizeof(traceData) == T109_RX_START_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "RxStart", traceData);

        }
        else {
            ostringstream msgStream;
            msgStream.precision(tracePrecisionDigitsForDbm);

            msgStream << "PktId= " << aPacket.GetPacketId() << " RxPow= " << currentSignalPowerDbm;

            simulationEngineInterfacePtr->OutputTrace(modelName, interfaceId, "RxStart", msgStream.str());

        }//if//

    }//if//

}//OutputTraceForRxStart//

inline
void T109Phy::OutputTraceAndStatsForRxEnd(const IncomingSignal& aSignal, const bool& receiveIsEndedByCapture)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TracePhy)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            T109RxEndTraceRecord traceData;
            const PacketIdType& packetId = aSignal.GetFrame().macFramePtr->GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            traceData.error = currentPacketHasAnError;
            traceData.captured = receiveIsEndedByCapture;
            if (!currentPacketHasAnError) {
                assert(!receiveIsEndedByCapture);
            }

            assert(sizeof(traceData) == T109_RX_END_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "RxEnd", traceData);

        }
        else {

            ostringstream msgStream;

            msgStream.precision(tracePrecisionDigitsForDbm);

            msgStream << "PktId= " << aSignal.GetFrame().macFramePtr->GetPacketId();

            if (currentPacketHasAnError) {
                if (receiveIsEndedByCapture) {
                    msgStream << " Err= YesCapture";
                }
                else {
                    msgStream << " Err= Yes";

                }//if//
            }
            else {
                assert(!receiveIsEndedByCapture);
                msgStream << " Err= No";
            }//if//

            simulationEngineInterfacePtr->OutputTrace(modelName, interfaceId, "RxEnd", msgStream.str());

        }//if//
    }//if//

    receivedFrameRssiDbmStatPtr->RecordStatValue(ConvertToNonDb(aSignal.GetReceivedPowerDbm()));

    const double signalToNoiseAndInterferenceRatio =
        ConvertToNonDb(aSignal.GetReceivedPowerDbm()) / ((thermalNoisePower + currentInterferencePower).ConvertToMilliwatts());
    receivedFrameSinrDbStatPtr->RecordStatValue(signalToNoiseAndInterferenceRatio);

    if (!currentPacketHasAnError) {
        (*this).numberOfReceivedFrames++;
        receivedFramesStatPtr->UpdateCounter(numberOfReceivedFrames);
    }
    else {
        if (!receiveIsEndedByCapture) {
            (*this).numberOfFramesWithErrors++;
            framesWithErrorsStatPtr->UpdateCounter(numberOfFramesWithErrors);
        }
        else {
            (*this).numberOfSignalCaptures++;
            signalCaptureStatPtr->UpdateCounter(numberOfSignalCaptures);
        }//if//
    }//if//

}//OutputTraceAndStatsForRxEnd//


}//namespace//


#endif
