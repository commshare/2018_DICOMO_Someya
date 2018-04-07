// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

//--------------------------------------------------------------------------------------------------
// "NotUsed" data items in header structs are placeholders for standard
// 802.11 fields that are not currently used in the model.  The purpose of
// not including the unused standard field names is to make plain the
// features that are and are NOT implemented.  The "Not Used" fields should always be
// zeroed so that packets do not include random garbage.  Likewise, only
// frame types and codes that are used in model logic will be defined.
//
// This code ignores machine endian issues because it is a model, i.e. fields are the
// correct sizes but the actual bits will not be in "network order" on small endian machines.
//

#ifndef DOT11_PHY_H
#define DOT11_PHY_H

#include <sstream>
#include <list>
#include <numeric>
#include <array>

#include "scensim_engine.h"
#include "scensim_netsim.h"
#include "scensim_prop.h"
#include "scensim_mimo_channel.h"
#include "scensim_freqsel_channel.h"
#include "scensim_bercurves.h"

#include "dot11_common.h"
#include "dot11_tracedefs.h"
#include "dot11_info_interface.h"

namespace Dot11 {

using std::ostringstream;
using std::endl;
using std::cerr;
using std::list;
using std::accumulate;
using std::shared_ptr;
using std::enable_shared_from_this;
using std::unique_ptr;
using std::move;
using std::array;
using std::map;


using ScenSim::TimeType;
using ScenSim::INFINITE_TIME;
using ScenSim::ZERO_TIME;
using ScenSim::SECOND;
using ScenSim::MICRO_SECOND;
using ScenSim::NANO_SECOND;
using ScenSim::ConvertTimeToStringSecs;
using ScenSim::SimulationEngineInterface;
using ScenSim::SimulationEvent;
using ScenSim::EventRescheduleTicket;
using ScenSim::SimplePropagationModelForNode;
using ScenSim::MimoChannelModel;
using ScenSim::FrequencySelectiveFadingModel;
using ScenSim::BitOrBlockErrorRateCurveDatabase;
using ScenSim::BitErrorRateCurve;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeIdType;
using ScenSim::InterfaceIdType;
using ScenSim::InterfaceOrInstanceIdType;
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
using ScenSim::DivideAndRoundUp;
using ScenSim::ConvertStringToUpperCase;
using ScenSim::MakeUpperCaseString;
using ScenSim::MakeLowerCaseString;
using ScenSim::RoundToUint;
using ScenSim::ConvertTimeToDoubleSecs;
using ScenSim::DeleteTrailingSpaces;


class Dot11Mac;
class Dot11Phy;

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

enum ModulationAndCodingSchemesType {
    McsBpsk1Over2,
    McsBpsk3Over4,
    McsQpsk1Over2,
    McsQpsk3Over4,
    Mcs16Qam1Over2,
    Mcs16Qam3Over4,
    Mcs64Qam2Over3,
    Mcs64Qam3Over4,
    Mcs64Qam5Over6,
    McsInvalid,
};

const unsigned int NumberModAndCodingSchemes = 9;

const ModulationAndCodingSchemesType minModulationAndCodingScheme = McsBpsk1Over2;
const ModulationAndCodingSchemesType maxModulationAndCodingScheme = Mcs64Qam5Over6;


inline
string GetModulationAndCodingName(const ModulationAndCodingSchemesType modulationAndCoding)
{
    static const array<string, NumberModAndCodingSchemes> mcsName =
        {"BPSK_0.5", "BPSK_0.75", "QPSK_0.5", "QPSK_0.75", "16QAM_0.5", "16QAM_0.75",
         "64QAM_0.67", "64QAM_0.75", "64QAM_0.83" };
    return (mcsName.at(static_cast<size_t>(modulationAndCoding)));
}


inline
void IncrementModAndCodingScheme(ModulationAndCodingSchemesType& mcs)
{
    assert((minModulationAndCodingScheme <= mcs) && (mcs <= maxModulationAndCodingScheme));
    mcs = static_cast<ModulationAndCodingSchemesType>(static_cast<unsigned int>(mcs) + 1);
}


inline
void DecrementModAndCodingScheme(ModulationAndCodingSchemesType& mcs)
{
    assert((minModulationAndCodingScheme < mcs) && (mcs <= maxModulationAndCodingScheme));
    mcs = static_cast<ModulationAndCodingSchemesType>(static_cast<unsigned int>(mcs) - 1);
}


inline
bool IsAHighThroughputModulationAndCoding(const ModulationAndCodingSchemesType modulationAndCoding)
{
    return (modulationAndCoding != McsBpsk3Over4);
}

inline
bool IsANonHtModulationAndCoding(const ModulationAndCodingSchemesType modulationAndCoding)
{
    return (modulationAndCoding != Mcs64Qam5Over6);
}


inline
void GetModulationAndCodingSchemeFromName(
    const string& modAndCodingNameString,
    bool& wasFound,
    ModulationAndCodingSchemesType& modulationAndCoding)
{
    const string theName = MakeUpperCaseString(modAndCodingNameString);
    wasFound = false;
    for(ModulationAndCodingSchemesType mcs = minModulationAndCodingScheme;
        (mcs <= maxModulationAndCodingScheme);
        IncrementModAndCodingScheme(mcs)) {

        if (GetModulationAndCodingName(mcs) == theName) {
            wasFound = true;
            modulationAndCoding = mcs;
        }//if//
    }//for//
}//GetModulationAndCodingSchemeFromName//



inline
ModulationAndCodingSchemesType ConvertNameToModulationAndCodingScheme(
    const string& modAndCodingNameString,
    const string& parameterNameForErrorOutput)
{
    bool wasFound;
    ModulationAndCodingSchemesType modulationAndCoding;

    GetModulationAndCodingSchemeFromName(modAndCodingNameString, wasFound, modulationAndCoding);

    if (!wasFound) {
        cerr << "Error in " << parameterNameForErrorOutput << " parameter: "
             << "Modulation and coding name \"" << modAndCodingNameString << "\" not valid." << endl;
        exit(1);
    }//if//

    return (modulationAndCoding);

}//ConvertNameToModulationAndCodingScheme//


inline
unsigned int GetBitsPerSymbolForModulation(const ModulationAndCodingSchemesType modulationAndCoding)
{
    static const array<unsigned int, NumberModAndCodingSchemes> bitsPerSymbol =
        {1, 1, 2, 2, 4, 4, 6, 6, 6};
    return (bitsPerSymbol.at(static_cast<size_t>(modulationAndCoding)));
}

inline
double GetCodingRate(const ModulationAndCodingSchemesType modulationAndCoding)
{
    static const array<double, NumberModAndCodingSchemes> codingRates =
        {0.5, 0.75, 0.5, 0.75, 0.5, 0.75, (2.0/3.0), 0.75, (5.0/6.0) };

    return (codingRates.at(static_cast<size_t>(modulationAndCoding)));
}

inline
double GetFractionalBitsPerSymbol(const ModulationAndCodingSchemesType modulationAndCoding)
{
    return (GetBitsPerSymbolForModulation(modulationAndCoding) * GetCodingRate(modulationAndCoding));
}

const unsigned int numOfdmSubcarriers = 64;
const unsigned int numOfdmDataSubcarriers = 48;
const unsigned int numUsedOfdmSubcarriers = 52;

// const unsigned int numOfdmPilotSubcarriers = (numUsedOfdmSubcarriers - numOfdmDataSubcarriers);

const unsigned int numHighThroughputOfdmDataSubcarriers = 52;
const unsigned int numUsedHighThroughputOfdmSubcarriers = 56;

// const unsigned int numHighThroughputOfdmPilotSubcarriers =
//     (numUsedHighThroughputOfdmSubcarriers - numHighThroughputOfdmDataSubcarriers);

const unsigned int numOfdmSubcarriersAt40MhzBandwidth = 128;
const unsigned int numOfdmDataSubcarriersAt40MhzBandwidth = 108;
const unsigned int numUsedOfdmSubcarriersAt40MhzBandwidth = 114;

// const unsigned int numOfdmPilotSubcarriersAt40MhzBandwidth =
//    (numUsedOfdmSubcarriersAt40MhzBandwidth - numOfdmDataSubcarriersAt40MhzBandwidth);

const double normalGuardIntervalOfdmSymbolDurationFactor = 0.8;
const double shortGuardIntervalOfdmSymbolDurationFactor = 0.9;


const double ofdm40MhzOn20MhzHtSubcarrierInterferenceFactor = (50.0 / 57);
const double ofdm20MhzHtOn40MhzSubcarrierInterferenceFactor = (51.0 / 56);

//     -3                                                              3
//      2                               0                              1
//           0    1    1    2    2    3    3    4    4    5    5    6
//      0****5****0****5****0****5****0****5****0****5****0****5****0***
// 40:  NNDDDDDDDDDPDDDDDDDDDDDDDPDDDDDDDDDDDDDDDDDDDDDDDDDDDPDDDDDNNNNN
// 20:  NNNNDDDDDDDPDDDDDDDDDDDDDPDDDDDDNDDDDDDPDDDDDDDDDDDDDPDDDDDDDNNN
//        --+++++++-+++++++++++++-++++++-++++++-+++++++++++++-+++++
// 40->20: 57 Tx subcarriers (1/2 40Mhz BW) and 50 data receivers (20 Mhz HT BW)
//          +++++++-+++++++++++++-++++++ ++++++++++++++++++++-+++++--
// 20->40: 56 Tx subcarriers and 51 data receivers.
//
//     -3                                                             +3
//      2                               0                              1
//         -6   -5   -5   -4   -4   -3   -3   -2   -2   -1   -1   -0    0
//      ****0****5****0****5****0****5****0****5****0****5****0****5****0
// 40:  NNNNNNDDDDDPDDDDDDDDDDDDDDDDDDDDDDDDDDDPDDDDDDDDDDDDDPDDDDDDDDDN
// 20:  NNNNDDDDDDDPDDDDDDDDDDDDDPDDDDDDNDDDDDDPDDDDDDDDDDDDDPDDDDDDDNNN
//            +++++-+++++++++++++-++++++-++++++-+++++++++++++-+++++++--
// 57 Tx subcarriers (1/2 40Mhz BW) and 50 data receivers (20 Mhz HT BW)



//---------------------------------------------------------

struct TransmissionParametersType {
    unsigned int firstChannelNumber;
    unsigned int channelBandwidthMhz;
    bool isHighThroughputFrame;
    ModulationAndCodingSchemesType modulationAndCodingScheme;
    unsigned int numberSpatialStreams;

    TransmissionParametersType() : firstChannelNumber(0),
        channelBandwidthMhz(0), isHighThroughputFrame(false),
        modulationAndCodingScheme(McsInvalid), numberSpatialStreams(1) { }
};


// Calculate wasted power for pilots and guard.



inline
double CalcPilotSubcarriersPowerAdjustmentFactor(const TransmissionParametersType& txParameters)
{
    double subcarrierFactor = 0.0;

    if (!txParameters.isHighThroughputFrame) {
        assert((txParameters.channelBandwidthMhz <= 20) &&
               "High Throughput Mode Must be enabled for 40Mhz or greater bandwidth.");

        subcarrierFactor = (static_cast<double>(numOfdmDataSubcarriers) / numUsedOfdmSubcarriers);
    }
    else {
        if (txParameters.channelBandwidthMhz == 40) {
            subcarrierFactor =
                (static_cast<double>(numOfdmDataSubcarriersAt40MhzBandwidth) /
                 numUsedOfdmSubcarriersAt40MhzBandwidth);

        }
        else {
            subcarrierFactor =
                (static_cast<double>(numHighThroughputOfdmDataSubcarriers) /
                 numUsedHighThroughputOfdmSubcarriers);
        }//if//
    }//if//

    return subcarrierFactor;

}//CalcPilotSubcarriersPowerAdjustmentFactor//



inline
double CalcSignalPowerAdjustmentFactor(const TransmissionParametersType& txParameters)
{
    return
        (CalcPilotSubcarriersPowerAdjustmentFactor(txParameters) *
         normalGuardIntervalOfdmSymbolDurationFactor);

}//CalcSignalPowerAdjustmentFactor//



// Factor adjusts "full bandwidth" noise power down to "used bandwidth" noise.

inline
double CalcThermalNoiseAdjustmentFactor(const TransmissionParametersType& txParameters)
{
    if (!txParameters.isHighThroughputFrame) {
        assert((txParameters.channelBandwidthMhz <= 20) &&
               "High Throughput Mode Must be enabled for 40Mhz or greater bandwidth.");

        return (static_cast<double>(numUsedOfdmSubcarriers) / numOfdmSubcarriers);
    }
    else {
        if (txParameters.channelBandwidthMhz == 40) {
            return
                (static_cast<double>(numUsedOfdmSubcarriersAt40MhzBandwidth) /
                 numOfdmSubcarriersAt40MhzBandwidth);
        }
        else {
            return
                (static_cast<double>(numUsedHighThroughputOfdmSubcarriers) / numOfdmSubcarriers);
        }//if//
    }//if//

}//CalcThermalNoiseAdjustmentFactor//


inline
unsigned int CalcNumberOfBitsPerOfdmSymbol(const TransmissionParametersType& txParameters)
{
    assert((txParameters.channelBandwidthMhz == 5) ||
           (txParameters.channelBandwidthMhz == 10) ||
           (txParameters.channelBandwidthMhz == 20) ||
           (txParameters.channelBandwidthMhz == 40));

    unsigned int numberSubcarriers;
    if (!txParameters.isHighThroughputFrame) {
        assert((txParameters.channelBandwidthMhz <= 20) &&
               "High Throughput Mode Must be enabled for 40Mhz or greater bandwidth.");

        numberSubcarriers = numOfdmDataSubcarriers;
    }
    else {
        if (txParameters.channelBandwidthMhz == 40) {
            numberSubcarriers =  numOfdmDataSubcarriersAt40MhzBandwidth;
        }
        else {
            numberSubcarriers = numHighThroughputOfdmDataSubcarriers;
        }//if//
    }//if//

    return (
        static_cast<unsigned int>(
            numberSubcarriers *
            GetFractionalBitsPerSymbol(txParameters.modulationAndCodingScheme)) *
            txParameters.numberSpatialStreams);

}//CalcNumberOfBitsPerOfdmSymbol//


inline
DatarateBitsPerSecType CalcDatarateBitsPerSecond(
    const TimeType& ofdmSymbolDuration,
    const TransmissionParametersType& txParameters)
{
    const unsigned int ofdmSymbolsPerSecond =
        static_cast<unsigned int>(SECOND / ofdmSymbolDuration);

    return (static_cast<DatarateBitsPerSecType>(
        ofdmSymbolsPerSecond * CalcNumberOfBitsPerOfdmSymbol(txParameters)));

}//CalcDatarateBitsPerSecond//


inline
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
    cerr << "Error: Channel model is available for dot11." << endl;
    cerr << "Please confirm that the executable (simulator) disables ITS Module." << endl;
    exit(1);
}//ChooseAndCreateChannelModel//

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

class Dot11MacInterfaceForPhy {
public:
    virtual void BusyChannelAtPhysicalLayerNotification() = 0;
    virtual void ClearChannelAtPhysicalLayerNotification() = 0;
    virtual void TransmissionIsCompleteNotification() = 0;
    virtual void DoSuccessfulTransmissionPostProcessing(const bool wasJustTransmitting) = 0;

    virtual void ReceiveFrameFromPhy(
        const Packet& aFrame, const TransmissionParametersType& receivedFrameTxParameters) = 0;

    virtual void ReceiveAggregatedSubframeFromPhy(
        unique_ptr<Packet>& subframePtr,
        const TransmissionParametersType& receivedFrameTxParameters,
        const unsigned int aggregateFrameSubframeIndex,
        const unsigned int numberSubframes) = 0;

    virtual void NotifyThatPhyReceivedCorruptedFrame() = 0;

    virtual void NotifyThatPhyReceivedCorruptedAggregatedSubframe(
        const TransmissionParametersType& receivedFrameTxParameters,
        const unsigned int aggregateFrameSubframeIndex,
        const unsigned int numberSubframes) = 0;

    virtual bool AggregatedSubframeIsForThisNode(const Packet& frame) const = 0;

};//Dot11MacInterfaceForPhy//


//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------


struct ModulationAndCodingSchemeKey {
    DatarateBitsPerSecType datarateBitsPerSec;
    unsigned int channelBandwidthMhz;
    bool isHighThroughputFrame;

    ModulationAndCodingSchemeKey(
        const DatarateBitsPerSecType& initDatarateBitsPerSec,
        const unsigned int initChannelBandwidthMhz,
        const bool initIsHighThroughputFrame)
        :
        datarateBitsPerSec(initDatarateBitsPerSec),
        channelBandwidthMhz(initChannelBandwidthMhz),
        isHighThroughputFrame(initIsHighThroughputFrame)
    {}

    bool operator<(const ModulationAndCodingSchemeKey& right) const {
        return ((datarateBitsPerSec < right.datarateBitsPerSec) ||
                ((datarateBitsPerSec == right.datarateBitsPerSec) &&
                 (channelBandwidthMhz < right.channelBandwidthMhz)) ||
                ((datarateBitsPerSec == right.datarateBitsPerSec) &&
                 (channelBandwidthMhz == right.channelBandwidthMhz) &&
                 (!isHighThroughputFrame && right.isHighThroughputFrame)));
    }                
};


//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------


class Dot11Phy {
public:
    static const string modelName;

    static const unsigned int phyFrameDataPaddingBits = 16 + 6; // Service and Tail

    struct PropFrameType {
        TransmissionParametersType txParameters;
        // For trace:
        PacketIdType packetId;

        // One of:
        unique_ptr<ScenSim::Packet> macFramePtr;
        unique_ptr<vector<unique_ptr<ScenSim::Packet> > > aggregateFramePtr;
        bool isAMpduAggregate;

        PropFrameType() : isAMpduAggregate(true) { }

    };//PropFrameType//


    typedef SimplePropagationModelForNode<PropFrameType>::IncomingSignal IncomingSignal;

    Dot11Phy(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<SimplePropagationModelForNode<PropFrameType> >& propModelInterfacePtr,
        const shared_ptr<BitOrBlockErrorRateCurveDatabase>& berCurveDatabasePtr,
        const RandomNumberGeneratorSeedType& nodeSeed);


    ~Dot11Phy() {
        propagationModelInterfacePtr->DisconnectThisInterface();
    }

    shared_ptr<Dot11InfoInterface> GetDot11InfoInterface() const
    {
        return shared_ptr<Dot11InfoInterface>(new Dot11InfoInterface(this));
    }

    void SetMacInterfaceForPhy(const shared_ptr<Dot11MacInterfaceForPhy>& initMacLayerPtr)
    {
        macLayerPtr = initMacLayerPtr;
    }    

    map<ModulationAndCodingSchemeKey, ModulationAndCodingSchemesType> GetModulationAndCodingShemeMap();

    shared_ptr<SimplePropagationModelForNode<PropFrameType> > GetPropModelInterface() const { return propagationModelInterfacePtr; }

    InterfaceIdType GetInterfaceId() const { return interfaceId; }

    bool IsReceivingAFrame() const { return (phyState == PhyReceiving); }

    bool IsTransmittingAFrame() const {
        return ((phyState == PhyTxStarting) || (phyState == PhyTransmitting));
    }

    bool ChannelIsClear() const { return (!currentlySensingBusyMedium); }

    void StopReceivingSignalSoCanTransmit();

    void TransmitFrame(
        unique_ptr<Packet>& packetPtr,
        const TransmissionParametersType& txParameters,
        const double& transmitPowerDbm,
        const TimeType& delayUntilAirborne);


    void TransmitAggregateFrame(
        unique_ptr<vector<unique_ptr<Packet> > >& aggregatedFramePtr,
        const bool isAMpduAggregate,
        const TransmissionParametersType& txParameters,
        const double& transmitPowerDbm,
        const TimeType& delayUntilAirborne);

    // Take back only after transmitted, propagated and received by all nodes.


    void TakeOwnershipOfLastTransmittedFrame(unique_ptr<Packet>& framePtr)
    {
        framePtr = move((*this).currentPropagatedFramePtr->macFramePtr);
    }

    bool LastSentFrameWasAggregate() const
        {  return (currentPropagatedFramePtr->aggregateFramePtr != nullptr); }

    void TakeOwnershipOfLastTransmittedAggregateFrame(
        unique_ptr<vector<unique_ptr<Packet> > >& aggregateFramePtr)
    {
        aggregateFramePtr = move((*this).currentPropagatedFramePtr->aggregateFramePtr);
    }

    TimeType CalculatePhysicalLayerHeaderDuration(
        const TransmissionParametersType& txParameters) const;

    TimeType CalculateFrameDataDuration(
        const unsigned int frameLengthBytes,
        const TransmissionParametersType& txParameters) const;

    TimeType CalculateFrameTransmitDuration(
        const unsigned int frameLengthBytes,
        const TransmissionParametersType& txParameters) const;

    TimeType CalculateAggregateFrameTransmitDuration(
        const vector<unique_ptr<ScenSim::Packet> >& aggregateFrame,
        const TransmissionParametersType& txParameters) const;

    TimeType GetSlotDuration() const { return aSlotTimeDuration; }
    TimeType GetShortInterframeSpaceDuration() const { return aShortInterframeSpaceDuration; }
    TimeType GetRxTxTurnaroundTime() const { return aRxTxTurnaroundTime; }
    TimeType GetPhyRxStartDelay() const { return aPhyRxStartDelay; }

    // Part of Slot Duration, but this completely separate parameter provided here for
    // new "TDMA-esque"  802.11 extensions such as 11ad.

    TimeType GetAirPropagationTimeDuration() const { return aAirPropagationTimeDuration; }

    unsigned int GetBaseChannelBandwidthMhz() const { return (baseChannelBandwidthMhz); }
    unsigned int GetChannelCount() const { return propagationModelInterfacePtr->GetChannelCount(); }
    unsigned int GetCurrentChannelNumber() const {
        assert(!currentBondedChannelList.empty());
        return (currentBondedChannelList.front());
    }

    const vector<unsigned int>& GetCurrentBondedChannelList() const {
        return (currentBondedChannelList);
    }


    bool DistributedEmulationChannelIsLocal(const unsigned int channelNum) const {
        return (propagationModelInterfacePtr->ChannelIsBeingUsed(channelNum));
    }

    void SwitchToChannels(const vector<unsigned int>& bondedChannelList);

    void SwitchToChannelNumber(const unsigned int channelNumber)
    {
        vector<unsigned int> channels(1);
        channels[0] = channelNumber;
        (*this).SwitchToChannels(channels);
    }

    void UpdateCcaEnergyDetectionThresholdDbm(const double& newCCAThresholdDbm);

    double GetRssiOfLastFrameDbm() const { return lastReceivedPacketRssiDbm; }
    double GetSinrOfLastFrameDb() const { return lastReceivedPacketSinrDb; }

    TimeType GetOutgoingTransmissionEndTime() const { return outgoingTransmissionEndTime; }

    //double GetMovingAverageOfRssiDbm() const;

    // These methods allow node to stop (and start) receiving frames. Interference is still accumulated.

    void StopReceivingFrames();
    void StartReceivingFrames();
    bool IsNotReceivingFrames() const { return (phyState == PhyIgnoring); }

    // Used by emulator:
    const ScenSim::ObjectMobilityPosition GetPosition() const {
        return (propagationModelInterfacePtr->GetCurrentMobilityPosition());
    }

    unsigned int GetNumberOfReceivedFrames() const { return numberOfReceivedFrames; }
    unsigned int GetNumberOfFramesWithErrors() const { return numberOfFramesWithErrors; }
    unsigned int GetNumberOfSignalCaptures() const { return numberOfSignalCaptures; }

    TimeType GetTotalIdleChannelTime() const { return totalIdleChannelTime; }
    TimeType GetTotalBusyChannelTime() const { return totalBusyChannelTime; }
    TimeType GetTotalTransmissionTime() const { return totalTransmissionTime; }

private:

    static const int SEED_HASH = 23788567;

    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;

    shared_ptr<SimplePropagationModelForNode<PropFrameType> > propagationModelInterfacePtr;

    shared_ptr<BitOrBlockErrorRateCurveDatabase> berCurveDatabasePtr;

    shared_ptr<Dot11MacInterfaceForPhy> macLayerPtr;

    // If (unsynchronized) noise energy is over this threshold, then the station
    // is sensing a "busy medium"
    // Note: these variables should always be the same, but in different units.

    double energyDetectionPowerThresholdDbm;
    double energyDetectionPowerThresholdMw;

    // If the signal energy is over the "preambleDetectionPowerThresholdDbm",
    // then the station will lock on the signal and will try to recieve the
    // frame unless it does not pass the optional SINR based preamble detection
    // condition.  The parameter is also used to differentiate between
    // weak and non-weak signals in a few statistics.

    double preambleDetectionPowerThresholdDbm;

    // Supports optional probablistic preamble detection based on SINR, must first pass
    // the RSSI threshold in "preambleDetectionPowerThresholdDbm" parameter which is
    // the minimum receive power with no interference (thermal noise only).

    ScenSim::InterpolatedTable preambleDetectionProbBySinrTable;

    // This is only used when the preamble was missed (for example, if station was
    // transmitting), the station will do a degraded "lock" on a carrier.
    // Note in OFDM, the radio (as defined in the standard) can detect much lower energy
    // levels when it is locked onto the signal (preamble detection) than when it
    // is not (100X=20dB more sensitive).

    double carrierDetectionPowerThresholdDbm;
    TimeType currentLastCarrierDetectableSignalEnd;

    // Delays

    TimeType aSlotTimeDuration;
    TimeType aShortInterframeSpaceDuration;
    TimeType aRxTxTurnaroundTime;
    TimeType aPreambleLengthDuration;
    TimeType aPhysicalLayerCpHeaderLengthDuration; // aka aPLCPHeaderLength
    TimeType highThroughputPhyHeaderAdditionalDuration;
    TimeType highThroughputPhyHeaderAdditionalPerStreamDuration;
    TimeType ofdmSymbolDuration;

    static const TimeType defaultAirPropagationTimeDuration = 1 * MICRO_SECOND;
    TimeType aAirPropagationTimeDuration;

    TimeType aPhyRxStartDelay;

    unsigned int baseChannelBandwidthMhz;
    unsigned int maxChannelBandwidthMhz;

    vector<unsigned int> currentBondedChannelList;
    unsigned int firstChannelNumber;

    // Standard radio parameter for how much noise the radio circuitry adds.
    double radioNoiseFigureDb;
    double thermalNoisePowerPerBaseChannelMw;
    double thermalNoisePowerAllChannelsMw;

    // New signal must be at least this dB over the signal currently being received to preempt.

    double signalCaptureRatioThresholdDb;

    // Used for error messages:

    NodeIdType nodeId;
    InterfaceIdType interfaceId;

    // Model State variabbles

    enum PhyStateType { PhyScanning, PhyReceiving, PhyTxStarting, PhyTransmitting, PhyIgnoring };

    PhyStateType phyState;

    string phyProtocolString;

    shared_ptr<PropFrameType> currentPropagatedFramePtr;

    // To keep the frame alive for long propagation delays.
    shared_ptr<PropFrameType> previousPropagatedFramePtr;

    DatarateBitsPerSecType outgoingTransmissionDatarateBitsPerSecond;
    double outgoingTransmissionPowerDbm;
    TimeType outgoingTransmissionEndTime;

    double currentSignalPowerDbm;

    // Subtracts power wasted on guard and pilots. Converted to milliwatts.

    double currentAdjustedSignalPowerMilliwatts;


    DatarateBitsPerSecType currentIncomingSignalDatarateBitsPerSec;
    double currentThermalNoisePowerMw;

    vector<shared_ptr<BitErrorRateCurve> > bitErrorCurves;


    bool currentPacketHasAnError;

    vector<IntegralPowerType> currentInterferencePowers;

    TimeType lastErrorCalculationUpdateTime;

    bool currentlySensingBusyMedium;

    TimeType currentIncomingSignalStartTime;
    NodeIdType currentIncomingSignalSourceNodeId;

    TransmissionParametersType currentIncomingSignalTxParameters;

    PacketIdType currentLockedOnFramePacketId;

    // Aggregation

    struct AggregateFrameSubframeInfoElementType {
        unique_ptr<Packet> macFramePtr;
        bool hasError;
        unsigned int lengthBytes;

        AggregateFrameSubframeInfoElementType() : hasError(false) {}

        void operator=(AggregateFrameSubframeInfoElementType&& right)
        {
            assert(this != &right);
            hasError = right.hasError;
            lengthBytes = right.lengthBytes;
            macFramePtr = move(right.macFramePtr);
        }
        AggregateFrameSubframeInfoElementType(AggregateFrameSubframeInfoElementType&& right)
            {  (*this) = move(right); }
    };

    vector<AggregateFrameSubframeInfoElementType> aggregateFrameSubframeInfo;

    unsigned int currentAggregateFrameSubframeIndex;

    // Simulation optimization to avoid copying frames in Phy.

    unique_ptr<Packet> notForMeHeaderOnlyFramePtr;

    RandomNumberGenerator aRandomNumberGenerator;

    //For Stats

    shared_ptr<CounterStatistic> transmittedFramesStatPtr;

    double lastReceivedPacketRssiDbm;
    double lastReceivedPacketSinrDb;

    shared_ptr<RealStatistic> receivedFrameRssiMwStatPtr;
    shared_ptr<RealStatistic> receivedFrameSinrStatPtr;

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

    bool redundantTraceInformationModeIsOn;

    int tracePrecisionDigitsForDbm;

    //-----------------------------------------------------
    class SignalArrivalHandler: public SimplePropagationModelForNode<PropFrameType>::SignalHandler {
    public:
        SignalArrivalHandler(Dot11Phy* initPhyPtr) : phyPtr(initPhyPtr) { }
        void ProcessSignal(const IncomingSignal& aSignal) { phyPtr->ProcessSignalArrivalFromChannel(aSignal); }
    private:
        Dot11Phy* phyPtr;
    };//SignalArrivalHandler//

    class SignalEndHandler: public SimplePropagationModelForNode<PropFrameType>::SignalHandler {
    public:
        SignalEndHandler(Dot11Phy* initPhyPtr) : phyPtr(initPhyPtr) { }
        void ProcessSignal(const IncomingSignal& aSignal) { phyPtr->ProcessSignalEndFromChannel(aSignal); }
    private:
        Dot11Phy* phyPtr;
    };//SignalEndHandler//


    class TransmissionTimerEvent: public SimulationEvent {
    public:
        TransmissionTimerEvent(Dot11Phy* initPhyPtr) : phyPtr(initPhyPtr) { }
        void ExecuteEvent() { phyPtr->StartOrEndTransmission(); }
    private:
        Dot11Phy* phyPtr;
    };//EndOfTransmissionEvent//


    shared_ptr<SimulationEvent> transmissionTimerEventPtr;

    class AggregatedMpduFrameEndEvent: public SimulationEvent {
    public:
        AggregatedMpduFrameEndEvent(Dot11Phy* initPhyPtr) : phyPtr(initPhyPtr) { }
        void ExecuteEvent() { phyPtr->ProcessAggregatedMpduFrameEndEvent(); }
    private:
        Dot11Phy* phyPtr;

    };//AggregatedMpduFrameEndEvent//


    shared_ptr<SimulationEvent> aggregatedMpduFrameEndEventPtr;
    EventRescheduleTicket aggregatedMpduFrameEndEventTicket;


    //-----------------------------------------------------

    void PerformBernoulliTrialBitErrorCalculation(
        const BitErrorRateCurve& bitErrorRateCurve,
        const double& signalToNoiseAndInterferenceRatio,
        const unsigned int numberBits,
        bool& foundAnError);

    double CalcCurrentInterferencePowerMw(
        const TransmissionParametersType& txParameters) const;

    double CalcCurrentInterferencePowerMw() const;

    void UpdatePacketReceptionCalculation();

    void StartTransmission();
    void EndCurrentTransmission();
    void StartOrEndTransmission();
    void ProcessAggregatedMpduFrameEndEvent();

    void SetupReceiveOfMpduAggregateFrame(const PropFrameType& incomingFrame);

    void StartReceivingThisSignal(const IncomingSignal& aSignal);
    void ProcessNewSignal(const IncomingSignal& aSignal);

    void AddSignalPowerToInterferenceLevel(
        const unsigned int signalStartChannelNumber,
        const unsigned int signalNumberChannels,
        const double& signalPowerMw);

    void SubtractSignalPowerFromInterferenceLevel(
        const unsigned int signalStartChannelNumber,
        const unsigned int signalNumberChannels,
        const double& signalPowerMw);

    void AddSignalToInterferenceLevel(const IncomingSignal& aSignal);

    void SubtractSignalFromInterferenceLevel(const IncomingSignal& aSignal);

    void SubtractSignalFromInterferenceLevelAndNotifyMacIfMediumIsClear(const IncomingSignal& aSignal);

    bool IsCurrentlyReceivingThisSignal(const IncomingSignal& aSignal) const;

    void ProcessEndOfTheSignalCurrentlyBeingReceived(const IncomingSignal& aSignal);

    void ProcessSignalArrivalFromChannel(const IncomingSignal& aSignal);
    void ProcessSignalEndFromChannel(const IncomingSignal& aSignal);

    void OutputTraceAndStatsForAddSignalToInterferenceLevel(
        const double& signalPowerDbm,
        const double& adjustedSignalPowerMw,
        const NodeIdType& signalSourceNodeId,
        const PacketIdType& signalPacketId) const;

    void OutputTraceForSubtractSignalFromInterferenceLevel(
        const IncomingSignal& aSignal,
        const double& receivedSignalPowerMw) const;

    void OutputTraceAndStatsForTxStart(
        const Packet& aPacket,
        const double& txPowerDbm,
        const TransmissionParametersType& txParameters,
        const TimeType& duration) const;

    void OutputTraceAndStatsForRxStart(const Packet& aPacket);

    void OutputTraceAndStatsForRxEnd(const PacketIdType& packetId, const bool& rxIsEndedByCapture);

    void ProcessStatsForTxStartingStateTransition();
    void ProcessStatsForEndCurrentTransmission();
    void ProcessStatsForTransitionToBusyChannel();
    void ProcessStatsForTransitionToIdleChannel();

    void DiscardOldestMovingAverageRecord();

    const vector<unsigned int> MakeChannelListFromTxParameters(
        const TransmissionParametersType& txParameters) const;

    double CalcSubcarrierPowerAdjustmentFactorForInterferringFrame(
        const TransmissionParametersType& interferringFrameTxParameters);

    // Parallelism Stuff:

    unsigned int eotIndex;

};//Dot11Phy//



//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

inline
Dot11Phy::Dot11Phy(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& initNodeId,
    const InterfaceIdType& initInterfaceId,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<SimplePropagationModelForNode<PropFrameType> >& initPropModelInterfacePtr,
    const shared_ptr<BitOrBlockErrorRateCurveDatabase>& initBerCurveDatabasePtr,
    const RandomNumberGeneratorSeedType& nodeSeed)
    :
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    propagationModelInterfacePtr(initPropModelInterfacePtr),
    berCurveDatabasePtr(initBerCurveDatabasePtr),
    nodeId(initNodeId),
    interfaceId(initInterfaceId),
    phyState(PhyScanning),
    outgoingTransmissionDatarateBitsPerSecond(0),
    outgoingTransmissionPowerDbm(0.0),
    outgoingTransmissionEndTime(ZERO_TIME),
    currentlySensingBusyMedium(false),
    lastErrorCalculationUpdateTime(ZERO_TIME),
    currentIncomingSignalStartTime(ZERO_TIME),
    currentLastCarrierDetectableSignalEnd(ZERO_TIME),
    currentIncomingSignalDatarateBitsPerSec(0),
    currentSignalPowerDbm(-DBL_MAX),
    currentAdjustedSignalPowerMilliwatts(0.0),
    currentPropagatedFramePtr(new PropFrameType()),
    previousPropagatedFramePtr(new PropFrameType()),
    currentPacketHasAnError(false),
    currentIncomingSignalSourceNodeId(0),
    aRandomNumberGenerator(HashInputsToMakeSeed(nodeSeed, initInterfaceId, SEED_HASH)),
    numberOfReceivedFrames(0),
    numberOfFramesWithErrors(0),
    numberOfSignalCaptures(0),
    transmittedFramesStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_FramesTransmitted"))),
    receivedFrameRssiMwStatPtr(
        simulationEngineInterfacePtr->CreateRealStatWithDbConversion(
            (modelName + '_' + interfaceId + "_ReceivedFrameRssiDbm"))),
    receivedFrameSinrStatPtr(
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
    redundantTraceInformationModeIsOn(false),
    tracePrecisionDigitsForDbm(8)
{
    phyProtocolString =
        theParameterDatabaseReader.ReadString(
            (parameterNamePrefix + "phy-protocol"),
            nodeId,
            interfaceId);

    ConvertStringToUpperCase(phyProtocolString);

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "base-channel-bandwidth-mhz"), nodeId, interfaceId)) {

        // For the unusual scenario of 802.11a/g/n/ac and 802.11p coexistence.

        baseChannelBandwidthMhz =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "base-channel-bandwidth-mhz"), nodeId, interfaceId);
    }
    else {
        // Initialize channel bandwidth to first channel.

        const double firstChannelsChannelBandwidthMhz =
           propagationModelInterfacePtr->GetChannelBandwidthMhz(
               propagationModelInterfacePtr->GetBaseChannelNumber());

        baseChannelBandwidthMhz = RoundToUint(firstChannelsChannelBandwidthMhz);

        if (fabs(firstChannelsChannelBandwidthMhz - baseChannelBandwidthMhz) > DBL_EPSILON) {

            cerr << "Error in Dot11 Model: Channel bandwidth must be multiple of 1 MHz." << endl;
            exit(1);
        }//if//
    }//if//

    maxChannelBandwidthMhz = baseChannelBandwidthMhz;

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "max-channel-bandwidth-mhz"),nodeId, interfaceId)) {

        maxChannelBandwidthMhz =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "max-channel-bandwidth-mhz"), nodeId, interfaceId);
    }//if//

    radioNoiseFigureDb =
        theParameterDatabaseReader.ReadDouble(
            (parameterNamePrefix + "radio-noise-figure-db"),
            nodeId,
            interfaceId);

    thermalNoisePowerPerBaseChannelMw =
        CalculateThermalNoisePowerWatts(radioNoiseFigureDb, baseChannelBandwidthMhz) * 1000.0;

    thermalNoisePowerAllChannelsMw =
        CalculateThermalNoisePowerWatts(radioNoiseFigureDb, maxChannelBandwidthMhz) * 1000.0;

    currentThermalNoisePowerMw = thermalNoisePowerAllChannelsMw;

    energyDetectionPowerThresholdDbm =
        theParameterDatabaseReader.ReadDouble(
            (parameterNamePrefix + "energy-detection-power-threshold-dbm"),
            nodeId,
            interfaceId);

    energyDetectionPowerThresholdMw = ConvertToNonDb(energyDetectionPowerThresholdDbm);

    preambleDetectionPowerThresholdDbm =
        theParameterDatabaseReader.ReadDouble(
            (parameterNamePrefix + "preamble-detection-power-threshold-dbm"),
            nodeId,
            interfaceId);

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "preamble-detection-probability-for-sinr-db-table"),
            nodeId,
            interfaceId)) {

        string tableString =
            theParameterDatabaseReader.ReadString(
                (parameterNamePrefix + "preamble-detection-probability-for-sinr-db-table"),
                nodeId,
                interfaceId);

        DeleteTrailingSpaces(tableString);

        bool success;
        map<double, double> preambleDetectionProbForSinrDbMap;

        ScenSim::ConvertAStringSequenceOfRealValuedPairsIntoAMap(
            tableString,
            success,
            preambleDetectionProbForSinrDbMap);

        if (!success) {
            cerr << "Error in "
                 << (parameterNamePrefix + "preamble-detection-probability-for-sinr-db-table")
                 << " parameter:" << endl;
            cerr << "    Value= \"" << tableString << "\"" << endl;
            cerr << "    Example string: \"-5.0:0.0 -4.0:0.25 -3.0:0.5 -2.0:0.75 -1.0:1.0\"" << endl;
            exit(1);
        }//if//

        typedef map<double, double>::const_iterator IterType;

        for(IterType iter = preambleDetectionProbForSinrDbMap.begin();
             (iter != preambleDetectionProbForSinrDbMap.end()); ++iter) {

             preambleDetectionProbBySinrTable.AddDatapoint(
                 ConvertToNonDb(iter->first),
                 iter->second);
        }//for//
    }//if//


    //Default Disabled
    carrierDetectionPowerThresholdDbm = DBL_MAX;

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "carrier-detection-power-threshold-dbm"), nodeId, interfaceId)) {

        carrierDetectionPowerThresholdDbm =
            theParameterDatabaseReader.ReadDouble(
                (parameterNamePrefix + "carrier-detection-power-threshold-dbm"),
                nodeId,
                interfaceId);
    }//if//

    signalCaptureRatioThresholdDb =
        theParameterDatabaseReader.ReadDouble(
            (parameterNamePrefix + "signal-capture-ratio-threshold-db"),
            nodeId,
            interfaceId);

    ofdmSymbolDuration =
        theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "ofdm-symbol-duration"),
            nodeId,
            interfaceId);

    aSlotTimeDuration =
        theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "slot-time"),
            nodeId,
            interfaceId);

    aAirPropagationTimeDuration = defaultAirPropagationTimeDuration;
    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "air-propagation-time"), nodeId, interfaceId)) {

        aAirPropagationTimeDuration =
            theParameterDatabaseReader.ReadTime(
                (parameterNamePrefix + "air-propagation-time"),
                nodeId,
                interfaceId);
    }//if//


    aShortInterframeSpaceDuration =
        theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "sifs-time"),
            nodeId,
            interfaceId);

    aRxTxTurnaroundTime =
        theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "rx-tx-turnaround-time"),
            nodeId,
            interfaceId);

    aPhyRxStartDelay =
        theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "phy-rx-start-delay"),
            nodeId,
            interfaceId);

    aPreambleLengthDuration =
        theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "preamble-length-duration"),
            nodeId,
            interfaceId);

    aPhysicalLayerCpHeaderLengthDuration =
        theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "plcp-header-length-duration"),
            nodeId,
            interfaceId);

    highThroughputPhyHeaderAdditionalDuration =
        theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "phy-high-throughput-header-additional-duration"),
            nodeId,
            interfaceId);

    highThroughputPhyHeaderAdditionalPerStreamDuration =
        theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "phy-high-throughput-header-additional-per-stream-duration"),
            nodeId,
            interfaceId);

    assert((NumberModAndCodingSchemes-1) == static_cast<unsigned int>(maxModulationAndCodingScheme));

    bitErrorCurves.resize(NumberModAndCodingSchemes);

    for(ModulationAndCodingSchemesType mcs = minModulationAndCodingScheme;
        (mcs <= maxModulationAndCodingScheme);
        IncrementModAndCodingScheme(mcs)) {

        bitErrorCurves.at(static_cast<size_t>(mcs)) =
            berCurveDatabasePtr->GetBerCurve(
                MakeUpperCaseString(phyProtocolString),
                MakeUpperCaseString(GetModulationAndCodingName(mcs)));
    }//for//


    transmissionTimerEventPtr.reset(new TransmissionTimerEvent(this));
    propagationModelInterfacePtr->RegisterSignalHandler(
        unique_ptr<SimplePropagationModelForNode<PropFrameType>::SignalHandler>(
            new SignalArrivalHandler(this)));
    propagationModelInterfacePtr->RegisterSignalEndHandler(
        unique_ptr<SimplePropagationModelForNode<PropFrameType>::SignalHandler>(
            new SignalEndHandler(this)));

    aggregatedMpduFrameEndEventPtr.reset(new AggregatedMpduFrameEndEvent(this));

    // Parallelism Stuff:

    (*this).eotIndex = (*this).simulationEngineInterfacePtr->AllocateEarliestOutputTimeIndex();
    simulationEngineInterfacePtr->SetAnEarliestOutputTimeForThisNode(INFINITE_TIME, eotIndex);

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "redundant-trace-information-mode"))) {

        redundantTraceInformationModeIsOn =
            theParameterDatabaseReader.ReadBool(
                (parameterNamePrefix + "redundant-trace-information-mode"));
    }//if//

    notForMeHeaderOnlyFramePtr = Packet::CreatePacketWithoutSimInfo(CommonFrameHeaderType());

}//Dot11Phy()//


inline
map<ModulationAndCodingSchemeKey, ModulationAndCodingSchemesType> Dot11Phy::GetModulationAndCodingShemeMap()
{
    vector<unsigned int> channelBandwidthMhzSet;
    vector<bool> isHighThroughputFrameFlags;

    channelBandwidthMhzSet.push_back(5);
    channelBandwidthMhzSet.push_back(10);
    channelBandwidthMhzSet.push_back(20);
    channelBandwidthMhzSet.push_back(40);

    isHighThroughputFrameFlags.push_back(false);
    isHighThroughputFrameFlags.push_back(true);

    TransmissionParametersType txParams;
    map<ModulationAndCodingSchemeKey, ModulationAndCodingSchemesType> modulrationAndCodingSchemeMap;

    for(unsigned int i = 0; i < NumberModAndCodingSchemes; i++) {
        const ModulationAndCodingSchemesType modulationAndCodingScheme = ModulationAndCodingSchemesType(i);

        for(size_t j = 0; j < channelBandwidthMhzSet.size(); j++) {
            const unsigned int channelBandwidthMhz = channelBandwidthMhzSet[j];
            
            for(size_t k = 0; k < isHighThroughputFrameFlags.size(); k++) {
                const bool isHighThroughputFrame = isHighThroughputFrameFlags[k];

                if ((isHighThroughputFrame) ||
                    ((!isHighThroughputFrame) && channelBandwidthMhz <= 20)) {
                    
                    txParams.channelBandwidthMhz = channelBandwidthMhz;
                    txParams.isHighThroughputFrame = isHighThroughputFrame;
                    txParams.modulationAndCodingScheme = modulationAndCodingScheme;
                    
                    const DatarateBitsPerSecType datarateBps =
                        CalcDatarateBitsPerSecond(ofdmSymbolDuration, txParams);
                    
                    const ModulationAndCodingSchemeKey mcsKey(
                        datarateBps, channelBandwidthMhz, isHighThroughputFrame);
                    
                    modulrationAndCodingSchemeMap[mcsKey] = modulationAndCodingScheme;
                }//if//
            }//for//
        }//for//
    }//for//

    return modulrationAndCodingSchemeMap;

}//GetModulationAndCodingShemeMap//


inline
TimeType Dot11Phy::CalculatePhysicalLayerHeaderDuration(
    const TransmissionParametersType& txParameters) const
{
    if (txParameters.isHighThroughputFrame) {
        return
            (aPreambleLengthDuration +
             aPhysicalLayerCpHeaderLengthDuration +
             highThroughputPhyHeaderAdditionalDuration +
             (highThroughputPhyHeaderAdditionalPerStreamDuration * txParameters.numberSpatialStreams));
    }
    else {
        return (aPreambleLengthDuration + aPhysicalLayerCpHeaderLengthDuration);

    }//if//

}//CalculatePhysicalLayerHeaderDuration//


inline
TimeType Dot11Phy::CalculateFrameDataDuration(
    const unsigned int frameLengthBytes,
    const TransmissionParametersType& txParameters) const
{
    const unsigned int numberFrameBits = ((frameLengthBytes * 8) + phyFrameDataPaddingBits);

    const unsigned int numberOfOfdmBitsPerSymbol = CalcNumberOfBitsPerOfdmSymbol(txParameters);

    const unsigned int numberOfOfdmSymbols =
        DivideAndRoundUp(numberFrameBits, numberOfOfdmBitsPerSymbol);

    return (numberOfOfdmSymbols * ofdmSymbolDuration);

}//CalculateFrameDataDuration//


inline
TimeType Dot11Phy::CalculateFrameTransmitDuration(
    const unsigned int frameLengthBytes,
    const TransmissionParametersType& txParameters) const
{
    return
        (CalculatePhysicalLayerHeaderDuration(txParameters) +
         CalculateFrameDataDuration(frameLengthBytes, txParameters));

}//CalculateFrameTransmitDuration//



inline
TimeType Dot11Phy::CalculateAggregateFrameTransmitDuration(
    const vector<unique_ptr<ScenSim::Packet> >& aggregateFrame,
    const TransmissionParametersType& txParameters) const
{
    assert(!aggregateFrame.empty());

    TimeType duration =
        CalculateFrameTransmitDuration(aggregateFrame[0]->LengthBytes(), txParameters);

    for(unsigned int i = 1; (i < aggregateFrame.size()); i++) {
        duration +=
            CalculateFrameDataDuration(aggregateFrame[i]->LengthBytes(), txParameters);
    }//for//

    return (duration);

}//CalculateAggregateFrameTransmitDuration//





inline
void Dot11Phy::ProcessStatsForTxStartingStateTransition()
{
    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
    if (currentlySensingBusyMedium) {
        (*this).totalBusyChannelTime += (currentTime - lastChannelStateTransitionTime);
    }
    else {
        (*this).totalIdleChannelTime += (currentTime - lastChannelStateTransitionTime);
    }//if//

    (*this).lastChannelStateTransitionTime = currentTime;

}//ProcessStatsForTxStartingStateTransition//


inline
void Dot11Phy::StopReceivingSignalSoCanTransmit()
{
    signalsDuringTransmissionStatPtr->IncrementCounter();

    (*this).AddSignalPowerToInterferenceLevel(
        currentIncomingSignalTxParameters.firstChannelNumber,
        (currentIncomingSignalTxParameters.channelBandwidthMhz / baseChannelBandwidthMhz),
        currentAdjustedSignalPowerMilliwatts);

    OutputTraceAndStatsForAddSignalToInterferenceLevel(
        currentSignalPowerDbm,
        currentAdjustedSignalPowerMilliwatts,
        currentIncomingSignalSourceNodeId,
        currentLockedOnFramePacketId);

    (*this).currentSignalPowerDbm = -DBL_MAX;
    (*this).currentAdjustedSignalPowerMilliwatts = 0.0;
    (*this).currentThermalNoisePowerMw = thermalNoisePowerAllChannelsMw;
    (*this).currentIncomingSignalSourceNodeId = 0;
    (*this).currentLockedOnFramePacketId = PacketIdType::nullPacketId;
    (*this).currentIncomingSignalDatarateBitsPerSec = 0;

    (*this).phyState = PhyTxStarting;

}//StopReceivingSignalSoCanTransmit//



inline
void Dot11Phy::TransmitFrame(
    unique_ptr<Packet>& packetPtr,
    const TransmissionParametersType& txParameters,
    const double& transmitPowerDbm,
    const TimeType& delayUntilAirborne)
{
    assert((phyState != PhyTxStarting) && (phyState != PhyTransmitting));

    if (phyState == PhyReceiving) {
        (*this).StopReceivingSignalSoCanTransmit();
    }//if//

    (*this).phyState = PhyTxStarting;

    (*this).ProcessStatsForTxStartingStateTransition();

    std::swap((*this).currentPropagatedFramePtr, (*this).previousPropagatedFramePtr);

    (*this).currentPropagatedFramePtr->macFramePtr = move(packetPtr);

    if (currentPropagatedFramePtr->aggregateFramePtr != nullptr) {
        (*this).currentPropagatedFramePtr->aggregateFramePtr.reset();
    }//if//

    (*this).currentPropagatedFramePtr->txParameters = txParameters;
    (*this).currentPropagatedFramePtr->packetId = currentPropagatedFramePtr->macFramePtr->GetPacketId();

    outgoingTransmissionPowerDbm = transmitPowerDbm;

    const TimeType startTransmissionTime = simulationEngineInterfacePtr->CurrentTime() + delayUntilAirborne;
    simulationEngineInterfacePtr->ScheduleEvent(transmissionTimerEventPtr, startTransmissionTime);

    (*this).ProcessStatsForTxStartingStateTransition();

    outgoingTransmissionEndTime =
        startTransmissionTime +
        CalculateFrameTransmitDuration(
            currentPropagatedFramePtr->macFramePtr->LengthBytes(),
            currentPropagatedFramePtr->txParameters);

    // Parallelism Stuff: Transmission is imminent.

    //Parallel simulationEngineInterfacePtr->SetAnEarliestOutputTimeForThisNode(
    //Parallel     (simulationEngineInterfacePtr->CurrentTime() + delayUntilAirborne),
    //Parallel    eotIndex);

}//TransmitFrame//



inline
void Dot11Phy::TransmitAggregateFrame(
    unique_ptr<vector<unique_ptr<Packet> > >& aggregateFramePtr,
    const bool isAMpduAggregate,
    const TransmissionParametersType& txParameters,
    const double& transmitPowerDbm,
    const TimeType& delayUntilAirborne)
{
    assert((phyState != PhyTxStarting) && (phyState != PhyTransmitting));

    if (phyState == PhyReceiving) {
        (*this).StopReceivingSignalSoCanTransmit();
    }//if//

    (*this).phyState = PhyTxStarting;

    (*this).ProcessStatsForTxStartingStateTransition();

    std::swap((*this).currentPropagatedFramePtr, (*this).previousPropagatedFramePtr);

    (*this).currentPropagatedFramePtr->aggregateFramePtr = move(aggregateFramePtr);

    if (currentPropagatedFramePtr->macFramePtr != nullptr) {
        (*this).currentPropagatedFramePtr->macFramePtr.reset();
    }//if//

    (*this).currentPropagatedFramePtr->txParameters = txParameters;
    (*this).currentPropagatedFramePtr->packetId =
        currentPropagatedFramePtr->aggregateFramePtr->front()->GetPacketId();
    (*this).currentPropagatedFramePtr->isAMpduAggregate = isAMpduAggregate;

    outgoingTransmissionPowerDbm = transmitPowerDbm;

    TimeType startTransmissionTime = simulationEngineInterfacePtr->CurrentTime() + delayUntilAirborne;
    simulationEngineInterfacePtr->ScheduleEvent(transmissionTimerEventPtr, startTransmissionTime);

    (*this).ProcessStatsForTxStartingStateTransition();

    // Parallelism Stuff: Transmission is imminent.

    //Parallel simulationEngineInterfacePtr->SetAnEarliestOutputTimeForThisNode(
    //Parallel    (simulationEngineInterfacePtr->CurrentTime() + delayUntilAirborne),
    //Parallel    eotIndex);

}//TransmitAggregateFrame//



inline
const vector<unsigned int> Dot11Phy::MakeChannelListFromTxParameters(
    const TransmissionParametersType& txParameters) const
{
    const unsigned int numberChannels = (txParameters.channelBandwidthMhz / baseChannelBandwidthMhz);
    assert(numberChannels > 0);
    vector<unsigned int> channelList(numberChannels);
    for (unsigned int i = 0; (i < numberChannels); i++) {
        channelList[i] = txParameters.firstChannelNumber + i;
    }//for//

    return channelList;

}//MakeChannelListFromTxParameters//



inline
double Dot11Phy::CalcSubcarrierPowerAdjustmentFactorForInterferringFrame(
    const TransmissionParametersType& interferringFrameTxParameters)
{
    const unsigned int receiveChannelBandwidthMhz =
        static_cast<unsigned int>(baseChannelBandwidthMhz * currentBondedChannelList.size());

    if (interferringFrameTxParameters.channelBandwidthMhz == receiveChannelBandwidthMhz) {

        assert((phyState != PhyReceiving) ||
               (interferringFrameTxParameters.isHighThroughputFrame ==
                currentIncomingSignalTxParameters.isHighThroughputFrame));

        return (CalcPilotSubcarriersPowerAdjustmentFactor(interferringFrameTxParameters));

    }
    else if (interferringFrameTxParameters.channelBandwidthMhz == 20) {

        assert(interferringFrameTxParameters.isHighThroughputFrame);

        return (ofdm20MhzHtOn40MhzSubcarrierInterferenceFactor);
    }
    else {
        assert(interferringFrameTxParameters.channelBandwidthMhz == 40);
        assert(interferringFrameTxParameters.isHighThroughputFrame);

        return (ofdm40MhzOn20MhzHtSubcarrierInterferenceFactor);
    }//if//

    assert(false); abort(); return 0.0;

}//CalcSubcarrierPowerAdjustmentFactorForInterferringFrame//


inline
void Dot11Phy::StartTransmission()
{
    assert(phyState == PhyTxStarting);
    phyState = PhyTransmitting;

    // The medium is busy because we are transmitting on it!
    (*this).currentlySensingBusyMedium = true;


    TimeType duration;

    if (currentPropagatedFramePtr->macFramePtr != nullptr) {
        duration =
            CalculateFrameTransmitDuration(
                currentPropagatedFramePtr->macFramePtr->LengthBytes(),
                currentPropagatedFramePtr->txParameters);
    }
    else {
        duration =
            CalculateAggregateFrameTransmitDuration(
                *currentPropagatedFramePtr->aggregateFramePtr,
                currentPropagatedFramePtr->txParameters);
    }//if//


    if (currentPropagatedFramePtr->macFramePtr != nullptr) {
        OutputTraceAndStatsForTxStart(
            *currentPropagatedFramePtr->macFramePtr,
            outgoingTransmissionPowerDbm,
            currentPropagatedFramePtr->txParameters,
            duration);
    }
    else {
        assert(currentPropagatedFramePtr->aggregateFramePtr != nullptr);

        for(unsigned int i = 0; (i < currentPropagatedFramePtr->aggregateFramePtr->size()); i++) {

            OutputTraceAndStatsForTxStart(
                *(*currentPropagatedFramePtr->aggregateFramePtr)[i],
                outgoingTransmissionPowerDbm,
                currentPropagatedFramePtr->txParameters,
                duration);

        }//for//
    }//if//


    if (currentBondedChannelList.empty()) {
        propagationModelInterfacePtr->TransmitSignal(
            outgoingTransmissionPowerDbm, duration, currentPropagatedFramePtr);
    }
    else {
        propagationModelInterfacePtr->TransmitSignal(
            MakeChannelListFromTxParameters(currentPropagatedFramePtr->txParameters),
            outgoingTransmissionPowerDbm,
            duration,
            currentPropagatedFramePtr);
    }//if//

    TimeType endOfTransmissionTime = simulationEngineInterfacePtr->CurrentTime() + duration;

    simulationEngineInterfacePtr->ScheduleEvent(transmissionTimerEventPtr, endOfTransmissionTime);

    // Parallelism: Delete stale Earliest Output Time.

    (*this).simulationEngineInterfacePtr->SetAnEarliestOutputTimeForThisNode(INFINITE_TIME, eotIndex);

}//StartTransmission//



inline
void Dot11Phy::StartOrEndTransmission()
{
    if (phyState == PhyTxStarting) {
        (*this).StartTransmission();
    }
    else if (phyState == PhyTransmitting) {
        (*this).EndCurrentTransmission();
    }
    else {
        assert(false); abort();
    }//if//
}

inline
void Dot11Phy::PerformBernoulliTrialBitErrorCalculation(
    const BitErrorRateCurve& bitErrorRateCurve,
    const double& signalToNoiseAndInterferenceRatio,
    const unsigned int numberBits,
    bool& foundAnError)
{
    if (numberBits == 0) {
        foundAnError = false;
    }
    else {
        const double bitErrorRate = bitErrorRateCurve.CalculateBitErrorRate(signalToNoiseAndInterferenceRatio);

        const double probabilityOfZeroErrors = pow((1.0-bitErrorRate), static_cast<int>(numberBits));
        const double randomNumber = (*this).aRandomNumberGenerator.GenerateRandomDouble();

        foundAnError = (randomNumber > probabilityOfZeroErrors);

    }//if//

}//PerformBernoulliTrialBitErrorCalculation//


inline
double Dot11Phy::CalcCurrentInterferencePowerMw(const TransmissionParametersType& txParameters) const
{
    // Calc interference relative to a signal (could be less than full bandwidth).

    const unsigned int startIndex = (txParameters.firstChannelNumber - firstChannelNumber);

    const unsigned int numberChannels = (txParameters.channelBandwidthMhz / baseChannelBandwidthMhz);

    assert(startIndex < numberChannels);

    IntegralPowerType totalInterferencePower(0.0);

    for(unsigned int i = 0; (i < numberChannels); i++) {
        totalInterferencePower += currentInterferencePowers[startIndex + i];
    }//for//

    return (totalInterferencePower.ConvertToMilliwatts());

}//CalcCurrentInterferencePowerMw//


inline
double Dot11Phy::CalcCurrentInterferencePowerMw() const
{
    if (currentLockedOnFramePacketId != PacketIdType::nullPacketId) {
        return (CalcCurrentInterferencePowerMw(currentIncomingSignalTxParameters));
    }
    else {
        // Don't restrict interference power to current signal's channels.

        IntegralPowerType totalInterferencePower(0.0);

        for(unsigned int i = 0; (i < currentInterferencePowers.size()); i++) {
            totalInterferencePower += currentInterferencePowers[i];
        }//for//

        return (totalInterferencePower.ConvertToMilliwatts());

    }//if//

}//CalcCurrentInterferencePowerMw//


inline
void Dot11Phy::UpdatePacketReceptionCalculation()
{
    using std::max;
    using std::min;

    if (!currentPacketHasAnError) {

        const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
        const TimeType startHeaderTime = (currentIncomingSignalStartTime + aPreambleLengthDuration);
        const TimeType startDataTime = startHeaderTime + aPhysicalLayerCpHeaderLengthDuration;

        if (currentTime <= startHeaderTime) {
            // Still in preamble, nothing to do.
            return;
        }//if//

        const double totalNoiseAndInterferencePowerMw =
            (currentThermalNoisePowerMw + CalcCurrentInterferencePowerMw());

        const double signalToNoiseAndInterferenceRatio =
            (currentAdjustedSignalPowerMilliwatts / totalNoiseAndInterferencePowerMw);

        assert(currentTime > startHeaderTime);

        if (lastErrorCalculationUpdateTime < startDataTime) {

            const TimeType timeInHeader =
                min(currentTime, startDataTime) -
                max(lastErrorCalculationUpdateTime, startHeaderTime);

            const unsigned int numberOfBits =
                RoundToUint(
                    (ConvertTimeToDoubleSecs(timeInHeader) * currentIncomingSignalDatarateBitsPerSec));

            (*this).PerformBernoulliTrialBitErrorCalculation(
                *bitErrorCurves[static_cast<size_t>(minModulationAndCodingScheme)],
                signalToNoiseAndInterferenceRatio,
                numberOfBits,
                currentPacketHasAnError);

            if (currentPacketHasAnError) {
                return;
            }//if//

        }//if//

        if (currentTime > startDataTime) {
            const TimeType timeInDataPart =
                currentTime -
                max(lastErrorCalculationUpdateTime, startDataTime);

            const unsigned int numberOfBits =
                RoundToUint(
                    (ConvertTimeToDoubleSecs(timeInDataPart) * currentIncomingSignalDatarateBitsPerSec));

            (*this).PerformBernoulliTrialBitErrorCalculation(
                *bitErrorCurves.at(static_cast<size_t>(
                    currentIncomingSignalTxParameters.modulationAndCodingScheme)),
                signalToNoiseAndInterferenceRatio,
                numberOfBits,
                currentPacketHasAnError);
        }//if//

        (*this).lastErrorCalculationUpdateTime = simulationEngineInterfacePtr->CurrentTime();

    }//if//

}//UpdatePacketReceptionCalculation//


inline
void Dot11Phy::AddSignalPowerToInterferenceLevel(
    const unsigned int signalStartChannelNumber,
    const unsigned int signalNumberChannels,
    const double& signalPowerMw)
{
    if (signalStartChannelNumber < firstChannelNumber) {
        return;
    }//if//

    if (phyState == PhyReceiving) {
        (*this).UpdatePacketReceptionCalculation();
    }//if//

    const double signalPowerPerChannel = (signalPowerMw / signalNumberChannels);

    const unsigned int startIndex = signalStartChannelNumber - firstChannelNumber;

    for (unsigned int i = 0; (i < signalNumberChannels); i++) {
        if ((startIndex + i) >= currentInterferencePowers.size()) {
            break;
        }//if//

        (*this).currentInterferencePowers[startIndex + i] += signalPowerPerChannel;
    }//for//

}//AddSignalPowerToInterferenceLevel//


inline
void Dot11Phy::AddSignalToInterferenceLevel(const IncomingSignal& aSignal)
{
    PacketIdType packetId;
    double signalPowerMw;

    if (aSignal.HasAFrame()) {
        const PropFrameType& aFrame = aSignal.GetFrame();

        // Assuming pilots don't contribute to interference.

        signalPowerMw =
            (ConvertToNonDb(aSignal.GetReceivedPowerDbm()) *
             CalcSubcarrierPowerAdjustmentFactorForInterferringFrame(aFrame.txParameters));

        (*this).AddSignalPowerToInterferenceLevel(
            aFrame.txParameters.firstChannelNumber,
            (aFrame.txParameters.channelBandwidthMhz / baseChannelBandwidthMhz),
            signalPowerMw);

        packetId = aFrame.packetId;

        // Carrier Detection logic.

        if (aSignal.GetReceivedPowerDbm() >= carrierDetectionPowerThresholdDbm) {
            const TimeType signalEndTime =
                (simulationEngineInterfacePtr->CurrentTime() + aSignal.GetDuration());

            if (signalEndTime > currentLastCarrierDetectableSignalEnd) {
                currentLastCarrierDetectableSignalEnd = signalEndTime;
            }//if//
        }//if//
    }
    else {
        // Raw noise/interference signal (Not a Dot11 frame signal).

        signalPowerMw = ConvertToNonDb(aSignal.GetReceivedPowerDbm());

        (*this).AddSignalPowerToInterferenceLevel(
            aSignal.GetChannelNumber(),
            aSignal.GetNumberBondedChannels(),
            signalPowerMw);

    }//if//

    OutputTraceAndStatsForAddSignalToInterferenceLevel(
        aSignal.GetReceivedPowerDbm(),
        signalPowerMw,
        aSignal.GetSourceNodeId(),
        packetId);

}//AddSignalToInterferenceLevel//


inline
void Dot11Phy::SubtractSignalPowerFromInterferenceLevel(
    const unsigned int signalStartChannelNumber,
    const unsigned int signalNumberChannels,
    const double& signalPowerMw)
{
    if (signalStartChannelNumber < firstChannelNumber) {
        return;
    }//if//

    if (phyState == PhyReceiving) {
        (*this).UpdatePacketReceptionCalculation();
    }//if//

    const double signalPowerPerChannel = (signalPowerMw / signalNumberChannels);

    const unsigned int startIndex = signalStartChannelNumber - firstChannelNumber;

    for (unsigned int i = 0; (i < signalNumberChannels); i++) {
        if ((startIndex + i) >= currentInterferencePowers.size()) {
            break;
        }//if//

        (*this).currentInterferencePowers[startIndex + i] -= signalPowerPerChannel;
    }//for//

}//SubtractSignalPowerFromInterferenceLevel//


inline
void Dot11Phy::SubtractSignalFromInterferenceLevel(const IncomingSignal& aSignal)
{
    double signalPowerMw;

    if (aSignal.HasAFrame()) {
        const PropFrameType& aFrame = aSignal.GetFrame();

        // Assuming pilots don't contribute to interference.

        signalPowerMw =
            (ConvertToNonDb(aSignal.GetReceivedPowerDbm()) *
             CalcSubcarrierPowerAdjustmentFactorForInterferringFrame(aFrame.txParameters));

        (*this).SubtractSignalPowerFromInterferenceLevel(
            aFrame.txParameters.firstChannelNumber,
            (aFrame.txParameters.channelBandwidthMhz / baseChannelBandwidthMhz),
            signalPowerMw);
    }
    else {
        // Raw noise/interference signal (Not a Dot11 frame signal).

        signalPowerMw = ConvertToNonDb(aSignal.GetReceivedPowerDbm());

        (*this).SubtractSignalPowerFromInterferenceLevel(
            aSignal.GetChannelNumber(),
            aSignal.GetNumberBondedChannels(),
            signalPowerMw);
    }//if//

    OutputTraceForSubtractSignalFromInterferenceLevel(aSignal, signalPowerMw);

}//SubtractSignalFromInterferenceLevel//



inline
void Dot11Phy::SubtractSignalFromInterferenceLevelAndNotifyMacIfMediumIsClear(
    const IncomingSignal& aSignal)
{
    (*this).SubtractSignalFromInterferenceLevel(aSignal);

    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

    if ((phyState == PhyScanning) &&
        (currentlySensingBusyMedium) &&
        ((currentTime >= currentLastCarrierDetectableSignalEnd) &&
         (CalcCurrentInterferencePowerMw() < energyDetectionPowerThresholdMw))) {

        (*this).currentlySensingBusyMedium = false;
        (*this).ProcessStatsForTransitionToIdleChannel();

        macLayerPtr->ClearChannelAtPhysicalLayerNotification();

    }//if//

}//SubtractSignalFromInterferenceLevelAndNotifyMacIfMediumIsClear//



inline
void Dot11Phy::SetupReceiveOfMpduAggregateFrame(const PropFrameType& incomingFrame)
{
    assert(incomingFrame.macFramePtr == nullptr);
    assert(incomingFrame.isAMpduAggregate);

    // Simulation Optimization: Peek (extra-simulation) at frame to avoid useless packet copies.

    const bool frameIsForMe =
        macLayerPtr->AggregatedSubframeIsForThisNode(*incomingFrame.aggregateFramePtr->front());

    if (!frameIsForMe) {
        // Just copy header into bogus "header-only" frame.

        (*this).notForMeHeaderOnlyFramePtr->GetAndReinterpretPayloadData<CommonFrameHeaderType>() =
            incomingFrame.aggregateFramePtr->front()->GetAndReinterpretPayloadData<CommonFrameHeaderType>(
                sizeof(MpduDelimiterFrameType));

        assert(aggregateFrameSubframeInfo.empty());
        return;
    }//if//

    (*this).aggregateFrameSubframeInfo.resize(incomingFrame.aggregateFramePtr->size());

    for(unsigned int i = 0; (i < incomingFrame.aggregateFramePtr->size()); i++) {
        AggregateFrameSubframeInfoElementType& info = (*this).aggregateFrameSubframeInfo[i];

        info.lengthBytes = (*incomingFrame.aggregateFramePtr)[i]->LengthBytes();
        info.hasError = false;

        assert(info.macFramePtr == nullptr);

        info.macFramePtr.reset(new Packet(*(*incomingFrame.aggregateFramePtr)[i]));
    }//for//

    (*this).currentAggregateFrameSubframeIndex = 0;

    if (incomingFrame.aggregateFramePtr->size() > 1) {

        const TimeType subframeEndTime =
            (simulationEngineInterfacePtr->CurrentTime() +
             CalculateFrameTransmitDuration(
                 incomingFrame.aggregateFramePtr->front()->LengthBytes(),
                 incomingFrame.txParameters));

        simulationEngineInterfacePtr->ScheduleEvent(
            aggregatedMpduFrameEndEventPtr,
            subframeEndTime,
            aggregatedMpduFrameEndEventTicket);
    }//if//

}//SetupReceiveOfAggregateFrame//



inline
void Dot11Phy::StartReceivingThisSignal(const IncomingSignal& aSignal)
{
    const PropFrameType& incomingFrame = aSignal.GetFrame();

    (*this).currentIncomingSignalStartTime = simulationEngineInterfacePtr->CurrentTime();
    (*this).currentIncomingSignalSourceNodeId = aSignal.GetSourceNodeId();
    (*this).currentIncomingSignalTxParameters = incomingFrame.txParameters;
    (*this).currentIncomingSignalDatarateBitsPerSec =
        CalcDatarateBitsPerSecond(ofdmSymbolDuration, incomingFrame.txParameters);

    (*this).currentPacketHasAnError = false;
    (*this).currentSignalPowerDbm = aSignal.GetReceivedPowerDbm();
    (*this).currentAdjustedSignalPowerMilliwatts =
        (ConvertToNonDb(currentSignalPowerDbm) *
         CalcSignalPowerAdjustmentFactor(incomingFrame.txParameters));

    (*this).currentThermalNoisePowerMw =
        ((incomingFrame.txParameters.channelBandwidthMhz / baseChannelBandwidthMhz) *
        thermalNoisePowerPerBaseChannelMw *
        CalcThermalNoiseAdjustmentFactor(incomingFrame.txParameters));

    // Start interference calculation.
    (*this).lastErrorCalculationUpdateTime = simulationEngineInterfacePtr->CurrentTime();

    (*this).currentLockedOnFramePacketId = incomingFrame.packetId;

    if (incomingFrame.macFramePtr != nullptr) {
        OutputTraceAndStatsForRxStart(*incomingFrame.macFramePtr);
    }
    else {
        assert(!incomingFrame.aggregateFramePtr->empty());

        OutputTraceAndStatsForRxStart(*incomingFrame.aggregateFramePtr->front());
        if (incomingFrame.isAMpduAggregate) {
            (*this).SetupReceiveOfMpduAggregateFrame(incomingFrame);
        }//if//
    }//if//

}//StartReceivingThisSignal//




inline
void Dot11Phy::ProcessSignalArrivalFromChannel(const IncomingSignal& aSignal)
{
    switch (phyState) {
    case PhyScanning:
    case PhyReceiving:
        (*this).ProcessNewSignal(aSignal);

        break;
    case PhyTransmitting:
    case PhyTxStarting:
    case PhyIgnoring:
        (*this).AddSignalToInterferenceLevel(aSignal);

        break;
    default:
        assert(false); abort(); break;
    }//switch//

}//ProcessSignalArrivalFromChannel//


inline
bool Dot11Phy::IsCurrentlyReceivingThisSignal(const IncomingSignal& aSignal) const
{
    assert(phyState == PhyReceiving);
    return (aSignal.GetSourceNodeId() == currentIncomingSignalSourceNodeId);
}



inline
void Dot11Phy::ProcessSignalEndFromChannel(const IncomingSignal& aSignal)
{
    switch (phyState) {
    case PhyReceiving:
        if (IsCurrentlyReceivingThisSignal(aSignal)) {
            (*this).ProcessEndOfTheSignalCurrentlyBeingReceived(aSignal);
        }
        else {
            (*this).SubtractSignalFromInterferenceLevelAndNotifyMacIfMediumIsClear(aSignal);
        }//if//

        break;

    case PhyScanning:
    case PhyTransmitting:
    case PhyTxStarting:
    case PhyIgnoring:

        (*this).SubtractSignalFromInterferenceLevelAndNotifyMacIfMediumIsClear(aSignal);

        break;

    default:
        assert(false); abort(); break;
    }//switch//

}//ProcessSignalEndFromChannel//


inline
void Dot11Phy::StopReceivingFrames()
{
    assert((phyState == PhyScanning) || (phyState == PhyReceiving));

    if (phyState == PhyReceiving) {
        (*this).currentSignalPowerDbm = -DBL_MAX;
        (*this).currentAdjustedSignalPowerMilliwatts = 0.0;
        (*this).currentIncomingSignalSourceNodeId = 0;
        (*this).currentLockedOnFramePacketId = PacketIdType::nullPacketId;
        (*this).currentThermalNoisePowerMw = thermalNoisePowerAllChannelsMw;
    }//if//

    (*this).phyState = PhyIgnoring;

}//StopReceivingFrames//


inline
void Dot11Phy::StartReceivingFrames()
{
    assert(phyState == PhyIgnoring);
    (*this).phyState = PhyScanning;
}



inline
void Dot11Phy::SwitchToChannels(const vector<unsigned int>& bondedChannelList)
{
    assert(!bondedChannelList.empty());

    if (currentBondedChannelList == bondedChannelList) {
        return;
    }

    (*this).currentBondedChannelList = bondedChannelList;

    vector<unsigned int> sortedBondedChannelList = bondedChannelList;
    std::sort(sortedBondedChannelList.begin(), sortedBondedChannelList.end());
    (*this).firstChannelNumber = sortedBondedChannelList[0];

    for(unsigned int i = 1; (i < sortedBondedChannelList.size()); i++) {
        assert((sortedBondedChannelList[i] == (firstChannelNumber + i))
               && "Dot11 bonded channel list is not continguous");

        const unsigned int channelBandwidthMhz =
            RoundToUint(
                propagationModelInterfacePtr->GetChannelBandwidthMhz(sortedBondedChannelList[i]));

        assert((channelBandwidthMhz == baseChannelBandwidthMhz) &&
               "Bonded channel bandwidths must be same as the Base channel bandwidth.");

    }//for//

    (*this).currentSignalPowerDbm = -DBL_MAX;
    (*this).currentAdjustedSignalPowerMilliwatts = 0.0;

    // May change in channel width.

    (*this).thermalNoisePowerAllChannelsMw =
        (thermalNoisePowerPerBaseChannelMw * bondedChannelList.size());

    (*this).currentThermalNoisePowerMw = thermalNoisePowerAllChannelsMw;
    (*this).currentIncomingSignalDatarateBitsPerSec = 0;
    (*this).currentIncomingSignalSourceNodeId = 0;
    (*this).currentLockedOnFramePacketId = PacketIdType::nullPacketId;
    (*this).currentInterferencePowers.clear();
    (*this).currentInterferencePowers.resize(bondedChannelList.size(), IntegralPowerType(0.0));
    (*this).currentlySensingBusyMedium = false;

    switch (phyState) {
    case PhyTransmitting:
    case PhyTxStarting:
    case PhyIgnoring:

        assert(false && "Should be prevented from occuring"); abort();
        break;

    case PhyReceiving:
    case PhyScanning:

        phyState = PhyScanning;

        propagationModelInterfacePtr->SwitchToChannelNumbers(sortedBondedChannelList);

        break;

    default:
        assert(false); abort(); break;
    }//switch//

}//SwitchToChannels//




inline
void Dot11Phy::ProcessStatsForEndCurrentTransmission()
{
    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

    (*this).totalTransmissionTime += (currentTime - lastChannelStateTransitionTime);
    (*this).lastChannelStateTransitionTime = currentTime;
}


inline
void Dot11Phy::EndCurrentTransmission()
{
    assert(phyState == PhyTransmitting);

    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

    phyState = PhyScanning;

    (*this).ProcessStatsForEndCurrentTransmission();

    macLayerPtr->TransmissionIsCompleteNotification();

    // Note that the MAC may (theoretically) have issued a transmit command
    // in the immediately previous statement (TransmissionIsCompleteNotification call).

    if ((phyState == PhyScanning) &&
        ((currentTime >= currentLastCarrierDetectableSignalEnd) &&
         (CalcCurrentInterferencePowerMw() < energyDetectionPowerThresholdMw))) {

        (*this).currentlySensingBusyMedium = false;
        macLayerPtr->ClearChannelAtPhysicalLayerNotification();

    }//if//

}//EndCurrentTransmission//



inline
void Dot11Phy::ProcessStatsForTransitionToBusyChannel()
{
    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

    (*this).totalIdleChannelTime += (currentTime - lastChannelStateTransitionTime);
    (*this).lastChannelStateTransitionTime = currentTime;
}

inline
void Dot11Phy::ProcessStatsForTransitionToIdleChannel()
{
    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

    (*this).totalBusyChannelTime += (currentTime - lastChannelStateTransitionTime);
    (*this).lastChannelStateTransitionTime = currentTime;
}


inline
void Dot11Phy::ProcessNewSignal(const IncomingSignal& aSignal)
{
    if (phyState == PhyScanning) {
        bool preambleHasBeenDetected = false;

        if (aSignal.HasACompleteFrame()) {

            if (aSignal.GetReceivedPowerDbm() >= preambleDetectionPowerThresholdDbm) {

                if (!preambleDetectionProbBySinrTable.HasData()) {
                    preambleHasBeenDetected = true;
                }
                else {
                    const PropFrameType& incomingFrame = aSignal.GetFrame();

                    const double thermalNoisePowerMw =
                        ((incomingFrame.txParameters.channelBandwidthMhz / baseChannelBandwidthMhz) *
                         thermalNoisePowerPerBaseChannelMw *
                         CalcThermalNoiseAdjustmentFactor(incomingFrame.txParameters));

                    const double totalNoiseAndInterferencePowerMw =
                        (thermalNoisePowerMw + CalcCurrentInterferencePowerMw(incomingFrame.txParameters));

                    const double signalToNoiseAndInterferenceRatio =
                        (ConvertToNonDb(aSignal.GetReceivedPowerDbm()) / totalNoiseAndInterferencePowerMw);

                    const double probabilityOfDetection =
                        preambleDetectionProbBySinrTable.CalcValue(signalToNoiseAndInterferenceRatio);

                    if (probabilityOfDetection == 0.0) {
                        preambleHasBeenDetected = false;
                    }
                    else if (probabilityOfDetection == 1.0) {
                        preambleHasBeenDetected = true;
                    }
                    else {
                        const double randomNumber = (*this).aRandomNumberGenerator.GenerateRandomDouble();

                        preambleHasBeenDetected = (randomNumber < probabilityOfDetection);
                    }//if//
                }//if//
            }//if//
        }//if//

        if (preambleHasBeenDetected) {
            phyState = PhyReceiving;

            (*this).StartReceivingThisSignal(aSignal);

            if (!currentlySensingBusyMedium) {

                (*this).currentlySensingBusyMedium = true;
                (*this).ProcessStatsForTransitionToBusyChannel();

                macLayerPtr->BusyChannelAtPhysicalLayerNotification();

            }//if//
        }
        else {

            (*this).AddSignalToInterferenceLevel(aSignal);

            const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

            if ((!currentlySensingBusyMedium) &&
                ((currentTime < currentLastCarrierDetectableSignalEnd) ||
                 (CalcCurrentInterferencePowerMw() >= energyDetectionPowerThresholdMw))) {

                (*this).currentlySensingBusyMedium = true;
                (*this).ProcessStatsForTransitionToBusyChannel();

                macLayerPtr->BusyChannelAtPhysicalLayerNotification();

            }//if//
        }//if//
    }
    else if ((phyState == PhyReceiving) && (aSignal.HasACompleteFrame()) &&
             (aSignal.GetReceivedPowerDbm() > (currentSignalPowerDbm + signalCaptureRatioThresholdDb))) {

        // Signal capture.

        (*this).currentPacketHasAnError = true;

        OutputTraceAndStatsForRxEnd(currentLockedOnFramePacketId, true/*Capture*/);

        // Assuming Pilot energy doesn't interfere.

        const double signalPowerMw =
            (ConvertToNonDb(currentSignalPowerDbm) *
             CalcSubcarrierPowerAdjustmentFactorForInterferringFrame(
                 currentIncomingSignalTxParameters));

        (*this).AddSignalPowerToInterferenceLevel(
            currentIncomingSignalTxParameters.firstChannelNumber,
            (currentIncomingSignalTxParameters.channelBandwidthMhz / baseChannelBandwidthMhz),
            signalPowerMw);

        OutputTraceAndStatsForAddSignalToInterferenceLevel(
            currentSignalPowerDbm,
            signalPowerMw,
            currentIncomingSignalSourceNodeId,
            currentLockedOnFramePacketId);

        // Clear preempted aggregate frame if it exists.

        for(unsigned int i = 0; (i < aggregateFrameSubframeInfo.size()); i++) {
        }//for//

        (*this).aggregateFrameSubframeInfo.clear();

        if (!aggregatedMpduFrameEndEventTicket.IsNull()) {
            simulationEngineInterfacePtr->CancelEvent(aggregatedMpduFrameEndEventTicket);
        }//if//

        (*this).StartReceivingThisSignal(aSignal);
    }
    else {
        // Present signal keeps going

        (*this).AddSignalToInterferenceLevel(aSignal);

    }//if//

}//ProcessNewSignal//



inline
void Dot11Phy::ProcessEndOfTheSignalCurrentlyBeingReceived(const IncomingSignal& aSignal)
{
    const PropFrameType& incomingFrame = aSignal.GetFrame();

    assert(phyState == PhyReceiving);

    (*this).UpdatePacketReceptionCalculation();

    if (incomingFrame.macFramePtr != nullptr) {

        assert(incomingFrame.aggregateFramePtr == nullptr);

        OutputTraceAndStatsForRxEnd(currentLockedOnFramePacketId, false/*No Capture*/);

        (*this).phyState = PhyScanning;

        if (!currentPacketHasAnError) {

            (*this).lastReceivedPacketRssiDbm = currentSignalPowerDbm;


            macLayerPtr->ReceiveFrameFromPhy(*incomingFrame.macFramePtr, incomingFrame.txParameters);
        }
        else {
            macLayerPtr->NotifyThatPhyReceivedCorruptedFrame();

        }//if//
    }
    else {
        (*this).phyState = PhyScanning;

        // Check if aggregate frame has been "pre-screened" by Phy.

        if (!aggregateFrameSubframeInfo.empty()) {
            assert(currentAggregateFrameSubframeIndex == (aggregateFrameSubframeInfo.size()-1));

            AggregateFrameSubframeInfoElementType& subframeInfo =
                (*this).aggregateFrameSubframeInfo[currentAggregateFrameSubframeIndex];

            OutputTraceAndStatsForRxEnd(subframeInfo.macFramePtr->GetPacketId(), false/*No Capture*/);

            subframeInfo.hasError = currentPacketHasAnError;

            if (!currentPacketHasAnError) {

                (*this).lastReceivedPacketRssiDbm = currentSignalPowerDbm;

                macLayerPtr->ReceiveAggregatedSubframeFromPhy(
                    subframeInfo.macFramePtr,
                    currentIncomingSignalTxParameters,
                    currentAggregateFrameSubframeIndex,
                    static_cast<unsigned int>(aggregateFrameSubframeInfo.size()));
            }
            else {
                macLayerPtr->NotifyThatPhyReceivedCorruptedAggregatedSubframe(
                    currentIncomingSignalTxParameters,
                    currentAggregateFrameSubframeIndex,
                    static_cast<unsigned int>(aggregateFrameSubframeInfo.size()));

                subframeInfo.macFramePtr.reset();

            }//if//

            aggregateFrameSubframeInfo.clear();
        }
        else {
            // Optimimistic assumption: At least one subframe header was received.
            // Used only for setting NAV.

            macLayerPtr->ReceiveFrameFromPhy(
                (*notForMeHeaderOnlyFramePtr),
                currentIncomingSignalTxParameters);
        }//if//
    }//if//

    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

    // Note that the MAC may have issued a transmit command in the immediately
    // previous statement (ReceiveFrameFromPhy call).

    if ((phyState == PhyScanning) &&
        ((currentTime >= currentLastCarrierDetectableSignalEnd) &&
         (CalcCurrentInterferencePowerMw() < energyDetectionPowerThresholdMw))) {

        assert(currentlySensingBusyMedium);
        (*this).currentlySensingBusyMedium = false;
        (*this).ProcessStatsForTransitionToIdleChannel();
        macLayerPtr->ClearChannelAtPhysicalLayerNotification();
    }//if//

    (*this).currentSignalPowerDbm = -DBL_MAX;
    (*this).currentAdjustedSignalPowerMilliwatts = 0.0;

    (*this).currentIncomingSignalSourceNodeId = 0;
    (*this).currentLockedOnFramePacketId = PacketIdType::nullPacketId;

}//ProcessEndOfTheSignalCurrentlyBeingReceived//




inline
void Dot11Phy::ProcessAggregatedMpduFrameEndEvent()
{
    assert(phyState == PhyReceiving);

    (*this).aggregatedMpduFrameEndEventTicket.Clear();

    (*this).UpdatePacketReceptionCalculation();

    AggregateFrameSubframeInfoElementType& subframeInfo =
        (*this).aggregateFrameSubframeInfo[currentAggregateFrameSubframeIndex];

    OutputTraceAndStatsForRxEnd(subframeInfo.macFramePtr->GetPacketId(), false/*No Capture*/);

    subframeInfo.hasError = currentPacketHasAnError;

    if (!currentPacketHasAnError) {

        (*this).lastReceivedPacketRssiDbm = currentSignalPowerDbm;

        // Check for pre-filtering.
        if (subframeInfo.macFramePtr != nullptr) {
            macLayerPtr->ReceiveAggregatedSubframeFromPhy(
                subframeInfo.macFramePtr,
                currentIncomingSignalTxParameters,
                currentAggregateFrameSubframeIndex,
                static_cast<unsigned int>(aggregateFrameSubframeInfo.size()));
        }//if//
    }
    else {
        macLayerPtr->NotifyThatPhyReceivedCorruptedAggregatedSubframe(
            currentIncomingSignalTxParameters,
            currentAggregateFrameSubframeIndex,
            static_cast<unsigned int>(aggregateFrameSubframeInfo.size()));

        subframeInfo.macFramePtr.reset();
    }//if//

    // Reset for next MPDU subframe.  Assumes perfect MPDU delimiter acquisition.

    (*this).currentPacketHasAnError = false;

    (*this).currentAggregateFrameSubframeIndex++;

    // Rely on end of signal event for final subframe.

    if (currentAggregateFrameSubframeIndex < (aggregateFrameSubframeInfo.size() - 1)) {

        const AggregateFrameSubframeInfoElementType& nextSubframeInfo =
            aggregateFrameSubframeInfo[currentAggregateFrameSubframeIndex];

        const TimeType subframeEndTime =
            (simulationEngineInterfacePtr->CurrentTime() +
             CalculateFrameDataDuration(
                 nextSubframeInfo.macFramePtr->LengthBytes(),
                 currentIncomingSignalTxParameters));

        simulationEngineInterfacePtr->ScheduleEvent(
            aggregatedMpduFrameEndEventPtr,
            subframeEndTime,
            aggregatedMpduFrameEndEventTicket);
    }//if//

}//ProcessAggregatedMpduFrameEndEvent//


inline
void Dot11Phy::UpdateCcaEnergyDetectionThresholdDbm(const double& newCCAThresholdDbm)
{

    energyDetectionPowerThresholdDbm = newCCAThresholdDbm;

    energyDetectionPowerThresholdMw = ConvertToNonDb(energyDetectionPowerThresholdDbm);

}//UpdateSensitivityDbm//


//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------


inline
void Dot11Phy::OutputTraceAndStatsForAddSignalToInterferenceLevel(
    const double& signalPowerDbm,
    const double& adjustedSignalPowerMw,
    const NodeIdType& signalSourceNodeId,
    const PacketIdType& signalPacketId) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TracePhyInterference)) {

        const double currentInterferencePowerMw = CalcCurrentInterferencePowerMw();

        const double currentInterferenceAndNoisePowerDbm =
           ConvertToDb(currentThermalNoisePowerMw + currentInterferencePowerMw);

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {
            NoiseStartTraceRecord traceData;

            traceData.sourceNodeId = signalSourceNodeId;
            traceData.rxPower = ConvertToDb(adjustedSignalPowerMw);
            traceData.interferenceAndNoisePower = currentInterferenceAndNoisePowerDbm;

            assert(!redundantTraceInformationModeIsOn);// no implementation for redundant trace

            assert(sizeof(traceData) == NOISE_START_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName,
                                                              interfaceId,
                                                              "NoiseStart",
                                                              traceData);
        }
        else {
            ostringstream msgStream;
            msgStream.precision(tracePrecisionDigitsForDbm);

            msgStream << "SrcN= " << signalSourceNodeId
                      << " RxPow= " << ConvertToDb(adjustedSignalPowerMw)
                      << " I&NPow= " << currentInterferenceAndNoisePowerDbm;

            if (redundantTraceInformationModeIsOn) {
                msgStream << " PktId= " << signalPacketId;
                if (signalPacketId != currentLockedOnFramePacketId) {
                    msgStream << " LockedOnPacketId= " << currentLockedOnFramePacketId;
                }
                else {
                    // Preempted Frame Receive output null packet Id.
                    msgStream << " LockedOnPacketId= " << PacketIdType::nullPacketId;
                }//if//
            }//if//

            simulationEngineInterfacePtr->OutputTrace(modelName, interfaceId, "NoiseStart", msgStream.str());
        }//if//
    }//if//

    interferingSignalsStatPtr->IncrementCounter();

    if (signalPowerDbm < preambleDetectionPowerThresholdDbm) {

        weakSignalsStatPtr->IncrementCounter();
    }//if//


    if (((phyState == PhyTxStarting) || (phyState == PhyTransmitting)) &&
        (signalPowerDbm >= preambleDetectionPowerThresholdDbm)) {

        // Count all signals that the PHY could lock on to staring during a transmission.

        signalsDuringTransmissionStatPtr->IncrementCounter();

    }//if//

}//OutputTraceForAddSignalToInterferenceLevel//



inline
void Dot11Phy::OutputTraceForSubtractSignalFromInterferenceLevel(
    const IncomingSignal& aSignal,
    const double& signalPowerMw) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TracePhyInterference)) {

        const double currentInterferencePowerMw = CalcCurrentInterferencePowerMw();

        const double currentInterferenceAndNoisePowerDbm =
            ConvertToDb(currentThermalNoisePowerMw + currentInterferencePowerMw);

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            NoiseEndTraceRecord traceData;

            traceData.rxPower = ConvertToDb(signalPowerMw);
            traceData.interferenceAndNoisePower = currentInterferenceAndNoisePowerDbm;

            assert(!redundantTraceInformationModeIsOn);// no implementation for redundant trace

            assert(sizeof(traceData) == NOISE_END_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName,
                                                              interfaceId,
                                                              "NoiseEnd",
                                                              traceData);
        }
        else {
            ostringstream msgStream;
            msgStream.precision(tracePrecisionDigitsForDbm);

            msgStream << "RxPow= " << ConvertToDb(signalPowerMw)
                      << " I&NPow= " << currentInterferenceAndNoisePowerDbm;

            if (redundantTraceInformationModeIsOn) {
                msgStream << " PktId= " << aSignal.GetFrame().packetId
                          << " LockedOnPacketId= " << currentLockedOnFramePacketId;
            }//if//

            simulationEngineInterfacePtr->OutputTrace(modelName, interfaceId, "NoiseEnd", msgStream.str());
        }//if//
    }//if//

}//OutputTraceForSubtractSignalFromInterferenceLevel//




inline
void Dot11Phy::OutputTraceAndStatsForTxStart(
    const Packet& aPacket,
    const double& txPowerDbm,
    const TransmissionParametersType& txParameters,
    const TimeType& duration) const
{
    // Future idea: More detailed output from txParameters data.

    if (simulationEngineInterfacePtr->TraceIsOn(TracePhy)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            TxStartTraceRecord traceData;
            const PacketIdType& packetId = aPacket.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            traceData.txPower = txPowerDbm;
            traceData.dataRate = CalcDatarateBitsPerSecond(ofdmSymbolDuration, txParameters);
            traceData.duration = duration;

            assert(sizeof(traceData) == TX_START_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "TxStart", traceData);
        }
        else {
            ostringstream msgStream;
            msgStream.precision(tracePrecisionDigitsForDbm);

            msgStream << "PktId= " << aPacket.GetPacketId()
                      << " TxPow= " << txPowerDbm
                      << " Rate= " << CalcDatarateBitsPerSecond(ofdmSymbolDuration, txParameters)
                      << " Dur= " << ConvertTimeToStringSecs(duration);

            simulationEngineInterfacePtr->OutputTrace(modelName, interfaceId, "TxStart", msgStream.str());
        }//if//
    }//if//

    transmittedFramesStatPtr->IncrementCounter();

}//OutputTraceForTxStart//

inline
void Dot11Phy::OutputTraceAndStatsForRxStart(const Packet& aPacket)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TracePhy)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            RxStartTraceRecord traceData;
            const PacketIdType& packetId = aPacket.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            traceData.rxPower = currentSignalPowerDbm;

            assert(!redundantTraceInformationModeIsOn); // no implementation for redundant trace

            assert(sizeof(traceData) == RX_START_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "RxStart", traceData);

        }
        else {
            ostringstream msgStream;
            msgStream.precision(tracePrecisionDigitsForDbm);

            msgStream << "PktId= " << aPacket.GetPacketId() << " RxPow= " << currentSignalPowerDbm;

            if (redundantTraceInformationModeIsOn) {
                const double currentInterferencePowerMw = CalcCurrentInterferencePowerMw();
                const double currentInterferenceAndNoisePowerDbm =
                    ConvertToDb(currentThermalNoisePowerMw + currentInterferencePowerMw);

                msgStream << " I&NPow= " << currentInterferenceAndNoisePowerDbm;
            }//if//

            simulationEngineInterfacePtr->OutputTrace(modelName, interfaceId, "RxStart", msgStream.str());

        }//if//

    }//if//

}//OutputTraceForRxStart//

inline
void Dot11Phy::OutputTraceAndStatsForRxEnd(const PacketIdType& packetId, const bool& receiveIsEndedByCapture)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TracePhy)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            RxEndTraceRecord traceData;
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            traceData.error = currentPacketHasAnError;
            traceData.captured = receiveIsEndedByCapture;
            if (!currentPacketHasAnError) {
                assert(!receiveIsEndedByCapture);
            }

            assert(!redundantTraceInformationModeIsOn); // no implementation for redundant trace

            assert(sizeof(traceData) == RX_END_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "RxEnd", traceData);
        }
        else {

            ostringstream msgStream;

            msgStream.precision(tracePrecisionDigitsForDbm);

            msgStream << "PktId= " << packetId;

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

            if (redundantTraceInformationModeIsOn) {
                const double currentInterferencePowerMw = CalcCurrentInterferencePowerMw();
                const double currentInterferenceAndNoisePowerDbm =
                    ConvertToDb(currentThermalNoisePowerMw + currentInterferencePowerMw);

                msgStream << " I&NPow= " << currentInterferenceAndNoisePowerDbm;
            }//if//

            simulationEngineInterfacePtr->OutputTrace(modelName, interfaceId, "RxEnd", msgStream.str());

        }//if//
    }//if//

    receivedFrameRssiMwStatPtr->RecordStatValue(ConvertToNonDb(currentSignalPowerDbm));

    const double currentInterferencePowerMw = CalcCurrentInterferencePowerMw();

    const double currentInterferenceAndNoisePowerMw =
        currentThermalNoisePowerMw + currentInterferencePowerMw;

    const double signalToNoiseAndInterferenceRatio =
        (currentAdjustedSignalPowerMilliwatts / currentInterferenceAndNoisePowerMw);

    receivedFrameSinrStatPtr->RecordStatValue(signalToNoiseAndInterferenceRatio);

    (*this).lastReceivedPacketSinrDb = ConvertToDb(signalToNoiseAndInterferenceRatio);

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


//-------------------------------------------------------------------------------------------------


}//namespace//


#endif
