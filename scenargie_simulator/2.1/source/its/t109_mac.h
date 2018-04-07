// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

 #ifndef T109_MAC_H
#define T109_MAC_H

#include "scensim_engine.h"
#include "scensim_netsim.h"
#include "scensim_prop.h"
#include "scensim_bercurves.h"
#include "scensim_support.h"

#include "t109_phy.h"

#include <map>
#include <string>

namespace T109 {

using std::queue;
using std::map;

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

// Duration in us.
typedef unsigned short int DurationFieldType;

const unsigned char QOS_DATA_FRAME_TYPE_CODE = 0x28; // 10 1000;
const size_t NUMBER_RSU_FRAME_SLOTS = 16;

struct FrameControlFieldType {
    unsigned char notUsed1:2;

    unsigned char frameTypeAndSubtype:6;

    unsigned char notUsed2:1;
    unsigned char notUsed3:1;
    unsigned char notUsed4:1;

    unsigned char isRetry:1;

    unsigned char notUsed5:1;

    unsigned char notUsed_moreData:1;

    unsigned char notUsed6:1;
    unsigned char notUsed7:1;

    FrameControlFieldType()
        : isRetry(0), notUsed1(0), notUsed2(0), notUsed3(0), notUsed4(0), notUsed5(0),
          notUsed6(0), notUsed7(0), notUsed_moreData(0)
    {}
};//FrameControlFieldType//

struct MacFrameHeaderType {
    FrameControlFieldType frameControlField;
    DurationFieldType duration;
    SixByteMacAddressType receiverAddress;
    SixByteMacAddressType transmitterAddress;
    unsigned char notUsedByte[6];
    unsigned short int sequenceNumber;
};

struct LLcHeaderType {
    unsigned char notUsedByte[8];
};

struct RsuControlHeaderType {
    unsigned char version:4;
    unsigned char id:4;

    unsigned char  sync:3;
    unsigned char notUsed1:5;

    unsigned char notUsedd[4];

    struct RsuSlotInformation {
        unsigned char forwardCount:2;
        unsigned char durationCount:6;

        RsuSlotInformation() : forwardCount(0), durationCount(0) {}
    };

    RsuSlotInformation slotInformations[NUMBER_RSU_FRAME_SLOTS];
};

enum AccessMode {
    ACCESS_MODE_OBE,
    ACCESS_MODE_RSU,
};

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

class T109App;

class T109Mac : public MacLayer {
public:
    static const string modelName;

    static const int noBackoffSlots = -1;

    typedef T109Phy::PropFrameType PropFrameType;

    T109Mac(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<SimplePropagationModelForNode<PropFrameType> >& propModelInterfacePtr,
        const shared_ptr<BitOrBlockErrorRateCurveDatabase>& berCurveDatabasePtr,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const size_t initInterfaceIndex,
        T109App* initAppLayerPtr,
        const RandomNumberGeneratorSeedType& nodeSeed);

    void ReceivePacketFromUpperLayer(
        unique_ptr<Packet>& packetPtr,
        const SixByteMacAddressType& nextHopAddress);

    void ClearQueuedPacket();

    // Physical Layer Interface:

    void BusyChannelAtPhysicalLayerNotification();
    void ClearChannelAtPhysicalLayerNotification();
    void TransmissionIsCompleteNotification();

    void ReceiveFrameFromPhy(
        const Packet& aFrame,
        const double receiveRssiDbm,
        const DatarateBitsPerSecType& datarateBitsPerSecond);

    void SwitchToChannel(const unsigned int channel);

    TimeType GetShortInterframeSpaceDuration() const { return aShortInterframeSpaceTime; }
    TimeType CalculateFrameTransmissionDuration(const size_t networkPacketSizeBytes) const;

    AccessMode GetAccessMode() const { return accessMode; }

    virtual void NetworkLayerQueueChangeNotification() { }
    virtual void DisconnectFromOtherLayers() { }

private:
    enum MacStateType {
        IDLE_STATE,
        BUSY_MEDIUM_STATE,
        WAITING_FOR_NAV_EXPIRATION_STATE,
        WAITING_FOR_IFS_AND_BACKOFF_STATE,
        CHANGING_CHANNELS_STATE,
        TRANSMISSION_SUPRESSION_STATE
    };

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    NodeIdType nodeId;
    InterfaceIdType interfaceId;

    T109App* appLayerPtr;

    SixByteMacAddressType myMacAddress;
    MacStateType macState;

    T109Phy physicalLayer;

    RandomNumberGenerator aRandomNumberGenerator;

    DatarateBitsPerSecType datarateBitsPerSec;
    double transmitPowerDbm;

    AccessMode accessMode;

    TimeType timeDivisionInterval;

    struct RsuFrameSlot {
        TimeType startTimeFromBase;
        TimeType endTimeFromBase;
        int durationCount;

        TimeType informationExpirationTime;

        RsuFrameSlot()
            :
            startTimeFromBase(ZERO_TIME),
            endTimeFromBase(ZERO_TIME),
            durationCount(0),
            informationExpirationTime(ZERO_TIME)
        {}
    };
    vector<RsuFrameSlot> rsuFrameSlots;
    vector<size_t> rsuFrameSlotNumbers;

    int rsuFrameSlotSyncCount;
    TimeType allSlotInformationExpirationTime;
    TimeType rsuSlotInformationAvailableDuration;
    int rsuFrameForwardCount;
    TimeType rsuControlUnitDuration;
    TimeType guardTime;

    TimeType aShortInterframeSpaceTime;
    TimeType aSlotTime;
    TimeType aRxTxTurnaroundTime;
    TimeType mediumReservedUntilTimeAkaNAV;
    TimeType currentIfsAndBackoffStartTime;
    TimeType currentIfsDuration;

    int maxContentionWindowSlots;
    int currentNumOfBackoffSlots;
    unsigned int currentPacketSequenceNumber;

    unsigned int switchingToThisChannel;

    struct PacketEntry {
        unique_ptr<Packet> packetPtr;
        SixByteMacAddressType nextHopAddress;

        PacketEntry() : packetPtr(nullptr) {}
        PacketEntry(
            unique_ptr<Packet>& initPacketPtr,
            const SixByteMacAddressType& initNextHopAddress)
            :
            packetPtr(move(initPacketPtr)),
            nextHopAddress(initNextHopAddress)
        {}

        void operator=(PacketEntry&& right) {
            packetPtr = move(right.packetPtr);
            nextHopAddress = right.nextHopAddress;
        }

        PacketEntry(PacketEntry&& right) { (*this) = move(right); }
    };

    queue<PacketEntry> packetEntries;
    TimeType maxTransmissionDuration;


    //-------------------------------------------------------------------------

    class WakeupTimerEvent : public SimulationEvent {
    public:
        WakeupTimerEvent(T109Mac* initMacPtr) : macPtr(initMacPtr) { }
        void ExecuteEvent() { macPtr->ProcessWakeupTimerEvent(); }
    private:
        T109Mac* macPtr;
    };

    shared_ptr<WakeupTimerEvent> wakeupTimerEventPtr;
    EventRescheduleTicket wakeupTimerEventTicket;
    TimeType currentWakeupTimerExpirationTime;

    TimeType mediumBecameIdleTime;

    map<SixByteMacAddressType, unsigned short int> lastSequenceNumberMap;

    // Statistics:
    shared_ptr<CounterStatistic> broadcastDataFramesSentStatPtr;
    shared_ptr<CounterStatistic> dataFramesReceivedStatPtr;

    // Parallelism Stuff:
    unsigned int lookaheadIndex;

    //-------------------------------------------------------------------------

    bool FrameWithThisAddressIsForThisNode(const SixByteMacAddressType& theMacAddress) const;

    void ScheduleWakeupTimer(const TimeType& wakeupTime);

    TimeType CalculateCurrentIfsDuration() const;

    TimeType CalculateCurrentBackoffDuration() const;

    TimeType CalculateCurrentIfsAndBackoffDuration() const;

    TimeType CalculateNextTransmissionEnabledTime() const;

    bool CanRestartBackoff() const;

    void StartIfsAndBackoffIfNecessary();

    void PauseBackoff();
    void DecreseElapsedBackoffSlots();

    void ProcessVirtualCarrierSenseAkaNavExpiration();

    void GoIntoWaitForExpirationOfVirtualCarrierSenseAkaNavState();

    void TransmitAPacket();

    void ProcessIfsAndBackoffExpiration();
    bool IsOverlappedWithTransmissionSupressionDuration(const TimeType& startTime, const TimeType& endTime) const;

    void RememberNewSequenceNumber(
        const SixByteMacAddressType& transmitterAddress,
        const unsigned short int sequenceNumber);

    bool HaveAlreadySeenThisPacketBefore(
        const SixByteMacAddressType& transmitterAddress,
        const unsigned short int sequenceNumber) const;

    void ProcessWakeupTimerEvent();

    void OutputTraceAndStatsForFrameReceive(const Packet& aFrame) const;
    void OutputTraceForClearChannel() const;
    void OutputTraceForBusyChannel() const;
    void OutputTraceForNavStart(const TimeType& expirationTime) const;
    void OutputTraceForNavExpiration() const;
    void OutputTraceForIfsAndBackoffStart(const TimeType& ifsAndBackoffDuration) const;
    void OutputTraceForInterruptedIfsAndBackoff() const;
    void OutputTraceForIfsAndBackoffExpiration() const;
    void OutputTraceAndStatsForBroadcastDataFrameTransmission(const Packet& packet) const;

    void ProcessDataFrame(
        const Packet& dataFrame,
        const double receiveRssiDbm,
        const DatarateBitsPerSecType& datarateBitsPerSecond);

};//T109Mac//

#pragma warning(disable:4355)

inline
T109Mac::T109Mac(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const shared_ptr<SimplePropagationModelForNode<PropFrameType> >& propModelInterfacePtr,
    const shared_ptr<BitOrBlockErrorRateCurveDatabase>& berCurveDatabasePtr,
    const NodeIdType& initNodeId,
    const InterfaceIdType& initInterfaceId,
    const size_t initInterfaceIndex,
    T109App* initAppLayerPtr,
    const RandomNumberGeneratorSeedType& nodeSeed)
    :
    simEngineInterfacePtr(simulationEngineInterfacePtr),
    nodeId(initNodeId),
    interfaceId(initInterfaceId),
    appLayerPtr(initAppLayerPtr),
    macState(IDLE_STATE),
    physicalLayer(
        simulationEngineInterfacePtr,
        propModelInterfacePtr,
        berCurveDatabasePtr,
        this,
        theParameterDatabaseReader,
        initNodeId,
        initInterfaceId,
        nodeSeed),
    aRandomNumberGenerator(HashInputsToMakeSeed(nodeSeed, initInterfaceIndex)),
    datarateBitsPerSec(
        theParameterDatabaseReader.ReadInt("t109-datarate-bits-per-second", nodeId, interfaceId)),
    transmitPowerDbm(
        theParameterDatabaseReader.ReadDouble("t109-tx-power-dbm", nodeId, interfaceId)),
    accessMode(ACCESS_MODE_OBE),
    timeDivisionInterval(
        theParameterDatabaseReader.ReadTime("t109-time-division-interval", nodeId, interfaceId)),
    rsuFrameSlots(NUMBER_RSU_FRAME_SLOTS),
    rsuFrameSlotSyncCount(0),
    allSlotInformationExpirationTime(ZERO_TIME),
    rsuSlotInformationAvailableDuration(
        theParameterDatabaseReader.ReadTime("t109-rsu-slot-information-available-duration", nodeId, interfaceId)),
    rsuFrameForwardCount(3),
    rsuControlUnitDuration(
        theParameterDatabaseReader.ReadTime("t109-rsu-control-unit-duration", nodeId, interfaceId)),
    guardTime(ZERO_TIME),
    aShortInterframeSpaceTime(INFINITE_TIME),
    aSlotTime(INFINITE_TIME),
    aRxTxTurnaroundTime(INFINITE_TIME),
    mediumReservedUntilTimeAkaNAV(ZERO_TIME),
    currentIfsAndBackoffStartTime(ZERO_TIME),
    currentIfsDuration(ZERO_TIME),
    maxContentionWindowSlots(
        theParameterDatabaseReader.ReadInt(("t109-contention-window-max-slots"), nodeId, interfaceId)),
    currentNumOfBackoffSlots(noBackoffSlots),
    currentPacketSequenceNumber(0),
    switchingToThisChannel(0),
    maxTransmissionDuration(
        theParameterDatabaseReader.ReadTime(
            "t109-max-transmission-duration", initNodeId, initInterfaceId)),
    currentWakeupTimerExpirationTime(ZERO_TIME),
    mediumBecameIdleTime(ZERO_TIME),
    broadcastDataFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_Data_BroadcastFramesSent"))),
    dataFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_Data_FramesReceived"))),
    lookaheadIndex(0)
{
    const string accessString = MakeLowerCaseString(
        theParameterDatabaseReader.ReadString("t109-access-mode", nodeId, interfaceId));

    for(size_t i = 0; i < rsuFrameSlots.size(); i++) {
        RsuFrameSlot& rsuFrameSlot = rsuFrameSlots[i];

        rsuFrameSlot.startTimeFromBase =
            theParameterDatabaseReader.ReadTime(
                "t109-rsu-slot-" + ConvertToString(i) + "-start-time", nodeId, interfaceId);
    }

    if (accessString == "obe") {
        accessMode = ACCESS_MODE_OBE;
    } else if (accessString == "rsu") {
        accessMode = ACCESS_MODE_RSU;

        guardTime =
            theParameterDatabaseReader.ReadTime("t109-rsu-frame-guard-time", nodeId, interfaceId);

        for(size_t i = 0; i < rsuFrameSlots.size(); i++) {
            RsuFrameSlot& rsuFrameSlot = rsuFrameSlots[i];

            rsuFrameSlot.endTimeFromBase =
                rsuFrameSlot.startTimeFromBase +
                theParameterDatabaseReader.ReadTime(
                    "t109-rsu-slot-" + ConvertToString(i) + "-duration", nodeId, interfaceId);
        }

        const string rsuFrameSlotsString =
            theParameterDatabaseReader.ReadString("t109-rsu-slot-number", nodeId, interfaceId);

        if (rsuFrameSlotsString == "*") {

            for(size_t i = 0; i < rsuFrameSlots.size(); i++) {
                rsuFrameSlotNumbers.push_back(i);
            }

        } else {

            deque<string> rsuFrameSlotStrings;

            TokenizeToTrimmedLowerString(rsuFrameSlotsString, ",", rsuFrameSlotStrings);

            for(size_t i = 0; i < rsuFrameSlotStrings.size(); i++) {

                bool success;
                int aValue;

                ConvertStringToInt(rsuFrameSlotStrings[i], aValue, success);

                if (!success) {
                    cerr << "Error: failed to read slot numbers " << rsuFrameSlotsString << endl;
                    exit(1);
                }

                rsuFrameSlotNumbers.push_back(aValue);
            }
        }

        if (theParameterDatabaseReader.ParameterExists(
                "t109-slot-information-forward-count", nodeId, interfaceId)) {
            rsuFrameForwardCount =
                theParameterDatabaseReader.ReadInt("t109-slot-information-forward-count", nodeId, interfaceId);
        }

        if (rsuFrameForwardCount >= 4) {
            cerr << "t109-slot-information-forward-count must be less than 4" << endl;
            exit(1);
        }

    } else {
        cerr << "Error: invalid access mode" << endl;
        exit(1);
    }

    aShortInterframeSpaceTime = physicalLayer.GetShortInterframeSpaceDuration();
    aSlotTime = physicalLayer.GetSlotDuration();
    aRxTxTurnaroundTime = physicalLayer.GetRxTxTurnaroundTime();

    wakeupTimerEventPtr = shared_ptr<WakeupTimerEvent>(new WakeupTimerEvent(this));

    // Mac address related multiple interface restrictions:
    //   - Only one Adhoc interface on a single propagation environment.
    //   - Can't have two interfaces associate with same AP.

    myMacAddress.SetLowerBitsWithNodeId(nodeId);

    // Parallelism Stuff

    (*this).lookaheadIndex = (*this).simEngineInterfacePtr->AllocateLookaheadTimeIndex();

    simEngineInterfacePtr->SetALookaheadTimeForThisNode(
        physicalLayer.GetRxTxTurnaroundTime(), lookaheadIndex);

}//T109Mac()//


#pragma warning(default:4355)


inline
bool T109Mac::FrameWithThisAddressIsForThisNode(const SixByteMacAddressType& theMacAddress) const {

    return ((theMacAddress == this->myMacAddress) ||
            (theMacAddress.IsABroadcastOrAMulticastAddress()));
}


inline
void T109Mac::ScheduleWakeupTimer(const TimeType& wakeupTime)
{
    if (wakeupTimerEventTicket.IsNull()) {
        simEngineInterfacePtr->ScheduleEvent(
            wakeupTimerEventPtr, wakeupTime, wakeupTimerEventTicket);
    }
    else {
        simEngineInterfacePtr->RescheduleEvent(wakeupTimerEventTicket, wakeupTime);

    }//if//

}//ScheduleWakeupTimer//

inline
TimeType T109Mac::CalculateCurrentIfsDuration() const
{
    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
    const TimeType clearChannelDuration = currentTime - mediumBecameIdleTime;

    TimeType ifsDuration;

    if (accessMode == ACCESS_MODE_OBE) {

        ifsDuration = (aShortInterframeSpaceTime + aSlotTime*2);

    } else {

        ifsDuration = (aShortInterframeSpaceTime);
    }

    // Always add a full IFS even if channel is already idle?
    //return ifsDuration;

    // Calculate IFS from idle time.
    return std::max<TimeType>(0, ifsDuration - clearChannelDuration);
}

inline
TimeType T109Mac::CalculateCurrentBackoffDuration() const
{
    if (accessMode == ACCESS_MODE_OBE) {
        return (aSlotTime * currentNumOfBackoffSlots);
    } else {
        return ZERO_TIME;
    }

}//CalculateCurrentBackoffDuration//

inline
TimeType T109Mac::CalculateCurrentIfsAndBackoffDuration() const
{
    return std::max<TimeType>(
        ZERO_TIME,
        ((*this).CalculateCurrentIfsDuration() +
         (*this).CalculateCurrentBackoffDuration() -
         aRxTxTurnaroundTime));
}

inline
bool T109Mac::CanRestartBackoff() const
{
    if (macState == WAITING_FOR_IFS_AND_BACKOFF_STATE) {
        return false;
    }

    return (macState == IDLE_STATE && !packetEntries.empty());
}

inline
void T109Mac::StartIfsAndBackoffIfNecessary()
{
    if ((*this).CanRestartBackoff()) {

        if (currentNumOfBackoffSlots == noBackoffSlots) {
            currentNumOfBackoffSlots =
                aRandomNumberGenerator.GenerateRandomInt(0, maxContentionWindowSlots);
        }

        const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
        const TimeType ifsAndBackoffDuration =
            (*this).CalculateCurrentIfsAndBackoffDuration();
        const TimeType frameTransmissionDuration =
            (*this).CalculateFrameTransmissionDuration(packetEntries.front().packetPtr->LengthBytes());

        if ((*this).IsOverlappedWithTransmissionSupressionDuration(
                currentTime,
                currentTime + ifsAndBackoffDuration + aRxTxTurnaroundTime + frameTransmissionDuration)) {
            macState = TRANSMISSION_SUPRESSION_STATE;

            (*this).ScheduleWakeupTimer(
                (*this).CalculateNextTransmissionEnabledTime());

        } else {
            assert(ifsAndBackoffDuration >= ZERO_TIME);

            macState = WAITING_FOR_IFS_AND_BACKOFF_STATE;

            currentIfsAndBackoffStartTime = currentTime;
            currentIfsDuration = (*this).CalculateCurrentIfsDuration();

            (*this).ScheduleWakeupTimer(
                currentIfsAndBackoffStartTime +
                ifsAndBackoffDuration);

            OutputTraceForIfsAndBackoffStart(ifsAndBackoffDuration);
        }
    }

}//StartIfsAndBackoffIfNecessary//

inline
void T109Mac::DecreseElapsedBackoffSlots()
{
    if (currentNumOfBackoffSlots == 0 ||
        currentNumOfBackoffSlots == noBackoffSlots) {
        return;
    }

    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
    const TimeType elapsedTime = (currentTime - currentIfsAndBackoffStartTime);
    const TimeType remainingIfsAndBackoffDuration =
        currentIfsDuration + (*this).CalculateCurrentBackoffDuration();

    if (elapsedTime + aRxTxTurnaroundTime >= remainingIfsAndBackoffDuration) {

        currentNumOfBackoffSlots = 0;

    } else {

        const int numberElapsedSlots =
            std::max<int>(0, static_cast<int>((elapsedTime - currentIfsDuration)/aSlotTime));

        assert(currentNumOfBackoffSlots >= numberElapsedSlots);

        currentNumOfBackoffSlots -= numberElapsedSlots;
    }
}

inline
void T109Mac::PauseBackoff()
{
    (*this).DecreseElapsedBackoffSlots();

    OutputTraceForInterruptedIfsAndBackoff();

}//PauseBackoff//

//--------------------------------------------------------------------------------------------------
//
// The following routines have to do with effect of on/off transitions of carrier sense
// and virtual carrier sense (NAV) on the backoff of the EDCA access categories.
//

inline
void T109Mac::ProcessVirtualCarrierSenseAkaNavExpiration()
{
    assert(macState == WAITING_FOR_NAV_EXPIRATION_STATE);

    OutputTraceForNavExpiration();

    macState = IDLE_STATE;

    mediumBecameIdleTime = simEngineInterfacePtr->CurrentTime();

    (*this).StartIfsAndBackoffIfNecessary();
}

inline
void T109Mac::BusyChannelAtPhysicalLayerNotification()
{
    OutputTraceForBusyChannel();

    if (accessMode == ACCESS_MODE_RSU) {
        return;
    }

    switch (macState) {
    case WAITING_FOR_NAV_EXPIRATION_STATE:
    case TRANSMISSION_SUPRESSION_STATE:
        simEngineInterfacePtr->CancelEvent(wakeupTimerEventTicket);
        macState = BUSY_MEDIUM_STATE;

        break;

    case WAITING_FOR_IFS_AND_BACKOFF_STATE:
        simEngineInterfacePtr->CancelEvent(wakeupTimerEventTicket);
        macState = BUSY_MEDIUM_STATE;
        (*this).PauseBackoff();

        break;

    case IDLE_STATE:
        macState = BUSY_MEDIUM_STATE;
        break;

    case BUSY_MEDIUM_STATE:
        // Ignore: Still busy
        //assert(false && "PHY sending duplicate notifications, should not happen."); abort();
        break;

    default:
        assert(false); abort(); break;
    }//switch//

}//BusyChannelAtPhysicalLayerNotification//


inline
void T109Mac::GoIntoWaitForExpirationOfVirtualCarrierSenseAkaNavState()
{
    assert(macState == BUSY_MEDIUM_STATE);
    assert(wakeupTimerEventTicket.IsNull());

    macState = WAITING_FOR_NAV_EXPIRATION_STATE;

    (*this).ScheduleWakeupTimer(mediumReservedUntilTimeAkaNAV);

    OutputTraceForNavStart(mediumReservedUntilTimeAkaNAV);
}


inline
void T109Mac::ClearChannelAtPhysicalLayerNotification()
{
    OutputTraceForClearChannel();

    if (accessMode == ACCESS_MODE_RSU) {
        macState = IDLE_STATE;
        mediumBecameIdleTime = simEngineInterfacePtr->CurrentTime();
        (*this).StartIfsAndBackoffIfNecessary();
        return;
    }

    if (macState == WAITING_FOR_NAV_EXPIRATION_STATE) {
        return;
    }//if//

    assert(macState == BUSY_MEDIUM_STATE);

    macState = IDLE_STATE;

    if (simEngineInterfacePtr->CurrentTime() < mediumReservedUntilTimeAkaNAV) {
        (*this).GoIntoWaitForExpirationOfVirtualCarrierSenseAkaNavState();
    }
    else {
        mediumBecameIdleTime = simEngineInterfacePtr->CurrentTime();
        (*this).StartIfsAndBackoffIfNecessary();

    }//if//

}//ClearChannelAtPhysicalLayerNotification//


// Could infer this functionality from Carrier/No Carrier notifications
// but added for clarity.

inline
void T109Mac::TransmissionIsCompleteNotification()
{
    if (macState == CHANGING_CHANNELS_STATE) {

        if (accessMode == ACCESS_MODE_OBE) {
            // Assume the medium is busy, but if it is idle the PHY will notify the MAC at channel
            // switch.
            macState = BUSY_MEDIUM_STATE;
        }

        (*this).SwitchToChannel(switchingToThisChannel);
    }

}//TransmissionIsCompleteNotification//

inline
void T109Mac::TransmitAPacket()
{
    PacketEntry& packetEntry = packetEntries.front();

    unique_ptr<Packet> packetToSendPtr = move(packetEntry.packetPtr);
    SixByteMacAddressType nextHopAddress = packetEntry.nextHopAddress;
    packetEntries.pop();

    RsuControlHeaderType rsuControlHeader;
    LLcHeaderType llcHeader;
    MacFrameHeaderType dataMacFrameHeader;

    if (accessMode == ACCESS_MODE_OBE) {
        const TimeType currentTime = simEngineInterfacePtr->CurrentTime();

        rsuControlHeader.id = 0;
        rsuControlHeader.sync = rsuFrameSlotSyncCount;

        for(size_t i = 0; i < NUMBER_RSU_FRAME_SLOTS; i++) {
            const RsuFrameSlot& rsuFrameSlot = rsuFrameSlots.at(i);

            if (currentTime + rsuSlotInformationAvailableDuration < rsuFrameSlot.informationExpirationTime) {

                RsuControlHeaderType::RsuSlotInformation& slotInformation =
                    rsuControlHeader.slotInformations[i];

                const int forwardCount = static_cast<int>(
                    (rsuFrameSlot.informationExpirationTime - (currentTime + rsuSlotInformationAvailableDuration)) /
                    rsuSlotInformationAvailableDuration);

                slotInformation.forwardCount = forwardCount;
                slotInformation.durationCount = rsuFrameSlot.durationCount;
            }
        }

    } else {

        rsuControlHeader.id = 1;
        rsuControlHeader.sync = 0;

        for(size_t i = 0; i < rsuFrameSlotNumbers.size(); i++) {
            const size_t slotNumber = rsuFrameSlotNumbers[i];
            const RsuFrameSlot& rsuFrameSlot = rsuFrameSlots.at(slotNumber);
            const TimeType duration =
                rsuFrameSlot.endTimeFromBase - rsuFrameSlot.startTimeFromBase;

            RsuControlHeaderType::RsuSlotInformation& slotInformation =
                rsuControlHeader.slotInformations[slotNumber];

            slotInformation.forwardCount = rsuFrameForwardCount;
            slotInformation.durationCount =
                static_cast<int>(duration / (rsuControlUnitDuration*3));

            assert(slotInformation.durationCount*rsuControlUnitDuration > guardTime*2);
        }
    }

    dataMacFrameHeader.frameControlField.frameTypeAndSubtype = QOS_DATA_FRAME_TYPE_CODE;
    dataMacFrameHeader.frameControlField.isRetry = 0;
    dataMacFrameHeader.duration = 0;
    dataMacFrameHeader.receiverAddress = nextHopAddress;
    dataMacFrameHeader.sequenceNumber = static_cast<unsigned short int>(currentPacketSequenceNumber);
    dataMacFrameHeader.transmitterAddress = myMacAddress;

    packetToSendPtr->AddPlainStructHeader(rsuControlHeader);
    packetToSendPtr->AddPlainStructHeader(llcHeader);
    packetToSendPtr->AddPlainStructHeader(dataMacFrameHeader);

    OutputTraceAndStatsForBroadcastDataFrameTransmission(*packetToSendPtr);

    macState = BUSY_MEDIUM_STATE;

    physicalLayer.TransmitFrame(
        packetToSendPtr,
        datarateBitsPerSec,
        transmitPowerDbm,
        aRxTxTurnaroundTime);

    currentPacketSequenceNumber++;

}//TransmitAPacket//

inline
void T109Mac::ProcessIfsAndBackoffExpiration()
{
    OutputTraceForIfsAndBackoffExpiration();
    assert(macState == WAITING_FOR_IFS_AND_BACKOFF_STATE);

    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
    const TimeType frameTransmissionDuration =
        (*this).CalculateFrameTransmissionDuration(
            packetEntries.front().packetPtr->LengthBytes());

    if ((*this).IsOverlappedWithTransmissionSupressionDuration(
            currentTime,
            currentTime + aRxTxTurnaroundTime + frameTransmissionDuration)) {

        macState = TRANSMISSION_SUPRESSION_STATE;

        (*this).ScheduleWakeupTimer(
            (*this).CalculateNextTransmissionEnabledTime());

    } else {

        currentNumOfBackoffSlots = noBackoffSlots;

        (*this).TransmitAPacket();
    }

}//ProcessIfsAndBackoffExpiration//

inline
bool T109Mac::IsOverlappedWithTransmissionSupressionDuration(
    const TimeType& startTime,
    const TimeType& endTime) const
{
    assert(startTime < endTime);

    const TimeType startTimeFromBaseTime = (startTime % timeDivisionInterval);
    const TimeType currentBaseTime = startTime - startTimeFromBaseTime;

    vector<pair<TimeType, TimeType> > availableTimes;

    if (accessMode == ACCESS_MODE_OBE) {

        if (startTime >= allSlotInformationExpirationTime) {
            return false;
        }

        vector<size_t> availableSlotNumbers;
        for(size_t i = 0; i < rsuFrameSlots.size(); i++) {
            if (startTime < rsuFrameSlots[i].informationExpirationTime) {
                availableSlotNumbers.push_back(i);
            }
        }

        if (availableSlotNumbers.empty()) {
            return false;
        }

        for(size_t i = 0; (i < availableSlotNumbers.size() - 1); i++) {
            const RsuFrameSlot& rsuFrameSlot1 = rsuFrameSlots.at(availableSlotNumbers[i]);
            const RsuFrameSlot& rsuFrameSlot2 = rsuFrameSlots.at(availableSlotNumbers[i+1]);

            if (currentBaseTime + rsuFrameSlot1.endTimeFromBase + guardTime <= startTime &&
                endTime < currentBaseTime + rsuFrameSlot2.startTimeFromBase - guardTime) {
                return false;
            }
        }

        return !(currentBaseTime + rsuFrameSlots.at(availableSlotNumbers.back()).endTimeFromBase + guardTime <= startTime &&
                 endTime < currentBaseTime + timeDivisionInterval + rsuFrameSlots.at(availableSlotNumbers.front()).startTimeFromBase - guardTime);

    } else {

        for(size_t i = 0; i < rsuFrameSlotNumbers.size(); i++) {
            const RsuFrameSlot& rsuFrameSlot = rsuFrameSlots.at(rsuFrameSlotNumbers[i]);

            if (currentBaseTime + rsuFrameSlot.startTimeFromBase + guardTime <= startTime &&
                endTime < currentBaseTime + rsuFrameSlot.endTimeFromBase - guardTime) {
                return false;
            }
        }

        return true;
    }

}//IsOverlappedWithRsuFrameDuration//

inline
TimeType T109Mac::CalculateNextTransmissionEnabledTime() const
{
    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
    const TimeType timeFromBaseTime = (currentTime % timeDivisionInterval);
    const TimeType currentBaseTime = currentTime - timeFromBaseTime;

    if (accessMode == ACCESS_MODE_OBE) {

        assert(currentTime < allSlotInformationExpirationTime);

        for(size_t i = 0; i < rsuFrameSlots.size(); i++) {
            const RsuFrameSlot& rsuFrameSlot = rsuFrameSlots[i];

            if (currentTime < rsuFrameSlot.informationExpirationTime &&
                timeFromBaseTime < rsuFrameSlot.endTimeFromBase) {
                return currentBaseTime + rsuFrameSlot.endTimeFromBase + guardTime;
            }
        }

        const RsuFrameSlot& rsuFrameSlot = rsuFrameSlots.front();

        return currentBaseTime + timeDivisionInterval + rsuFrameSlot.endTimeFromBase + guardTime;
    } else {
        for(size_t i = 0; i < rsuFrameSlotNumbers.size(); i++) {
            const RsuFrameSlot& rsuFrameSlot = rsuFrameSlots.at(rsuFrameSlotNumbers[i]);

            if (timeFromBaseTime <= rsuFrameSlot.startTimeFromBase) {
                return currentBaseTime + rsuFrameSlot.startTimeFromBase + guardTime;
            }
        }

        assert(!rsuFrameSlotNumbers.empty());
        return currentBaseTime + timeDivisionInterval + rsuFrameSlots.at(rsuFrameSlotNumbers.front()).startTimeFromBase + guardTime;
    }
}

inline
TimeType T109Mac::CalculateFrameTransmissionDuration(const size_t networkPacketSizeBytes) const
{
    const size_t macFrameSizeBytes =
        networkPacketSizeBytes +
        sizeof(MacFrameHeaderType) +
        sizeof(LLcHeaderType) +
        sizeof(RsuControlHeaderType);

    return physicalLayer.CalculateFrameTransmitDuration(
        macFrameSizeBytes, datarateBitsPerSec);
}

inline
void T109Mac::RememberNewSequenceNumber(
    const SixByteMacAddressType& transmitterAddress,
    const unsigned short int sequenceNumber)
{
    lastSequenceNumberMap[transmitterAddress] = sequenceNumber;
}

inline
bool T109Mac::HaveAlreadySeenThisPacketBefore(
    const SixByteMacAddressType& transmitterAddress,
    const unsigned short int sequenceNumber) const
{
    typedef map<SixByteMacAddressType, unsigned short int>::const_iterator IterType;

    IterType iter = lastSequenceNumberMap.find(transmitterAddress);

    if ((iter == lastSequenceNumberMap.end()) || (sequenceNumber != iter->second)) {
        return false;
    }

    return true;

}//HaveAlreadySeenThisPacketBefore//

inline
void T109Mac::ProcessWakeupTimerEvent()
{
    wakeupTimerEventTicket.Clear();

    switch (macState) {
    case WAITING_FOR_NAV_EXPIRATION_STATE:
        (*this).ProcessVirtualCarrierSenseAkaNavExpiration();
        break;

    case WAITING_FOR_IFS_AND_BACKOFF_STATE:
        (*this).ProcessIfsAndBackoffExpiration();
        break;

    case TRANSMISSION_SUPRESSION_STATE:
        macState = IDLE_STATE;
        (*this).StartIfsAndBackoffIfNecessary();
        break;

    case IDLE_STATE:
        assert(false); abort(); break;
    default:
        assert(false); abort(); break;
    }//switch//

}//ProcessWakeupTimerEvent//

inline
void T109Mac::SwitchToChannel(const unsigned int channel)
{
    if (physicalLayer.IsTransmittingAFrame()) {

        macState = CHANGING_CHANNELS_STATE;
        switchingToThisChannel = channel;
    }
    else {
        physicalLayer.SwitchToChannelNumber(channel);

    }//if//

}//SwitchToChannel//


inline
void T109Mac::OutputTraceAndStatsForFrameReceive(const Packet& aFrame) const
{
    const MacFrameHeaderType& header =
        aFrame.GetAndReinterpretPayloadData<MacFrameHeaderType>();

    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            T109MacFrameReceiveTraceRecord traceData;

            const PacketIdType& packetId = aFrame.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            traceData.frameType = header.frameControlField.frameTypeAndSubtype;

            assert(sizeof(traceData) == T109_MAC_FRAME_RECEIVE_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "RxFrame", traceData);
        }
        else {

            ostringstream msgStream;

            msgStream << "PktId= " << aFrame.GetPacketId();

            msgStream << " FrameType= ";

            switch (header.frameControlField.frameTypeAndSubtype) {
            case QOS_DATA_FRAME_TYPE_CODE: {
                msgStream << "Data";
                break;
            }
            default:
                assert(false); abort();
                break;
            }//switch//

            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "RxFrame", msgStream.str());

        }//if//
    }//if//

    switch (header.frameControlField.frameTypeAndSubtype) {
    case QOS_DATA_FRAME_TYPE_CODE: {
        dataFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    default:
        assert(false); abort();
        break;
    }//switch//

}//OutputTraceAndStatsForFrameReceive//



inline
void T109Mac::OutputTraceForClearChannel() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "ClearCh");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "ClearCh", "");
        }//if//
    }//if//
}

inline
void T109Mac::OutputTraceForBusyChannel() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "BusyCh");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "BusyCh", "");
        }//if//
    }//if//
}

inline
void T109Mac::OutputTraceForNavStart(const TimeType& expirationTime) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            TimeType currentTime = simEngineInterfacePtr->CurrentTime();

            TimeType duration = expirationTime - currentTime;

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "NAV-Start", duration);
        }
        else {
            ostringstream msgStream;

            TimeType currentTime = simEngineInterfacePtr->CurrentTime();

            msgStream << "Dur= " << ConvertTimeToStringSecs(expirationTime - currentTime);

            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "NAV-Start", msgStream.str());
        }//if//
    }//if//
}

inline
void T109Mac::OutputTraceForNavExpiration() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "NAV-End");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "NAV-End", "");
        }//if//
    }//if//
}

inline
void T109Mac::OutputTraceForIfsAndBackoffStart(const TimeType& ifsAndBackoffDuration) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            T109MacIfsAndBackoffStartTraceRecord traceData;

            traceData.accessCategory = 0;
            traceData.duration = ifsAndBackoffDuration;
            traceData.frameCorrupt = false;

            assert(sizeof(traceData) == T109_MAC_IFS_AND_BACKOFF_START_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "IFSAndBackoffStart", traceData);
        }
        else {
            ostringstream msgStream;

            msgStream
                << "AC= " << 0
                << " Dur= " << ConvertTimeToStringSecs(ifsAndBackoffDuration)
                << " Ext= No";

            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "IFSAndBackoffStart", msgStream.str());

        }//if//
    }//if//

}//OutputTraceForIfsAndBackoffStart//


inline
void T109Mac::OutputTraceForInterruptedIfsAndBackoff() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            T109MacIfsAndBackoffPauseTraceRecord traceData;

            traceData.accessCategory = 0;
            traceData.leftDuration = (*this).CalculateCurrentBackoffDuration();

            assert(sizeof(traceData) == T109_MAC_IFS_AND_BACKOFF_PAUSE_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "IFSAndBackoffPause", traceData);
        }
        else {
            ostringstream msgStream;

            msgStream
                << "AC= " << 0
                << " Left= " << ConvertTimeToStringSecs((*this).CalculateCurrentBackoffDuration());

            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "IFSAndBackoffPause", msgStream.str());

        }//if//
    }//if//
}


inline
void T109Mac::OutputTraceForIfsAndBackoffExpiration() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "IFSAndBackoffEnd");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "IFSAndBackoffEnd", "");
        }//if//
    }//if//
}

inline
void T109Mac::OutputTraceAndStatsForBroadcastDataFrameTransmission(const Packet& packet) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            T109MacTxBroadcastDataTraceRecord traceData;

            const PacketIdType& packetId = packet.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();
            traceData.accessCategory = 0;

            assert(sizeof(traceData) == T109_MAC_TX_BROADCAST_DATA_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "Tx-DATA-B", traceData);
        }
        else {
            ostringstream msgStream;

            msgStream << "PktId= " << packet.GetPacketId() << " AC= " << 0;

            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "Tx-DATA-B", msgStream.str());
        }//if//
    }//if//

    broadcastDataFramesSentStatPtr->IncrementCounter();

}//OutputTraceAndStatsForBroadcastDataFrameTransmission//


inline
void T109Mac::ReceiveFrameFromPhy(
    const Packet& aFrame,
    const double receiveRssiDbm,
    const DatarateBitsPerSecType& datarateBitsPerSecond)
{
    using std::max;

    const MacFrameHeaderType& header =
        aFrame.GetAndReinterpretPayloadData<MacFrameHeaderType>();

    if (FrameWithThisAddressIsForThisNode(header.receiverAddress)) {

        OutputTraceAndStatsForFrameReceive(aFrame);

        switch (header.frameControlField.frameTypeAndSubtype) {
        case QOS_DATA_FRAME_TYPE_CODE: {

            (*this).ProcessDataFrame(aFrame, receiveRssiDbm, datarateBitsPerSecond);

            break;
        }
        default:
            assert(false); abort();
            break;

        }//switch//

    }
    else {
        // Not for this node.

        // Set Virtual(Software) Carrier Sense aka NAV.

        const TimeType endOfDurationTime =
            simEngineInterfacePtr->CurrentTime() + (header.duration * MICRO_SECOND);

        mediumReservedUntilTimeAkaNAV =
            std::max<TimeType>(mediumReservedUntilTimeAkaNAV, endOfDurationTime);
    }//if//

}//ReceiveFrameFromPhy//

inline
void T109Mac::ReceivePacketFromUpperLayer(
    unique_ptr<Packet>& packetPtr,
    const SixByteMacAddressType& nextHopAddress)
{
    if ((*this).CalculateFrameTransmissionDuration(packetPtr->LengthBytes()) > maxTransmissionDuration) {
        packetPtr = nullptr;
        return;
    }

    packetEntries.push(PacketEntry(packetPtr, nextHopAddress));

    (*this).StartIfsAndBackoffIfNecessary();
}

inline
void T109Mac::ClearQueuedPacket()
{
    while (!packetEntries.empty()) {
        packetEntries.pop();
    }
}

}//namespace//

#endif
