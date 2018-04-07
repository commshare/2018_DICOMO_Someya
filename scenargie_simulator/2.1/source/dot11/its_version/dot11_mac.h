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

#ifndef DOT11_MAC_H
#define DOT11_MAC_H

#include "scensim_engine.h"
#include "scensim_netsim.h"
#include "scensim_prop.h"
#include "scensim_bercurves.h"
#include "scensim_support.h"
#include "scensim_netif.h"

#include "dot11_common.h"
#include "dot11_headers.h"
#include "dot11_phy.h"
#include "dot11_ratecontrol.h"
#include "dot11_txpowercontrol.h"
#include "dot11_mac_ap.h"
#include "dot11_mac_sta.h"
#include "dot11_incoming_buffer.h"
#include "dot11_tracedefs.h"

#include "its_queues.h"

#include <queue>
#include <map>
#include <string>
#include <vector>
#include <iomanip>

namespace Dot11 {

using std::queue;
using std::deque;
using std::map;
using std::unique_ptr;
using std::cout;
using std::hex;
using std::string;

using ScenSim::MILLI_SECOND;
using ScenSim::MICRO_SECOND;
using ScenSim::NetworkAddress;
using ScenSim::InterfaceOrInstanceIdType;
using ScenSim::NetworkLayer;
using ScenSim::PacketPriorityType;
using ScenSim::MacLayer;
using ScenSim::MacAddressResolver;
using ScenSim::NetworkInterfaceManager;
using ScenSim::GenericMacAddressType;
using ScenSim::DeleteTrailingSpaces;
using ScenSim::ConvertStringToLowerCase;
using ScenSim::ConvertTimeToDoubleSecs;
using ScenSim::EtherTypeFieldType;
using ScenSim::ETHERTYPE_IS_NOT_SPECIFIED;
using ScenSim::EnqueueResultType;
using ScenSim::ENQUEUE_SUCCESS;
using ScenSim::TraceMac;
using ScenSim::MacAndPhyInfoInterface;
using ScenSim::MakeLowerCaseString;
using ScenSim::ConvertAStringSequenceOfANumericTypeIntoAVector;
using ScenSim::RoundToUint;
using ScenSim::IncrementTwelveBitSequenceNumber;
using ScenSim::CalcTwelveBitSequenceNumberDifference;
using ScenSim::RoundUpToNearestIntDivisibleBy4;
using ScenSim::ConvertToUShortInt;
using ScenSim::ConvertToUChar;
using ScenSim::SimpleMacPacketHandler;


//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

inline
void AddMpduDelimiterAndPaddingToFrame(Packet& aFrame)
{
    aFrame.AddPlainStructHeader(MpduDelimiterFrameType());
    MpduDelimiterFrameType& mpduDelimiter =
        aFrame.GetAndReinterpretPayloadData<MpduDelimiterFrameType>();

    mpduDelimiter.lengthBytes = aFrame.LengthBytes();

    aFrame.AddTrailingPadding(
        RoundUpToNearestIntDivisibleBy4(aFrame.LengthBytes()) - aFrame.LengthBytes());

}//AddMpduDelimiterAndPaddingToFrame//


inline
void RemoveMpduDelimiterAndPaddingFromFrame(Packet& aFrame)
{
    const MpduDelimiterFrameType& mpduDelimiter =
        aFrame.GetAndReinterpretPayloadData<MpduDelimiterFrameType>();

    aFrame.RemoveTrailingPadding(aFrame.LengthBytes() - mpduDelimiter.lengthBytes);

    aFrame.DeleteHeader(sizeof(MpduDelimiterFrameType));

}//RemoveMpduDelimiterAndPaddingFromFrame//


inline
void RemoveMpduAggregationPaddingFromFrame(Packet& aFrame)
{
    const MpduDelimiterFrameType& mpduDelimiter =
        aFrame.GetAndReinterpretPayloadData<MpduDelimiterFrameType>();

    aFrame.RemoveTrailingPadding(aFrame.LengthBytes() - mpduDelimiter.lengthBytes);

}//RemoveMpduAggregationPaddingFromFrame//


class Dot11Mac;


// Only allows for one non-infrastructure interface to the same channel at a time.


class SimpleMacAddressResolver : public MacAddressResolver<MacAddressType> {
public:
    SimpleMacAddressResolver(Dot11Mac* initMacPtr) : macPtr(initMacPtr) { }

    void GetMacAddress(
        const NetworkAddress& aNetworkAddress,
        const NetworkAddress& networkAddressMask,
        bool& wasFound,
        MacAddressType& resolvedMacAddress);

    // Used only to get last hop address.

    void GetNetworkAddressIfAvailable(
        const MacAddressType& macAddress,
        const NetworkAddress& subnetNetworkAddress,
        bool& wasFound,
        NetworkAddress& resolvedNetworkAddress);
private:
    Dot11Mac* macPtr;

};//SimpleMacAddressResolver//


//-------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------


class Dot11MacAccessPointScheduler {
public:
    virtual ~Dot11MacAccessPointScheduler() { }

    // virtual void MonitorQueueingOfOutgoingPacket(const size_t accessCategoryIndex) = 0;

    // virtual void MonitorTransmissionOfFrame(
    //    const size_t accessCategory,
    //    const MacAddressType& destinationAddress,
    //    const unsigned int numberFrameBytes,
    //    const DatarateBitsPerSecType& datarateBitsPerSec) = 0;

    // virtual void MonitorIncomingFrame(
    //    const size_t accessCategory,
    //    const MacAddressType& sourceAddress,
    //    const unsigned int numberFrameBytes,
    //    const DatarateBitsPerSecType& datarateBitsPerSec) = 0;

    virtual TimeType CalculateTxopDuration(const size_t accessCategory) = 0;

};//Dot11MacAccessPointScheduler//


// Schedulers are normally in the 802.11 APs and TXOP durations are sent to STAs via
// beacon frames.  As a stopgap, fixed TXOPs can be set for individual STAs.

class FixedTxopScheduler: public Dot11MacAccessPointScheduler {
public:

    FixedTxopScheduler(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType nodeId,
        const InterfaceIdType& interfaceId,
        const size_t numberAccessCategories);


    // Example constructor interface for dynamic scheduler:
    //
    // FixedTxopAccessPointScheduler(
    //     const ParameterDatabaseReader& theParameterDatabaseReader,
    //     const NodeIdType nodeId,
    //     const InterfaceIdType& interfaceId,
    //     const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
    //     const shared_ptr<OutputQueueType> outputQueuePtr,
    //     const Dot11Mac* const macPtr,
    //     const TimeType& fixedTxopDuration);

    // virtual
    TimeType CalculateTxopDuration(const size_t accessCategoryIndex)
        { return txopDurations.at(accessCategoryIndex); }

private:

    vector<TimeType> txopDurations;

};//FixedTxopScheduler//


inline
FixedTxopScheduler::FixedTxopScheduler(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType nodeId,
    const InterfaceIdType& interfaceId,
    const size_t numberAccessCategories)
{
    txopDurations.resize(numberAccessCategories, ZERO_TIME);

    for(size_t i = 0; (i < txopDurations.size()); i++) {
        ostringstream nameStream;
        nameStream << parameterNamePrefix << "edca-category-" << i << "-downlink-txop-duration";
        const string parmName = nameStream.str();

        if (theParameterDatabaseReader.ParameterExists(parmName, nodeId, interfaceId)) {
            txopDurations[i] =
                theParameterDatabaseReader.ReadTime(parmName, nodeId, interfaceId);
        }//if//
    }//for//

}//FixedTxopAccessPointScheduler//


//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

class CongestionMonitoringHandler {
public:

    CongestionMonitoringHandler() {}

    virtual void ScanAReceivedFrame(
        const Packet& aFrame,
        const TimeType& scannedTime) = 0;

private:



};//CongestionMonitoringHandler//


//-------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

enum Dot11MacOperationMode {
    AdhocMode,
    ApMode,
    InfrastructureMode
};

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------


class Dot11Mac : public MacLayer,
    public enable_shared_from_this<Dot11Mac> {
public:
    static const string modelName;

    static const unsigned char AdhocModeAddressSelectorByte = UCHAR_MAX;

    static const TimeType maxTransmitOpportunityAkaTxopDuration = 8160 * MICRO_SECOND;

    static const unsigned int transmitOpportunityNumberPacketNoLimitValue = 0;

    typedef Dot11Phy::PropFrameType PropFrameType;

    static shared_ptr<Dot11Mac> Create(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<NetworkLayer>& networkLayerPtr,
        const shared_ptr<Dot11Phy>& physicalLayerPtr,
        const RandomNumberGeneratorSeedType& nodeSeed)
    {
        shared_ptr<Dot11Mac> macPtr(
            new Dot11Mac(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                nodeId,
                interfaceId,
                interfaceIndex,
                networkLayerPtr,
                physicalLayerPtr,
                nodeSeed));

        macPtr->CompleteInitialization(theParameterDatabaseReader, nodeSeed);
        return (macPtr);

    }//Create//


    ~Dot11Mac();

    void SetMacPacketHandler(
        const EtherTypeFieldType& etherType,
        const shared_ptr<SimpleMacPacketHandler>& initMacPacketHandlerPtr) {
        macPacketHandlerPtrs[etherType] = initMacPacketHandlerPtr;
    }

    void SetNetworkOutputQueue(
        const shared_ptr<ItsOutputQueueWithPrioritySubqueues> initNetworkOutputQueuePtr) {
        networkOutputQueuePtr = initNetworkOutputQueuePtr;
    }

    void SetEdcaParameter(
        const size_t accessCategoryIndex,
        const bool admissionControlIsEnabled,
        const int arbitrationInterframeSpaceNumber,
        const int contentionWindowMin,
        const int contentionWindowMax,
        const TimeType& txopDuration);

    void SetCustomAdaptiveRateController(
        const shared_ptr<Dot11::AdaptiveRateController>& rateControllerPtr)
    {
        this->theAdaptiveRateControllerPtr = rateControllerPtr;
    }

    void SetCongestionMonitoringHandler(
        const shared_ptr<CongestionMonitoringHandler>& initCongestionMonitoringHandlerPtr)
    {
        this->congestionMonitoringHandlerPtr = initCongestionMonitoringHandlerPtr;
    }

    PacketPriorityType GetMaxPacketPriority() const { return maxPacketPriority; }

    shared_ptr<MacAndPhyInfoInterface> GetMacAndPhyInfoInterface() const
        { return physicalLayerPtr->GetDot11InfoInterface(); }

    shared_ptr<Dot11MacInterfaceForPhy> CreateInterfaceForPhy() {
        return shared_ptr<InterfaceForPhy>(new InterfaceForPhy(this));
    }

    // Network Layer Interface:

    virtual void NetworkLayerQueueChangeNotification() override;

    virtual void DisconnectFromOtherLayers() override {
        physicalLayerPtr.reset();
        networkLayerPtr.reset();
        apControllerPtr.reset();
        staControllerPtr.reset();
        macPacketHandlerPtrs.clear();
    }

    virtual GenericMacAddressType GetGenericMacAddress() const override {
        return (myMacAddress.ConvertToGenericMacAddress());
    }

    void SuspendTransmissionFunction();
    void ResumeTransmissionFunction(const TimeType& initTransmissionPermissionEndTime);

    MacAddressType GetMacAddress() const { return (myMacAddress); }

    Dot11MacOperationMode GetOperationMode() const;

    bool GetIpMulticastAddressToMacAddressMappingIsEnabled() const
        { return (ipMulticastAddressToMacAddressMappingIsEnabled); }

    void SendManagementFrame(unique_ptr<Packet>& framePtr);

    unsigned int GetNumberOfChannels() const { return (physicalLayerPtr->GetChannelCount()); }
    unsigned int GetCurrentChannelId() const { return (physicalLayerPtr->GetCurrentChannelNumber()); }

    const vector<unsigned int>& GetCurrentBondedChannelList()
    {
        return (physicalLayerPtr->GetCurrentBondedChannelList());
    }

    double GetRssiOfLastFrameDbm() const { return (physicalLayerPtr->GetRssiOfLastFrameDbm()); }

    void SendAssociationRequest(const MacAddressType& apAddress);

    void SendReassociationRequest(
        const MacAddressType& apAddress,
        const MacAddressType& currentApAddress);

    void SendAssociationResponse(
        const MacAddressType& staAddress,
        const AssociationIdType& associationId);

    void SendReassociationResponse(
        const MacAddressType& staAddress,
        const AssociationIdType& associationId);

    void SendDisassociation(const MacAddressType& receiverAddress);

    void StopReceivingFrames() { physicalLayerPtr->StopReceivingFrames(); }
    void StartReceivingFrames() { physicalLayerPtr->StartReceivingFrames(); }
    bool IsNotReceivingFrames() const { return (physicalLayerPtr->IsNotReceivingFrames()); }

    void SendAuthentication(const MacAddressType& receiverAddress);

    void SendPowerSaveNullFrame(
        const MacAddressType& receiverAddress,
        const bool goingToPowerManagementMode);

    void SwitchToChannel(const unsigned int channelNumber);
    void SwitchToChannels(const vector<unsigned int>& channels);

    // For 802.11AC primary secondary channel scheme.

    unsigned int GetFirstChannelNumberForChannelBandwidth(
        const unsigned int signalChannelBandwidthMhz) const;

    void RequeueBufferedPackets();

    void RequeueBufferedPacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType priority,
        const EtherTypeFieldType etherType,
        const TimeType& timestamp,
        const unsigned int retryTxCount,
        const bool datarateAndTxPowerAreaSpecified,
        const DatarateBitsPerSecType& datarateBitsPerSec,
        const double& txPowerDbm);

    void RequeueManagementFrame(unique_ptr<Packet>& framePtr);

    // For other Mac Layer subsystems:

    void SendLinkIsUpNotificationToNetworkLayer()
        { networkLayerPtr->ProcessLinkIsUpNotification(interfaceIndex); }

    void SendLinkIsDownNotificationToNetworkLayer()
        { networkLayerPtr->ProcessLinkIsDownNotification(interfaceIndex); }

    void SendNewLinkToANodeNotificationToNetworkLayer(const MacAddressType& macAddress) {
        networkLayerPtr->ProcessNewLinkToANodeNotification(
            interfaceIndex, macAddress.ConvertToGenericMacAddress());
    }

    void LookupMacAddressForNeighbor(const NodeIdType targetNodeId, bool& wasFound, MacAddressType& macAddress);

    bool MpduFrameAggregationIsEnabled() const { return (maxAggregateMpduSizeBytes > 0); }

    void SetMpduFrameAggregationIsEnabledFor(const MacAddressType& destinationAddress);

private:

    Dot11Mac(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<NetworkLayer>& networkLayerPtr,
        const shared_ptr<Dot11Phy>& physicalLayerPtr,
        const RandomNumberGeneratorSeedType& nodeSeed);

    void CompleteInitialization(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const RandomNumberGeneratorSeedType& nodeSeed);

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    NodeIdType nodeId;
    InterfaceIdType interfaceId;

    Dot11MacOperationMode operationMode;

    // Note only one "Management Controller" exists at one time
    shared_ptr<Dot11ApManagementController> apControllerPtr;
    shared_ptr<Dot11StaManagementController> staControllerPtr;

    shared_ptr<NetworkLayer> networkLayerPtr;
    map<EtherTypeFieldType, shared_ptr<SimpleMacPacketHandler> > macPacketHandlerPtrs;

    unsigned int interfaceIndex;
    MacAddressType myMacAddress;

    bool ipMulticastAddressToMacAddressMappingIsEnabled;
    std::set<MacAddressType> myMulticastMacAddresses;

    shared_ptr<ItsOutputQueueWithPrioritySubqueues> networkOutputQueuePtr;

    bool refreshNextHopOnDequeueModeIsOn;

    deque<unique_ptr<Packet> > managementFrameQueue;

    IncomingFrameBuffer theIncomingFrameBuffer;

    shared_ptr<AdaptiveRateController> theAdaptiveRateControllerPtr;
    shared_ptr<AdaptiveTxPowerController> theAdaptiveTxPowerControllerPtr;

    shared_ptr<Dot11Phy> physicalLayerPtr;

    unique_ptr<MacAddressResolver<MacAddressType> > theMacAddressResolverPtr;

    shared_ptr<CongestionMonitoringHandler> congestionMonitoringHandlerPtr;

    unique_ptr<Dot11MacAccessPointScheduler> macSchedulerPtr;

    //-----------------------------------------------------

    // Acronyms: AIFS,DIFS,EIFS,SIFS: {Arbitration, Distributed, Extended, Short} InterFrame Space

    // EDCA = "Enhanced Distributed Channel Access" for QoS (added in 802.11e)
    // DCF = "Distributed Coordination Function" (802.11 classic access protocol).

    // Mac Non-EDCA specific parameters:

    unsigned int rtsThresholdSizeBytes;

    unsigned int shortFrameRetryLimit;
    unsigned int longFrameRetryLimit;

    // "Short Interframe Space" = SIFS

    TimeType aShortInterframeSpaceTime;
    TimeType aSlotTime;

    // Cannot detect an incoming signal while switching to transmission mode.

    TimeType aRxTxTurnaroundTime;

    // Extra interframe delay is added if last frame was corrupted (and thus virtual carrier sense
    // is disabled).

    TimeType additionalDelayForExtendedInterframeSpaceMode;

    TimeType clearToSendTimeoutDuration;
    TimeType ackTimeoutDuration;

    int contentionWindowSlotsMin;
    int contentionWindowSlotsMax;

    enum AckDatarateSelectionType {
        SameAsData,
        Lowest
    };

    // To make this (counter-intuitive) behavior explicit.

    bool allowFrameAggregationWithTxopZero;

    bool protectAggregateFramesWithSingleAckedFrame;

    unsigned int maxAggregateMpduSizeBytes;

    AckDatarateSelectionType ackDatarateSelection;

    RandomNumberGenerator aRandomNumberGenerator;

    enum MacStateType {
        IdleState,
        BusyMediumState,
        CtsOrAckTransmissionState,
        WaitingForNavExpirationState,
        WaitingForIfsAndBackoffState,
        WaitingForCtsState,
        WaitingForAckState,
        ChangingChannelsState,
        TransientState,
    };

    MacStateType macState;

    enum SentFrameTypesType {
        ShortFrame,
        RequestToSendFrame,
        LongFrame,
        AggregateFrame,
        BlockAckRequestFrame,
        PowerSavePollResponse,
    };

    SentFrameTypesType lastSentFrameWasAn;

    MacAddressType lastSentDataFrameDestinationMacAddress;

    unsigned int powerSavePollResponsePacketTxCount;

    TimeType mediumReservedUntilTimeAkaNAV;

    bool lastFrameReceivedWasCorrupt;

    TimeType transmissionPermissionEndTime; // for receiving only mode

    // Used by block ack processing.

    MacAddressType currentIncomingAggregateFramesSourceMacAddress;
    PacketPriorityType currentIncomingAggregateFramesTrafficId;
    unsigned int numSubframesReceivedFromCurrentAggregateFrame;

    //-----------------------------------------------------

    struct EdcaAccessCategoryInfo {
        // Parameters
        vector<PacketPriorityType> listOfPriorities;
        unsigned int minContentionWindowSlots;
        unsigned int maxContentionWindowSlots;
        unsigned int arbitrationInterframeSpaceDurationSlots;
        TimeType transmitOpportunityDurationAkaTxop;
        TimeType frameLifetime;//dot11EDCATableMSDULifetime//

        // State variables
        unsigned int currentContentionWindowSlots;
        unsigned int currentNumOfBackoffSlots;

        // Calculated backoff duration without dynamically added Extended Interframe space.
        TimeType currentNonExtendedBackoffDuration;

        bool tryingToJumpOnMediumWithoutABackoff;
        bool hasPacketToSend;

        unsigned int currentShortFrameRetryCount;
        unsigned int currentLongFrameRetryCount;
        unsigned int currentAggregateFrameRetryCount;

        TimeType ifsAndBackoffStartTime;

        // currentPacketPtr (non-aggregate) sent before aggregate frame.

        unique_ptr<Packet> currentPacketPtr;
        unique_ptr<vector<unique_ptr<Packet> > > currentAggregateFramePtr;
        bool currentAggregateFrameIsAMpduAggregate;

        bool currentPacketIsAManagementFrame;
        NetworkAddress currentPacketsNextHopNetworkAddress;
        MacAddressType currentPacketsDestinationMacAddress;
        unsigned short int currentPacketSequenceNumber;
        PacketPriorityType currentPacketPriorityAkaTrafficId;
        EtherTypeFieldType currentPacketsEtherType;
        TimeType currentPacketsTimestamp;

        TimeType timeLeftInCurrentTransmitOpportunity;

        bool currentPacketDatarateAndTxPowerAreSpecified;
        DatarateBitsPerSecType specifiedPacketDatarateBitsPerSec;
        double specifiedPacketTxPowerDbm;

        EdcaAccessCategoryInfo()
            :
            currentNumOfBackoffSlots(0),
            currentNonExtendedBackoffDuration(INFINITE_TIME),
            currentShortFrameRetryCount(0),
            currentLongFrameRetryCount(0),
            currentAggregateFrameRetryCount(0),
            currentPacketsTimestamp(ZERO_TIME),
            hasPacketToSend(false),
            currentPacketSequenceNumber(0),
            currentPacketPriorityAkaTrafficId(0),
            currentPacketsEtherType(ETHERTYPE_IS_NOT_SPECIFIED),
            tryingToJumpOnMediumWithoutABackoff(false),
            frameLifetime(INFINITE_TIME),
            transmitOpportunityDurationAkaTxop(ZERO_TIME),
            timeLeftInCurrentTransmitOpportunity(ZERO_TIME),
            currentPacketIsAManagementFrame(false),
            ifsAndBackoffStartTime(INFINITE_TIME),
            currentAggregateFrameIsAMpduAggregate(true),
            currentPacketDatarateAndTxPowerAreSpecified(false),
            specifiedPacketDatarateBitsPerSec(0),
            specifiedPacketTxPowerDbm(0.0)
        {}

        void operator=(EdcaAccessCategoryInfo&& right)
        {
            assert(this != &right);

            listOfPriorities = right.listOfPriorities;
            minContentionWindowSlots = right.minContentionWindowSlots;
            maxContentionWindowSlots = right.maxContentionWindowSlots;
            arbitrationInterframeSpaceDurationSlots = right.arbitrationInterframeSpaceDurationSlots;
            transmitOpportunityDurationAkaTxop = right.transmitOpportunityDurationAkaTxop;
            frameLifetime = right.frameLifetime;
            currentContentionWindowSlots = right.currentContentionWindowSlots;
            currentNumOfBackoffSlots = right.currentNumOfBackoffSlots;
            currentNonExtendedBackoffDuration = right.currentNonExtendedBackoffDuration;
            tryingToJumpOnMediumWithoutABackoff = right.tryingToJumpOnMediumWithoutABackoff;
            hasPacketToSend = right.hasPacketToSend;
            currentShortFrameRetryCount = right.currentShortFrameRetryCount;
            currentLongFrameRetryCount = right.currentLongFrameRetryCount;
            currentAggregateFrameRetryCount = right.currentAggregateFrameRetryCount;
            ifsAndBackoffStartTime = right.ifsAndBackoffStartTime;
            currentPacketPtr = move(right.currentPacketPtr);
            currentAggregateFramePtr = move(right.currentAggregateFramePtr);
            currentAggregateFrameIsAMpduAggregate = right.currentAggregateFrameIsAMpduAggregate;
            currentPacketIsAManagementFrame = right.currentPacketIsAManagementFrame;
            currentPacketsNextHopNetworkAddress = right.currentPacketsNextHopNetworkAddress;
            currentPacketsDestinationMacAddress = right.currentPacketsDestinationMacAddress;
            currentPacketSequenceNumber = right.currentPacketSequenceNumber;
            currentPacketPriorityAkaTrafficId = right.currentPacketPriorityAkaTrafficId;
            currentPacketsEtherType = right.currentPacketsEtherType;
            currentPacketsTimestamp = right.currentPacketsTimestamp;
            currentPacketDatarateAndTxPowerAreSpecified = right.currentPacketDatarateAndTxPowerAreSpecified;
            specifiedPacketDatarateBitsPerSec = right.specifiedPacketDatarateBitsPerSec;
            specifiedPacketTxPowerDbm = right.specifiedPacketTxPowerDbm;
        }//=//

        EdcaAccessCategoryInfo(EdcaAccessCategoryInfo&& right) { (*this) = move(right); }

    };//EdcaAccessCatagoryInfo//


    //-----------------------------------------------------

    PacketPriorityType maxPacketPriority;

    vector<EdcaAccessCategoryInfo> accessCategories;

    unsigned int numberAccessCategories;

    unsigned int accessCategoryIndexForLastSentFrame;

    unsigned int accessCategoryIndexForManagementFrames;

    TimeType currentTransmitOpportunityAkaTxopStartTime;
    TimeType currentTransmitOpportunityAkaTxopEndTime;

    unsigned int currentTransmitOpportunityAckedFrameCount;

    vector<unsigned int> switchingToThisChannelList;

    bool disabledToJumpOnMediumWithoutBackoff;

    //-------------------------------------------------------------------------

    class WakeupTimerEvent : public SimulationEvent {
    public:
        WakeupTimerEvent(Dot11Mac* initMacPtr) : macPtr(initMacPtr) { }
        void ExecuteEvent() { macPtr->ProcessWakeupTimerEvent(); }
    private:
        Dot11Mac* macPtr;
    };


    shared_ptr<WakeupTimerEvent> wakeupTimerEventPtr;
    EventRescheduleTicket wakeupTimerEventTicket;
    TimeType currentWakeupTimerExpirationTime;

    //-------------------------------------------------------------------------

    TimeType mediumBecameIdleTime;

    struct AddressAndTrafficIdMapKeyType {
        MacAddressType transmitterAddress;
        PacketPriorityType trafficId;

        AddressAndTrafficIdMapKeyType(
            const MacAddressType& initTransmitterAddress, const PacketPriorityType& initTrafficId)
            :
            transmitterAddress(initTransmitterAddress), trafficId(initTrafficId) {}

        bool operator<(const AddressAndTrafficIdMapKeyType& right) const {
            return ((transmitterAddress < right.transmitterAddress) ||
                    ((transmitterAddress == right.transmitterAddress) && (trafficId < right.trafficId)));
        }
    };

    struct OutgoingLinkInfoType {
        unsigned short int outgoingSequenceNumber;
        unsigned short int windowStartSequenceNumber;
        // Frames at and before this sequence number will not be resent.
        unsigned short int lastDroppedFrameSequenceNumber;
        bool blockAckSessionIsEnabled;
        bool blockAckRequestNeedsToBeSent;

        OutgoingLinkInfoType() :
            outgoingSequenceNumber(0),
            lastDroppedFrameSequenceNumber(0),
            windowStartSequenceNumber(0),
            blockAckSessionIsEnabled(false),
            blockAckRequestNeedsToBeSent(false) {}
    };

    map<AddressAndTrafficIdMapKeyType, OutgoingLinkInfoType> outgoingLinkInfoMap;

    struct NeighborCapabilitiesType {
        bool mpduFrameAggregationIsEnabled;

        NeighborCapabilitiesType() :
            mpduFrameAggregationIsEnabled(false)
        { }
    };

    map<MacAddressType, NeighborCapabilitiesType> neighborCapabilitiesMap;

    // Statistics:

    //dropped by exceeded retries
    shared_ptr<CounterStatistic> droppedPacketsStatPtr;

    //data
    shared_ptr<CounterStatistic> unicastDataFramesSentStatPtr;
    shared_ptr<CounterStatistic> broadcastDataFramesSentStatPtr;
    shared_ptr<CounterStatistic> unicastDataFramesResentStatPtr;
    shared_ptr<CounterStatistic> dataFramesReceivedStatPtr;
    shared_ptr<CounterStatistic> dataDuplicatedFramesReceivedStatPtr;
    shared_ptr<CounterStatistic> dataAggregateFramesSentStatPtr;
    shared_ptr<CounterStatistic> dataAggregateFramesResentStatPtr;
    shared_ptr<CounterStatistic> dataAggregatedSubframesReceivedStatPtr;

    //control
    shared_ptr<CounterStatistic> ackFramesSentStatPtr;
    shared_ptr<CounterStatistic> ackFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> blockAckFramesSentStatPtr;
    shared_ptr<CounterStatistic> blockAckFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> rtsFramesSentStatPtr;
    shared_ptr<CounterStatistic> rtsFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> ctsFramesSentStatPtr;
    shared_ptr<CounterStatistic> ctsFramesReceivedStatPtr;

    //management
    shared_ptr<CounterStatistic> beaconFramesSentStatPtr;
    shared_ptr<CounterStatistic> beaconFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> associationRequestFramesSentStatPtr;
    shared_ptr<CounterStatistic> associationRequestFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> associationResponseFramesSentStatPtr;
    shared_ptr<CounterStatistic> associationResponseFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> reassociationRequestFramesSentStatPtr;
    shared_ptr<CounterStatistic> reassociationRequestFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> reassociationResponseFramesSentStatPtr;
    shared_ptr<CounterStatistic> reassociationResponseFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> disassociationFramesSentStatPtr;
    shared_ptr<CounterStatistic> disassociationFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> authenticationFramesSentStatPtr;
    shared_ptr<CounterStatistic> authenticationFramesReceivedStatPtr;

    // Parallelism Stuff:

    unsigned int lookaheadIndex;

    //-------------------------------------------------------------------------


    TimeType CalculateAdditionalDelayForExtendedInterframeSpace() const;

    void AddPrioritiesToAnAccessCategory(
        const string& prioritiesInAString,
        const unsigned int accessCategoryIndex);


    void InitializeAccessCategories(const ParameterDatabaseReader& theParameterDatabaseReader);

    void ReadBondedChannelList(
        const string& bondedChannelListString,
        const string& parameterNameForErrorOutput,
        vector<unsigned int>& bondedChannelList);

    void ReadInMulticastGroupNumberList(const ParameterDatabaseReader& theParameterDatabaseReader);

    void SetAccessCategoriesAsEdca(const ParameterDatabaseReader& theParameterDatabaseReader);
    void SetAccessCategoriesAsDcf(const ParameterDatabaseReader& theParameterDatabaseReader);

    bool WakeupTimerIsActive() const { return !wakeupTimerEventTicket.IsNull(); }

    void ScheduleWakeupTimer(const TimeType& wakeupTime);
    void CancelWakeupTimer();

    bool FrameIsForThisNode(const Packet& aFrame) const;

    bool AggregatedSubframeIsForThisNode(const Packet& frame) const;

    // Note: returns accessCategories.size() when all backoffs are INFINITE_TIME.

    unsigned int FindIndexOfAccessCategoryWithShortestBackoff() const;
    unsigned int FindIndexOfAccessCategoryWithEarliestBackoffExpiration() const;

    bool AccessCategoryIsActive(const unsigned int accessCategoryIndex) const;

    bool ThereAreQueuedPacketsForAccessCategory(const unsigned int accessCategoryIndex) const;

    bool NetworkLayerHasPacketForAccessCategory(const unsigned int accessCategoryIndex) const;

    void RetrievePacketFromNetworkLayerForAccessCategory(
        const unsigned int accessCategoryIndex,
        bool& wasRetrieved);

    TimeType CurrentBackoffDuration() const;
    TimeType CurrentBackoffExpirationTime() const;

    void RecalcRandomBackoff(EdcaAccessCategoryInfo& accessCategoryInfo);

    void StartBackoffIfNecessary();
    void StartBackoffForCategory(const unsigned int accessCategoryIndex);

    void StartPacketSendProcessForAnAccessCategory(
        const unsigned int accessCategoryIndex,
        const bool forceBackoff = false);

    void PauseBackoffForAnAccessCategory(const unsigned int accessCategoryIndex, const TimeType& elapsedTime);

    void SubtractElapsedTimeFromBackoffDuration(
        const unsigned int accessCategoryIndex,
        const TimeType& elapsedTime);

    void PerformInternalCollisionBackoff(const unsigned int accessCategoryIndex);

    void DoubleTheContentionWindowAndPickANewBackoff(const unsigned int accessCategoryIndex);

    TimeType CalculateFrameDuration(
        const unsigned int frameWithMacHeaderSizeBytes,
        const TransmissionParametersType& txParameters) const;

    TimeType CalculateFrameDurationWithoutPhyHeader(
        const unsigned int frameWithMacHeaderSizeBytes,
        const TransmissionParametersType& txParameters) const;

    TimeType CalculateControlFrameDuration(const unsigned int frameSizeBytes);

    TimeType CalculateNonExtendedBackoffDuration(const EdcaAccessCategoryInfo& accessCategory) const;
    TimeType CalculateNonExtendedDurationForJumpingOnMediumWithNoBackoff(const EdcaAccessCategoryInfo& accessCategory) const;

    int NumberBackoffSlotsUsedDuringElapsedTime(
        const EdcaAccessCategoryInfo& accessCategory,
        const TimeType& elapsedTime) const;

    void GoIntoWaitForExpirationOfVirtualCarrierSenseAkaNavState();

    TimeType CalculateNavDurationForRtsFrame(
        const MacAddressType& destinationMacAddress,
        const unsigned int packetSizeBytes,
        const TransmissionParametersType& txParameters) const;

    TimeType CalculateNavDurationForCtsFrame(
        const DurationFieldType& durationFromRtsFrame,
        const MacAddressType& destinationAddress) const;

    TimeType CalculateNavDurationForAckedDataFrame(
        const TransmissionParametersType& txParameters) const;

    TimeType CalculateNavDurationForAckedAggregateDataFrame(
        const TransmissionParametersType& txParameters) const;

    TimeType CalculateNextFrameSequenceDuration(const unsigned int accessCategoryIndex) const;

    void AddExtraNavDurationToPacketForNextFrameIfInATxop(
        const unsigned int accessCategoryIndex,
        const TimeType& frameTransmissionEndTime,
        Packet& thePacket) const;

    void CalculateAndSetTransmitOpportunityDurationAkaTxop(const size_t accessCategoryIndex);

    bool FrameTransmissionDurationExceedsTransmissionPermissionEndTime(
        const unsigned int& frameSizeBytes,
        const TransmissionParametersType& txParameters,
        const TimeType& delayUntilTransmitting) const;

    void TransmitAFrame(
        const unsigned int accessCategoryIndex,
        const bool doNotRequestToSendAkaRts,
        const TimeType& delayUntilTransmitting,
        bool& packetHasBeenSentToPhy);

    void TransmitAnAggregateFrame(
        const unsigned int accessCategoryIndex,
        const TransmissionParametersType& txParameters,
        const double& transmitPowerDbm,
        const TimeType& delayUntilTransmitting);

    void TransmitABlockAckRequest(
        const MacAddressType& destinationAddress,
        const PacketPriorityType& trafficId,
        const TimeType& delayUntilTransmitting);

    void DropCurrentPacketAndGoToNextPacket(const unsigned int accessCategoryIndex);
    void DropCurrentAggregateFrameAndGoToNextPacket(const unsigned int accessCategoryIndex);

    void NeverReceivedClearToSendOrAcknowledgementAction();

    void ProcessRequestToSendFrame(
        const RequestToSendFrameType& rtsFrame,
        const TransmissionParametersType& receivedFrameTxParameters);

    void ProcessClearToSendFrame(const ClearToSendFrameType& ctsFrame);

    void ProcessDataFrame(
        const Packet& dataFrame,
        const TransmissionParametersType& receivedFrameTxParameters);

    void ProcessManagementFrame(
        const Packet& managementFrame,
        const TransmissionParametersType& receivedFrameTxParameters);

    void ProcessNullFrame(
        const Packet& nullFrame,
        const TransmissionParametersType& receivedFrameTxParameters);

    void ProcessAcknowledgementFrame(const AcknowledgementAkaAckFrameType& ackFrame);

    void ProcessBlockAckRequestFrame(
        const BlockAcknowledgementRequestFrameType& blockAckRequestFrame,
        const TransmissionParametersType& receivedFrameTxParameters);

    void ProcessBlockAckFrame(const BlockAcknowledgementFrameType& blockAckFrame);

    unique_ptr<Packet> CreateABlockAckRequestFrame(
        const MacAddressType& destinationAddress,
        const PacketPriorityType& trafficId,
        const unsigned short int startFrameSequenceNumber) const;

    void ProcessPowerSavePollFrame(
        const PowerSavePollFrameType& psPollFrame,
        const TransmissionParametersType& pollFrameTxParameters);

    void SendAckFrame(
        const MacAddressType& destinationAddress,
        const TransmissionParametersType& receivedFrameTxParameters);

    void ProcessWakeupTimerEvent();
    void ProcessVirtualCarrierSenseAkaNavExpiration();
    void ProcessInterframeSpaceAndBackoffTimeout();
    void ProcessClearToSendOrAcknowledgementTimeout();

    void GetNewSequenceNumber(
        const MacAddressType& destinationAddress,
        const PacketPriorityType& priority,
        const bool isNonBlockAckedFrame,
        unsigned short int& newSequenceNumber);

    unsigned int CalcNumberFramesLeftInSequenceNumWindow(
        const MacAddressType& destinationAddress,
        const PacketPriorityType& priority) const;

    void SetSequenceNumberWindowStart(
        const MacAddressType& destinationAddress,
        const PacketPriorityType& priority,
        const unsigned short int sequenceNum);

    bool BlockAckSessionIsEnabled(
        const MacAddressType& destinationAddress,
        const PacketPriorityType& priority) const;

    void CheckPriorityToAccessCategoryMapping() const;

    void QueueOutgoingEthernetPacket(unique_ptr<Packet>& ethernetPacketPtr);
    void QueueOutgoingPacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType priority);

    // Phy interface routines

    void BusyChannelAtPhysicalLayerNotification();
    void ClearChannelAtPhysicalLayerNotification();
    void TransmissionIsCompleteNotification();
    void DoSuccessfulTransmissionPostProcessing(const bool wasJustTransmitting);

    void ReceiveFrameFromPhy(
        const Packet& aFrame,
        const TransmissionParametersType& receivedFrameTxParameters);

    void ReceiveAggregatedSubframeFromPhy(
        unique_ptr<Packet>& subframePtr,
        const TransmissionParametersType& receivedFrameTxParameters,
        const unsigned int aggregateFrameSubframeIndex,
        const unsigned int numberSubframes);

    void NotifyThatPhyReceivedCorruptedFrame();

    void NotifyThatPhyReceivedCorruptedAggregatedSubframe(
        const TransmissionParametersType& receivedFrameTxParameters,
        const unsigned int aggregateFrameSubframeIndex,
        const unsigned int numberSubframes);

    class InterfaceForPhy : public Dot11MacInterfaceForPhy {
    public:
        InterfaceForPhy(Dot11Mac* initMacLayerPtr) : macLayerPtr(initMacLayerPtr) { }

        virtual void BusyChannelAtPhysicalLayerNotification() override
            { macLayerPtr->BusyChannelAtPhysicalLayerNotification(); }

        virtual void ClearChannelAtPhysicalLayerNotification() override
            { macLayerPtr->ClearChannelAtPhysicalLayerNotification(); }

        virtual void TransmissionIsCompleteNotification() override
            { macLayerPtr->TransmissionIsCompleteNotification(); }

        virtual void DoSuccessfulTransmissionPostProcessing(const bool wasJustTransmitting) override
            { macLayerPtr->DoSuccessfulTransmissionPostProcessing(wasJustTransmitting); }

        virtual void ReceiveFrameFromPhy(
            const Packet& aFrame,
            const TransmissionParametersType& receivedFrameTxParameters) override
        {
            macLayerPtr->ReceiveFrameFromPhy(aFrame, receivedFrameTxParameters);
        }

        virtual void ReceiveAggregatedSubframeFromPhy(
            unique_ptr<Packet>& subframePtr,
            const TransmissionParametersType& receivedFrameTxParameters,
            const unsigned int aggregateFrameSubframeIndex,
            const unsigned int numberSubframes) override
        {
            macLayerPtr->ReceiveAggregatedSubframeFromPhy(
                subframePtr, receivedFrameTxParameters, aggregateFrameSubframeIndex, numberSubframes);
        }


        virtual void NotifyThatPhyReceivedCorruptedFrame() override
            { macLayerPtr->NotifyThatPhyReceivedCorruptedFrame(); }


        virtual void NotifyThatPhyReceivedCorruptedAggregatedSubframe(
            const TransmissionParametersType& receivedFrameTxParameters,
            const unsigned int aggregateFrameSubframeIndex,
            const unsigned int numberSubframes) override
        {
            macLayerPtr->NotifyThatPhyReceivedCorruptedAggregatedSubframe(
                receivedFrameTxParameters, aggregateFrameSubframeIndex, numberSubframes);
        }

        bool AggregatedSubframeIsForThisNode(const Packet& frame) const override
            { return (macLayerPtr->AggregatedSubframeIsForThisNode(frame)); }

    private:
        Dot11Mac* macLayerPtr;

    };//InterfaceForPhy//

    bool FrameAggregationIsEnabledFor(const MacAddressType& destinationAddress) const;
    bool MpduFrameAggregationIsEnabledFor(const MacAddressType& destinationAddress) const;

    bool NeedToSendABlockAckRequest(
        const MacAddressType& destinationAddress,
        const PacketPriorityType& trafficId) const;

    void BuildAggregateFrameFromCurrentFrame(const unsigned int accessCategoryIndex);

    void SendPacketToNetworkLayer(unique_ptr<Packet>& dataFramePtr);

    void SendBlockAcknowledgementFrame(
        const MacAddressType& destinationAddress,
        const PacketPriorityType& trafficId,
        const TransmissionParametersType& receivedFrameTxParameters);

    //For trace output

    void OutputTraceForClearChannel() const;
    void OutputTraceForBusyChannel() const;
    void OutputTraceForNavStart(const TimeType& expirationTime) const;
    void OutputTraceForNavExpiration() const;

    void OutputTraceForIfsAndBackoffStart(const TimeType& expirationTime) const;
    void OutputTraceForInterruptedIfsAndBackoff(
        const unsigned int accessCategoryIndex,
        const TimeType nonExtendedDurationLeft) const;
    void OutputTraceForIfsAndBackoffExpiration() const;

    void OutputTraceAndStatsForFrameReceive(const Packet& aFrame) const;

    void OutputTraceAndStatsForAggregateSubframeReceive(
        const Packet& aFrame,
        const QosDataFrameHeaderType& dataFrameHeader);

    void OutputTraceForPacketDequeue(const unsigned int accessCategoryIndex,  const TimeType delayUntilAirBorne = 0) const;

    void OutputTraceAndStatsForRtsFrameTransmission(const unsigned int accessCategoryIndex) const;
    void OutputTraceForCtsFrameTransmission() const;
    void OutputTraceAndStatsForAggregatedFrameTransmission(const unsigned int accessCategoryIndex) const;
    void OutputTraceAndStatsForBroadcastDataFrameTransmission(const unsigned int accessCategoryIndex) const;
    void OutputTraceAndStatsForUnicastDataFrameTransmission(const unsigned int accessCategoryIndex) const;
    void OutputTraceAndStatsForManagementFrameTransmission(const unsigned int accessCategoryIndex) const;
    void OutputTraceForAckFrameTransmission() const;
    void OutputTraceAndStatsForBlockAckFrameTransmission() const;

    void OutputTraceForCtsOrAckTimeout() const;

    void OutputTraceAndStatsForPacketRetriesExceeded(const unsigned int accessCategoryIndex) const;
    bool redundantTraceInformationModeIsOn;

};//Dot11Mac//


const unsigned int DefaultRtsThresholdSizeBytes = UINT_MAX;
const unsigned int DefaultShortFrameRetryLimit = 7;
const unsigned int DefaultLongFrameRetryLimit = 4;
const unsigned int DefaultContentionWindowSlotsMin = 15;
const unsigned int DefaultContentionWindowSlotsMax = 1023;
const PacketPriorityType DefaultMaxPacketPriority = 3;
const unsigned int MaxNumberAccessCategories = 4;

inline
TimeType Dot11Mac::CalculateAdditionalDelayForExtendedInterframeSpace() const
{
    TransmissionParametersType txParameters;
    txParameters.channelBandwidthMhz = physicalLayerPtr->GetBaseChannelBandwidthMhz();

    txParameters.modulationAndCodingScheme =
        theAdaptiveRateControllerPtr->GetLowestModulationAndCoding();

    TimeType longestPossibleAckDuration =
        CalculateFrameDuration(sizeof(AcknowledgementAkaAckFrameType), txParameters);

    return (aShortInterframeSpaceTime + longestPossibleAckDuration);

}//CalculateAdditionalDelayForExtendedInterframeSpace//



inline
void Dot11Mac::AddPrioritiesToAnAccessCategory(
    const string& prioritiesInAString,
    const unsigned int accessCategoryIndex)
{
    bool success;
    vector<PacketPriorityType> priorityVector;

    ConvertAStringSequenceOfANumericTypeIntoAVector<PacketPriorityType>(
        prioritiesInAString,
        success,
        priorityVector);

    if (!success) {
        cerr << "Error: Configuration: Bad Priority sequence: " << prioritiesInAString << endl;
        exit(1);
    }//if//

    EdcaAccessCategoryInfo& accessCategory = accessCategories.at(accessCategoryIndex);

    accessCategory.listOfPriorities.clear();

    for(unsigned int i = 0; (i < priorityVector.size()); i++) {
        accessCategory.listOfPriorities.push_back(priorityVector[i]);
    }//for//

}//AddPrioritiesToAnAccessCategory//

inline
void Dot11Mac::CheckPriorityToAccessCategoryMapping() const
{
    for(PacketPriorityType priority = 0; (priority <= maxPacketPriority); priority++) {
        bool found = false;
        for(unsigned int cat = 0; (cat < numberAccessCategories); cat++) {
            for(unsigned int i = 0; (i < accessCategories[cat].listOfPriorities.size()); i++) {
                if (priority == accessCategories[cat].listOfPriorities[i]) {
                    if (found) {
                        cerr << "Error: Duplicate access category to priority mapping for "
                             << "priority = " << priority << endl;
                        exit(1);
                    }//if//
                    found = true;
                }//if//
                if (accessCategories[cat].listOfPriorities[i] > maxPacketPriority) {
                    cerr << "Error: Unexpected priority mapping for "
                         << "priority = " << int(accessCategories[cat].listOfPriorities[i])
                         << ", max priority = " << static_cast<int>(maxPacketPriority) << endl;
                    exit(1);
                }//if//
            }//for//
        }//for//
        if (!found) {
            cerr << "Error: No access category to priority mapping for priority = "
                 << priority << endl;
            exit(1);
        }//if//
    }//for//

}//CheckPriorityToAccessCategoryMapping//




inline
void Dot11Mac::InitializeAccessCategories(const ParameterDatabaseReader& theParameterDatabaseReader)
{
    string qosTypeString = "edca";

    if (theParameterDatabaseReader.ParameterExists(
            "dot11-qos-type", nodeId, interfaceId)) {

        qosTypeString =  MakeLowerCaseString(
            theParameterDatabaseReader.ReadString("dot11-qos-type", nodeId, interfaceId));

    }//if//

    if (qosTypeString == "edca") {
        (*this).SetAccessCategoriesAsEdca(theParameterDatabaseReader);
    }
    else if (qosTypeString == "dcf") {
        (*this).SetAccessCategoriesAsDcf(theParameterDatabaseReader);
    }
    else {
        cerr << "Error: Unknown dot11-qos-type: " << qosTypeString << endl;
        exit(1);
    }//if//

    CheckPriorityToAccessCategoryMapping();

    for(unsigned int i = 0; (i < accessCategories.size()); i++) {
        accessCategories[i].currentContentionWindowSlots = accessCategories[i].minContentionWindowSlots;
    }//for//

    accessCategoryIndexForManagementFrames = static_cast<unsigned int>(accessCategories.size() - 1);

}//InitializeAccessCategories//


inline
void Dot11Mac::SetAccessCategoriesAsEdca(const ParameterDatabaseReader& theParameterDatabaseReader)
{
    // Set defaults for WAVE.  Maybe overidden.

    accessCategories.clear();
    accessCategories.resize(4);

    accessCategories.at(0).minContentionWindowSlots = contentionWindowSlotsMin;
    accessCategories.at(0).maxContentionWindowSlots = contentionWindowSlotsMax;
    accessCategories.at(0).arbitrationInterframeSpaceDurationSlots = 9;

    accessCategories.at(1).minContentionWindowSlots = (contentionWindowSlotsMin+1)/2 - 1;
    accessCategories.at(1).maxContentionWindowSlots = contentionWindowSlotsMax;
    accessCategories.at(1).arbitrationInterframeSpaceDurationSlots = 6;

    accessCategories.at(2).minContentionWindowSlots = (contentionWindowSlotsMin+1)/4 - 1;
    accessCategories.at(2).maxContentionWindowSlots = (contentionWindowSlotsMin+1)/2 - 1;
    accessCategories.at(2).arbitrationInterframeSpaceDurationSlots = 3;

    accessCategories.at(3).minContentionWindowSlots = (contentionWindowSlotsMin+1)/4 - 1;
    accessCategories.at(3).maxContentionWindowSlots = (contentionWindowSlotsMin+1)/2 - 1;
    accessCategories.at(3).arbitrationInterframeSpaceDurationSlots = 2;

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "num-edca-access-categories"), nodeId, interfaceId)) {

        numberAccessCategories =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "num-edca-access-categories"),
                nodeId,
                interfaceId);

        if (numberAccessCategories > MaxNumberAccessCategories) {
            cerr << "Error: the number of 802.11 access categories is greater than: "
                 << MaxNumberAccessCategories << endl;
            exit(1);
        }//if//

        accessCategories.resize(numberAccessCategories);

    }//if//

    assert(maxPacketPriority == 3);

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "max-packet-priority"), nodeId, interfaceId)) {

        unsigned int readMaxPacketPriority =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "max-packet-priority"),
                nodeId,
                interfaceId);

        if (readMaxPacketPriority > 7) {
            cerr << "Error in parameter \"" << (parameterNamePrefix + "max-packet-priority")
                 << "\",  maximum 802.11 priority is 7." << endl;
            exit(1);
        }//if//

        maxPacketPriority = static_cast<PacketPriorityType>(readMaxPacketPriority);

    }//if//


    // Default simple priority mapping: Distribute priorities across categories,
    // excess priorities in the front (low priority access categories).

    const unsigned int minPriPerCategory = (maxPacketPriority+1) / numberAccessCategories;

    assert(minPriPerCategory >= 1);

    const unsigned int excessPriorities = (maxPacketPriority+1) % numberAccessCategories;

    PacketPriorityType priority = 0;
    for(unsigned int i = 0; (i < numberAccessCategories); i++) {
        unsigned int numberPriorities = minPriPerCategory;
        if (i < excessPriorities) {
            numberPriorities++;
        }//if//

        for(unsigned int j = 0; (j < numberPriorities); j++) {
            accessCategories[i].listOfPriorities.push_back(priority);
            priority++;
        }//for//
    }//for//


    // Override defaults if parameters exist.

    for(unsigned int i = 0; (i < accessCategories.size()); i++) {
        ostringstream prefixStream;
        prefixStream << parameterNamePrefix << "edca-category-" << i << '-';
        string prefix = prefixStream.str();

        if (theParameterDatabaseReader.ParameterExists((prefix+"num-aifs-slots"), nodeId, interfaceId)) {
            accessCategories[i].arbitrationInterframeSpaceDurationSlots =
                theParameterDatabaseReader.ReadNonNegativeInt(
                    (prefix+"num-aifs-slots"), nodeId, interfaceId);
        }//if//

        if (theParameterDatabaseReader.ParameterExists((prefix+"contention-window-min-slots"), nodeId, interfaceId)) {
            accessCategories[i].minContentionWindowSlots =
                theParameterDatabaseReader.ReadNonNegativeInt(
                    (prefix+"contention-window-min-slots"), nodeId, interfaceId);
        }//if//

        if (theParameterDatabaseReader.ParameterExists((prefix+"contention-window-max-slots"), nodeId, interfaceId)) {
            accessCategories[i].maxContentionWindowSlots =
                theParameterDatabaseReader.ReadNonNegativeInt(
                    (prefix+"contention-window-max-slots"), nodeId, interfaceId);
        }//if//


        if (theParameterDatabaseReader.ParameterExists((prefix+"priority-list"), nodeId, interfaceId)) {
            string prioritiesInAString =
                theParameterDatabaseReader.ReadString((prefix+"priority-list"), nodeId, interfaceId);
            DeleteTrailingSpaces(prioritiesInAString);
            (*this).AddPrioritiesToAnAccessCategory(prioritiesInAString, i);
        }//if//

        if (theParameterDatabaseReader.ParameterExists((prefix+"frame-lifetime"), nodeId, interfaceId)) {
            accessCategories[i].frameLifetime =
                theParameterDatabaseReader.ReadTime((prefix+"frame-lifetime"), nodeId, interfaceId);
        }//if//

    }//for//

}//SetAccessCategoriesAsEdca//


inline
void Dot11Mac::SetAccessCategoriesAsDcf(const ParameterDatabaseReader& theParameterDatabaseReader)
{
    //Set defaults for DCF. Maybe overidden.

    accessCategories.clear();
    accessCategories.resize(1);

    EdcaAccessCategoryInfo& accessCategoryForDcf = accessCategories.at(0);
    accessCategoryForDcf.minContentionWindowSlots = contentionWindowSlotsMin;
    accessCategoryForDcf.maxContentionWindowSlots = contentionWindowSlotsMax;
    accessCategoryForDcf.arbitrationInterframeSpaceDurationSlots = 2;

    numberAccessCategories = 1;
    maxPacketPriority = 0;

    accessCategoryForDcf.listOfPriorities.push_back(0);

    // Override defaults if parameters exist.

    if (theParameterDatabaseReader.ParameterExists("dot11-dcf-num-difs-slots", nodeId, interfaceId)) {
        accessCategoryForDcf.arbitrationInterframeSpaceDurationSlots =
            theParameterDatabaseReader.ReadNonNegativeInt(
                "dot11-dcf-num-difs-slots", nodeId, interfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("dot11-dcf-contention-window-min-slots", nodeId, interfaceId)) {
        accessCategoryForDcf.minContentionWindowSlots  =
            theParameterDatabaseReader.ReadNonNegativeInt(
                "dot11-dcf-contention-window-min-slots", nodeId, interfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("dot11-dcf-contention-window-max-slots", nodeId, interfaceId)) {
        accessCategoryForDcf.maxContentionWindowSlots =
            theParameterDatabaseReader.ReadNonNegativeInt(
                "dot11-dcf-contention-window-max-slots", nodeId, interfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("dot11-dcf-frame-lifetime", nodeId, interfaceId)) {
        accessCategoryForDcf.frameLifetime =
            theParameterDatabaseReader.ReadTime("dot11-dcf-frame-lifetime", nodeId, interfaceId);
    }//if//

}//SetAccessCategoriesAsDcf//


inline
void Dot11Mac::ReadBondedChannelList(
    const string& bondedChannelListString,
    const string& parameterNameForErrorOutput,
    vector<unsigned int>& bondedChannelList)
{
    bool success;
    ConvertAStringSequenceOfANumericTypeIntoAVector<unsigned int>(
        bondedChannelListString,
        success,
        bondedChannelList);

    if ((!success) || (bondedChannelList.empty())) {
        cerr << "Error in configuration parameter: " << parameterNameForErrorOutput << ":" << endl;
        cerr << "     " << bondedChannelListString << endl;
        exit(1);
    }//if//

    for(unsigned int i = 0; (i < bondedChannelList.size()); i++) {
        if (bondedChannelList[i] >= GetNumberOfChannels()) {
            cerr << "Error: invalid channel number: " << bondedChannelList[i] << endl;
            exit(1);
        }//if//
    }//for//

}//InitBondedChannel//



inline
void Dot11Mac::ReadInMulticastGroupNumberList(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{
    (*this).ipMulticastAddressToMacAddressMappingIsEnabled = false;

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "map-ip-multicast-addresses"), nodeId, interfaceId)) {

        (*this).ipMulticastAddressToMacAddressMappingIsEnabled =
            theParameterDatabaseReader.ReadBool(
                (parameterNamePrefix + "map-ip-multicast-addresses"), nodeId, interfaceId);
    }//if//

    const string parameterName = parameterNamePrefix + "multicast-group-number-list";

    if (theParameterDatabaseReader.ParameterExists(parameterName, nodeId, interfaceId)) {

        if (!ipMulticastAddressToMacAddressMappingIsEnabled) {
            cerr << "Error: MAC multicast is used but "
                 << (parameterNamePrefix + "map-ip-multicast-addresses") << " not enabled." << endl;
            exit(1);
        }//if//

        string multicastGroupNumberListString =
            theParameterDatabaseReader.ReadString(parameterName, nodeId, interfaceId);

        DeleteTrailingSpaces(multicastGroupNumberListString);

        istringstream listStream(multicastGroupNumberListString);

        while (!listStream.eof()) {
            unsigned int multicastGroupNumber;

            listStream >> multicastGroupNumber;

            if (listStream.fail()) {
                cerr << "Error reading " << parameterName << " parameter:" << endl;
                cerr << "   " << multicastGroupNumberListString << endl;
                exit(1);
            }//if//

            if (multicastGroupNumber > NetworkAddress::maxMulticastGroupNumber) {
                cerr << "Error in " << parameterName << ": group number too large:" << endl;
                cerr << "     " << multicastGroupNumberListString << endl;
                exit(1);
            }//if//

            MacAddressType multicastMacAddress;
            multicastMacAddress.SetToAMulticastAddress(multicastGroupNumber);

            (*this).myMulticastMacAddresses.insert(multicastMacAddress);

        }//while//
    }//if//

}//ReadInMulticastGroupNumberList//


#pragma warning(disable:4355)

inline
Dot11Mac::Dot11Mac(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const NodeIdType& initNodeId,
    const InterfaceIdType& initInterfaceId,
    const unsigned int initInterfaceIndex,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
    const shared_ptr<Dot11Phy>& initPhysicalLayerPtr,
    const RandomNumberGeneratorSeedType& nodeSeed)
    :
    simEngineInterfacePtr(simulationEngineInterfacePtr),
    nodeId(initNodeId),
    interfaceId(initInterfaceId),
    operationMode(AdhocMode),
    networkLayerPtr(initNetworkLayerPtr),
    refreshNextHopOnDequeueModeIsOn(false),
    interfaceIndex(initInterfaceIndex),
    rtsThresholdSizeBytes(DefaultRtsThresholdSizeBytes),
    shortFrameRetryLimit(DefaultShortFrameRetryLimit),
    longFrameRetryLimit(DefaultLongFrameRetryLimit),
    contentionWindowSlotsMin(DefaultContentionWindowSlotsMin),
    contentionWindowSlotsMax(DefaultContentionWindowSlotsMax),
    aShortInterframeSpaceTime(INFINITE_TIME),
    aSlotTime(INFINITE_TIME),
    aRxTxTurnaroundTime(INFINITE_TIME),
    additionalDelayForExtendedInterframeSpaceMode(INFINITE_TIME),
    maxPacketPriority(DefaultMaxPacketPriority),
    numberAccessCategories(MaxNumberAccessCategories),
    ackDatarateSelection(SameAsData),
    macState(IdleState),
    mediumReservedUntilTimeAkaNAV(ZERO_TIME),
    mediumBecameIdleTime(ZERO_TIME),
    currentWakeupTimerExpirationTime(INFINITE_TIME),
    lastFrameReceivedWasCorrupt(false),
    transmissionPermissionEndTime(INFINITE_TIME),
    numSubframesReceivedFromCurrentAggregateFrame(0),
    accessCategoryIndexForLastSentFrame(0),
    currentTransmitOpportunityAkaTxopStartTime(ZERO_TIME),
    currentTransmitOpportunityAkaTxopEndTime(ZERO_TIME),
    currentTransmitOpportunityAckedFrameCount(0),
    redundantTraceInformationModeIsOn(false),
    disabledToJumpOnMediumWithoutBackoff(false),
    protectAggregateFramesWithSingleAckedFrame(true),
    allowFrameAggregationWithTxopZero(false),
    maxAggregateMpduSizeBytes(0),
    ipMulticastAddressToMacAddressMappingIsEnabled(false),
    physicalLayerPtr(initPhysicalLayerPtr),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(nodeSeed, initInterfaceIndex)),
    droppedPacketsStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_FramesDropped"))),
    unicastDataFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_Data_UnicastFramesSent"))),
    unicastDataFramesResentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_Data_UnicastFramesResent"))),
    broadcastDataFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_Data_BroadcastFramesSent"))),
    dataFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_Data_FramesReceived"))),
    dataAggregateFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_Data_AggregateFramesSent"))),
    dataAggregateFramesResentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_Data_AggregateFramesResent"))),
    dataAggregatedSubframesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_Data_AggregatedSubframesReceived"))),
    dataDuplicatedFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_Data_DuplicatedFramesReceived"))),
    ackFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_ACK_FramesSent"))),
    ackFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_ACK_FramesReceived"))),
    blockAckFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_BlockACK_FramesSent"))),
    blockAckFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_BlockACK_FramesReceived"))),
    rtsFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_RTS_FramesSent"))),
    rtsFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_RTS_FramesReceived"))),
    ctsFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_CTS_FramesSent"))),
    ctsFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_CTS_FramesReceived"))),
    beaconFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_Beacon_FramesSent"))),
    beaconFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_Beacon_FramesReceived"))),
    associationRequestFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_AssociationRequest_FramesSent"))),
    associationRequestFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_AssociationRequest_FramesReceived"))),
    associationResponseFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_AssociationResponse_FramesSent"))),
    associationResponseFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_AssociationResponse_FramesReceived"))),
    reassociationRequestFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_ReassociationRequest_FramesSent"))),
    reassociationRequestFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_ReassociationRequest_FramesReceived"))),
    reassociationResponseFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_ReassociationResponse_FramesSent"))),
    reassociationResponseFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_ReassociationResponse_FramesReceived"))),
    disassociationFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_Disassociation_FramesSent"))),
    disassociationFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_Disassociation_FramesReceived"))),
    authenticationFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_Authentication_FramesSent"))),
    authenticationFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_Authentication_FramesReceived")))
{
    physicalLayerPtr->SetMacInterfaceForPhy((*this).CreateInterfaceForPhy());

    shared_ptr<NetworkInterfaceManager> networkInterfaceManagerPtr =
        networkLayerPtr->GetNetworkInterfaceManagerPtr(interfaceIndex);

    if (networkInterfaceManagerPtr != nullptr) {
        theMacAddressResolverPtr.reset(
            networkInterfaceManagerPtr->CreateMacAddressResolver());
    }//if//

    if (theMacAddressResolverPtr.get() == nullptr) {
        theMacAddressResolverPtr.reset(new SimpleMacAddressResolver(this));
    }//if//

    aShortInterframeSpaceTime = physicalLayerPtr->GetShortInterframeSpaceDuration();
    aSlotTime = physicalLayerPtr->GetSlotDuration();
    aRxTxTurnaroundTime = physicalLayerPtr->GetRxTxTurnaroundTime();
    TimeType aPhyRxStartDelay = physicalLayerPtr->GetPhyRxStartDelay();
    clearToSendTimeoutDuration = aShortInterframeSpaceTime + aSlotTime + aPhyRxStartDelay;
    ackTimeoutDuration = clearToSendTimeoutDuration;

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "rts-threshold-size-bytes"), nodeId, interfaceId)) {

        rtsThresholdSizeBytes =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "rts-threshold-size-bytes"), nodeId, interfaceId);

    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "short-frame-retry-limit"), nodeId, interfaceId)) {

        shortFrameRetryLimit =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "short-frame-retry-limit"), nodeId, interfaceId);

        if (shortFrameRetryLimit < 1) {
            cerr << "Error: "<< parameterNamePrefix
                << "short-frame-retry-limit must be greater than or equal to 1: "
                << shortFrameRetryLimit << endl;
            exit(1);
        }//if//

    }//if//


    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "long-frame-retry-limit"), nodeId, interfaceId)) {

        longFrameRetryLimit =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "long-frame-retry-limit"), nodeId, interfaceId);

        if (longFrameRetryLimit < 1) {
            cerr << "Error: "<< parameterNamePrefix
                << "long-frame-retry-limit must be greater than or equal to 1: "
                << longFrameRetryLimit << endl;
            exit(1);
        }//if//

    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "contention-window-min-slots"), nodeId, interfaceId)) {

        contentionWindowSlotsMin =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "contention-window-min-slots"), nodeId, interfaceId);

    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "contention-window-max-slots"), nodeId, interfaceId)) {

        contentionWindowSlotsMax =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "contention-window-max-slots"), nodeId, interfaceId);

    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "disabled-to-jump-on-medium-without-backoff"), nodeId, interfaceId)) {

        disabledToJumpOnMediumWithoutBackoff =
            theParameterDatabaseReader.ReadBool(
                (parameterNamePrefix + "disabled-to-jump-on-medium-without-backoff"), nodeId, interfaceId);

    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "max-aggregate-mpdu-size-bytes"), nodeId, interfaceId)) {

        maxAggregateMpduSizeBytes =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "max-aggregate-mpdu-size-bytes"), nodeId, interfaceId);

    }//if//

    protectAggregateFramesWithSingleAckedFrame = true;

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "protect-aggregate-frames-with-single-acked-frame"), nodeId, interfaceId)) {

        protectAggregateFramesWithSingleAckedFrame =
            theParameterDatabaseReader.ReadBool(
                (parameterNamePrefix + "protect-aggregate-frames-with-single-acked-frame"),
                nodeId, interfaceId);
    }//if//


    allowFrameAggregationWithTxopZero = false;

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "allow-frame-aggregation-with-txop-zero"), nodeId, interfaceId)) {

        allowFrameAggregationWithTxopZero =
            theParameterDatabaseReader.ReadBool(
                (parameterNamePrefix + "allow-frame-aggregation-with-txop-zero"), nodeId, interfaceId);

    }//if//


    const shared_ptr<SimplePropagationModelForNode<PropFrameType> > propModelInterfacePtr =
        physicalLayerPtr->GetPropModelInterface();

    //rate controller
    theAdaptiveRateControllerPtr =
        CreateAdaptiveRateController(
            simulationEngineInterfacePtr,
            theParameterDatabaseReader,
            nodeId,
            interfaceId,
            physicalLayerPtr->GetBaseChannelBandwidthMhz());

    theAdaptiveRateControllerPtr->SetModulationAndCondingMapForDatarateSpecifiedFrame(
        physicalLayerPtr->GetModulationAndCodingShemeMap());

    //power controller
    theAdaptiveTxPowerControllerPtr =
        CreateAdaptiveTxPowerController(
            simulationEngineInterfacePtr,
            theParameterDatabaseReader,
            nodeId,
           physicalLayerPtr->GetInterfaceId());

    additionalDelayForExtendedInterframeSpaceMode = CalculateAdditionalDelayForExtendedInterframeSpace();

    (*this).InitializeAccessCategories(theParameterDatabaseReader);

    wakeupTimerEventPtr = shared_ptr<WakeupTimerEvent>(new WakeupTimerEvent(this));

    networkOutputQueuePtr.reset(
        new ItsOutputQueueWithPrioritySubqueues(
            theParameterDatabaseReader,
            interfaceId,
            simulationEngineInterfacePtr,
            maxPacketPriority));

    if (networkLayerPtr != nullptr) {
        networkLayerPtr->SetInterfaceOutputQueue(interfaceIndex, networkOutputQueuePtr);
    }
    else {
        // Only happens with emulation "headless nodes".
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "bonded-channel-number-list"), nodeId, interfaceId)) {

        const string bondedChannelListString =
            theParameterDatabaseReader.ReadString(
                parameterNamePrefix + "bonded-channel-number-list", nodeId, interfaceId);

        vector<unsigned int> bondedChannelList;

        (*this).ReadBondedChannelList(
            bondedChannelListString,
            (parameterNamePrefix + "bonded-channel-number-list"),
            bondedChannelList);

        if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "initial-channel-number"), nodeId, interfaceId)) {

            cerr << "Error in configuration file: both "
                 << (parameterNamePrefix + "bonded-channel-number-list") << " and "
                 << (parameterNamePrefix + "initial-channel-number") << " have been specified." << endl;

            exit(1);
        }//if//

        (*this).SwitchToChannels(bondedChannelList);
    }
    else {
        unsigned int initialChannel = 0;

        if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "initial-channel-number"), nodeId, interfaceId)) {

            initialChannel =
                theParameterDatabaseReader.ReadNonNegativeInt(
                    (parameterNamePrefix + "initial-channel-number"), nodeId, interfaceId);

            if (initialChannel >= GetNumberOfChannels()) {
                cerr << "Error: invalid channel number: " << initialChannel << endl;
                exit(1);
            }//if//
        }//if//

        // Multiple MAC can refere the same PHY device on WAVE.

        if (physicalLayerPtr->GetCurrentBondedChannelList().empty()) {
            (*this).SwitchToChannel(initialChannel);
        }//if//

    }//if//

    // Parallelism Stuff

    (*this).lookaheadIndex = (*this).simEngineInterfacePtr->AllocateLookaheadTimeIndex();
    simEngineInterfacePtr->SetALookaheadTimeForThisNode(
        physicalLayerPtr->GetRxTxTurnaroundTime(), lookaheadIndex);

    if (theParameterDatabaseReader.ParameterExists(parameterNamePrefix + "redundant-trace-information-mode")) {
        redundantTraceInformationModeIsOn =
            theParameterDatabaseReader.ReadBool(parameterNamePrefix + "redundant-trace-information-mode");
    }//if//


    if (theParameterDatabaseReader.ParameterExists("channel-instance-id", nodeId, interfaceId)) {
        InterfaceOrInstanceIdType propModelInstanceId =
            theParameterDatabaseReader.ReadString("channel-instance-id", nodeId, interfaceId);

        ConvertStringToLowerCase(propModelInstanceId);

        if (propModelInstanceId != propModelInterfacePtr->GetInstanceId()) {
            cerr << "Error: Declared propagation model instance ID, does not match actual prop model ID." << endl;
            cerr << "       Node = " << nodeId << ", Interface = " << interfaceId << endl;
            cerr << "       Declared ID = " << propModelInstanceId << ", Actual ID = "
                 << propModelInterfacePtr->GetInstanceId() << endl;
            exit(1);
        }//if//

    }//if//


    // Only not equal to nodeId in emulation "split node" mode.

    const NodeIdType primaryNodeId = theParameterDatabaseReader.GetPossibleNodeIdRemap(nodeId);

    // Mac address related multiple interface restrictions:
    //   - Only one Adhoc interface on a single propagation environment.
    //   - Can't have two interfaces associate with same AP.

    myMacAddress.SetLowerBitsWithNodeId(primaryNodeId);

    if (operationMode == AdhocMode) {
        myMacAddress.SetInterfaceSelectorByte(AdhocModeAddressSelectorByte);
    }
    else {
        myMacAddress.SetInterfaceSelectorByte(static_cast<unsigned char>(interfaceIndex));
    }//if//


    (*this).ReadInMulticastGroupNumberList(theParameterDatabaseReader);

    // Allow STAs to use fixed TXOP durations set in the parameter file
    // Normally, STA TXOPs are controlled by the AP sent via Beacon frames.

    macSchedulerPtr.reset(
        new FixedTxopScheduler(
            theParameterDatabaseReader,
            nodeId,
            interfaceId,
            accessCategories.size()));

}//Dot11Mac()//


#pragma warning(default:4355)


inline
void Dot11Mac::CompleteInitialization(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const RandomNumberGeneratorSeedType& nodeSeed)
{
    //default

    string nodeTypeString = "ad-hoc";

    if (theParameterDatabaseReader.ParameterExists(parameterNamePrefix + "node-type", nodeId, interfaceId)) {

        nodeTypeString = theParameterDatabaseReader.ReadString(parameterNamePrefix + "node-type", nodeId, interfaceId);
        ConvertStringToLowerCase(nodeTypeString);
    }//if//

    if (nodeTypeString == "access-point") {
        operationMode = ApMode;
        apControllerPtr.reset(
            new Dot11ApManagementController(
                shared_from_this(),
                simEngineInterfacePtr,
                theParameterDatabaseReader,
                nodeId,
                interfaceId,
                HashInputsToMakeSeed(nodeSeed, interfaceIndex)));
    }
    else if (nodeTypeString == "mobile-sta") {

        operationMode = InfrastructureMode;

        staControllerPtr.reset(
            new Dot11StaManagementController(
                shared_from_this(),
                simEngineInterfacePtr,
                theParameterDatabaseReader,
                nodeId,
                interfaceId,
                HashInputsToMakeSeed(nodeSeed, interfaceIndex)));
    }
    else if (nodeTypeString == "ad-hoc") {

        operationMode = AdhocMode;

        // No controller needed
    }
    else {
        cerr << parameterNamePrefix << "node-type \"" << nodeTypeString << "\" is not defined." << endl;
        exit(1);
    }//if//

    assert((apControllerPtr == nullptr) || (staControllerPtr == nullptr));

}//CompleteInitialization//



inline
bool Dot11Mac::FrameIsForThisNode(const Packet& aFrame) const
{
    const CommonFrameHeaderType& header = aFrame.GetAndReinterpretPayloadData<CommonFrameHeaderType>();

    bool isForMe = false;

    if ((header.receiverAddress == myMacAddress) ||
        (header.receiverAddress.IsABroadcastAddress()) ||
        ((header.receiverAddress.IsAMulticastAddress() &&
         (myMulticastMacAddresses.count(header.receiverAddress) == 1)))) {

        // For me or broadcast

        switch (GetOperationMode()) {
        case AdhocMode:
            isForMe = true;
            break;

        case InfrastructureMode:
        {
            if (header.frameControlField.frameTypeAndSubtype == QOS_DATA_FRAME_TYPE_CODE) {

                // Data frame

                const QosDataFrameHeaderType& dataFrameHeader =
                    aFrame.GetAndReinterpretPayloadData<QosDataFrameHeaderType>();

                bool wasFound = false;
                MacAddressType apMacAddress;
                staControllerPtr->GetCurrentAccessPointAddress(wasFound, apMacAddress);

                // Receive only from associated ap

                isForMe = ((wasFound) && (dataFrameHeader.transmitterAddress == apMacAddress));
            }
            else {
                //Control and managment frames

                isForMe = true;
            }//if//
            break;
        }

        case ApMode:
        {
            if (header.frameControlField.frameTypeAndSubtype == QOS_DATA_FRAME_TYPE_CODE) {

                // Data frame

                const QosDataFrameHeaderType& dataFrameHeader =
                    aFrame.GetAndReinterpretPayloadData<QosDataFrameHeaderType>();

                // Receive only from associated sta
                isForMe = apControllerPtr->IsAnAssociatedStaAddress(dataFrameHeader.transmitterAddress);

            }
            else {
                // Control and managment frames

                isForMe = true;
            }//if//
            break;
        }
        default:
            assert(false); abort(); break;
        }//switch//
    }
    else {
        // For others
        // Need to receive frames for others if mac supports link level multi-hop

    }//if//

    if ((!isForMe) && (header.receiverAddress == myMacAddress)) {
        // Delete this if it starts (correctly) spewing too many messages .
        cerr << "Warning: 802.11 Frame addressed to node is unreceivable (not associated): frame dropped" << endl;
    }//if//

    return (isForMe);

}//FrameIsForThisNode//


inline
bool Dot11Mac::AggregatedSubframeIsForThisNode(const Packet& frame) const
{
    const CommonFrameHeaderType& header =
        frame.GetAndReinterpretPayloadData<CommonFrameHeaderType>(sizeof(MpduDelimiterFrameType));

    assert(!header.receiverAddress.IsABroadcastOrAMulticastAddress());

    return (header.receiverAddress == myMacAddress);
}



inline
void Dot11Mac::ScheduleWakeupTimer(const TimeType& wakeupTime)
{
    (*this).currentWakeupTimerExpirationTime = wakeupTime;

    if (wakeupTimerEventTicket.IsNull()) {
        simEngineInterfacePtr->ScheduleEvent(
            wakeupTimerEventPtr, wakeupTime, wakeupTimerEventTicket);
    }
    else {
        simEngineInterfacePtr->RescheduleEvent(wakeupTimerEventTicket, wakeupTime);

    }//if//

}//ScheduleWakeupTimer//


inline
void Dot11Mac::CancelWakeupTimer()
{
    assert(!wakeupTimerEventTicket.IsNull());
    (*this).currentWakeupTimerExpirationTime = INFINITE_TIME;
    simEngineInterfacePtr->CancelEvent(wakeupTimerEventTicket);
}



inline
TimeType Dot11Mac::CalculateNonExtendedBackoffDuration(
    const EdcaAccessCategoryInfo& accessCategory) const
{
    const int totalSlots =
        (accessCategory.arbitrationInterframeSpaceDurationSlots + accessCategory.currentNumOfBackoffSlots);

    return (aShortInterframeSpaceTime + (aSlotTime * totalSlots) - aRxTxTurnaroundTime);

}//CalculateNonExtendedBackoffDuration//


inline
TimeType Dot11Mac::CalculateNonExtendedDurationForJumpingOnMediumWithNoBackoff(
    const EdcaAccessCategoryInfo& accessCategory) const
{
    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();

    TimeType ifsDuration = aShortInterframeSpaceTime +
        (aSlotTime * accessCategory.arbitrationInterframeSpaceDurationSlots) - aRxTxTurnaroundTime;

    if ((mediumBecameIdleTime + ifsDuration) >= currentTime) {

        // Wait for the interframe space.

        return (mediumBecameIdleTime + ifsDuration - currentTime);
    } else {
        // Wait until next slot start.

        return (aSlotTime - ((currentTime - mediumBecameIdleTime - ifsDuration) % aSlotTime));
    }//if//

}//CalculateNonExtendedDurationForJumpingOnMediumWithNoBackoff//


inline
bool Dot11Mac::AccessCategoryIsActive(const unsigned int accessCategoryIndex) const
{
    const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    return ((accessCategoryInfo.currentNonExtendedBackoffDuration != INFINITE_TIME) ||
            (accessCategoryInfo.hasPacketToSend));
}

inline
unsigned int Dot11Mac::FindIndexOfAccessCategoryWithEarliestBackoffExpiration() const
{
    unsigned int shortestIndex = static_cast<unsigned int>(accessCategories.size());
    TimeType earliestExpirationTime = INFINITE_TIME;

    // Higher Access Indices have higher priority

    for(int i = (numberAccessCategories - 1); (i >= 0); i--) {
        const EdcaAccessCategoryInfo& accessCategoryInfo =
            accessCategories[i];

        if ((accessCategoryInfo.ifsAndBackoffStartTime != INFINITE_TIME) &&
            (accessCategoryInfo.currentNonExtendedBackoffDuration != INFINITE_TIME)) {

            const TimeType expirationTime =
                (accessCategories[i].ifsAndBackoffStartTime +
                 accessCategories[i].currentNonExtendedBackoffDuration);

            if (expirationTime < earliestExpirationTime) {
                earliestExpirationTime = expirationTime;
                shortestIndex = i;
            }//if//
        }//if//
    }//for//

    return shortestIndex;

}//FindIndexOfAccessCategoryWithEarliestBackoffExpiration//

inline
unsigned int Dot11Mac::FindIndexOfAccessCategoryWithShortestBackoff() const
{
    unsigned int shortestIndex = static_cast<unsigned int>(accessCategories.size());
    TimeType shortestBackoff = INFINITE_TIME;

    // Higher Access Indices have higher priority

    for(int i = (numberAccessCategories - 1); (i >= 0); i--) {
        if (shortestBackoff > accessCategories[i].currentNonExtendedBackoffDuration) {
            shortestBackoff = accessCategories[i].currentNonExtendedBackoffDuration;
            shortestIndex = i;
        }//if//
    }//for//

    return shortestIndex;

}//FindIndexOfAccessCategoryWithShortestBackoff//


inline
TimeType Dot11Mac::CurrentBackoffDuration() const
{
    unsigned int accessCategoryIndex = FindIndexOfAccessCategoryWithShortestBackoff();

    if (accessCategoryIndex == accessCategories.size()) {
        return INFINITE_TIME;
    }//if//

    const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    if (lastFrameReceivedWasCorrupt) {
        return (additionalDelayForExtendedInterframeSpaceMode +
                accessCategoryInfo.currentNonExtendedBackoffDuration);
    }
    else {
        return accessCategoryInfo.currentNonExtendedBackoffDuration;
    }//if//

}//CurrentBackoffDuration//



inline
TimeType Dot11Mac::CurrentBackoffExpirationTime() const
{
    assert(macState == WaitingForIfsAndBackoffState);

    unsigned int accessCategoryIndex = FindIndexOfAccessCategoryWithEarliestBackoffExpiration();

    if (accessCategoryIndex == accessCategories.size()) {
        return INFINITE_TIME;
    }//if//

    const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    if (lastFrameReceivedWasCorrupt) {
        return (additionalDelayForExtendedInterframeSpaceMode +
                accessCategoryInfo.ifsAndBackoffStartTime +
                accessCategoryInfo.currentNonExtendedBackoffDuration);
    }
    else {
        return (accessCategoryInfo.ifsAndBackoffStartTime +
                accessCategoryInfo.currentNonExtendedBackoffDuration);
    }//if//

}//CurrentBackoffExpirationTime//


//-------------------------------------------------------------------------------------------------


inline
void Dot11Mac::RecalcRandomBackoff(EdcaAccessCategoryInfo& accessCategoryInfo)
{
    accessCategoryInfo.currentNumOfBackoffSlots =
        aRandomNumberGenerator.GenerateRandomInt(0, accessCategoryInfo.currentContentionWindowSlots);

    accessCategoryInfo.currentNonExtendedBackoffDuration =
        CalculateNonExtendedBackoffDuration(accessCategoryInfo);
}


inline
void Dot11Mac::StartBackoffIfNecessary()
{
    assert(macState != WaitingForIfsAndBackoffState);

    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();

    if (currentTime >= transmissionPermissionEndTime) {
        macState = IdleState;
        return;
    }

    const TimeType backoffDuration = CurrentBackoffDuration();

    if (backoffDuration == INFINITE_TIME) {
        macState = IdleState;
        return;
    }//if//

    macState = WaitingForIfsAndBackoffState;

    for(unsigned int i = 0; (i < accessCategories.size()); i++) {
        EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[i];
        if (accessCategoryInfo.currentNonExtendedBackoffDuration != INFINITE_TIME) {
            accessCategoryInfo.ifsAndBackoffStartTime = currentTime;
        }
        else {
            accessCategoryInfo.ifsAndBackoffStartTime = INFINITE_TIME;
        }//if//
    }//for//

    (*this).ScheduleWakeupTimer(currentTime + backoffDuration);

    OutputTraceForIfsAndBackoffStart(backoffDuration);

}//StartBackoffIfNecessary//


inline
void Dot11Mac::StartBackoffForCategory(const unsigned int accessCategoryIndex)
{
    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();

    if (currentTime >= transmissionPermissionEndTime) {
        macState = IdleState;
        return;
    }

    assert((macState == IdleState) || (macState == WaitingForIfsAndBackoffState));
    (*this).macState = WaitingForIfsAndBackoffState;

    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];
    accessCategoryInfo.ifsAndBackoffStartTime = currentTime;

    const TimeType wakeupTime = CurrentBackoffExpirationTime();

    if (wakeupTime != currentWakeupTimerExpirationTime) {
        (*this).ScheduleWakeupTimer(wakeupTime);

        OutputTraceForIfsAndBackoffStart(wakeupTime - currentTime);
    }//if//

    assert(macState == WaitingForIfsAndBackoffState);

}//StartBackoffForCategory//




//-------------------------------------------------------------------------------------------------
//
// The following routines are for when a new packet appears at the Network Layer and a
// access category goes from idle to active.
//


inline
void Dot11Mac::StartPacketSendProcessForAnAccessCategory(
    const unsigned int accessCategoryIndex,
    const bool forceBackoff)
{
    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff = false;

    if ((accessCategoryInfo.currentPacketPtr == nullptr) &&
        (accessCategoryInfo.currentAggregateFramePtr == nullptr)) {

        accessCategoryInfo.hasPacketToSend =
            ThereAreQueuedPacketsForAccessCategory(accessCategoryIndex);

    }//if//

    if ((!forceBackoff) && (!accessCategoryInfo.hasPacketToSend)) {
        accessCategoryInfo.currentNonExtendedBackoffDuration = INFINITE_TIME;

        return;

    }//if//

    assert(accessCategoryInfo.currentNonExtendedBackoffDuration == INFINITE_TIME);

    switch (macState) {
    case BusyMediumState:
    case CtsOrAckTransmissionState:
    case WaitingForNavExpirationState:
    case WaitingForCtsState:
    case WaitingForAckState:
    case ChangingChannelsState:
    {
        (*this).RecalcRandomBackoff(accessCategoryInfo);

        break;
    }
    case IdleState:
    case WaitingForIfsAndBackoffState:
    {
        if (disabledToJumpOnMediumWithoutBackoff) {

            // Note: Add complete IFS even if channel is completely idle.

            (*this).RecalcRandomBackoff(accessCategoryInfo);

            accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff = false;
        }
        else {
            assert(!forceBackoff);

            accessCategoryInfo.currentNumOfBackoffSlots = 0;
            accessCategoryInfo.currentNonExtendedBackoffDuration =
                CalculateNonExtendedDurationForJumpingOnMediumWithNoBackoff(accessCategoryInfo);
            accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff = true;
        }

        (*this).StartBackoffForCategory(accessCategoryIndex);

        break;
    }
    default:
        assert(false); abort(); break;
    }//switch//

}//StartPacketSendProcessForAnAccessCategory//


inline
bool Dot11Mac::NetworkLayerHasPacketForAccessCategory(const unsigned int accessCategoryIndex) const
{
    const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    for(unsigned int i = 0; (i < accessCategoryInfo.listOfPriorities.size()); i++) {
        if (networkOutputQueuePtr->HasPacketWithPriority(accessCategoryInfo.listOfPriorities[i])) {
            return true;
        }//if//
    }//for//

    return false;

}//NetworkLayerHasPacketForAccessCategory//



inline
void Dot11Mac::NetworkLayerQueueChangeNotification()
{
    // High Priority to Low priority category order:

    for(int i = (numberAccessCategories-1); (i >= 0); i--) {
        if (!AccessCategoryIsActive(i)) {

            (*this).StartPacketSendProcessForAnAccessCategory(i);

        }//if//
    }//for//

}//NetworkLayerQueueChangeNotification//




inline
bool Dot11Mac::ThereAreQueuedPacketsForAccessCategory(const unsigned int accessCategoryIndex) const
{
    return (((accessCategoryIndex == accessCategoryIndexForManagementFrames) &&
             (!managementFrameQueue.empty())) ||
            (NetworkLayerHasPacketForAccessCategory(accessCategoryIndex)));
}


inline
void Dot11Mac::SuspendTransmissionFunction()
{
    assert(macState != CtsOrAckTransmissionState &&
           macState != TransientState &&
           macState != ChangingChannelsState);

    assert(!physicalLayerPtr->IsTransmittingAFrame());

    switch (macState) {
    case WaitingForCtsState:
    case WaitingForAckState:
        if ((*this).WakeupTimerIsActive()) {
            simEngineInterfacePtr->CancelEvent(wakeupTimerEventTicket);
        }//if//
        if (!physicalLayerPtr->IsReceivingAFrame()) {
            macState = BusyMediumState;
        }//if//
        break;

    case WaitingForIfsAndBackoffState: {
        const TimeType currentTime = simEngineInterfacePtr->CurrentTime();

        for(unsigned int i = 0; (i < accessCategories.size()); i++) {
            TimeType elapsedTime = (currentTime - accessCategories[i].ifsAndBackoffStartTime);

            (*this).SubtractElapsedTimeFromBackoffDuration(i, elapsedTime);
        }//for//
        if ((*this).WakeupTimerIsActive()) {
            simEngineInterfacePtr->CancelEvent(wakeupTimerEventTicket);
        }//if//
        macState = IdleState;
        break;
    }

    case IdleState:
    case BusyMediumState:
    case WaitingForNavExpirationState:
    default:
        break;
    }//switch//

    transmissionPermissionEndTime = simEngineInterfacePtr->CurrentTime();

    // receiving only mode
}//SuspendTransmissionFunction//

inline
void Dot11Mac::ResumeTransmissionFunction(const TimeType& initTransmissionPermissionEndTime)
{
    transmissionPermissionEndTime = initTransmissionPermissionEndTime;
    assert(transmissionPermissionEndTime > simEngineInterfacePtr->CurrentTime());

    if (macState == IdleState) {
        (*this).StartBackoffIfNecessary();;
    }//if//

}//ResumeTransmissionFunction//



//--------------------------------------------------------------------------------------------------
//
// The following routines have to do with effect of on/off transitions of carrier sense
// and virtual carrier sense (NAV) on the backoff of the EDCA access categories.
//


inline
void Dot11Mac::ProcessVirtualCarrierSenseAkaNavExpiration()
{
    assert(macState == WaitingForNavExpirationState);

    OutputTraceForNavExpiration();

    macState = TransientState;
    mediumBecameIdleTime = simEngineInterfacePtr->CurrentTime();
    (*this).StartBackoffIfNecessary();
}



inline
int Dot11Mac::NumberBackoffSlotsUsedDuringElapsedTime(
    const EdcaAccessCategoryInfo& accessCategory,
    const TimeType& elapsedTime) const
{
    TimeType interframeSpaceDuration =
        aShortInterframeSpaceTime + (aSlotTime * accessCategory.arbitrationInterframeSpaceDurationSlots);

    if (lastFrameReceivedWasCorrupt) {
        interframeSpaceDuration += additionalDelayForExtendedInterframeSpaceMode;
    }//if//


    if (elapsedTime <= interframeSpaceDuration) {
        return 0;
    }//if//

    return int((elapsedTime + aRxTxTurnaroundTime - interframeSpaceDuration) / aSlotTime);

}//NumberBackoffSlotsUsedDuringElapsedTime//


inline
void Dot11Mac::PauseBackoffForAnAccessCategory(
    const unsigned int accessCategoryIndex,
    const TimeType& elapsedTime)
{
    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    if (accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff) {
        // Interrupted in Interframe space (IFS), must start backoff process.

        accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff = false;
        accessCategoryInfo.currentNonExtendedBackoffDuration = INFINITE_TIME;

        (*this).StartPacketSendProcessForAnAccessCategory(accessCategoryIndex, true);
    }
    else {

        (*this).SubtractElapsedTimeFromBackoffDuration(accessCategoryIndex, elapsedTime);

    }//if//

}//PauseBackoffForAnAccessCategory//


inline
void Dot11Mac::SubtractElapsedTimeFromBackoffDuration(
    const unsigned int accessCategoryIndex,
    const TimeType& elapsedTime)
{
    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    if (!accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff &&
        (accessCategoryInfo.currentNonExtendedBackoffDuration != INFINITE_TIME)) {
        accessCategoryInfo.currentNumOfBackoffSlots -=
            NumberBackoffSlotsUsedDuringElapsedTime(accessCategoryInfo, elapsedTime);

        assert(accessCategoryInfo.currentNumOfBackoffSlots >= 0);

        accessCategoryInfo.currentNonExtendedBackoffDuration =
            CalculateNonExtendedBackoffDuration(accessCategoryInfo);

        OutputTraceForInterruptedIfsAndBackoff(
            accessCategoryIndex,
            accessCategoryInfo.currentNonExtendedBackoffDuration);
    }//if//

}//SubtractElapsedTimeFromBackoffDuration//

inline
void Dot11Mac::BusyChannelAtPhysicalLayerNotification()
{
    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();

    OutputTraceForBusyChannel();

    switch (macState) {
    case WaitingForNavExpirationState:
        (*this).CancelWakeupTimer();
        macState = BusyMediumState;

        break;

    case WaitingForIfsAndBackoffState: {

        macState = BusyMediumState;

        for(unsigned int i = 0; (i < accessCategories.size()); i++) {
            TimeType elapsedTime = (currentTime - accessCategories[i].ifsAndBackoffStartTime);

            (*this).PauseBackoffForAnAccessCategory(i, elapsedTime);
        }//for//

        (*this).CancelWakeupTimer();

        break;
    }
    case IdleState:
        macState = BusyMediumState;
        break;

    case WaitingForCtsState:
    case WaitingForAckState:

        // Ignore (incoming frame could be for this station).
        break;

    case BusyMediumState:
    case CtsOrAckTransmissionState:

        // Ignore: Still busy
        //assert(false && "PHY sending duplicate notifications, should not happen."); abort();
        break;

    default:
        assert(false); abort(); break;
    }//switch//

}//BusyChannelAtPhysicalLayerNotification//


inline
void Dot11Mac::GoIntoWaitForExpirationOfVirtualCarrierSenseAkaNavState()
{
    assert((macState == BusyMediumState) || (macState == TransientState));
    assert(wakeupTimerEventTicket.IsNull());

    macState = WaitingForNavExpirationState;

    (*this).ScheduleWakeupTimer(mediumReservedUntilTimeAkaNAV);

    OutputTraceForNavStart(mediumReservedUntilTimeAkaNAV);
}


inline
void Dot11Mac::ClearChannelAtPhysicalLayerNotification()
{
    OutputTraceForClearChannel();

    if ((macState == WaitingForCtsState) || (macState == WaitingForAckState)) {
        if (WakeupTimerIsActive()) {
            // Wait until the CTS/ACK timeout completes before taking action.
            return;
        }
        else {
            // Get out of the ACK/CTS timeout state.
            macState = TransientState;

            (*this).NeverReceivedClearToSendOrAcknowledgementAction();
        }//if//

    }//if//

    if (macState == WaitingForNavExpirationState) {
        return;
    }//if//

    assert((macState == BusyMediumState) || (macState == TransientState));

    if (simEngineInterfacePtr->CurrentTime() < mediumReservedUntilTimeAkaNAV) {
        (*this).GoIntoWaitForExpirationOfVirtualCarrierSenseAkaNavState();
    }
    else {
        mediumBecameIdleTime = simEngineInterfacePtr->CurrentTime();
        (*this).StartBackoffIfNecessary();

    }//if//

    assert((macState != BusyMediumState) && (macState != TransientState));

}//ClearChannelAtPhysicalLayerNotification//





inline
void Dot11Mac::DoSuccessfulTransmissionPostProcessing(const bool wasJustTransmitting)
{
    if (currentTransmitOpportunityAkaTxopEndTime == ZERO_TIME) {

        // TXOP is not being used (1 frame per medium access).
        // or number packet limit is exhausted.

        (*this).StartPacketSendProcessForAnAccessCategory(accessCategoryIndexForLastSentFrame, true);
        return;
    }//if//

    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
    const TimeType nextFrameSequenceDuration =
        CalculateNextFrameSequenceDuration(accessCategoryIndexForLastSentFrame);

    if (nextFrameSequenceDuration == ZERO_TIME) {

        // Nothing To Send

        currentTransmitOpportunityAkaTxopEndTime = ZERO_TIME;
        (*this).StartPacketSendProcessForAnAccessCategory(accessCategoryIndexForLastSentFrame, true);

        return;
    }//if//

    if ((currentTime + nextFrameSequenceDuration) > currentTransmitOpportunityAkaTxopEndTime) {

        // Not enough time left in the TXOP for next packet.

        currentTransmitOpportunityAkaTxopEndTime = ZERO_TIME;
        (*this).StartPacketSendProcessForAnAccessCategory(accessCategoryIndexForLastSentFrame, true);
    }
    else {
        // Send another packet in the TXOP (Transmit opportunity).

        bool packetHasBeenSentToPhy;

        TimeType transmitDelay = ZERO_TIME;
        if (!wasJustTransmitting) {
            transmitDelay = aShortInterframeSpaceTime;
        }//if//

        (*this).TransmitAFrame(
            accessCategoryIndexForLastSentFrame,
            true,
            transmitDelay,
            packetHasBeenSentToPhy);

        assert(packetHasBeenSentToPhy);
    }//if//

}//DoSuccessfulTransmissionPostProcessing//



// Could infer this functionality from Carrier/No Carrier notifications
// but added for clarity.

inline
void Dot11Mac::TransmissionIsCompleteNotification()
{
    if (macState == WaitingForCtsState) {

        (*this).ScheduleWakeupTimer(
            simEngineInterfacePtr->CurrentTime() + clearToSendTimeoutDuration);
    }
    else if (macState == WaitingForAckState) {

        (*this).ScheduleWakeupTimer(simEngineInterfacePtr->CurrentTime() + ackTimeoutDuration);
    }
    else if (macState == CtsOrAckTransmissionState) {
        macState = BusyMediumState;
    }
    else if (macState == ChangingChannelsState) {

        // Assume the medium is busy, but if it is idle the PHY will notify the MAC at channel
        // switch.

        macState = BusyMediumState;

        // Assume last transmission failed (unless it was a broadcast).

        (*this).StartPacketSendProcessForAnAccessCategory(accessCategoryIndexForLastSentFrame);
        (*this).SwitchToChannels(switchingToThisChannelList);
    }
    else {
        assert(macState == BusyMediumState);

        (*this).DoSuccessfulTransmissionPostProcessing(true);

    }//if//

}//TransmissionIsCompleteNotification//



//--------------------------------------------------------------------------------------------------

inline
void Dot11Mac::SetMpduFrameAggregationIsEnabledFor(const MacAddressType& destinationAddress)
{
    neighborCapabilitiesMap[destinationAddress].mpduFrameAggregationIsEnabled = true;
}


inline
bool Dot11Mac::FrameAggregationIsEnabledFor(const MacAddressType& destinationAddress) const
{
    typedef map<MacAddressType, NeighborCapabilitiesType>::const_iterator IterType;

    if (neighborCapabilitiesMap.empty()) {
        return false;
    }//if//

    IterType iter = neighborCapabilitiesMap.find(destinationAddress);

    if (iter == neighborCapabilitiesMap.end()) {
        return false;
    }//if//

    const NeighborCapabilitiesType& capabilities = iter->second;

    return (capabilities.mpduFrameAggregationIsEnabled);

}//FrameAggregationIsEnabledForLink//



inline
bool Dot11Mac::MpduFrameAggregationIsEnabledFor(const MacAddressType& destinationAddress) const
{
    typedef map<MacAddressType, NeighborCapabilitiesType>::const_iterator IterType;

    if (neighborCapabilitiesMap.empty()) {
        return false;
    }//if//

    IterType iter = neighborCapabilitiesMap.find(destinationAddress);

    if (iter == neighborCapabilitiesMap.end()) {
        return false;
    }//if//

    const NeighborCapabilitiesType& capabilities = iter->second;

    return (capabilities.mpduFrameAggregationIsEnabled);

}//MpduFrameAggregationIsEnabledForLink//



inline
bool Dot11Mac::NeedToSendABlockAckRequest(
    const MacAddressType& destinationAddress,
    const PacketPriorityType& trafficId) const
{
    typedef map<AddressAndTrafficIdMapKeyType, OutgoingLinkInfoType>::const_iterator IterType;

    IterType iter =
        outgoingLinkInfoMap.find(AddressAndTrafficIdMapKeyType(destinationAddress, trafficId));

    if (iter == outgoingLinkInfoMap.end()) {
        return false;
    }//if//

    return ((*iter).second.blockAckRequestNeedsToBeSent);

}//NeedToSendBlockAckRequest//



inline
void Dot11Mac::BuildAggregateFrameFromCurrentFrame(const unsigned int accessCategoryIndex)
{
    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();

    assert((allowFrameAggregationWithTxopZero) ||
           (currentTransmitOpportunityAkaTxopEndTime != ZERO_TIME));

    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    const bool useMpduAggregation =
        MpduFrameAggregationIsEnabledFor(accessCategoryInfo.currentPacketsDestinationMacAddress);

    accessCategoryInfo.currentAggregateFrameIsAMpduAggregate = useMpduAggregation;

    assert(accessCategoryInfo.currentAggregateFramePtr == nullptr);
    accessCategoryInfo.currentAggregateFramePtr.reset(new vector<unique_ptr<Packet> >());

    accessCategoryInfo.currentAggregateFramePtr->push_back(move(accessCategoryInfo.currentPacketPtr));

    accessCategoryInfo.currentShortFrameRetryCount = 0;
    accessCategoryInfo.currentLongFrameRetryCount = 0;
    accessCategoryInfo.currentAggregateFrameRetryCount = 0;

    vector<unique_ptr<Packet> >& frameList = *accessCategoryInfo.currentAggregateFramePtr;

    const QosDataFrameHeaderType copyOfOriginalFrameHeader =
        frameList[0]->GetAndReinterpretPayloadData<QosDataFrameHeaderType>();

    AddMpduDelimiterAndPaddingToFrame(*frameList[0]);

    // Assuming No RTS/CTS for aggregated frames ("Greenfield" not supported).

    TransmissionParametersType txParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForDataFrameToStation(
        accessCategoryInfo.currentPacketsDestinationMacAddress,
        txParameters);

    unsigned int totalBytes = frameList[0]->LengthBytes();

    TimeType endTime =
        currentTime +
        CalculateFrameDuration(frameList[0]->LengthBytes(), txParameters);

    endTime += aShortInterframeSpaceTime;
    endTime += CalculateFrameDuration(sizeof(BlockAcknowledgementFrameType), txParameters);

    const PacketPriorityType& priority = accessCategoryInfo.currentPacketPriorityAkaTrafficId;

    const unsigned int maxNumSubframes =
        CalcNumberFramesLeftInSequenceNumWindow(
            accessCategoryInfo.currentPacketsDestinationMacAddress,
            accessCategoryInfo.currentPacketPriorityAkaTrafficId) + 1;

    while ((accessCategoryInfo.currentAggregateFramePtr->size() < maxNumSubframes) &&
           (((allowFrameAggregationWithTxopZero) &&
             (currentTransmitOpportunityAkaTxopEndTime == ZERO_TIME)) ||
            (endTime < currentTransmitOpportunityAkaTxopEndTime)) &&
           (networkOutputQueuePtr->HasPacketWithPriority(priority)) &&
           (networkOutputQueuePtr->NextHopForTopPacket(priority) ==
            accessCategoryInfo.currentPacketsNextHopNetworkAddress)) {

        unsigned int packetLengthBytes;

        packetLengthBytes =
            (networkOutputQueuePtr->TopPacket(priority).LengthBytes() +
            sizeof(QosDataFrameHeaderType) +
            sizeof(MpduDelimiterFrameType));

        if ((totalBytes + packetLengthBytes) > maxAggregateMpduSizeBytes) {
            break;
        }//if//

        // Only first frame of aggregate has Phy header.

        endTime += CalculateFrameDurationWithoutPhyHeader(packetLengthBytes, txParameters);

        if ((!allowFrameAggregationWithTxopZero) &&
            (endTime >= currentTransmitOpportunityAkaTxopEndTime)) {

            break;
        }//if//

        NetworkAddress nextHopAddress;
        EtherTypeFieldType etherType;
        TimeType timestamp;
        unsigned int retryTxCount;

        unique_ptr<Packet> framePtr;

        networkOutputQueuePtr->DequeuePacketWithPriority(
            priority,
            framePtr,
            nextHopAddress,
            etherType,
            timestamp,
            retryTxCount);

        assert(nextHopAddress == accessCategoryInfo.currentPacketsNextHopNetworkAddress);
        assert(retryTxCount == 0);
        assert(etherType == accessCategoryInfo.currentPacketsEtherType);

        unsigned short int newSequenceNumber;
        (*this).GetNewSequenceNumber(
            accessCategoryInfo.currentPacketsDestinationMacAddress,
            priority, false, newSequenceNumber);

        QosDataFrameHeaderType dataFrameHeader = copyOfOriginalFrameHeader;
        dataFrameHeader.sequenceControlField.sequenceNumber = newSequenceNumber;

        framePtr->AddPlainStructHeader(dataFrameHeader);
        AddMpduDelimiterAndPaddingToFrame(*framePtr);

        totalBytes += framePtr->LengthBytes();

        accessCategoryInfo.currentAggregateFramePtr->push_back(move(framePtr));

    }//while//

    if (accessCategoryInfo.currentAggregateFramePtr->size() > 1) {
        // No padding needed on last frame.
        RemoveMpduAggregationPaddingFromFrame(
            *accessCategoryInfo.currentAggregateFramePtr->back());
    }
    else {
        // Aggregation Failure (Back to single frame).

        accessCategoryInfo.currentPacketPtr =
            move(accessCategoryInfo.currentAggregateFramePtr->front());

        accessCategoryInfo.currentAggregateFramePtr.reset();

        RemoveMpduDelimiterAndPaddingFromFrame(*accessCategoryInfo.currentPacketPtr);
    }//if//

}//BuildAggregateFrameFromCurrentFrame//



inline
void Dot11Mac::RetrievePacketFromNetworkLayerForAccessCategory(
    const unsigned int accessCategoryIndex,
    bool& wasRetrieved)
{
    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    assert(accessCategoryInfo.currentPacketPtr == nullptr);

    wasRetrieved = false;

    for(unsigned int i = 0; (i < accessCategoryInfo.listOfPriorities.size()); i++) {
        PacketPriorityType priority = accessCategoryInfo.listOfPriorities[i];

        while ((!wasRetrieved) && (networkOutputQueuePtr->HasPacketWithPriority(priority))) {

            NetworkAddress nextHopAddress;
            EtherTypeFieldType etherType;
            TimeType timestamp;
            unsigned int retryTxCount;
            bool datarateAndTxPowerAreSpecified;
            DatarateBitsPerSecType specifiedPacketDatarateBitsPerSec;
            double specifiedPacketTxPowerDbm;

            networkOutputQueuePtr->DequeuePacketWithEtherTypeAndDatarateAndTxPower(
                priority,
                accessCategoryInfo.currentPacketPtr,
                nextHopAddress,
                timestamp,
                retryTxCount,
                etherType,
                datarateAndTxPowerAreSpecified,
                specifiedPacketDatarateBitsPerSec,
                specifiedPacketTxPowerDbm);

            assert((nextHopAddress.IsTheBroadcastAddress()) ||
                   !(nextHopAddress.IsABroadcastAddress(networkLayerPtr->GetSubnetMask(interfaceIndex))) &&
                   "Make sure no mask dependent broadcast address from network layer");

            const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
            if (accessCategoryInfo.frameLifetime < (currentTime - timestamp)) {
                //lifetime expired
                networkLayerPtr->ReceiveUndeliveredPacketFromMac(
                    interfaceIndex,
                    accessCategoryInfo.currentPacketPtr,
                    NetworkAddress());

                // Loop again to get another packet.
                continue;
            }//if//

            accessCategoryInfo.currentPacketPriorityAkaTrafficId = priority;
            accessCategoryInfo.currentPacketsNextHopNetworkAddress = nextHopAddress;
            accessCategoryInfo.currentPacketsEtherType = etherType;
            accessCategoryInfo.currentPacketsTimestamp = timestamp;

            accessCategoryInfo.currentPacketDatarateAndTxPowerAreSpecified = datarateAndTxPowerAreSpecified;
            accessCategoryInfo.specifiedPacketDatarateBitsPerSec = specifiedPacketDatarateBitsPerSec;
            accessCategoryInfo.specifiedPacketTxPowerDbm = specifiedPacketTxPowerDbm;

            const unsigned int adjustedFrameLengthBytes =
                accessCategoryInfo.currentPacketPtr->LengthBytes() +
                sizeof(QosDataFrameHeaderType);

            if (adjustedFrameLengthBytes < rtsThresholdSizeBytes) {
                accessCategoryInfo.currentShortFrameRetryCount = retryTxCount;
                accessCategoryInfo.currentLongFrameRetryCount = 0;
            }
            else {
                accessCategoryInfo.currentShortFrameRetryCount = 0;
                accessCategoryInfo.currentLongFrameRetryCount = retryTxCount;
            }//if//

            if (redundantTraceInformationModeIsOn) {
                OutputTraceForPacketDequeue(accessCategoryIndex, aRxTxTurnaroundTime);
            } else {
                OutputTraceForPacketDequeue(accessCategoryIndex);
            }//if//

            if ((refreshNextHopOnDequeueModeIsOn) && (!nextHopAddress.IsTheBroadcastAddress())) {
                // Update the next hop to latest.

                bool nextHopWasFound;
                unsigned int nextHopInterfaceIndex;

                networkLayerPtr->GetNextHopAddressAndInterfaceIndexForNetworkPacket(
                    *accessCategoryInfo.currentPacketPtr,
                    nextHopWasFound,
                    accessCategoryInfo.currentPacketsNextHopNetworkAddress,
                    nextHopInterfaceIndex);

               if ((!nextHopWasFound) || (nextHopInterfaceIndex != interfaceIndex)) {
                    // Next hop is no longer valid (dynamic routing).

                    networkLayerPtr->ReceiveUndeliveredPacketFromMac(
                        interfaceIndex,
                        accessCategoryInfo.currentPacketPtr,
                        NetworkAddress());

                    // Loop to get another packet.
                    continue;
               }//if//
            }//if//

            wasRetrieved = false;

            theMacAddressResolverPtr->GetMacAddress(
                nextHopAddress,
                networkLayerPtr->GetSubnetMask(interfaceIndex),
                wasRetrieved,
                accessCategoryInfo.currentPacketsDestinationMacAddress);

            if (!wasRetrieved) {
                // There is not a mac address entry.

                networkLayerPtr->ReceiveUndeliveredPacketFromMac(
                    interfaceIndex,
                    accessCategoryInfo.currentPacketPtr,
                    NetworkAddress());

                // Loop to get another packet.
                continue;

            }//if//

            (*this).GetNewSequenceNumber(
                accessCategoryInfo.currentPacketsDestinationMacAddress,
                accessCategoryInfo.currentPacketPriorityAkaTrafficId,
                true,
                /*out*/accessCategoryInfo.currentPacketSequenceNumber);

            // Check for Power Saving destination.

            if ((GetOperationMode() == ApMode) &&
                (apControllerPtr->StationIsAsleep(
                    accessCategoryInfo.currentPacketsDestinationMacAddress))) {

                apControllerPtr->BufferPacketForSleepingStation(
                    accessCategoryInfo.currentPacketsDestinationMacAddress,
                    accessCategoryInfo.currentPacketPtr,
                    accessCategoryInfo.currentPacketsNextHopNetworkAddress,
                    priority,
                    etherType,
                    timestamp,
                    accessCategoryInfo.currentPacketDatarateAndTxPowerAreSpecified,
                    accessCategoryInfo.specifiedPacketDatarateBitsPerSec,
                    accessCategoryInfo.specifiedPacketTxPowerDbm);

                wasRetrieved = false;
                // Loop to get another packet.
                continue;
            }//if//

            assert(wasRetrieved);

            QosDataFrameHeaderType dataFrameHeader;

            dataFrameHeader.header.frameControlField.frameTypeAndSubtype = QOS_DATA_FRAME_TYPE_CODE;
            dataFrameHeader.header.frameControlField.isRetry = 0;
            dataFrameHeader.header.duration = 0;
            dataFrameHeader.header.receiverAddress = accessCategoryInfo.currentPacketsDestinationMacAddress;
            dataFrameHeader.sequenceControlField.sequenceNumber = accessCategoryInfo.currentPacketSequenceNumber;
            dataFrameHeader.transmitterAddress = myMacAddress;
            dataFrameHeader.qosControlField.trafficId = accessCategoryInfo.currentPacketPriorityAkaTrafficId;
            dataFrameHeader.linkLayerHeader.etherType = HostToNet16(accessCategoryInfo.currentPacketsEtherType);

            accessCategoryInfo.currentPacketPtr->AddPlainStructHeader(dataFrameHeader);

            if (((FrameAggregationIsEnabledFor(
                    accessCategoryInfo.currentPacketsDestinationMacAddress)) &&
                 (!NeedToSendABlockAckRequest(
                    accessCategoryInfo.currentPacketsDestinationMacAddress,
                    accessCategoryInfo.currentPacketPriorityAkaTrafficId))) &&
                ((allowFrameAggregationWithTxopZero) ||
                 (currentTransmitOpportunityAkaTxopEndTime != ZERO_TIME)) &&
                ((!protectAggregateFramesWithSingleAckedFrame) ||
                 (currentTransmitOpportunityAckedFrameCount > 0) ||
                 (dataFrameHeader.header.receiverAddress.IsABroadcastOrAMulticastAddress()))) {

                if (!BlockAckSessionIsEnabled(
                    accessCategoryInfo.currentPacketsDestinationMacAddress,
                    accessCategoryInfo.currentPacketPriorityAkaTrafficId)) {

                    OutgoingLinkInfoType& linkInfo =
                        outgoingLinkInfoMap[
                            AddressAndTrafficIdMapKeyType(
                                accessCategoryInfo.currentPacketsDestinationMacAddress,
                                accessCategoryInfo.currentPacketPriorityAkaTrafficId)];

                    // Using Block Ack Request as ADDBA (Add Block Ack Session Request)

                    linkInfo.blockAckRequestNeedsToBeSent = true;
                    linkInfo.blockAckSessionIsEnabled = true;

                    // This makes the initial Block Ack Request to start the session
                    // Block Ack window on the next data/management frame.

                    linkInfo.lastDroppedFrameSequenceNumber = linkInfo.outgoingSequenceNumber;

                    return;
                }//if//

                (*this).BuildAggregateFrameFromCurrentFrame(accessCategoryIndex);

            }//if//

            return;

        }//while//
    }//for//

    assert(!wasRetrieved);

}//RetrievePacketFromNetworkLayerForAccessCategory//



inline
void Dot11Mac::RequeueBufferedPacket(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriorityType priority,
    const EtherTypeFieldType etherType,
    const TimeType& timestamp,
    const unsigned int txRetryCount,
    const bool datarateAndTxPowerAreaSpecified,
    const DatarateBitsPerSecType& datarateBitsPerSec,
    const double& txPowerDbm)
{
    packetPtr->DeleteHeader(sizeof(QosDataFrameHeaderType));

    networkOutputQueuePtr->RequeueAtFront(
        packetPtr,
        nextHopAddress,
        priority,
        etherType,
        timestamp,
        txRetryCount,
        datarateAndTxPowerAreaSpecified,
        datarateBitsPerSec,
        txPowerDbm);

}//RequeueBufferedPacket//



inline
void Dot11Mac::RequeueManagementFrame(unique_ptr<Packet>& framePtr)
{
    (*this).managementFrameQueue.push_front(move(framePtr));

}//RequeueManagementFrame//



inline
void Dot11Mac::RequeueBufferedPackets()
{
    for(unsigned int i = 0; (i < accessCategories.size()); i++) {
        EdcaAccessCategoryInfo& info = accessCategories[i];

        if (info.currentPacketPtr != nullptr) {
            // Requeue packet and set TX count to 0.
            if (!info.currentPacketIsAManagementFrame) {
                (*this).RequeueBufferedPacket(
                    info.currentPacketPtr,
                    info.currentPacketsNextHopNetworkAddress,
                    info.currentPacketPriorityAkaTrafficId,
                    info.currentPacketsEtherType,
                    info.currentPacketsTimestamp,
                    0,
                    info.currentPacketDatarateAndTxPowerAreSpecified,
                    info.specifiedPacketDatarateBitsPerSec,
                    info.specifiedPacketTxPowerDbm);
            }
            else {
                // Just Delete management frames.

                info.currentPacketPtr.reset();
            }//if//
        }//if//
    }//for//
}//RequeueBufferedDataPackets//



//--------------------------------------------------------------------------------------------------

inline
void Dot11Mac::DoubleTheContentionWindowAndPickANewBackoff(const unsigned int accessCategoryIndex)
{
    using std::min;

    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    // Double contention window up to the maximum size.
    // (Actual sequence is CW(n) = (2^n - 1) ) {1,3,7,15,31,63,...}

    accessCategoryInfo.currentContentionWindowSlots =
        min(((accessCategoryInfo.currentContentionWindowSlots * 2) + 1) ,
            accessCategoryInfo.maxContentionWindowSlots);

    (*this).RecalcRandomBackoff(accessCategoryInfo);

}//DoubleTheContentionWindowAndPickANewBackoff//


//--------------------------------------------------------------------------------------------------

inline
void Dot11Mac::PerformInternalCollisionBackoff(const unsigned int accessCategoryIndex)
{
    (*this).DoubleTheContentionWindowAndPickANewBackoff(accessCategoryIndex);

}//PerformInternalCollisionBackoff//



//-------------------------------------------------------------------------------------------------

inline
DurationFieldType ConvertTimeToDurationFieldValue(const TimeType& duration)
{
    using std::min;

    // Round up to nearest us and limit to maximum value.
    return (
        min(MaxDurationFieldValue, DurationFieldType((duration + (MICRO_SECOND-1)) / MICRO_SECOND)));
}

inline
TimeType ConvertDurationFieldValueToTime(const DurationFieldType& durationField)
{
    return (durationField * MICRO_SECOND);
}


inline
TimeType Dot11Mac::CalculateFrameDuration(
    const unsigned int frameWithMacHeaderSizeBytes,
    const TransmissionParametersType& txParameters) const
{
    return (
        physicalLayerPtr->CalculateFrameTransmitDuration(frameWithMacHeaderSizeBytes, txParameters));
}


inline
TimeType Dot11Mac::CalculateFrameDurationWithoutPhyHeader(
    const unsigned int frameWithMacHeaderSizeBytes,
    const TransmissionParametersType& txParameters) const
{
    return (
        physicalLayerPtr->CalculateFrameDataDuration(frameWithMacHeaderSizeBytes, txParameters));
}



inline
TimeType Dot11Mac::CalculateNavDurationForRtsFrame(
    const MacAddressType& destinationMacAddress,
    const unsigned int packetWithMacHeaderSizeBytes,
    const TransmissionParametersType& dataFrameTxParameters) const
{
    // Assume same adaptive modulation subsystems.  Alternative would be conservative
    // maximums.

    TransmissionParametersType ctsTxParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForManagementFrameToStation(
        destinationMacAddress,
        ctsTxParameters);

    const TimeType ctsFrameDuration =
        CalculateFrameDuration(sizeof(ClearToSendFrameType), ctsTxParameters);

    const TimeType dataFrameDuration =
        CalculateFrameDuration(packetWithMacHeaderSizeBytes, dataFrameTxParameters);

    TransmissionParametersType ackTxParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForAckFrame(
        destinationMacAddress,
        dataFrameTxParameters,
        ackTxParameters);

    const TimeType ackFrameDuration =
        CalculateFrameDuration(sizeof(AcknowledgementAkaAckFrameType), ackTxParameters);

    return
        (aShortInterframeSpaceTime +
         ctsFrameDuration +
         aShortInterframeSpaceTime +
         dataFrameDuration +
         aShortInterframeSpaceTime +
         ackFrameDuration);

}//CalculateNavDurationForRtsFrame//


inline
TimeType Dot11Mac::CalculateNavDurationForCtsFrame(
    const DurationFieldType& durationFromRtsFrame,
    const MacAddressType& destinationMacAddress) const
{
    TimeType timeTypedDurationFromRtsFrame = TimeType(MICRO_SECOND * durationFromRtsFrame);

    const unsigned int clearToSendFrameSizeBytes = sizeof(ClearToSendFrameType);

    TransmissionParametersType txParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForManagementFrameToStation(
        destinationMacAddress,
        txParameters);

    const TimeType clearToSendFrameDuration =
        CalculateFrameDuration(clearToSendFrameSizeBytes, txParameters);

    return (timeTypedDurationFromRtsFrame - (aShortInterframeSpaceTime + clearToSendFrameDuration));

}//CalculateNavDurationForCtsFrame//


inline
TimeType Dot11Mac::CalculateNavDurationForAckedDataFrame(
    const TransmissionParametersType& txParameters) const
{
    const unsigned int ackFrameSizeBytes = sizeof(AcknowledgementAkaAckFrameType);

    const TimeType ackFrameDuration = CalculateFrameDuration(ackFrameSizeBytes, txParameters);

    // NAV Duration is the ACK (and a SIFs)

    return (aShortInterframeSpaceTime + ackFrameDuration);

}//CalculateNavDurationForAckedDataFrame//


inline
TimeType Dot11Mac::CalculateNavDurationForAckedAggregateDataFrame(
    const TransmissionParametersType& txParameters) const
{
    const unsigned int blockAckFrameSizeBytes = sizeof(BlockAcknowledgementFrameType);

    const TimeType blockAckFrameDuration = CalculateFrameDuration(blockAckFrameSizeBytes, txParameters);

    // NAV Duration is the ACK (and a SIFs)

    return (aShortInterframeSpaceTime + blockAckFrameDuration);

}//CalculateNavDurationForAckedAggregateDataFrame//


inline
TimeType Dot11Mac::CalculateNextFrameSequenceDuration(const unsigned int accessCategoryIndex) const
{
    const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

    unsigned int frameSizeBytes = 0;
    MacAddressType destinationMacAddress;
    TransmissionParametersType txParameters;

    if (accessCategoryInfo.currentAggregateFramePtr != nullptr) {
        // This situation should only occur when retrying an aggregate frame.
        // Warning:  This calculation is only approximate (low) for this case.
        // (Jay): Check (NAV) durations for block acked sequences.

        destinationMacAddress = accessCategoryInfo.currentPacketsDestinationMacAddress;

        theAdaptiveRateControllerPtr->GetDataRateInfoForManagementFrameToStation(
            destinationMacAddress, txParameters);

        for(unsigned int i = 0; (i < accessCategoryInfo.currentAggregateFramePtr->size()); i++) {
            frameSizeBytes += (*accessCategoryInfo.currentAggregateFramePtr)[i]->LengthBytes();
        }//for//
    }
    else if ((accessCategoryIndex == accessCategoryIndexForManagementFrames) && (!managementFrameQueue.empty())) {

        const Packet& aPacket = *managementFrameQueue.front();
        frameSizeBytes = aPacket.LengthBytes();

        const CommonFrameHeaderType& frameHeader =
            aPacket.GetAndReinterpretPayloadData<CommonFrameHeaderType>();

        destinationMacAddress = frameHeader.receiverAddress;

        theAdaptiveRateControllerPtr->GetDataRateInfoForManagementFrameToStation(
            destinationMacAddress, txParameters);
    }
    else {
        for(unsigned int i = 0; (i < accessCategoryInfo.listOfPriorities.size()); i++) {
            PacketPriorityType priority = accessCategoryInfo.listOfPriorities[i];

            if (networkOutputQueuePtr->HasPacketWithPriority(priority)) {
                frameSizeBytes =
                    (networkOutputQueuePtr->TopPacket(priority).LengthBytes() +
                     sizeof(QosDataFrameHeaderType));

                bool macAddressWasResolved;

                theMacAddressResolverPtr->GetMacAddress(
                    networkOutputQueuePtr->NextHopForTopPacket(priority),
                    networkLayerPtr->GetSubnetMask(interfaceIndex),
                    macAddressWasResolved,
                    destinationMacAddress);

                if (!macAddressWasResolved) {
                    // Next packet is not deliverable so give up.
                    frameSizeBytes = 0;
                }//if//

                theAdaptiveRateControllerPtr->GetDataRateInfoForDataFrameToStation(
                    destinationMacAddress, txParameters);

                break;
            }//if//
        }//for//
    }//if//

    if (frameSizeBytes == 0) {
        return (ZERO_TIME);
    }//if//

    TimeType extraTimeBeforeFrame = ZERO_TIME;
    if (!accessCategoryInfo.currentPacketsDestinationMacAddress.IsABroadcastOrAMulticastAddress()) {
        extraTimeBeforeFrame = aShortInterframeSpaceTime;
    }//if//

    const TimeType frameDuration = CalculateFrameDuration(frameSizeBytes, txParameters);

    if (destinationMacAddress.IsABroadcastOrAMulticastAddress()) {
        return (extraTimeBeforeFrame + frameDuration);
    }
    else {
        return
            (extraTimeBeforeFrame +
             frameDuration +
             CalculateNavDurationForAckedDataFrame(txParameters));
    }//if//

}//CalculateNextFrameSequenceDuration//



inline
void Dot11Mac::AddExtraNavDurationToPacketForNextFrameIfInATxop(
    const unsigned int accessCategoryIndex,
    const TimeType& frameTransmissionEndTime,
    Packet& aFrame) const
{
    assert(currentTransmitOpportunityAkaTxopEndTime != ZERO_TIME);

    TimeType nextFrameSequenceDuration = CalculateNextFrameSequenceDuration(accessCategoryIndex);
    if (nextFrameSequenceDuration == ZERO_TIME) {
        // Nothing To Send next.
        return;
    }//if//

    CommonFrameHeaderType& header = aFrame.GetAndReinterpretPayloadData<CommonFrameHeaderType>();

    const TimeType currentFrameSequenceEndTime =
        frameTransmissionEndTime + ConvertDurationFieldValueToTime(header.duration);

    if ((currentFrameSequenceEndTime + nextFrameSequenceDuration) > currentTransmitOpportunityAkaTxopEndTime) {
        // Not enough time left for next frame sequence.
        return;
    }//if//

    header.duration += ConvertTimeToDurationFieldValue(nextFrameSequenceDuration);

}//AddExtraNavDurationToPacketForNextFrameIfInATxop//



//-------------------------------------------------------------------------------------------------


inline
void Dot11Mac::CalculateAndSetTransmitOpportunityDurationAkaTxop(const size_t accessCategoryIndex)
{
    EdcaAccessCategoryInfo& accessCategoryInfo = (*this).accessCategories.at(accessCategoryIndex);

    (*this).currentTransmitOpportunityAckedFrameCount = 0;

    accessCategoryInfo.transmitOpportunityDurationAkaTxop = ZERO_TIME;

    if (macSchedulerPtr.get() != nullptr) {
        accessCategoryInfo.transmitOpportunityDurationAkaTxop =
            macSchedulerPtr->CalculateTxopDuration(accessCategoryIndex);

    }//if//

    if (accessCategoryInfo.transmitOpportunityDurationAkaTxop == ZERO_TIME) {
        (*this).currentTransmitOpportunityAkaTxopEndTime = ZERO_TIME;
    }
    else {
        const TimeType currentTime = simEngineInterfacePtr->CurrentTime();

        (*this).currentTransmitOpportunityAkaTxopStartTime = currentTime;
        (*this).currentTransmitOpportunityAkaTxopEndTime =
               std::min(transmissionPermissionEndTime,
                        currentTime + accessCategoryInfo.transmitOpportunityDurationAkaTxop);
    }//if//

}//CalculateAndSetTransmitOpportunityDurationAkaTxop//

//-------------------------------------------------------------------------------------------------


inline
void Dot11Mac::TransmitAnAggregateFrame(
    const unsigned int accessCategoryIndex,
    const TransmissionParametersType& txParameters,
    const double& transmitPowerDbm,
    const TimeType& delayUntilTransmitting)
{
    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();

    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);
    assert(accessCategoryInfo.currentAggregateFramePtr != nullptr);

    const DurationFieldType navDurationField =
        ConvertTimeToDurationFieldValue(CalculateNavDurationForAckedAggregateDataFrame(txParameters));


    if (accessCategoryInfo.currentAggregateFrameIsAMpduAggregate) {
        for(unsigned int i = 0; (i < accessCategoryInfo.currentAggregateFramePtr->size()); i++) {
            Packet& aPacket = (*(*accessCategoryInfo.currentAggregateFramePtr)[i]);

            CommonFrameHeaderType& header =
                aPacket.GetAndReinterpretPayloadData<CommonFrameHeaderType>(
                    sizeof(MpduDelimiterFrameType));

            header.duration = navDurationField;

        }//for//
    }
    else {
        CommonFrameHeaderType& header =
            accessCategoryInfo.currentAggregateFramePtr->front()->
                GetAndReinterpretPayloadData<CommonFrameHeaderType>();

        header.duration = navDurationField;

    }//if//

    (*this).accessCategoryIndexForLastSentFrame = accessCategoryIndex;
    (*this).macState = WaitingForAckState;
    (*this).lastSentFrameWasAn = AggregateFrame;

    OutputTraceAndStatsForAggregatedFrameTransmission(accessCategoryIndex);

    physicalLayerPtr->TransmitAggregateFrame(
        accessCategoryInfo.currentAggregateFramePtr,
        accessCategoryInfo.currentAggregateFrameIsAMpduAggregate,
        txParameters,
        transmitPowerDbm,
        delayUntilTransmitting);

}//TransmitAnAggregateFrame//



inline
void Dot11Mac::TransmitABlockAckRequest(
    const MacAddressType& destinationAddress,
    const PacketPriorityType& trafficId,
    const TimeType& delayUntilTransmitting)
{
    const OutgoingLinkInfoType& linkInfo =
        outgoingLinkInfoMap[AddressAndTrafficIdMapKeyType(destinationAddress, trafficId)];

    unique_ptr<Packet> frameToSendPtr =
        CreateABlockAckRequestFrame(
            destinationAddress,
            trafficId,
            (linkInfo.lastDroppedFrameSequenceNumber+1));

    const double transmitPowerDbm =
        theAdaptiveTxPowerControllerPtr->CurrentTransmitPowerDbm(destinationAddress);

    TransmissionParametersType txParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForManagementFrameToStation(
        destinationAddress, txParameters);

    CommonFrameHeaderType& header =
        frameToSendPtr->GetAndReinterpretPayloadData<CommonFrameHeaderType>();

    header.duration =
        ConvertTimeToDurationFieldValue(
            CalculateNavDurationForAckedAggregateDataFrame(txParameters));

    (*this).macState = WaitingForAckState;
    (*this).lastSentFrameWasAn = BlockAckRequestFrame;

    physicalLayerPtr->TransmitFrame(
        frameToSendPtr,
        txParameters,
        transmitPowerDbm,
        delayUntilTransmitting);

}//TransmitABlockAckRequest//


inline
void Dot11Mac::TransmitAFrame(
    const unsigned int accessCategoryIndex,
    const bool doNotRequestToSendAkaRts,
    const TimeType& delayUntilTransmitting,
    bool& packetHasBeenSentToPhy)
{
    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();

    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

    if ((accessCategoryInfo.currentPacketPtr == nullptr) &&
        (accessCategoryInfo.currentAggregateFramePtr == nullptr)) {

        if (accessCategoryIndex == accessCategoryIndexForManagementFrames) {

            while (!managementFrameQueue.empty()) {

                const CommonFrameHeaderType& frameHeader =
                    managementFrameQueue.front()->GetAndReinterpretPayloadData<CommonFrameHeaderType>();

                if ((GetOperationMode() != ApMode) ||
                    (!apControllerPtr->StationIsAsleep(frameHeader.receiverAddress))) {

                    accessCategoryInfo.currentPacketPtr = move(managementFrameQueue.front());
                    managementFrameQueue.pop_front();
                    accessCategoryInfo.currentPacketIsAManagementFrame = true;
                    accessCategoryInfo.currentPacketsDestinationMacAddress = frameHeader.receiverAddress;
                    accessCategoryInfo.currentShortFrameRetryCount = 0;
                    accessCategoryInfo.currentLongFrameRetryCount = 0;

                    break;

                }//if//

                unique_ptr<Packet> aManagementFramePtr = move(managementFrameQueue.front());

                apControllerPtr->BufferManagementFrameForSleepingStation(
                    frameHeader.receiverAddress, aManagementFramePtr, currentTime);

                managementFrameQueue.pop_front();

                assert(accessCategoryInfo.currentPacketPtr == nullptr);

            }//while//

        }//if//

        if (accessCategoryInfo.currentPacketPtr == nullptr) {

            accessCategoryInfo.currentPacketIsAManagementFrame = false;

            bool wasRetrieved;

            (*this).RetrievePacketFromNetworkLayerForAccessCategory(accessCategoryIndex, wasRetrieved);

            if (!wasRetrieved) {
                packetHasBeenSentToPhy = false;
                accessCategoryInfo.hasPacketToSend = false;
                assert(accessCategoryInfo.currentPacketPtr == nullptr);
                return;
            }//if//
        }//if//
    }//if//


    TransmissionParametersType txParameters;

    if (accessCategoryInfo.currentPacketIsAManagementFrame) {

        theAdaptiveRateControllerPtr->GetDataRateInfoForManagementFrameToStation(
            accessCategoryInfo.currentPacketsDestinationMacAddress, txParameters);

        assert(txParameters.channelBandwidthMhz == physicalLayerPtr->GetBaseChannelBandwidthMhz());
    }
    else {
        if (accessCategoryInfo.currentPacketDatarateAndTxPowerAreSpecified) {
            if (theAdaptiveTxPowerControllerPtr->TxPowerIsSpecifiedByPhyLayer()) {
                cerr << "Error: Set dot11-tx-power-specified-by = UpperLayer to specify datarate and tx-power by Upper Layer" << endl;
                exit(1);
            }

            theAdaptiveRateControllerPtr->GetDataRateInfoForDatarateSpecifiedFrame(
                accessCategoryInfo.specifiedPacketDatarateBitsPerSec, txParameters);
        }
        else {
            theAdaptiveRateControllerPtr->GetDataRateInfoForDataFrameToStation(
                accessCategoryInfo.currentPacketsDestinationMacAddress, txParameters);
        }//if//

    }//if//

    txParameters.firstChannelNumber =
        GetFirstChannelNumberForChannelBandwidth(txParameters.channelBandwidthMhz);

    double transmitPowerDbm;

    if (accessCategoryInfo.currentPacketDatarateAndTxPowerAreSpecified) {
        assert(!theAdaptiveTxPowerControllerPtr->TxPowerIsSpecifiedByPhyLayer());

        transmitPowerDbm = accessCategoryInfo.specifiedPacketTxPowerDbm;
    }
    else {

        transmitPowerDbm =
            theAdaptiveTxPowerControllerPtr->CurrentTransmitPowerDbm(
                accessCategoryInfo.currentPacketsDestinationMacAddress);
    }//if//

    if (accessCategoryInfo.currentPacketPtr == nullptr) {
        if ((protectAggregateFramesWithSingleAckedFrame) &&
            (accessCategoryInfo.currentAggregateFrameIsAMpduAggregate) &&
            (currentTransmitOpportunityAckedFrameCount == 0)) {

            // Only retry first frame of MPDU aggregate frame for first acked TX of a TXOP.

            accessCategoryInfo.currentPacketPtr =
                move(accessCategoryInfo.currentAggregateFramePtr->front());

            accessCategoryInfo.currentAggregateFramePtr->erase(
                accessCategoryInfo.currentAggregateFramePtr->begin());

            if (accessCategoryInfo.currentAggregateFramePtr->empty()) {
                accessCategoryInfo.currentAggregateFramePtr.reset();
            }//if//

            if (accessCategoryInfo.currentAggregateFrameIsAMpduAggregate) {
                RemoveMpduDelimiterAndPaddingFromFrame(*accessCategoryInfo.currentPacketPtr);
            }
            else {
                const QosDataFrameHeaderType copyOfHeader =
                    accessCategoryInfo.currentPacketPtr->GetAndReinterpretPayloadData<QosDataFrameHeaderType>();

                accessCategoryInfo.currentPacketPtr->DeleteHeader(sizeof(QosDataFrameHeaderType));
                accessCategoryInfo.currentPacketPtr->AddPlainStructHeader(copyOfHeader);

                // Move header to next frame and get new sequence number.

                if (accessCategoryInfo.currentAggregateFramePtr != nullptr) {
                    accessCategoryInfo.currentAggregateFramePtr->front()->AddPlainStructHeader(copyOfHeader);

                    QosDataFrameHeaderType& dataFrameHeader =
                        accessCategoryInfo.currentPacketPtr->GetAndReinterpretPayloadData<QosDataFrameHeaderType>();

                    unsigned short int sequenceNumber;

                    (*this).GetNewSequenceNumber(
                        dataFrameHeader.header.receiverAddress,
                        dataFrameHeader.qosControlField.trafficId,
                        true,
                        /*out*/sequenceNumber);

                    dataFrameHeader.sequenceControlField.sequenceNumber = sequenceNumber;
                }//if//
            }//if//

            QosDataFrameHeaderType& header =
                accessCategoryInfo.currentPacketPtr->GetAndReinterpretPayloadData<QosDataFrameHeaderType>();

            assert(header.header.frameControlField.frameTypeAndSubtype == QOS_DATA_FRAME_TYPE_CODE);

            accessCategoryInfo.currentPacketIsAManagementFrame = false;
            accessCategoryInfo.currentPacketSequenceNumber =
                header.sequenceControlField.sequenceNumber;

            if (accessCategoryInfo.currentPacketPtr->LengthBytes() < rtsThresholdSizeBytes) {
                accessCategoryInfo.currentShortFrameRetryCount =
                    accessCategoryInfo.currentAggregateFrameRetryCount;
                accessCategoryInfo.currentLongFrameRetryCount = 0;
            }
            else {
                accessCategoryInfo.currentShortFrameRetryCount = 0;
                accessCategoryInfo.currentLongFrameRetryCount =
                    accessCategoryInfo.currentAggregateFrameRetryCount;
            }//if//
        }
        else {

            (*this).lastSentDataFrameDestinationMacAddress =
                accessCategoryInfo.currentPacketsDestinationMacAddress;

            (*this).TransmitAnAggregateFrame(
                accessCategoryIndex,
                txParameters,
                transmitPowerDbm,
                delayUntilTransmitting);

            packetHasBeenSentToPhy = true;
            return;

        }//if//
    }//if//

    if (NeedToSendABlockAckRequest(
        accessCategoryInfo.currentPacketsDestinationMacAddress,
        accessCategoryInfo.currentPacketPriorityAkaTrafficId)) {

        (*this).accessCategoryIndexForLastSentFrame = accessCategoryIndex;
        (*this).lastSentDataFrameDestinationMacAddress =
            accessCategoryInfo.currentPacketsDestinationMacAddress;

        (*this).TransmitABlockAckRequest(
            accessCategoryInfo.currentPacketsDestinationMacAddress,
            accessCategoryInfo.currentPacketPriorityAkaTrafficId,
            delayUntilTransmitting);

        packetHasBeenSentToPhy = true;

        return;
    }//if//

    const unsigned int frameSizeBytes = accessCategoryInfo.currentPacketPtr->LengthBytes();

    unsigned int txFrameSizeBytes;

    if ((accessCategoryInfo.currentPacketsDestinationMacAddress.IsABroadcastOrAMulticastAddress()) ||
        (doNotRequestToSendAkaRts) ||
        (frameSizeBytes < rtsThresholdSizeBytes)) {

        if (accessCategoryInfo.currentPacketIsAManagementFrame) {
            txFrameSizeBytes = frameSizeBytes;
        }
        else {
            txFrameSizeBytes = frameSizeBytes + sizeof(QosDataFrameHeaderType);
        }//if//
    }
    else {
         txFrameSizeBytes = sizeof(RequestToSendFrameType);
    }//if//

    if ((*this).FrameTransmissionDurationExceedsTransmissionPermissionEndTime(
            txFrameSizeBytes,
            txParameters,
            delayUntilTransmitting)) {
        packetHasBeenSentToPhy = false;
        accessCategoryInfo.hasPacketToSend = false;
        return;
    }//if//


    if (frameSizeBytes < rtsThresholdSizeBytes) {
        (*this).lastSentFrameWasAn = ShortFrame;
    }
    else {
        (*this).lastSentFrameWasAn = LongFrame;
    }//if//

    if (accessCategoryInfo.currentPacketsDestinationMacAddress.IsABroadcastOrAMulticastAddress()) {

        if (accessCategoryInfo.currentPacketIsAManagementFrame) {
            OutputTraceAndStatsForManagementFrameTransmission(accessCategoryIndex);
        }
        else {
            OutputTraceAndStatsForBroadcastDataFrameTransmission(accessCategoryIndex);

        }//if//

        // A transmission that does not require an immediate frame as a response is defined as a successful transmission.
        // Next Packet => Reset Contention window.

        accessCategoryInfo.currentContentionWindowSlots = accessCategoryInfo.minContentionWindowSlots;

        macState = BusyMediumState;
    }
    else if ((doNotRequestToSendAkaRts) || (frameSizeBytes < rtsThresholdSizeBytes)) {

        bool isARetry;
        if (frameSizeBytes < rtsThresholdSizeBytes) {
            isARetry = (accessCategoryInfo.currentShortFrameRetryCount > 0);
        }
        else {
            isARetry = (accessCategoryInfo.currentLongFrameRetryCount > 0);
        }//if//

        const TimeType navDuration = CalculateNavDurationForAckedDataFrame(txParameters);

        CommonFrameHeaderType& frameHeader =
            accessCategoryInfo.currentPacketPtr->GetAndReinterpretPayloadData<CommonFrameHeaderType>();

        frameHeader.frameControlField.isRetry = isARetry;
        frameHeader.duration = ConvertTimeToDurationFieldValue(navDuration);

        if (accessCategoryInfo.currentPacketIsAManagementFrame) {
            OutputTraceAndStatsForManagementFrameTransmission(accessCategoryIndex);
        }
        else {
            OutputTraceAndStatsForUnicastDataFrameTransmission(accessCategoryIndex);
        }//if//

        (*this).lastSentDataFrameDestinationMacAddress = frameHeader.receiverAddress;
        (*this).macState = WaitingForAckState;
    }
    else {
        // RTS Special Case.

        (*this).lastSentFrameWasAn = RequestToSendFrame;

        RequestToSendFrameType rtsFrame;

        rtsFrame.header.frameControlField.frameTypeAndSubtype = RTS_FRAME_TYPE_CODE;
        rtsFrame.header.frameControlField.isRetry = (accessCategoryInfo.currentShortFrameRetryCount > 0);

        rtsFrame.header.duration =
            ConvertTimeToDurationFieldValue(
                CalculateNavDurationForRtsFrame(
                    rtsFrame.header.receiverAddress,
                    accessCategoryInfo.currentPacketPtr->LengthBytes(),
                    txParameters));

        rtsFrame.header.receiverAddress = accessCategoryInfo.currentPacketsDestinationMacAddress;
        rtsFrame.transmitterAddress = myMacAddress;

        unique_ptr<Packet> rtsPacketToSendPtr =
            Packet::CreatePacket(*simEngineInterfacePtr, rtsFrame);

        (*this).accessCategoryIndexForLastSentFrame = accessCategoryIndex;

        TransmissionParametersType rtsTxParameters;

        theAdaptiveRateControllerPtr->GetDataRateInfoForManagementFrameToStation(
            accessCategoryInfo.currentPacketsDestinationMacAddress, rtsTxParameters);

        if (currentTransmitOpportunityAkaTxopEndTime != ZERO_TIME) {
            TimeType frameTransmissionEndTime =
                (currentTime + delayUntilTransmitting +
                 CalculateFrameDuration(rtsPacketToSendPtr->LengthBytes(), rtsTxParameters));

            AddExtraNavDurationToPacketForNextFrameIfInATxop(
                accessCategoryIndex, frameTransmissionEndTime, *rtsPacketToSendPtr);
        }//if//

        physicalLayerPtr->TransmitFrame(
            rtsPacketToSendPtr,
            rtsTxParameters,
            transmitPowerDbm,
            delayUntilTransmitting);

        macState = WaitingForCtsState;

        OutputTraceAndStatsForRtsFrameTransmission(accessCategoryIndex);

        packetHasBeenSentToPhy = true;

        return;

    }//if//

    // Going to move frame down to Phy, but will retrieve it later if necessary (retry).

    unique_ptr<Packet> packetToSendPtr = move(accessCategoryInfo.currentPacketPtr);

    if (currentTransmitOpportunityAkaTxopEndTime != ZERO_TIME) {
        TimeType frameTransmissionEndTime =
            (currentTime + delayUntilTransmitting +
             CalculateFrameDuration(packetToSendPtr->LengthBytes(), txParameters));

        AddExtraNavDurationToPacketForNextFrameIfInATxop(
            accessCategoryIndex, frameTransmissionEndTime, *packetToSendPtr);
    }//if//

    (*this).accessCategoryIndexForLastSentFrame = accessCategoryIndex;

    physicalLayerPtr->TransmitFrame(
        packetToSendPtr,
        txParameters,
        transmitPowerDbm,
        delayUntilTransmitting);

    packetHasBeenSentToPhy = true;

}//TransmitAFrame//


inline
bool Dot11Mac::FrameTransmissionDurationExceedsTransmissionPermissionEndTime(
    const unsigned int& frameSizeBytes,
    const TransmissionParametersType& txParameters,
    const TimeType& delayUntilTransmitting) const
{
    if (transmissionPermissionEndTime == INFINITE_TIME) {
        return false;
    }//if//

    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();

    if (frameSizeBytes == 0) {
        return false;
    }//if//

    const TimeType transmissionDuration =
        (*this).CalculateFrameDuration(frameSizeBytes, txParameters);

    return (currentTime + delayUntilTransmitting + transmissionDuration) >= transmissionPermissionEndTime;
}

inline
void Dot11Mac::ProcessInterframeSpaceAndBackoffTimeout()
{
    // Try to send a frame

    OutputTraceForIfsAndBackoffExpiration();

    assert(macState == WaitingForIfsAndBackoffState);

    TimeType currentTime = simEngineInterfacePtr->CurrentTime();

    // Go through Access Categories in reverse order (high priority to low priority).

    bool packetHasBeenSentToThePhy = false;
    for(int i = (numberAccessCategories - 1); (i >= 0); i--) {

        const unsigned int accessCategoryIndex = static_cast<unsigned int>(i);

        EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

        if (accessCategoryInfo.currentNonExtendedBackoffDuration != INFINITE_TIME) {
            TimeType elapsedTime = currentTime - accessCategoryInfo.ifsAndBackoffStartTime;

            accessCategoryInfo.currentNumOfBackoffSlots -=
               NumberBackoffSlotsUsedDuringElapsedTime(accessCategoryInfo, elapsedTime);

            assert(accessCategoryInfo.currentNumOfBackoffSlots >= 0);

            if ((accessCategoryInfo.currentNumOfBackoffSlots == 0) &&
                (!packetHasBeenSentToThePhy)) {

                accessCategoryInfo.currentNonExtendedBackoffDuration = INFINITE_TIME;
                accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff = false;

                //If a packet should be sent exists, change hasPacketToSend.
                if (accessCategoryInfo.currentPacketPtr == nullptr) {
                    accessCategoryInfo.hasPacketToSend =
                        ThereAreQueuedPacketsForAccessCategory(accessCategoryIndex);
                }//if//

                if (accessCategoryInfo.hasPacketToSend) {
                   (*this).CalculateAndSetTransmitOpportunityDurationAkaTxop(accessCategoryIndex);

                   (*this).TransmitAFrame(
                       accessCategoryIndex, false, aRxTxTurnaroundTime, packetHasBeenSentToThePhy);
                }
                else {
                    // Post-transmission forced backoff only, no packets to send.
                }//if//
            }
            else {
                if (accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff) {
                    // Failed to jump on medium because another Access Category got there first.
                    // Must start normal backoff process.

                    accessCategoryInfo.tryingToJumpOnMediumWithoutABackoff = false;
                    accessCategoryInfo.currentNonExtendedBackoffDuration = INFINITE_TIME;

                    (*this).StartPacketSendProcessForAnAccessCategory(accessCategoryIndex, true);
                }
                else if (accessCategoryInfo.currentNumOfBackoffSlots == 0) {
                    assert(packetHasBeenSentToThePhy);
                    // Another higher priority Access Category is transmitting.

                    (*this).PerformInternalCollisionBackoff(accessCategoryIndex);
                }
                else {

                    accessCategoryInfo.currentNonExtendedBackoffDuration =
                        CalculateNonExtendedBackoffDuration(accessCategoryInfo);
                }//if//

            }//if//
        }//if//
    }//for//

    if (!packetHasBeenSentToThePhy) {

        // Timeout was for post-transmission forced backoff (no actual packets to send).
        macState = TransientState;
        (*this).StartBackoffIfNecessary();

    }//if//

}//ProcessInterframeSpaceAndBackoffTimeout//


inline
void Dot11Mac::DropCurrentPacketAndGoToNextPacket(const unsigned int accessCategoryIndex)
{
    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    const MacAddressType& destinationMacAddress =
        accessCategoryInfo.currentPacketsDestinationMacAddress;

    OutputTraceAndStatsForPacketRetriesExceeded(accessCategoryIndex);

    OutgoingLinkInfoType& linkInfo =
        outgoingLinkInfoMap[
            AddressAndTrafficIdMapKeyType(
                destinationMacAddress,
                accessCategoryInfo.currentPacketPriorityAkaTrafficId)];

    linkInfo.lastDroppedFrameSequenceNumber = accessCategoryInfo.currentPacketSequenceNumber;

    if (linkInfo.blockAckSessionIsEnabled) {

        linkInfo.blockAckRequestNeedsToBeSent = true;

    }//if//

    if (!accessCategoryInfo.currentPacketIsAManagementFrame) {

        networkLayerPtr->ReceiveUndeliveredPacketFromMac(
            interfaceIndex,
            accessCategoryInfo.currentPacketPtr,
            accessCategoryInfo.currentPacketsNextHopNetworkAddress);
    }
    else {
        accessCategoryInfo.currentPacketPtr.reset();
    }//if//

    assert(accessCategoryInfo.currentPacketPtr == nullptr);

    accessCategoryInfo.hasPacketToSend = ThereAreQueuedPacketsForAccessCategory(accessCategoryIndex);

    accessCategoryInfo.currentShortFrameRetryCount = 0;
    accessCategoryInfo.currentLongFrameRetryCount = 0;

    // Next Packet => Reset Contention window.
    // Always do backoff procedure regardless of whether there is a packet to send.

    accessCategoryInfo.currentContentionWindowSlots = accessCategoryInfo.minContentionWindowSlots;

    (*this).RecalcRandomBackoff(accessCategoryInfo);

}//DropCurrentPacketAndGoToNextPacket//



inline
void Dot11Mac::DropCurrentAggregateFrameAndGoToNextPacket(const unsigned int accessCategoryIndex)
{
    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndex];

    assert(accessCategoryInfo.currentAggregateFramePtr != nullptr);

    accessCategoryInfo.currentAggregateFrameRetryCount = 0;

    // Assuming sequence numbers are in order.

    const Packet& lastSubframe = *accessCategoryInfo.currentAggregateFramePtr->back();

    const QosDataFrameHeaderType& header =
        lastSubframe.GetAndReinterpretPayloadData<QosDataFrameHeaderType>(
            sizeof(MpduDelimiterFrameType));

    const unsigned short int droppedFrameSequenceNumber = header.sequenceControlField.sequenceNumber;

    vector<unique_ptr<Packet> >& subframes = *accessCategoryInfo.currentAggregateFramePtr;

    // For loop is useless for non-dummy unique_ptr.
    for(unsigned int i = 0; (i <subframes.size()); i++) {
        subframes[i].reset();
    }//for//

    accessCategoryInfo.currentAggregateFramePtr.reset();

    // Send Block Ack to shift sequence number
    // window at destination for dropped frame.

    OutgoingLinkInfoType& linkInfo =
        outgoingLinkInfoMap[
            AddressAndTrafficIdMapKeyType(
                accessCategoryInfo.currentPacketsDestinationMacAddress,
                accessCategoryInfo.currentPacketPriorityAkaTrafficId)];

    assert(linkInfo.blockAckSessionIsEnabled);
    linkInfo.blockAckRequestNeedsToBeSent = true;
    linkInfo.lastDroppedFrameSequenceNumber = droppedFrameSequenceNumber;

    //Future Idea // Obsolete G++ hack (combine statements after upgrade).
    //Future Idea
    //Future Idea unique_ptr<Packet> framePtr =
    //Future Idea     CreateABlockAckRequestFrame(
    //Future Idea         accessCategoryInfo.currentPacketsDestinationMacAddress,
    //Future Idea         accessCategoryInfo.currentPacketPriorityAkaTrafficId,
    //Future Idea         (droppedFrameSequenceNumber + 1));
    //Future Idea
    //Future Idea (*this).SendManagementFrame(framePtr);

}//DropCurrentAggregateFrameAndGoToNextPacket//




inline
void Dot11Mac::NeverReceivedClearToSendOrAcknowledgementAction()
{
    // Make sure that no longer in waiting for ACK or CTS state.

    assert((macState != WaitingForCtsState) && (macState != WaitingForAckState));

    if (WakeupTimerIsActive()) {
        (*this).CancelWakeupTimer();
    }//if//

    EdcaAccessCategoryInfo& accessCategoryInfo = (*this).accessCategories[accessCategoryIndexForLastSentFrame];

    if (lastSentFrameWasAn != PowerSavePollResponse) {
        (*this).DoubleTheContentionWindowAndPickANewBackoff(accessCategoryIndexForLastSentFrame);
    }//if//

    OutputTraceForCtsOrAckTimeout();
    //CTS or ACK failed.
    theAdaptiveRateControllerPtr->NotifyAckFailed(accessCategoryInfo.currentPacketsDestinationMacAddress);

    switch (lastSentFrameWasAn) {

    case RequestToSendFrame:
        accessCategoryInfo.currentShortFrameRetryCount++;
        if (accessCategoryInfo.currentShortFrameRetryCount >= shortFrameRetryLimit) {
            (*this).DropCurrentPacketAndGoToNextPacket(accessCategoryIndexForLastSentFrame);
        }//if//

        break;

    case ShortFrame:
        physicalLayerPtr->TakeOwnershipOfLastTransmittedFrame(accessCategoryInfo.currentPacketPtr);

        accessCategoryInfo.currentShortFrameRetryCount++;
        if (accessCategoryInfo.currentShortFrameRetryCount >= shortFrameRetryLimit) {
            (*this).DropCurrentPacketAndGoToNextPacket(accessCategoryIndexForLastSentFrame);
        }//if//
        break;

    case LongFrame:
        physicalLayerPtr->TakeOwnershipOfLastTransmittedFrame(accessCategoryInfo.currentPacketPtr);

        accessCategoryInfo.currentLongFrameRetryCount++;

        if (accessCategoryInfo.currentLongFrameRetryCount >= longFrameRetryLimit) {
            (*this).DropCurrentPacketAndGoToNextPacket(accessCategoryIndexForLastSentFrame);
        }//if//
        break;

    case AggregateFrame:
        physicalLayerPtr->TakeOwnershipOfLastTransmittedAggregateFrame(
            accessCategoryInfo.currentAggregateFramePtr);

        accessCategoryInfo.currentAggregateFrameRetryCount++;

        if (accessCategoryInfo.currentAggregateFrameRetryCount >= shortFrameRetryLimit) {
            assert(accessCategoryInfo.currentPacketPtr == nullptr);
            assert(!accessCategoryInfo.currentPacketIsAManagementFrame);

            (*this).DropCurrentAggregateFrameAndGoToNextPacket(accessCategoryIndexForLastSentFrame);

        }//if//
        break;

    case BlockAckRequestFrame:

        accessCategoryInfo.currentShortFrameRetryCount++;
        if (accessCategoryInfo.currentShortFrameRetryCount >= shortFrameRetryLimit) {
            // Block Ack Request is being used like an RTS, if it fails to be sent
            // then the DATA packet is thrown away (Stops infinite Block Ack Requests).

            (*this).DropCurrentPacketAndGoToNextPacket(accessCategoryIndexForLastSentFrame);
        }//if//

        break;

    case PowerSavePollResponse:

        // Review this (Jay):
        assert(false && "Not implemented or unnecessary else"); abort();
        break;

    default:
        assert(false); abort();
        break;
    }//switch//

}//NeverReceivedClearToSendOrAcknowledgementAction//



inline
void Dot11Mac::ProcessClearToSendOrAcknowledgementTimeout()
{
    if (!physicalLayerPtr->IsReceivingAFrame()) {
        // No possible ACK or CTS packet coming in.

        // Get out of the waiting for ACK/CTS state.
        macState = BusyMediumState;

        (*this).NeverReceivedClearToSendOrAcknowledgementAction();

        if (physicalLayerPtr->ChannelIsClear()) {
            if (simEngineInterfacePtr->CurrentTime() < mediumReservedUntilTimeAkaNAV) {

                (*this).GoIntoWaitForExpirationOfVirtualCarrierSenseAkaNavState();
            }
            else {
                mediumBecameIdleTime = simEngineInterfacePtr->CurrentTime();
                (*this).StartBackoffIfNecessary();

            }//if//
        }//if//
    }//if//

}//ProcessClearToSendOrAcknowledgementTimeout//


//--------------------------------------------------------------------------------------------------



inline
void Dot11Mac::ProcessRequestToSendFrame(
    const RequestToSendFrameType& rtsFrame,
    const TransmissionParametersType& receivedFrameTxParameters)
{
    if ((macState == WaitingForCtsState) || (macState == WaitingForAckState)) {
        macState = BusyMediumState;
        (*this).NeverReceivedClearToSendOrAcknowledgementAction();
    }//if//

    if (simEngineInterfacePtr->CurrentTime() < mediumReservedUntilTimeAkaNAV) {
        // Virtual Carrier Sense is active, ignore RTS.
        return;
    }//if//

    const unsigned int notUsedAccessCategoryIndex = 0;

    if ((*this).FrameTransmissionDurationExceedsTransmissionPermissionEndTime(
            sizeof(ClearToSendFrameType),
            receivedFrameTxParameters,
            aShortInterframeSpaceTime)) {
        macState = BusyMediumState;
        return;
    }//if//

    ClearToSendFrameType ctsFrame;

    ctsFrame.header.frameControlField.frameTypeAndSubtype = CTS_FRAME_TYPE_CODE;
    ctsFrame.header.receiverAddress = rtsFrame.transmitterAddress;
    ctsFrame.header.duration =
        ConvertTimeToDurationFieldValue(
            CalculateNavDurationForCtsFrame(
                rtsFrame.header.duration, ctsFrame.header.receiverAddress));

    unique_ptr<Packet> packetPtr = Packet::CreatePacket(*simEngineInterfacePtr, ctsFrame);

    const double transmitPowerDbm =
        theAdaptiveTxPowerControllerPtr->CurrentTransmitPowerDbm(ctsFrame.header.receiverAddress);

    // Clear to send response, ensure not in a TXOP.

    (*this).currentTransmitOpportunityAkaTxopEndTime = ZERO_TIME;

    (*this).macState = CtsOrAckTransmissionState;

    OutputTraceForCtsFrameTransmission();

    physicalLayerPtr->TransmitFrame(
        packetPtr,
        receivedFrameTxParameters,
        transmitPowerDbm,
        aShortInterframeSpaceTime);

}//ProcessRequestToSendFrame//



inline
void Dot11Mac::ProcessClearToSendFrame(const ClearToSendFrameType& ctsFrame)
{
    if (macState != WaitingForCtsState) {
        // The request to send (RTS) has been canceled for some reason, ignore the CTS.
        return;
    }//if//

    // Cancel CTS timeout if it hasn't expired yet (fast CTS).

    if (!wakeupTimerEventTicket.IsNull()) {
        (*this).CancelWakeupTimer();
    }//if//

    EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories[accessCategoryIndexForLastSentFrame];

    // Reset short retry count on successful RTS/CTS.

    accessCategoryInfo.currentShortFrameRetryCount = 0;

    bool packetWasSentToPhy;

    (*this).TransmitAFrame(
        accessCategoryIndexForLastSentFrame, true, aShortInterframeSpaceTime, packetWasSentToPhy);

    assert(packetWasSentToPhy);

}//ProcessClearToSendFrame//



inline
void Dot11Mac::ProcessPowerSavePollFrame(
    const PowerSavePollFrameType& psPollFrame,
    const TransmissionParametersType& pollFrameTxParameters)
{
    assert(GetOperationMode() == ApMode);

    bool datarateAndTxPowerAreSpecified;
    DatarateBitsPerSecType specifiedPacketDatarateBitsPerSec = 0;
    unsigned int frameSizeBytes;

    apControllerPtr->GetBufferedTopPacketInfo(
        psPollFrame.transmitterAddress,
        frameSizeBytes,
        datarateAndTxPowerAreSpecified,
        specifiedPacketDatarateBitsPerSec);

    frameSizeBytes += sizeof(QosDataFrameHeaderType);

    bool wasRetrieved = false;
    unique_ptr<Packet> powerSavePollResponsePacketPtr;
    PacketPriorityType priority;
    unsigned int retryTxCount = 0;
    EtherTypeFieldType etherType = 0;
    double specifiedPacketTxPowerDbm = 0.0;

    if (!(*this).FrameTransmissionDurationExceedsTransmissionPermissionEndTime(
            frameSizeBytes,
            pollFrameTxParameters,
            aShortInterframeSpaceTime)) {

        apControllerPtr->GetPowerSaveBufferedPacket(
            psPollFrame.transmitterAddress,
            wasRetrieved,
            powerSavePollResponsePacketPtr,
            retryTxCount,
            priority,
            etherType,
            datarateAndTxPowerAreSpecified,
            specifiedPacketDatarateBitsPerSec,
            specifiedPacketTxPowerDbm);
    }//if//

    if (!wasRetrieved) {

        if (!(*this).FrameTransmissionDurationExceedsTransmissionPermissionEndTime(
                sizeof(AcknowledgementAkaAckFrameType),
                pollFrameTxParameters,
                aShortInterframeSpaceTime)) {
            (*this).SendAckFrame(psPollFrame.transmitterAddress, pollFrameTxParameters);
        }//if//

        return;
    }//if//

    (*this).lastSentFrameWasAn = PowerSavePollResponse;
    (*this).lastSentDataFrameDestinationMacAddress = psPollFrame.transmitterAddress;
    (*this).powerSavePollResponsePacketTxCount = (retryTxCount + 1);

    TransmissionParametersType txParameters;

    if (specifiedPacketDatarateBitsPerSec) {
        if (theAdaptiveTxPowerControllerPtr->TxPowerIsSpecifiedByPhyLayer()) {
            cerr << "Error: Set dot11-tx-power-specified-by = UpperLayer to specify datarate and tx-power by Upper Layer" << endl;
            exit(1);
        }

        theAdaptiveRateControllerPtr->GetDataRateInfoForDatarateSpecifiedFrame(
            specifiedPacketDatarateBitsPerSec,
            txParameters);
    }
    else {
        theAdaptiveRateControllerPtr->GetDataRateInfoForDataFrameToStation(
            psPollFrame.transmitterAddress,
            txParameters);
    }//if//

    QosDataFrameHeaderType dataFrameHeader;

    dataFrameHeader.header.frameControlField.frameTypeAndSubtype = QOS_DATA_FRAME_TYPE_CODE;
    dataFrameHeader.header.frameControlField.isRetry = (powerSavePollResponsePacketTxCount > 1);
    dataFrameHeader.header.duration =
        ConvertTimeToDurationFieldValue(CalculateNavDurationForAckedDataFrame(txParameters));

    dataFrameHeader.header.receiverAddress = psPollFrame.transmitterAddress;
    (*this).lastSentDataFrameDestinationMacAddress = dataFrameHeader.header.receiverAddress;

    unsigned short int sequenceNumber;

    (*this).GetNewSequenceNumber(
        psPollFrame.transmitterAddress,
        priority,
        true,
        /*out*/sequenceNumber);

    dataFrameHeader.sequenceControlField.sequenceNumber = sequenceNumber;
    dataFrameHeader.transmitterAddress = myMacAddress;
    dataFrameHeader.qosControlField.trafficId = priority;
    dataFrameHeader.linkLayerHeader.etherType = HostToNet16(etherType);

    powerSavePollResponsePacketPtr->AddPlainStructHeader(dataFrameHeader);

    txParameters.firstChannelNumber =
        GetFirstChannelNumberForChannelBandwidth(txParameters.channelBandwidthMhz);

    double transmitPowerDbm;

    if (datarateAndTxPowerAreSpecified) {
        assert(!theAdaptiveTxPowerControllerPtr->TxPowerIsSpecifiedByPhyLayer());
        transmitPowerDbm = specifiedPacketTxPowerDbm;
    }
    else {
        transmitPowerDbm =
            theAdaptiveTxPowerControllerPtr->CurrentTransmitPowerDbm(psPollFrame.transmitterAddress);
    }//if//

    (*this).macState = WaitingForAckState;

    physicalLayerPtr->TransmitFrame(
        powerSavePollResponsePacketPtr,
        txParameters,
        transmitPowerDbm,
        aShortInterframeSpaceTime);

}//ProcessPowerSavePollFrame//


//--------------------------------------------------------------------------------------------------

inline
bool Dot11Mac::BlockAckSessionIsEnabled(
    const MacAddressType& destinationAddress,
    const PacketPriorityType& trafficId) const
{
    typedef map<AddressAndTrafficIdMapKeyType, OutgoingLinkInfoType>::const_iterator IterType;

    AddressAndTrafficIdMapKeyType aKey(destinationAddress, trafficId);

    const IterType iter = outgoingLinkInfoMap.find(aKey);

    if (iter == outgoingLinkInfoMap.end()) {
        return false;
    }
    else {
        return (iter->second.blockAckSessionIsEnabled);
    }//if//

}//BlockAckSessionIsEnabled//


inline
void Dot11Mac::GetNewSequenceNumber(
    const MacAddressType& destinationAddress,
    const PacketPriorityType& trafficId,
    const bool isNonBlockAckedFrame,
    unsigned short int& newSequenceNumber)
{
    typedef map<AddressAndTrafficIdMapKeyType, OutgoingLinkInfoType>::iterator IterType;

    AddressAndTrafficIdMapKeyType aKey(destinationAddress, trafficId);

    const IterType iter = outgoingLinkInfoMap.find(aKey);

    if (iter == outgoingLinkInfoMap.end()) {
        outgoingLinkInfoMap[aKey].outgoingSequenceNumber = 1;
        outgoingLinkInfoMap[aKey].windowStartSequenceNumber = 1;

        newSequenceNumber = 1;
    }
    else {
        OutgoingLinkInfoType& linkInfo = iter->second;
        unsigned short int& lastOutgoingSequenceNumber = linkInfo.outgoingSequenceNumber;
        IncrementTwelveBitSequenceNumber(lastOutgoingSequenceNumber);
        newSequenceNumber = lastOutgoingSequenceNumber;

        if (isNonBlockAckedFrame) {
            linkInfo.windowStartSequenceNumber = newSequenceNumber;
        }//if//

        assert(
            CalcTwelveBitSequenceNumberDifference(
                newSequenceNumber, linkInfo.windowStartSequenceNumber) < BlockAckBitMapNumBits);
    }//if//

}//GetNewSequenceNumber//


inline
unsigned int Dot11Mac::CalcNumberFramesLeftInSequenceNumWindow(
    const MacAddressType& destinationAddress,
    const PacketPriorityType& trafficId) const
{
    const OutgoingLinkInfoType& linkInfo =
        outgoingLinkInfoMap.find(AddressAndTrafficIdMapKeyType(destinationAddress, trafficId))->second;

    const int diff =
        CalcTwelveBitSequenceNumberDifference(
            linkInfo.outgoingSequenceNumber, linkInfo.windowStartSequenceNumber);

    assert((diff >= 0) && (diff <= BlockAckBitMapNumBits));

    return (BlockAckBitMapNumBits - diff - 1);

}//CalcNumberFramesLeftInSequenceNumWindow//



inline
void Dot11Mac::SetSequenceNumberWindowStart(
    const MacAddressType& destinationAddress,
    const PacketPriorityType& trafficId,
    const unsigned short int sequenceNum)
{
    typedef map<AddressAndTrafficIdMapKeyType, OutgoingLinkInfoType>::iterator IterType;

    const IterType iter =
        outgoingLinkInfoMap.find(AddressAndTrafficIdMapKeyType(destinationAddress, trafficId));

    assert(iter != outgoingLinkInfoMap.end());
    OutgoingLinkInfoType& linkInfo = iter->second;

    linkInfo.windowStartSequenceNumber = sequenceNum;
}



//--------------------------------------------------------------------------------------------------


inline
void Dot11Mac::ProcessAcknowledgementFrame(const AcknowledgementAkaAckFrameType& ackFrame)
{
    if (macState != WaitingForAckState) {
        // Received an ACK, but was not expecting it ==> ignore.
        return;
    }//if//

    // Cancel ACK timeout if it hasn't expired yet (fast ACK response).

    if (!wakeupTimerEventTicket.IsNull()) {
        (*this).CancelWakeupTimer();
    }//if//

    (*this).macState = BusyMediumState;

    if (lastSentFrameWasAn != PowerSavePollResponse) {

        EdcaAccessCategoryInfo& accessCategoryInfo =
            accessCategories[accessCategoryIndexForLastSentFrame];

        // Success => Reset Contention window.

        accessCategoryInfo.currentContentionWindowSlots = accessCategoryInfo.minContentionWindowSlots;

        // Ownership of packet was given to PHY.

        assert(accessCategoryInfo.currentPacketPtr == nullptr);

        assert(lastSentDataFrameDestinationMacAddress ==
               accessCategoryInfo.currentPacketsDestinationMacAddress);

    }//if//

    theAdaptiveRateControllerPtr->NotifyAckReceived(lastSentDataFrameDestinationMacAddress);

    (*this).currentTransmitOpportunityAckedFrameCount++;
    (*this).DoSuccessfulTransmissionPostProcessing(false);

}//ProcessAcknowledgementFrame//



inline
void Dot11Mac::ProcessBlockAckFrame(const BlockAcknowledgementFrameType& blockAckFrame)
{
    if (macState != WaitingForAckState) {
        // Received an ACK, but was not expecting it ==> ignore.
        return;
    }//if//

    // Cancel ACK timeout if it hasn't expired yet (fast ACK response).

    if (!wakeupTimerEventTicket.IsNull()) {
        (*this).CancelWakeupTimer();
    }//if//

    assert(lastSentFrameWasAn != PowerSavePollResponse);

    OutgoingLinkInfoType& linkInfo =
        outgoingLinkInfoMap[
            AddressAndTrafficIdMapKeyType(
                blockAckFrame.transmitterAddress,
                blockAckFrame.blockAckControl.trafficId)];

    linkInfo.blockAckRequestNeedsToBeSent = false;

    (*this).currentTransmitOpportunityAckedFrameCount++;

    linkInfo.windowStartSequenceNumber = blockAckFrame.startingSequenceControl;

    for(unsigned int i = 0; (i < blockAckFrame.blockAckBitmap.size()); i++) {
        if (blockAckFrame.blockAckBitmap[i]) {
            IncrementTwelveBitSequenceNumber(linkInfo.windowStartSequenceNumber);
        }
        else {
            break;
        }//if//
    }//for//

    EdcaAccessCategoryInfo& accessCategoryInfo =
        accessCategories[accessCategoryIndexForLastSentFrame];

    if (lastSentFrameWasAn == AggregateFrame) {
        assert(lastSentFrameWasAn == AggregateFrame);

        assert(lastSentDataFrameDestinationMacAddress ==
           accessCategoryInfo.currentPacketsDestinationMacAddress);

        assert(accessCategoryInfo.currentPacketPtr == nullptr);
        assert(accessCategoryInfo.currentAggregateFramePtr == nullptr);

        physicalLayerPtr->TakeOwnershipOfLastTransmittedAggregateFrame(
            accessCategoryInfo.currentAggregateFramePtr);

        (*this).macState = BusyMediumState;

        vector<unique_ptr<Packet> >& sentSubframes = *accessCategoryInfo.currentAggregateFramePtr;

        unsigned int numberAckedSubframes = 0;
        for(unsigned int i = 0; (i < sentSubframes.size()); i++) {
            const Packet& aSubframe = *sentSubframes[i];
            const QosDataFrameHeaderType& header =
                aSubframe.GetAndReinterpretPayloadData<QosDataFrameHeaderType>(
                    sizeof(MpduDelimiterFrameType));

            if (blockAckFrame.IsAcked(header.sequenceControlField.sequenceNumber)) {
                sentSubframes[i].reset();
                numberAckedSubframes++;
                theAdaptiveRateControllerPtr->NotifyAckReceived(lastSentDataFrameDestinationMacAddress);
            }
            else {
                theAdaptiveRateControllerPtr->NotifyAckFailed(lastSentDataFrameDestinationMacAddress);
            }//if//
        }//for//

        accessCategoryInfo.currentAggregateFrameRetryCount++;

        if (numberAckedSubframes == sentSubframes.size()) {
            accessCategoryInfo.currentAggregateFramePtr.reset();
            accessCategoryInfo.currentAggregateFrameRetryCount = 0;
        }
        else if (accessCategoryInfo.currentAggregateFrameRetryCount >= shortFrameRetryLimit) {

            unsigned short int droppedFrameSequenceNumber = 0;
            bool sequenceNumberWasSet = false;

            for(unsigned int i = 0; (i < sentSubframes.size()); i++) {
                if (sentSubframes[i] != nullptr) {
                    const Packet& aSubframe = *sentSubframes[i];
                    const QosDataFrameHeaderType& header =
                        aSubframe.GetAndReinterpretPayloadData<QosDataFrameHeaderType>(
                            sizeof(MpduDelimiterFrameType));

                    droppedFrameSequenceNumber = header.sequenceControlField.sequenceNumber;
                    sequenceNumberWasSet = true;
                }//if//
            }//for//

            accessCategoryInfo.currentAggregateFramePtr.reset();
            accessCategoryInfo.currentAggregateFrameRetryCount = 0;

            // Send Block Ack Request as a "management frame" to shift sequence number
            // window at destination for dropped frame.  In case that fails,
            // "mark" the link as needing an sequence number window update.

            linkInfo.blockAckRequestNeedsToBeSent = true;
            assert(sequenceNumberWasSet);
            linkInfo.lastDroppedFrameSequenceNumber = droppedFrameSequenceNumber;

            //Future Idea: Force a BAR by queueing a management frame that must be deleted if has
            //Future Idea: already been sent.  For quick delivery of buffered out-of-order frames at
            //Future Idea: the receiver.
            //Future Idea
            //Future Idea // Obsolete G++ Hack.
            //Future Idea unique_ptr<Packet> framePtr =
            //Future Idea    CreateABlockAckRequestFrame(
            //Future Idea        accessCategoryInfo.currentPacketsDestinationMacAddress,
            //Future Idea        accessCategoryInfo.currentPacketPriorityAkaTrafficId,
            //Future Idea        (droppedFrameSequenceNumber + 1));
            //Future Idea
            //Future Idea (*this).SendManagementFrame(framePtr);
        }
        else {
            unsigned int i = 0;
            while(i < sentSubframes.size()) {
                if (sentSubframes[i] == nullptr) {
                    sentSubframes.erase(sentSubframes.begin() + i);
                }
                else {
                    i++;
                }//if//
            }//while//
        }//if//

        assert((!protectAggregateFramesWithSingleAckedFrame) ||
            (accessCategoryInfo.currentContentionWindowSlots ==
               accessCategoryInfo.minContentionWindowSlots));

        accessCategoryInfo.currentContentionWindowSlots = accessCategoryInfo.minContentionWindowSlots;

        (*this).DoSuccessfulTransmissionPostProcessing(false);
    }
    else {
        assert(lastSentFrameWasAn == BlockAckRequestFrame);

        // This case is responding to block ack request (sequence # window sync)
        // (there are no buffered subframes in the aggregate).

        accessCategoryInfo.currentShortFrameRetryCount = 0;

        // Success => Reset Contention window.

        accessCategoryInfo.currentContentionWindowSlots = accessCategoryInfo.minContentionWindowSlots;

        assert(lastSentDataFrameDestinationMacAddress == accessCategoryInfo.currentPacketsDestinationMacAddress);

        (*this).currentTransmitOpportunityAckedFrameCount++;

        theAdaptiveRateControllerPtr->NotifyAckReceived(lastSentDataFrameDestinationMacAddress);

        (*this).macState = BusyMediumState;
        (*this).DoSuccessfulTransmissionPostProcessing(false);
    }//if//

}//ProcessBlockAckFrame//



inline
unique_ptr<Packet> Dot11Mac::CreateABlockAckRequestFrame(
    const MacAddressType& destinationAddress,
    const PacketPriorityType& trafficId,
    const unsigned short int startFrameSequenceNumber) const
{
    BlockAcknowledgementRequestFrameType blockAckRequest;

    blockAckRequest.header.frameControlField.frameTypeAndSubtype = BLOCK_ACK_REQUEST_FRAME_TYPE_CODE;
    blockAckRequest.header.receiverAddress = destinationAddress;
    blockAckRequest.header.duration = 0;
    blockAckRequest.header.frameControlField.isRetry = 0;
    blockAckRequest.blockAckRequestControl.trafficId = trafficId;
    blockAckRequest.transmitterAddress = GetMacAddress();
    blockAckRequest.startingSequenceControl = startFrameSequenceNumber;

    return (Packet::CreatePacket(*simEngineInterfacePtr, blockAckRequest));

}//CreateABlockAckRequestFrame//



inline
void Dot11Mac::NotifyThatPhyReceivedCorruptedFrame()
{
    (*this).lastFrameReceivedWasCorrupt = true;

    //globalMacStatistics.totalNumberReceivedCorruptedFrame++;

    if ((macState == WaitingForCtsState) || (macState == WaitingForAckState)) {
        macState = BusyMediumState;
        // Received a corrupt packet that wasn't the desired ACK or CTS for this node, must retry.
        (*this).NeverReceivedClearToSendOrAcknowledgementAction();

    }//if//

    assert(macState == BusyMediumState);
}


inline
void Dot11Mac::ProcessWakeupTimerEvent()
{
    (*this).wakeupTimerEventTicket.Clear();
    (*this).currentWakeupTimerExpirationTime = INFINITE_TIME;

    switch (macState) {
    case WaitingForNavExpirationState:
        (*this).ProcessVirtualCarrierSenseAkaNavExpiration();
        break;

    case WaitingForIfsAndBackoffState:
        (*this).ProcessInterframeSpaceAndBackoffTimeout();
        break;

    case WaitingForCtsState:
    case WaitingForAckState:
        (*this).ProcessClearToSendOrAcknowledgementTimeout();
        break;

    case IdleState:
        assert(false); abort(); break;
    default:
        assert(false); abort(); break;
    }//switch//

}//ProcessWakeupTimerEvent//


inline
void Dot11Mac::SendManagementFrame(unique_ptr<Packet>& framePtr)
{
    (*this).managementFrameQueue.push_back(move(framePtr));
    if (!AccessCategoryIsActive(accessCategoryIndexForManagementFrames)) {
        (*this).StartPacketSendProcessForAnAccessCategory(accessCategoryIndexForManagementFrames);
    }//if//
}



inline
void Dot11Mac::SendAssociationRequest(const MacAddressType& apAddress)
{
    AssociationRequestFrameType associationRequestFrame;
    associationRequestFrame.managementHeader.header.frameControlField.frameTypeAndSubtype = ASSOCIATION_REQUEST_FRAME_TYPE_CODE;
    associationRequestFrame.managementHeader.header.frameControlField.isRetry = 0;
    associationRequestFrame.managementHeader.header.duration = 0;
    associationRequestFrame.managementHeader.header.receiverAddress = apAddress;
    associationRequestFrame.managementHeader.transmitterAddress = myMacAddress;
    associationRequestFrame.htCapabilitiesFrameElement.aggregateMpdusAreEnabled =
        (maxAggregateMpduSizeBytes > 0);

    unsigned short int newSequenceNumber;
    (*this).GetNewSequenceNumber(apAddress, maxPacketPriority, true, newSequenceNumber);
    associationRequestFrame.managementHeader.sequenceControlField.sequenceNumber = newSequenceNumber;

    unique_ptr<Packet> packetPtr = Packet::CreatePacket<AssociationRequestFrameType>(
        *simEngineInterfacePtr,
        associationRequestFrame);

    (*this).SendManagementFrame(packetPtr);
}


inline
void Dot11Mac::SendReassociationRequest(
    const MacAddressType& apAddress,
    const MacAddressType& currentApAddress)
{
    ReassociationRequestFrameType reassociationRequestFrame;
    reassociationRequestFrame.managementHeader.header.frameControlField.frameTypeAndSubtype = REASSOCIATION_REQUEST_FRAME_TYPE_CODE;
    reassociationRequestFrame.managementHeader.header.frameControlField.isRetry = 0;
    reassociationRequestFrame.managementHeader.header.duration = 0;
    reassociationRequestFrame.managementHeader.header.receiverAddress = apAddress;
    reassociationRequestFrame.managementHeader.transmitterAddress = myMacAddress;
    reassociationRequestFrame.currentApAddress = currentApAddress;
    reassociationRequestFrame.htCapabilitiesFrameElement.aggregateMpdusAreEnabled =
        (maxAggregateMpduSizeBytes > 0);

    unsigned short int newSequenceNumber;
    (*this).GetNewSequenceNumber(apAddress, maxPacketPriority, true, newSequenceNumber);
    reassociationRequestFrame.managementHeader.sequenceControlField.sequenceNumber = newSequenceNumber;

    unique_ptr<Packet> packetPtr = Packet::CreatePacket<ReassociationRequestFrameType>(
        *simEngineInterfacePtr,
        reassociationRequestFrame);

    (*this).SendManagementFrame(packetPtr);
}



inline
void Dot11Mac::SendDisassociation(const MacAddressType& receiverAddress)
{
    DisassociationFrameType disassociationFrame;

    disassociationFrame.managementHeader.header.frameControlField.frameTypeAndSubtype = DISASSOCIATION_FRAME_TYPE_CODE;
    disassociationFrame.managementHeader.header.frameControlField.isRetry = 0;
    disassociationFrame.managementHeader.header.duration = 0;
    disassociationFrame.managementHeader.header.receiverAddress = receiverAddress;
    disassociationFrame.managementHeader.transmitterAddress = myMacAddress;

    unsigned short int newSequenceNumber;
    (*this).GetNewSequenceNumber(receiverAddress, maxPacketPriority, true, newSequenceNumber);
    disassociationFrame.managementHeader.sequenceControlField.sequenceNumber = newSequenceNumber;

    unique_ptr<Packet> packetPtr = Packet::CreatePacket<DisassociationFrameType>(
        *simEngineInterfacePtr,
        disassociationFrame);

    (*this).SendManagementFrame(packetPtr);

}//SendDisassociation//



inline
void Dot11Mac::SendAssociationResponse(
    const MacAddressType& staAddress,
    const AssociationIdType& associationId)
{
    AssociationResponseFrameType associationResponseFrame;
    associationResponseFrame.managementHeader.header.frameControlField.frameTypeAndSubtype = ASSOCIATION_RESPONSE_FRAME_TYPE_CODE;
    associationResponseFrame.managementHeader.header.frameControlField.isRetry = 0;
    associationResponseFrame.managementHeader.header.duration = 0;
    associationResponseFrame.managementHeader.header.receiverAddress = staAddress;
    associationResponseFrame.managementHeader.transmitterAddress = myMacAddress;
    associationResponseFrame.associationId = associationId;
    associationResponseFrame.htCapabilitiesFrameElement.aggregateMpdusAreEnabled =
        (maxAggregateMpduSizeBytes > 0);

    unsigned short int newSequenceNumber;
    (*this).GetNewSequenceNumber(staAddress, maxPacketPriority, true, newSequenceNumber);
    associationResponseFrame.managementHeader.sequenceControlField.sequenceNumber = newSequenceNumber;

    unique_ptr<Packet> packetPtr = Packet::CreatePacket<AssociationResponseFrameType>(
        *simEngineInterfacePtr,
        associationResponseFrame);

    (*this).SendManagementFrame(packetPtr);

}//SendAssociationResponse//



inline
void Dot11Mac::SendReassociationResponse(
    const MacAddressType& staAddress,
    const AssociationIdType& associationId)
{
    ReassociationResponseFrameType reassociationResponseFrame;
    reassociationResponseFrame.managementHeader.header.frameControlField.frameTypeAndSubtype = REASSOCIATION_RESPONSE_FRAME_TYPE_CODE;
    reassociationResponseFrame.managementHeader.header.frameControlField.isRetry = 0;
    reassociationResponseFrame.managementHeader.header.duration = 0;
    reassociationResponseFrame.managementHeader.header.receiverAddress = staAddress;
    reassociationResponseFrame.managementHeader.transmitterAddress = myMacAddress;
    reassociationResponseFrame.htCapabilitiesFrameElement.aggregateMpdusAreEnabled =
        (maxAggregateMpduSizeBytes > 0);

    reassociationResponseFrame.associationId = associationId;

    unsigned short int newSequenceNumber;
    (*this).GetNewSequenceNumber(staAddress, maxPacketPriority, true, newSequenceNumber);
    reassociationResponseFrame.managementHeader.sequenceControlField.sequenceNumber = newSequenceNumber;

    unique_ptr<Packet> packetPtr = Packet::CreatePacket<ReassociationResponseFrameType>(
        *simEngineInterfacePtr,
        reassociationResponseFrame);

    (*this).SendManagementFrame(packetPtr);
}




inline
void Dot11Mac::SendAuthentication(const MacAddressType& receiverAddress)
{
    AuthenticationFrameType authenticationFrame;

    authenticationFrame.managementHeader.header.frameControlField.frameTypeAndSubtype = AUTHENTICATION_FRAME_TYPE_CODE;
    authenticationFrame.managementHeader.header.frameControlField.isRetry = 0;
    authenticationFrame.managementHeader.header.duration = 0;
    authenticationFrame.managementHeader.header.receiverAddress = receiverAddress;
    authenticationFrame.managementHeader.transmitterAddress = myMacAddress;

    unsigned short int newSequenceNumber;
    (*this).GetNewSequenceNumber(receiverAddress, maxPacketPriority, true, newSequenceNumber);
    authenticationFrame.managementHeader.sequenceControlField.sequenceNumber = newSequenceNumber;

    unique_ptr<Packet> packetPtr = Packet::CreatePacket<AuthenticationFrameType>(
        *simEngineInterfacePtr,
        authenticationFrame);

    (*this).SendManagementFrame(packetPtr);

}//SendAuthentication//



inline
void Dot11Mac::SendPowerSaveNullFrame(
    const MacAddressType& receiverAddress,
    const bool goingToPowerManagementMode)
{
    assert(managementFrameQueue.empty());

    QosDataFrameHeaderType nullFrame;
    nullFrame.header.frameControlField.frameTypeAndSubtype = NULL_FRAME_TYPE_CODE;
    nullFrame.header.frameControlField.isRetry = 0;
    nullFrame.header.frameControlField.powerManagement = 0;
    if (goingToPowerManagementMode) {
        nullFrame.header.frameControlField.powerManagement = 1;
    }//if//
    nullFrame.header.duration = 0;
    nullFrame.header.receiverAddress = receiverAddress;

    nullFrame.sequenceControlField.sequenceNumber = 0;
    nullFrame.transmitterAddress = myMacAddress;
    nullFrame.qosControlField.trafficId = 0;

    unique_ptr<Packet> packetPtr = Packet::CreatePacket<QosDataFrameHeaderType>(*simEngineInterfacePtr, nullFrame);

    (*this).SendManagementFrame(packetPtr);

}//SendPowerSaveNullFrame//



inline
unsigned int Dot11Mac::GetFirstChannelNumberForChannelBandwidth(
    const unsigned int signalChannelBandwidthMhz) const
{
    const vector<unsigned int>& bondedChannelList = physicalLayerPtr->GetCurrentBondedChannelList();

    const unsigned int numberChannelsInSignal =
        signalChannelBandwidthMhz / physicalLayerPtr->GetBaseChannelBandwidthMhz();

    assert(numberChannelsInSignal > 0);
    assert(numberChannelsInSignal <= bondedChannelList.size());

    unsigned int firstChannelNumber = bondedChannelList[0];
    for(unsigned int i = 1; (i < numberChannelsInSignal); i++) {
        if (firstChannelNumber > bondedChannelList[i]) {
            firstChannelNumber = bondedChannelList[i];
        }//if//
    }//for//

    return (firstChannelNumber);

}//GetFirstChannelNumberForChannelBandwidth//


inline
void Dot11Mac::SwitchToChannels(const vector<unsigned int>& newBondedChannelList)
{
    if (physicalLayerPtr->IsTransmittingAFrame()) {

        (*this).macState = ChangingChannelsState;
        (*this).switchingToThisChannelList = newBondedChannelList;
    }
    else {
        physicalLayerPtr->SwitchToChannels(newBondedChannelList);
    }//if//

}//SwitchToChannels//

inline
void Dot11Mac::SwitchToChannel(const unsigned int channelNumber)
{
    vector<unsigned int> bondedChannelList(1);
    bondedChannelList[0] = channelNumber;
    (*this).SwitchToChannels(bondedChannelList);
}



inline
Dot11MacOperationMode Dot11Mac::GetOperationMode() const {
    return (operationMode);
}


inline
void Dot11Mac::OutputTraceAndStatsForFrameReceive(const Packet& aFrame) const
{
    const CommonFrameHeaderType& header = aFrame.GetAndReinterpretPayloadData<CommonFrameHeaderType>();

    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            MacFrameReceiveTraceRecord traceData;

            const PacketIdType& packetId = aFrame.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            traceData.frameType = header.frameControlField.frameTypeAndSubtype;

            traceData.packetLengthBytes = static_cast<uint16_t>(aFrame.LengthBytes());

            assert(sizeof(traceData) == MAC_FRAME_RECEIVE_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "RxFrame", traceData);
        }
        else {

            ostringstream msgStream;

            msgStream << "PktId= " << aFrame.GetPacketId();

            msgStream << " FrameBytes= " << aFrame.LengthBytes();

            msgStream << " FrameType= ";

            msgStream << ConvertToDot11FrameTypeName(header.frameControlField.frameTypeAndSubtype);

            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "RxFrame", msgStream.str());

        }//if//
    }//if//

    switch (header.frameControlField.frameTypeAndSubtype) {
    case RTS_FRAME_TYPE_CODE: {
        rtsFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case CTS_FRAME_TYPE_CODE: {
        ctsFramesReceivedStatPtr->IncrementCounter();
        break;
    }

    case QOS_DATA_FRAME_TYPE_CODE: {
        dataFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case ACK_FRAME_TYPE_CODE: {
        ackFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case BEACON_FRAME_TYPE_CODE: {
        beaconFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case ASSOCIATION_REQUEST_FRAME_TYPE_CODE: {
        associationRequestFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case ASSOCIATION_RESPONSE_FRAME_TYPE_CODE: {
        associationResponseFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case REASSOCIATION_REQUEST_FRAME_TYPE_CODE: {
        reassociationRequestFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case REASSOCIATION_RESPONSE_FRAME_TYPE_CODE: {
        reassociationResponseFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case DISASSOCIATION_FRAME_TYPE_CODE: {
        disassociationFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case AUTHENTICATION_FRAME_TYPE_CODE: {
        authenticationFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case BLOCK_ACK_FRAME_TYPE_CODE: {
        blockAckFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case BLOCK_ACK_REQUEST_FRAME_TYPE_CODE: {
        break;
    }
    default:
        assert(false); abort();
        break;
    }//switch//

}//OutputTraceAndStatsForFrameReceive//



inline
void Dot11Mac::OutputTraceAndStatsForAggregateSubframeReceive(
    const Packet& aFrame,
    const QosDataFrameHeaderType& dataFrameHeader)
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            MacFrameReceiveTraceRecord traceData;

            const PacketIdType& packetId = aFrame.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            traceData.frameType = dataFrameHeader.header.frameControlField.frameTypeAndSubtype;

            traceData.packetLengthBytes = static_cast<uint16_t>(aFrame.LengthBytes());

            assert(sizeof(traceData) == MAC_FRAME_RECEIVE_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "RxFrame", traceData);
        }
        else {

            ostringstream msgStream;

            msgStream << "PktId= " << aFrame.GetPacketId();

            msgStream << " FrameBytes= " << aFrame.LengthBytes();

            msgStream << " FrameType= ";

            msgStream << ConvertToDot11FrameTypeName(
                dataFrameHeader.header.frameControlField.frameTypeAndSubtype);

            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "RxFrame", msgStream.str());

        }//if//
    }//if//

    dataAggregatedSubframesReceivedStatPtr->IncrementCounter();

}//OutputTraceAndStatsForAggregateSubframeReceive//



inline
void Dot11Mac::OutputTraceForClearChannel() const
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
void Dot11Mac::OutputTraceForBusyChannel() const
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
void Dot11Mac::OutputTraceForNavStart(const TimeType& expirationTime) const
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
void Dot11Mac::OutputTraceForNavExpiration() const
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
void Dot11Mac::OutputTraceForIfsAndBackoffStart(const TimeType& backoffDuration) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacIfsAndBackoffStartTraceRecord traceData;

            traceData.accessCategory = static_cast<uint32_t>(FindIndexOfAccessCategoryWithShortestBackoff());
            traceData.duration = backoffDuration;
            traceData.frameCorrupt = lastFrameReceivedWasCorrupt;

            assert(sizeof(traceData) == MAC_IFS_AND_BACKOFF_START_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "IFSAndBackoffStart", traceData);
        }
        else {
            ostringstream msgStream;

            msgStream << "AC= " << FindIndexOfAccessCategoryWithShortestBackoff();
            msgStream << " Dur= " << ConvertTimeToStringSecs(backoffDuration);

            if (lastFrameReceivedWasCorrupt) {
                msgStream << " Ext= Yes";
            }
            else {
                msgStream << " Ext= No";
            }//if//

            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "IFSAndBackoffStart", msgStream.str());

        }//if//
    }//if//

}//OutputTraceForIfsAndBackoffStart//


inline
void Dot11Mac::OutputTraceForInterruptedIfsAndBackoff(
    const unsigned int accessCategoryIndex,
    const TimeType nonExtendedDurationLeft) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacIfsAndBackoffPauseTraceRecord traceData;

            traceData.accessCategory = static_cast<uint32_t>(FindIndexOfAccessCategoryWithShortestBackoff());
            traceData.leftDuration = nonExtendedDurationLeft;

            assert(sizeof(traceData) == MAC_IFS_AND_BACKOFF_PAUSE_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "IFSAndBackoffPause", traceData);
        }
        else {
            ostringstream msgStream;

            msgStream << "AC= " << FindIndexOfAccessCategoryWithShortestBackoff();
            msgStream << " Left= " << ConvertTimeToStringSecs(nonExtendedDurationLeft);

            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "IFSAndBackoffPause", msgStream.str());

        }//if//
    }//if//
}


inline
void Dot11Mac::OutputTraceForIfsAndBackoffExpiration() const
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
void Dot11Mac::OutputTraceForPacketDequeue(const unsigned int accessCategoryIndex, const TimeType delayUntilAirBorne) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacPacketDequeueTraceRecord traceData;

            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

            const PacketIdType& packetId = accessCategoryInfo.currentPacketPtr->GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            traceData.accessCategory = static_cast<uint32_t>(accessCategoryIndex);

            assert(!redundantTraceInformationModeIsOn);// no implementation for redundant trace

            assert(sizeof(traceData) == MAC_PACKET_DEQUEUE_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "Dequeue", traceData);
        }
        else {
            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

            ostringstream msgStream;

            msgStream << "PktId= " << accessCategoryInfo.currentPacketPtr->GetPacketId();
            msgStream << " AC= " << accessCategoryIndex;
            if (redundantTraceInformationModeIsOn) {
                msgStream << " Dur= " <<  ConvertTimeToStringSecs(delayUntilAirBorne);
            }
            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "Dequeue", msgStream.str());
        }//if//
    }//if//

}//OutputTraceForPacketDequeue//

inline
void Dot11Mac::OutputTraceAndStatsForRtsFrameTransmission(const unsigned int accessCategoryIndex) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacTxRtsTraceRecord traceData;

            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

            traceData.accessCategory = static_cast<uint32_t>(accessCategoryIndex);
            traceData.retry = accessCategoryInfo.currentShortFrameRetryCount;

            assert(sizeof(traceData) == MAC_TX_RTS_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "Tx-RTS", traceData);
        }
        else {
            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

            ostringstream msgStream;

            msgStream << "AC= " << accessCategoryIndex;
            msgStream << " S_Retry= " << accessCategoryInfo.currentShortFrameRetryCount;

            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "Tx-RTS", msgStream.str());
        }//if//
    }//if//

    rtsFramesSentStatPtr->IncrementCounter();

}//OutputTraceAndStatsForRtsFrameTransmission//


inline
void Dot11Mac::OutputTraceForCtsFrameTransmission() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "Tx-CTS");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "Tx-CTS", "");
        }//if//
    }//if//

    ctsFramesSentStatPtr->IncrementCounter();

}//OutputTraceForCtsFrameTransmission//


inline
void Dot11Mac::OutputTraceAndStatsForAggregatedFrameTransmission(const unsigned int accessCategoryIndex) const
{
    const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacTxUnicastDataTraceRecord traceData;

            const PacketIdType& packetId = accessCategoryInfo.currentAggregateFramePtr->front()->GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            traceData.accessCategory = static_cast<uint32_t>(accessCategoryIndex);
            traceData.shortFrameRetry = accessCategoryInfo.currentAggregateFrameRetryCount;
            traceData.longFrameRetry = accessCategoryInfo.currentLongFrameRetryCount;

            assert(sizeof(traceData) == MAC_TX_UNICAST_DATA_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "Tx-DATA-A", traceData);
        }
        else {

            ostringstream msgStream;

            msgStream << "PktId= " << accessCategoryInfo.currentAggregateFramePtr->front()->GetPacketId();
            msgStream << " AC= " << accessCategoryIndex;
            msgStream << " S-Retry= " << accessCategoryInfo.currentAggregateFrameRetryCount;
            msgStream << " L-Retry= " << accessCategoryInfo.currentLongFrameRetryCount;

            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "Tx-DATA-A", msgStream.str());
        }//if//
    }//if//

    dataAggregateFramesSentStatPtr->IncrementCounter();

    if (accessCategoryInfo.currentAggregateFrameRetryCount > 0) {
        dataAggregateFramesResentStatPtr->IncrementCounter();
    }//if//

}//OutputTraceAndStatsForAggregatedFrameTransmission//


inline
void Dot11Mac::OutputTraceAndStatsForBroadcastDataFrameTransmission(const unsigned int accessCategoryIndex) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacTxBroadcastDataTraceRecord traceData;
            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

            const PacketIdType& packetId = accessCategoryInfo.currentPacketPtr->GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            traceData.accessCategory = static_cast<uint32_t>(accessCategoryIndex);

            assert(sizeof(traceData) == MAC_TX_BROADCAST_DATA_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "Tx-DATA-B", traceData);
        }
        else {
            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);
            ostringstream msgStream;

            msgStream << "PktId= " << accessCategoryInfo.currentPacketPtr->GetPacketId();
            msgStream << " AC= " << accessCategoryIndex;

            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "Tx-DATA-B", msgStream.str());
        }//if//
    }//if//

    broadcastDataFramesSentStatPtr->IncrementCounter();

}//OutputTraceAndStatsForBroadcastDataFrameTransmission//



inline
void Dot11Mac::OutputTraceAndStatsForUnicastDataFrameTransmission(const unsigned int accessCategoryIndex) const
{
    const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacTxUnicastDataTraceRecord traceData;

            const PacketIdType& packetId = accessCategoryInfo.currentPacketPtr->GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            traceData.accessCategory = static_cast<uint32_t>(accessCategoryIndex);
            traceData.shortFrameRetry = accessCategoryInfo.currentShortFrameRetryCount;
            traceData.longFrameRetry = accessCategoryInfo.currentLongFrameRetryCount;

            assert(sizeof(traceData) == MAC_TX_UNICAST_DATA_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "Tx-DATA-U", traceData);
        }
        else {

            ostringstream msgStream;

            msgStream << "PktId= " << accessCategoryInfo.currentPacketPtr->GetPacketId();
            msgStream << " AC= " << accessCategoryIndex;
            msgStream << " S-Retry= " << accessCategoryInfo.currentShortFrameRetryCount;
            msgStream << " L-Retry= " << accessCategoryInfo.currentLongFrameRetryCount;

            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "Tx-DATA-U", msgStream.str());
        }//if//
    }//if//

    unicastDataFramesSentStatPtr->IncrementCounter();

    if ((accessCategoryInfo.currentShortFrameRetryCount > 0) || (accessCategoryInfo.currentLongFrameRetryCount > 0)) {
        unicastDataFramesResentStatPtr->IncrementCounter();
    }//if//

}//OutputTraceAndStatsForUnicastDataFrameTransmission//


inline
void Dot11Mac::OutputTraceAndStatsForManagementFrameTransmission(const unsigned int accessCategoryIndex) const
{
    const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);
    const CommonFrameHeaderType& header =
                accessCategoryInfo.currentPacketPtr->GetAndReinterpretPayloadData<CommonFrameHeaderType>();
    const unsigned char frameType = header.frameControlField.frameTypeAndSubtype;

    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            Dot11MacTxManagementTraceRecord traceData;

            const PacketIdType& packetId = accessCategoryInfo.currentPacketPtr->GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            traceData.frameType = frameType;

            assert(sizeof(traceData) == DOT11_MAC_TX_MANAGEMENT_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "Tx-Management", traceData);
        }
        else {

            ostringstream msgStream;

            msgStream << "PktId= " << accessCategoryInfo.currentPacketPtr->GetPacketId();

            msgStream << " FrameType= ";

            msgStream << ConvertToDot11FrameTypeName(frameType);

            simEngineInterfacePtr->OutputTrace(
                modelName, interfaceId, "Tx-Management", msgStream.str());

        }//if//
    }//if//

    //stat
    switch (frameType) {
    case BEACON_FRAME_TYPE_CODE: {
        beaconFramesSentStatPtr->IncrementCounter();
        break;
    }
    case ASSOCIATION_REQUEST_FRAME_TYPE_CODE: {
        associationRequestFramesSentStatPtr->IncrementCounter();
        break;
    }
    case ASSOCIATION_RESPONSE_FRAME_TYPE_CODE: {
        associationResponseFramesSentStatPtr->IncrementCounter();
        break;
    }
    case REASSOCIATION_REQUEST_FRAME_TYPE_CODE: {
        reassociationRequestFramesSentStatPtr->IncrementCounter();
        break;
    }
    case REASSOCIATION_RESPONSE_FRAME_TYPE_CODE: {
        reassociationResponseFramesSentStatPtr->IncrementCounter();
        break;
    }
    case DISASSOCIATION_FRAME_TYPE_CODE: {
        disassociationFramesSentStatPtr->IncrementCounter();
        break;
    }
    case AUTHENTICATION_FRAME_TYPE_CODE: {
        authenticationFramesSentStatPtr->IncrementCounter();
        break;
    }
    case BLOCK_ACK_REQUEST_FRAME_TYPE_CODE:
        // Management-esque frame.
        break;

    case QOS_DATA_FRAME_TYPE_CODE:
    case ACK_FRAME_TYPE_CODE:
    case RTS_FRAME_TYPE_CODE:
    case CTS_FRAME_TYPE_CODE:
    default:
        assert(false); abort();
        break;
    }//switch//

}//OutputTraceAndStatsForManagementFrameTransmission//


inline
void Dot11Mac::OutputTraceForAckFrameTransmission() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "Tx-ACK");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "Tx-ACK", "");
        }//if//
    }//if//

    ackFramesSentStatPtr->IncrementCounter();

}//OutputTraceForAckFrameTransmission//


inline
void Dot11Mac::OutputTraceAndStatsForBlockAckFrameTransmission() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "Tx-BlockACK");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "Tx-BlockACK", "");
        }//if//
    }//if//

     blockAckFramesSentStatPtr->IncrementCounter();

}//OutputTraceAndStatsForBlockAckFrameTransmission//



inline
void Dot11Mac::OutputTraceForCtsOrAckTimeout() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {

        const bool lastTransmissionWasAShortFrame =
            ((lastSentFrameWasAn == ShortFrame) || (lastSentFrameWasAn == RequestToSendFrame) ||
             (lastSentFrameWasAn == BlockAckRequestFrame));

        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacCtsOrAckTimeoutTraceRecord traceData;

            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndexForLastSentFrame);

            traceData.accessCategory = static_cast<uint32_t>(accessCategoryIndexForLastSentFrame);
            traceData.windowSlot = accessCategoryInfo.currentContentionWindowSlots;
            traceData.shortFrameRetry = accessCategoryInfo.currentShortFrameRetryCount;
            traceData.longFrameRetry = accessCategoryInfo.currentLongFrameRetryCount;
            traceData.shortFrameOrNot = lastTransmissionWasAShortFrame;

            assert(sizeof(traceData) == MAC_CTSORACK_TIMEOUT_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "Timeout", traceData);
        }
        else {
            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndexForLastSentFrame);
            ostringstream msgStream;

            msgStream << "AC= " << accessCategoryIndexForLastSentFrame;
            msgStream << " Win= " << accessCategoryInfo.currentContentionWindowSlots;
            msgStream << " S-Retry= " << accessCategoryInfo.currentShortFrameRetryCount;
            msgStream << " L-Retry= " << accessCategoryInfo.currentLongFrameRetryCount;

            if (lastTransmissionWasAShortFrame) {
                msgStream << " Len= short";
            }
            else {
                msgStream << " Len= long";
            }//if//

            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "Timeout", msgStream.str());
        }
    }//if//

}//OutputTraceForCtsOrAckTimeout//



inline
void Dot11Mac::OutputTraceAndStatsForPacketRetriesExceeded(const unsigned int accessCategoryIndex) const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            MacPacketRetryExceededTraceRecord traceData;

            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);

            const PacketIdType& packetId = accessCategoryInfo.currentPacketPtr->GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == MAC_PACKET_RETRY_EXCEEDED_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "Drop", traceData);
        }
        else {
            const EdcaAccessCategoryInfo& accessCategoryInfo = accessCategories.at(accessCategoryIndex);
            ostringstream msgStream;

            msgStream << "PktId= " << accessCategoryInfo.currentPacketPtr->GetPacketId();
            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "Drop", msgStream.str());
        }//if//
    }//if//

    droppedPacketsStatPtr->IncrementCounter();

}//OutputTraceAndStatsForPacketRetriesExceeded//

inline
Dot11Mac::~Dot11Mac(){ }


inline
void Dot11Mac::SendAckFrame(
    const MacAddressType& destinationAddress,
    const TransmissionParametersType& receivedFrameTxParameters)
{
    AcknowledgementAkaAckFrameType ackFrame;

    ackFrame.header.frameControlField.frameTypeAndSubtype = ACK_FRAME_TYPE_CODE;
    ackFrame.header.receiverAddress = destinationAddress;
    ackFrame.header.duration = 0;

    unique_ptr<Packet> ackPacketPtr = Packet::CreatePacket(*simEngineInterfacePtr, ackFrame);

    TransmissionParametersType ackTxParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForAckFrame(
        destinationAddress, receivedFrameTxParameters, ackTxParameters);

    ackTxParameters.firstChannelNumber =
        GetFirstChannelNumberForChannelBandwidth(ackTxParameters.channelBandwidthMhz);

    const double transmitPowerDbm = theAdaptiveTxPowerControllerPtr->CurrentTransmitPowerDbm(destinationAddress);

    (*this).macState = CtsOrAckTransmissionState;

    OutputTraceForAckFrameTransmission();

    physicalLayerPtr->TransmitFrame(
        ackPacketPtr,
        ackTxParameters,
        transmitPowerDbm,
        aShortInterframeSpaceTime);

}//SendAckFrame//



inline
void Dot11Mac::SendPacketToNetworkLayer(unique_ptr<Packet>& dataPacketPtr)
{
    const QosDataFrameHeaderType& header =
        dataPacketPtr->GetAndReinterpretPayloadData<QosDataFrameHeaderType>();

    bool wasFound;
    NetworkAddress lastHopAddress;
    theMacAddressResolverPtr->GetNetworkAddressIfAvailable(
        header.transmitterAddress,
        networkLayerPtr->GetSubnetAddress(interfaceIndex),
        wasFound,
        lastHopAddress);

    assert(wasFound || (lastHopAddress == NetworkAddress::invalidAddress));

    const EtherTypeFieldType etherType = NetToHost16(header.linkLayerHeader.etherType);
    dataPacketPtr->DeleteHeader(sizeof(QosDataFrameHeaderType));


    if (etherType == ScenSim::ETHERTYPE_IP ||
        etherType == ScenSim::ETHERTYPE_IPV6 ||
        etherType == ScenSim::ETHERTYPE_IS_NOT_SPECIFIED /*default*/) {

        networkLayerPtr->ReceivePacketFromMac(interfaceIndex, dataPacketPtr, lastHopAddress, etherType);

    }
    else if (macPacketHandlerPtrs.find(etherType) != macPacketHandlerPtrs.end()) {

        macPacketHandlerPtrs[etherType]->ReceivePacketFromMac(
            dataPacketPtr,
            header.transmitterAddress.ConvertToGenericMacAddress());
    }
    else {
        dataPacketPtr = nullptr;
    }//if//


}//SendPacketToNetworkLayer//



inline
void Dot11Mac::ProcessDataFrame(
    const Packet& dataFrame,
    const TransmissionParametersType& receivedFrameTxParameters)
{
    if ((macState == WaitingForCtsState) || (macState == WaitingForAckState)) {
        macState = BusyMediumState;
        (*this).NeverReceivedClearToSendOrAcknowledgementAction();
    }//if//

    const QosDataFrameHeaderType& dataFrameHeader = dataFrame.GetAndReinterpretPayloadData<QosDataFrameHeaderType>();

    bool frameIsInOrder = true;
    vector<unique_ptr<Packet> > bufferedPacketsToSendUp;

    if (!dataFrameHeader.header.receiverAddress.IsABroadcastOrAMulticastAddress()) {

        bool haveAlreadySeenThisFrame = false;

        theIncomingFrameBuffer.ProcessIncomingFrame(
            dataFrame,
            dataFrameHeader.transmitterAddress,
            dataFrameHeader.qosControlField.trafficId,
            dataFrameHeader.sequenceControlField.sequenceNumber,
            frameIsInOrder,
            haveAlreadySeenThisFrame,
            bufferedPacketsToSendUp);

        const unsigned int notUsedAccessCategoryIndex = 0;

        if (!(*this).FrameTransmissionDurationExceedsTransmissionPermissionEndTime(
            sizeof(AcknowledgementAkaAckFrameType),
            receivedFrameTxParameters,
            aShortInterframeSpaceTime)) {

            (*this).SendAckFrame(dataFrameHeader.transmitterAddress, receivedFrameTxParameters);
        }//if//

        if (haveAlreadySeenThisFrame) {
            //duplicated
            dataDuplicatedFramesReceivedStatPtr->IncrementCounter();
        }//if//
    }//if//

    if (frameIsInOrder) {
        unique_ptr<Packet> packetPtr(new Packet(dataFrame));
        (*this).SendPacketToNetworkLayer(packetPtr);
    }//if//

    if (!bufferedPacketsToSendUp.empty()) {
        for(unsigned int i = 0; (i < bufferedPacketsToSendUp.size()); i++) {
            (*this).SendPacketToNetworkLayer(bufferedPacketsToSendUp[i]);
        }//for//
    }//if//

}//ProcessDataFrame//


inline
void Dot11Mac::ProcessBlockAckRequestFrame(
    const BlockAcknowledgementRequestFrameType& blockAckRequestFrame,
    const TransmissionParametersType& receivedFrameTxParameters)
{
    vector<unique_ptr<Packet> > bufferedPacketsToSendUp;
    theIncomingFrameBuffer.ProcessBlockAckRequestFrame(blockAckRequestFrame, bufferedPacketsToSendUp);

    (*this).SendBlockAcknowledgementFrame(
        blockAckRequestFrame.transmitterAddress,
        blockAckRequestFrame.blockAckRequestControl.trafficId,
        receivedFrameTxParameters);

    if (!bufferedPacketsToSendUp.empty()) {
        for(unsigned int i = 0; (i < bufferedPacketsToSendUp.size()); i++) {
            (*this).SendPacketToNetworkLayer(bufferedPacketsToSendUp[i]);
        }//for//
    }//if//

}//ProcessBlockAckRequestFrame//



inline
void Dot11Mac::ProcessManagementFrame(
    const Packet& managementFrame,
    const TransmissionParametersType& receivedFrameTxParameters)
{
    if ((macState == WaitingForCtsState) || (macState == WaitingForAckState)) {
        macState = BusyMediumState;
        (*this).NeverReceivedClearToSendOrAcknowledgementAction();
    }//if//

    const ManagementFrameHeaderType& managementFrameHeader =
        managementFrame.GetAndReinterpretPayloadData<ManagementFrameHeaderType>();

    if (!managementFrameHeader.header.receiverAddress.IsABroadcastOrAMulticastAddress()) {

        bool haveAlreadySeenThisFrame = false;
        vector<unique_ptr<Packet> > bufferedPacketsToSendUp;

        theIncomingFrameBuffer.ProcessIncomingNonDataFrame(
            managementFrameHeader.transmitterAddress,
            maxPacketPriority,
            managementFrameHeader.sequenceControlField.sequenceNumber,
            haveAlreadySeenThisFrame,
            bufferedPacketsToSendUp);

        if (!(*this).FrameTransmissionDurationExceedsTransmissionPermissionEndTime(
                sizeof(AcknowledgementAkaAckFrameType),
                receivedFrameTxParameters,
                aShortInterframeSpaceTime)) {

            (*this).SendAckFrame(managementFrameHeader.transmitterAddress, receivedFrameTxParameters);
        }//if//

        if (haveAlreadySeenThisFrame) {
            // duplicate => drop
            dataDuplicatedFramesReceivedStatPtr->IncrementCounter();
            return;
        }//if//

        if (!bufferedPacketsToSendUp.empty()) {
            for(unsigned int i = 0; (i < bufferedPacketsToSendUp.size()); i++) {
                (*this).SendPacketToNetworkLayer(bufferedPacketsToSendUp[i]);
            }//for//
        }//if//
    }//if//

    switch (operationMode) {
    case ApMode:
        apControllerPtr->ProcessManagementFrame(managementFrame);
        break;
    case InfrastructureMode:
        staControllerPtr->ProcessManagementFrame(managementFrame);
        break;
    case AdhocMode:
        // Ignore
        break;
    default:
        assert(false); abort();
    }//switch//

}//ProcessManagementFrame//


inline
void Dot11Mac::ProcessNullFrame(
    const Packet& nullFrame,
    const TransmissionParametersType& receivedFrameTxParameters)
{
    if ((macState == WaitingForCtsState) || (macState == WaitingForAckState)) {
        macState = BusyMediumState;
        (*this).NeverReceivedClearToSendOrAcknowledgementAction();
    }//if//

    const QosNullFrameHeaderType& nullFrameHeader =
        nullFrame.GetAndReinterpretPayloadData<QosNullFrameHeaderType>();

    bool haveAlreadySeenThisFrame;
    vector<unique_ptr<Packet> > bufferedPacketsToSendUp;

    theIncomingFrameBuffer.ProcessIncomingNonDataFrame(
        nullFrameHeader.transmitterAddress,
        nullFrameHeader.qosControlField.trafficId,
        nullFrameHeader.sequenceControlField.sequenceNumber,
        haveAlreadySeenThisFrame,
        bufferedPacketsToSendUp);

    if (!(*this).FrameTransmissionDurationExceedsTransmissionPermissionEndTime(
            sizeof(AcknowledgementAkaAckFrameType),
            receivedFrameTxParameters,
            aShortInterframeSpaceTime)) {

        (*this).SendAckFrame(nullFrameHeader.transmitterAddress, receivedFrameTxParameters);
    }//if//

    if ((!haveAlreadySeenThisFrame) && (GetOperationMode() == ApMode)) {

        apControllerPtr->ReceiveFramePowerManagementBit(
            nullFrameHeader.transmitterAddress,
            nullFrameHeader.header.frameControlField.powerManagement);

    }//if//

    if (!bufferedPacketsToSendUp.empty()) {
        // This may never actually happen.
        for(unsigned int i = 0; (i < bufferedPacketsToSendUp.size()); i++) {
            (*this).SendPacketToNetworkLayer(bufferedPacketsToSendUp[i]);
        }//for//
    }//if//

}//ProcessNullFrame//



inline
void Dot11Mac::ReceiveFrameFromPhy(
    const Packet& aFrame,
    const TransmissionParametersType& receivedFrameTxParameters)
{
    using std::max;

    //promiscuous scan for congestion monitoring
    if (congestionMonitoringHandlerPtr != nullptr) {
        congestionMonitoringHandlerPtr->ScanAReceivedFrame(aFrame, simEngineInterfacePtr->CurrentTime());
    }

    (*this).lastFrameReceivedWasCorrupt = false;

    const CommonFrameHeaderType& header = aFrame.GetAndReinterpretPayloadData<CommonFrameHeaderType>();

    if (FrameIsForThisNode(aFrame)) {

        OutputTraceAndStatsForFrameReceive(aFrame);

        switch (header.frameControlField.frameTypeAndSubtype) {
        case RTS_FRAME_TYPE_CODE: {
            const RequestToSendFrameType& rtsFrame = aFrame.GetAndReinterpretPayloadData<RequestToSendFrameType>();

            (*this).ProcessRequestToSendFrame(rtsFrame, receivedFrameTxParameters);

            break;
        }
        case CTS_FRAME_TYPE_CODE: {
            const ClearToSendFrameType& ctsFrame = aFrame.GetAndReinterpretPayloadData<ClearToSendFrameType>();

            (*this).ProcessClearToSendFrame(ctsFrame);

            break;
        }
        case NULL_FRAME_TYPE_CODE:
            (*this).ProcessNullFrame(aFrame, receivedFrameTxParameters);

            break;
        case QOS_DATA_FRAME_TYPE_CODE: {

            (*this).ProcessDataFrame(aFrame, receivedFrameTxParameters);

            break;
        }
        case ACK_FRAME_TYPE_CODE: {
            const AcknowledgementAkaAckFrameType& ackFrame =
                aFrame.GetAndReinterpretPayloadData<AcknowledgementAkaAckFrameType>();

            (*this).ProcessAcknowledgementFrame(ackFrame);

            break;
        }
        case BLOCK_ACK_REQUEST_FRAME_TYPE_CODE: {

            const BlockAcknowledgementRequestFrameType& blockAckRequestFrame =
                aFrame.GetAndReinterpretPayloadData<BlockAcknowledgementRequestFrameType>();

            (*this).ProcessBlockAckRequestFrame(blockAckRequestFrame, receivedFrameTxParameters);

            break;
        }
        case BLOCK_ACK_FRAME_TYPE_CODE: {

            const BlockAcknowledgementFrameType& blockAckFrame =
                aFrame.GetAndReinterpretPayloadData<BlockAcknowledgementFrameType>();

            (*this).ProcessBlockAckFrame(blockAckFrame);

            break;
        }
        case POWER_SAVE_POLL_FRAME_TYPE_CODE: {
            const PowerSavePollFrameType& psPollFrame =
                aFrame.GetAndReinterpretPayloadData<PowerSavePollFrameType>();

            (*this).ProcessPowerSavePollFrame(psPollFrame, receivedFrameTxParameters);

            break;
        }
        default:
            if (IsAManagementFrameTypeCode(header.frameControlField.frameTypeAndSubtype)) {
                (*this).ProcessManagementFrame(aFrame, receivedFrameTxParameters);
            } else {
                assert(false); abort();
            }
            break;

        }//switch//

    }
    else {
        // Not for this node.

        // Set Virtual(Software) Carrier Sense aka NAV.

        TimeType endOfDurationTime;

        if (header.frameControlField.frameTypeAndSubtype != POWER_SAVE_POLL_FRAME_TYPE_CODE) {
            endOfDurationTime =
                simEngineInterfacePtr->CurrentTime() + (header.duration * MICRO_SECOND);
        }
        else {
            // Duration field is not a duration but an Assocation ID.
            endOfDurationTime =
                simEngineInterfacePtr->CurrentTime() +
                CalculateNavDurationForAckedDataFrame(receivedFrameTxParameters);
        }//if//

        mediumReservedUntilTimeAkaNAV = max(mediumReservedUntilTimeAkaNAV, endOfDurationTime);

        bool wasInWaitingForCtsOrAckState =
            ((macState == WaitingForCtsState) || (macState == WaitingForAckState));

        macState = BusyMediumState;

        if (wasInWaitingForCtsOrAckState) {
            // Received a packet but wasn't the desired ACK or CTS for this node, must retry.
            (*this).NeverReceivedClearToSendOrAcknowledgementAction();

        }//if//
    }//if//

}//ReceiveFrameFromPhy//


inline
void Dot11Mac::SendBlockAcknowledgementFrame(
    const MacAddressType& destinationAddress,
    const PacketPriorityType& trafficId,
    const TransmissionParametersType& receivedFrameTxParameters)
{
    BlockAcknowledgementFrameType blockAckFrame;
    blockAckFrame.header.frameControlField.frameTypeAndSubtype = BLOCK_ACK_FRAME_TYPE_CODE;
    blockAckFrame.header.receiverAddress = destinationAddress;
    blockAckFrame.header.duration = 0;

    blockAckFrame.transmitterAddress = myMacAddress;
    blockAckFrame.blockAckControl.trafficId = trafficId;

    bool success;

    unsigned short startingSequenceControlNum;

    theIncomingFrameBuffer.GetBlockAckInfo(
        destinationAddress,
        trafficId,
        success,
        startingSequenceControlNum,
        blockAckFrame.blockAckBitmap);

    assert(success);

    blockAckFrame.startingSequenceControl = startingSequenceControlNum;

    unique_ptr<Packet> ackPacketPtr = Packet::CreatePacket(*simEngineInterfacePtr, blockAckFrame);

    TransmissionParametersType ackTxParameters;

    theAdaptiveRateControllerPtr->GetDataRateInfoForAckFrame(
        destinationAddress, receivedFrameTxParameters, ackTxParameters);

    ackTxParameters.firstChannelNumber =
        GetFirstChannelNumberForChannelBandwidth(ackTxParameters.channelBandwidthMhz);

    const double transmitPowerDbm =
        theAdaptiveTxPowerControllerPtr->CurrentTransmitPowerDbm(destinationAddress);

    physicalLayerPtr->TransmitFrame(
        ackPacketPtr,
        ackTxParameters,
        transmitPowerDbm,
        aShortInterframeSpaceTime);

    macState = CtsOrAckTransmissionState;

    OutputTraceAndStatsForBlockAckFrameTransmission();

}//SendBlockAcknowledgementFrame//


inline
void Dot11Mac::ReceiveAggregatedSubframeFromPhy(
    unique_ptr<Packet>& subframePtr,
    const TransmissionParametersType& receivedFrameTxParameters,
    const unsigned int aggregateFrameSubframeIndex,
    const unsigned int numberSubframes)
{
    (*this).lastFrameReceivedWasCorrupt = false;

    if (aggregateFrameSubframeIndex == 0) {
        numSubframesReceivedFromCurrentAggregateFrame = 1;
    }
    else {
        numSubframesReceivedFromCurrentAggregateFrame++;
    }//if//

    RemoveMpduDelimiterAndPaddingFromFrame(*subframePtr);

    const QosDataFrameHeaderType header =
        subframePtr->GetAndReinterpretPayloadData<QosDataFrameHeaderType>();

    const MacAddressType transmitterAddress = header.transmitterAddress;
    const PacketPriorityType trafficId = header.qosControlField.trafficId;

    (*this).currentIncomingAggregateFramesSourceMacAddress = transmitterAddress;
    (*this).currentIncomingAggregateFramesTrafficId = trafficId;

    assert(FrameIsForThisNode(*subframePtr));

    bool frameIsInOrder;
    bool haveAlreadySeenThisFrame;
    vector<unique_ptr<Packet> > bufferedPacketsToSendUp;

    OutputTraceAndStatsForAggregateSubframeReceive(*subframePtr, header);

    theIncomingFrameBuffer.ProcessIncomingSubframe(
        subframePtr,
        transmitterAddress,
        trafficId,
        header.sequenceControlField.sequenceNumber,
        frameIsInOrder,
        haveAlreadySeenThisFrame,
        bufferedPacketsToSendUp);

    if (aggregateFrameSubframeIndex >= (numberSubframes - 1)) {
        (*this).SendBlockAcknowledgementFrame(
            transmitterAddress, trafficId, receivedFrameTxParameters);
    }//if//

    if (frameIsInOrder) {
        (*this).SendPacketToNetworkLayer(subframePtr);
    }
    else if (haveAlreadySeenThisFrame) {
        subframePtr.reset();
    }//if//

    if (!bufferedPacketsToSendUp.empty()) {
        for(unsigned int i = 0; (i < bufferedPacketsToSendUp.size()); i++) {
            (*this).SendPacketToNetworkLayer(bufferedPacketsToSendUp[i]);
        }//for//
    }//if//

}//ReceiveAggregatedSubframeFromPhy//


inline
void Dot11Mac::NotifyThatPhyReceivedCorruptedAggregatedSubframe(
    const TransmissionParametersType& receivedFrameTxParameters,
    const unsigned int aggregateFrameSubframeIndex,
    const unsigned int numberSubframes)
{
    if (aggregateFrameSubframeIndex == 0) {
        numSubframesReceivedFromCurrentAggregateFrame = 0;
        currentIncomingAggregateFramesSourceMacAddress = MacAddressType::invalidMacAddress;
    }//if//

    if ((aggregateFrameSubframeIndex >= (numberSubframes - 1)) &&
        (numSubframesReceivedFromCurrentAggregateFrame > 0)) {

        // Only send if at least one frame was good.

        (*this).SendBlockAcknowledgementFrame(
            currentIncomingAggregateFramesSourceMacAddress,
            currentIncomingAggregateFramesTrafficId,
            receivedFrameTxParameters);
    }//if//

}//NotifyThatPhyReceivedCorruptedAggregatedSubframe//




inline
void Dot11Mac::LookupMacAddressForNeighbor(
    const NodeIdType targetNodeId, bool& wasFound, MacAddressType& macAddress)
{
    switch (GetOperationMode()) {
    case InfrastructureMode:
        staControllerPtr->GetCurrentAccessPointAddress(wasFound, macAddress);
        break;
    case ApMode:
        apControllerPtr->LookupAssociatedNodeMacAddress(targetNodeId, wasFound, macAddress);
        break;
    case AdhocMode:
        macAddress.Clear();
        macAddress.SetLowerBitsWithNodeId(targetNodeId);
        macAddress.SetInterfaceSelectorByte(AdhocModeAddressSelectorByte);
        wasFound = true;
        break;
    default:
        assert(false); abort(); break;
    }//switch//

}//LookupMacAddressForNeighbor//


inline
void Dot11Mac::QueueOutgoingPacket(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopIpAddress,
    const PacketPriorityType priority)
{
    EnqueueResultType enqueueResult;
    unique_ptr<Packet> packetToDrop;

    networkOutputQueuePtr->Insert(
        packetPtr,
        nextHopIpAddress,
        priority,
        enqueueResult,
        packetToDrop);

    if (enqueueResult != ENQUEUE_SUCCESS) {
        packetToDrop = nullptr;
    }//if//

    (*this).NetworkLayerQueueChangeNotification();

}//QueueOutgoingPacket//


//--------------------------------------------------------------------------------------------------

inline
void SimpleMacAddressResolver::GetMacAddress(
    const NetworkAddress& aNetworkAddress,
    const NetworkAddress& networkAddressMask,
    bool& wasFound,
    MacAddressType& resolvedMacAddress)
{
    //assume lower bits of NetworkAddress is node ID.

    wasFound = true;

    assert(MacAddressType::numberMacAddressBytes == 6);

    if (aNetworkAddress.IsTheBroadcastAddress()) {
        resolvedMacAddress = MacAddressType::GetBroadcastAddress();
    }
    else if (aNetworkAddress.IsAMulticastAddress()) {
        if (macPtr->GetIpMulticastAddressToMacAddressMappingIsEnabled()) {
            resolvedMacAddress.SetToAMulticastAddress(aNetworkAddress.GetMulticastGroupNumber());
        }
        else {
            resolvedMacAddress = MacAddressType::GetBroadcastAddress();
        }//if//
    }
    else {
        const NodeIdType nodeId =
            aNetworkAddress.MakeAddressWithZeroedSubnetBits(networkAddressMask).GetRawAddressLow32Bits();

        switch (macPtr->GetOperationMode()) {
        case AdhocMode:
            resolvedMacAddress = MacAddressType(nodeId, Dot11Mac::AdhocModeAddressSelectorByte);
            break;

        case InfrastructureMode:
        case ApMode:
            macPtr->LookupMacAddressForNeighbor(nodeId, wasFound, resolvedMacAddress);
            break;

        default:
            assert(false && "Unhandled Case"); abort();

        }//switch//

    }//if//

}//GetMacAddress//


inline
void SimpleMacAddressResolver::GetNetworkAddressIfAvailable(
    const MacAddressType& macAddress,
    const NetworkAddress& subnetNetworkAddress,
    bool& wasFound,
    NetworkAddress& resolvedNetworkAddress)
{
    wasFound = true;
    resolvedNetworkAddress =
        NetworkAddress(subnetNetworkAddress, NetworkAddress(macAddress.ExtractNodeId()));

}//GetNetworkAddress//


}//namespace//

#endif
