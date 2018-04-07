// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11_MAC_AP_H
#define DOT11_MAC_AP_H

#include "scensim_engine.h"
#include "scensim_netsim.h"

#include "dot11_common.h"
#include "dot11_headers.h"

#include <queue>
#include <map>
#include <string>
#include <iomanip>

namespace Dot11 {

using std::shared_ptr;
using std::unique_ptr;
using std::deque;
using std::map;
using std::unique_ptr;
using std::cout;
using std::cerr;
using std::endl;
using std::hex;
using std::string;
using std::move;

using ScenSim::SimulationEngineInterface;
using ScenSim::SimulationEvent;
using ScenSim::EventRescheduleTicket;
using ScenSim::TimeType;
using ScenSim::MILLI_SECOND;
using ScenSim::ZERO_TIME;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeIdType;
using ScenSim::InterfaceOrInstanceIdType;
using ScenSim::RandomNumberGenerator;
using ScenSim::RandomNumberGeneratorSeedType;
using ScenSim::PacketPriorityType;
using ScenSim::EtherTypeFieldType;
using ScenSim::NetworkAddress;
using ScenSim::HashInputsToMakeSeed;


using ScenSim::ConvertTimeToDoubleSecs;

class Dot11Mac;

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

class Dot11ApManagementController {
public:
    Dot11ApManagementController(
        const shared_ptr<Dot11Mac>& initMacLayerPtr,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& initNodeId,
        const InterfaceOrInstanceIdType& initInterfaceId,
        const RandomNumberGeneratorSeedType& interfaceSeed);

    void ProcessManagementFrame(const Packet& managementFrame);

    void ReceiveFramePowerManagementBit(
        const MacAddressType& sourceAddress,
        const bool framePowerManagementBitIsOn);

    bool IsAnAssociatedStaAddress(const MacAddressType& theMacAddress) const;

    void LookupAssociatedNodeMacAddress(
        const NodeIdType& targetNodeId,
        bool& wasFound,
        MacAddressType& macAddress) const;

    void GetAssociatedStaAddressList(vector<MacAddressType>& associatedStaAddressList) const;

    bool StationIsAsleep(const MacAddressType& staAddress) const;

    void BufferPacketForSleepingStation(
        const MacAddressType& staAddress,
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& destinationNetworkAddress,
        const PacketPriorityType& priority,
        const EtherTypeFieldType etherType,
        const TimeType& timestamp,
        const bool datarateAndTxPowerAreaSpecified,
        const DatarateBitsPerSecType& specifiedDatarateBitsPerSec,
        const double specifiedTxPowerDbm);

    void BufferManagementFrameForSleepingStation(
        const MacAddressType& staAddress,
        unique_ptr<Packet>& framePtr,
        const TimeType& timestamp);

    void GetPowerSaveBufferedPacket(
        const MacAddressType& staAddress,
        bool& wasRetrieved,
        unique_ptr<Packet>& packetToSendPtr,
        unsigned int& retryTxCount,
        PacketPriorityType& priority,
        EtherTypeFieldType& etherType,
        bool& datarateAndTxPowerAreaSpecified,
        DatarateBitsPerSecType& specifiedDatarateBitsPerSec,
        double& specifiedTxPowerDbm);

    void GetBufferedTopPacketInfo(
        const MacAddressType& staAddress,
        unsigned int& frameLengthBytes,
        bool& datarateAndTxPowerAreaSpecified,
        DatarateBitsPerSecType& specifiedDatarateBitsPerSec) const;

private:

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    NodeIdType nodeId;
    InterfaceOrInstanceIdType interfaceId;
    string ssid;

    shared_ptr<Dot11Mac> macLayerPtr;

    AssociationIdType currentAssociationId;

    static const int SEED_HASH = 31843124;
    RandomNumberGenerator aRandomNumberGenerator;

    struct PowerSavePacketBufferElemType {
        unique_ptr<Packet> packetPtr;
        NetworkAddress destinationNetworkAddress;
        PacketPriorityType priority;
        EtherTypeFieldType etherType;
        unsigned int retryTxCount;
        TimeType timestamp;

        bool datarateAndTxPowerAreaSpecified;
        DatarateBitsPerSecType specifiedDatarateBitsPerSec;
        double specifiedTxPowerDbm;

        PowerSavePacketBufferElemType() : packetPtr(nullptr) { }

        PowerSavePacketBufferElemType(
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initDestinationNetworkAddress,
            const PacketPriorityType& initPriority,
            const EtherTypeFieldType& initEtherType,
            const unsigned int initRetryTxCount,
            const TimeType& initTimestamp,
            const bool initDatarateAndTxPowerAreaSpecified,
            const DatarateBitsPerSecType& initSpecifiedDatarateBitsPerSec,
            const double initSpecifiedTxPowerDbm)
        :
            packetPtr(move(initPacketPtr)),
            destinationNetworkAddress(initDestinationNetworkAddress),
            priority(initPriority),
            etherType(initEtherType),
            retryTxCount(initRetryTxCount),
            timestamp(initTimestamp),
            datarateAndTxPowerAreaSpecified(initDatarateAndTxPowerAreaSpecified),
            specifiedDatarateBitsPerSec(initSpecifiedDatarateBitsPerSec),
            specifiedTxPowerDbm(initSpecifiedTxPowerDbm)
        {
        }

        void operator=(PowerSavePacketBufferElemType&& right)
        {
            packetPtr = move(right.packetPtr);
            destinationNetworkAddress = right.destinationNetworkAddress;
            priority = right.priority;
            etherType = right.etherType;
            retryTxCount = right.retryTxCount;
            timestamp = right.timestamp;
            datarateAndTxPowerAreaSpecified = right.datarateAndTxPowerAreaSpecified;
            specifiedDatarateBitsPerSec = right.specifiedDatarateBitsPerSec;
            specifiedTxPowerDbm = right.specifiedTxPowerDbm;
        }

        PowerSavePacketBufferElemType(PowerSavePacketBufferElemType&& right) { (*this) = move(right); }

    };//PowerSavePacketBufferElemType//


    struct AssociatedStaInformationEntry {
        AssociationIdType associationId;
        bool isInPowersaveMode;
        deque<PowerSavePacketBufferElemType> powerSavePacketBuffer;

        AssociatedStaInformationEntry() : isInPowersaveMode(false) { }
    };

    map<MacAddressType, shared_ptr<AssociatedStaInformationEntry> > associatedStaInformation;
    std::bitset<MaxAssociationId+1> associationIdIsBeingUsed;

    void SendReassociationNotification(
        const MacAddressType& staAddress,
        const MacAddressType& apAddress);

    //------------------------------------------------------

    TimeType beaconInterval;

    void BeaconIntervalTimeout();

    class BeaconIntervalTimeoutEvent : public SimulationEvent {
    public:
        BeaconIntervalTimeoutEvent(Dot11ApManagementController* initApControllerPtr) :
            apControllerPtr(initApControllerPtr) { }
        void ExecuteEvent() { apControllerPtr->BeaconIntervalTimeout(); }
    private:
        Dot11ApManagementController* apControllerPtr;
    };

    shared_ptr <BeaconIntervalTimeoutEvent> beaconIntervalTimeoutEventPtr;
    EventRescheduleTicket beaconIntervalRescheduleTicket;


    //------------------------------------------------------
    TimeType authProcessingDelay;
    void ProcessAuthenticationFrame(const MacAddressType& transmitterAddress);

    void SendAuthentication(const MacAddressType& transmitterAddress);

    class SendAuthenticationEvent : public SimulationEvent {
    public:
        SendAuthenticationEvent(
            Dot11ApManagementController* initApControllerPtr,
            const MacAddressType& initReceiverAddress)
            :
            apControllerPtr(initApControllerPtr),
            receiverAddress(initReceiverAddress)
        {}

        void ExecuteEvent() { apControllerPtr->SendAuthentication(receiverAddress); }
    private:
        Dot11ApManagementController* apControllerPtr;
        MacAddressType receiverAddress;
    };

    void SendBeaconFrame();
    void ProcessAssociationRequestFrame(const Packet& aFrame);
    void ProcessReassociationRequestFrame(const Packet& aFrame);
    void AddNewAssociatedStaRecord(const MacAddressType& staAddress);

};//Dot11ApManagementController//


inline
Dot11ApManagementController::Dot11ApManagementController(
    const shared_ptr<Dot11Mac>& initMacLayerPtr,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& initNodeId,
    const InterfaceOrInstanceIdType& initInterfaceId,
    const RandomNumberGeneratorSeedType& interfaceSeed)
    :
    macLayerPtr(initMacLayerPtr),
    simEngineInterfacePtr(initSimulationEngineInterfacePtr),
    beaconInterval(100 * MILLI_SECOND),
    nodeId(initNodeId),
    interfaceId(initInterfaceId),
    ssid(""),
    authProcessingDelay(ZERO_TIME),
    currentAssociationId(0),
    aRandomNumberGenerator(HashInputsToMakeSeed(interfaceSeed, SEED_HASH))
{
    if (theParameterDatabaseReader.ParameterExists(
            "dot11-beacon-transmit-interval", nodeId, interfaceId)) {
        beaconInterval = theParameterDatabaseReader.ReadTime(
            "dot11-beacon-transmit-interval", nodeId, interfaceId);

        if (beaconInterval < ZERO_TIME) {
            cerr << "Invalid beacon interval: " << ConvertTimeToDoubleSecs(beaconInterval) << endl;
            exit(1);
        }
    }//if//

    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
    const TimeType beaconStartJitter =
        static_cast<TimeType>(beaconInterval * aRandomNumberGenerator.GenerateRandomDouble());

    beaconIntervalTimeoutEventPtr.reset(new BeaconIntervalTimeoutEvent(this));

    simEngineInterfacePtr->ScheduleEvent(
        beaconIntervalTimeoutEventPtr,
        currentTime + beaconStartJitter,
        beaconIntervalRescheduleTicket);

    //read ssid
    if (theParameterDatabaseReader.ParameterExists("dot11-access-point-ssid", nodeId, interfaceId)) {
        ssid = theParameterDatabaseReader.ReadString("dot11-access-point-ssid", nodeId, interfaceId);

        if (ssid.length() > SSID_LENGTH) {
            cerr << "Error: SSID length must be " << SSID_LENGTH << " or under: " << ssid << endl;
            exit(1);
        }//if//
    }//if//

    //read authentication processing delay
    if (theParameterDatabaseReader.ParameterExists("dot11-access-point-auth-processing-delay", nodeId, interfaceId)) {

        authProcessingDelay =
            theParameterDatabaseReader.ReadTime("dot11-access-point-auth-processing-delay", nodeId, interfaceId);

    }//if//

}//Dot11ApManagementController//



inline
void Dot11ApManagementController::ProcessAuthenticationFrame(const MacAddressType& transmitterAddress)
{
    shared_ptr<SendAuthenticationEvent> sendAuthenticationEventPtr(new SendAuthenticationEvent(this, transmitterAddress));
    simEngineInterfacePtr->ScheduleEvent(
        sendAuthenticationEventPtr,
        simEngineInterfacePtr->CurrentTime() + authProcessingDelay);

}//ProcessAuthenticationFrame//


inline
bool Dot11ApManagementController::IsAnAssociatedStaAddress(const MacAddressType& theMacAddress) const
{
    typedef map<MacAddressType, shared_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;

    IterType staIterator = associatedStaInformation.find(theMacAddress);

    if (staIterator == associatedStaInformation.end()) {
        return false;
    } else {
        return true;
    }//if//

}//IsAnAssociatedStaAddress//



inline
void Dot11ApManagementController::LookupAssociatedNodeMacAddress(
    const NodeIdType& targetNodeId,
    bool& wasFound,
    MacAddressType& macAddress) const
{
    // This code assumes that the AP will not associate with two interfaces on the same node.
    // To provide this functionality would require ARP/Neighbor Discovery.

    wasFound = false;
    MacAddressType searchAddress(targetNodeId, 0);

    typedef map<MacAddressType, shared_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;

    IterType staIterator = associatedStaInformation.lower_bound(searchAddress);

    if (staIterator == associatedStaInformation.end()) {
        return;
    }//if//

    if (staIterator->first.ExtractNodeId() == targetNodeId) {
        wasFound = true;
        macAddress = staIterator->first;
    }//if//

}//LookupAssociatedNodeMacAddress//



inline
bool Dot11ApManagementController::StationIsAsleep(const MacAddressType& staAddress) const
{
    typedef map<MacAddressType, shared_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;
    IterType staIterator = associatedStaInformation.find(staAddress);
    if (staIterator == associatedStaInformation.end()) {
        return false;
    }//if//

    const AssociatedStaInformationEntry& staInfo = *(staIterator->second);
    return (staInfo.isInPowersaveMode);

}//StationIsAsleep//


inline
void Dot11ApManagementController::BufferPacketForSleepingStation(
    const MacAddressType& staAddress,
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& destinationNetworkAddress,
    const PacketPriorityType& priority,
    const EtherTypeFieldType etherType,
    const TimeType& timestamp,
    const bool datarateAndTxPowerAreaSpecified,
    const DatarateBitsPerSecType& specifiedDatarateBitsPerSec,
    const double specifiedTxPowerDbm)
{
    AssociatedStaInformationEntry& staInfo = *associatedStaInformation[staAddress];

    assert(staInfo.isInPowersaveMode);

    staInfo.powerSavePacketBuffer.push_back(move(
        PowerSavePacketBufferElemType(
            packetPtr,
            destinationNetworkAddress,
            priority,
            etherType,
            0,
            timestamp,
            datarateAndTxPowerAreaSpecified,
            specifiedDatarateBitsPerSec,
            specifiedTxPowerDbm)));

}//BufferPacketForSleepingStation//



inline
void Dot11ApManagementController::BufferManagementFrameForSleepingStation(
    const MacAddressType& staAddress,
    unique_ptr<Packet>& framePtr,
    const TimeType& timestamp)
{
    AssociatedStaInformationEntry& staInfo = *associatedStaInformation[staAddress];

    assert(staInfo.isInPowersaveMode);
    staInfo.powerSavePacketBuffer.push_back(move(
        PowerSavePacketBufferElemType(
            framePtr,
            NetworkAddress::invalidAddress,
            ScenSim::InvalidPacketPriority,
            ScenSim::ETHERTYPE_IS_NOT_SPECIFIED,
            0,
            timestamp,
            false,/*datarateAndTxPowerAreaSpecified*/
            0,/*specifiedDatarateBitsPerSec*/
            0/*specifiedTxPowerDbm*/)));

}//BufferPacketForSleepingStation//


inline
void Dot11ApManagementController::GetAssociatedStaAddressList(vector<MacAddressType>& associatedStaAddressList) const
{
    associatedStaAddressList.clear();

    typedef map<MacAddressType, shared_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;

    for(IterType iter = associatedStaInformation.begin(); iter != associatedStaInformation.end(); ++iter) {
        const MacAddressType& associatedStaAddress = iter->first;
        associatedStaAddressList.push_back(associatedStaAddress);
    }//for//

}//GetAssociatedStaAddressList//



inline
void Dot11ApManagementController::GetPowerSaveBufferedPacket(
    const MacAddressType& staAddress,
    bool& wasRetrieved,
    unique_ptr<Packet>& packetPtr,
    unsigned int& retryTxCount,
    PacketPriorityType& priority,
    EtherTypeFieldType& etherType,
    bool& datarateAndTxPowerAreaSpecified,
    DatarateBitsPerSecType& specifiedDatarateBitsPerSec,
    double& specifiedTxPowerDbm)
{
    AssociatedStaInformationEntry& staInfo = *associatedStaInformation[staAddress];
    PowerSavePacketBufferElemType& elem = staInfo.powerSavePacketBuffer.front();

    packetPtr = move(elem.packetPtr);
    priority = elem.priority;
    etherType = elem.etherType;
    retryTxCount = elem.retryTxCount;
    staInfo.powerSavePacketBuffer.pop_front();
    datarateAndTxPowerAreaSpecified = elem.datarateAndTxPowerAreaSpecified;
    specifiedDatarateBitsPerSec = elem.specifiedDatarateBitsPerSec;
    specifiedTxPowerDbm = elem.specifiedTxPowerDbm;

}//GetPowerSaveBufferedPacket//


inline
void Dot11ApManagementController::GetBufferedTopPacketInfo(
    const MacAddressType& staAddress,
    unsigned int& frameLengthBytes,
    bool& datarateAndTxPowerAreaSpecified,
    DatarateBitsPerSecType& specifiedDatarateBitsPerSec) const
{
    typedef map<MacAddressType, shared_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;

    IterType iter = associatedStaInformation.find(staAddress);

    assert(iter != associatedStaInformation.end());

    const AssociatedStaInformationEntry& staInfo = *(*iter).second;
    const PowerSavePacketBufferElemType& elem = staInfo.powerSavePacketBuffer.front();

    frameLengthBytes = elem.packetPtr->LengthBytes();
    datarateAndTxPowerAreaSpecified = elem.datarateAndTxPowerAreaSpecified;
    specifiedDatarateBitsPerSec = elem.specifiedDatarateBitsPerSec;
}//GetBufferedTopPacketInfo//


inline
void Dot11ApManagementController::BeaconIntervalTimeout()
{
    (*this).SendBeaconFrame();

    simEngineInterfacePtr->ScheduleEvent(
        beaconIntervalTimeoutEventPtr,
        simEngineInterfacePtr->CurrentTime() + beaconInterval,
        beaconIntervalRescheduleTicket);

}//BeaconIntervalTimeout//


inline
void Dot11ApManagementController::AddNewAssociatedStaRecord(const MacAddressType& staAddress)
{
    shared_ptr<AssociatedStaInformationEntry>
        newStaInfoEntryPtr(new AssociatedStaInformationEntry());

    if (currentAssociationId < MaxAssociationId) {
        (*this).currentAssociationId++;
        newStaInfoEntryPtr->associationId = currentAssociationId;
    }
    else {
        bool wasFound = false;
        for (AssociationIdType i = 1; (i <= MaxAssociationId); i++) {
            if (!associationIdIsBeingUsed[i]) {
                wasFound = true;
                newStaInfoEntryPtr->associationId = i;
                break;
            }//if//
        }//for//
        assert((wasFound) && "Too Many STAs trying to associate with AP");
    }//if//

    (*this).associationIdIsBeingUsed.set(newStaInfoEntryPtr->associationId);

    associatedStaInformation.insert(
        make_pair(staAddress, newStaInfoEntryPtr));

}//AddNewAssociatedStaRecord//

}//namespace//

#endif
