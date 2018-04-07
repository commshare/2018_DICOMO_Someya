// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_QUEUES_H
#define SCENSIM_QUEUES_H

#include <queue>
#include "scensim_parmio.h"
#include "scensim_engine.h"
#include "scensim_netaddress.h"
#include "scensim_packet.h"

namespace ScenSim {

typedef unsigned short EtherTypeFieldType;

const EtherTypeFieldType ETHERTYPE_IS_NOT_SPECIFIED = 65535; //0xFFFF
const EtherTypeFieldType ETHERTYPE_IP = 2048; //0x0800
const EtherTypeFieldType ETHERTYPE_ARP = 2054; //0x0806
const EtherTypeFieldType ETHERTYPE_VLAN = 33024; //0x8100
const EtherTypeFieldType ETHERTYPE_IPV6 = 34525; //0x86DD
const EtherTypeFieldType ETHERTYPE_WSMP = 35036; //0x88DC
const EtherTypeFieldType ETHERTYPE_GEONET = 1799; //0x0707


enum EnqueueResultType {
    ENQUEUE_SUCCESS,
    ENQUEUE_FAILURE_BY_MAX_PACKETS,
    ENQUEUE_FAILURE_BY_MAX_BYTES,
    ENQUEUE_FAILURE_BY_OUT_OF_SCOPE
};


class OutputQueuePriorityMapper {
public:
    virtual PacketPriorityType MaxMappedPriority() const = 0;
    virtual PacketPriorityType MapIpToMacPriority(const PacketPriorityType& ipPriority) const = 0;
};


//--------------------------------------

class InterfaceOutputQueue {
public:
    virtual ~InterfaceOutputQueue() { }

    virtual bool InsertWithFullPacketInformationModeIsOn() const { return false; }

    virtual void Insert(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType priority,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDropPtr,
        const EtherTypeFieldType etherType = ETHERTYPE_IS_NOT_SPECIFIED) = 0;

    virtual void InsertWithFullPacketInformation(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const NetworkAddress& sourceAddress,
        const unsigned short int sourcePort,
        const NetworkAddress& destinationAddress,
        const unsigned short int destinationPort,
        const unsigned char protocolCode,
        const PacketPriorityType priority,
        const unsigned short int ipv6FlowLabel,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDropPtr) { assert(false); abort(); }

    virtual PacketPriorityType MaxPossiblePacketPriority() const { return MAX_AVAILABLE_PACKET_PRIORITY; }

    virtual bool IsEmpty() const = 0;

    virtual void DequeuePacket(
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        PacketPriorityType& priority,
        EtherTypeFieldType& etherType) = 0;

};//InterfaceOutputQueue//


//--------------------------------------------------------------------------------------------------

class FifoInterfaceOutputQueue : public InterfaceOutputQueue {
public:
    FifoInterfaceOutputQueue(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceIdType& interfaceId,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const string& parameterNamePrefix = "interface-output-queue-");

    bool IsEmpty() const { return theQueue.empty(); }

    void Insert(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType priority,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDrop,
        const EtherTypeFieldType etherType = ETHERTYPE_IS_NOT_SPECIFIED);

    void DequeuePacket(
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        PacketPriorityType& priority,
        EtherTypeFieldType& etherType);

private:

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;

    struct OutputQueueRecord {
        OutputQueueRecord(
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initNextHopAddress,
            const PacketPriorityType& initTypeOfService,
            const EtherTypeFieldType initEtherType)
            :
            packetPtr(move(initPacketPtr)),
            nextHopAddress(initNextHopAddress),
            trafficClass(initTypeOfService),
            etherType(initEtherType)
        {
        }

        OutputQueueRecord(OutputQueueRecord&& right) :
            packetPtr(move(right.packetPtr)),
            nextHopAddress(right.nextHopAddress),
            trafficClass(right.trafficClass),
            etherType(right.etherType) {}

        unique_ptr<Packet> packetPtr;
        NetworkAddress nextHopAddress;
        PacketPriorityType trafficClass;
        EtherTypeFieldType etherType;
    };

    std::queue<OutputQueueRecord> theQueue;

    unsigned int maxNumberPackets;
    unsigned int maxNumberBytes;
    size_t currentNumberBytes;

};//FifoInterfaceOutputQueue//


inline
FifoInterfaceOutputQueue::FifoInterfaceOutputQueue(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceIdType& interfaceId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const string& parameterNamePrefix)
    :
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    maxNumberPackets(0),
    maxNumberBytes(0),
    currentNumberBytes(0)
{
    const NodeIdType nodeId = simEngineInterfacePtr->GetNodeId();

    if (theParameterDatabaseReader.ParameterExists((parameterNamePrefix + "max-packets"),
        nodeId, interfaceId)) {

        maxNumberPackets =
            theParameterDatabaseReader.ReadNonNegativeInt((parameterNamePrefix + "max-packets"), nodeId, interfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists((parameterNamePrefix + "max-bytes"), nodeId, interfaceId)) {
        maxNumberBytes =
            theParameterDatabaseReader.ReadNonNegativeInt((parameterNamePrefix + "max-bytes"), nodeId, interfaceId);
    }//if//

}//FifoInterfaceOutputQueue()//


inline
void FifoInterfaceOutputQueue::Insert(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriorityType priority,
    EnqueueResultType& enqueueResult,
    unique_ptr<Packet>& packetToDrop,
    const EtherTypeFieldType etherType)
{

    assert((maxNumberPackets == 0) || (theQueue.size() <= maxNumberPackets));
    assert((maxNumberBytes == 0) || (currentNumberBytes <= maxNumberBytes));

    const size_t packetSizeBytes = packetPtr->LengthBytes();

    if ((maxNumberPackets != 0) && (theQueue.size() == maxNumberPackets)) {
        enqueueResult = ENQUEUE_FAILURE_BY_MAX_PACKETS;
        packetToDrop = move(packetPtr);
    }
    else if ((maxNumberBytes != 0) && ((currentNumberBytes + packetSizeBytes) > maxNumberBytes)) {
        enqueueResult = ENQUEUE_FAILURE_BY_MAX_BYTES;
        packetToDrop = move(packetPtr);
    }
    else {
        enqueueResult = ENQUEUE_SUCCESS;
        currentNumberBytes += packetSizeBytes;
        theQueue.push(move(OutputQueueRecord(packetPtr, nextHopAddress, priority, etherType)));
    }//if//

}//Insert//


inline
void FifoInterfaceOutputQueue::DequeuePacket(
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    PacketPriorityType& priority,
    EtherTypeFieldType& etherType)
{
    assert(!theQueue.empty());

    OutputQueueRecord& queueRecord = theQueue.front();

    packetPtr = move(queueRecord.packetPtr);
    nextHopAddress = queueRecord.nextHopAddress;
    priority = queueRecord.trafficClass;
    etherType = queueRecord.etherType;
    theQueue.pop();

    currentNumberBytes -= packetPtr->LengthBytes();

}//DequeuePacket//




//--------------------------------------------------------------------------------------------------

class AbstractOutputQueueWithPrioritySubqueues: public InterfaceOutputQueue {
public:
    virtual void Insert(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType priority,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDrop,
        const EtherTypeFieldType etherType = ETHERTYPE_IS_NOT_SPECIFIED) override = 0;

    virtual void RequeueAtFront(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType priority,
        const EtherTypeFieldType etherType,
        const TimeType& queueInsertionTime,
        const unsigned int retryTxCount,
        const unsigned short int sequenceNumber) = 0;

    virtual PacketPriorityType MaxPossiblePacketPriority() const override = 0;

    virtual bool IsEmpty() const override = 0;

    virtual unsigned int NumberPackets() const = 0;

    virtual unsigned long long int NumberPacketBytes() const = 0;

    virtual bool HasPacketWithPriority(const PacketPriorityType priority) const = 0;

    virtual const Packet& TopPacket(const PacketPriorityType priority) const = 0;
    virtual const NetworkAddress& NextHopAddressForTopPacket(const PacketPriorityType priority) const = 0;
    //Jay virtual const TimeType& InsertionTimeForTopPacket(const PacketPriorityType priority) const = 0;


    virtual void DequeuePacket(
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        PacketPriorityType& priority,
        EtherTypeFieldType& etherType) override = 0;


    virtual void DequeuePacketWithPriority(
        const PacketPriorityType& priority,
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        EtherTypeFieldType& etherType,
        TimeType& queueInsertionTime,
        bool& isANewPacket,
        unsigned int& retryTxCount,
        unsigned short int& sequenceNumber) = 0;

    // Allows extracting frames to a destination in non-FIFO order.  For frame aggregation.

    virtual void EnableNextHopSpecificDequeues() = 0;

    virtual bool NextHopSpecificDequeueIsEnabled() const = 0;

    virtual bool HasPacketWithPriorityAndNextHop(
        const PacketPriorityType priority,
        const NetworkAddress& nextHopAddress) const = 0;

    // These methods still work in FIFO mode, but the next hop address must match
    // top of queue (assert).

    virtual const Packet& GetNextPacketWithPriorityAndNextHop(
        const PacketPriorityType priority,
        const NetworkAddress& nextHopAddress) const = 0;

    virtual bool NextPacketIsARetry(
        const PacketPriorityType priority,
        const NetworkAddress& nextHopAddress) const = 0;

    virtual void DequeuePacketWithPriorityAndNextHop(
        const PacketPriorityType& priority,
        const NetworkAddress& nextHopAddress,
        unique_ptr<Packet>& packetPtr,
        EtherTypeFieldType& etherType,
        TimeType& queueInsertionTime,
        bool& isANewPacket,
        unsigned int& retryTxCount,
        unsigned short int& sequenceNumber) = 0;

    // Round robin support (Assuming order based on address).

    virtual NetworkAddress GetNetworkAddressOfNextActiveStationAfter(
        const PacketPriorityType& priority,
        const NetworkAddress& address) const = 0;


    // For Getting rid of expired packets.

    struct ExpiredPacketInfoType {
        unique_ptr<Packet> packetPtr;
        NetworkAddress nextHopAddress;

        ExpiredPacketInfoType(
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initNextHopAddress)
        :
            packetPtr(move(initPacketPtr)),
            nextHopAddress(initNextHopAddress)
        {}

        void operator=(ExpiredPacketInfoType&& right)
        {
            packetPtr = move(right.packetPtr);
            nextHopAddress = right.nextHopAddress;
        }

        ExpiredPacketInfoType(ExpiredPacketInfoType&& right) { (*this) = move(right); }
    };

    virtual void DequeueLifetimeExpiredPackets(
        const PacketPriorityType& priority,
        const TimeType& packetsInsertedBeforeTime,
        vector<ExpiredPacketInfoType>& packetList) = 0;

    // For delayed Block ACKed packets.

    virtual void DeletePacketsBySequenceNumber(
        const PacketPriorityType& priority,
        const NetworkAddress& nextHopAddress,
        const vector<unsigned short int>& sequenceNumberList) = 0;

};//AbstractOutputQueueWithPrioritySubqueues//




//--------------------------------------------------------------------------------------------------

class OutputQueueWithPrioritySubqueues: public AbstractOutputQueueWithPrioritySubqueues {
public:

    OutputQueueWithPrioritySubqueues(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceIdType& interfaceId,
        const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
        const PacketPriorityType& maximumPriority,
        const shared_ptr<OutputQueuePriorityMapper>& priorityMapperPtr);

    virtual void Insert(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType priority,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDrop,
        const EtherTypeFieldType etherType = ETHERTYPE_IS_NOT_SPECIFIED) override;

    void RequeueAtFront(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType priority,
        const EtherTypeFieldType etherType,
        const TimeType& queueInsertionTime,
        const unsigned int retryTxCount,
        const unsigned short int sequenceNumber) override;

    virtual PacketPriorityType MaxPossiblePacketPriority() const override { return (maximumPriority); }

    virtual bool IsEmpty() const override { return (totalPackets == 0); }

    unsigned int NumberPackets() const override { return (totalPackets); }

    unsigned long long int NumberPacketBytes() const override { return (totalPacketBytes); }

    bool HasPacketWithPriority(const PacketPriorityType priority) const override
    {
        assert(priority <= maximumPriority);
        return (!outputSubqueues.at(priority).fifoQueue.empty());
    }

    const Packet& TopPacket(const PacketPriorityType priority) const override;
    const NetworkAddress& NextHopAddressForTopPacket(const PacketPriorityType priority) const override;

    virtual NetworkAddress GetNetworkAddressOfNextActiveStationAfter(
        const PacketPriorityType& priority,
        const NetworkAddress& address) const override;

    virtual void DequeuePacket(
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        PacketPriorityType& priority,
        EtherTypeFieldType& etherType) override;

    virtual void DequeuePacketWithPriority(
        const PacketPriorityType& priority,
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        EtherTypeFieldType& etherType,
        TimeType& queueInsertionTime,
        bool& isANewPacket,
        unsigned int& retryTxCount,
        unsigned short int& sequenceNumber) override;

    // Allows extracting frames to a destination in non-FIFO order.  For frame aggregation.

    virtual void EnableNextHopSpecificDequeues() override {
        assert(IsEmpty());
        (*this).nextHopSpecificQueuesAreEnabled = true;
    }

    virtual bool NextHopSpecificDequeueIsEnabled() const  override { return (nextHopSpecificQueuesAreEnabled); }

    virtual bool HasPacketWithPriorityAndNextHop(
        const PacketPriorityType priority,
        const NetworkAddress& nextHopAddress) const override;

    // These methods still work in FIFO mode, but the next hop address must match
    // top of queue (assert).

    virtual const Packet& GetNextPacketWithPriorityAndNextHop(
        const PacketPriorityType priority,
        const NetworkAddress& nextHopAddress) const override;

    virtual bool NextPacketIsARetry(
        const PacketPriorityType priority,
        const NetworkAddress& nextHopAddress) const override;

    virtual void DequeuePacketWithPriorityAndNextHop(
        const PacketPriorityType& priority,
        const NetworkAddress& nextHopAddress,
        unique_ptr<Packet>& packetPtr,
        EtherTypeFieldType& etherType,
        TimeType& queueInsertionTime,
        bool& isANewPacket,
        unsigned int& retryTxCount,
        unsigned short int& sequenceNumber) override;

    virtual void DequeueLifetimeExpiredPackets(
        const PacketPriorityType& priority,
        const TimeType& packetsInsertedBeforeTime,
        vector<ExpiredPacketInfoType>& packetList) override;

    virtual void DeletePacketsBySequenceNumber(
        const PacketPriorityType& priority,
        const NetworkAddress& nextHopAddress,
        const vector<unsigned short int>& sequenceNumberList) override;

private:

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;

    shared_ptr<OutputQueuePriorityMapper> priorityMapperPtr;

    struct OutputQueueRecordType {
        OutputQueueRecordType(
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initNextHopAddress,
            const EtherTypeFieldType initEtherType,
            const TimeType& initQueueInsertionTime)
            :
            packetPtr(move(initPacketPtr)),
            nextHopAddress(initNextHopAddress),
            etherType(initEtherType),
            queueInsertionTime(initQueueInsertionTime),
            retryTxCount(0),
            sequenceNumber(0),
            isARequeuedPacket(false)
        {}

        OutputQueueRecordType(
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initNextHopAddress,
            const EtherTypeFieldType initEtherType,
            const TimeType& initQueueInsertionTime,
            const unsigned int initRetryTxCount,
            const unsigned short int initSequenceNumber)
            :
            packetPtr(move(initPacketPtr)),
            nextHopAddress(initNextHopAddress),
            etherType(initEtherType),
            queueInsertionTime(initQueueInsertionTime),
            retryTxCount(initRetryTxCount),
            sequenceNumber(initSequenceNumber),
            isARequeuedPacket(true)
        {}

        void operator=(OutputQueueRecordType&& right) {
            assert(this != &right);
            packetPtr = move(right.packetPtr);
            nextHopAddress = right.nextHopAddress;
            etherType = right.etherType;
            queueInsertionTime = right.queueInsertionTime;
            retryTxCount = right.retryTxCount;
            sequenceNumber = right.sequenceNumber;
            isARequeuedPacket = right.isARequeuedPacket;
        }

        OutputQueueRecordType(OutputQueueRecordType&& right)  { (*this) = move(right); }

        unique_ptr<Packet> packetPtr;
        TimeType queueInsertionTime;
        NetworkAddress nextHopAddress;
        unsigned int retryTxCount;
        EtherTypeFieldType etherType;
        unsigned short int sequenceNumber;
        bool isARequeuedPacket;

    };//OutputQueueRecordType//


    struct DestinationSpecificInfoType {
        std::deque<OutputQueueRecordType*> aQueue;
        TimeType transitionedToEmptyTime;

        DestinationSpecificInfoType(): transitionedToEmptyTime(INFINITE_TIME) { }
    };


    struct OutputSubqueueInfoType {

        unsigned long long int currentNumberBytes;

        // Records are "owned" by fifoQueue.  Raw pointers instead of shared_ptr for speed.

        std::deque<unique_ptr<OutputQueueRecordType> > fifoQueue;

        map<NetworkAddress, DestinationSpecificInfoType> destinationSpecificInfos;

        OutputSubqueueInfoType() : currentNumberBytes(0) { }

        void operator=(OutputSubqueueInfoType&& right) {
            currentNumberBytes = right.currentNumberBytes;
            fifoQueue = move(right.fifoQueue);
            destinationSpecificInfos = move(right.destinationSpecificInfos);
        }

        OutputSubqueueInfoType(OutputSubqueueInfoType&& right) { (*this) = move(right); }

    };//OutputSubqueueInfoType//


    bool nextHopSpecificQueuesAreEnabled;

    PacketPriorityType maximumPriority;

    unsigned int totalPackets;
    unsigned long long int totalPacketBytes;

    unsigned int subqueueMaxPackets;
    unsigned int subqueueMaxBytes;

    vector<OutputSubqueueInfoType> outputSubqueues;

    // Disable:

    OutputQueueWithPrioritySubqueues(const OutputQueueWithPrioritySubqueues&);
    void operator=(const OutputQueueWithPrioritySubqueues&);

};//OutputQueueWithPrioritySubqueues//

//----------------------------------------------------------

inline
OutputQueueWithPrioritySubqueues::OutputQueueWithPrioritySubqueues(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceIdType& interfaceId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const PacketPriorityType& initMaximumPriority,
    const shared_ptr<OutputQueuePriorityMapper>& initPriorityMapperPtr)
    :
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    priorityMapperPtr(initPriorityMapperPtr),
    maximumPriority(initMaximumPriority),
    nextHopSpecificQueuesAreEnabled(false),
    totalPackets(0),
    totalPacketBytes(0),
    outputSubqueues(initMaximumPriority + 1),
    subqueueMaxPackets(0),
    subqueueMaxBytes(0)
{
    const NodeIdType nodeId = simEngineInterfacePtr->GetNodeId();

    if (theParameterDatabaseReader.ParameterExists("interface-output-queue-max-packets-per-subq", nodeId, interfaceId)){
        subqueueMaxPackets =
            theParameterDatabaseReader.ReadNonNegativeInt("interface-output-queue-max-packets-per-subq", nodeId, interfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("interface-output-queue-max-bytes-per-subq", nodeId, interfaceId)){
        subqueueMaxBytes =
            theParameterDatabaseReader.ReadNonNegativeInt("interface-output-queue-max-bytes-per-subq", nodeId, interfaceId);
    }//if//

}//OutputQueueWithPrioritySubqueues()/


inline
const Packet& OutputQueueWithPrioritySubqueues::TopPacket(const PacketPriorityType priority) const
{
    assert(priority < outputSubqueues.size());
    assert(!outputSubqueues[priority].fifoQueue.empty());

    return (*outputSubqueues[priority].fifoQueue.front()->packetPtr);
}


inline
const NetworkAddress& OutputQueueWithPrioritySubqueues::NextHopAddressForTopPacket(
    const PacketPriorityType priority) const
{
    assert(priority < outputSubqueues.size());
    assert(!outputSubqueues[priority].fifoQueue.empty());

    return (outputSubqueues[priority].fifoQueue.front()->nextHopAddress);
}

inline
NetworkAddress OutputQueueWithPrioritySubqueues::GetNetworkAddressOfNextActiveStationAfter(
    const PacketPriorityType& priority,
    const NetworkAddress& address) const
{
    assert(priority < outputSubqueues.size());
    const OutputSubqueueInfoType& outputSubqueue = outputSubqueues.at(priority);

    assert(nextHopSpecificQueuesAreEnabled);
    assert(!outputSubqueue.destinationSpecificInfos.empty());

    if (outputSubqueue.destinationSpecificInfos.size() == 1) {

        // Only one station. Next station is the station.

        assert(!outputSubqueue.destinationSpecificInfos.begin()->second.aQueue.empty());

        return(outputSubqueue.destinationSpecificInfos.begin()->first);
    }//if//

    typedef map<NetworkAddress, DestinationSpecificInfoType>::const_iterator IterType;

    // upper_bound() returns next address or the end().

    const IterType startIter = outputSubqueue.destinationSpecificInfos.upper_bound(address);

    IterType iter = startIter;

    // Search for non-empty queue in order (wrapped): forloop only stops infinite loop.

    while (true) {
        if (iter == outputSubqueue.destinationSpecificInfos.end()) {
            iter = outputSubqueue.destinationSpecificInfos.begin();
        }//if//

        if (!iter->second.aQueue.empty()) {
            return (iter->first);
        }//if//

        ++iter;

        assert((iter != startIter) && "No non-empty queues were found!");

    }//while//

    assert(false); abort();  return (NetworkAddress());

}//GetNetworkAddressOfNextActiveStationAfter//


inline
bool OutputQueueWithPrioritySubqueues::HasPacketWithPriorityAndNextHop(
    const PacketPriorityType priority,
    const NetworkAddress& nextHopAddress) const
{
    assert(nextHopSpecificQueuesAreEnabled);
    assert(priority < outputSubqueues.size());
    const OutputSubqueueInfoType& outputSubqueue = outputSubqueues.at(priority);

    typedef map<NetworkAddress, DestinationSpecificInfoType>::const_iterator IterType;
    const IterType iter = outputSubqueue.destinationSpecificInfos.find(nextHopAddress);

    if (iter == outputSubqueue.destinationSpecificInfos.end()) {
        return false;
    }//if//

    return (!iter->second.aQueue.empty());

}//HasPacketWithPriorityAndNextHop//



inline
const Packet& OutputQueueWithPrioritySubqueues::GetNextPacketWithPriorityAndNextHop(
    const PacketPriorityType priority,
    const NetworkAddress& nextHopAddress) const
{
    assert(priority < outputSubqueues.size());
    const OutputSubqueueInfoType& outputSubqueue = outputSubqueues.at(priority);

    if (!nextHopSpecificQueuesAreEnabled) {
        assert((outputSubqueue.fifoQueue.front()->nextHopAddress == nextHopAddress) &&
               "Access must be strictly FIFO without destination specific queues.");

        return (*outputSubqueue.fifoQueue.front()->packetPtr);
    }//if//

    typedef map<NetworkAddress, DestinationSpecificInfoType>::const_iterator IterType;

    const IterType iter = outputSubqueue.destinationSpecificInfos.find(nextHopAddress);
    assert(iter != outputSubqueue.destinationSpecificInfos.end());

    return (*iter->second.aQueue.front()->packetPtr);

}//GetNextPacketWithPriorityAndNextHop//



inline
bool OutputQueueWithPrioritySubqueues::NextPacketIsARetry(
    const PacketPriorityType priority,
    const NetworkAddress& nextHopAddress) const
{
    assert(priority < outputSubqueues.size());

    const OutputSubqueueInfoType& outputSubqueue = outputSubqueues.at(priority);

    if (!nextHopSpecificQueuesAreEnabled) {
        assert((outputSubqueue.fifoQueue.front()->nextHopAddress == nextHopAddress) &&
               "Access must be strictly FIFO without destination specific queues.");

        if (outputSubqueue.fifoQueue.empty()) {
            return false;
        }//if//

        return (outputSubqueue.fifoQueue.front()->retryTxCount > 0);
    }//if//

    typedef map<NetworkAddress, DestinationSpecificInfoType>::const_iterator IterType;

    const IterType iter = outputSubqueue.destinationSpecificInfos.find(nextHopAddress);

    if ((iter == outputSubqueue.destinationSpecificInfos.end()) ||
        (iter->second.aQueue.empty())) {

        return false;
    }//if//

    return (iter->second.aQueue.front()->retryTxCount > 0);

}//NextPacketWithPriorityAndNextHop//



inline
void OutputQueueWithPrioritySubqueues::Insert(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriorityType ipPriority,
    EnqueueResultType& enqueueResult,
    unique_ptr<Packet>& packetToDrop,
    const EtherTypeFieldType etherType)
{
    PacketPriorityType priority = ipPriority;
    if (priorityMapperPtr != nullptr) {
        priority = priorityMapperPtr->MapIpToMacPriority(ipPriority);
    }//if//

    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);

    if ((subqueueMaxPackets != 0) && (queueInfo.fifoQueue.size() >= subqueueMaxPackets)) {
        enqueueResult = ENQUEUE_FAILURE_BY_MAX_PACKETS;
        packetToDrop = move(packetPtr);
    }
    else if ((subqueueMaxBytes != 0) &&
             ((queueInfo.currentNumberBytes + packetPtr->LengthBytes()) > subqueueMaxBytes)) {

        enqueueResult = ENQUEUE_FAILURE_BY_MAX_BYTES;
        packetToDrop = move(packetPtr);
    }
    else {
        enqueueResult = ENQUEUE_SUCCESS;
        packetToDrop = nullptr;
        totalPacketBytes += packetPtr->LengthBytes();
        queueInfo.currentNumberBytes += packetPtr->LengthBytes();
        totalPackets++;
        const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
        queueInfo.fifoQueue.push_back(
            unique_ptr<OutputQueueRecordType>(
                new OutputQueueRecordType(packetPtr, nextHopAddress, etherType, currentTime)));

        if (nextHopSpecificQueuesAreEnabled) {

            // Also add to destination specific queue.

            queueInfo.destinationSpecificInfos[nextHopAddress].aQueue.push_back(
                queueInfo.fifoQueue.back().get());
        }//if//
    }//if//

}//Insert//



inline
void OutputQueueWithPrioritySubqueues::RequeueAtFront(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriorityType priority,
    const EtherTypeFieldType etherType,
    const TimeType& queueInsertionTime,
    const unsigned int retryTxCount,
    const unsigned short int sequenceNumber)
{
    assert(priority < outputSubqueues.size());

    const unsigned int packetLengthBytes = packetPtr->LengthBytes();
    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);

    //Can overstuff

    queueInfo.fifoQueue.push_front(
        unique_ptr<OutputQueueRecordType>(
            new OutputQueueRecordType(
                packetPtr, nextHopAddress, etherType,
                queueInsertionTime, retryTxCount, sequenceNumber)));

    queueInfo.currentNumberBytes += packetLengthBytes;
    totalPackets++;
    totalPacketBytes += packetLengthBytes;

    if (nextHopSpecificQueuesAreEnabled) {
        // Also add to destination specific.
        queueInfo.destinationSpecificInfos[nextHopAddress].aQueue.push_front(
            queueInfo.fifoQueue.front().get());
    }//if//

    assert(packetPtr == nullptr);

}//InsertAtFront//



inline
void OutputQueueWithPrioritySubqueues::DequeuePacketWithPriority(
    const PacketPriorityType& priority,
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    EtherTypeFieldType& etherType,
    TimeType& queueInsertionTime,
    bool& isANewPacket,
    unsigned int& retryTxCount,
    unsigned short int& sequenceNumber)
{
    assert(priority < outputSubqueues.size());

    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);

    OutputQueueRecordType& queueRecord = *queueInfo.fifoQueue.front();
    packetPtr = move(queueRecord.packetPtr);
    nextHopAddress = queueRecord.nextHopAddress;
    etherType = queueRecord.etherType;
    queueInsertionTime = queueRecord.queueInsertionTime;
    isANewPacket = !queueRecord.isARequeuedPacket;
    retryTxCount = queueRecord.retryTxCount;
    sequenceNumber = queueRecord.sequenceNumber;

    if (nextHopSpecificQueuesAreEnabled) {

        DestinationSpecificInfoType& destinationInfo =
            queueInfo.destinationSpecificInfos[nextHopAddress];
        std::deque<OutputQueueRecordType*>& destSpecificQueue = destinationInfo.aQueue;

        assert(destSpecificQueue.front() == queueInfo.fifoQueue.front().get());
        destSpecificQueue.pop_front();

        if (destSpecificQueue.empty()) {
            destinationInfo.transitionedToEmptyTime = simEngineInterfacePtr->CurrentTime();
        }//if//
    }//if//

    queueInfo.fifoQueue.pop_front();

    queueInfo.currentNumberBytes -= packetPtr->LengthBytes();
    totalPackets--;
    totalPacketBytes -= packetPtr->LengthBytes();

    // Cleanup:

    if (nextHopSpecificQueuesAreEnabled) {
        while((!queueInfo.fifoQueue.empty()) && (queueInfo.fifoQueue.front()->packetPtr == nullptr)) {
            queueInfo.fifoQueue.pop_front();
        }//while//
    }//if//

}//DequeuePacketWithPriority//


inline
void OutputQueueWithPrioritySubqueues::DequeuePacketWithPriorityAndNextHop(
    const PacketPriorityType& priority,
    const NetworkAddress& nextHopAddress,
    unique_ptr<Packet>& packetPtr,
    EtherTypeFieldType& etherType,
    TimeType& queueInsertionTime,
    bool& isANewPacket,
    unsigned int& retryTxCount,
    unsigned short int& sequenceNumber)
{
    if (!nextHopSpecificQueuesAreEnabled) {
        NetworkAddress actualNextHopAddress;

        (*this).DequeuePacketWithPriority(
            priority,
            packetPtr,
            actualNextHopAddress,
            etherType,
            queueInsertionTime,
            isANewPacket,
            retryTxCount,
            sequenceNumber);

        assert(actualNextHopAddress == nextHopAddress);
        return;

    }//if//

    assert(priority < outputSubqueues.size());

    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);

    DestinationSpecificInfoType& destinationInfo = queueInfo.destinationSpecificInfos[nextHopAddress];
    std::deque<OutputQueueRecordType*>& destinationSpecificQueue = destinationInfo.aQueue;

    OutputQueueRecordType& queueRecord = *destinationSpecificQueue.front();

    packetPtr = move(queueRecord.packetPtr);
    etherType = queueRecord.etherType;
    queueInsertionTime = queueRecord.queueInsertionTime;
    isANewPacket = !queueRecord.isARequeuedPacket;
    retryTxCount = queueRecord.retryTxCount;
    sequenceNumber = queueRecord.sequenceNumber;

    destinationSpecificQueue.pop_front();

    queueInfo.currentNumberBytes -= packetPtr->LengthBytes();
    totalPackets--;
    totalPacketBytes -= packetPtr->LengthBytes();

    if (destinationSpecificQueue.empty()) {
        destinationInfo.transitionedToEmptyTime = simEngineInterfacePtr->CurrentTime();
    }//if//

    // Cleanup

    while((!queueInfo.fifoQueue.empty()) && (queueInfo.fifoQueue.front()->packetPtr == nullptr)) {
        queueInfo.fifoQueue.pop_front();
    }//while//

}//DequeuePacketWithPriorityAndNextHop//



inline
void OutputQueueWithPrioritySubqueues::DequeuePacket(
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    PacketPriorityType& priority,
    EtherTypeFieldType& etherType)
{
    size_t i = outputSubqueues.size() - 1;
    while(true) {
        OutputSubqueueInfoType& queueInfo = outputSubqueues.at(i);

        if (!queueInfo.fifoQueue.empty()) {

            priority = PacketPriorityType(i);
            TimeType notUsed1;
            bool isANewPacket;
            unsigned int notUsed2;
            unsigned short int notUsed3;

            (*this).DequeuePacketWithPriority(
                priority, packetPtr, nextHopAddress, etherType,
                notUsed1, isANewPacket, notUsed2, notUsed3);

            assert(isANewPacket);

            return;
        }//if//

        if (i == 0) {
            break;
        }//if//

        i--;

    }//while//

    assert(false && "Program Error: All Queues are Empty!"); abort();

}//DequeuePacket//



inline
void OutputQueueWithPrioritySubqueues::DequeueLifetimeExpiredPackets(
    const PacketPriorityType& priority,
    const TimeType& packetsInsertedBeforeTime,
    vector<ExpiredPacketInfoType>& packetList)
{
    packetList.clear();
    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);

    if (!nextHopSpecificQueuesAreEnabled) {

        while((!queueInfo.fifoQueue.empty()) &&
            (queueInfo.fifoQueue.front()->queueInsertionTime < packetsInsertedBeforeTime)) {

            OutputQueueRecordType& info = *queueInfo.fifoQueue.front();

            queueInfo.currentNumberBytes -= info.packetPtr->LengthBytes();
            totalPackets--;
            totalPacketBytes -= info.packetPtr->LengthBytes();

            packetList.push_back(ExpiredPacketInfoType(info.packetPtr, info.nextHopAddress));
            queueInfo.fifoQueue.pop_front();

        }//while//
    }
    else {
        typedef map<NetworkAddress, DestinationSpecificInfoType>::iterator IterType;

        for(IterType iter = queueInfo.destinationSpecificInfos.begin();
            (iter != queueInfo.destinationSpecificInfos.end()); ++iter) {

            DestinationSpecificInfoType& destInfo = iter->second;

            while((destInfo.aQueue.empty()) &&
                  (destInfo.aQueue.front()->queueInsertionTime < packetsInsertedBeforeTime)) {

                OutputQueueRecordType& record = *destInfo.aQueue.front();

                queueInfo.currentNumberBytes -= record.packetPtr->LengthBytes();
                totalPackets--;
                totalPacketBytes -= record.packetPtr->LengthBytes();

                packetList.push_back(ExpiredPacketInfoType(record.packetPtr, record.nextHopAddress));
                queueInfo.fifoQueue.pop_front();

            }//for//
        }//for//

        // Cleanup

        while((!queueInfo.fifoQueue.empty()) && (queueInfo.fifoQueue.front()->packetPtr == nullptr)) {
            queueInfo.fifoQueue.pop_front();
        }//while//

    }//if//

}//DequeueLifetimeExpiredPackets//



inline
bool SequenceNumberListIsInOrder(const vector<unsigned short int>& sequenceNumberList)
{
    for(unsigned int i = 1; (i < sequenceNumberList.size()); i++) {
        if (!TwelveBitSequenceNumberIsLessThan(sequenceNumberList[i-1], sequenceNumberList[i])) {
            return false;
        }//if//
    }//for//
    return true;
}


inline
void OutputQueueWithPrioritySubqueues::DeletePacketsBySequenceNumber(
    const PacketPriorityType& priority,
    const NetworkAddress& nextHopAddress,
    const vector<unsigned short int>& sequenceNumberList)
{
    assert(SequenceNumberListIsInOrder(sequenceNumberList));

    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);

    DestinationSpecificInfoType& destinationInfo = queueInfo.destinationSpecificInfos[nextHopAddress];
    std::deque<OutputQueueRecordType*>& destinationSpecificQueue = destinationInfo.aQueue;

    unsigned int i = 0;
    unsigned int pos = 0;

    while ((i < sequenceNumberList.size()) && (pos < destinationSpecificQueue.size())) {
        OutputQueueRecordType& record = *destinationSpecificQueue[pos];
        if (!record.isARequeuedPacket) {
            break;
        }//if//

        if (record.packetPtr == nullptr) {
            pos++;
        }
        else if (record.sequenceNumber == sequenceNumberList[i]) {
            queueInfo.currentNumberBytes -= record.packetPtr->LengthBytes();
            totalPackets--;
            totalPacketBytes -= record.packetPtr->LengthBytes();
            record.packetPtr.reset();
            i++;
            pos++;
        }
        else if (TwelveBitSequenceNumberIsLessThan(sequenceNumberList[i], record.sequenceNumber)) {
            i++;
        }
        else {
            pos++;
        }//if//
    }//while//


    // Cleanup

    while((!queueInfo.fifoQueue.empty()) && (queueInfo.fifoQueue.front()->packetPtr == nullptr)) {
        queueInfo.fifoQueue.pop_front();
    }//while//

}//DeletePacketsBySequenceNumber//



//--------------------------------------------------------------------------------------------------


class BasicOutputQueueWithPrioritySubqueues: public InterfaceOutputQueue {
public:
    BasicOutputQueueWithPrioritySubqueues(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceIdType& interfaceId,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const PacketPriorityType& initMaximumPriority);

    virtual void Insert(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType priority,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDrop,
        const EtherTypeFieldType etherType = ETHERTYPE_IS_NOT_SPECIFIED) override;

    virtual PacketPriorityType MaxPossiblePacketPriority() const override { return (maximumPriority); }

    virtual bool IsEmpty() const override { return (totalPackets == 0); }

    unsigned int NumberPackets() const { return (totalPackets); }

    unsigned long long int NumberPacketBytes() const { return (totalPacketBytes); }

    bool HasPacketWithPriority(const PacketPriorityType priority) const
    {
        assert(priority <= maximumPriority);
        return (!outputSubqueues.at(priority).aQueue.empty());
    }

    unsigned int NumPacketsWithPriority(const PacketPriorityType priority) const
    {
        assert(priority <= maximumPriority);
        return (static_cast<unsigned int>(outputSubqueues.at(priority).aQueue.size()));
    }

    const Packet& TopPacket(const PacketPriorityType priority) const;

    const Packet& GetPacket(
        const PacketPriorityType priority,
        const unsigned int positionInSubqueue) const;

    const NetworkAddress NextHopForTopPacket(const PacketPriorityType priority) const;

    const NetworkAddress NextHopForPacket(
        const PacketPriorityType priority,
        const unsigned int positionInSubqueue) const;

    virtual void DequeuePacket(
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        PacketPriorityType& priority,
        EtherTypeFieldType& etherType) override;

    virtual void DequeuePacketWithPriority(
        const PacketPriorityType& priority,
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        EtherTypeFieldType& etherType,
        TimeType& timestamp,
        unsigned int& retryTxCount)
    {
        (*this).DequeuePacketWithPriorityAndPosition(
            priority, 0, packetPtr, nextHopAddress, etherType, timestamp, retryTxCount);
    }

    void DequeuePacketWithPriorityAndPosition(
        const PacketPriorityType& priority,
        const unsigned int positionInSubqueue,
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        EtherTypeFieldType& etherType,
        TimeType& timestamp,
        unsigned int& retryTxCount);

private:

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;

    struct OutputQueueRecord {
        OutputQueueRecord(
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initNextHopAddress,
            const EtherTypeFieldType initEtherType,
            const TimeType& initTimestamp,
            const unsigned int initRetryTxCount = 0)
            :
            packetPtr(move(initPacketPtr)),
            nextHopAddress(initNextHopAddress),
            etherType(initEtherType),
            timestamp(initTimestamp),
            retryTxCount(initRetryTxCount)
        {
        }

        void operator=(OutputQueueRecord&& right) {
            assert(this != &right);
            packetPtr = move(right.packetPtr);
            nextHopAddress = right.nextHopAddress;
            etherType = right.etherType;
            timestamp = right.timestamp;
            retryTxCount = right.retryTxCount;
        }

        OutputQueueRecord(OutputQueueRecord&& right)  { (*this) = move(right); }

        unique_ptr<Packet> packetPtr;
        NetworkAddress nextHopAddress;
        EtherTypeFieldType etherType;
        TimeType timestamp;
        unsigned int retryTxCount;

    };//OutputQueueRecord//

    struct OutputSubqueueInfo {
        unsigned long long int currentNumberBytes;
        std::deque<OutputQueueRecord> aQueue;

        OutputSubqueueInfo() : currentNumberBytes(0) { }

        void operator=(OutputSubqueueInfo&& right) {
            currentNumberBytes = right.currentNumberBytes;
            aQueue = move(right.aQueue);
        }

        OutputSubqueueInfo(OutputSubqueueInfo&& right) { (*this) = move(right); }
    };

    PacketPriorityType maximumPriority;

    unsigned int totalPackets;
    unsigned long long int totalPacketBytes;

    unsigned int subqueueMaxPackets;
    unsigned int subqueueMaxBytes;

    vector<OutputSubqueueInfo> outputSubqueues;

    // Disable:

    BasicOutputQueueWithPrioritySubqueues(const BasicOutputQueueWithPrioritySubqueues&);
    void operator=(const BasicOutputQueueWithPrioritySubqueues&);

};//BasicOutputQueueWithPrioritySubqueues//


//----------------------------------------------------------

inline
BasicOutputQueueWithPrioritySubqueues::BasicOutputQueueWithPrioritySubqueues(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceIdType& interfaceId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const PacketPriorityType& initMaximumPriority)
    :
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    maximumPriority(initMaximumPriority),
    totalPackets(0),
    totalPacketBytes(0),
    outputSubqueues(initMaximumPriority + 1),
    subqueueMaxPackets(0),
    subqueueMaxBytes(0)
{
    const NodeIdType nodeId = simEngineInterfacePtr->GetNodeId();

    if (theParameterDatabaseReader.ParameterExists("interface-output-queue-max-packets-per-subq", nodeId, interfaceId)){
        subqueueMaxPackets =
            theParameterDatabaseReader.ReadNonNegativeInt("interface-output-queue-max-packets-per-subq", nodeId, interfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("interface-output-queue-max-bytes-per-subq", nodeId, interfaceId)){
        subqueueMaxBytes =
            theParameterDatabaseReader.ReadNonNegativeInt("interface-output-queue-max-bytes-per-subq", nodeId, interfaceId);
    }//if//

}//BasicOutputQueueWithPrioritySubqueues()/


inline
const Packet& BasicOutputQueueWithPrioritySubqueues::TopPacket(const PacketPriorityType priority) const
{
    assert(priority <= maximumPriority);
    assert(!outputSubqueues[priority].aQueue.empty());

    return (*outputSubqueues[priority].aQueue.front().packetPtr);
}


inline
const Packet& BasicOutputQueueWithPrioritySubqueues::GetPacket(
    const PacketPriorityType priority,
    const unsigned int positionInSubqueue) const
{
    assert(priority <= maximumPriority);
    return (*outputSubqueues[priority].aQueue.at(positionInSubqueue).packetPtr);
}



inline
const NetworkAddress BasicOutputQueueWithPrioritySubqueues::NextHopForTopPacket(const PacketPriorityType priority) const
{
    assert(priority <= maximumPriority);
    assert(!outputSubqueues[priority].aQueue.empty());

    return (outputSubqueues[priority].aQueue.front().nextHopAddress);
}

inline
const NetworkAddress BasicOutputQueueWithPrioritySubqueues::NextHopForPacket(
    const PacketPriorityType priority,
    const unsigned int positionInSubqueue) const
{
    assert(priority <= maximumPriority);
    return (outputSubqueues[priority].aQueue.at(positionInSubqueue).nextHopAddress);
}


inline
void BasicOutputQueueWithPrioritySubqueues::Insert(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriorityType priority,
    EnqueueResultType& enqueueResult,
    unique_ptr<Packet>& packetToDrop,
    const EtherTypeFieldType etherType)
{
    assert(priority <= maximumPriority);

    OutputSubqueueInfo& queueInfo = outputSubqueues.at(priority);

    if ((subqueueMaxPackets != 0) && (queueInfo.aQueue.size() >= subqueueMaxPackets)) {
        enqueueResult = ENQUEUE_FAILURE_BY_MAX_PACKETS;
        packetToDrop = move(packetPtr);
    }
    else if ((subqueueMaxBytes != 0) && ((queueInfo.currentNumberBytes + packetPtr->LengthBytes()) > subqueueMaxBytes)) {
        enqueueResult = ENQUEUE_FAILURE_BY_MAX_BYTES;
        packetToDrop = move(packetPtr);
    }
    else {
        enqueueResult = ENQUEUE_SUCCESS;
        packetToDrop = nullptr;
        totalPacketBytes += packetPtr->LengthBytes();
        queueInfo.currentNumberBytes += packetPtr->LengthBytes();
        totalPackets++;
        const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
        queueInfo.aQueue.push_back(
            move(OutputQueueRecord(packetPtr, nextHopAddress, etherType, currentTime)));
    }//if//

}//Insert//


inline
void BasicOutputQueueWithPrioritySubqueues::DequeuePacket(
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    PacketPriorityType& priority,
    EtherTypeFieldType& etherType)
{
    size_t i = outputSubqueues.size() - 1;
    while(true) {
        OutputSubqueueInfo& queueInfo = outputSubqueues.at(i);

        if (!queueInfo.aQueue.empty()) {
            OutputQueueRecord& queueRecord = queueInfo.aQueue.front();

            packetPtr = move(queueRecord.packetPtr);
            nextHopAddress = queueRecord.nextHopAddress;
            priority = PacketPriorityType(i);
            etherType = queueRecord.etherType;

            queueInfo.aQueue.pop_front();
            queueInfo.currentNumberBytes -= packetPtr->LengthBytes();
            totalPackets--;
            totalPacketBytes -= packetPtr->LengthBytes();

            return;
        }//if//

        if (i == 0) {
            break;
        }//if//

        i--;

    }//while//

    assert(false && "Program Error: All Queues are Empty!"); abort();

}//DequeuePacket//



inline
void BasicOutputQueueWithPrioritySubqueues::DequeuePacketWithPriorityAndPosition(
    const PacketPriorityType& priority,
    const unsigned int positionInSubqueue,
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    EtherTypeFieldType& etherType,
    TimeType& timestamp,
    unsigned int& retryTxCount)
{
    assert(priority <= maximumPriority);
    OutputSubqueueInfo& queueInfo = outputSubqueues.at(priority);

    OutputQueueRecord& queueRecord = queueInfo.aQueue.at(positionInSubqueue);
    packetPtr = move(queueRecord.packetPtr);
    nextHopAddress = queueRecord.nextHopAddress;
    etherType = queueRecord.etherType;
    timestamp = queueRecord.timestamp;
    retryTxCount = queueRecord.retryTxCount;

    if (positionInSubqueue == 0) {
        queueInfo.aQueue.pop_front();
    }
    else {
        queueInfo.aQueue.erase(queueInfo.aQueue.begin() + positionInSubqueue);
    }//if//

    queueInfo.currentNumberBytes -= packetPtr->LengthBytes();
    totalPackets--;
    totalPacketBytes -= packetPtr->LengthBytes();

}//DequeuePacketWithPriorityAndPosition//



//--------------------------------------------------------------------------------------------------

class OutputQueueWithPrioritySubqueuesOlderVer: public InterfaceOutputQueue {
public:
    OutputQueueWithPrioritySubqueuesOlderVer(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceIdType& interfaceId,
        const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
        const PacketPriorityType& maximumPriority);

    virtual void Insert(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType priority,
        EnqueueResultType& enqueueResult,
        unique_ptr<Packet>& packetToDrop,
        const EtherTypeFieldType etherType = ETHERTYPE_IS_NOT_SPECIFIED) override;

    void RequeueAtFront(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType priority,
        const EtherTypeFieldType etherType,
        const TimeType& timestamp,
        const unsigned int retryTxCount);

    virtual PacketPriorityType MaxPossiblePacketPriority() const override { return (maximumPriority); }

    virtual bool IsEmpty() const override { return (totalPackets == 0); }

    unsigned int NumberPackets() const { return (totalPackets); }

    unsigned long long int NumberPacketBytes() const { return (totalPacketBytes); }

    bool HasPacketWithPriority(const PacketPriorityType priority) const
    {
        assert(priority <= maximumPriority);
        return (!outputSubqueues.at(priority).fifoQueue.empty());
    }

    const Packet& TopPacket(const PacketPriorityType priority) const;
    const NetworkAddress& NextHopAddressForTopPacket(const PacketPriorityType priority) const;

    virtual void DequeuePacket(
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        PacketPriorityType& priority,
        EtherTypeFieldType& etherType) override;

    virtual void DequeuePacketWithPriority(
        const PacketPriorityType& priority,
        unique_ptr<Packet>& packetPtr,
        NetworkAddress& nextHopAddress,
        EtherTypeFieldType& etherType,
        TimeType& timestamp,
        unsigned int& retryTxCount);

    // Allows extracting frames to a destination in non-FIFO order.  For frame aggregation.

    void EnableNextHopSpecificDequeues() {
        assert(IsEmpty());
        (*this).nextHopSpecificQueuesAreEnabled = true;
    }

    bool NextHopSpecificDequeueIsEnabled() const { return (nextHopSpecificQueuesAreEnabled); }

    bool HasPacketWithPriorityAndNextHop(
        const PacketPriorityType priority,
        const NetworkAddress& nextHopAddress) const;

    // These methods still work in FIFO mode, but the next hop address must match
    // top of queue (assert).

    const Packet& NextPacketWithPriorityAndNextHop(
        const PacketPriorityType priority,
        const NetworkAddress& nextHopAddress) const;

    void DequeuePacketWithPriorityAndNextHop(
        const PacketPriorityType& priority,
        const NetworkAddress& nextHopAddress,
        unique_ptr<Packet>& packetPtr,
        EtherTypeFieldType& etherType,
        TimeType& timestamp,
        unsigned int& retryTxCount);

private:

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;

    struct OutputQueueRecordType {
        OutputQueueRecordType(
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initNextHopAddress,
            const EtherTypeFieldType initEtherType,
            const TimeType& initTimestamp,
            const unsigned int initRetryTxCount = 0)
            :
            packetPtr(move(initPacketPtr)),
            nextHopAddress(initNextHopAddress),
            etherType(initEtherType),
            timestamp(initTimestamp),
            retryTxCount(initRetryTxCount)
        {
        }

        void operator=(OutputQueueRecordType&& right) {
            assert(this != &right);
            packetPtr = move(right.packetPtr);
            nextHopAddress = right.nextHopAddress;
            etherType = right.etherType;
            timestamp = right.timestamp;
            retryTxCount = right.retryTxCount;
        }

        OutputQueueRecordType(OutputQueueRecordType&& right)  { (*this) = move(right); }

        unique_ptr<Packet> packetPtr;
        NetworkAddress nextHopAddress;
        EtherTypeFieldType etherType;
        TimeType timestamp;
        unsigned int retryTxCount;

    };//OutputQueueRecordType//


    struct OutputSubqueueInfoType {

        unsigned long long int currentNumberBytes;

        // Records are "owned" by fifoQueue.  Raw pointers instead of shared_ptr for speed.

        std::deque<unique_ptr<OutputQueueRecordType> > fifoQueue;
        map<NetworkAddress, std::deque<OutputQueueRecordType*> > destinationSpecificQueues;

        OutputSubqueueInfoType() : currentNumberBytes(0) { }

        void operator=(OutputSubqueueInfoType&& right) {
            currentNumberBytes = right.currentNumberBytes;
            fifoQueue = move(right.fifoQueue);
            destinationSpecificQueues = move(right.destinationSpecificQueues);
        }

        OutputSubqueueInfoType(OutputSubqueueInfoType&& right) { (*this) = move(right); }
    };

    bool nextHopSpecificQueuesAreEnabled;

    PacketPriorityType maximumPriority;

    unsigned int totalPackets;
    unsigned long long int totalPacketBytes;

    unsigned int subqueueMaxPackets;
    unsigned int subqueueMaxBytes;

    vector<OutputSubqueueInfoType> outputSubqueues;

    // Disable:

    OutputQueueWithPrioritySubqueuesOlderVer(const OutputQueueWithPrioritySubqueuesOlderVer&);
    void operator=(const OutputQueueWithPrioritySubqueuesOlderVer&);

};//OutputQueueWithPrioritySubqueuesOlderVer//


//----------------------------------------------------------

inline
OutputQueueWithPrioritySubqueuesOlderVer::OutputQueueWithPrioritySubqueuesOlderVer(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceIdType& interfaceId,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const PacketPriorityType& initMaximumPriority)
    :
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    maximumPriority(initMaximumPriority),
    nextHopSpecificQueuesAreEnabled(false),
    totalPackets(0),
    totalPacketBytes(0),
    outputSubqueues(initMaximumPriority + 1),
    subqueueMaxPackets(0),
    subqueueMaxBytes(0)
{
    const NodeIdType nodeId = simEngineInterfacePtr->GetNodeId();

    if (theParameterDatabaseReader.ParameterExists("interface-output-queue-max-packets-per-subq", nodeId, interfaceId)){
        subqueueMaxPackets =
            theParameterDatabaseReader.ReadNonNegativeInt("interface-output-queue-max-packets-per-subq", nodeId, interfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("interface-output-queue-max-bytes-per-subq", nodeId, interfaceId)){
        subqueueMaxBytes =
            theParameterDatabaseReader.ReadNonNegativeInt("interface-output-queue-max-bytes-per-subq", nodeId, interfaceId);
    }//if//

}//OutputQueueWithPrioritySubqueuesOlderVer()/


inline
const Packet& OutputQueueWithPrioritySubqueuesOlderVer::TopPacket(const PacketPriorityType priority) const
{
    assert(priority <= maximumPriority);
    assert(!outputSubqueues[priority].fifoQueue.empty());

    return (*outputSubqueues[priority].fifoQueue.front()->packetPtr);
}


inline
const NetworkAddress& OutputQueueWithPrioritySubqueuesOlderVer::NextHopAddressForTopPacket(
    const PacketPriorityType priority) const
{
    assert(priority <= maximumPriority);
    assert(!outputSubqueues[priority].fifoQueue.empty());

    return (outputSubqueues[priority].fifoQueue.front()->nextHopAddress);
}


inline
bool OutputQueueWithPrioritySubqueuesOlderVer::HasPacketWithPriorityAndNextHop(
    const PacketPriorityType priority,
    const NetworkAddress& nextHopAddress) const
{
    assert(nextHopSpecificQueuesAreEnabled);
    assert(priority <= maximumPriority);
    const OutputSubqueueInfoType& outputSubqueue = outputSubqueues.at(priority);

    typedef map<NetworkAddress, std::deque<OutputQueueRecordType*> >::const_iterator IterType;
    const IterType iter = outputSubqueue.destinationSpecificQueues.find(nextHopAddress);

    if (iter == outputSubqueue.destinationSpecificQueues.end()) {
        return false;
    }//if//

    return (!iter->second.empty());

}//HasPacketWithPriorityAndNextHop//


inline
const Packet& OutputQueueWithPrioritySubqueuesOlderVer::NextPacketWithPriorityAndNextHop(
    const PacketPriorityType priority,
    const NetworkAddress& nextHopAddress) const
{
    assert(priority <= maximumPriority);
    const OutputSubqueueInfoType& outputSubqueue = outputSubqueues.at(priority);

    if (!nextHopSpecificQueuesAreEnabled) {
        assert((outputSubqueue.fifoQueue.front()->nextHopAddress == nextHopAddress) &&
               "Access must be strictly FIFO without destination specific queues.");

        return (*outputSubqueue.fifoQueue.front()->packetPtr);
    }//if//

    typedef map<NetworkAddress, std::deque<OutputQueueRecordType*> >::const_iterator IterType;

    const IterType iter = outputSubqueue.destinationSpecificQueues.find(nextHopAddress);
    assert(iter != outputSubqueue.destinationSpecificQueues.end());

    return (*iter->second.front()->packetPtr);

}//NextPacketWithPriorityAndNextHop//



inline
void OutputQueueWithPrioritySubqueuesOlderVer::Insert(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriorityType priority,
    EnqueueResultType& enqueueResult,
    unique_ptr<Packet>& packetToDrop,
    const EtherTypeFieldType etherType)
{
    assert(priority <= maximumPriority);

    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);

    if ((subqueueMaxPackets != 0) && (queueInfo.fifoQueue.size() >= subqueueMaxPackets)) {
        enqueueResult = ENQUEUE_FAILURE_BY_MAX_PACKETS;
        packetToDrop = move(packetPtr);
    }
    else if ((subqueueMaxBytes != 0) &&
             ((queueInfo.currentNumberBytes + packetPtr->LengthBytes()) > subqueueMaxBytes)) {

        enqueueResult = ENQUEUE_FAILURE_BY_MAX_BYTES;
        packetToDrop = move(packetPtr);
    }
    else {
        enqueueResult = ENQUEUE_SUCCESS;
        packetToDrop = nullptr;
        totalPacketBytes += packetPtr->LengthBytes();
        queueInfo.currentNumberBytes += packetPtr->LengthBytes();
        totalPackets++;
        const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
        queueInfo.fifoQueue.push_back(
            unique_ptr<OutputQueueRecordType>(
                new OutputQueueRecordType(packetPtr, nextHopAddress, etherType, currentTime)));

        if (nextHopSpecificQueuesAreEnabled) {

            // Also add to destination specific queue.

            queueInfo.destinationSpecificQueues[nextHopAddress].push_back(
                queueInfo.fifoQueue.back().get());
        }//if//
    }//if//

}//Insert//



inline
void OutputQueueWithPrioritySubqueuesOlderVer::RequeueAtFront(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriorityType priority,
    const EtherTypeFieldType etherType,
    const TimeType& timestamp,
    const unsigned int retryTxCount)
{
    assert(priority <= maximumPriority);

    const unsigned int packetLengthBytes = packetPtr->LengthBytes();
    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);

    //Can overstuff

    queueInfo.fifoQueue.push_front(
        unique_ptr<OutputQueueRecordType>(
            new OutputQueueRecordType(packetPtr, nextHopAddress, etherType, timestamp, retryTxCount)));

    queueInfo.currentNumberBytes += packetLengthBytes;
    totalPackets++;
    totalPacketBytes += packetLengthBytes;

    if (nextHopSpecificQueuesAreEnabled) {
        // Also add to destination specific.
        queueInfo.destinationSpecificQueues[nextHopAddress].push_front(
            queueInfo.fifoQueue.front().get());
    }//if//

    assert(packetPtr == nullptr);

}//InsertAtFront//



inline
void OutputQueueWithPrioritySubqueuesOlderVer::DequeuePacketWithPriority(
    const PacketPriorityType& priority,
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    EtherTypeFieldType& etherType,
    TimeType& timestamp,
    unsigned int& retryTxCount)
{
    assert(priority <= maximumPriority);
    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);

    OutputQueueRecordType& queueRecord = *queueInfo.fifoQueue.front();
    packetPtr = move(queueRecord.packetPtr);
    nextHopAddress = queueRecord.nextHopAddress;
    etherType = queueRecord.etherType;
    timestamp = queueRecord.timestamp;
    retryTxCount = queueRecord.retryTxCount;

    if (nextHopSpecificQueuesAreEnabled) {
        std::deque<OutputQueueRecordType*>& destSpecificQueue =
            queueInfo.destinationSpecificQueues[nextHopAddress];
        assert(destSpecificQueue.front() == queueInfo.fifoQueue.front().get());
        destSpecificQueue.pop_front();
    }//if//

    queueInfo.fifoQueue.pop_front();

    queueInfo.currentNumberBytes -= packetPtr->LengthBytes();
    totalPackets--;
    totalPacketBytes -= packetPtr->LengthBytes();

    // Cleanup:

    if (nextHopSpecificQueuesAreEnabled) {
        while((!queueInfo.fifoQueue.empty()) && (queueInfo.fifoQueue.front()->packetPtr == nullptr)) {
            queueInfo.fifoQueue.pop_front();
        }//while//
    }//if//

}//DequeuePacketWithPriority//


inline
void OutputQueueWithPrioritySubqueuesOlderVer::DequeuePacketWithPriorityAndNextHop(
    const PacketPriorityType& priority,
    const NetworkAddress& nextHopAddress,
    unique_ptr<Packet>& packetPtr,
    EtherTypeFieldType& etherType,
    TimeType& timestamp,
    unsigned int& retryTxCount)
{
    if (!nextHopSpecificQueuesAreEnabled) {
        NetworkAddress actualNextHopAddress;

        (*this).DequeuePacketWithPriority(
            priority,
            packetPtr,
            actualNextHopAddress,
            etherType,
            timestamp,
            retryTxCount);

        assert(actualNextHopAddress == nextHopAddress);
        return;

    }//if//

    assert(priority <= maximumPriority);
    OutputSubqueueInfoType& queueInfo = outputSubqueues.at(priority);

    std::deque<OutputQueueRecordType*>& destinationSpecificQueue =
        queueInfo.destinationSpecificQueues[nextHopAddress];

    OutputQueueRecordType& queueRecord = *destinationSpecificQueue.front();

    packetPtr = move(queueRecord.packetPtr);
    etherType = queueRecord.etherType;
    timestamp = queueRecord.timestamp;
    retryTxCount = queueRecord.retryTxCount;

    destinationSpecificQueue.pop_front();

    queueInfo.currentNumberBytes -= packetPtr->LengthBytes();
    totalPackets--;
    totalPacketBytes -= packetPtr->LengthBytes();

    // Cleanup

    while((!queueInfo.fifoQueue.empty()) && (queueInfo.fifoQueue.front()->packetPtr == nullptr)) {
        queueInfo.fifoQueue.pop_front();
    }//while//

}//DequeuePacketWithPriorityAndNextHop//


inline
void OutputQueueWithPrioritySubqueuesOlderVer::DequeuePacket(
    unique_ptr<Packet>& packetPtr,
    NetworkAddress& nextHopAddress,
    PacketPriorityType& priority,
    EtherTypeFieldType& etherType)
{
    size_t i = outputSubqueues.size() - 1;
    while(true) {
        OutputSubqueueInfoType& queueInfo = outputSubqueues.at(i);

        if (!queueInfo.fifoQueue.empty()) {

            priority = PacketPriorityType(i);
            TimeType notUsed1;
            unsigned int notUsed2;

            (*this).DequeuePacketWithPriority(
                priority, packetPtr, nextHopAddress, etherType, notUsed1, notUsed2);

            return;
        }//if//

        if (i == 0) {
            break;
        }//if//

        i--;

    }//while//

    assert(false && "Program Error: All Queues are Empty!"); abort();

}//DequeuePacket//

//--------------------------------------------------------------------------------------------------


}//namespace//

#endif
