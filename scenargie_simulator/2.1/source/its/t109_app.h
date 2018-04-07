// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef T109_APP_H
#define T109_APP_H

#include "scensim_parmio.h"
#include "scensim_engine.h"
#include "scensim_netsim.h"
#include "scensim_gis.h"

#include "t109_mac.h"

namespace T109 {

using std::pair;
using std::vector;

typedef int16_t PacketType;
enum {
    PACKET_TYPE_RSU_TO_CAR,
    PACKET_TYPE_CAR_TO_CAR,
};

struct ApplicationHeaderType {
    int16_t packetType;
    int16_t forwardCount; //-1 is ack
    int32_t sequenceNumber;

    NodeIdType sourceNodeId;
    NodeIdType lastNodeId;

    TimeType creationTime;

    ApplicationHeaderType(
        const PacketType& initPacketType,
        const NodeIdType& initSourceNodeId,
        const int initSequenceNumber,
        const TimeType& initCreationTime)
        :
        packetType(initPacketType),
        forwardCount(0),
        sequenceNumber(initSequenceNumber),
        sourceNodeId(initSourceNodeId),
        lastNodeId(initSourceNodeId),
        creationTime(initCreationTime)
    {}
};

class T109App : public Application {
public:
    typedef T109Phy::PropFrameType PropFrameType;

    T109App(
        const ParameterDatabaseReader& initParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const shared_ptr<SimplePropagationModelForNode<PropFrameType> >& initPropModelInterfacePtr,
        const shared_ptr<BitOrBlockErrorRateCurveDatabase>& initBerCurveDatabasePtr,
        const NodeIdType& initNodeId,
        const InterfaceIdType& initInterfaceId,
        const size_t initInterfaceIndex,
        const RandomNumberGeneratorSeedType& initNodeSeed,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr);

    void ReceivePacketFromMac(
        unique_ptr<Packet>& packetPtr,
        const double receiveRssiDbm,
        const SixByteMacAddressType& lastHopAddress);

    shared_ptr<T109Mac> GetMacPtr() const { return macPtr; }

private:
    static const ApplicationIdType applicationId;
    static const uint16_t portNumber = 1000;
    static const uint16_t packetPriority = 0;
    static const int SEED_HASH = 2732192;
    static const string modelName;

    const NodeIdType nodeId;
    const InterfaceIdType interfaceId;
    RandomNumberGenerator aRandomNumberGenerator;

    shared_ptr<ObjectMobilityModel> mobilityModelPtr;
    shared_ptr<T109Mac> macPtr;

    int currentPacketPayloadSizeByte;
    TimeType transmissionInterval;
    TimeType startTime;
    TimeType endTime;

    TimeType currentStepStartTimeOffset;
    TimeType currentStepEndTimeOffset;
    int startTimePacketSize;
    int endTimePacketSize;

    int32_t currentPacketSequenceNumber;
    int32_t numberPacketsReceived;

    vector<pair<TimeType, size_t> > packetSizeToTimes;

    shared_ptr<CounterStatistic> packetsSentStatPtr;
    shared_ptr<CounterStatistic> bytesSentStatPtr;
    shared_ptr<CounterStatistic> packetsReceivedStatPtr;
    shared_ptr<CounterStatistic> bytesReceivedStatPtr;

    void PeriodicallySendPacket();

    void UpdateCurrentTimePacketSize();

    void OutputTraceAndStatsForSendPacket(
        const unsigned int sequenceNumber,
        const PacketIdType& packetId,
        const unsigned int packetLengthBytes);

    void OutputTraceForReceivePacket(
        const unsigned int sequenceNumber,
        const PacketIdType& packetId,
        const unsigned int packetLengthBytes,
        const TimeType& delay);

    class PacketSendEvent : public SimulationEvent {
    public:
        PacketSendEvent(T109App* initAppPtr) : appPtr(initAppPtr) {}

        virtual void ExecuteEvent() { appPtr->PeriodicallySendPacket(); }
    private:
        T109App* appPtr;
    };
};

}//namespace

#endif
