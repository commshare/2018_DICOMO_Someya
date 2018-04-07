// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "scensim_proploss.h"

#include "t109_app.h"
#include "t109_mac.h"

namespace T109 {

using std::unique_ptr;
using std::move;

static inline
NodeIdType ConvertToNodeId(
    const NodeIdType& baseNodeId,
    const string& aString)
{
    string simpleString;

    for(size_t i = 0; i < aString.length(); i++) {
        if (aString[i] != ' ' && aString[i] != '\t') {
            simpleString.push_back(static_cast<char>(tolower(aString[i])));
        }
    }

    int code = 1;
    size_t currentPos = 0;
    NodeIdType nodeId = 0;

    while (currentPos < simpleString.length()) {
        if (simpleString[currentPos] == '+') {
            code = 1;
            currentPos++;
        } else if (simpleString[currentPos] == '-') {
            code = -1;
            currentPos++;
        } else {
            code = 1;
        }

        if (simpleString[currentPos] == '$') {
            assert(simpleString.substr(currentPos, 2) == "$n");

            nodeId += code*baseNodeId;
            currentPos += 2;

        } else {
            assert(isdigit(simpleString[currentPos]));

            const size_t startPos = currentPos;
            const size_t endPos = simpleString.find_first_not_of("0123456789", currentPos);
            const string numberString = simpleString.substr(startPos, endPos - startPos);

            bool success;
            int aValue;

            ConvertStringToInt(numberString, aValue, success);

            if (!success) {
                cerr << "Error: failed to read int " << numberString << endl;
                exit(1);
            }

            nodeId += code*aValue;
            currentPos = endPos;
        }
    }

    return nodeId;
}

const ApplicationIdType  T109App::applicationId = "T109App";
const string T109App::modelName = "T109App";

#pragma warning(disable:4355)

T109App::T109App(
    const ParameterDatabaseReader& initParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<SimplePropagationModelForNode<PropFrameType> >& initPropModelInterfacePtr,
    const shared_ptr<BitOrBlockErrorRateCurveDatabase>& initBerCurveDatabasePtr,
    const NodeIdType& initNodeId,
    const InterfaceIdType& initInterfaceId,
    const size_t initInterfaceIndex,
    const RandomNumberGeneratorSeedType& initNodeSeed,
    const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr)
    :
    Application(initSimulationEngineInterfacePtr, applicationId),
    nodeId(initNodeId),
    interfaceId(initInterfaceId),
    aRandomNumberGenerator(HashInputsToMakeSeed(initNodeSeed, initInterfaceIndex, SEED_HASH)),
    mobilityModelPtr(initNodeMobilityModelPtr),
    macPtr(
        new T109Mac(
            initParameterDatabaseReader,
            initSimulationEngineInterfacePtr,
            initPropModelInterfacePtr,
            initBerCurveDatabasePtr,
            nodeId,
            initInterfaceId,
            initInterfaceIndex,
            this,
            initNodeSeed)),
    currentPacketPayloadSizeByte(0),
    transmissionInterval(
        initParameterDatabaseReader.ReadTime(
            "t109-packet-interval", initNodeId, initInterfaceId)),
    startTime(
        initParameterDatabaseReader.ReadTime(
            "t109-packet-start-time", initNodeId, initInterfaceId)),
    endTime(
        initParameterDatabaseReader.ReadTime(
            "t109-packet-end-time", initNodeId, initInterfaceId)),
    currentStepStartTimeOffset(ZERO_TIME),
    currentStepEndTimeOffset(ZERO_TIME),
    startTimePacketSize(0),
    endTimePacketSize(0),
    currentPacketSequenceNumber(0),
    numberPacketsReceived(0),
    packetsSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + initInterfaceId + "_PacketsSent"))),
    bytesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + initInterfaceId + "_BytesSent"))),
    packetsReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + initInterfaceId + "_PacketsReceived"))),
    bytesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + initInterfaceId + "_BytesReceived")))
{
    simulationEngineInterfacePtr = initSimulationEngineInterfacePtr;

    string packetPayloadSizeByteString =
        initParameterDatabaseReader.ReadString(
            "t109-packet-size-bytes", initNodeId, initInterfaceId);

    deque<string> packetSizeStrings;

    TokenizeToTrimmedLowerString(packetPayloadSizeByteString, ",", packetSizeStrings);

    assert(packetSizeStrings.size() >= 1);

    if (packetSizeStrings.size() == 1 &&
        packetSizeStrings.front().find(":") == string::npos) {

        bool success;

        ConvertStringToInt(packetSizeStrings.front(), currentPacketPayloadSizeByte, success);

        if (!success) {
            cerr << "Error: packet size must be numeric string." << endl;
            exit(1);
        }

        packetSizeToTimes.push_back(make_pair(INFINITE_TIME, currentPacketPayloadSizeByte));

    } else {

        for(size_t i = 0; i < packetSizeStrings.size(); i++) {

            deque<string> durationAndSize;
            TokenizeToTrimmedLowerString(packetSizeStrings[i], ":", durationAndSize);

            if (durationAndSize.size() != 2) {
                cerr << "Error: Specify packet size with transmission duration \"Duration\":\"Packet Size Bytes\"" << endl;
                exit(1);
            }

            bool success1;
            bool success2;

            TimeType duration;
            int packetSizeByte;

            ConvertStringToTime(durationAndSize[0], duration, success1);
            ConvertStringToInt(durationAndSize[1], packetSizeByte, success2);

            if (!success1 || !success2) {
                cerr << "Error: Specify packet size with transmission duration \"Duration\":\"Packet Size Bytes\"" << endl;
                exit(1);
            }

            if (!packetSizeToTimes.empty() && duration < packetSizeToTimes.back().first) {
                cerr << "Error: Specify packet size with time-lined transmission durations \"Duration1\":\"Packet Size Bytes1\", \"Duration2\":\"Packet Size Bytes2\"..." << endl;
                exit(1);
            }

            packetSizeToTimes.push_back(make_pair(duration, packetSizeByte));
        }
    }

    const TimeType startTimeMaxJitter =
        initParameterDatabaseReader.ReadTime(
            "t109-packet-start-time-jitter", initNodeId, initInterfaceId);

    startTime +=
        static_cast<TimeType>(startTimeMaxJitter*aRandomNumberGenerator.GenerateRandomDouble());

    const TimeType currentTime = initSimulationEngineInterfacePtr->CurrentTime();

    if (currentTime > startTime) {
        const size_t nextTransmissionTime = size_t(ceil(double(currentTime - startTime) / transmissionInterval));
        startTime += nextTransmissionTime*transmissionInterval;
    }

    initSimulationEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(new PacketSendEvent(this)), startTime);
}

void T109App::UpdateCurrentTimePacketSize()
{
    assert(!packetSizeToTimes.empty());

    TimeType currentTimeOffset = ZERO_TIME;

    if (packetSizeToTimes.back().first > ZERO_TIME) {
        currentTimeOffset =
            (simulationEngineInterfacePtr->CurrentTime() % packetSizeToTimes.back().first);
    }

    if (!(currentStepStartTimeOffset <= currentTimeOffset &&
          currentTimeOffset < currentStepEndTimeOffset)) {

        currentStepStartTimeOffset = ZERO_TIME;
        currentStepEndTimeOffset = ZERO_TIME;
        startTimePacketSize = static_cast<int>(packetSizeToTimes.back().second);
        endTimePacketSize = startTimePacketSize;

        for(size_t i = 0; i < packetSizeToTimes.size(); i++) {
            const pair<TimeType, size_t>& packetSizeToTime = packetSizeToTimes[i];

            currentStepEndTimeOffset = packetSizeToTime.first;
            endTimePacketSize = static_cast<int>(packetSizeToTime.second);

            if (currentStepStartTimeOffset <= currentTimeOffset &&
                currentTimeOffset < currentStepEndTimeOffset) {
                break;
            }

            currentStepStartTimeOffset = currentStepEndTimeOffset;
            startTimePacketSize = endTimePacketSize;
        }
    }

    assert(currentTimeOffset >= currentStepStartTimeOffset);

    double offsetRate = 0;

    if (currentStepEndTimeOffset != currentStepStartTimeOffset) {
        offsetRate =
            double(currentTimeOffset - currentStepStartTimeOffset) /
            (currentStepEndTimeOffset - currentStepStartTimeOffset);
    }

    currentPacketPayloadSizeByte =
        static_cast<int>(offsetRate*(int(endTimePacketSize) - int(startTimePacketSize)) + startTimePacketSize);
}

void T109App::PeriodicallySendPacket()
{
    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

    if (currentTime >= endTime) {
        return;
    }

    macPtr->ClearQueuedPacket();

    (*this).UpdateCurrentTimePacketSize();

    if (currentPacketPayloadSizeByte > 0) {
        PacketType packetType;

        if (macPtr->GetAccessMode() == ACCESS_MODE_RSU) {
            packetType = PACKET_TYPE_RSU_TO_CAR;
        } else {
            packetType = PACKET_TYPE_CAR_TO_CAR;
        }

        currentPacketSequenceNumber++;

        unique_ptr<Packet> defaultPacketPtr =
            Packet::CreatePacket(
                *simulationEngineInterfacePtr,
                ApplicationHeaderType(
                    packetType,
                    nodeId,
                    currentPacketSequenceNumber,
                    currentTime),
            currentPacketPayloadSizeByte);

        (*this).OutputTraceAndStatsForSendPacket(
            currentPacketSequenceNumber,
            defaultPacketPtr->GetPacketId(),
            defaultPacketPtr->LengthBytes());

        macPtr->ReceivePacketFromUpperLayer(
            defaultPacketPtr,
            SixByteMacAddressType::GetBroadcastAddress());
    }

    simulationEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(new PacketSendEvent(this)),
        (currentTime + transmissionInterval));
}

void T109App::ReceivePacketFromMac(
    unique_ptr<Packet>& packetPtr,
    const double receiveRssiDbm,
    const SixByteMacAddressType& lastHopAddress)
{
    numberPacketsReceived++;

    ApplicationHeaderType& appHeader =
        packetPtr->GetAndReinterpretPayloadData<ApplicationHeaderType>();

    const TimeType delay =
        simulationEngineInterfacePtr->CurrentTime() - appHeader.creationTime;

    (*this).OutputTraceForReceivePacket(
        appHeader.sequenceNumber,
        packetPtr->GetPacketId(),
        packetPtr->LengthBytes(),
        delay);

    packetPtr = nullptr;
}

void T109App::OutputTraceAndStatsForSendPacket(
    const unsigned int sequenceNumber,
    const PacketIdType& packetId,
    const unsigned int packetLengthBytes)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationSendTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.destinationNodeId = ANY_NODEID;
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == APPLICATION_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "AppSend", traceData);

        }
        else {

            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << packetId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, interfaceId, "AppSend", outStream.str());
        }//if//

    }//if//

    packetsSentStatPtr->IncrementCounter();
    bytesSentStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForSendPacket//

void T109App::OutputTraceForReceivePacket(
    const unsigned int sequenceNumber,
    const PacketIdType& packetId,
    const unsigned int packetLengthBytes,
    const TimeType& delay)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationReceiveTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();
            traceData.delay = delay;
            traceData.receivedPackets = numberPacketsReceived;
            traceData.packetLengthBytes = static_cast<uint16_t>(packetLengthBytes);

            assert(sizeof(traceData) == APPLICATION_RECEIVE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "AppRecv", traceData);

        }
        else {
            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << packetId
                      << " Delay= " << ConvertTimeToStringSecs(delay)
                      << " Pdr= " << numberPacketsReceived << '/' << sequenceNumber
                      << " PacketBytes= " << packetLengthBytes;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, interfaceId, "AppRecv", outStream.str());

        }//if//

    }//if//

    packetsReceivedStatPtr->IncrementCounter();
    bytesReceivedStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceForReceivePacket//

}//namespace
