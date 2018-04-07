// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11_TRACEDEF_H
#define DOT11_TRACEDEF_H

#include <stdint.h>

#include "scensim_time.h"
#include "scensim_nodeid.h"

#include "dot11_common.h"
#include "dot11_headers.h"

namespace Dot11 {

using std::string;

using ScenSim::NodeIdType;
using ScenSim::TimeType;

//mac

const size_t MAC_FRAME_RECEIVE_TRACE_RECORD_BYTES = 16;
struct MacFrameReceiveTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    unsigned char frameType; // const static defined
    unsigned char padding[1];
    uint16_t packetLengthBytes; // after format version 2
};

const size_t DOT11_MAC_TX_MANAGEMENT_TRACE_RECORD_BYTES = 16;
struct Dot11MacTxManagementTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    unsigned char frameType; // const static defined
    unsigned char padding[3];
};

const size_t MAC_IFS_AND_BACKOFF_START_TRACE_RECORD_BYTES = 16;
struct MacIfsAndBackoffStartTraceRecord {
    TimeType duration;
    uint32_t accessCategory;
    bool frameCorrupt:8; //0:No, 1:Yes
    unsigned char padding[3];
};

const size_t MAC_IFS_AND_BACKOFF_PAUSE_TRACE_RECORD_BYTES = 16;
struct MacIfsAndBackoffPauseTraceRecord {
    TimeType leftDuration;
    uint32_t accessCategory;
    unsigned char padding[4];
};

const size_t MAC_PACKET_DEQUEUE_TRACE_RECORD_BYTES = 16;
struct MacPacketDequeueTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    uint32_t accessCategory;
};

const size_t MAC_TX_RTS_TRACE_RECORD_BYTES = 8;
struct MacTxRtsTraceRecord {
    uint32_t accessCategory;
    int32_t retry;
};

const size_t MAC_TX_BROADCAST_DATA_TRACE_RECORD_BYTES = 16;
struct MacTxBroadcastDataTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    uint32_t accessCategory;
};

const size_t MAC_TX_UNICAST_DATA_TRACE_RECORD_BYTES = 24;
struct MacTxUnicastDataTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    uint32_t accessCategory;
    int32_t shortFrameRetry;
    int32_t longFrameRetry;
};

const size_t MAC_CTSORACK_TIMEOUT_TRACE_RECORD_BYTES = 24;
struct MacCtsOrAckTimeoutTraceRecord {
    uint32_t accessCategory;
    int32_t windowSlot;
    int32_t shortFrameRetry;
    int32_t longFrameRetry;
    bool shortFrameOrNot:8; //1:short, 0:long
    unsigned char padding[7];
};

const size_t MAC_PACKET_RETRY_EXCEEDED_TRACE_RECORD_BYTES = 16;
struct MacPacketRetryExceededTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    unsigned char padding[4];
};


//phy

const size_t NOISE_START_TRACE_RECORD_BYTES = 24;
struct NoiseStartTraceRecord {
    NodeIdType sourceNodeId;
    unsigned char padding[4];
    double rxPower;
    double interferenceAndNoisePower;

};

const size_t NOISE_END_TRACE_RECORD_BYTES = 16;
struct NoiseEndTraceRecord {
    double rxPower;
    double interferenceAndNoisePower;

};

const size_t TX_START_TRACE_RECORD_BYTES = 40;
struct TxStartTraceRecord {
    NodeIdType sourceNodeId;
    unsigned char padding[4];
    uint64_t sourceNodeSequenceNumber;
    double txPower;
    uint64_t dataRate;//actual: DatarateBitsPerSecType
    TimeType duration; //long long int

};

const size_t RX_START_TRACE_RECORD_BYTES = 24;
struct RxStartTraceRecord {
    NodeIdType sourceNodeId;
    unsigned char padding[4];
    uint64_t sourceNodeSequenceNumber;
    double rxPower;
};

const size_t RX_END_TRACE_RECORD_BYTES = 16;
struct RxEndTraceRecord {
    NodeIdType sourceNodeId;
    bool error;
    bool captured;
    unsigned char padding[2];
    uint64_t sourceNodeSequenceNumber;
};


}//namespace//


#endif

