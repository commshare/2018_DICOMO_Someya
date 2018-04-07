// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11_HEADERS_H
#define DOT11_HEADERS_H

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


#include <string>
#include <bitset>
#include "scensim_support.h"
#include "scensim_queues.h"
#include "scensim_prop.h"
#include "dot11_common.h"

namespace Dot11 {

using std::string;
using std::vector;
using std::array;

using ScenSim::EtherTypeFieldType;
using ScenSim::MaxNumBondedChannels;
using ScenSim::OneZeroedByteType;
using ScenSim::TwoZeroedBytesType;
using ScenSim::FourZeroedBytesType;
using ScenSim::EightZeroedBytesType;
using ScenSim::CalcTwelveBitSequenceNumberDifference;

// Duration in us.

typedef uint16_t DurationFieldType;
const DurationFieldType MaxDurationFieldValue = 32768;


typedef uint16_t AssociationIdType;
const AssociationIdType MaxAssociationId = 2007;

const unsigned char ASSOCIATION_REQUEST_FRAME_TYPE_CODE = 0x00; // 00 0000;
const unsigned char ASSOCIATION_RESPONSE_FRAME_TYPE_CODE = 0x01; // 00 0001;
const unsigned char REASSOCIATION_REQUEST_FRAME_TYPE_CODE = 0x02; // 00 0010;
const unsigned char REASSOCIATION_RESPONSE_FRAME_TYPE_CODE = 0x03; // 00 0011;
const unsigned char BEACON_FRAME_TYPE_CODE = 0x08; // 00 1000;
const unsigned char DISASSOCIATION_FRAME_TYPE_CODE = 0x0A; // 00 1010;
const unsigned char AUTHENTICATION_FRAME_TYPE_CODE = 0x0B; // 00 1011
const unsigned char NULL_FRAME_TYPE_CODE = 0x24; // 10 0100;
const unsigned char BLOCK_ACK_REQUEST_FRAME_TYPE_CODE = 0x18;  // 01 1000
const unsigned char BLOCK_ACK_FRAME_TYPE_CODE = 0x19;  // 01 1001
const unsigned char RTS_FRAME_TYPE_CODE  = 0x1B; // 01 1011;
const unsigned char CTS_FRAME_TYPE_CODE  = 0x1C; // 01 1100;
const unsigned char ACK_FRAME_TYPE_CODE  = 0x1D; // 01 1101;
const unsigned char QOS_DATA_FRAME_TYPE_CODE = 0x28; // 10 1000;
const unsigned char POWER_SAVE_POLL_FRAME_TYPE_CODE = 0x1A;  // 01 1010;


inline
bool IsAManagementFrameTypeCode(const unsigned char frameTypeCode)
{
    // True if top 2 bits (out of 6) are 0.

    return ((frameTypeCode & 0x30) == 0x0);
}



inline
string ConvertToDot11FrameTypeName(const unsigned char frameTypeCode)
{
    switch (frameTypeCode) {
    case ASSOCIATION_REQUEST_FRAME_TYPE_CODE: return "Association-Request";
    case ASSOCIATION_RESPONSE_FRAME_TYPE_CODE: return "Association-Response";
    case REASSOCIATION_REQUEST_FRAME_TYPE_CODE: return "Reassociation-Request";
    case REASSOCIATION_RESPONSE_FRAME_TYPE_CODE: return "Reassociation-Response";
    case BEACON_FRAME_TYPE_CODE: return "Beacon";
    case DISASSOCIATION_FRAME_TYPE_CODE: return "Disassociation";
    case AUTHENTICATION_FRAME_TYPE_CODE: return "Authentication";
    case RTS_FRAME_TYPE_CODE: return "RTS";
    case CTS_FRAME_TYPE_CODE: return "CTS";
    case ACK_FRAME_TYPE_CODE: return "ACK";
    case QOS_DATA_FRAME_TYPE_CODE: return "Data";
    case NULL_FRAME_TYPE_CODE: return "Null";
    case BLOCK_ACK_REQUEST_FRAME_TYPE_CODE: return "BlockACK-Request";
    case BLOCK_ACK_FRAME_TYPE_CODE: return "BlockACK";

    default:
        assert(false); abort();
        break;
    }//switch//

    return "";

}//ConvertToDot11FrameTypeName//



struct FrameControlFieldType {
    enum ToDsFromDsChoicesType {
        NotWirelessDistributionSystemFrame = 0,
        FromWirelessDistributionSystemFrame = 1,
        ToWirelessDistributionSystemFrame = 2,
        WirelessDistributionSystemFrame = 3
    };

    unsigned char notUsed1:2;

    unsigned char frameTypeAndSubtype:6;

    unsigned char toDsFromDs:2;

    unsigned char notUsed2:1;

    unsigned char isRetry:1;

    unsigned char powerManagement:1;

    unsigned char notUsed_moreData:1;

    unsigned char notUsed3:1;
    unsigned char notUsed4:1;

    FrameControlFieldType()
        : isRetry(0), toDsFromDs(NotWirelessDistributionSystemFrame),
          powerManagement(0),
          notUsed1(0), notUsed2(0), notUsed3(0), notUsed4(0),
          notUsed_moreData(0)
    {}
};//FrameControlFieldType//


struct CommonFrameHeaderType {
    FrameControlFieldType frameControlField;
    DurationFieldType duration;
    MacAddressType receiverAddress;

    CommonFrameHeaderType() : duration(0) {}
};


// Request To Send aka RTS

struct RequestToSendFrameType {
    CommonFrameHeaderType header;
    MacAddressType transmitterAddress;

    FourZeroedBytesType notUsed_FCS;
};


// Clear To Send aka CTS

struct ClearToSendFrameType {
    CommonFrameHeaderType header;

    FourZeroedBytesType notUsed_FCS;

};


// Power Save Poll aka PS-Poll

struct PowerSavePollFrameType {
    CommonFrameHeaderType header;
    MacAddressType transmitterAddress;
    FourZeroedBytesType notUsed_FCS;
};


inline
AssociationIdType GetAssociationIdFromCommonFrameHeaderDurationField(
    const CommonFrameHeaderType& header)
{
    assert(header.frameControlField.frameTypeAndSubtype == POWER_SAVE_POLL_FRAME_TYPE_CODE);
    assert(header.duration <= MaxAssociationId);

    return (static_cast<AssociationIdType>(header.duration));
}


// Data Frame Types

const unsigned short int MaxSequenceNumber = 4095;

struct SequenceControlFieldType {
    unsigned short int sequenceNumber:12;

    unsigned short int notUsed:4;

};


// QoS = "Quality of Service"
// "Traffic ID" aka TID.

// Jay: Traffic ID seems only useful if 802.11 is used for multiple "link layer" hops
//      without going to the official network layer (which has the priority).
//      The only real use of ackPolicy currently would be to send unicast packets
//      without ack-ing.  Thus, will leave the QoS Control Field stuff commented out for now.
//
// const unsigned short NORMAL_ACK_POLICY = 0x0; // 0b00;
// const unsigned short DONT_ACK_POLICY = 0x2;   // 0b10;
//

struct QosControlFieldType {
    unsigned char trafficId:4;
    unsigned char notUsed1:1;
    unsigned char notUsed_ackPolicy:2;
    unsigned char reserved:1;
    unsigned char notUsed2:8;

    QosControlFieldType() : reserved(0), notUsed_ackPolicy(0), notUsed1(0), notUsed2(0) { }
};

struct Ieee802p2LinkLayerHeaderType {
    TwoZeroedBytesType notUsed;
    EtherTypeFieldType cheatingVlanTagTpid;
    unsigned char cheatingVlanTagPcp:3;
    unsigned char cheatingVlanTagCfi:1;
    unsigned char cheatingVlanTagVidHigh:4;
    unsigned char cheatingVlanTagVidLow:8;
    EtherTypeFieldType etherType;
};

struct QosDataFrameHeaderType {
    CommonFrameHeaderType header;
    MacAddressType transmitterAddress;
    MacAddressType sourceOrDestinationAddress;
    SequenceControlFieldType sequenceControlField;
    QosControlFieldType qosControlField;
    FourZeroedBytesType notUsed_FCS;
    Ieee802p2LinkLayerHeaderType linkLayerHeader;
};

struct QosNullFrameHeaderType {
    CommonFrameHeaderType header;
    MacAddressType transmitterAddress;
    MacAddressType notUsed_Address3;
    SequenceControlFieldType sequenceControlField;
    QosControlFieldType qosControlField;
    FourZeroedBytesType notUsed_FCS;
};

struct DistributionSystemQosDataFrameHeaderType {
    CommonFrameHeaderType header;
    MacAddressType transmitterAddress;
    MacAddressType destinationAddress;
    SequenceControlFieldType sequenceControlField;
    MacAddressType sourceAddress;
    QosControlFieldType qosControlField;
    FourZeroedBytesType notUsed_FCS;
    Ieee802p2LinkLayerHeaderType linkLayerHeader;
};


// Acknowledge aka ACK

struct AcknowledgementAkaAckFrameType {
    CommonFrameHeaderType header;

    FourZeroedBytesType notUsed_FCS;
};


//---------------------------------------------------------

const unsigned int BlockAckBitMapNumBits = 64;


struct BlockAckOrBlockAckRequestControlFieldType {
    unsigned char notUsed_blockAckPolicy:1;    // Always "Normal".
    unsigned char notUsed_multiTid:1;  // Never multi-TID.
    unsigned char notUsed_compressedBitmap:1;  // Always compressed.
    unsigned char reserved1:5;
    unsigned char reserved2:4;
    unsigned char trafficId:4;

    BlockAckOrBlockAckRequestControlFieldType() : notUsed_blockAckPolicy(0),  notUsed_multiTid(0),
        notUsed_compressedBitmap(0), reserved1(0), reserved2(0), trafficId(0) { }
};


struct BlockAcknowledgementFrameType {
    CommonFrameHeaderType header;
    MacAddressType transmitterAddress;
    BlockAckOrBlockAckRequestControlFieldType blockAckControl;
    unsigned short reserved:4;
    unsigned short startingSequenceControl:12;
    FourZeroedBytesType notUsed_FCS;
    std::bitset<BlockAckBitMapNumBits> blockAckBitmap;  // order in struct swapped for 8 byte alignment.

    BlockAcknowledgementFrameType() : reserved(0) {
        assert(sizeof(blockAckBitmap) == 8);
        assert(sizeof(*this) == 32);
    }

    bool IsAcked(const unsigned short sequenceNumber) const;
};


inline
bool BlockAcknowledgementFrameType::IsAcked(const unsigned short int sequenceNumber) const
{
    const int offset =
        CalcTwelveBitSequenceNumberDifference(sequenceNumber, startingSequenceControl);

    assert(offset >= 0);
    if (offset >= BlockAckBitMapNumBits) {
        return false;
    }//if//

    return (blockAckBitmap[offset]);
}


struct BlockAcknowledgementRequestFrameType {
    CommonFrameHeaderType header;
    MacAddressType transmitterAddress;
    BlockAckOrBlockAckRequestControlFieldType blockAckRequestControl;
    unsigned short reserved:4;
    unsigned short startingSequenceControl:12;
    FourZeroedBytesType notUsed_FCS;

    BlockAcknowledgementRequestFrameType() : reserved(0) { }
};



//---------------------------------------------------------


struct ManagementFrameHeaderType {
    CommonFrameHeaderType header;
    MacAddressType transmitterAddress;
    MacAddressType notUsed_Address3;
    SequenceControlFieldType sequenceControlField;
    FourZeroedBytesType notUsed_FCS;
};

//---------------------------------------------------------


struct MpduDelimiterFrameType {
    unsigned short int notUsed_EndOfFrameBitAkaEof:1;
    unsigned short int reserved:1;
    // Assume expansion 12->14 bit (11AC). Do not limit to 4k.
    unsigned short int lengthBytes:14;
    OneZeroedByteType notUsed_Crc;
    OneZeroedByteType notUsed_signature;

    MpduDelimiterFrameType() : notUsed_EndOfFrameBitAkaEof(0), reserved(0) { }
};


//---------------------------------------------------------

// Using Block Ack Request (BAR) in place of Add Block Ack Session Request (ADDBA).

//NotUsed struct BlockAckParameterSetFieldType
//NotUsed {
//NotUsed     unsigned short notUsed_aMsduSupported:1;  // Always Supported.
//NotUsed     unsigned short notUsed_blockAckPolicy:1;  // Never delayed.
//NotUsed     unsigned short trafficId:4;
//NotUsed     unsigned short notUsed_bufferSize:10;
//NotUsed };

//NotUsed // Aka ADDBA Request.
//NotUsed
//NotUsed struct AddBlockAckSessionRequestFrameType {
//NotUsed     static const unsigned int frameSizeBytes =
//NotUsed         sizeof(ManagementFrameHeaderType) + 1 + 1 + 1 + 2 + 2 + 2;
//NotUsed
//NotUsed     static const unsigned int zeroedBytesSize =
//NotUsed         (frameSizeBytes - sizeof(ManagementFrameHeaderType) - 3);
//NotUsed
//NotUsed     ManagementFrameHeaderType header;
//NotUsed
//NotUsed     unsigned short reserved:4;
//NotUsed     unsigned short startingSequenceControl:12;
//NotUsed
//NotUsed     unsigned char trafficId;
//NotUsed
//NotUsed
//NotUsed     array<unsigned char, zeroedBytesSize> zeroedBytes;
//NotUsed
//NotUsed     AddBlockAckSessionRequestFrameType() {
//NotUsed         assert(sizeof(AddBlockAckSessionRequestFrameType) == frameSizeBytes);
//NotUsed         zeroedBytes.fill(0);
//NotUsed     }
//NotUsed
//NotUsed };//AddBlockAckSessionRequestFrameType//
//NotUsed
//NotUsed
//NotUsed // Aka ADDBA Response.
//NotUsed
//NotUsed struct AddBlockAckSessionResponseFrameType {
//NotUsed     static const unsigned int frameSizeBytes =
//NotUsed         sizeof(ManagementFrameHeaderType) + 1 + 1 + 1 + 2 + 2 + 2;;
//NotUsed
//NotUsed     static const unsigned int zeroedBytesSize =
//NotUsed         (frameSizeBytes - sizeof(ManagementFrameHeaderType) - 1);
//NotUsed
//NotUsed     CommonFrameHeaderType header;
//NotUsed
//NotUsed     unsigned char trafficId;
//NotUsed
//NotUsed     array<unsigned char, zeroedBytesSize> zeroedBytes;
//NotUsed
//NotUsed     AddBlockAckSessionResponseFrameType() {
//NotUsed         assert(sizeof(AddBlockAckSessionResponseFrameType) == frameSizeBytes);
//NotUsed         zeroedBytes.fill(0);
//NotUsed     }
//NotUsed };//AddBlockAckSessionResponseFrameType//


const string SsidWildcardString = "";
const int SSID_LENGTH = 32;
const int SUPPORTED_RATES_LENGTH = 8;


//fixed length: 34(1+1+32) bytes
struct SsidFieldType {
    unsigned char elementId;
    unsigned char length;
    char ssid[SSID_LENGTH];

    bool IsWildcardSsid() const { return (length == 0); }

    bool IsEqualTo(const string& aString) const {
        if (aString.length() != length) {
            return false;
        }
        for(unsigned int i = 0; (i < length); i++) {
            if (aString[i] != ssid[i]) {
                return false;
            }
        }//for//
        return true;
    }//IsEqualTo//

    SsidFieldType()
        :
        length(0) {}

    SsidFieldType(const string& ssidString) {

        assert(ssidString.length() <= SSID_LENGTH);
        length = static_cast<unsigned char>(ssidString.length());
        ssidString.copy(ssid, length);
    }

};//SsidFieldType//


//fixed length: 10(1+1+8) bytes
struct SupportedRatesFieldType {
    unsigned char elementId;
    unsigned char length;
    unsigned char supportedRates[SUPPORTED_RATES_LENGTH];
};//SupportedRatesFieldType//


// "Legacy" = pre-802.11n.

struct LegacyBeaconFrameType {
    ManagementFrameHeaderType managementHeader;
    EightZeroedBytesType notUsed_Timestamp;
    TwoZeroedBytesType notUsed_BeaconInterval;
    TwoZeroedBytesType notUsed_CapabilityInformation;

    SsidFieldType ssidElement;
    SupportedRatesFieldType notUsed_SupportedRatesElement;

    LegacyBeaconFrameType(const string& ssidString) : ssidElement(ssidString) {}

};//LegacyBeaconFrameType//


struct HtCapabilitiesFrameElementType {
    static const unsigned int elementSize = 28;
    static const unsigned int zeroedBytesSize = elementSize - 1;

    unsigned char aggregateMpdusAreEnabled;

    array<unsigned char, zeroedBytesSize> zeroedBytes;

    HtCapabilitiesFrameElementType() : aggregateMpdusAreEnabled(0) {
        assert(sizeof(HtCapabilitiesFrameElementType) == elementSize);
        zeroedBytes.fill(0);
    }

};//HtCapabilitiesFrameElementType//



struct HtOperationFrameElementType {
    static const unsigned int elementSize = 24;
    static const unsigned int zeroedBytesSize = elementSize - MaxNumBondedChannels;
    array<unsigned char, zeroedBytesSize> zeroedBytes;

    // Abstracted:
    array<unsigned char, MaxNumBondedChannels> bondedChannelList;
    unsigned int GetNumberBondedChannels() const
    {
        for(unsigned int i = 0; (i < MaxNumBondedChannels); i++) {
            if (bondedChannelList[i] == UCHAR_MAX) {
                return (i);
            }//if//
        }//for//
        return (MaxNumBondedChannels);
    }

    HtOperationFrameElementType() {
        assert(sizeof(HtOperationFrameElementType) == elementSize);
        zeroedBytes.fill(0);
        bondedChannelList.fill(UCHAR_MAX);
    }

};//HtOperationFrameElementType//


struct BeaconFrameType {
    ManagementFrameHeaderType managementHeader;
    EightZeroedBytesType notUsed_Timestamp;
    TwoZeroedBytesType notUsed_BeaconInterval;
    TwoZeroedBytesType notUsed_CapabilityInformation;

    SsidFieldType ssidElement;
    SupportedRatesFieldType notUsed_SupportedRatesElement;

    HtCapabilitiesFrameElementType htCapabilitiesFrameElement;
    HtOperationFrameElementType htOperationFrameElement;

    BeaconFrameType() {}

    BeaconFrameType(const string& ssidString) : ssidElement(ssidString) {}

};//BeaconFrameType//


inline
bool IsABeaconFrame(const Packet& aFrame)
{
    const CommonFrameHeaderType& header =
        aFrame.GetAndReinterpretPayloadData<CommonFrameHeaderType>();

    return (header.frameControlField.frameTypeAndSubtype == BEACON_FRAME_TYPE_CODE);
}


//NotUsed: struct ProbeRequestWildcardSsidFrameType {
//NotUsed:
//NotUsed:     ManagementFrameHeaderType managementHeader;
//NotUsed:     SupportedRatesFieldType NotUsed:_SupportedRatesElement;
//NotUsed:     TwoZeroedBytesType wildcardSsid;
//NotUsed:
//NotUsed:
//NotUsed: };//ProbeRequestWildcardSsidFrameType//


struct ProbeRequestFrameType {

    ManagementFrameHeaderType managementHeader;
    SupportedRatesFieldType notUsed_SupportedRatesElement;
    SsidFieldType ssidElement;

    HtCapabilitiesFrameElementType htCapabilitiesFrameElement;

};//ProbeRequestFrameType//


typedef BeaconFrameType ProbeResponseFrameType;


struct AssociationRequestFrameType {
    ManagementFrameHeaderType managementHeader;
    TwoZeroedBytesType notUsed_CapabilityInformation;
    TwoZeroedBytesType notUsed_ListenInterval;
    SsidFieldType ssidElement;
    SupportedRatesFieldType notUsed_SupportedRatesElement;

    HtCapabilitiesFrameElementType htCapabilitiesFrameElement;

    AssociationRequestFrameType() { }

    AssociationRequestFrameType(const string& ssidString)
        :
        ssidElement(ssidString)
    { }

};//AssociationRequestFrameType//


struct AssociationResponseFrameType {
    ManagementFrameHeaderType managementHeader;
    TwoZeroedBytesType notUsed_CapabilityInformation;
    TwoZeroedBytesType notUsed_StatusCode;
    AssociationIdType associationId;
    SupportedRatesFieldType notUsed_SupportedRatesElement;

    HtCapabilitiesFrameElementType htCapabilitiesFrameElement;

};//AssociationResponseFrameType//


struct ReassociationRequestFrameType {
    ManagementFrameHeaderType managementHeader;
    TwoZeroedBytesType notUsed_CapabilityInformation;
    TwoZeroedBytesType notUsed_ListenInterval;
    MacAddressType currentApAddress;
    SsidFieldType ssidElement;
    SupportedRatesFieldType notUsed_SupportedRatesElement;

    HtCapabilitiesFrameElementType htCapabilitiesFrameElement;

    ReassociationRequestFrameType() { }

    ReassociationRequestFrameType(const string& ssidString)
        :
        ssidElement(ssidString)
    { }

}; // ReassociationRequestFrameType

typedef AssociationResponseFrameType ReassociationResponseFrameType;

struct DisassociationFrameType {
    ManagementFrameHeaderType managementHeader;
    TwoZeroedBytesType notUsed_reasonCode;

}; //DisassociationFrameType


struct AuthenticationFrameType {
    ManagementFrameHeaderType managementHeader;
    TwoZeroedBytesType notUsed_AlgorithmNumber;
    TwoZeroedBytesType notUsed_SequenceNumber;
    TwoZeroedBytesType notUsed_StatusCode;

};//AuthenticationFrameType//


struct CommonFrameHeaderWithTransmitterAddressFieldOverlayType {
    CommonFrameHeaderType header;
    MacAddressType transmitterAddress;
};


inline
void CheckFrameHeaderDefinitions()
{
    assert((sizeof(QosNullFrameHeaderType) + sizeof(Ieee802p2LinkLayerHeaderType)) ==
           sizeof(QosDataFrameHeaderType));

    QosDataFrameHeaderType h1;
    QosNullFrameHeaderType& h2 = *reinterpret_cast<QosNullFrameHeaderType*>(&h1);
    assert(&h1.header == &h2.header);
    assert(sizeof(h1.header) == sizeof(h2.header));
    assert(&h1.transmitterAddress == &h2.transmitterAddress);
    assert(sizeof(h1.transmitterAddress) == sizeof(h2.transmitterAddress));
    assert(&h1.sequenceControlField == &h2.sequenceControlField);
    assert(sizeof(h1.sequenceControlField) == sizeof(h2.sequenceControlField));
    assert(&h1.qosControlField == &h2.qosControlField);
    assert(sizeof(h1.qosControlField) == sizeof(h2.qosControlField));

}//CheckQosNullFrameHeaderDefinition//


inline
MacAddressType GetTransmitterAddressFromFrame(const Packet& aFrame)
{
    const CommonFrameHeaderWithTransmitterAddressFieldOverlayType& aHeader =
        aFrame.GetAndReinterpretPayloadData<CommonFrameHeaderWithTransmitterAddressFieldOverlayType>();

    if (aHeader.header.frameControlField.frameTypeAndSubtype == CTS_FRAME_TYPE_CODE) {

        // Transmitter address in header.
        return MacAddressType::invalidMacAddress;
    }//if//

    return (aHeader.transmitterAddress);

}//GetTransmitterAddressFromFrame//



struct TrafficIndicationMapElementHeaderType {
    FourZeroedBytesType notUsed;
    unsigned char bitMapByteOffset;
};


class TrafficIndicationBitMap {
public:
    TrafficIndicationBitMap() { (*this).Clear(); }

    void Clear();

    bool IsEmpty() const { return (bitVector.empty()); }

    void AddBit(const AssociationIdType associationId);
    bool BitIsSet(const AssociationIdType associationId) const;

    static
    bool BitIsSetInRawBitMap(
        const unsigned char rawBitMapData[],
        const unsigned int rawBitMapSizeBytes,
        const unsigned char bitMapByteOffset,
        const AssociationIdType associationId);

    const vector<unsigned char>& GetBitMapByteVector() const { return bitVector; }

    unsigned char GetStartByteOffset() const { return (static_cast<unsigned char>(startByteOffset)); }

private:
    unsigned int startByteOffset;
    vector<unsigned char> bitVector;

    void SetBit(const AssociationIdType associationId);

};//TrafficIndicationBitMap//


inline
void TrafficIndicationBitMap::Clear()
{
    startByteOffset = 0;
    bitVector.clear();
}


inline
void TrafficIndicationBitMap::SetBit(const AssociationIdType associationId)
{
    const unsigned int byteOffset = associationId / 8;
    assert(byteOffset >= startByteOffset);

    const unsigned int byteIndex = (byteOffset - startByteOffset);
    const unsigned int bitPos = associationId % 8;

    assert(byteIndex < bitVector.size());

    bitVector[byteIndex] |= (1 << bitPos);

}//SetBit//


inline
void TrafficIndicationBitMap::AddBit(const AssociationIdType associationId)
{
    assert(associationId != 0);

    const unsigned int byteOffset = associationId / 8;
    const unsigned int endbyteOffset = static_cast<unsigned int>(startByteOffset + bitVector.size() - 1);

    if (bitVector.empty()) {
        startByteOffset = byteOffset;
        bitVector.resize(1, 0);
    }
    else if (startByteOffset > byteOffset) {

        const unsigned int oldstartByteOffset = startByteOffset;
        (*this).startByteOffset = byteOffset;
        const size_t originalSize = bitVector.size();

        bitVector.resize((bitVector.size() + (oldstartByteOffset - startByteOffset)), 0);

        std::copy_backward(
            bitVector.begin(),
            (bitVector.begin() + originalSize),
            (bitVector.begin() + (oldstartByteOffset - startByteOffset)));
    }
    else if (endbyteOffset < byteOffset) {
        bitVector.resize(((byteOffset - startByteOffset) + 1), 0);
    }//if/

    (*this).SetBit(associationId);

    assert(startByteOffset <= UCHAR_MAX);

}//AddBit//


inline
bool TrafficIndicationBitMap::BitIsSet(const AssociationIdType associationId) const
{
    const unsigned int byteOffset = associationId / 8;
    const unsigned int endByteOffset = static_cast<unsigned int>(startByteOffset + bitVector.size() - 1);

    if ((byteOffset < startByteOffset) || (byteOffset > endByteOffset)) {
         return false;
    }
    else {
        const unsigned int bitPos = associationId % 8;
        return ((bitVector[(byteOffset - startByteOffset)] & (1 << bitPos)) == 1);
    }//if//


}//BitIsSet//

inline
bool TrafficIndicationBitMap::BitIsSetInRawBitMap(
    const unsigned char rawBitMapData[],
    const unsigned int rawBitMapSizeBytes,
    const unsigned char bitMapByteOffset,
    const AssociationIdType associationId)
{
    const unsigned int byteOffset = associationId / 8;
    const unsigned int endByteOffset = bitMapByteOffset + rawBitMapSizeBytes - 1;

    if ((byteOffset < bitMapByteOffset) || (byteOffset > endByteOffset)) {
         return false;
    }
    else {
        const unsigned int bitPos = associationId % 8;
        return ((rawBitMapData[(byteOffset - bitMapByteOffset)] & (1 << bitPos)) == 1);
    }//if//

}//BitIsSetInRawBitMap//



}//namespace//


#endif
