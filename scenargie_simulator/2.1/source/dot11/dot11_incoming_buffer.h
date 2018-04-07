// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11_INCOMING_BUFFER_H
#define DOT11_INCOMING_BUFFER_H

#include <memory>
#include <map>
#include "scensim_support.h"
#include "dot11_headers.h"

namespace Dot11 {

using std::unique_ptr;
using std::move;
using std::map;

using ScenSim::PacketPriorityType;
using ScenSim::CalcTwelveBitSequenceNumberDifference;


//--------------------------------------------------------------------------------------------------

class IncomingFrameBuffer {
public:
    void AddBlockAckSession(
        const MacAddressType& transmitterAddress,
        const PacketPriorityType& trafficId);

    void DeleteBlockAckSession();

    void ProcessIncomingFrame(
        const Packet& aFrame,
        const MacAddressType& transmitterAddress,
        const PacketPriorityType& trafficId,
        const unsigned short int headerSequenceNumber,
        bool& frameIsInOrder,
        bool& haveAlreadySeenThisFrame,
        vector<unique_ptr<Packet> >& bufferedPacketsToSendUp);

    void ProcessIncomingNonDataFrame(
        const MacAddressType& transmitterAddress,
        const PacketPriorityType& trafficId,
        const unsigned short int headerSequenceNumber,
        bool& haveAlreadySeenThisFrame,
        vector<unique_ptr<Packet> >& bufferedPacketsToSendUp);

    void ProcessIncomingSubframe(
        unique_ptr<Packet>& framePtr,
        const MacAddressType& transmitterAddress,
        const PacketPriorityType& trafficId,
        const unsigned short int headerSequenceNumber,
        bool& frameIsInOrder,
        bool& haveAlreadySeenThisFrame,
        vector<unique_ptr<Packet> >& bufferedPacketsToSendUp);

    bool HasFramesToSendUpStack(
        const MacAddressType& transmitterAddress,
        const PacketPriorityType& trafficId) const;

    void RetrieveFramesToSendUpStack(
        const MacAddressType& transmitterAddress,
        const PacketPriorityType& trafficId,
        vector<unique_ptr<Packet> >& framePtrs);

    void ProcessBlockAckRequestFrame(
        const BlockAcknowledgementRequestFrameType& blockAckRequest,
        vector<unique_ptr<Packet> >& bufferedPacketsToSendUp);

    void GetBlockAckInfo(
        const MacAddressType& transmitterAddress,
        const PacketPriorityType& trafficId,
        bool& success,
        unsigned short int& startSequenceNumber,
        std::bitset<BlockAckBitMapNumBits>& blockAckBitmap) const;

private:

    struct LinkInfoMapKeyType {
        MacAddressType transmitterAddress;
        PacketPriorityType trafficId;

        LinkInfoMapKeyType(
            const MacAddressType& initTransmitterAddress, const PacketPriorityType& initTrafficId)
            :
            transmitterAddress(initTransmitterAddress), trafficId(initTrafficId) {}

        bool operator<(const LinkInfoMapKeyType& right) const {
            return ((transmitterAddress < right.transmitterAddress) ||
                    ((transmitterAddress == right.transmitterAddress) && (trafficId < right.trafficId)));
        }
    };//LinkInfoMapKeyType//


    struct PacketBufferElementType {
        unsigned long long int sequenceNumber;
        unique_ptr<Packet> framePtr;

        PacketBufferElementType() : sequenceNumber(0) { }

        PacketBufferElementType(
            unique_ptr<Packet>& initFramePtr,
            const unsigned long long int initSequenceNumber)
        :
            framePtr(move(initFramePtr)),
            sequenceNumber(initSequenceNumber)
        {
        }

        void operator=(PacketBufferElementType&& right)
        {
            sequenceNumber = right.sequenceNumber;
            framePtr = move(right.framePtr);
        }

        PacketBufferElementType(PacketBufferElementType&& right) { (*this) = move(right); }

        bool operator<(const PacketBufferElementType& right) const
        {
            return ((*this).sequenceNumber > right.sequenceNumber);
        }

    };//PacketBufferElementType//


    struct BufferLinkInfo {
        bool blockAckSessionIsActive;
        std::priority_queue<PacketBufferElementType> frameBuffer;
        unsigned long long int currentLowestUnreceivedSequenceNumber;
        unsigned long long int bitmapWindowStartSequenceNumber;
        std::bitset<BlockAckBitMapNumBits> blockAckBitmap;

        BufferLinkInfo() :
            blockAckSessionIsActive(false),
            currentLowestUnreceivedSequenceNumber(1),
            bitmapWindowStartSequenceNumber(1) {}

        void operator=(BufferLinkInfo&& right)
        {
            blockAckSessionIsActive = right.blockAckSessionIsActive;
            currentLowestUnreceivedSequenceNumber = right.currentLowestUnreceivedSequenceNumber;
            bitmapWindowStartSequenceNumber = right.bitmapWindowStartSequenceNumber;
            blockAckBitmap = right.blockAckBitmap;
            frameBuffer = move(right.frameBuffer);
        }

        BufferLinkInfo(BufferLinkInfo&& right) { (*this) = move(right); }

    };//BufferLinkInfo//

    map<LinkInfoMapKeyType, BufferLinkInfo> linkInfoMap;

    void UpdateReceivedFrameBitmap(
        BufferLinkInfo& linkInfo,
        const unsigned long long int headerSequenceNumber,
        bool& frameIsInOrder,
        bool& haveAlreadySeenThisFrame,
        vector<unique_ptr<Packet> >& bufferedPacketsToSendUp);

};//IncomingFrameBuffer//


inline
unsigned short int ConvertNonWrappingSequenceNumberToRealOne(
    const unsigned long long int bigSequenceNumber)
{
    return (static_cast<unsigned short int>(bigSequenceNumber & 0xFFF));

}//ConvertSequenceNumberToNonWrappingOne//


inline
unsigned long long int ConvertSequenceNumberToNonWrappingOne(
    const unsigned long long int referenceSequenceNumber,
    const unsigned short int sequenceNumber)
{
    const int difference =
        CalcTwelveBitSequenceNumberDifference(
            sequenceNumber,
            ConvertNonWrappingSequenceNumberToRealOne(referenceSequenceNumber));

    assert((difference >= 0) || (referenceSequenceNumber >= -difference));

    return (referenceSequenceNumber + difference);

}//ConvertSequenceNumberToNonWrappingOne//



inline
void IncomingFrameBuffer::UpdateReceivedFrameBitmap(
    BufferLinkInfo& linkInfo,
    const unsigned long long int sequenceNumber,
    bool& frameIsInOrder,
    bool& haveAlreadySeenThisFrame,
    vector<unique_ptr<Packet> >& bufferedPacketsToSendUp)
{
    frameIsInOrder = false;
    haveAlreadySeenThisFrame = false;
    bufferedPacketsToSendUp.clear();

    if (sequenceNumber < linkInfo.currentLowestUnreceivedSequenceNumber) {
        haveAlreadySeenThisFrame = true;
        return;
    }//if//

    assert(linkInfo.bitmapWindowStartSequenceNumber <= linkInfo.currentLowestUnreceivedSequenceNumber);

    if ((sequenceNumber >= linkInfo.bitmapWindowStartSequenceNumber) &&
        (sequenceNumber < (linkInfo.bitmapWindowStartSequenceNumber + BlockAckBitMapNumBits))) {

        const unsigned int offset =
            static_cast<unsigned int>(sequenceNumber - linkInfo.bitmapWindowStartSequenceNumber);

        if (linkInfo.blockAckBitmap[offset]) {
            haveAlreadySeenThisFrame = true;
            return;
        }//if//

        linkInfo.blockAckBitmap.set(offset);

        if (sequenceNumber == linkInfo.currentLowestUnreceivedSequenceNumber) {
            frameIsInOrder = true;
            linkInfo.currentLowestUnreceivedSequenceNumber++;
            unsigned int i = offset + 1;
            while ((i < BlockAckBitMapNumBits) && (linkInfo.blockAckBitmap[i])) {
                assert(!linkInfo.frameBuffer.empty());
                assert(linkInfo.frameBuffer.top().sequenceNumber ==
                       linkInfo.currentLowestUnreceivedSequenceNumber);

                // const_cast due to priority_queue conflict with movable types (unique_ptr).

                bufferedPacketsToSendUp.push_back(
                    move(const_cast<PacketBufferElementType&>(linkInfo.frameBuffer.top()).framePtr));

                linkInfo.frameBuffer.pop();
                linkInfo.currentLowestUnreceivedSequenceNumber++;
                i++;
            }//while//
        }//if//
    }
    else {
        if ((sequenceNumber == linkInfo.currentLowestUnreceivedSequenceNumber) &&
            (sequenceNumber == (linkInfo.bitmapWindowStartSequenceNumber + BlockAckBitMapNumBits))) {

            // Totally inorder packet receives special case (currently no missing frames).

            assert(linkInfo.frameBuffer.empty());

            frameIsInOrder = true;
            linkInfo.currentLowestUnreceivedSequenceNumber++;
            linkInfo.bitmapWindowStartSequenceNumber++;
            // Set everything to received.
            linkInfo.blockAckBitmap.set();
        }
        else {
            // Shift bitmap

            const unsigned int shiftCount = static_cast<unsigned int>(
                (sequenceNumber - (linkInfo.bitmapWindowStartSequenceNumber + BlockAckBitMapNumBits - 1)));

            linkInfo.blockAckBitmap >>= shiftCount;
            linkInfo.blockAckBitmap.set(BlockAckBitMapNumBits - 1);
            linkInfo.bitmapWindowStartSequenceNumber += shiftCount;
            assert(linkInfo.bitmapWindowStartSequenceNumber <= linkInfo.currentLowestUnreceivedSequenceNumber);
        }//if//
    }//if//

}//UpdateReceivedFrameBitmap//




inline
void IncomingFrameBuffer::ProcessIncomingFrame(
    const Packet& aFrame,
    const MacAddressType& transmitterAddress,
    const PacketPriorityType& trafficId,
    const unsigned short int headerSequenceNumber,
    bool& frameIsInOrder,
    bool& haveAlreadySeenThisFrame,
    vector<unique_ptr<Packet> >& bufferedPacketsToSendUp)
{
    frameIsInOrder = false;
    haveAlreadySeenThisFrame = false;

    BufferLinkInfo& info = (*this).linkInfoMap[LinkInfoMapKeyType(transmitterAddress, trafficId)];

    if (!info.blockAckSessionIsActive) {
        // Regular Ack Processing (No buffering).

        const int sequenceNumberDifference =
            CalcTwelveBitSequenceNumberDifference(
                headerSequenceNumber,
                ConvertNonWrappingSequenceNumberToRealOne(
                    info.currentLowestUnreceivedSequenceNumber));

        if (sequenceNumberDifference < 0) {
            haveAlreadySeenThisFrame = true;
        }
        else {
            info.currentLowestUnreceivedSequenceNumber += (1 + sequenceNumberDifference);
            frameIsInOrder = true;
        }//if//

        return;

    }//if//

    // Block Ack Mode Processing.

    const unsigned long long int sequenceNumber =
        ConvertSequenceNumberToNonWrappingOne(
            (info.bitmapWindowStartSequenceNumber + BlockAckBitMapNumBits),
            headerSequenceNumber);

    (*this).UpdateReceivedFrameBitmap(
        info, sequenceNumber, frameIsInOrder, haveAlreadySeenThisFrame, bufferedPacketsToSendUp);

    if ((!frameIsInOrder) && (!haveAlreadySeenThisFrame)) {
        unique_ptr<Packet> frameCopyPtr(new Packet(aFrame));
        info.frameBuffer.push(PacketBufferElementType(frameCopyPtr, sequenceNumber));
    }//if//

}//ProcessIncomingFrame//



inline
void IncomingFrameBuffer::ProcessIncomingSubframe(
    unique_ptr<Packet>& framePtr,
    const MacAddressType& transmitterAddress,
    const PacketPriorityType& trafficId,
    const unsigned short int headerSequenceNumber,
    bool& frameIsInOrder,
    bool& haveAlreadySeenThisFrame,
    vector<unique_ptr<Packet> >& bufferedPacketsToSendUp)
{
    frameIsInOrder = false;
    haveAlreadySeenThisFrame = false;

    BufferLinkInfo& info = (*this).linkInfoMap[LinkInfoMapKeyType(transmitterAddress, trafficId)];

    if (!info.blockAckSessionIsActive) {

        assert(false && "This should not happen with new Forced BAR (Simple ADDBA)"); abort();

        //NotUsed // Modelling Hack to not have to do ADDBA Request/ADDBA Response.
        //NotUsed
        //NotUsed info.blockAckSessionIsActive = true;
        //NotUsed info.bitmapWindowStartSequenceNumber = info.currentLowestUnreceivedSequenceNumber;

    }//if//

    // Block Ack Mode Processing.

    const unsigned long long int sequenceNumber =
        ConvertSequenceNumberToNonWrappingOne(
            (info.bitmapWindowStartSequenceNumber + BlockAckBitMapNumBits),
            headerSequenceNumber);

    (*this).UpdateReceivedFrameBitmap(
        info, sequenceNumber, frameIsInOrder, haveAlreadySeenThisFrame, bufferedPacketsToSendUp);

    if ((!frameIsInOrder) && (!haveAlreadySeenThisFrame)) {
        info.frameBuffer.push(PacketBufferElementType(framePtr, sequenceNumber));
    }//if//

}//ProcessIncomingSubframe//



inline
void IncomingFrameBuffer::ProcessIncomingNonDataFrame(
    const MacAddressType& transmitterAddress,
    const PacketPriorityType& trafficId,
    const unsigned short int headerSequenceNumber,
    bool& haveAlreadySeenThisFrame,
    vector<unique_ptr<Packet> >& bufferedPacketsToSendUp)
{
    haveAlreadySeenThisFrame = false;

    BufferLinkInfo& info = (*this).linkInfoMap[LinkInfoMapKeyType(transmitterAddress, trafficId)];

    if (!info.blockAckSessionIsActive) {
        // Regular Ack Processing (No buffering).

        const int sequenceNumberDifference =
            CalcTwelveBitSequenceNumberDifference(
                headerSequenceNumber,
                ConvertNonWrappingSequenceNumberToRealOne(
                    info.currentLowestUnreceivedSequenceNumber));

        if (sequenceNumberDifference < 0) {
            haveAlreadySeenThisFrame = true;
        }
        else {
            info.currentLowestUnreceivedSequenceNumber += (1 + sequenceNumberDifference);
        }//if//

        return;

    }//if//

    const unsigned long long int sequenceNumber =
        ConvertSequenceNumberToNonWrappingOne(
            (info.bitmapWindowStartSequenceNumber + BlockAckBitMapNumBits),
            headerSequenceNumber);

    bool notUsed;

    (*this).UpdateReceivedFrameBitmap(
        info, sequenceNumber, notUsed, haveAlreadySeenThisFrame, bufferedPacketsToSendUp);

}//ProcessIncomingNullFrame//



inline
void IncomingFrameBuffer::ProcessBlockAckRequestFrame(
    const BlockAcknowledgementRequestFrameType& blockAckRequest,
    vector<unique_ptr<Packet> >& bufferedPacketsToSendUp)
{
    bufferedPacketsToSendUp.clear();

    BufferLinkInfo& info =
        (*this).linkInfoMap[
            LinkInfoMapKeyType(
                blockAckRequest.transmitterAddress,
                blockAckRequest.blockAckRequestControl.trafficId)];

    unsigned long long int startingSequenceNumber;

    if (!info.blockAckSessionIsActive) {
        // Modelling Hack to not have to do ADDBA Request/ADDBA Response (use BAR instead).

        startingSequenceNumber = blockAckRequest.startingSequenceControl;
        info.blockAckSessionIsActive = true;
        info.bitmapWindowStartSequenceNumber = startingSequenceNumber;
        info.currentLowestUnreceivedSequenceNumber = startingSequenceNumber;
    }
    else {
        startingSequenceNumber =
            ConvertSequenceNumberToNonWrappingOne(
                (info.bitmapWindowStartSequenceNumber + BlockAckBitMapNumBits),
                blockAckRequest.startingSequenceControl);
    }//if//

    if (info.bitmapWindowStartSequenceNumber == startingSequenceNumber) {
        return;
    }//if//

    assert(info.bitmapWindowStartSequenceNumber < startingSequenceNumber);

    const unsigned int shiftCount = static_cast<unsigned int>(
        (startingSequenceNumber - info.bitmapWindowStartSequenceNumber));

    info.bitmapWindowStartSequenceNumber = startingSequenceNumber;
    info.blockAckBitmap >>= shiftCount;

    if (info.currentLowestUnreceivedSequenceNumber < startingSequenceNumber) {

        // Deliver buffered packets with Seq # before new start sequence number .

        while ((!info.frameBuffer.empty()) &&
               (info.frameBuffer.top().sequenceNumber < startingSequenceNumber)) {

            // std::priority_queue is broken with respect to movable types (unique_ptr)

            bufferedPacketsToSendUp.push_back(
                move(const_cast<PacketBufferElementType&>(info.frameBuffer.top()).framePtr));

            info.frameBuffer.pop();

        }//while//

        info.currentLowestUnreceivedSequenceNumber = startingSequenceNumber;

        // Deliver buffered packets that are now "inorder" (gap was removed).

        unsigned int i = 0;
        while ((i < BlockAckBitMapNumBits) && (info.blockAckBitmap[i])) {
            assert(!info.frameBuffer.empty());
            assert(info.frameBuffer.top().sequenceNumber ==
                   info.currentLowestUnreceivedSequenceNumber);

            bufferedPacketsToSendUp.push_back(
                move(const_cast<PacketBufferElementType&>(info.frameBuffer.top()).framePtr));

            info.frameBuffer.pop();

            info.currentLowestUnreceivedSequenceNumber++;
            i++;
        }//while//
    }//if//

}//ProcessBlockAckRequest//



inline
void IncomingFrameBuffer::GetBlockAckInfo(
    const MacAddressType& transmitterAddress,
    const PacketPriorityType& trafficId,
    bool& success,
    unsigned short int& startSequenceNumber,
    std::bitset<BlockAckBitMapNumBits>& blockAckBitmap) const
{
    typedef map<LinkInfoMapKeyType, BufferLinkInfo>::const_iterator IterType;

    success = false;

    IterType iter = linkInfoMap.find(LinkInfoMapKeyType(transmitterAddress, trafficId));
    if (iter == linkInfoMap.end()) {
        return;
    }//if//

    const BufferLinkInfo& info = iter->second;

    startSequenceNumber =
        ConvertNonWrappingSequenceNumberToRealOne(info.bitmapWindowStartSequenceNumber);
    blockAckBitmap = info.blockAckBitmap;
    success = true;

}//GetBlockAckInfo//


inline
bool IncomingFrameBuffer::HasFramesToSendUpStack(
    const MacAddressType& transmitterAddress,
    const PacketPriorityType& trafficId) const
{
    typedef map<LinkInfoMapKeyType, BufferLinkInfo>::const_iterator IterType;

    IterType iter = linkInfoMap.find(LinkInfoMapKeyType(transmitterAddress, trafficId));
    if (iter == linkInfoMap.end()) {
        return false;
    }//if//

    const BufferLinkInfo& info = iter->second;

    return
        ((!info.frameBuffer.empty()) &&
         (info.frameBuffer.top().sequenceNumber < info.currentLowestUnreceivedSequenceNumber));

}//HasFramesToSendUpStack//

inline
void IncomingFrameBuffer::RetrieveFramesToSendUpStack(
    const MacAddressType& transmitterAddress,
    const PacketPriorityType& trafficId,
    vector<unique_ptr<Packet> >& framePtrs)
{
    framePtrs.clear();
    BufferLinkInfo& info = (*this).linkInfoMap[LinkInfoMapKeyType(transmitterAddress, trafficId)];

    while((!info.frameBuffer.empty()) &&
          (info.frameBuffer.top().sequenceNumber < info.currentLowestUnreceivedSequenceNumber)) {

        // const_cast due to priority_queue conflict with movable types.
        framePtrs.push_back(
            move(const_cast<PacketBufferElementType&>(info.frameBuffer.top()).framePtr));

        info.frameBuffer.pop();
    }//while//

}//RetrieveFramesToSendUpStack//


}//namespace//

#endif
