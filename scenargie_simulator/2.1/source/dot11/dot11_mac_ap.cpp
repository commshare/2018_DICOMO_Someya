// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "dot11_mac.h"

namespace Dot11 {

using ScenSim::ConvertToUChar;

void Dot11ApManagementController::ProcessAssociationRequestFrame(const Packet& aFrame)
{
    const AssociationRequestFrameType& associationFrame =
        aFrame.GetAndReinterpretPayloadData<AssociationRequestFrameType>();

    assert(associationFrame.managementHeader.header.frameControlField.frameTypeAndSubtype ==
           ASSOCIATION_REQUEST_FRAME_TYPE_CODE);

    const MacAddressType& transmitterAddress = associationFrame.managementHeader.transmitterAddress;

    if (!IsAnAssociatedStaAddress(transmitterAddress)) {
        (*this).AddNewAssociatedStaRecord(transmitterAddress);
    }//if//

    if (associationFrame.htCapabilitiesFrameElement.aggregateMpdusAreEnabled != 0) {
        macLayerPtr->SetMpduFrameAggregationIsEnabledFor(transmitterAddress);
    }//if//

    macLayerPtr->SendNewLinkToANodeNotificationToNetworkLayer(transmitterAddress);

    macLayerPtr->SendAssociationResponse(
        transmitterAddress,
        associatedStaInformation[transmitterAddress]->associationId);

}//ProcessAssociationRequestFrame//



void Dot11ApManagementController::ProcessReassociationRequestFrame(const Packet& aFrame)
{
    const ReassociationRequestFrameType& reassociationFrame =
        aFrame.GetAndReinterpretPayloadData<ReassociationRequestFrameType>();

    assert(reassociationFrame.managementHeader.header.frameControlField.frameTypeAndSubtype ==
           REASSOCIATION_REQUEST_FRAME_TYPE_CODE);

    const MacAddressType& transmitterAddress =
        reassociationFrame.managementHeader.transmitterAddress;


    if (!IsAnAssociatedStaAddress(transmitterAddress)) {
        (*this).AddNewAssociatedStaRecord(transmitterAddress);
    }//if//

    SendReassociationNotification(transmitterAddress, reassociationFrame.currentApAddress);

    if (reassociationFrame.htCapabilitiesFrameElement.aggregateMpdusAreEnabled != 0) {
        macLayerPtr->SetMpduFrameAggregationIsEnabledFor(transmitterAddress);
    }//if//

    macLayerPtr->SendNewLinkToANodeNotificationToNetworkLayer(transmitterAddress);
    macLayerPtr->SendReassociationResponse(
        transmitterAddress,
        associatedStaInformation[transmitterAddress]->associationId);

}//ProcessReassociationRequestFrame//



void Dot11ApManagementController::ProcessManagementFrame(const Packet& managementFrame)
{
    const ManagementFrameHeaderType& header =
        managementFrame.GetAndReinterpretPayloadData<ManagementFrameHeaderType>();

    switch(header.header.frameControlField.frameTypeAndSubtype) {
    case ASSOCIATION_REQUEST_FRAME_TYPE_CODE: {

        (*this).ProcessAssociationRequestFrame(managementFrame);

        break;
    }
    case REASSOCIATION_REQUEST_FRAME_TYPE_CODE: {

        (*this).ProcessReassociationRequestFrame(managementFrame);

        break;
    }
    case DISASSOCIATION_FRAME_TYPE_CODE: {
        if (IsAnAssociatedStaAddress(header.transmitterAddress)) {
            (*this).associationIdIsBeingUsed.reset(
                associatedStaInformation[header.transmitterAddress]->associationId);
            (*this).associatedStaInformation.erase(header.transmitterAddress);
        }
        break;
    }
    case AUTHENTICATION_FRAME_TYPE_CODE: {

        (*this).ProcessAuthenticationFrame(header.transmitterAddress);

        break;
    }
    case ASSOCIATION_RESPONSE_FRAME_TYPE_CODE:
    case REASSOCIATION_RESPONSE_FRAME_TYPE_CODE:
    case BEACON_FRAME_TYPE_CODE:
        // Ignore management frames from other access points.
        break;

    default:
        // Should not receive other management frame types
        assert(false); abort();
        break;
    }//switch//

}//ProcessManagementFrame//




void Dot11ApManagementController::ReceiveFramePowerManagementBit(
    const MacAddressType& sourceAddress,
    const bool framePowerManagementBitIsOn)
{
    typedef map<MacAddressType, shared_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;

    IterType staIterator = associatedStaInformation.find(sourceAddress);
    if (staIterator == associatedStaInformation.end()) {
        // Unknown client station, ignore.
        return;
    }//if//

    AssociatedStaInformationEntry& staInfo = *(*staIterator).second;

    if (staInfo.isInPowersaveMode == framePowerManagementBitIsOn) {
        // No change, do nothing.
        return;
    }//if//

    staInfo.isInPowersaveMode = framePowerManagementBitIsOn;

    if (framePowerManagementBitIsOn) {
        while (!staInfo.powerSavePacketBuffer.empty()) {
            PowerSavePacketBufferElemType& bufferElem = staInfo.powerSavePacketBuffer.back();

            if (bufferElem.destinationNetworkAddress == NetworkAddress::invalidAddress) {
                macLayerPtr->RequeueManagementFrame(bufferElem.packetPtr);
            }
            else {
                macLayerPtr->RequeueBufferedPacket(
                    bufferElem.packetPtr,
                    bufferElem.destinationNetworkAddress,
                    bufferElem.priority,
                    bufferElem.etherType,
                    bufferElem.timestamp,
                    bufferElem.retryTxCount,
                    bufferElem.datarateAndTxPowerAreaSpecified,
                    bufferElem.specifiedDatarateBitsPerSec,
                    bufferElem.specifiedTxPowerDbm);
            }//if//

            staInfo.powerSavePacketBuffer.pop_back();
        }//while//
    }//if//

}//ReceiveFramePowerManagementBit//


void Dot11ApManagementController::SendAuthentication(const MacAddressType& transmitterAddress)
{
    macLayerPtr->SendAuthentication(transmitterAddress);
}


void Dot11ApManagementController::SendReassociationNotification(
    const MacAddressType& staAddress,
    const MacAddressType& apAddress)
{
    // TBD: notify previous AP to disassociate

}//SendReassociationNotification//


void Dot11ApManagementController::SendBeaconFrame()
{
    typedef map<MacAddressType, shared_ptr<AssociatedStaInformationEntry> >::const_iterator IterType;

    TrafficIndicationBitMap aTrafficIndicationBitMap;

    for(IterType iter = associatedStaInformation.begin(); iter != associatedStaInformation.end(); ++iter) {
        const AssociatedStaInformationEntry& staInfo = *iter->second;
        if (!staInfo.powerSavePacketBuffer.empty()) {
            aTrafficIndicationBitMap.AddBit(staInfo.associationId);
        }//if//
    }//for//

    BeaconFrameType beaconHeader(ssid);
    beaconHeader.managementHeader.header.frameControlField.frameTypeAndSubtype = BEACON_FRAME_TYPE_CODE;
    beaconHeader.managementHeader.header.frameControlField.isRetry = 0;
    beaconHeader.managementHeader.header.duration = 0;
    beaconHeader.managementHeader.header.receiverAddress = MacAddressType::GetBroadcastAddress();
    beaconHeader.managementHeader.transmitterAddress = macLayerPtr->GetMacAddress();

    const vector<unsigned int>& channelList = macLayerPtr->GetCurrentBondedChannelList();
    for(unsigned int i = 0; (i < channelList.size()); i++) {
        beaconHeader.htOperationFrameElement.bondedChannelList[i] = ConvertToUChar(channelList[i]);
    }//for//


    if (!aTrafficIndicationBitMap.IsEmpty()) {
        unique_ptr<Packet> framePtr =
            Packet::CreatePacketWithExtraHeaderSpace(
                *simEngineInterfacePtr,
                aTrafficIndicationBitMap.GetBitMapByteVector(),
                (sizeof(TrafficIndicationMapElementHeaderType) + sizeof(BeaconFrameType)));

        TrafficIndicationMapElementHeaderType timHeader;
        timHeader.bitMapByteOffset =  aTrafficIndicationBitMap.GetStartByteOffset();
        framePtr->AddPlainStructHeader(timHeader);

        framePtr->AddPlainStructHeader(beaconHeader);
        macLayerPtr->SendManagementFrame(framePtr);
    }
    else {
        unique_ptr<Packet> framePtr = Packet::CreatePacket(*simEngineInterfacePtr, beaconHeader);
        macLayerPtr->SendManagementFrame(framePtr);
    }//if//

}//SendBeaconFrame//



}//namespace//

