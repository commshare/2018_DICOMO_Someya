// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "dot11_phy.h"
#include "dot11_info_interface.h"

namespace Dot11 {

using ScenSim::TimeType;

double Dot11InfoInterface::GetRssiOfLastFrameDbm() const
{
    return (phyLayerPtr->GetRssiOfLastFrameDbm());

}//GetRssiOfLastFrameDbm//

double Dot11InfoInterface::GetSinrOfLastFrameDb() const
{
    return (phyLayerPtr->GetSinrOfLastFrameDb());

}//GetSinrOfLastFrameDb//

unsigned int Dot11InfoInterface::GetNumberOfReceivedFrames() const
{
    return (phyLayerPtr->GetNumberOfReceivedFrames());

}//GetNumberOfReceivedFrames//

unsigned int Dot11InfoInterface::GetNumberOfFramesWithErrors() const
{
    return (phyLayerPtr->GetNumberOfFramesWithErrors());

}//GetNumberOfFramesWithErrors//

unsigned int Dot11InfoInterface::GetNumberOfSignalCaptures() const
{
    return (phyLayerPtr->GetNumberOfSignalCaptures());

}//GetNumberOfSignalCaptures//

TimeType Dot11InfoInterface::GetTotalIdleChannelTime() const
{
    return (phyLayerPtr->GetTotalIdleChannelTime());

}//GetTotalIdleChannelTime//

TimeType Dot11InfoInterface::GetTotalBusyChannelTime() const
{
    return (phyLayerPtr->GetTotalBusyChannelTime());

}//GetTotalBusyChannelTime//

TimeType Dot11InfoInterface::GetTotalTransmissionTime() const
{
    return (phyLayerPtr->GetTotalTransmissionTime());

}//GetTotalTransmissionTime//

}//namespace//
