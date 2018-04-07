// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11_COMMON_H
#define DOT11_COMMON_H

#include <memory>
#include <string>
#include "scensim_mac.h"

namespace Dot11 {

using ScenSim::Packet;

using ScenSim::SimulationEngineInterface;
using std::shared_ptr;
using ScenSim::NodeIdType;
using std::string;

typedef ScenSim::SixByteMacAddressType MacAddressType;

const string parameterNamePrefix("dot11-");

// Note: 4.2 Gigabits/s limit
typedef unsigned int DatarateBitsPerSecType;

}//namespace//

#endif
