// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef MULTIAGENT_TRACEDEFS_H
#define MULTIAGENT_TRACEDEFS_H

#include <cstdlib>

namespace MultiAgent {

static const size_t MUTLTAGENT_TRACE_RECORD_BYTES = 8;
struct MultiAgentTraceRecord {
    double value;

    MultiAgentTraceRecord() {}
    MultiAgentTraceRecord(const double initValue) : value(initValue) {}
};


}//namespace//


#endif

