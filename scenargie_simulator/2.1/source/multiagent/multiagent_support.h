// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef MULTIAGENT_SUPPORT_H
#define MULTIAGENT_SUPPORT_H

#include <map>
#include <vector>
#include <list>
#include "multiagent_common.h"

namespace MultiAgent {

using std::map;
using std::pair;
using std::make_pair;
using std::vector;
using std::list;

static inline
pair<string, string> SeparateString(const string& aString, const string& separator)
{
    pair<string, string> tagAndValue;

    const size_t separatorPos = aString.find(separator);

    if (separatorPos != string::npos) {
        tagAndValue.first = TrimmedString(aString.substr(0, separatorPos));
        tagAndValue.second = TrimmedString(aString.substr(separatorPos + separator.length()));
    }

    return tagAndValue;
}

static inline
int CalculateNumberChars(const string& aString, const char aCharactor)
{
    int numberChars = 0;
    for(size_t i = 0; i < aString.length(); i++) {
        if (aString[i] == aCharactor) {
            numberChars++;
        }
    }

    return numberChars;
}

static inline
void TokenizeToTrimmedLowerStringWithArc(
    const string& aString,
    const string& deliminator,
    deque<string>& tokens)
{
    tokens.clear();

    size_t posOfReading = 0;
    size_t posOfDelim = aString.find_first_of(deliminator);
    int numberRemainingArcs = 0;

    while (posOfDelim != string::npos) {

        const string token =
            TrimmedString(aString.substr(posOfReading, posOfDelim - posOfReading));

        if (numberRemainingArcs == 0) {

            if (!token.empty()) {
                tokens.push_back(token);
            }
        } else {

            assert(!tokens.empty());
            tokens.back() += ',' + token;
        }

        numberRemainingArcs += CalculateNumberChars(token, '(') - CalculateNumberChars(token, ')');

        ConvertStringToLowerCase(tokens.back());

        posOfReading = posOfDelim + 1;
        posOfDelim = aString.find_first_of(deliminator, posOfReading);
    }

    const size_t lastTokenPos =
        aString.find_first_not_of(" ", posOfReading);

    if (lastTokenPos != string::npos) {
        const string lastOneToken = aString.substr(
            lastTokenPos,
            aString.find_last_not_of(" ") - lastTokenPos + 1);

        if (numberRemainingArcs == 0) {
            tokens.push_back(lastOneToken);
        } else {
            tokens.back() += ',' + lastOneToken;
        }

        ConvertStringToLowerCase(tokens.back());
    }
}

}//namespace

#endif
