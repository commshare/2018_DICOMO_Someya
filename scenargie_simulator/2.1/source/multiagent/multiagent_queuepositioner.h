// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef MULTIAGENT_QUEUEPOSITIONER_H
#define MULTIAGENT_QUEUEPOSITIONER_H

#include "multiagent_agentsim.h"

namespace MultiAgent {

// "Ped" = Pedestrian

class PedQueuePositioner {
public:
    virtual const Vertex GetPosition(const unsigned int queuePositionIndex) const = 0;
    virtual ~PedQueuePositioner() { }

};//PedQueuePositioner//


class LinePedQueuePositioner: public PedQueuePositioner {
public:
    LinePedQueuePositioner(
        const vector<Vertex>& lineSegmentPoints,
        const unsigned int peoplePerRow,
        const double& rowSeparation,
        const double& columnSeparation);

    virtual const Vertex GetPosition(const unsigned int queuePositionIndex) const override;

private:
    vector<Vertex> lineSegmentPoints;
    vector<double> distanceForPoints;
    unsigned int peoplePerRow;
    double rowSeparation;
    double columnSeparation;
};


inline
LinePedQueuePositioner::LinePedQueuePositioner(
    const vector<Vertex>& initLineSegmentPoints,
    const unsigned int initPeoplePerRow,
    const double& initRowSeparation,
    const double& initColumnSeparation)
    :
    lineSegmentPoints(initLineSegmentPoints),
    peoplePerRow(initPeoplePerRow),
    rowSeparation(initRowSeparation),
    columnSeparation(initColumnSeparation)
{
    distanceForPoints.resize(lineSegmentPoints.size());

    distanceForPoints[0] = 0.0;
    for(unsigned int i = 1; (i < lineSegmentPoints.size()); i++) {
        distanceForPoints[i] =
            distanceForPoints[i-1] +
            lineSegmentPoints[i-1].DistanceTo(lineSegmentPoints[i]);
    }//for//
}


inline
const Vertex LinePedQueuePositioner::GetPosition(const unsigned int queuePositionIndex) const
{
    const unsigned int row = queuePositionIndex / peoplePerRow;

    const double rowDistance = row * rowSeparation;

    if (rowDistance >= distanceForPoints.back()) {

        // All people pile on same point when queue length exceeded.

        return (lineSegmentPoints.back());
    }//if//

    unsigned int segmentIndex = UINT_MAX;

    for(unsigned int i = 0; (i < (distanceForPoints.size()-1)); i++) {
        if (rowDistance < distanceForPoints[i+1]) {
            segmentIndex = i;
        }//if//
    }//for//

    assert(segmentIndex != UINT_MAX);

    const Vertex segmentDirection =
        (lineSegmentPoints[segmentIndex+1] - lineSegmentPoints[segmentIndex]).Normalized();

    const Vertex columnVertex = lineSegmentPoints[segmentIndex] + (segmentDirection * rowDistance);

    const unsigned int column =  queuePositionIndex % peoplePerRow;

    const unsigned int halfRowWidth = (peoplePerRow / 2);

    if (column == 0) {
        return (columnVertex);
    }//if//

    const Vertex perpendicularDirection = segmentDirection.NormalVector();

    // Column position 0 is always the center of the line, i.e. first person in
    // queue is at the start of the line.

    double columnOffset;
    if (column <= halfRowWidth) {
        columnOffset = column * columnSeparation;
    }
    else {
        columnOffset = -((column - halfRowWidth) * columnSeparation);
    }//if//

    return (columnVertex + (perpendicularDirection * columnOffset));

}//GetPosition//




//----------------------------------------------------------------------------------------


class GisObjectEntranceQueue {
public:
    GisObjectEntranceQueue(
        unique_ptr<PedQueuePositioner>& queuePositionerPtr,
        const TimeType& timestepDuration,
        const double& maxPeopleFlowRatePerSec);

    void SetPositioner(unique_ptr<PedQueuePositioner>& positionerPtr)
        { queuePositionerPtr = move(positionerPtr); }

    void SetMaxPeopleFlowRate(const double& newMaxPeopleFlowRatePerSec) {
        maxPeopleFlowRatePerSec = newMaxPeopleFlowRatePerSec;
        
        if (maxPeopleFlowRatePerSec == DBL_MAX) {
            maxAccumulatedFlow = UINT_MAX;
            accumulatedFlow = DBL_MAX;
        }
        else {
            maxAccumulatedFlow = static_cast<unsigned int>(ceil(maxPeopleFlowRatePerSec * timestepDurationSecs));

            if (accumulatedFlow > maxAccumulatedFlow) {
                accumulatedFlow = maxAccumulatedFlow;
            }//if//
        }//if//
    }

    bool IsEmpty() const { return (queue.empty()); }
    unsigned int CurrentNumberQueuedPeople() const
        { return (static_cast<unsigned int>(queue.size())); }

    const Vertex& GetPositionOfQueuedPerson(const unsigned int positionIndex) const;


    const TimeType& GetEarliestQueueTime() const {
        assert(!IsEmpty());
        return (queue.front().timeOfQueueing);
    }

    void InsertIntoQueue(
        const TimeType& currentTime,
        const AgentResource& person);

    void TryToEnter(
        const TimeType& currentTime,
        const AgentResource& person,
        bool& wasQueued);

    void UpdateToTime(const TimeType& currentTime);
    bool HasPersonWhoCanEnterAtThisTime(const TimeType& currentTime) const;

    void GetPersonWhoCanEnterAtThisTime(
        const TimeType& currentTime,
        const unsigned int maxNumberOfPeople,
        bool& wasRetrieved,
        AgentResource& person);

    void RemovePerson(
        const AgentResource& person,
        bool& wasRemoved);

private:
    double timestepDurationSecs;
    TimeType timestepDuration;
    double maxPeopleFlowRatePerSec;
    double accumulatedFlow;
    unsigned int maxAccumulatedFlow;

    unique_ptr<PedQueuePositioner> queuePositionerPtr;

    struct QueueElemType {
        AgentResource agent;
        TimeType timeOfQueueing;

        QueueElemType(
            const AgentResource& initAgent,
            const TimeType& initTimeOfQueueing)
            :
            agent(initAgent),
            timeOfQueueing(initTimeOfQueueing)
        {}
    };

    std::deque<QueueElemType> queue;

    // For Checking
    TimeType lastCurrentTime;

    void UpdatePositionsOfQueuedPeople(const unsigned int startIndex);

};//GisObjectEntranceQueue//



inline
GisObjectEntranceQueue::GisObjectEntranceQueue(
    unique_ptr<PedQueuePositioner>& initQueuePositionerPtr,
    const TimeType& initTimestepDuration,
    const double& initMaxPeopleFlowRatePerSec)
    :
    queuePositionerPtr(move(initQueuePositionerPtr)),
    lastCurrentTime(ZERO_TIME),
    timestepDuration(initTimestepDuration),
    timestepDurationSecs(ConvertTimeToDoubleSecs(initTimestepDuration)),
    maxPeopleFlowRatePerSec(initMaxPeopleFlowRatePerSec)
{
    using std::max;

    if (maxPeopleFlowRatePerSec == DBL_MAX) {
        maxAccumulatedFlow = UINT_MAX;
        accumulatedFlow = DBL_MAX;
    }
    else {
        maxAccumulatedFlow = static_cast<unsigned int>(ceil(maxPeopleFlowRatePerSec * timestepDurationSecs));
        accumulatedFlow = maxAccumulatedFlow;
    }//if//

}//GisObjectEntranceQueue//


inline
void GisObjectEntranceQueue::UpdateToTime(const TimeType& currentTime)
{
    assert(currentTime >= lastCurrentTime);
    if (maxPeopleFlowRatePerSec == DBL_MAX) {
        lastCurrentTime = currentTime;
        return;
    }//if//

    if (currentTime > lastCurrentTime) {
        const TimeType duration = currentTime - lastCurrentTime;
        lastCurrentTime = currentTime;

        const double newFlow = ConvertTimeToDoubleSecs(duration) * maxPeopleFlowRatePerSec;

        (*this).accumulatedFlow += newFlow;

        if (queue.empty()) {

            // Start with max accumulated flow count if there is no waiting people.

            if (accumulatedFlow > maxAccumulatedFlow) {
                (*this).accumulatedFlow = maxAccumulatedFlow;
            }//if//
        }
        else {

            // Continue increasing a flow count to handle grouped
            // people(;"people.NumberPeople() >= 2") if queue is not empty.

        }//if//

    }//if//

}//UpdateToTime//

inline
bool GisObjectEntranceQueue::HasPersonWhoCanEnterAtThisTime(const TimeType& currentTime) const
{
    assert((currentTime == lastCurrentTime) && "UpdateToTime was not called");

    return (!queue.empty() && (accumulatedFlow >= 1.0));

}//GisObjectEntranceQueue//


inline
void GisObjectEntranceQueue::InsertIntoQueue(
    const TimeType& currentTime,
    const AgentResource& person)
{
    queue.push_back(QueueElemType(person, currentTime));
    const unsigned int queueIndex = static_cast<unsigned int>(queue.size()) - 1;
    (*this).UpdatePositionsOfQueuedPeople(queueIndex);

}//Insert//



inline
void GisObjectEntranceQueue::TryToEnter(
    const TimeType& currentTime,
    const AgentResource& person,
    bool& wasQueued)
{
    const double numberOfPeople = person.NumberPeople();

    if (queue.empty()) {
        (*this).UpdateToTime(currentTime);

        if (accumulatedFlow >= numberOfPeople) {
            if (accumulatedFlow != DBL_MAX) {
                accumulatedFlow -= numberOfPeople;
            }//if//
            wasQueued = false;
            return;
        }//if//
    }//if//

    wasQueued = true;

    (*this).InsertIntoQueue(currentTime, person);

}//TryToEnter//


inline
void GisObjectEntranceQueue::GetPersonWhoCanEnterAtThisTime(
    const TimeType& currentTime,
    const unsigned int maxNumberOfPeople,
    bool& wasRetrieved,
    AgentResource& person)
{
    assert(!queue.empty());

    wasRetrieved = false;

    (*this).UpdateToTime(currentTime);

    person = queue.front().agent;

    const double numberOfPeople = person.NumberPeople();

    if (numberOfPeople <= static_cast<double>(maxNumberOfPeople)) {
        
        if (accumulatedFlow >= numberOfPeople) {
            wasRetrieved = true;
            (*this).queue.pop_front();
            
            if (accumulatedFlow != DBL_MAX) {
                (*this).accumulatedFlow -= numberOfPeople;
            }//if//
            
            (*this).UpdatePositionsOfQueuedPeople(0);
        }//if//
    }//if//

}//GetPeopleWhoMustEnterAtThisTime//


inline
void GisObjectEntranceQueue::RemovePerson(
    const AgentResource& person,
    bool& wasRemoved)
{
    wasRemoved = false;
    for(unsigned int i = 0; (i < queue.size()); i++) {
        if (queue[i].agent == person) {
            queue.erase(queue.begin() + i);
            (*this).UpdatePositionsOfQueuedPeople(i);
            wasRemoved = true;
            break;
        }//if//
    }//for//

}//RemovePerson//

inline
void GisObjectEntranceQueue::UpdatePositionsOfQueuedPeople(const unsigned int startIndex)
{
    for (unsigned int i = startIndex; (i < queue.size()); i++) {
        QueueElemType& elem = queue[i];
        elem.agent.SetPosition(queuePositionerPtr->GetPosition(i));
    }//for//

}//UpdatePositionsOfQueuedPeople//




//-----------------------------------------------------------------------------

class GisObjectMultiEntranceQueue {
public:

    void AddEntrance(
        const EntranceIdType& entranceId,
        unique_ptr<PedQueuePositioner>& queuePositionerPtr,
        const TimeType& timestepDuration,
        const double& maxPeopleFlowRatePerSec);

    bool IsEmpty() const { return (numberPersonAgents == 0); }

    void InsertIntoQueue(
        const TimeType& currentTime,
        const AgentResource& person,
        const EntranceIdType& entranceId);

    void TryToEnter(
        const TimeType& currentTime,
        const AgentResource& person,
        const EntranceIdType& entranceId,
        bool& wasQueued);

    void DetermineIfHasPersonWhoCanEnterAtThisTime(
        const TimeType& currentTime,
        bool& aPersonCanEnter);

    void GetPersonWhoCanEnterAtThisTime(
        const TimeType& currentTime,
        const unsigned int maxNumberOfPeople,
        bool& wasRetrieved,
        AgentResource& person);

    void RemovePerson(const AgentResource& person);

private:
    unsigned int numberPersonAgents = 0;

    map<EntranceIdType, unique_ptr<GisObjectEntranceQueue> > queues;

};//GisObjectMultiEntranceQueue//



inline
void GisObjectMultiEntranceQueue::AddEntrance(
    const EntranceIdType& entranceId,
    unique_ptr<PedQueuePositioner>& queuePositionerPtr,
    const TimeType& timestepDuration,
    const double& maxPeopleFlowRatePerSec)
{
    assert(queues.find(entranceId) == queues.end());
    queues[entranceId].reset(
        new GisObjectEntranceQueue(queuePositionerPtr, timestepDuration, maxPeopleFlowRatePerSec));
}



inline
void GisObjectMultiEntranceQueue::InsertIntoQueue(
    const TimeType& currentTime,
    const AgentResource& person,
    const EntranceIdType& entranceId)
{
    assert(queues.find(entranceId) != queues.end());
    queues[entranceId]->InsertIntoQueue(currentTime, person);
    (*this).numberPersonAgents++;

}//InsertIntoQueue//



inline
void GisObjectMultiEntranceQueue::TryToEnter(
    const TimeType& currentTime,
    const AgentResource& person,
    const EntranceIdType& entranceId,
    bool& wasQueued)
{
    if (numberPersonAgents > 1) {
        (*this).InsertIntoQueue(currentTime, person, entranceId);
        wasQueued = true;
    }
    else {
        assert(queues.find(entranceId) != queues.end());
        queues[entranceId]->TryToEnter(currentTime, person, wasQueued);
    }//if//

    if (wasQueued) {
        (*this).numberPersonAgents++;
    }//if//

}//TryToEnter//



inline
void GisObjectMultiEntranceQueue::DetermineIfHasPersonWhoCanEnterAtThisTime(
    const TimeType& currentTime,
    bool& aPersonCanEnter)
{
    aPersonCanEnter = false;

    if (numberPersonAgents == 0) {
        return;
    }//if//

    typedef map<EntranceIdType, unique_ptr<GisObjectEntranceQueue> >::iterator IterType;

    for (IterType iter = queues.begin(); (iter != queues.end()); ++iter) {
        GisObjectEntranceQueue& queue = *iter->second;

        if (!queue.IsEmpty()) {
            queue.UpdateToTime(currentTime);

            if (queue.HasPersonWhoCanEnterAtThisTime(currentTime)) {
                aPersonCanEnter = true;
                break;
            }//if//
        }//if//
    }//for//

}//DetermineIfHasPersonWhoCanEnterAtThisTime//


inline
void GisObjectMultiEntranceQueue::GetPersonWhoCanEnterAtThisTime(
    const TimeType& currentTime,
    const unsigned int maxNumberOfPeople,
    bool& wasRetrieved,
    AgentResource& person)
{
    wasRetrieved = false;

    if (numberPersonAgents == 0) {
        return;
    }//if//

    typedef map<EntranceIdType, unique_ptr<GisObjectEntranceQueue> >::const_iterator IterType;

    IterType earliestQueueIter = queues.end();
    TimeType earliestQueueTime = INFINITE_TIME;
    for(IterType iter = queues.begin(); (iter != queues.end()); ++iter) {
        GisObjectEntranceQueue& queue = *iter->second;

        if ((!queue.IsEmpty()) && (earliestQueueTime > queue.GetEarliestQueueTime())) {
            earliestQueueTime = queue.GetEarliestQueueTime();
            earliestQueueIter = iter;
        }//if//
    }//for//

    if (earliestQueueIter == queues.end()) {
        return;
    }//if//

    GisObjectEntranceQueue& queue = *earliestQueueIter->second;
    queue.GetPersonWhoCanEnterAtThisTime(currentTime, maxNumberOfPeople, wasRetrieved, person);

    if (wasRetrieved) {
        assert(numberPersonAgents > 0);
        numberPersonAgents--;
    }//if//

}//GetPeopleWhoMustEnterAtThisTime//



inline
void GisObjectMultiEntranceQueue::RemovePerson(const AgentResource& person)
{
    typedef map<EntranceIdType, unique_ptr<GisObjectEntranceQueue> >::const_iterator IterType;

    for(IterType iter = queues.begin(); (iter != queues.end()); ++iter) {
        GisObjectEntranceQueue& queue = *iter->second;
        bool wasRemoved;
        queue.RemovePerson(person, wasRemoved);
        if (wasRemoved) {
            assert(numberPersonAgents > 0);
            numberPersonAgents--;
            return;
        }//if//
    }//for//

    assert(false && "Was not found"); abort();

}//RemovePerson//


}//namespace//


#endif

