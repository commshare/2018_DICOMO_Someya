// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef MULTIAGENT2_ROUTING_H
#define MULTIAGENT2_ROUTING_H

#include <limits.h>
#include <assert.h>
#include <float.h>
#include <stack>
#include <vector>
#include <map>
#include <set>
#include "boost/heap/pairing_heap.hpp"

namespace MultiAgent2 {

using namespace boost::heap;
using std::vector;
using std::map;

typedef unsigned int GraphVertexIdType;
const GraphVertexIdType InvalidGraphVertexId = UINT_MAX;

template<typename WeightType>
class WeightedGraph {
public:
    struct EdgeInfoType {
        WeightType weight;
        GraphVertexIdType otherVertexId;

        EdgeInfoType(
            const WeightType& initWeight,
            const GraphVertexIdType& initOtherVertexId)
        :
            weight(initWeight),
            otherVertexId(initOtherVertexId)
        {}

        EdgeInfoType() { }

    };//EdgeInfoType//

    void Clear() {
        theGraph.clear();
        freeVertexIdList.clear();
    }

    void AddNewVertex(GraphVertexIdType& vertexId);
    void DeleteVertex(const GraphVertexIdType& vertexId);

    void AddEdge(const GraphVertexIdType& v1, const GraphVertexIdType& v2, const WeightType& weight);
    void DeleteEdge(const GraphVertexIdType& v1, const GraphVertexIdType& v2);

    void AddBidirectionalEdge(const GraphVertexIdType& v1, const GraphVertexIdType& v2, const WeightType& weight);
    void DeleteBidirectionalEdge(const GraphVertexIdType& v1, const GraphVertexIdType& v2);

    void SetEdgeWeight(
        const GraphVertexIdType& v1,
        const GraphVertexIdType& v2,
        const WeightType& weight,
        bool& wasSet);

    void SwapEdgeWeight(
        const GraphVertexIdType& v1,
        const GraphVertexIdType& v2,
        WeightType& swapWeight,
        bool& wasSwapped);

    unsigned int GetNumberEdgesAtVertex(const GraphVertexIdType& vertexId) const
    {
        assert(vertexId < theGraph.size());
        return (static_cast<unsigned int>(theGraph[vertexId].size()));
    }

    const EdgeInfoType& GetEdgeInfo(
        const GraphVertexIdType& vertexId,
        const unsigned int edgeIndex) const;

    GraphVertexIdType GetHighestVertexIdPlusOne() const
        { return (static_cast<GraphVertexIdType>(theGraph.size())); }

    unsigned int GetNumberVertices() const
        { return (static_cast<unsigned int>(theGraph.size() - freeVertexIdList.size())); }

    void GetListOfVertexIds(vector<GraphVertexIdType>& vertexList) const;

private:

    vector<vector<EdgeInfoType> > theGraph;
    vector<GraphVertexIdType> freeVertexIdList;

};//WeightedGraph//




template<typename WeightType>
void WeightedGraph<WeightType>::AddNewVertex(GraphVertexIdType& vertexId)
{
    if (freeVertexIdList.empty()) {
        assert(theGraph.size() < UINT_MAX);
        theGraph.resize(theGraph.size() + 1);
        vertexId = static_cast<GraphVertexIdType>(theGraph.size() - 1);
    }
    else {
        vertexId = freeVertexIdList.back();
        freeVertexIdList.resize(freeVertexIdList.size() - 1);
        assert(theGraph.at(vertexId).empty());
    }//if//

}//AddNewVertex//


template<typename WeightType>
void WeightedGraph<WeightType>::DeleteVertex(const GraphVertexIdType& vertexId)
{
    vector<EdgeInfoType>& theEdges = theGraph.at(vertexId);

    for(unsigned int i = 0; (i < theEdges.size()); i++) {
        (*this).DeleteEdge(vertexId, theEdges[i].otherVertexId);
    }//for//

    assert(theEdges.empty());

    if ((vertexId + 1) == theGraph.size()) {
        theGraph.resize(theGraph.size() - 1);
    }
    else {
        freeVertexIdList.push_back(vertexId);
    }//if//

}//DeleteVertex//


template<typename WeightType>
void WeightedGraph<WeightType>::AddEdge(
    const GraphVertexIdType& v1,
    const GraphVertexIdType& v2,
    const WeightType& weight)
{
    assert(v1 < theGraph.size());
    assert(v2 < theGraph.size());

    vector<EdgeInfoType>& v1Edges = theGraph[v1];

    // Check for duplicates:

    for(unsigned int i = 0; (i < v1Edges.size()); i++) {
        assert(v1Edges[i].otherVertexId != v2);
    }//for//

    v1Edges.push_back(EdgeInfoType(weight, v2));

}//AddEdge//


template<typename WeightType>
void WeightedGraph<WeightType>::AddBidirectionalEdge(
    const GraphVertexIdType& v1,
    const GraphVertexIdType& v2,
    const WeightType& weight)
{
    assert(v1 < theGraph.size());
    assert(v2 < theGraph.size());

    vector<EdgeInfoType>& v1Edges = theGraph[v1];
    vector<EdgeInfoType>& v2Edges = theGraph[v2];

    // Check for duplicates:

    for(unsigned int i = 0; (i < v1Edges.size()); i++) {
        assert(v1Edges[i].otherVertexId != v2);
    }//for//

    for(unsigned int i = 0; (i < v2Edges.size()); i++) {
        assert(v2Edges[i].otherVertexId != v1);
    }//for//

    v1Edges.push_back(EdgeInfoType(weight, v2));
    v2Edges.push_back(EdgeInfoType(weight, v1));

}//AddEdge//


template<typename WeightType>
void WeightedGraph<WeightType>::DeleteEdge(
    const GraphVertexIdType& v1,
    const GraphVertexIdType& v2)
{
    vector<EdgeInfoType>& v1Edges = theGraph.at(v1);

    for(unsigned int i = 0; (i < v1Edges.size()); i++) {
        if (v1Edges[i].otherVertexId == v2) {
            v1Edges.erase(v1Edges.begin() + i);
            break;
        }//if//
    }//for//

}//DeleteEdge//


template<typename WeightType>
void WeightedGraph<WeightType>::DeleteBidirectionalEdge(
    const GraphVertexIdType& v1,
    const GraphVertexIdType& v2)
{
    vector<EdgeInfoType>& v1Edges = theGraph.at(v1);
    vector<EdgeInfoType>& v2Edges = theGraph.at(v2);

    for(unsigned int i = 0; (i < v1Edges.size()); i++) {
        if (v1Edges[i].otherVertexId == v2) {
            v1Edges.erase(v1Edges.begin() + i);
            break;
        }//if//
    }//for//

    for(unsigned int i = 0; (i < v2Edges.size()); i++) {
        if (v2Edges[i].otherVertexId == v1) {
            v2Edges.erase(v2Edges.begin() + i);
            break;
        }//if//
    }//for//

}//DeleteEdge//


template<typename WeightType>
void WeightedGraph<WeightType>::SetEdgeWeight(
    const GraphVertexIdType& v1,
    const GraphVertexIdType& v2,
    const WeightType& weight,
    bool& wasSet)
{
    wasSet = false;
    vector<EdgeInfoType>& v1Edges = theGraph[v1];

    for(unsigned int i = 0; (i < v1Edges.size()); i++) {
        if (v1Edges[i].otherVertexId == v2) {
            v1Edges[i].weight = weight;
            wasSet = true;
            break;
        }//if//
    }//for//

}//SetEdgeWeight//



template<typename WeightType>
void WeightedGraph<WeightType>::SwapEdgeWeight(
    const GraphVertexIdType& v1,
    const GraphVertexIdType& v2,
    WeightType& swapWeight,
    bool& wasSwapped)
{
    wasSwapped = false;
    vector<EdgeInfoType>& v1Edges = theGraph[v1];

    for(unsigned int i = 0; (i < v1Edges.size()); i++) {
        if (v1Edges[i].otherVertexId == v2) {
            std::swap(v1Edges[i].weight, swapWeight);
            wasSwapped = true;
            break;
        }//if//
    }//for//

}//SetEdgeWeight//





template<typename WeightType>
const typename WeightedGraph<WeightType>::EdgeInfoType& WeightedGraph<WeightType>::GetEdgeInfo(
    const GraphVertexIdType& vertexId,
    const unsigned int edgeIndex) const
{
    assert(vertexId < theGraph.size());
    const vector<EdgeInfoType>& theEdges = theGraph[vertexId];
    assert(edgeIndex < theEdges.size());
    return (theEdges[edgeIndex]);
}


template<typename WeightType>
void WeightedGraph<WeightType>::GetListOfVertexIds(vector<GraphVertexIdType>& vertexList) const
{
    vertexList.clear();

    for(unsigned int i = 0; (i < theGraph.size()); i++) {
        bool wasFound = false;
        for(unsigned j = 0; (j < freeVertexIdList.size()); j++) {
            if (i == freeVertexIdList[j]) {
                wasFound = true;
                break;
            }//if//
        }//for//

        if (!wasFound) {
            vertexList.push_back(i);
        }//if//
    }//for//

}//GetListOfVertexIds//


//--------------------------------------------------------------------------------------------------

class AstarHeuristicFunction {
public:
    virtual double estimate(
        const GraphVertexIdType& startVertexId,
        const GraphVertexIdType& endVertexId) const = 0;
};


//--------------------------------------------------------------------------------------------------

// A* Search algorithm support datastructures (Pulled out of function due circlar definition problem
// and local (member) function restriction not allowed on Linux).  Other options would have
// been to wrap function in a class or could have used pointer to a forward declaration.


struct AstarSearchGraphNodeSearchInfoType;

class AstarSearchGraphQueueComparator {
public:
    explicit AstarSearchGraphQueueComparator(
        const vector<AstarSearchGraphNodeSearchInfoType>& vertexInfos)
       : refToVertexInfos(vertexInfos) {}

    bool operator()(
        const GraphVertexIdType& leftVertexId,
        const GraphVertexIdType& rightVertexId) const;

private:
    const vector<AstarSearchGraphNodeSearchInfoType>& refToVertexInfos;

};//AstarSearchGraphQueueComparator//


typedef boost::heap::pairing_heap<
    GraphVertexIdType,
    boost::heap::compare<AstarSearchGraphQueueComparator>> AstarSearchGraphPriorityQueueType;

// Search specific data

struct AstarSearchGraphNodeSearchInfoType {
    bool hasBeenVisited;
    double bestPathLengthSoFar;
    double estimateToGoal;
    double totalEstimatedPathLength;
    GraphVertexIdType cameFromVertexId;
    AstarSearchGraphPriorityQueueType::handle_type queueElementHandle;

    AstarSearchGraphNodeSearchInfoType() : hasBeenVisited(false), bestPathLengthSoFar(DBL_MAX) { }

};//AstarSearchGraphNodeSearchInfoTypee//

inline
bool AstarSearchGraphQueueComparator::operator()(
    const GraphVertexIdType& leftVertexId,
    const GraphVertexIdType& rightVertexId) const
{
    const AstarSearchGraphNodeSearchInfoType& leftInfo = refToVertexInfos[leftVertexId];
    const AstarSearchGraphNodeSearchInfoType& rightInfo = refToVertexInfos[rightVertexId];

    // Note: Invert comparison for max-heap (highest at top).

    return ((leftInfo.totalEstimatedPathLength > rightInfo.totalEstimatedPathLength) ||
            ((leftInfo.totalEstimatedPathLength == rightInfo.totalEstimatedPathLength) &&
             (leftVertexId > rightVertexId)));
}


//------------------------------------------------


// Algorithm assumes "Consistent Heuristic Function".


void AstarSearchGraph(
    const AstarHeuristicFunction& consistentHFunction,
    const WeightedGraph<double>& graph,
    const GraphVertexIdType& startVertexId,
    const GraphVertexIdType& endVertexId,
    vector<GraphVertexIdType>& vertexPath)
{
    using namespace boost::heap;  // Necessary because of Boost bug.

    typedef AstarSearchGraphPriorityQueueType::handle_type QueueHandleType;
    typedef AstarSearchGraphNodeSearchInfoType NodeSearchInfoType;
    typedef AstarSearchGraphPriorityQueueType PriorityQueueType;

    vertexPath.clear();

    vector<AstarSearchGraphNodeSearchInfoType> vertexInfos(graph.GetHighestVertexIdPlusOne());

    AstarSearchGraphQueueComparator aComparator(vertexInfos);

    PriorityQueueType searchQueue(aComparator);

    NodeSearchInfoType& startVertexInfo = vertexInfos[startVertexId];
    startVertexInfo.bestPathLengthSoFar = 0.0;
    startVertexInfo.estimateToGoal = 0.0;
    startVertexInfo.totalEstimatedPathLength = 0.0;
    startVertexInfo.cameFromVertexId = InvalidGraphVertexId;
    searchQueue.push(startVertexId);

    bool foundPath = false;

    while (!searchQueue.empty()) {
        const GraphVertexIdType topVertexId = searchQueue.top();
        searchQueue.pop();

        NodeSearchInfoType& vertexInfo = vertexInfos[topVertexId];

        assert(!vertexInfo.hasBeenVisited);

        vertexInfo.hasBeenVisited = true;

        if (topVertexId == endVertexId) {
            foundPath = true;
            break;
        }//if//

        for(unsigned int i = 0; (i < graph.GetNumberEdgesAtVertex(topVertexId)); i++) {
            const WeightedGraph<double>::EdgeInfoType& edge =
                graph.GetEdgeInfo(topVertexId, i);

            NodeSearchInfoType& otherVertexInfo = vertexInfos[edge.otherVertexId];

            if (otherVertexInfo.hasBeenVisited) {

                // In "Closed Set".  "Consistent" heuristic function implies that
                // vertex already has optimal path.

                continue;
            }//if//

            const double newPathLength = vertexInfo.bestPathLengthSoFar + edge.weight;

            if (otherVertexInfo.bestPathLengthSoFar == DBL_MAX) {
                // First time vertex has been seen put in search queue (open set).

                otherVertexInfo.cameFromVertexId = topVertexId;
                otherVertexInfo.bestPathLengthSoFar = newPathLength;
                otherVertexInfo.estimateToGoal =
                    consistentHFunction.estimate(edge.otherVertexId, endVertexId);
                otherVertexInfo.totalEstimatedPathLength =
                    otherVertexInfo.bestPathLengthSoFar + otherVertexInfo.estimateToGoal;

                otherVertexInfo.queueElementHandle = searchQueue.push(edge.otherVertexId);
            }
            else if (newPathLength < otherVertexInfo.bestPathLengthSoFar) {
                otherVertexInfo.cameFromVertexId = topVertexId;
                otherVertexInfo.bestPathLengthSoFar = newPathLength;
                otherVertexInfo.totalEstimatedPathLength =
                    otherVertexInfo.bestPathLengthSoFar + otherVertexInfo.estimateToGoal;

                // Update search queue
                searchQueue.increase(otherVertexInfo.queueElementHandle);

            }//if//
        }//for//
    }//while//

    if (foundPath) {
        GraphVertexIdType pathVertexId = endVertexId;

        while (pathVertexId != startVertexId) {
            vertexPath.push_back(pathVertexId);
            pathVertexId = vertexInfos[pathVertexId].cameFromVertexId;
        }//while//

        vertexPath.push_back(startVertexId);
        std::reverse(vertexPath.begin(), vertexPath.end());
    }//if//

}//AstarSearchGraph//


template<typename WeightType>
bool GraphIsConnected(const WeightedGraph<WeightType>& graph)
{
    vector<GraphVertexIdType> vertexIds;
    graph.GetListOfVertexIds(vertexIds);
    if (vertexIds.empty()) {
        return true;
    }//if//

    std::set<GraphVertexIdType> seenSet;
    std::stack<GraphVertexIdType> vertexHorizon;
    vertexHorizon.push(vertexIds[0]);
    while (!vertexHorizon.empty()) {
        const GraphVertexIdType vertexId = vertexHorizon.top();
        vertexHorizon.pop();
        seenSet.insert(vertexId);
        for (unsigned int i = 0; (i < graph.GetNumberEdgesAtVertex(vertexId)); i++) {
            const typename WeightedGraph<WeightType>::EdgeInfoType& edge = graph.GetEdgeInfo(vertexId, i);

            if (seenSet.count(edge.otherVertexId) == 0) {
                vertexHorizon.push(edge.otherVertexId);
            }//if//

        }//for//
    }//while//

    return (vertexIds.size() == seenSet.size());

}//GraphIsConnected//


}//namespace//


#endif

