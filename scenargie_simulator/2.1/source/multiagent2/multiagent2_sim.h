// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef MULTIAGENT2_SIM_H
#define MULTIAGENT2_SIM_H

#include <memory>
#include <string>
#include <iostream>

#include "scensim_engine.h"
#include "scensim_parmio.h"
#include "scensim_netsim.h"
#include "randomnumbergen.h"
#include "multiagent2_common.h"
#include "multiagent2_gis.h"
#include "two_d_trees.h"
#include "multiagent2_routing.h"
#include "multiagent2_auto.h"

namespace MultiAgent2 {

using ScenSim::ParameterDatabaseReader;
using ScenSim::SimulationEngine;
using ScenSim::SimulationEngineInterface;
using ScenSim::SimulationEvent;
using ScenSim::TimeType;
using ScenSim::MILLI_SECOND;
using ScenSim::ConvertTimeToDoubleSecs;
using ScenSim::ZERO_TIME;
using ScenSim::INFINITE_TIME;
using ScenSim::ObjectMobilityPosition;
using ScenSim::RandomNumberGeneratorSeedType;
using ScenSim::DeleteTrailingSpaces;
using ScenSim::IsAConfigFileCommentLine;
using ScenSim::ConvertDoubleSecsToTime;


using std::shared_ptr;
using std::make_shared;
using std::vector;
using std::string;
using std::map;
using std::cerr;
using std::endl;
using std::ifstream;
using std::istringstream;
using std::deque;



inline
bool ReadPedestrianModelIsEnabled(const ParameterDatabaseReader& parameterDatabaseReader)
{
    return (
        (parameterDatabaseReader.ParameterExists(parameterNamePrefix + "pedestrian-model-is-on")) &&
        (parameterDatabaseReader.ReadBool(parameterNamePrefix + "pedestrian-model-is-on")));
}


class PedestrianSandbox {
public:
    typedef WaypointPortal::WaypointIdType WaypointIdType;

    PedestrianSandbox(const ParameterDatabaseReader& theParameterDatabaseReader);

    double GetMinX() const { return minX; }
    double GetMaxX() const { return maxX; }
    double GetMinY() const { return minY; }
    double GetMaxY() const { return maxY; }

    //Future: void EnableBarrier();
    //Future: void DisableBarrier();

    void GetCloseBarrierPoints(
        const TwoDVector& position,
        const double& maxDistanceToBarrier,
        vector<TwoDVector>& barrierPoints);

    class CachedBarrierInfoType {
    public:
        CachedBarrierInfoType() : hasBeenSet(false) { }
    private:
        typedef LineSegmentTwoDTree<TwoDVector>::RadiusLimitedIterator::LineSegmentType LineSegmentType;

        bool hasBeenSet;
        vector<LineSegmentType> lineSegments;

        friend class PedestrianSandbox;
    };

    void GetCloseBarrierPoints(
        const TwoDVector& position,
        const double& maxDistanceToBarrier,
        const bool useCachedBarriers,
        CachedBarrierInfoType& cachedInfo,
        vector<TwoDVector>& barrierPoints);

    unsigned int GetNumberWaypoints() const { return (static_cast<unsigned int>(waypoints.size())); }

    void FindWaypoint(
        const TwoDVector& position,
        bool& wasFound,
        WaypointIdType& waypointId) const;

    WaypointIdType FindClosestWaypoint(const TwoDVector& position) const;

    WaypointIdType CalcNextWaypoint(
        const WaypointIdType& currentWaypoint,
        const WaypointIdType& destinationWaypoint) const;

    const WaypointPortal& GetWaypoint(const WaypointIdType& waypointId) const
        { return (waypoints.at(waypointId)); }

private:
    double minX;
    double maxX;
    double minY;
    double maxY;

    LineSegmentTwoDTree<TwoDVector> barriers;
    vector<WaypointPortal> waypoints;
    WeightedGraph<double> waypointGraph;

    class WaypointLinkIdType {
    public:
        WaypointLinkIdType(
            const TwoDVector& initFirstPointWaypoint1,
            const TwoDVector& initFirstPointWaypoint2)
        :
            firstPointWaypoint1(initFirstPointWaypoint1),
            firstPointWaypoint2(initFirstPointWaypoint2)
        {}

        bool operator==(const WaypointLinkIdType& right) const {
            return ((firstPointWaypoint1 == right.firstPointWaypoint1) &&
                    (firstPointWaypoint2 == right.firstPointWaypoint2));
        }

        bool operator<(const WaypointLinkIdType& right) const {
            return ((firstPointWaypoint1 < right.firstPointWaypoint1) ||
                    ((firstPointWaypoint1 == right.firstPointWaypoint1) &&
                      firstPointWaypoint2 < right.firstPointWaypoint2));
        }
    private:
        TwoDVector firstPointWaypoint1;
        TwoDVector firstPointWaypoint2;
    };

    map<WaypointLinkIdType, double> waypointGraphLinkWeightAdjustments;


    void AddBuildingBarriers(const vector<GisBuildingPolygon>& buildingPolygons);
    void AddRoadBarriers(const vector<GisRoadSegment>& roadSegments);

    void CreateWaypointGraph();

    bool DoesNotTunnelThroughIntermediateWaypoint(
        const unsigned int waypoint1Id,
        const unsigned int waypoint2Id) const;

    void ReadWaypointGraphLinkAdjustFile(const string& linkAdjustmentFilename);

};//PedestrianSandbox//


const double minSpacingMeters = 1.0;


inline
PedestrianSandbox::PedestrianSandbox(const ParameterDatabaseReader& theParameterDatabaseReader)
    :
    barriers(minSpacingMeters),
    minX(DBL_MAX),
    maxX(-DBL_MAX),
    minY(DBL_MAX),
    maxY(-DBL_MAX)
{
    using std::min;
    using std::max;


    if (!ReadPedestrianModelIsEnabled(theParameterDatabaseReader)) {
        return;
    }//if//

    string shapeFileDirPath;
    if (theParameterDatabaseReader.ParameterExists("gis-object-file-path")) {
        shapeFileDirPath = theParameterDatabaseReader.ReadString("gis-object-file-path");
    }//if//

    const string gisRoadsFilenamePrefix = shapeFileDirPath + "road";
    const string gisBuildingFilenamePrefix = shapeFileDirPath + "building";
    const string gisWaypointFilenamePrefix = shapeFileDirPath + "waypoint";

    vector<GisBuildingPolygon> buildingPolygons;

    double newMinX;
    double newMaxX;
    double newMinY;
    double newMaxY;

    ReadBuildingPolygons(
        gisBuildingFilenamePrefix,
        newMinX,
        newMaxX,
        newMinY,
        newMaxY,
        buildingPolygons);

    minX = min(minX, newMinX);
    maxX = max(maxX, newMaxX);
    minY = min(minY, newMinY);
    maxY = max(maxY, newMaxY);

    //--------------------------

    vector<GisRoadSegment> roadSegments;

    ReadRoads(
        gisRoadsFilenamePrefix,
        newMinX,
        newMaxX,
        newMinY,
        newMaxY,
        roadSegments);

    minX = min(minX, newMinX);
    maxX = max(maxX, newMaxX);
    minY = min(minY, newMinY);
    maxY = max(maxY, newMaxY);

    (*this).AddBuildingBarriers(buildingPolygons);
    (*this).AddRoadBarriers(roadSegments);

    (*this).barriers.BuildTree();

    const double waypointCloseEnoughDistanceMeters =
        theParameterDatabaseReader.ReadDouble(
            parameterNamePrefix + "waypoint-close-enough-distance-meters");

    ReadWaypointPortals(gisWaypointFilenamePrefix, waypointCloseEnoughDistanceMeters, (*this).waypoints);

    if (theParameterDatabaseReader.ParameterExists(
        parameterNamePrefix + "waypoint-link-weight-adjustment-filename")) {

        const string linkAdjustmentFilename =
            theParameterDatabaseReader.ReadString(
                parameterNamePrefix + "waypoint-link-weight-adjustment-filename");

        (*this).ReadWaypointGraphLinkAdjustFile(linkAdjustmentFilename);
    }//if//

    (*this).CreateWaypointGraph();

}//PedestrianSandbox()//



inline
void PedestrianSandbox::ReadWaypointGraphLinkAdjustFile(const string& linkAdjustmentFilename)
{
    ifstream linkAdjustStream(linkAdjustmentFilename);

    assert(linkAdjustStream.good());

    while (!linkAdjustStream.eof()) {
        string aLine;
        getline(linkAdjustStream, aLine);

        if (linkAdjustStream.eof()) {
            break;
        }
        else if (linkAdjustStream.fail()) {
            cerr << "Error reading waypoint link weight adjustment file: " << linkAdjustmentFilename << endl;
            exit(1);
        }//if//

        DeleteTrailingSpaces(aLine);

        if (IsAConfigFileCommentLine(aLine)) {
            continue;
        }//if//

        istringstream lineStream(aLine);

        TwoDVector waypoint1;
        TwoDVector waypoint2;
        double adjustmentFactor;

        lineStream >> waypoint1.x;
        lineStream >> waypoint1.y;
        lineStream >> waypoint2.x;
        lineStream >> waypoint2.y;
        lineStream >> adjustmentFactor;

        assert(!lineStream.fail()  && "Problem in link weight adjustment file");

        TwoDVector firstWaypoint;
        TwoDVector secondWaypoint;

        bool firstWaypointHasBeenSet = false;
        bool secondWaypointHasBeenSet = false;
        for(unsigned int i = 0; (i < waypoints.size()); i++) {
            const WaypointPortal& aWaypoint = waypoints[i];

            if ((aWaypoint.IsInsideWaypoint(waypoint1)) ||
                 (aWaypoint.IsInsideWaypoint(waypoint2))) {

                if (!firstWaypointHasBeenSet) {
                    firstWaypoint = aWaypoint.GetFirstPoint();
                    firstWaypointHasBeenSet = true;
                }
                else {
                    secondWaypoint = aWaypoint.GetFirstPoint();
                    secondWaypointHasBeenSet = true;
                    break;
                }//if//
            }//if//
        }//for//

        if (!secondWaypointHasBeenSet) {
            cerr << "Error in Link Weight Adjustment File at line:" << endl;
            cerr << "   " << aLine << endl;
            exit(1);
        }//if//

        const WaypointLinkIdType linkId(firstWaypoint, secondWaypoint);
        (*this).waypointGraphLinkWeightAdjustments[linkId] = adjustmentFactor;

    }//while//

}//ReadWaypointGraphLinkAdjustFile//


inline
void PedestrianSandbox::AddBuildingBarriers(const vector<GisBuildingPolygon>& buildingPolygons)
{
    for(unsigned int i = 0; (i < buildingPolygons.size()); i++) {
        const GisBuildingPolygon& aPolygon = buildingPolygons[i];
        for(unsigned int j = 0; (j < (aPolygon.buildingVertices.size() - 1)); j++) {
            barriers.Insert(aPolygon.buildingVertices[j], aPolygon.buildingVertices[j+1]);
        }//for//
    }//for//

}//AddBuildingBarriers//



inline
void PedestrianSandbox::AddRoadBarriers(const vector<GisRoadSegment>& roadSegments)
{

    // Find intersections.

    map<TwoDVector, vector<unsigned int> > locationToRoadSegmentIndicesMap;

    for(unsigned int i = 0; (i < roadSegments.size()); i++) {
        const GisRoadSegment& aRoadSegment = roadSegments[i];

        const TwoDVector v1(
            aRoadSegment.GetFirstLocationXMeters(),
            aRoadSegment.GetFirstLocationYMeters());

        locationToRoadSegmentIndicesMap[v1].push_back(i);

        const TwoDVector v2(
            aRoadSegment.GetSecondLocationXMeters(),
            aRoadSegment.GetSecondLocationYMeters());

        locationToRoadSegmentIndicesMap[v2].push_back(i);

    }//for//

    struct RoadSegmentEdgeInfoType {
        TwoDVector edge1V1;
        TwoDVector edge1V2;
        TwoDVector edge2V1;
        TwoDVector edge2V2;
    };


    // Add preliminary road edges.

    vector<RoadSegmentEdgeInfoType> segmentEdges(roadSegments.size());

    for(unsigned int i = 0; (i < roadSegments.size()); i++) {
        const GisRoadSegment& aRoadSegment = roadSegments[i];
        RoadSegmentEdgeInfoType& roadSegmentEdges = segmentEdges[i];

        CalcParallelLine(
            TwoDVector(
                aRoadSegment.GetFirstLocationXMeters(),
                aRoadSegment.GetFirstLocationYMeters()),
            TwoDVector(
                aRoadSegment.GetSecondLocationXMeters(),
                aRoadSegment.GetSecondLocationYMeters()),
            (aRoadSegment.GetRoadWidthMeters() / 2.0),
            roadSegmentEdges.edge1V1,
            roadSegmentEdges.edge1V2);

        CalcParallelLine(
            TwoDVector(
                aRoadSegment.GetFirstLocationXMeters(),
                aRoadSegment.GetFirstLocationYMeters()),
            TwoDVector(
                aRoadSegment.GetSecondLocationXMeters(),
                aRoadSegment.GetSecondLocationYMeters()),
            -(aRoadSegment.GetRoadWidthMeters() / 2.0),
            roadSegmentEdges.edge2V1,
            roadSegmentEdges.edge2V2);
    }//for//


    // Fix up road edges.

    typedef map<TwoDVector, vector<unsigned int> >::const_iterator IterType;

    for(IterType iter = locationToRoadSegmentIndicesMap.begin();
        (iter != locationToRoadSegmentIndicesMap.end()); ++iter) {

        const TwoDVector& roadPosition = (*iter).first;
        const vector<unsigned int>& roadIndices = (*iter).second;

        switch (roadIndices.size()) {

        case 1: {
            // Culdesac

            const GisRoadSegment& aRoadSegment = roadSegments[roadIndices[0]];

            // Calc end of street points and close off end.

            TwoDVector otherPosition;


            if ((aRoadSegment.GetFirstLocationXMeters() == roadPosition.GetX()) &&
                (aRoadSegment.GetFirstLocationYMeters() == roadPosition.GetY())) {

                otherPosition =
                    TwoDVector(
                        aRoadSegment.GetSecondLocationXMeters(),
                        aRoadSegment.GetSecondLocationYMeters());
            }
            else {
                otherPosition =
                    TwoDVector(
                        aRoadSegment.GetFirstLocationXMeters(),
                        aRoadSegment.GetFirstLocationYMeters());
            }//if//

            TwoDVector edgeV1;
            TwoDVector edgeV2;

            CalcPerpendicularLine(
                roadPosition,
                otherPosition,
                0.0,
                aRoadSegment.GetRoadWidthMeters(),
                edgeV1,
                edgeV2);

            barriers.Insert(edgeV1, edgeV2);

            break;
        }
        case 2: {

            // Plug gaps or remove a segment cross, by changing to common end point.

            const GisRoadSegment& roadSegment1 = roadSegments[roadIndices[0]];
            const GisRoadSegment& roadSegment2 = roadSegments[roadIndices[1]];
            RoadSegmentEdgeInfoType edgesSeg1 = segmentEdges[roadIndices[0]];
            RoadSegmentEdgeInfoType edgesSeg2 = segmentEdges[roadIndices[1]];

            if (((roadSegment1.GetFirstLocationXMeters() == roadSegment2.GetFirstLocationXMeters()) &&
                 (roadSegment1.GetFirstLocationYMeters() == roadSegment2.GetFirstLocationYMeters())) ||
                ((roadSegment1.GetSecondLocationXMeters() == roadSegment2.GetSecondLocationXMeters()) &&
                 (roadSegment1.GetSecondLocationYMeters() == roadSegment2.GetSecondLocationYMeters()))) {

                // Swap edges so that two segments match.

                std::swap(edgesSeg2.edge1V1, edgesSeg1.edge2V1);
                std::swap(edgesSeg2.edge1V2, edgesSeg1.edge2V2);

            }//if//

            bool pointExists;
            TwoDVector newCommonPositionEdge1;

            CalcIntersectionPointOfTwoLines(
                edgesSeg1.edge1V1, edgesSeg1.edge1V2,
                edgesSeg2.edge1V1, edgesSeg2.edge1V2,
                pointExists,
                newCommonPositionEdge1);

            assert(pointExists);

            TwoDVector newCommonPositionEdge2;

            CalcIntersectionPointOfTwoLines(
                edgesSeg1.edge2V1, edgesSeg1.edge2V2,
                edgesSeg2.edge2V1, edgesSeg2.edge2V2,
                pointExists,
                newCommonPositionEdge2);

            assert(pointExists);

            // Set edges to new common point.

            if ((roadSegment1.GetFirstLocationXMeters() == roadPosition.GetX()) &&
                (roadSegment1.GetFirstLocationYMeters() == roadPosition.GetY())) {

                edgesSeg1.edge1V1 = newCommonPositionEdge1;
                edgesSeg1.edge2V1 = newCommonPositionEdge2;
            }
            else {
                edgesSeg1.edge1V2 = newCommonPositionEdge1;
                edgesSeg1.edge2V2 = newCommonPositionEdge2;
            }//if//


            if ((roadSegment2.GetFirstLocationXMeters() == roadPosition.GetX()) &&
                (roadSegment2.GetFirstLocationYMeters() == roadPosition.GetY())) {

                edgesSeg2.edge1V1 = newCommonPositionEdge1;
                edgesSeg2.edge2V1 = newCommonPositionEdge2;
            }
            else {

                edgesSeg2.edge1V2 = newCommonPositionEdge1;
                edgesSeg2.edge2V2 = newCommonPositionEdge2;

            }//if//

            //Future// if (roadSegments[roadIndices[0]].GetRoadId() == roadSegments[roadIndices[1]].GetRoadId()) {
            //Future//     // Road continuation.
            //Future// }
            //Future// else {
            //Future//     // "L" intersection.
            //Future//
            //Future//
            //Future// }//if//

            break;
        }

        //Future//: Automatic sidewalk generation based on low detail road information.

        //Future// case 3: {
        //Future//     // "T" intersection.
        //Future//
        //Future//
        //Future//     break;
        //Future// }
        //Future// case 4: {
        //Future//     // 4-way intersection.
        //Future//
        //Future//
        //Future//     break;
        //Future// }
        //Future//
        //Future// default:
        //Future//     cerr << "Multiagent2 Model: 5+-way intersections not supported" << endl;
        //Future//     exit(1);
        //Future//     break;

        }//switch//
    }//for//

    for(unsigned int i = 0; (i < roadSegments.size()); i++) {
        const RoadSegmentEdgeInfoType& edgeInfo = segmentEdges[i];

        barriers.Insert(edgeInfo.edge1V1, edgeInfo.edge1V2);
        barriers.Insert(edgeInfo.edge2V1, edgeInfo.edge2V2);

    }//for//

}//AddBuildingBarriers//


inline
void PedestrianSandbox::GetCloseBarrierPoints(
    const TwoDVector& position,
    const double& maxDistanceToBarrier,
    vector<TwoDVector>& barrierPoints)
{
    barrierPoints.clear();

    LineSegmentTwoDTree<TwoDVector>::RadiusLimitedIterator iter =
        barriers.MakeRadiusLimitedIterator(position, maxDistanceToBarrier);

    while (iter != barriers.end()) {

        typedef LineSegmentTwoDTree<TwoDVector>::RadiusLimitedIterator::LineSegmentType LineSegmentType;

        const LineSegmentType& lineSegment = *iter;

        const TwoDVector closestPoint =
            CalcClosestPointOnLineSegmentToAPoint(
                lineSegment.v1, lineSegment.v2, position);

        barrierPoints.push_back(closestPoint);

        ++iter;

    }//while//

}//GetCloseBarrierPoints//


inline
void PedestrianSandbox::GetCloseBarrierPoints(
    const TwoDVector& position,
    const double& maxDistanceToBarrier,
    const bool useCachedBarriers,
    CachedBarrierInfoType& cachedInfo,
    vector<TwoDVector>& barrierPoints)
{
    typedef LineSegmentTwoDTree<TwoDVector>::RadiusLimitedIterator::LineSegmentType LineSegmentType;

    barrierPoints.clear();

    if ((!cachedInfo.hasBeenSet) || (!useCachedBarriers)) {
        cachedInfo.lineSegments.clear();

        LineSegmentTwoDTree<TwoDVector>::RadiusLimitedIterator iter =
            barriers.MakeRadiusLimitedIterator(position, maxDistanceToBarrier);

        while (iter != barriers.end()) {
            cachedInfo.lineSegments.push_back(*iter);
            ++iter;
        }//while//

        cachedInfo.hasBeenSet = true;

    }//if//


    for(unsigned int i = 0; (i < cachedInfo.lineSegments.size()); i++) {
        const LineSegmentType& lineSegment = cachedInfo.lineSegments[i];

        const TwoDVector closestPoint =
            CalcClosestPointOnLineSegmentToAPoint(
                lineSegment.v1, lineSegment.v2, position);

        barrierPoints.push_back(closestPoint);

    }//for//

}//GetCloseBarrierPoints//


inline
bool PedestrianSandbox::DoesNotTunnelThroughIntermediateWaypoint(
    const unsigned int waypoint1Id,
    const unsigned int waypoint2Id) const
{
    const WaypointPortal& waypoint1 = waypoints.at(waypoint1Id);
    const WaypointPortal& waypoint2 = waypoints.at(waypoint2Id);

    for(unsigned int i = 0; (i < waypoints.size()); i++) {

        if ((i != waypoint1Id) && (i != waypoint2Id)) {

            const WaypointPortal& aWaypoint = waypoints[i];

            if (LineSegmentsCross(
                   waypoint1.GetCenterPoint(),
                   waypoint2.GetCenterPoint(),
                   aWaypoint.GetFirstPoint(),
                   aWaypoint.GetSecondPoint())) {

                return false;

            }//if//
        }//if//
    }//for//

    return true;

}//DoesNotTunnelThroughIntermediateWaypoint//


inline
void PedestrianSandbox::CreateWaypointGraph()
{
    for (GraphVertexIdType i = 0; (i < waypoints.size()); i++) {
        (*this).waypointGraph.AddNewVertex(i);
    }//for//

    for (GraphVertexIdType i = 0; (i < waypoints.size()); i++) {
        const WaypointPortal& waypoint1 = waypoints[i];
        for (GraphVertexIdType j = (i+1); (j < waypoints.size()); j++) {
            const WaypointPortal& waypoint2 = waypoints[j];

            if ((!barriers.LineIntersectsALineInTree(
                    waypoint1.GetFirstPoint(),
                    waypoint2.GetFirstPoint())) &&
                (!barriers.LineIntersectsALineInTree(
                    waypoint1.GetFirstPoint(),
                    waypoint2.GetSecondPoint())) &&
                (!barriers.LineIntersectsALineInTree(
                    waypoint1.GetSecondPoint(),
                    waypoint2.GetFirstPoint())) &&
                (!barriers.LineIntersectsALineInTree(
                    waypoint1.GetSecondPoint(),
                    waypoint2.GetSecondPoint())) &&
                (DoesNotTunnelThroughIntermediateWaypoint(i, j))) {

                double distanceMeters =
                    CalcDistance(waypoint1.GetCenterPoint(), waypoint2.GetCenterPoint());

                WaypointLinkIdType linkId(waypoint1.GetFirstPoint(), waypoint2.GetFirstPoint());

                if (waypointGraphLinkWeightAdjustments.find(linkId) !=
                    waypointGraphLinkWeightAdjustments.end()) {

                    distanceMeters *= waypointGraphLinkWeightAdjustments[linkId];
                }//if//

                (*this).waypointGraph.AddBidirectionalEdge(i, j, distanceMeters);

            }//if//
        }//for//
    }//for//

    if(!GraphIsConnected(waypointGraph)) {
        cerr << "Warning: Waypoint Graph is Disconnected." << endl;
    }//if//

}//CreateWaypointGraph//



inline
void PedestrianSandbox::FindWaypoint(
    const TwoDVector& position,
    bool& wasFound,
    WaypointIdType& waypointId) const
{
    wasFound = false;
    waypointId = WaypointPortal::InvalidWaypointId;

    for(unsigned int i = 0; (i < waypoints.size()); i++) {
        if (waypoints[i].IsInsideWaypoint(position)) {
            waypointId = static_cast<WaypointIdType>(i);
            wasFound = true;
            break;
        }//if//
    }//for//

}//FindWaypoint//


inline
PedestrianSandbox::WaypointIdType PedestrianSandbox::FindClosestWaypoint(
    const TwoDVector& position) const
{
    WaypointIdType waypointId = WaypointPortal::InvalidWaypointId;
    double minDistanceSquared = DBL_MAX;

    for(unsigned int i = 0; (i < waypoints.size()); i++) {
        const double distanceSquared =
            CalcDistanceSquared(position, waypoints[i].GetCenterPoint());

        if (distanceSquared < minDistanceSquared) {
            minDistanceSquared = distanceSquared;
            waypointId = static_cast<WaypointIdType>(i);
        }//if//
    }//for//

    return (waypointId);

}//FindClosestWaypoint//



inline
PedestrianSandbox::WaypointIdType PedestrianSandbox::CalcNextWaypoint(
    const WaypointIdType& currentWaypoint,
    const WaypointIdType& destinationWaypoint) const
{
    class DistanceHeuristicFunction: public AstarHeuristicFunction {
    public:
        DistanceHeuristicFunction(const vector<WaypointPortal>& initWaypointsRef) :
            waypointsRef(initWaypointsRef) { }

        virtual double estimate(
            const GraphVertexIdType& startVertexId,
            const GraphVertexIdType& endVertexId) const override
        {
            return (
                CalcDistance(
                    waypointsRef.at(startVertexId).GetCenterPoint(),
                    waypointsRef.at(endVertexId).GetCenterPoint()));
        }

    private:
        const vector<WaypointPortal>& waypointsRef;

        DistanceHeuristicFunction(const DistanceHeuristicFunction&) = delete;
        void* operator new(size_t) = delete;

    };//DistanceHeuristicFunction//

    //-----------------------------------------------------

    vector<GraphVertexIdType> bestPath;

    AstarSearchGraph(
        DistanceHeuristicFunction(waypoints),
        waypointGraph,
        currentWaypoint,
        destinationWaypoint,
        bestPath);

    assert(bestPath.size() > 1);

    return (bestPath[1]);

}//CalcNextWaypoint//



//-----------------------------------------------------------------------------



class Pedestrian : public ScenSim::NetworkNode {
public:
    typedef unsigned int PedestrianIdType;

    Pedestrian(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const RandomNumberGeneratorSeedType& runSeed,
        const PedestrianIdType& pedId,
        const ScenSim::NodeIdType& nodeId);

    PedestrianIdType GetPedestrianId() const { return (pedestrianId); }

    bool IsEnabled() const {
        return ((nextWaypointId != WaypointPortal::InvalidWaypointId) &&
                (startTime <= simulationEngineInterfacePtr->CurrentTime()));
    }

    void SetStartTime(const TimeType& newStartTime) { startTime = newStartTime; }

    void SetDesiredSpeed(const double& newDesiredSpeed) { desiredSpeed = newDesiredSpeed; }
    double GetDesiredSpeed() const { return (desiredSpeed); }

    void SetPosition(const TwoDVector& newPosition) { currentPosition = newPosition; }
    TwoDVector GetCurrentPosition() const { return (currentPosition); }

    double GetXPos() const { return (currentPosition.GetX()); }
    double GetYPos() const { return (currentPosition.GetY()); }

    TwoDVector GetCurrentVelocityVector() const { return (currentVelocity); }
    void SetVelocityVector(const TwoDVector& newVelocity) { currentVelocity = newVelocity; }

    TwoDVector GetCurrentAttractorPoint() const { return (currentAttractorPoint); }

    void SetCurrentAttractorPoint(const TwoDVector& newAttractorPoint, const double& newRadius) {
        currentAttractorPoint = newAttractorPoint;
        currentAttractorRadiusSquared = (newRadius * newRadius);
    }

    double GetCurrentAttractorRadiusSquared() const { return (currentAttractorRadiusSquared); }

    WaypointPortal::WaypointIdType GetNextWaypointId() const { return (nextWaypointId); }
    void SetNextWaypoint(const WaypointPortal::WaypointIdType& newNextWaypointId)
    {
        lastWaypointId = nextWaypointId;
        nextWaypointId = newNextWaypointId;
    }

    // Used when Ped is pushed back through an achieved waypoint.

    WaypointPortal::WaypointIdType GetLastWaypointId() const { return (lastWaypointId); }
    void RevertToLastWaypoint();

    TwoDVector GetDestinationPosition() const { return (destinationPosition); }
    void SetDestinationPosition(const TwoDVector& newDestinationPosition)
        { destinationPosition = newDestinationPosition; }

    WaypointPortal::WaypointIdType GetDestinationWaypointId() const { return (destinationWaypointId); }
    void SetDestinationWaypoint(const WaypointPortal::WaypointIdType& newWaypointId)
        {  destinationWaypointId = newWaypointId; }

    virtual const ObjectMobilityPosition GetCurrentLocation() const override {
        return (
            ObjectMobilityPosition(
                INFINITE_TIME,
                ZERO_TIME,
                GetXPos(), GetYPos(), 0.0, false,
                0.0, 0.0, 0.0, 0.0, 0.0));
    }

    PedestrianSandbox::CachedBarrierInfoType& GetCachedBarrierInfo() { return (cachedBarrierInfo); }

private:

    PedestrianIdType pedestrianId;
    double desiredSpeed;
    TimeType startTime;

    TwoDVector currentPosition;
    TwoDVector currentVelocity;
    TwoDVector currentAttractorPoint;
    TwoDVector destinationPosition;
    WaypointPortal::WaypointIdType nextWaypointId;
    WaypointPortal::WaypointIdType lastWaypointId;
    WaypointPortal::WaypointIdType destinationWaypointId;
    double currentAttractorRadiusSquared;

    PedestrianSandbox::CachedBarrierInfoType cachedBarrierInfo;

};//Pedestrian//



inline
Pedestrian::Pedestrian(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const RandomNumberGeneratorSeedType& runSeed,
    const PedestrianIdType& pedId,
    const ScenSim::NodeIdType& nodeId)
    :
    NetworkNode(
        theParameterDatabaseReader,
        ScenSim::GlobalNetworkingObjectBag(),
        simulationEngineInterfacePtr,
        nullptr,
        nodeId,
        runSeed,
        true),
    pedestrianId(pedId),
    startTime(ZERO_TIME),
    desiredSpeed(0.0),
    nextWaypointId(WaypointPortal::InvalidWaypointId),
    lastWaypointId(WaypointPortal::InvalidWaypointId),
    destinationWaypointId(WaypointPortal::InvalidWaypointId),
    currentAttractorRadiusSquared(0.0)
{
}


inline
void Pedestrian::RevertToLastWaypoint()
{
    assert(lastWaypointId != WaypointPortal::InvalidWaypointId);
    nextWaypointId = lastWaypointId;
    lastWaypointId = WaypointPortal::InvalidWaypointId;
}

//--------------------------------------------------------------------------------------------------

class MultiAgentSim2d: public ScenSim::NetworkSimulator {
public:
    MultiAgentSim2d(
        const shared_ptr<ParameterDatabaseReader>& parameterDatabaseReaderPtr,
        const shared_ptr<SimulationEngine>& simulationEnginePtr,
        const RandomNumberGeneratorSeedType& runSeed);

    virtual ~MultiAgentSim2d() { nodes.clear(); }

private:
    static const double GetPedestrianQuadTreeMinSplitDistance() { return 1.0; }
    static const unsigned int pedestrianQuadTreeSplitSize = 10;
    static const unsigned int pedestrianQuadTreeCombineSize = 5;

    shared_ptr<ParameterDatabaseReader> parameterDatabaseReaderPtr;
    shared_ptr<SimulationEngine> simulationEnginePtr;
    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;

    ScenSim::NodeIdType currentSequentialNodeId;

    class TimeStepEvent : public SimulationEvent {
    public:
        TimeStepEvent(MultiAgentSim2d* initSimPtr) : simPtr(initSimPtr) { }
        void ExecuteEvent() { simPtr->ProcessTimeStepEvent(); }
    private:
        MultiAgentSim2d* simPtr;
    };

    shared_ptr<TimeStepEvent> timeStepEventPtr;

    TimeType stepDuration;
    double stepDurationSecs;

    bool pedestrianModelIsEnabled;
    bool vehicleModelIsEnabled;

    TimeType barrierRecalcTimeInterval;
    unsigned int barrierRecalcStepInterval;
    unsigned int barrierRecalcStepCount;

    double directionalFormFactor;
    double maxForceCalcDistanceMeters;
    double pedestrianInteractionStrengthFactor;
    double pedestrianInteractionRangeMeters;
    double barrierInteractionStrengthFactor;
    double barrierInteractionRangeMeters;


    double pedRepulsionRadiusMeters;
    double ignoreBarrierDistanceMeters;
    double accelerationRelaxationTimeSecs;
    double meanDesiredPedestrianSpeedMetersSec;
    double desiredPedSpeedStandardDeviationMetersSec;

    PedestrianSandbox pedSandbox;

    MovingPointQuadtree<Pedestrian> pedestriansInTree;
    vector<shared_ptr<Pedestrian> > pedestrians;

    IntelligentDriverModelParameters idmParams;
    AutoRoadNetwork roadNetwork;
    vector<RoadSegmentState> roadStates;
    vector<shared_ptr<Vehicle> > vehicles;

    //--------------------------------------------------

    void GetNewNodeId(ScenSim::NodeIdType& newNodeId);
    void LoadPeopleInfo();
    void LoadVehicleInfo();

    void ProcessTimeStepEvent();

    void MovePeds();
    void MoveVehicles();

    void UpdatePedsAttractorPoint(Pedestrian& aPed);
    void CalcAndSetNextWaypointForPed(Pedestrian& aPed);
    bool CheckIfWaypointReversionIsLikely(
        const Pedestrian& aPed,
        const TwoDVector& barrierForceVector,
        const TwoDVector& otherPedForceVector) const;

};//MultiAgentSim2d//



inline
MultiAgentSim2d::MultiAgentSim2d(
    const shared_ptr<ParameterDatabaseReader>& initParameterDatabaseReaderPtr,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const RandomNumberGeneratorSeedType& runSeed)
    :
    NetworkSimulator(
        initParameterDatabaseReaderPtr,
        initSimulationEnginePtr,
        runSeed,
        true),
    parameterDatabaseReaderPtr(initParameterDatabaseReaderPtr),
    simulationEnginePtr(initSimulationEnginePtr),
    simEngineInterfacePtr(
        initSimulationEnginePtr->GetSimulationEngineInterface(
            *initParameterDatabaseReaderPtr, ScenSim::ANY_NODEID)),
    pedestriansInTree(
        GetPedestrianQuadTreeMinSplitDistance(),
        pedestrianQuadTreeSplitSize,
        pedestrianQuadTreeCombineSize),
    pedSandbox(*initParameterDatabaseReaderPtr),
    roadNetwork(*initParameterDatabaseReaderPtr),
    barrierRecalcStepCount(0),
    currentSequentialNodeId(0),
    pedestrianModelIsEnabled(ReadPedestrianModelIsEnabled(*initParameterDatabaseReaderPtr)),
    vehicleModelIsEnabled(ReadVehicleModelIsEnabled(*initParameterDatabaseReaderPtr))
{
    roadStates.resize(roadNetwork.GetNumberRoadSegments());

    stepDuration =
        parameterDatabaseReaderPtr->ReadTime(parameterNamePrefix + "step-duration");

    stepDurationSecs = ConvertTimeToDoubleSecs(stepDuration);

    if (pedestrianModelIsEnabled) {
        barrierRecalcTimeInterval = MILLI_SECOND * 250;
        barrierRecalcStepInterval = static_cast<unsigned int>(barrierRecalcTimeInterval / stepDuration);

        directionalFormFactor =
            parameterDatabaseReaderPtr->ReadDouble(parameterNamePrefix + "directional-form-factor");

        maxForceCalcDistanceMeters =
            parameterDatabaseReaderPtr->ReadDouble(parameterNamePrefix + "max-force-calc-distance-meters");

        pedestrianInteractionStrengthFactor =
            parameterDatabaseReaderPtr->ReadDouble(parameterNamePrefix + "pedestrian-interaction-strength-factor");

        pedestrianInteractionRangeMeters =
            parameterDatabaseReaderPtr->ReadDouble(parameterNamePrefix + "pedestrian-interaction-range-meters");

        barrierInteractionStrengthFactor =
            parameterDatabaseReaderPtr->ReadDouble(parameterNamePrefix + "barrier-interaction-strength-factor");

        barrierInteractionRangeMeters =
            parameterDatabaseReaderPtr->ReadDouble(parameterNamePrefix + "barrier-interaction-range-meters");

        pedRepulsionRadiusMeters =
            parameterDatabaseReaderPtr->ReadDouble(parameterNamePrefix + "pedestrian-repulsion-radius-meters");

        accelerationRelaxationTimeSecs =
            parameterDatabaseReaderPtr->ReadDouble(parameterNamePrefix + "acceleration-relaxation-time-secs");

        meanDesiredPedestrianSpeedMetersSec =
            parameterDatabaseReaderPtr->ReadDouble(parameterNamePrefix + "mean-desired-pedestrian-speed-meters-sec");

        desiredPedSpeedStandardDeviationMetersSec =
            parameterDatabaseReaderPtr->ReadDouble(
                parameterNamePrefix + "desired-pedestrian-speed-standard-deviation-meters-sec");

        ignoreBarrierDistanceMeters =
            parameterDatabaseReaderPtr->ReadDouble(parameterNamePrefix + "ignore-barrier-distance-meters");

        (*this).pedestriansInTree.SetBoundingBox(
            pedSandbox.GetMinX(),
            pedSandbox.GetMaxX(),
            pedSandbox.GetMinY(),
            pedSandbox.GetMaxY());

        (*this).LoadPeopleInfo();
    }//if//

    if (vehicleModelIsEnabled) {

        ReadIntelligentDriverModelParameters(*parameterDatabaseReaderPtr, (*this).idmParams);

        (*this).LoadVehicleInfo();
    }//if//

    timeStepEventPtr.reset(new TimeStepEvent(this));
    simEngineInterfacePtr->ScheduleEvent(
        timeStepEventPtr,
        simEngineInterfacePtr->CurrentTime());

}//MultiAgentSim2d()//



inline
void MultiAgentSim2d::GetNewNodeId(ScenSim::NodeIdType& newNodeId)
{
    currentSequentialNodeId++;
    newNodeId = currentSequentialNodeId;
}




inline
void MultiAgentSim2d::LoadPeopleInfo()
{
    using std::min;
    using std::max;

    if (!parameterDatabaseReaderPtr->ParameterExists(parameterNamePrefix + "peoplefilename")) {
        return;
    }//if//

    const string peopleInfoFileName =
        parameterDatabaseReaderPtr->ReadString(parameterNamePrefix + "peoplefilename");

    std::ifstream peopleInfoStream(peopleInfoFileName);

    if (peopleInfoStream.fail()) {
        cerr << "Error opening file: " << peopleInfoFileName << endl;
        exit(1);
    }//if//


    while (!peopleInfoStream.eof()) {

        string aLine;
        getline(peopleInfoStream, aLine);

        if (peopleInfoStream.eof()) {
            break;
        }
        else if (peopleInfoStream.fail()) {
            cerr << "Error reading people configuration file: " << peopleInfoFileName << endl;
            exit(1);
        }//if//

        DeleteTrailingSpaces(aLine);
        if (IsAConfigFileCommentLine(aLine)) {
            continue;
        }//if//

        istringstream lineStream(aLine);

        double startTimeSecs;
        lineStream >> startTimeSecs;

        double locationXMeters;
        lineStream >> locationXMeters;

        double locationYMeters;
        lineStream >> locationYMeters;

        double destinationXMeters;
        lineStream >> destinationXMeters;

        double destinationYMeters;
        lineStream >> destinationYMeters;

        double desiredSpeedMetersSec;
        lineStream >> desiredSpeedMetersSec;

        if ((lineStream.fail()) || (!lineStream.eof()))  {
            cerr << "Error in file: " << peopleInfoFileName << endl;
            exit(1);
        }//if//

        const Pedestrian::PedestrianIdType pedId =
            static_cast<Pedestrian::PedestrianIdType>(pedestrians.size() + 1);

        ScenSim::NodeIdType nodeId;
        (*this).GetNewNodeId(nodeId);

        shared_ptr<Pedestrian> pedPtr =
            make_shared<Pedestrian>(
                *parameterDatabaseReaderPtr,
                simulationEnginePtr->GetSimulationEngineInterface(*parameterDatabaseReaderPtr, nodeId, 0),
                runSeed,
                pedId,
                nodeId);

        pedPtr->SetStartTime(ConvertDoubleSecsToTime(startTimeSecs));
        pedPtr->SetPosition(TwoDVector(locationXMeters, locationYMeters));
        pedPtr->SetDestinationPosition(TwoDVector(destinationXMeters, destinationYMeters));
        pedPtr->SetDesiredSpeed(desiredSpeedMetersSec);

        const WaypointPortal::WaypointIdType waypointId =
            pedSandbox.FindClosestWaypoint(pedPtr->GetCurrentPosition());
        pedPtr->SetNextWaypoint(waypointId);

        bool waypointWasFound;
        WaypointPortal::WaypointIdType destinationWaypointId;
        pedSandbox.FindWaypoint(pedPtr->GetDestinationPosition(), waypointWasFound, destinationWaypointId);

        assert(waypointWasFound && "TBD destination location is not atnot at waypoint");
        pedPtr->SetDestinationWaypoint(destinationWaypointId);

        CalcAndSetNextWaypointForPed(*pedPtr);

        (*this).pedestrians.push_back(pedPtr);
        (*this).pedestriansInTree.Insert(pedPtr);
        (*this).AddNode(pedPtr);

    }//while//

}//LoadPeopleInfo//



inline
void MultiAgentSim2d::LoadVehicleInfo()
{
    using std::min;
    using std::max;

    if (!parameterDatabaseReaderPtr->ParameterExists(parameterNamePrefix + "vehiclefilename")) {
        return;
    }//if//

    const string vehicleInfoFileName =
        parameterDatabaseReaderPtr->ReadString(parameterNamePrefix + "vehiclefilename");

    std::ifstream vehicleInfoStream(vehicleInfoFileName);

    if (vehicleInfoStream.fail()) {
        cerr << "Error opening file: " << vehicleInfoFileName << endl;
        exit(1);
    }//if//

    const string errorMessage = "Error in file: " + vehicleInfoFileName;

    while (!vehicleInfoStream.eof()) {

        string aLine;
        getline(vehicleInfoStream, aLine);

        if (vehicleInfoStream.eof()) {
            break;
        }
        else if (vehicleInfoStream.fail()) {
            cerr << "Error reading vehicle configuration file: " << vehicleInfoFileName << endl;
            exit(1);
        }//if//

        DeleteTrailingSpaces(aLine);
        if (IsAConfigFileCommentLine(aLine)) {
            continue;
        }//if//

        istringstream lineStream(aLine);

        double startTimeSecs;
        lineStream >> startTimeSecs;

        string roadName;
        lineStream >> roadName;
        ConvertStringToLowerCase(roadName);
        const unsigned int roadId = roadNetwork.GetRoadIndexFromName(aLine, roadName);

        unsigned int laneNumber;
        lineStream >> laneNumber;

        double roadPositionMeters;
        lineStream >> roadPositionMeters;

        double startSpeedMetersSec;
        lineStream >> startSpeedMetersSec;

        string destinationRoadName;
        lineStream >> destinationRoadName;
        ConvertStringToLowerCase(destinationRoadName);
        const unsigned int destinationRoadId =
            roadNetwork.GetRoadIndexFromName(aLine, destinationRoadName);

        unsigned int destinationLaneNumber;
        lineStream >> destinationLaneNumber;

        double destinationRoadPositionMeters;
        lineStream >> destinationRoadPositionMeters;

        if ((lineStream.fail()) || (!lineStream.eof()))  {
            cerr << errorMessage << endl;
            exit(1);
        }//if//

        const Vehicle::VehicleIdType vehicleId =
            static_cast<Vehicle::VehicleIdType>(vehicles.size() + 1);

        ScenSim::NodeIdType nodeId;
        (*this).GetNewNodeId(nodeId);

        shared_ptr<Vehicle> vehiclePtr =
            make_shared<Vehicle>(
                *parameterDatabaseReaderPtr,
                simulationEnginePtr->GetSimulationEngineInterface(*parameterDatabaseReaderPtr, nodeId, 0),
                runSeed,
                vehicleId,
                nodeId);

        (*this).vehicles.push_back(vehiclePtr);
        (*this).AddNode(vehiclePtr);

        vehiclePtr->SetStartTime(ConvertDoubleSecsToTime(startTimeSecs));

        roadNetwork.CheckRoadPosition(roadId, laneNumber, roadPositionMeters, errorMessage);
        roadNetwork.CheckRoadPosition(
            destinationRoadId,
            destinationLaneNumber,
            destinationRoadPositionMeters,
            errorMessage);

        vehiclePtr->SetRoadId(roadId);
        const RoadSegment& aRoad = roadNetwork.GetRoadSegment(roadId);
        PositionInRoadType position(roadPositionMeters, laneNumber);

        TwoDVector xyPosition;
        double azimuthClockwiseRad;
        aRoad.GetXyPositionAndAzimuth(
            position,
            xyPosition,
            azimuthClockwiseRad);

        vehiclePtr->SetRoadPosition(
            position,
            xyPosition,
            azimuthClockwiseRad);

        vehiclePtr->SetVelocity(startSpeedMetersSec);

        vehiclePtr->SetDestinationRoadId(destinationRoadId);
        vehiclePtr->SetDestinationRoadPosition(
            PositionInRoadType(destinationRoadPositionMeters, destinationLaneNumber));

        deque<RoadSegmentIdType> newRoute;

        roadNetwork.CalcRoute(
            vehiclePtr->GetCurrentRoadId(),
            vehiclePtr->GetCurrentPosition(),
            vehiclePtr->GetDestinationRoadId(),
            vehiclePtr->GetDestinationPosition(),
            newRoute);

        vehiclePtr->SetRoute(newRoute);

        if (vehiclePtr->IsEnabled()) {
            (*this).roadStates.at(roadId).AddVehicle(vehiclePtr);
        }//if//

    }//while//

}//LoadVehicleInfo//



inline
void MultiAgentSim2d::ProcessTimeStepEvent()
{
    (*this).MoveVehicles();
    (*this).MovePeds();

    simEngineInterfacePtr->ScheduleEvent(
        timeStepEventPtr,
        (simEngineInterfacePtr->CurrentTime() + stepDuration));

}//ProcessTimeStepEvent//



inline
TwoDVector CalcSocialForceFromOtherPedestrian(
    const double& directionalFormFactor,
    const double& interactionStrengthFactor,
    const double& interactionRadiusFactor,
    const double& maxForceCalcDistance,
    const TwoDVector& pedPos,
    const bool currentlyIsNotMoving,
    const TwoDVector& pedVelocityUnitVector,
    const double& pedRepulsionRadius,
    const TwoDVector& otherPedPos,
    const double& otherPedRepulsionRadius)
{
    using std::max;
    const double centersDistance = CalcDistance(pedPos, otherPedPos);

    if ((centersDistance >= maxForceCalcDistance) || (centersDistance == 0.0)) {
        return (TwoDVector(0.0, 0.0));
    }//if//

    assert(centersDistance > 0.0);

    const TwoDVector forceUnitVector = ((pedPos - otherPedPos) / centersDistance);

    double directionalFactor = 1.0;

    if (!currentlyIsNotMoving) {
        const double cosVelocityAngle = CalcDotProduct(-forceUnitVector, pedVelocityUnitVector);

        directionalFactor =
            directionalFormFactor + (((1 - directionalFormFactor) * (1 + cosVelocityAngle)) / 2.0);
    }//if//

    const double force =
        directionalFactor * interactionStrengthFactor *
        exp(((pedRepulsionRadius + otherPedRepulsionRadius) - centersDistance) / interactionRadiusFactor);

    return (force * forceUnitVector);

}//CalcSocialForceFromOtherPedestrian//



inline
TwoDVector CalcSocialForceFromBarrierPoint(
    const double& interactionStrengthFactor,
    const double& interactionRange,
    const double& maxForceCalcDistance,
    const TwoDVector& pedPos,
    const double& pedRepulsionRadius,
    const TwoDVector& barrierPoint)
{
    using std::max;

    const double distance = CalcDistance(pedPos, barrierPoint);

    if (distance >= maxForceCalcDistance) {
        return (TwoDVector(0.0, 0.0));
    }//if//

    assert(distance > 0.0);

    const TwoDVector forceUnitVector = ((pedPos - barrierPoint) / distance);

    const double force =
        interactionStrengthFactor * exp(-((distance - pedRepulsionRadius) / interactionRange));

    return (force * forceUnitVector);

}//CalcSocialForceFromBarrierPoint//



inline
bool CheckIfPedestrianSeemsStuckBehindBarrier(
    const double& desiredVelocityMetersSec,
    const double& accelerationRelaxationTimeSecs,
    const TwoDVector& desiredVelocityAccelerationVector,
    const TwoDVector& barrierForceVector,
    const TwoDVector& otherPedForceVector)
{
    const double lowOtherPedFactor = 100.0;
    const double likelyStuckFactor = 0.90;
    const double likelyStuckCosOfAngleFactor = -0.98;

    const double barrierForceSquared = barrierForceVector.LengthSquared();
    const double otherPedForceSquared = otherPedForceVector.LengthSquared();

    // Check that the forces from other Peds are not relevant.

    if (barrierForceSquared < otherPedForceSquared) {
        return false;
    }//if//

    if ((otherPedForceSquared < DBL_EPSILON) ||
        ((barrierForceSquared / otherPedForceSquared) > lowOtherPedFactor)) {

        const double maxDesiredSpeedAcceleration =
            (desiredVelocityMetersSec / accelerationRelaxationTimeSecs);

        const double maxDesiredSpeedAccelerationSquared =
            (maxDesiredSpeedAcceleration * maxDesiredSpeedAcceleration);

        // Check that barrier force and desired speed acceleration are close.

        if ((barrierForceSquared / maxDesiredSpeedAccelerationSquared) > likelyStuckFactor) {

            // Check the angle between the two forces, must be close 180 degrees apart.

            const double cosOfAngle =
                CalcDotProduct(
                    CalcNormalizedVector(barrierForceVector),
                    CalcNormalizedVector(desiredVelocityAccelerationVector));

           return (cosOfAngle < likelyStuckCosOfAngleFactor);

        }//if//
    }//if//

    return false;

}//CheckIfPedestrianSeemsStuckBehindBarrier//



inline
void CalcNextPedestrianPositionAndVelocity(
    const double& stepDurationSecs,
    const double& accelerationRelaxationTimeSecs,
    const TwoDVector& currentPosition,
    const TwoDVector& currentVelocityVectorMetersSec,
    const TwoDVector& attractorPosition,
    const double& desiredVelocityMetersSec,
    const TwoDVector& barrierForceVector,
    const TwoDVector& otherPedsForceVector,
    TwoDVector& nextPosition,
    TwoDVector& nextVelocityVector,
    bool& pedSeemsStuck)
{
    assert(stepDurationSecs <= accelerationRelaxationTimeSecs);

    const TwoDVector forceAccelerationVector = barrierForceVector + otherPedsForceVector;

    // "Acceleration to desired velocity" plus the incoming force/acceleration vector.

    const TwoDVector desiredVelocityVectorMetersSec =
        CalcNormalizedVector(attractorPosition - currentPosition) * desiredVelocityMetersSec;

    const TwoDVector desiredVelocityAccelerationVector =
        ((desiredVelocityVectorMetersSec - currentVelocityVectorMetersSec) /
         accelerationRelaxationTimeSecs);

    const TwoDVector deltaVelocityVector =
        (desiredVelocityAccelerationVector + forceAccelerationVector) * stepDurationSecs;

    // Acceleration is assumed to be constant between steps.

    nextVelocityVector = currentVelocityVectorMetersSec + deltaVelocityVector;
    const TwoDVector averageVelocityVector = (currentVelocityVectorMetersSec + nextVelocityVector) * 0.5;

    nextPosition = currentPosition + (averageVelocityVector * stepDurationSecs);

    pedSeemsStuck =
        CheckIfPedestrianSeemsStuckBehindBarrier(
            desiredVelocityMetersSec,
            accelerationRelaxationTimeSecs,
            desiredVelocityAccelerationVector,
            barrierForceVector,
            otherPedsForceVector);

}//CalcNextPedestrianPosition//



inline
void MultiAgentSim2d::UpdatePedsAttractorPoint(Pedestrian& aPed)
{
    const WaypointPortal& aWaypoint = pedSandbox.GetWaypoint(aPed.GetNextWaypointId());

    const TwoDVector attractorPoint =
        CalcClosestPointOnLineSegmentToAPoint(
            aWaypoint.GetFirstPoint(),
            aWaypoint.GetSecondPoint(),
            aPed.GetCurrentPosition());

    aPed.SetCurrentAttractorPoint(attractorPoint, aWaypoint.GetRadius());

}//UpdatePedsAttactorPoint//


inline
void MultiAgentSim2d::CalcAndSetNextWaypointForPed(Pedestrian& aPed)
{
    const WaypointPortal::WaypointIdType currentWaypointId = aPed.GetNextWaypointId();

    if (currentWaypointId == aPed.GetDestinationWaypointId()) {
        // Destination achieved.

        const WaypointPortal::WaypointIdType InvalidWaypointId = WaypointPortal::InvalidWaypointId;

        aPed.SetNextWaypoint(InvalidWaypointId);
        aPed.SetCurrentAttractorPoint(aPed.GetCurrentPosition(), DBL_EPSILON);
        aPed.SetVelocityVector(TwoDVector(0.0, 0.0));
        return;
    }//if//

    const WaypointPortal::WaypointIdType nextWaypointId =
        pedSandbox.CalcNextWaypoint(currentWaypointId, aPed.GetDestinationWaypointId());

    aPed.SetNextWaypoint(nextWaypointId);

    (*this).UpdatePedsAttractorPoint(aPed);

}//CalcAndSetNextWaypoint//




inline
void MultiAgentSim2d::MovePeds()
{
    const TimeType currentTime = simulationEnginePtr->CurrentTime();

    const bool useCachedBarrierInfo = (barrierRecalcStepCount != 0);
    (*this).barrierRecalcStepCount++;
    if (barrierRecalcStepCount >= barrierRecalcStepInterval) {
        (*this).barrierRecalcStepCount = 0;
    }//if//

    vector<TwoDVector> nextPedPosition(pedestrians.size());
    vector<TwoDVector> nextPedVelocityVector(pedestrians.size());

    for(unsigned int i = 0; (i < pedestrians.size()); i++) {

        Pedestrian& aPed = *pedestrians[i];

        if (!aPed.IsEnabled()) {
            continue;
        }

        const TwoDVector pedPosition = aPed.GetCurrentPosition();
        const TwoDVector pedVelocityVector = aPed.GetCurrentVelocityVector();

        // Handle case with no velocity direction.

        bool currentlyIsNotMoving = true;
        TwoDVector pedVelocityUnitVector;
        if (pedVelocityVector.LengthSquared() > DBL_EPSILON) {
            pedVelocityUnitVector = CalcNormalizedVector(pedVelocityVector);
            currentlyIsNotMoving = false;
        }//if//

        TwoDVector barrierForceVector(0.0, 0.0);

        vector<TwoDVector> barrierPoints;

        pedSandbox.GetCloseBarrierPoints(
            pedPosition,
            ignoreBarrierDistanceMeters,
            useCachedBarrierInfo,
            aPed.GetCachedBarrierInfo(),
            barrierPoints);

        for(unsigned int j = 0; (j < barrierPoints.size()); j++) {

            barrierForceVector +=
                CalcSocialForceFromBarrierPoint(
                    barrierInteractionStrengthFactor,
                    barrierInteractionRangeMeters,
                    maxForceCalcDistanceMeters,
                    pedPosition,
                    pedRepulsionRadiusMeters,
                    barrierPoints[j]);
        }//for//

        TwoDVector otherPedForceVector(0.0, 0.0);

        const double halfRadiusMeters = (maxForceCalcDistanceMeters / 2.0);
        const double minX = pedPosition.GetX() - halfRadiusMeters;
        const double maxX = pedPosition.GetX() + halfRadiusMeters;
        const double minY = pedPosition.GetY() - halfRadiusMeters;
        const double maxY = pedPosition.GetY() + halfRadiusMeters;

        MovingPointQuadtree<Pedestrian>::BoxLimitedIterator iter =
            pedestriansInTree.MakeBoxLimitedIterator(minX, maxX, minY, maxY);

        while (iter != pedestriansInTree.end()) {

            if ((*iter).GetPedestrianId() != aPed.GetPedestrianId() &&
                ((*iter).IsEnabled())) {

                otherPedForceVector +=
                    CalcSocialForceFromOtherPedestrian(
                        directionalFormFactor,
                        pedestrianInteractionStrengthFactor,
                        pedestrianInteractionRangeMeters,
                        maxForceCalcDistanceMeters,
                        pedPosition,
                        currentlyIsNotMoving,
                        pedVelocityUnitVector,
                        pedRepulsionRadiusMeters,
                        (*iter).GetCurrentPosition(),
                        pedRepulsionRadiusMeters);
            }//if//

            ++iter;

        }//while//

        // Likely do not have update so frequently.

        (*this).UpdatePedsAttractorPoint(aPed);

        bool pedSeemsStuck;

        CalcNextPedestrianPositionAndVelocity(
            stepDurationSecs,
            accelerationRelaxationTimeSecs,
            pedPosition,
            pedVelocityVector,
            aPed.GetCurrentAttractorPoint(),
            aPed.GetDesiredSpeed(),
            barrierForceVector,
            otherPedForceVector,
            nextPedPosition[i],
            nextPedVelocityVector[i],
            pedSeemsStuck);

        // Check if Pedestrian has been pushed back through achieved waypoint.

        if ((pedSeemsStuck) && (aPed.GetLastWaypointId() != WaypointPortal::InvalidWaypointId)) {
            aPed.RevertToLastWaypoint();
        }//if//
    }//for//

    for(unsigned int i = 0; (i < pedestrians.size()); i++) {

        Pedestrian& aPed = *pedestrians[i];

        if (!aPed.IsEnabled()) {
            continue;
        }

        aPed.SetPosition(nextPedPosition[i]);
        aPed.SetVelocityVector(nextPedVelocityVector[i]);

        //cerr << ConvertTimeToDoubleSecs(currentTime) << " " << i << " " << aPed.GetXPos() << " " << aPed.GetYPos() << endl;


        if (CalcDistanceSquared(aPed.GetCurrentPosition(), aPed.GetCurrentAttractorPoint()) <
            aPed.GetCurrentAttractorRadiusSquared()) {

           (*this).CalcAndSetNextWaypointForPed(aPed);

        }//if//

    }//for//

    (*this).pedestriansInTree.UpdateForMovement();


}//MovePeds//


inline
void MultiAgentSim2d::MoveVehicles()
{
    const TimeType currentTime = simulationEnginePtr->CurrentTime();

    MoveTheVehicles(
        currentTime,
        stepDuration,
        stepDurationSecs,
        idmParams,
        roadNetwork,
        (*this).vehicles,
        (*this).roadStates);

}//MoveVehicles//


}//namespace//

#endif
