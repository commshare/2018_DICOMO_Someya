// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include <iostream>
#include <fstream>
#include "scensim_support.h"
#include "multiagent2_gis.h"
#include "shapefil.h"

namespace MultiAgent2 {

using std::cerr;
using std::endl;
using std::ifstream;
using std::istringstream;
using ScenSim::DeleteTrailingSpaces;
using ScenSim::IsAConfigFileCommentLine;
using ScenSim::ConvertStringToLowerCase;

inline
int GetShapeDbfFieldIndex(const DBFHandle& hDBF, const string& fieldName)
{
    const int fieldIndex = DBFGetFieldIndex(hDBF, fieldName.c_str());

    if (fieldIndex < 0) {
        cerr << "Error in Shape file DBF: Field \"" << fieldName << "\" not found." << endl;
        exit(1);
    }//if//

    return (fieldIndex);

}//GetShapeDbfFieldIndex//



void ReadRoads(
    const string& gisRoadsFilenamePrefix,
    double& minX,
    double& maxX,
    double& minY,
    double& maxY,
    vector<GisRoadSegment>& roadSegments)
{
    roadSegments.clear();

    const SHPHandle hSHP = SHPOpen((gisRoadsFilenamePrefix + ".shp").c_str(), "rb");
    if (hSHP == nullptr) {
        cerr << "Cannot open .shp file: " << (gisRoadsFilenamePrefix + ".shp") << endl;
        exit(1);
    }//if//

    const DBFHandle hDBF = DBFOpen((gisRoadsFilenamePrefix + ".dbf").c_str(), "rb");
    if (hDBF == NULL) {
        cerr << "Cannot open .dbf file: " << (gisRoadsFilenamePrefix + ".dbf") << endl;
        exit(1);
    }//if//

    const int nameIndex = GetShapeDbfFieldIndex(hDBF, "name");
    const int roadWidthFieldIndex = GetShapeDbfFieldIndex(hDBF, "width");
    const int numberForwardLanesIndex = GetShapeDbfFieldIndex(hDBF, "lane12");
    const int numberBackwardLanesIndex = GetShapeDbfFieldIndex(hDBF, "lane21");
    const int speedLimitIndex = GetShapeDbfFieldIndex(hDBF, "speedlimit");

    int numEntities = 0;
    double adfMinBound[4];
    double adfMaxBound[4];

    SHPGetInfo(hSHP, &numEntities, nullptr, adfMinBound, adfMaxBound);
    minX = adfMinBound[0];
    maxX = adfMaxBound[0];
    minY = adfMinBound[1];
    maxY = adfMaxBound[1];

    for(int i = 0; (i < numEntities); i++) {

        string roadName(DBFReadStringAttribute(hDBF, i, nameIndex));
        ConvertStringToLowerCase(roadName);

        const double roadWidth =
            DBFReadDoubleAttribute(hDBF, i, roadWidthFieldIndex);

        const int numberForwardLanes =
            DBFReadIntegerAttribute(hDBF, i, numberForwardLanesIndex);

        const int numberBackwardLanes =
            DBFReadIntegerAttribute(hDBF, i, numberBackwardLanesIndex);

        const double speedLimitKmHour = DBFReadDoubleAttribute(hDBF, i, speedLimitIndex);
        const double speedLimitMetersSec = speedLimitKmHour * (1000.0 / 3600.0);

        assert((numberForwardLanes + numberBackwardLanes) != 0);

        const double laneWidth = (roadWidth / (numberForwardLanes + numberBackwardLanes));

        SHPObject* shpObjPtr = SHPReadObject(hSHP, i);

        for(int j = 0; (j < (shpObjPtr->nVertices - 1)); j++) {

            const double x1 = shpObjPtr->padfX[j];
            const double y1 = shpObjPtr->padfY[j];
            const double x2 = shpObjPtr->padfX[j+1];
            const double y2 = shpObjPtr->padfY[j+1];

            roadSegments.push_back(
                GisRoadSegment(
                    i, roadName, x1, y1, x2, y2,
                    roadWidth,
                    numberForwardLanes,
                    numberBackwardLanes,
                    laneWidth,
                    speedLimitKmHour));
        }//for//

        SHPDestroyObject(shpObjPtr);

    }//for//

    DBFClose(hDBF);
    SHPClose(hSHP);

}//ReadRoads//



void ReadIntersections(
    const string& gisIntersectionsFilenamePrefix,
    double& minX,
    double& maxX,
    double& minY,
    double& maxY,
    vector<GisIntersection>& intersections)
{
    intersections.clear();

    const SHPHandle hSHP = SHPOpen((gisIntersectionsFilenamePrefix + ".shp").c_str(), "rb");
    if (hSHP == nullptr) {
        cerr << "Cannot open .shp file: " << (gisIntersectionsFilenamePrefix + ".shp") << endl;
        exit(1);
    }//if//

    const DBFHandle hDBF = DBFOpen((gisIntersectionsFilenamePrefix + ".dbf").c_str(), "rb");
    if (hDBF == NULL) {
        cerr << "Cannot open .dbf file: " << (gisIntersectionsFilenamePrefix + ".dbf") << endl;
        exit(1);
    }//if//

    const int intersectionNameFieldIndex = GetShapeDbfFieldIndex(hDBF, "name");

    int numEntities = 0;
    double adfMinBound[4];
    double adfMaxBound[4];

    SHPGetInfo(hSHP, &numEntities, nullptr, adfMinBound, adfMaxBound);
    minX = adfMinBound[0];
    maxX = adfMaxBound[0];
    minY = adfMinBound[1];
    maxY = adfMaxBound[1];

    for(int i = 0; (i < numEntities); i++) {

        const string nameString(DBFReadStringAttribute(hDBF, i, intersectionNameFieldIndex));

        SHPObject* shpObjPtr = SHPReadObject(hSHP, i);
        if (shpObjPtr->nVertices != 1) {
            cerr << "Error in Intersection file: " << gisIntersectionsFilenamePrefix + ".shp" << endl;
            exit(1);
        }//if//
        const double x = shpObjPtr->padfX[0];
        const double y = shpObjPtr->padfY[0];

        intersections.push_back(GisIntersection(i, nameString, x, y));

        SHPDestroyObject(shpObjPtr);

    }//for//

    DBFClose(hDBF);
    SHPClose(hSHP);

}//ReadIntersections//


void ReadBuildingPolygons(
    const string& gisBuildingFilenamePrefix,
    double& minX,
    double& maxX,
    double& minY,
    double& maxY,
    vector<GisBuildingPolygon>& buildingPolygons)
{
    buildingPolygons.clear();

    const SHPHandle hSHP = SHPOpen((gisBuildingFilenamePrefix + ".shp").c_str(), "rb");
    if (hSHP == nullptr) {
        cerr << "Cannot open .shp file: " << (gisBuildingFilenamePrefix + ".shp") << endl;
        exit(1);
    }//if//

    const DBFHandle hDBF = DBFOpen((gisBuildingFilenamePrefix + ".dbf").c_str(), "rb");
    if (hDBF == NULL) {
        cerr << "Cannot open .dbf file: " << (gisBuildingFilenamePrefix + ".dbf") << endl;
        exit(1);
    }//if//

    int numEntities = 0;

    double adfMinBound[4];
    double adfMaxBound[4];

    SHPGetInfo(hSHP, &numEntities, nullptr, adfMinBound, adfMaxBound);
    minX = adfMinBound[0];
    maxX = adfMaxBound[0];
    minY = adfMinBound[1];
    maxY = adfMaxBound[1];

    for(int i = 0; (i < numEntities); i++) {

        SHPObject* shpObjPtr = SHPReadObject(hSHP, i);

        buildingPolygons.push_back(GisBuildingPolygon());

        GisBuildingPolygon& buildingPolygon = buildingPolygons.back();

        for(int j = 0; (j < (shpObjPtr->nVertices)); j++) {

            buildingPolygon.buildingId = static_cast<unsigned int>(i);
            buildingPolygon.buildingVertices.push_back(
                TwoDVector(shpObjPtr->padfX[j], shpObjPtr->padfY[j]));

        }//for//

        SHPDestroyObject(shpObjPtr);

    }//for//

    DBFClose(hDBF);
    SHPClose(hSHP);

}//ReadBuildingPolygons//




void ReadWaypointPortals(
    const string& gisWaypointFilenamePrefix,
    const double& closeEnoughToWaypointDistanceMeters,
    vector<WaypointPortal>& waypoints)
{
    waypoints.clear();


    const SHPHandle hSHP = SHPOpen((gisWaypointFilenamePrefix + ".shp").c_str(), "rb");
    if (hSHP == nullptr) {
        cerr << "Cannot open .shp file: " << (gisWaypointFilenamePrefix + ".shp") << endl;
        exit(1);
    }//if//

    int numEntities = 0;

    double adfMinBound[4];
    double adfMaxBound[4];

    SHPGetInfo(hSHP, &numEntities, nullptr, adfMinBound, adfMaxBound);

    for(int i = 0; (i < numEntities); i++) {

        const SHPObject* const shpObjPtr = SHPReadObject(hSHP, i);
        assert(shpObjPtr->nVertices == 2);

        const double x1 = shpObjPtr->padfX[0];
        const double y1 = shpObjPtr->padfY[0];
        const double x2 = shpObjPtr->padfX[1];
        const double y2 = shpObjPtr->padfY[1];

        waypoints.push_back(WaypointPortal(x1, y1, x2, y2, closeEnoughToWaypointDistanceMeters));

    }//for//

}//ReadWaypointPortals//


}//namespace//




