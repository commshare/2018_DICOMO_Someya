// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef MULTIAGENT_COMMON_H
#define MULTIAGENT_COMMON_H

#include <assert.h>
#include <float.h>
#include <cmath>
#include <algorithm>
#include <string>

namespace MultiAgent2 {

using std::string;

const string parameterNamePrefix("multiagent2-");

using std::sqrt;

//-------------------------------------------------------------------------------------------------

struct TwoDVector {
    double x;
    double y;

    bool IsTheNullVector() const { return ((x == 0.0) && (y == 0.0)); }

    double GetX() const { return (x); }
    double GetY() const { return (y); }

    TwoDVector() : x(0.0), y(0.0) {}

    TwoDVector(const double& initX, const double& initY) : x(initX), y(initY) {}

    double LengthSquared() const { return (x*x + y*y); }
    double Length() const { return (sqrt(LengthSquared())); }

    void operator+=(const TwoDVector& right);
    void operator-=(const TwoDVector& right);
    TwoDVector operator+(const TwoDVector& right) const;
    TwoDVector operator-(const TwoDVector& right) const;
    TwoDVector operator-() const;

    bool operator<(const TwoDVector& right) const
    {
        return (((*this).x < right.x) || (((*this).x == right.x) && ((*this).y < right.y)));
    }

    bool operator==(const TwoDVector& right) const
    {
        return (((*this).x == right.x) && (((*this).y == right.y)));
    }

    bool operator!=(const TwoDVector& right) const
    {
        return (!((*this) == right));
    }


};//TwoDVector//


inline
void TwoDVector::operator+=(const TwoDVector& right)
{
    (*this).x += right.x;
    (*this).y += right.y;
}

inline
void TwoDVector::operator-=(const TwoDVector& right)
{
    (*this).x -= right.x;
    (*this).y -= right.y;
}


inline
TwoDVector TwoDVector::operator+(const TwoDVector& right) const
{
    return (TwoDVector(((*this).x + right.x), ((*this).y + right.y)));
}


inline
TwoDVector TwoDVector::operator-(const TwoDVector& right) const
{
    return (TwoDVector(((*this).x - right.x), ((*this).y - right.y)));
}

inline
TwoDVector TwoDVector::operator-() const
{
    return (TwoDVector(-x, -y));
}


inline
TwoDVector operator*(const TwoDVector& left, const double& right)
{
    return (TwoDVector((left.x * right), (left.y * right)));
}


inline
TwoDVector operator*(const double& left, const TwoDVector& right)
{
    return (right * left);
}

inline
TwoDVector operator/(const TwoDVector& left, const double& right)
{
    assert(right != 0.0);
    return (TwoDVector((left.x / right), (left.y / right)));
}


inline
double CalcDistanceSquared(const double& x1, const double& y1, const double& x2, const double& y2)
{
    const double deltaX = (x1 - x2);
    const double deltaY = (y1 - y2);
    return ((deltaX * deltaX) + (deltaY * deltaY));
}

inline
double CalcDistance(const double& x1, const double& y1, const double& x2, const double& y2)
{
    return (std::sqrt(CalcDistanceSquared(x1, y1, x2, y2)));
}

inline
double CalcDistanceSquared(const TwoDVector& left, const TwoDVector& right)
{
    return (CalcDistanceSquared(left.x, left.y, right.x, right.y));
}


inline
double CalcDistance(const TwoDVector& left, const TwoDVector& right)
{
    return (CalcDistance(left.x, left.y, right.x, right.y));
}


inline
double CalcDotProduct(const TwoDVector& left, const TwoDVector& right)
{
    return ((left.x * right.x) + (left.y * right.y));
}


inline
TwoDVector CalcNormalizedVector(const TwoDVector& aVector)
{
    return (aVector / aVector.Length());
}



inline
double CalcAngleBetweenVectorsRad(const TwoDVector& v1, const TwoDVector& v2)
{
    return (
        std::atan2(
            ((v1.GetX() * v2.GetY()) - (v1.GetY() * v2.GetX())),
             ((v1.GetX() * v2.GetX()) + (v1.GetY() * v2.GetY()))));
}


inline
double CalcClockwiseAzimuthAngleChangeRad(
    const TwoDVector& firstVector, const TwoDVector& midVector, const TwoDVector& lastVector)
{
    return (CalcAngleBetweenVectorsRad((lastVector - midVector), (midVector - firstVector)));
}



inline
double CalcAzimuthOfLineClockwiseFromNorthRad(const TwoDVector& v1, const TwoDVector& v2)
{
    const TwoDVector deltaVector = (v2 - v1);
    return (-std::atan2(-deltaVector.GetX(), deltaVector.GetY()));
}



//-------------------------------------------------------------------------------------------------


inline
TwoDVector CalcIntermediatePoint(
    const TwoDVector& v1,
    const TwoDVector& v2,
    const double& fractionOfTotalLength)
{
    assert(fractionOfTotalLength >= 0.0);
    assert(fractionOfTotalLength <= 1.0);

    return (
        TwoDVector(
            (((v2.x-v1.x) * fractionOfTotalLength) + v1.x),
            (((v2.y-v1.y) * fractionOfTotalLength) + v1.y)));
}



inline
TwoDVector CalcClosestPointOnLineSegmentToAPoint(
    const TwoDVector& lineVertex1,
    const TwoDVector& lineVertex2,
    const TwoDVector& point)
{
    const TwoDVector lineVector = (lineVertex2 - lineVertex1);
    const double lineLength = lineVector.Length();
    if (lineLength == 0.0) {
        return (lineVertex1);
    }//if//

    const TwoDVector unitLineVector = (lineVector / lineLength);

    const double lineDistance = CalcDotProduct((point - lineVertex1), unitLineVector);

    if (lineDistance <= 0.0) {
        return (lineVertex1);
    }
    else if (lineDistance >= lineLength) {
        return (lineVertex2);
    }
    else {
        return (lineVertex1 + (lineDistance * unitLineVector));
    }//if//

}//CalcClosestPointOnLineSegmentToAPoint//



inline
void CalcIntermediatePoint(
    const double& x1,
    const double& y1,
    const double& x2,
    const double& y2,
    const double& fractionOfTotalLength,
    double& x,
    double& y)
{
    assert(fractionOfTotalLength >= 0.0);
    assert(fractionOfTotalLength <= 1.0);

    x = ((x2-x1) * fractionOfTotalLength) + x1;
    y = ((y2-y1) * fractionOfTotalLength) + y1;
}



inline
bool LineSegmentsCross(
    const TwoDVector& line1V1,
    const TwoDVector& line1V2,
    const TwoDVector& line2V1,
    const TwoDVector& line2V2)
{
    using std::min;
    using std::max;

    const double denominator =
        (((line1V1.GetX() - line1V2.GetX()) * (line2V1.GetY() - line2V2.GetY())) -
         ((line1V1.GetY() - line1V2.GetY()) * (line2V1.GetX() - line2V2.GetX())));

    if (std::abs(denominator) < DBL_EPSILON) {
        return false;
    }//if//

    // Wikipedia formula based on determinates.

    const double xIntersect =
        ((((line1V1.GetX() * line1V2.GetY()) - (line1V1.GetY() * line1V2.GetX())) *
          (line2V1.GetX() - line2V2.GetX())) -
         ((line1V1.GetX() - line1V2.GetX()) *
          ((line2V1.GetX() * line2V2.GetY()) - (line2V1.GetY() * line2V2.GetX())))) / denominator;

    const double minX = max(min(line1V1.GetX(), line1V2.GetX()), min(line2V1.GetX(), line2V2.GetX()));
    if (minX > xIntersect) {
        return false;
    }//if//

    const double maxX = min(max(line1V1.GetX(), line1V2.GetX()), max(line2V1.GetX(), line2V2.GetX()));
    return (xIntersect <= maxX);

}//LineSegmentsIntersect//



inline
void CalcIntersectionPointOfTwoLines(
    const TwoDVector& line1V1,
    const TwoDVector& line1V2,
    const TwoDVector& line2V1,
    const TwoDVector& line2V2,
    bool& pointExists,
    TwoDVector& point)
{
    using std::min;
    using std::max;

    pointExists = false;

    // Wikipedia formula based on determinates.

    const double denominator =
        (((line1V1.GetX() - line1V2.GetX()) * (line2V1.GetY() - line2V2.GetY())) -
         ((line1V1.GetY() - line1V2.GetY()) * (line2V1.GetX() - line2V2.GetX())));

    if (std::abs(denominator) < DBL_EPSILON) {
        return;
    }//if//

    pointExists = true;

    const double term1 = ((line1V1.GetX() * line1V2.GetY()) - (line1V1.GetY() * line1V2.GetX()));
    const double term2 = ((line2V1.GetX() * line2V2.GetY()) - (line2V1.GetY() * line2V2.GetX()));

    point.x =
        ((term1 * (line2V1.GetX() - line2V2.GetX())) - ((line1V1.GetX() - line1V2.GetX()) * term2)) /
        denominator;

    point.y =
        ((term1 * (line2V1.GetY() - line2V2.GetY())) - ((line1V1.GetY() - line1V2.GetY()) * term2)) /
        denominator;

}//CalcIntersectionPointOfTwoLines//



inline
void CalcParallelLine(
    const TwoDVector& lineVertex1,
    const TwoDVector& lineVertex2,
    const double& offset,
    TwoDVector& newLineVertex1,
    TwoDVector& newLineVertex2)
{
    const TwoDVector deltaVector = lineVertex2 - lineVertex1;
    const double lineLength = deltaVector.Length();
    const TwoDVector normalizedDeltaVector = deltaVector / lineLength;
    const TwoDVector perpendicularUnitVector(-normalizedDeltaVector.y, normalizedDeltaVector.x);
    const TwoDVector offsetVector = perpendicularUnitVector * offset;

    newLineVertex1 = lineVertex1 + offsetVector;
    newLineVertex2 = lineVertex2 + offsetVector;

}//CalcParallelLine//



inline
void CalcPerpendicularLine(
    const TwoDVector& lineVertex1,
    const TwoDVector& lineVertex2,
    const double& offsetOnLineFromVertex1,
    const double& newLineLength,
    TwoDVector& newLineVertex1,
    TwoDVector& newLineVertex2)
{
    const TwoDVector deltaVector = lineVertex2 - lineVertex1;
    const double lineLength = deltaVector.Length();
    const TwoDVector normalizedDeltaVector = deltaVector / lineLength;
    const TwoDVector perpendicularVector(-normalizedDeltaVector.y, normalizedDeltaVector.x);

    TwoDVector vertex;
    if (offsetOnLineFromVertex1 == 0.0) {
        vertex = lineVertex1;
    }
    else {
        vertex =
            CalcIntermediatePoint(lineVertex1, lineVertex2, (offsetOnLineFromVertex1 / lineLength));
    }//if//

    const TwoDVector offsetVector = ((newLineLength / 2.0) * perpendicularVector);
    newLineVertex1 = vertex + offsetVector;
    newLineVertex2 = vertex - offsetVector;

}//CalcPerpendicularLine//


inline
TwoDVector CalcPointOnPerpendicularLine(
    const TwoDVector& lineVertex1,
    const TwoDVector& lineVertex2,
    const double& offsetOnLineFromVertex1,
    const double& offsetFromLine)
{
    const TwoDVector deltaVector = lineVertex2 - lineVertex1;
    const double lineLength = deltaVector.Length();
    const TwoDVector normalizedDeltaVector = deltaVector / lineLength;
    const TwoDVector perpendicularVector(normalizedDeltaVector.y, -normalizedDeltaVector.x);

    TwoDVector vertex;
    if (offsetOnLineFromVertex1 == 0.0) {
        vertex = lineVertex1;
    }
    else {
        vertex =
            CalcIntermediatePoint(lineVertex1, lineVertex2, (offsetOnLineFromVertex1 / lineLength));
    }//if//

    return (vertex + (offsetFromLine * perpendicularVector));

}//CalcPointOnPerpendicularLine//



}//namespace//

#endif




