// Copyright (c) 2007-2016 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef TWO_D_TREES_H
#define TWO_D_TREES_H

#include "scensim_support.h"
#include "multiagent2_common.h"

#include <assert.h>
#include <cmath>
#include <memory>
#include <vector>
#include <algorithm>

namespace MultiAgent2 {

using std::unique_ptr;
using std::shared_ptr;
using std::vector;
using ScenSim::RoundUpToUint;

template<typename TwoDVectorType>
class LineSegmentTwoDTree {
public:
    LineSegmentTwoDTree(const double& maxSpacing);

    bool IsEmpty() const { return (topPtr == nullptr); }

    void Insert(const TwoDVectorType& lineVertex1, const TwoDVectorType& lineVertex2);

    void BuildTree();

    // For dynamic objects such as doors that open and close.

    void DisableLineSegment(const TwoDVectorType& lineVertex1, const TwoDVectorType& lineVertex2);
    void EnableLineSegment(const TwoDVectorType& lineVertex1, const TwoDVectorType& lineVertex2);

    //-------------------

    private:
    struct CellType;
    public:

    class RadiusLimitedIterator {
    public:
        explicit RadiusLimitedIterator(const LineSegmentTwoDTree* const initContainerPtr)
            : containerPtr(initContainerPtr), currentIndex(UINT_MAX) { }

        struct LineSegmentType {
            TwoDVectorType v1;
            TwoDVectorType v2;
        };

        const LineSegmentType& operator*() const;
        void operator++();
        bool operator==(const RadiusLimitedIterator& right)
            { return ((*this).currentIndex == right.currentIndex); }

        bool operator!=(const RadiusLimitedIterator& right) { return (!((*this) == right)); }

    private:
        const LineSegmentTwoDTree* const containerPtr;

        TwoDVectorType position;
        double radiusSquared;
        double minX;
        double maxX;
        double minY;
        double maxY;
        std::set<unsigned int> alreadyVisitedSegmentIndices;

        vector<unsigned int> segmentIndices;
        unsigned int currentIndex;

        void AddAllClosePointsInSubtree(const CellType* startCellPtr);

        void SearchForClosePointsInSubtree(
            const CellType* startCellPtr,
            const double& subtreeMinX,
            const double& subtreeMaxX,
            const double& subtreeMinY,
            const double& subtreeMaxY);

        friend class LineSegmentTwoDTree;

    };//RadiusLimitedIterator//

    //-------------------

    RadiusLimitedIterator MakeRadiusLimitedIterator(const TwoDVectorType& position, const double& radius) const;

    RadiusLimitedIterator end() const {
        RadiusLimitedIterator iter(this);
        iter.currentIndex = UINT_MAX;
        return (iter);
    }

    bool LineIntersectsALineInTree(
        const TwoDVectorType& lineVertex1,
        const TwoDVectorType& lineVertex2) const;

private:
    friend class RadiusLimitedIterator;

    bool treeHasBeenBuilt;
    double maxSpacing;

    struct CellType {
        unique_ptr<CellType> leftPtr;
        unique_ptr<CellType> rightPtr;
        CellType* parentPtr;
        TwoDVectorType position;
        unsigned int lineSegmentIndex;
        bool useXForComparison;

        CellType() : parentPtr(nullptr) { }

    };//CellType//


    unique_ptr<CellType> topPtr;

    struct LineSegmentInfoType {
        typename RadiusLimitedIterator::LineSegmentType lineSegment;
        bool isEnabled;

        LineSegmentInfoType() : isEnabled(true) { }

        LineSegmentInfoType(const TwoDVectorType& v1, const TwoDVectorType& v2) : isEnabled(true)
        {
            lineSegment.v1 = v1;
            lineSegment.v2 = v2;
        }
    };

    vector<LineSegmentInfoType> lineSegments;

    struct SortElemType {
        TwoDVectorType position;
        unsigned int lineSegmentIndex;

        SortElemType(
            const TwoDVectorType& initPos,
            const unsigned int initLineSegmentIndex)
        :
            position(initPos),
            lineSegmentIndex(initLineSegmentIndex)
        {}
    };


    class SortByXComparitor {
    public:
        bool operator()(const SortElemType& left, const SortElemType& right) const
        {
            return ((left.position.GetX() < right.position.GetX()) ||
                    ((left.position.GetX() == right.position.GetX()) &&
                     (left.position.GetY() < right.position.GetY())));
        }
    };

    class SortByYComparitor {
    public:
        bool operator()(const SortElemType& left, const SortElemType& right)
        {
            return ((left.position.GetY() < right.position.GetY()) ||
                    ((left.position.GetY() == right.position.GetY()) &&
                     (left.position.GetX() < right.position.GetX())));
        }
    };

    static void BuildSubtree(
        vector<SortElemType>& sortArray,
        const unsigned int first,
        const unsigned int lastPlus1,
        const bool useXForSorting,
        const unique_ptr<CellType>& parentPtr,
        unique_ptr<CellType>& treePtr);

    static const CellType* SearchDownForLeftMostPointBoundingBox(
        const CellType*  startCellPtr,
        const double& minX,
        const double& maxX,
        const double& minY,
        const double& maxY);

    static const CellType* SearchRightForNextPointInBox(
        const CellType* startCellPtr,
        const double& minX,
        const double& maxX,
        const double& minY,
        const double& maxY);

    bool LineIntersectsALineInSubtree(
        const CellType* cellPtr,
        const double& boxMinX,
        const double& boxMaxX,
        const double& boxMinY,
        const double& boxMaxY,
        const TwoDVectorType& lineVertex1,
        const TwoDVectorType& lineVertex2,
        const double& lineLength) const;

};//LineSegmentTwoDTree//



template<typename TwoDVectorType> inline
LineSegmentTwoDTree<TwoDVectorType>::LineSegmentTwoDTree(const double& initMaxSpacing)
    :
    maxSpacing(initMaxSpacing),
    treeHasBeenBuilt(false)
{
}


template<typename TwoDVectorType> inline
void LineSegmentTwoDTree<TwoDVectorType>::Insert(
    const TwoDVectorType& lineVertex1, const TwoDVectorType& lineVertex2)
{
    assert(lineVertex1 != lineVertex2);
    (*this).lineSegments.push_back(LineSegmentInfoType(lineVertex1, lineVertex2));
    (*this).treeHasBeenBuilt = false;
}



template<typename TwoDVectorType> inline
void LineSegmentTwoDTree<TwoDVectorType>::BuildSubtree(
    vector<SortElemType>& sortArray,
    const unsigned int first,
    const unsigned int lastPlus1,
    const bool useXForSorting,
    const unique_ptr<CellType>& parentPtr,
    unique_ptr<CellType>& treePtr)
{
    assert(first < lastPlus1);
    const unsigned int rangeSize = (lastPlus1 - first);
    const unsigned int spliterIndex = first + (rangeSize / 2);

    if (rangeSize > 1) {
        if (useXForSorting) {
            std::sort(
                (sortArray.begin() + first),
                (sortArray.begin() + lastPlus1),
                SortByXComparitor());
        }
        else {
            std::sort(
                (sortArray.begin() + first),
                (sortArray.begin() + lastPlus1),
                SortByYComparitor());
        }//if//
    }//if//

    treePtr.reset(new CellType());
    treePtr->parentPtr = parentPtr.get();
    treePtr->useXForComparison = useXForSorting;
    treePtr->position = sortArray[spliterIndex].position;
    treePtr->lineSegmentIndex = sortArray[spliterIndex].lineSegmentIndex;

    // Classic Tree Recursion.

    if (spliterIndex > first) {

        BuildSubtree(
            sortArray,
            first,
            spliterIndex,
            (!useXForSorting),
            treePtr,
            treePtr->leftPtr);
    }//if//

    if ((spliterIndex + 1) < lastPlus1) {

        BuildSubtree(
            sortArray,
            (spliterIndex + 1),
            lastPlus1,
            (!useXForSorting),
            treePtr,
            treePtr->rightPtr);
    }//if//

}//BuildSubtree//



template<typename TwoDVectorType> inline
void LineSegmentTwoDTree<TwoDVectorType>::BuildTree()
{
    vector<SortElemType> sortArray;

    for(unsigned int segmentIndex = 0; (segmentIndex < lineSegments.size()); segmentIndex++) {

        const LineSegmentInfoType& aSegment = lineSegments[segmentIndex];
        const double segmentLength =
            CalcDistance(
                aSegment.lineSegment.v1,
                aSegment.lineSegment.v2);

        const unsigned int numberPoints = RoundUpToUint(segmentLength / maxSpacing) + 1;
        const double pointSpacing = segmentLength / numberPoints;
        assert(pointSpacing <= maxSpacing);

        for(unsigned int i = 0; (i < numberPoints); i++) {

            const TwoDVectorType aPoint =
                CalcIntermediatePoint(
                    aSegment.lineSegment.v1,
                    aSegment.lineSegment.v2,
                    (static_cast<double>(i) / (numberPoints - 1)));

            sortArray.push_back(SortElemType(aPoint, segmentIndex));

        }//for//
    }//for//

    (*this).topPtr.reset();

    BuildSubtree(
        sortArray,
        0,
        static_cast<unsigned int>(sortArray.size()),
        true,
        unique_ptr<CellType>(/*nullptr*/),
        topPtr);

    (*this).treeHasBeenBuilt = true;

}//BuildTree//



template<typename TwoDVectorType> inline
void LineSegmentTwoDTree<TwoDVectorType>::RadiusLimitedIterator::AddAllClosePointsInSubtree(
    const typename LineSegmentTwoDTree<TwoDVectorType>::CellType* startCellPtr)
{
    // Filter by radius.

    const TwoDVectorType deltaPosition = (position - startCellPtr->position);

    if (deltaPosition.LengthSquared() <= radiusSquared) {
        if (alreadyVisitedSegmentIndices.count(startCellPtr->lineSegmentIndex) == 0) {
            (*this).alreadyVisitedSegmentIndices.insert(startCellPtr->lineSegmentIndex);
            (*this).segmentIndices.push_back(startCellPtr->lineSegmentIndex);
        }//if//
    }//if//

    if (startCellPtr->leftPtr != nullptr) {
        (*this).AddAllClosePointsInSubtree(startCellPtr->leftPtr.get());
    }//if//

    if (startCellPtr->rightPtr != nullptr) {
        (*this).AddAllClosePointsInSubtree(startCellPtr->rightPtr.get());
    }//if//

}//AddAllClosePointsInSubtree//



inline
bool The2DRangesIntersect(
    const double& minX1,
    const double& maxX1,
    const double& minY1,
    const double& maxY1,
    const double& minX2,
    const double& maxX2,
    const double& minY2,
    const double& maxY2)
{
    return ((minX1 <= maxX2) && (maxX1 >= minX2)) && ((minY1 <= maxY2) && (maxY1 >= minY2));
}



inline
bool The2DRangeIsContainedIn(
    const double& minX,
    const double& maxX,
    const double& minY,
    const double& maxY,
    const double& containerMinX,
    const double& containerMaxX,
    const double& containerMinY,
    const double& containerMaxY)

{
    return ((minX >= containerMinX) && (maxX <= containerMaxX) &&
            (minY >= containerMinY) && (maxY <= containerMaxY));
}




template<typename TwoDVectorType> inline
void LineSegmentTwoDTree<TwoDVectorType>::RadiusLimitedIterator::SearchForClosePointsInSubtree(
    const typename LineSegmentTwoDTree<TwoDVectorType>::CellType* startCellPtr,
    const double& subtreeMinX,
    const double& subtreeMaxX,
    const double& subtreeMinY,
    const double& subtreeMaxY)
{
    if ((startCellPtr->leftPtr == nullptr) && (startCellPtr->rightPtr == nullptr)) {
        // Is a leaf.
        // Filter by radius.

        const TwoDVectorType deltaPosition = (position - startCellPtr->position);

        if (deltaPosition.LengthSquared() <= radiusSquared) {
            if (alreadyVisitedSegmentIndices.count(startCellPtr->lineSegmentIndex) == 0) {
                (*this).alreadyVisitedSegmentIndices.insert(startCellPtr->lineSegmentIndex);
                (*this).segmentIndices.push_back(startCellPtr->lineSegmentIndex);
            }//if//
        }//if//

        return;
    }//if//


    if (startCellPtr->useXForComparison) {
        if ((startCellPtr->leftPtr != nullptr) &&
            The2DRangesIntersect(
                subtreeMinX, startCellPtr->position.GetX(), subtreeMinY, subtreeMaxY,
                minX, maxX, minY, maxY)) {

            if (The2DRangeIsContainedIn(
                    subtreeMinX, startCellPtr->position.GetX(), subtreeMinY, subtreeMaxY,
                    minX, maxX, minY, maxY)) {

                (*this).AddAllClosePointsInSubtree(startCellPtr->leftPtr.get());
            }
            else {
                (*this).SearchForClosePointsInSubtree(
                    startCellPtr->leftPtr.get(),
                    subtreeMinX, startCellPtr->position.GetX(), subtreeMinY, subtreeMaxY);
            }//if//
        }//if//

        if ((startCellPtr->rightPtr != nullptr) &&
            (The2DRangesIntersect(
                startCellPtr->position.GetX(), subtreeMaxX, subtreeMinY, subtreeMaxY,
                minX, maxX, minY, maxY))) {

            if (The2DRangeIsContainedIn(
                    startCellPtr->position.GetX(), subtreeMaxX, subtreeMinY, subtreeMaxY,
                    minX, maxX, minY, maxY)) {

                (*this).AddAllClosePointsInSubtree(startCellPtr->rightPtr.get());
            }
            else {
                (*this).SearchForClosePointsInSubtree(
                    startCellPtr->rightPtr.get(),
                    startCellPtr->position.GetX(), subtreeMaxX, subtreeMinY, subtreeMaxY);
            }//if//
        }//if//
    }
    else {
        if ((startCellPtr->leftPtr != nullptr) &&
            The2DRangesIntersect(
                subtreeMinX, subtreeMaxX, subtreeMinY, startCellPtr->position.GetY(),
                minX, maxX, minY, maxY)) {

            if (The2DRangeIsContainedIn(
                    subtreeMinX, subtreeMaxX, subtreeMinY, startCellPtr->position.GetY(),
                    minX, maxX, minY, maxY)) {

                (*this).AddAllClosePointsInSubtree(startCellPtr->leftPtr.get());
            }
            else {
                (*this).SearchForClosePointsInSubtree(
                    startCellPtr->leftPtr.get(),
                    subtreeMinX, subtreeMaxX, subtreeMinY, startCellPtr->position.GetY());
            }//if//
        }//if//

        if ((startCellPtr->rightPtr != nullptr) &&
            (The2DRangesIntersect(
                subtreeMinX, subtreeMaxX, startCellPtr->position.GetY(), subtreeMaxY,
                minX, maxX, minY, maxY))) {

            if (The2DRangeIsContainedIn(
                    subtreeMinX, subtreeMaxX, startCellPtr->position.GetY(), subtreeMaxY,
                    minX, maxX, minY, maxY)) {

                (*this).AddAllClosePointsInSubtree(startCellPtr->rightPtr.get());
            }
            else {
                (*this).SearchForClosePointsInSubtree(
                    startCellPtr->rightPtr.get(),
                    subtreeMinX, subtreeMaxX, startCellPtr->position.GetY(), subtreeMaxY);
            }//if//
        }//if//
    }//if//

}//SearchForClosePointsInSubtree//



template<typename TwoDVectorType> inline
typename LineSegmentTwoDTree<TwoDVectorType>::RadiusLimitedIterator
    LineSegmentTwoDTree<TwoDVectorType>::MakeRadiusLimitedIterator(
        const TwoDVectorType& position,
        const double& radius) const
{
    RadiusLimitedIterator iter(this);

    iter.position = position;
    iter.radiusSquared = radius * radius;

    const double searchBoxWithPadding = sqrt(2.0) * radius;

    iter.minX = (position.GetX() - searchBoxWithPadding);
    iter.maxX = (position.GetX() + searchBoxWithPadding);
    iter.minY = (position.GetY() - searchBoxWithPadding);
    iter.maxY = (position.GetY() + searchBoxWithPadding);

    iter.SearchForClosePointsInSubtree(
        topPtr.get(),
        -DBL_MAX,
        DBL_MAX,
        -DBL_MAX,
        DBL_MAX);

    iter.currentIndex = 0;
    if (iter.segmentIndices.empty()) {
        iter.currentIndex = UINT_MAX;
    }//if//

    return (iter);

}//MakeRadiusLimitedIterator//



template<typename TwoDVectorType> inline
const typename LineSegmentTwoDTree<TwoDVectorType>::RadiusLimitedIterator::LineSegmentType&
LineSegmentTwoDTree<TwoDVectorType>::RadiusLimitedIterator::operator*() const
{
    assert(currentIndex <= segmentIndices.size());

    return (containerPtr->lineSegments.at(segmentIndices[currentIndex]).lineSegment);

}//operator*//


template<typename TwoDVectorType> inline
void LineSegmentTwoDTree<TwoDVectorType>::RadiusLimitedIterator::operator++()
{
    (*this).currentIndex++;
    if (currentIndex >= segmentIndices.size()) {
        (*this).currentIndex = UINT_MAX;
    }//if//

}//operator++//



template<typename TwoDVectorType> inline
bool LineSegmentTwoDTree<TwoDVectorType>::LineIntersectsALineInTree(
    const TwoDVectorType& inputLineVertex1,
    const TwoDVectorType& inputLineVertex2) const
{
    using std::min;
    using std::max;

    assert(treeHasBeenBuilt && (!lineSegments.empty()));

    //Future// // For efficency, LineIntersectsALineInSubtree assumes X axis order of its inputs.
    //Future//
    //Future// TwoDVectorType lineVertex1 = inputLineVertex1;
    //Future// TwoDVectorType lineVertex2 = inputLineVertex2;
    //Future//
    //Future// if (lineVertex1.GetX() > lineVertex2.GetX()) {
    //Future//    std::swap(lineVertex1, lineVertex2);
    //Future// }//if//
    //Future//
    //Future// return (
    //Future//     LineIntersectsALineInSubtree(
    //Future//         topPtr.get(),
    //Future//         (min(lineVertex1.GetX(), lineVertex2.GetX()) - maxSpacing),
    //Future//         (max(lineVertex1.GetX(), lineVertex2.GetX()) + maxSpacing),
    //Future//         (min(lineVertex1.GetY(), lineVertex2.GetY()) - maxSpacing),
    //Future//         (max(lineVertex1.GetY(), lineVertex2.GetY()) + maxSpacing),
    //Future//         lineVertex1,
    //Future//         lineVertex2,
    //Future//         (CalcDistance(lineVertex1, lineVertex2))));


    // Just brute force it.

    for(unsigned int i = 0; (i < lineSegments.size()); i++) {
        const LineSegmentInfoType& segmentInfo = lineSegments[i];

        if (LineSegmentsCross(
            segmentInfo.lineSegment.v1,
            segmentInfo.lineSegment.v2,
            inputLineVertex1,
            inputLineVertex2)) {

            return true;
        }//if//
    }//for//

    return false;

}//LineIntersectsALineInTree//



//Future// template<typename TwoDVectorType> inline
//Future// bool LineSegmentTwoDTree<TwoDVectorType>::LineIntersectsALineInSubtree(
//Future//     const typename CellType* cellPtr,
//Future//     const double& boxMinX,
//Future//     const double& boxMaxX,
//Future//     const double& boxMinY,
//Future//     const double& boxMaxY,
//Future//     const TwoDVectorType& lineVertex1,
//Future//     const TwoDVectorType& lineVertex2,
//Future//     const double& lineLength) const
//Future// {
//Future//     assert(cellPtr != nullptr);
//Future//     assert((lineVertex1.GetX() <= lineVertex2.GetX()) && "line vertices must be ordered (x-axis)");
//Future//
//Future//
//Future//     const LineSegmentInfoType& segmentInfo = lineSegments[cellPtr->lineSegmentIndex];
//Future//
//Future//     if ((segmentInfo.isEnabled) &&
//Future//         LineSegmentsCross(
//Future//             segmentInfo.lineSegment.v1,
//Future//             segmentInfo.lineSegment.v2,
//Future//             lineVertex1,
//Future//             lineVertex2)) {
//Future//
//Future//         return true;
//Future//     }//if//
//Future//
//Future//     bool searchLeftTree = (cellPtr->leftPtr != nullptr);
//Future//     bool searchRightTree = (cellPtr->rightPtr != nullptr);
//Future//
//Future//     if ((!searchLeftTree) && (!searchRightTree)) {
//Future//         return false;
//Future//     }//if//
//Future//
//Future//
//Future//     if (cellPtr->useXForComparison) {
//Future//
//Future//         // Adjust segment end points (xMin, xMax) on line to be within Y-axis box bounds
//Future//         // (boxMinY, boxMinX)
//Future//
//Future//         double xMin = lineVertex1.GetX();
//Future//         double xMax = lineVertex2.GetX();
//Future//
//Future//         if (lineVertex1.GetY() < lineVertex2.GetY()) {
//Future//             if (lineVertex1.GetY() < boxMinY) {
//Future//                 const double fractionOfTotalLength = (boxMinY - lineVertex1.GetY()) / lineLength;
//Future//
//Future//                 xMin =
//Future//                     lineVertex1.GetX() +
//Future//                     ((lineVertex2.GetX() - lineVertex1.GetX()) * fractionOfTotalLength);
//Future//             }//if//
//Future//
//Future//             if (lineVertex2.GetY() > boxMaxY) {
//Future//                 const double fractionOfTotalLength = (lineVertex2.GetY() - boxMaxY) / lineLength;
//Future//
//Future//                 xMax =
//Future//                     lineVertex2.GetX() -
//Future//                     ((lineVertex2.GetX() - lineVertex1.GetX()) * fractionOfTotalLength);
//Future//             }//if//
//Future//         }
//Future//         else {
//Future//
//Future//             if (lineVertex2.GetY() < boxMinY) {
//Future//                 const double fractionOfTotalLength = (boxMinY - lineVertex2.GetY()) / lineLength;
//Future//
//Future//                 xMin =
//Future//                     lineVertex2.GetX() -
//Future//                     ((lineVertex2.GetX() - lineVertex1.GetX()) * fractionOfTotalLength);
//Future//             }//if//
//Future//
//Future//             if (lineVertex1.GetY() > boxMaxY) {
//Future//                 const double fractionOfTotalLength = (lineVertex1.GetY() - boxMaxY) / lineLength;
//Future//
//Future//                 xMax =
//Future//                     lineVertex1.GetX() +
//Future//                     ((lineVertex2.GetX() - lineVertex1.GetX()) * fractionOfTotalLength);
//Future//             }//if//
//Future//         }//if//
//Future//
//Future//         // Try to prune subtree search
//Future//
//Future//
//Future//         if ((searchLeftTree) && (cellPtr->position.GetX() < (xMin - maxSpacing))) {
//Future//             searchLeftTree = false;
//Future//         }//if//
//Future//
//Future//         if ((searchRightTree) && (xMax + maxSpacing) < cellPtr->position.GetX()) {
//Future//             searchRightTree = false;
//Future//         }//if//
//Future//
//Future//
//Future//         if (searchLeftTree) {
//Future//             if (LineIntersectsALineInSubtree(
//Future//                     cellPtr->leftPtr.get(),
//Future//                     boxMinX,
//Future//                     cellPtr->position.GetX(),
//Future//                     boxMinY,
//Future//                     boxMaxY,
//Future//                     lineVertex1,
//Future//                     lineVertex2,
//Future//                     lineLength)) {
//Future//
//Future//                 return true;
//Future//             }//if//
//Future//         }//if//
//Future//
//Future//         if (searchRightTree) {
//Future//             return (
//Future//                 LineIntersectsALineInSubtree(
//Future//                     cellPtr->leftPtr.get(),
//Future//                     cellPtr->position.GetX(),
//Future//                     boxMaxX,
//Future//                     boxMinY,
//Future//                     boxMaxY,
//Future//                     lineVertex1,
//Future//                     lineVertex2,
//Future//                     lineLength));
//Future//         }//if//
//Future//     }
//Future//     else {
//Future//         double yMin = lineVertex1.GetY();
//Future//         double yMax = lineVertex2.GetY();
//Future//
//Future//         if (lineVertex1.GetY() < lineVertex2.GetY()) {
//Future//
//Future//             if (lineVertex1.GetX() < boxMinX) {
//Future//                 const double fractionOfTotalLength = (boxMinX - lineVertex1.GetX()) / lineLength;
//Future//
//Future//                 yMin =
//Future//                     lineVertex1.GetY() +
//Future//                     ((lineVertex2.GetY() - lineVertex1.GetY()) * fractionOfTotalLength);
//Future//             }//if//
//Future//
//Future//             if (lineVertex2.GetX() > boxMaxX) {
//Future//                 const double fractionOfTotalLength = (lineVertex2.GetX() - boxMaxX) / lineLength;
//Future//
//Future//                 yMax =
//Future//                     lineVertex2.GetY() -
//Future//                     ((lineVertex2.GetY() - lineVertex1.GetY()) * fractionOfTotalLength);
//Future//             }//if//
//Future//         }
//Future//         else {
//Future//             if (lineVertex2.GetX() < boxMinX) {
//Future//                 const double fractionOfTotalLength = (boxMinX - lineVertex2.GetX()) / lineLength;
//Future//
//Future//                 yMin =
//Future//                     lineVertex2.GetY() -
//Future//                     ((lineVertex2.GetY() - lineVertex1.GetY()) * fractionOfTotalLength);
//Future//             }//if//
//Future//
//Future//             if (lineVertex1.GetX() > boxMaxX) {
//Future//                 const double fractionOfTotalLength = (lineVertex1.GetX() - boxMaxX) / lineLength;
//Future//
//Future//                 yMax =
//Future//                     lineVertex1.GetY() +
//Future//                     ((lineVertex2.GetY() - lineVertex1.GetY()) * fractionOfTotalLength);
//Future//             }//if//
//Future//         }//if//
//Future//
//Future//         // Try to prune subtree search
//Future//
//Future//         if ((searchLeftTree) && (cellPtr->position.GetY() < (yMin - maxSpacing))) {
//Future//             searchLeftTree = false;
//Future//         }//if//
//Future//
//Future//         if ((searchRightTree) && (yMax + maxSpacing) < cellPtr->position.GetY()) {
//Future//             searchRightTree = false;
//Future//         }//if//
//Future//
//Future//
//Future//         if (searchLeftTree) {
//Future//             if (LineIntersectsALineInSubtree(
//Future//                     cellPtr->leftPtr.get(),
//Future//                     boxMinX,
//Future//                     boxMaxX,
//Future//                     boxMinY,
//Future//                     cellPtr->position.GetY(),
//Future//                     lineVertex1,
//Future//                     lineVertex2,
//Future//                     lineLength)) {
//Future//
//Future//                 return true;
//Future//             }//if//
//Future//         }//if//
//Future//
//Future//         if (searchRightTree) {
//Future//             return (
//Future//                 LineIntersectsALineInSubtree(
//Future//                     cellPtr->leftPtr.get(),
//Future//                     boxMinX,
//Future//                     boxMaxX,
//Future//                     cellPtr->position.GetY(),
//Future//                     boxMaxY,
//Future//                     lineVertex1,
//Future//                     lineVertex2,
//Future//                     lineLength));
//Future//         }//if//
//Future//     }//if//
//Future//
//Future//     return false;
//Future//
//Future// }//LineIntersectsALineInSubtree//


//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------


template<typename T>
class MovingPointQuadtree {
public:
    MovingPointQuadtree(
        const double& initMinSplitXOrYDistance,
        const unsigned int initSplitThresholdSize,
        const unsigned int initCombineThresholdSize)
        :
        minSplitXOrYDistance(initMinSplitXOrYDistance),
        splitThresholdSize(initSplitThresholdSize),
        combineThresholdSize(initCombineThresholdSize)
    {}

    void SetBoundingBox(
        const double& minX,
        const double& maxX,
        const double& minY,
        const double& maxY);

    void Insert(const shared_ptr<T>& elementPtr);
    void Delete(const shared_ptr<T>& elementPtr);

    void UpdateForMovement();
    void CleanUpLeaves();

    private:
    struct TreeNodeType;
    public:

    class Iterator {
    public:
        Iterator() : currentTreeNodePtr(nullptr), currentTreeNodePointIndex(0) { }
        const T& operator*() const;
        void operator++();

        bool operator==(const Iterator& right) const
        {
            return ((currentTreeNodePtr == right.currentTreeNodePtr) &&
                    (currentTreeNodePointIndex == right.currentTreeNodePointIndex));
        }

        bool operator!=(const Iterator& right) const
            { return (!((*this) == right)); }

    private:
        TreeNodeType* currentTreeNodePtr;
        unsigned int currentTreeNodePointIndex;

    };//Iterator//

    class BoxLimitedIterator {
    public:
        BoxLimitedIterator() : currentTreeNodePtr(nullptr), currentTreeNodePointIndex(0) { }
        const T& operator*() const;
        void operator++();

        bool operator==(const BoxLimitedIterator& right) const
        {
            return ((currentTreeNodePtr == right.currentTreeNodePtr) &&
                    (currentTreeNodePointIndex == right.currentTreeNodePointIndex));
        }

        bool operator!=(const BoxLimitedIterator& right) const
            { return (!((*this) == right)); }

    private:
        double minX;
        double maxX;
        double minY;
        double maxY;

        const TreeNodeType* currentTreeNodePtr;
        unsigned int currentTreeNodePointIndex;

        friend class MovingPointQuadtree;

    };//BoxLimitedIterator//


    BoxLimitedIterator end() const { return (BoxLimitedIterator()); }

    // Iterates points within radius.

    BoxLimitedIterator MakeBoxLimitedIterator(
        const double& minX, const double& maxX,
        const double& minY, const double& maxY) const;

    // Optimization (Update doesn't process stationary nodes)

    void SetPointAsNotMoving(const shared_ptr<T>& elementPtr);
    void SetPointAsMoving(const shared_ptr<T>& elementPtr);

private:

    double minSplitXOrYDistance;

    unsigned int splitThresholdSize;
    unsigned int combineThresholdSize;

    struct NonLeafInfoType;

    struct TreeNodeType {
        double minX;
        double maxX;
        double minY;
        double maxY;

        TreeNodeType* parentPtr;

        vector<shared_ptr<T> > points;

        unique_ptr<NonLeafInfoType> nonLeafInfoPtr;

        TreeNodeType(
            const double& initMinX,
            const double& initMaxX,
            const double& initMinY,
            const double& initMaxY,
            TreeNodeType* const initParentPtr)
        :
            minX(initMinX), maxX(initMaxX), minY(initMinY), maxY(initMaxY),
            parentPtr(initParentPtr)
        {}

    };//TreeNodeType//


    struct NonLeafInfoType {
        unique_ptr<TreeNodeType> lowerLeftPtr;
        unique_ptr<TreeNodeType> lowerRightPtr;
        unique_ptr<TreeNodeType> upperLeftPtr;
        unique_ptr<TreeNodeType> upperRightPtr;
        unsigned int numberPointsInLeaves;

        NonLeafInfoType() : numberPointsInLeaves(0) { }
    };

    unique_ptr<TreeNodeType> topPtr;

    //------------------------------------------------------

    void InsertInSubtree(
        TreeNodeType* const startNodePtr,
        const shared_ptr<T>& pointPtr) const;

    void SplitNode(TreeNodeType* const splitNodePtr) const;

    void RepositionPoint(
        TreeNodeType* const pointsNodePtr,
        const unsigned int pointIndex);

    static void AppendPointsAndDeleteLeaves(
        TreeNodeType* const subTreePtr,
        vector<shared_ptr<T> >& points);

    void CleanUpLeavesInSubtree(TreeNodeType* const subTreePtr);

    static void DeleteMovedElementsInSubtree(
        TreeNodeType* const subTreePtr,
        vector<shared_ptr<T> >& movedElements);

    static void SearchDownForLeftMostPointInBox(
        const TreeNodeType* const startNodePtr,
        const double& minX, const double& maxX,
        const double& minY, const double& maxY,
        const TreeNodeType*& foundNodePtr,
        unsigned int& foundPointIndex);

};//MovingPointQuadtree//


template<typename T> inline
void MovingPointQuadtree<T>::SetBoundingBox(
    const double& minX,
    const double& maxX,
    const double& minY,
    const double& maxY)
{
    assert((topPtr == nullptr) && ("SetBoundingBox can only be called once"));

    (*this).topPtr.reset(new TreeNodeType(minX, maxX, minY, maxY, nullptr));

}//SetBoundingBox//




template<typename T> inline
void MovingPointQuadtree<T>::SplitNode(TreeNodeType* const splitNodePtr) const
{
    assert(splitNodePtr != nullptr);
    assert(splitNodePtr->nonLeafInfoPtr == nullptr);

    // Turn node into Non-leaf.

    splitNodePtr->nonLeafInfoPtr.reset(new NonLeafInfoType());
    NonLeafInfoType& nonLeafInfo = *splitNodePtr->nonLeafInfoPtr;

    vector<shared_ptr<T> > points = move(splitNodePtr->points);
    splitNodePtr->points.clear();

    for(unsigned int i = 0; (i < points.size()); i++) {
        (*this).InsertInSubtree(splitNodePtr, points[i]);
    }//for//

}//SplitNode//



template<typename T> inline
void MovingPointQuadtree<T>::InsertInSubtree(
    TreeNodeType* const startNodePtr,
    const shared_ptr<T>& pointPtr) const
{
    using std::min;

    assert(startNodePtr != nullptr);
    assert(startNodePtr->minX <= pointPtr->GetXPos());
    assert(startNodePtr->maxX >= pointPtr->GetXPos());
    assert(startNodePtr->minY <= pointPtr->GetYPos());
    assert(startNodePtr->maxY >= pointPtr->GetYPos());

    TreeNodeType* currentNodePtr = startNodePtr;

    while (true) {
        if (currentNodePtr->nonLeafInfoPtr == nullptr) {
            // Is a leaf.

            if (currentNodePtr->points.size() < splitThresholdSize) {
                currentNodePtr->points.push_back(pointPtr);
                break;
            }
            else {
                // Split Node but stop tree from getting too deep from very close
                // distances.

                const double minXOrYDistance =
                    min((currentNodePtr->maxX - currentNodePtr->minX),
                        (currentNodePtr->maxY - currentNodePtr->minY));

                currentNodePtr->points.push_back(pointPtr);

                if (minXOrYDistance >= minSplitXOrYDistance) {
                    SplitNode(currentNodePtr);
                }//if//
            }//if//

            break;
        }
        else {
            NonLeafInfoType& nonLeafInfo = *currentNodePtr->nonLeafInfoPtr;

            const double xPos = pointPtr->GetXPos();
            const double yPos = pointPtr->GetYPos();
            const double centerX = (currentNodePtr->minX + currentNodePtr->maxX)/2.0;
            const double centerY = (currentNodePtr->minY + currentNodePtr->maxY)/2.0;

            nonLeafInfo.numberPointsInLeaves++;

            if (xPos <= centerX) {
                if (yPos <= centerY) {
                    if (nonLeafInfo.lowerLeftPtr.get() == nullptr) {
                        nonLeafInfo.lowerLeftPtr.reset(
                            new TreeNodeType(
                                currentNodePtr->minX, centerX,
                                currentNodePtr->minY, centerY,
                                currentNodePtr));
                    }//if//

                    currentNodePtr = nonLeafInfo.lowerLeftPtr.get();
                }
                else {
                    if (nonLeafInfo.upperLeftPtr.get() == nullptr) {
                        nonLeafInfo.upperLeftPtr.reset(
                            new TreeNodeType(
                                currentNodePtr->minX, centerX,
                                centerY, currentNodePtr->maxY,
                                currentNodePtr));
                    }//if//

                    currentNodePtr = nonLeafInfo.upperLeftPtr.get();
                }//if//
            }
            else {
                if (yPos <= centerY) {
                    if (nonLeafInfo.lowerRightPtr.get() == nullptr) {
                        nonLeafInfo.lowerRightPtr.reset(
                            new TreeNodeType(
                                centerX, currentNodePtr->maxX,
                                currentNodePtr->minY, centerY,
                                currentNodePtr));
                    }//if//

                    currentNodePtr = nonLeafInfo.lowerRightPtr.get();
                }
                else {
                    if (nonLeafInfo.upperRightPtr.get() == nullptr) {
                        nonLeafInfo.upperRightPtr.reset(
                            new TreeNodeType(
                                centerX, currentNodePtr->maxX,
                                centerY, currentNodePtr->maxY,
                                currentNodePtr));
                    }//if//

                    currentNodePtr = nonLeafInfo.upperRightPtr.get();

                }//if//
            }//if//
        }//if//
    }//while//

}//InsertInSubtree//



template<typename T> inline
void MovingPointQuadtree<T>::RepositionPoint(
    TreeNodeType* const pointsNodePtr,
    const unsigned int pointIndex)
{
    TreeNodeType* currentNodePtr = pointsNodePtr;

    assert(currentNodePtr->nonLeafInfoPtr == nullptr);

    const T& point = *currentNodePtr->points.at(pointIndex);
    const double xPos = point.GetXPos();
    const double yPos = point.GetYPos();

    if ((xPos >= currentNodePtr->minX) && (xPos <= currentNodePtr->minX) &&
        (yPos >= currentNodePtr->minY) && (yPos <= currentNodePtr->minY)) {

        // Still in the right place.

        return;
    }//if//

    shared_ptr<T> pointPtr = currentNodePtr->points.at(pointIndex);
    currentNodePtr->points.erase(currentNodePtr->points.begin() + pointIndex);

    while(true) {
        const TreeNodeType* const lastNodePtr = currentNodePtr;
        currentNodePtr = currentNodePtr->parentPtr;
        assert(currentNodePtr != nullptr);

        currentNodePtr->nonLeafInfoPtr->numberPointsInLeaves--;

        if ((xPos >= currentNodePtr->minX) && (xPos <= currentNodePtr->minX) &&
            (yPos >= currentNodePtr->minY) && (yPos <= currentNodePtr->minY)) {

            break;
        }//if//
    }//for//

    InsertInSubtree(currentNodePtr, pointPtr);

}//RepositionPoint//


template<typename T> inline
void MovingPointQuadtree<T>::Insert(const shared_ptr<T>& pointPtr)
{
    assert(topPtr != nullptr);

    (*this).InsertInSubtree(topPtr.get(), pointPtr);
}



template<typename T> inline
void MovingPointQuadtree<T>::Delete(const shared_ptr<T>& pointPtr)
{
    typedef typename vector<shared_ptr<T> >::iterator IterType;

    const double xPos = pointPtr->GetXPos();
    const double yPos = pointPtr->GetYPos();

    TreeNodeType* currentNodePtr = topPtr;

    while (true) {
        if (currentNodePtr->nonLeafInfoPtr == nullptr) {

            IterType iter =
                std::find(
                    currentNodePtr->points.begin(),
                    currentNodePtr->points.end(),
                    pointPtr);

            assert(iter != currentNodePtr->points.end());

            currentNodePtr->points.erase(currentNodePtr->points.begin() + iter);

            break;
        }
        else {

           const double centerX = (currentNodePtr->minX + currentNodePtr->maxX)/2.0;
           const double centerY = (currentNodePtr->minY + currentNodePtr->minY)/2.0;

           currentNodePtr->nonLeafInfoPtr->numberPointsInLeaves--;

           if (xPos <= centerX) {
               if (yPos <= centerY) {
                   currentNodePtr = currentNodePtr->lowerLeftPtr.get();
               }
               else {
                   currentNodePtr = currentNodePtr->upperLeftPtr.get();
               }//if//
           }
           else {
               if (yPos <= centerY) {
                   currentNodePtr = currentNodePtr->lowerRightPtr.get();
               }
               else {
                   currentNodePtr = currentNodePtr->upperRightPtr.get();

               }//if//
           }//if//
        }//if//
    }//while//

}//Delete//



template<typename T> inline
void MovingPointQuadtree<T>::AppendPointsAndDeleteLeaves(
    TreeNodeType* const subTreePtr,
    vector<shared_ptr<T> >& points)
{
    if (subTreePtr == nullptr) {
        return;
    }//if//

    if (subTreePtr->nonLeafInfoPtr == nullptr) {
        for(unsigned int i = 0; (i < subTreePtr->points.size()); i++) {
            points.push_back(subTreePtr->points[i]);
        }//for//

        subTreePtr->points.clear();
        return;
    }//if//

    // Classic tree recursion.

    const NonLeafInfoType& nonLeafInfo = *subTreePtr->nonLeafInfoPtr;

    AppendPointsAndDeleteLeaves(nonLeafInfo.lowerLeftPtr.get(), points);
    AppendPointsAndDeleteLeaves(nonLeafInfo.lowerRightPtr.get(), points);
    AppendPointsAndDeleteLeaves(nonLeafInfo.upperLeftPtr.get(), points);
    AppendPointsAndDeleteLeaves(nonLeafInfo.upperRightPtr.get(), points);

    subTreePtr->nonLeafInfoPtr.reset();

}//AppendPointsAndDeleteLeaves//



template<typename T> inline
void MovingPointQuadtree<T>::CleanUpLeavesInSubtree(TreeNodeType* const subTreePtr)
{
    if ((subTreePtr == nullptr) || (subTreePtr->nonLeafInfoPtr == nullptr)) {
        return;
    }//if//

    NonLeafInfoType& nonLeafInfo = *subTreePtr->nonLeafInfoPtr;

    if (subTreePtr->nonLeafInfoPtr->numberPointsInLeaves > combineThresholdSize) {

        //  Search each Subtree for small leaves. Classic tree recursion.

        CleanUpLeavesInSubtree(nonLeafInfo.lowerLeftPtr.get());
        CleanUpLeavesInSubtree(nonLeafInfo.lowerRightPtr.get());
        CleanUpLeavesInSubtree(nonLeafInfo.upperLeftPtr.get());
        CleanUpLeavesInSubtree(nonLeafInfo.upperRightPtr.get());
    }
    else {

        assert(subTreePtr->points.empty());

        AppendPointsAndDeleteLeaves(nonLeafInfo.lowerLeftPtr.get(), subTreePtr->points);
        AppendPointsAndDeleteLeaves(nonLeafInfo.lowerRightPtr.get(), subTreePtr->points);
        AppendPointsAndDeleteLeaves(nonLeafInfo.upperLeftPtr.get(), subTreePtr->points);
        AppendPointsAndDeleteLeaves(nonLeafInfo.upperRightPtr.get(), subTreePtr->points);

        subTreePtr->nonLeafInfoPtr.reset();

    }//if//

}//CleanUpLeaves//



template<typename T> inline
void MovingPointQuadtree<T>::CleanUpLeaves()
{
    CleanUpLeavesInSubtree(topPtr.get());

}//CleanUpLeaves//



template<typename T> inline
void MovingPointQuadtree<T>::DeleteMovedElementsInSubtree(
    TreeNodeType* const subTreePtr,
    vector<shared_ptr<T> >& movedElements)
{
    if (subTreePtr == nullptr) {
        return;
    }//if//

    if (subTreePtr->nonLeafInfoPtr == nullptr) {

        unsigned int i = 0;

        while (i < subTreePtr->points.size()) {

            const T& aPoint = *subTreePtr->points[i];
            const double x = aPoint.GetXPos();
            const double y = aPoint.GetYPos();

            if ((x < subTreePtr->minX) || (x > subTreePtr->maxX) ||
                (y < subTreePtr->minY) || (y > subTreePtr->maxY)) {

                movedElements.push_back(subTreePtr->points[i]);
                subTreePtr->points.erase(subTreePtr->points.begin() + i);
            }
            else {
                i++;
            }//if//
        }//for//

        return;

    }//if//

    // Classic Tree recursion.

    NonLeafInfoType& nonLeafInfo = *subTreePtr->nonLeafInfoPtr;

    DeleteMovedElementsInSubtree(nonLeafInfo.lowerLeftPtr.get(), movedElements);
    DeleteMovedElementsInSubtree(nonLeafInfo.lowerRightPtr.get(), movedElements);
    DeleteMovedElementsInSubtree(nonLeafInfo.upperLeftPtr.get(), movedElements);
    DeleteMovedElementsInSubtree(nonLeafInfo.upperRightPtr.get(), movedElements);

}//DeleteMovedElementsInSubtree//



template<typename T> inline
void MovingPointQuadtree<T>::UpdateForMovement()
{
    vector<shared_ptr<T> > movedElements;
    (*this).DeleteMovedElementsInSubtree(topPtr.get(), movedElements);

    for(unsigned int i = 0; (i < movedElements.size()); i++) {
        (*this).Insert(movedElements[i]);
    }//for//

    (*this).CleanUpLeaves();

}//UpdateForMovement//



inline
bool TheseTwoRangeBoxesOverlap(
    const double& minX1,
    const double& maxX1,
    const double& minY1,
    const double& maxY1,
    const double& minX2,
    const double& maxX2,
    const double& minY2,
    const double& maxY2)
{
    return ((maxX1 >= minX2) && (maxX2 >= minX1) && (maxY1 >= minY2) && (maxY2 >= minY1));
}



template<typename T> inline
void MovingPointQuadtree<T>::SearchDownForLeftMostPointInBox(
    const TreeNodeType* const startNodePtr,
    const double& minX, const double& maxX,
    const double& minY, const double& maxY,
    const TreeNodeType*& foundNodePtr,
    unsigned int& foundPointIndex)
{
    foundNodePtr = nullptr;
    foundPointIndex = 0;

    assert(startNodePtr != nullptr);

    if (!TheseTwoRangeBoxesOverlap(
            minX, maxX, minY, maxY,
            startNodePtr->minX, startNodePtr->maxX, startNodePtr->minY, startNodePtr->maxY)) {

        return;
    }//if//

    if (startNodePtr->nonLeafInfoPtr == nullptr) {
        for (unsigned int i = 0; (i < startNodePtr->points.size()); i++) {
            const T& point = *startNodePtr->points[i];

            if ((minX <= point.GetXPos()) && (point.GetXPos() <= maxX) &&
                (minY <= point.GetYPos()) && (point.GetYPos() <= maxY)) {

                foundNodePtr = startNodePtr;
                foundPointIndex = i;
                break;
            }//if//
        }//for//

        return;

    }//if//

    const NonLeafInfoType& nonLeafInfo = *startNodePtr->nonLeafInfoPtr;

    // Classic Tree recursion:

    if (nonLeafInfo.lowerLeftPtr != nullptr) {
        SearchDownForLeftMostPointInBox(
            nonLeafInfo.lowerLeftPtr.get(),
            minX, maxX, minY, maxY,
            foundNodePtr,
            foundPointIndex);

        if (foundNodePtr != nullptr) {
            return;
        }//if//
    }//if//

    if (nonLeafInfo.upperLeftPtr != nullptr) {
        SearchDownForLeftMostPointInBox(
            nonLeafInfo.upperLeftPtr.get(),
            minX, maxX, minY, maxY,
            foundNodePtr,
            foundPointIndex);

        if (foundNodePtr != nullptr) {
            return;
        }//if//
    }//if//

    if (nonLeafInfo.lowerRightPtr != nullptr) {
        SearchDownForLeftMostPointInBox(
            nonLeafInfo.lowerRightPtr.get(),
            minX, maxX, minY, maxY,
            foundNodePtr,
            foundPointIndex);

        if (foundNodePtr != nullptr) {
            return;
        }//if//
    }//if//

    if (nonLeafInfo.upperRightPtr != nullptr) {
        SearchDownForLeftMostPointInBox(
            nonLeafInfo.upperRightPtr.get(),
            minX, maxX, minY, maxY,
            foundNodePtr,
            foundPointIndex);
    }//if//

}//FindLeftLowestLeafNodeInBox//


template<typename T> inline
typename MovingPointQuadtree<T>::BoxLimitedIterator
    MovingPointQuadtree<T>::MakeBoxLimitedIterator(
        const double& minX, const double& maxX,
        const double& minY, const double& maxY) const
{
    BoxLimitedIterator iter;
    iter.minX = minX;
    iter.maxX = maxX;
    iter.minY = minY;
    iter.maxY = maxY;

    SearchDownForLeftMostPointInBox(
        topPtr.get(),
        minX, maxX, minY, maxY,
        /*out*/iter.currentTreeNodePtr,
        /*out*/iter.currentTreeNodePointIndex);

    return (iter);

}//MakeBoxLimitedIterator//



template<typename T> inline
const T& MovingPointQuadtree<T>::BoxLimitedIterator::operator*() const
{
    assert(currentTreeNodePtr != nullptr);
    assert(currentTreeNodePointIndex < currentTreeNodePtr->points.size());

    return (*(currentTreeNodePtr->points[currentTreeNodePointIndex]));
}


template<typename T> inline
void MovingPointQuadtree<T>::BoxLimitedIterator::operator++()
{
    assert(currentTreeNodePtr != nullptr);

    if ((currentTreeNodePointIndex + 1) < currentTreeNodePtr->points.size()) {
        currentTreeNodePointIndex++;
        return;
    }//if//

    currentTreeNodePointIndex = 0;

    const TreeNodeType* lastNodePtr = currentTreeNodePtr;

    currentTreeNodePtr = currentTreeNodePtr->parentPtr;
    if (currentTreeNodePtr == nullptr) {

        // Iteration is complete.

        return;
    }//if///

    assert(currentTreeNodePtr->nonLeafInfoPtr != nullptr);

    while (currentTreeNodePtr != nullptr) {

        const NonLeafInfoType& nonLeafInfo = *currentTreeNodePtr->nonLeafInfoPtr;

        bool keepSearching = false;

        if (lastNodePtr == nonLeafInfo.lowerLeftPtr.get()) {
            if (nonLeafInfo.upperLeftPtr != nullptr) {
                const TreeNodeType* nextTreeNodePtr = nullptr;

                SearchDownForLeftMostPointInBox(
                    nonLeafInfo.upperLeftPtr.get(),
                    minX, maxX, minY, maxY,
                    /*out*/nextTreeNodePtr,
                    /*out*/currentTreeNodePointIndex);

                if (nextTreeNodePtr != nullptr) {
                    currentTreeNodePtr = nextTreeNodePtr;
                    break;
                }//if//
            }//if//

            keepSearching = true;

        }//if//

        if (keepSearching || (lastNodePtr == nonLeafInfo.upperLeftPtr.get())) {
            if (nonLeafInfo.lowerRightPtr != nullptr) {
                const TreeNodeType* nextTreeNodePtr = nullptr;

                SearchDownForLeftMostPointInBox(
                    nonLeafInfo.lowerRightPtr.get(),
                    minX, maxX, minY, maxY,
                    /*out*/nextTreeNodePtr,
                    /*out*/currentTreeNodePointIndex);

                if (nextTreeNodePtr != nullptr) {
                    currentTreeNodePtr = nextTreeNodePtr;
                    break;
                }//if//
            }//if//

            keepSearching = true;

        }//if//

        if ((keepSearching || (lastNodePtr == nonLeafInfo.lowerRightPtr.get())) &&
            (nonLeafInfo.upperRightPtr != nullptr)) {

            const TreeNodeType* nextTreeNodePtr = nullptr;

            SearchDownForLeftMostPointInBox(
                nonLeafInfo.upperRightPtr.get(),
                minX, maxX, minY, maxY,
                /*out*/nextTreeNodePtr,
                /*out*/currentTreeNodePointIndex);

            if (nextTreeNodePtr != nullptr) {
                currentTreeNodePtr = nextTreeNodePtr;
                break;
            }//if//
        }//if//

        lastNodePtr = currentTreeNodePtr;
        currentTreeNodePtr = currentTreeNodePtr->parentPtr;

    }//while//

}//operator++//


}//namespace//

#endif
