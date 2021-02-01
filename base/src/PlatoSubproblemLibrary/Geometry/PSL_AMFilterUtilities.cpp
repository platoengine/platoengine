#include <PSL_AMFilterUtilities.hpp>
#include <iostream>
#include <limits>
#include <cmath>

namespace PlatoSubproblemLibrary
{

void AMFilterUtilities::computeBoundingBox(Vector& aMaxUVWCoords, Vector& aMinUVWCoords) const
{
    Vector tCoords(mCoordinates[0]);

    double tMaxU = dot_product(tCoords, mUBasisVector);
    double tMaxV = dot_product(tCoords, mVBasisVector);
    double tMaxW = dot_product(tCoords, mBuildDirection);
    double tMinU = dot_product(tCoords, mUBasisVector);
    double tMinV = dot_product(tCoords, mVBasisVector);
    double tMinW = dot_product(tCoords, mBuildDirection);

    aMaxUVWCoords = Vector({tMaxU,tMaxV,tMaxW});
    aMinUVWCoords = Vector({tMinU,tMinV,tMinW});

    for(auto tNodeCoordinates : mCoordinates)
    {
        Vector tNodeVec(tNodeCoordinates);

        if(dot_product(tNodeVec, mUBasisVector) > aMaxUVWCoords(0))
            aMaxUVWCoords.set(0,dot_product(tNodeVec, mUBasisVector));
        if(dot_product(tNodeVec, mVBasisVector) > aMaxUVWCoords(1))
            aMaxUVWCoords.set(1,dot_product(tNodeVec, mVBasisVector));
        if(dot_product(tNodeVec, mBuildDirection) > aMaxUVWCoords(2))
            aMaxUVWCoords.set(2,dot_product(tNodeVec, mBuildDirection));

        if(dot_product(tNodeVec, mUBasisVector) < aMinUVWCoords(0))
            aMinUVWCoords.set(0,dot_product(tNodeVec, mUBasisVector));
        if(dot_product(tNodeVec, mVBasisVector) < aMinUVWCoords(1))
            aMinUVWCoords.set(1,dot_product(tNodeVec, mVBasisVector));
        if(dot_product(tNodeVec, mBuildDirection) < aMinUVWCoords(2))
            aMinUVWCoords.set(2,dot_product(tNodeVec, mBuildDirection));
    }
}

double AMFilterUtilities::computeMinEdgeLength() const
{
    double tMinEdgeLength = std::numeric_limits<double>::max();

    for(auto tElement : mConnectivity)
    {
        Vector tNode0(mCoordinates[tElement[0]]);
        Vector tNode1(mCoordinates[tElement[1]]);
        Vector tNode2(mCoordinates[tElement[2]]);
        Vector tNode3(mCoordinates[tElement[3]]);

        Vector tEdge0 = tNode1 - tNode0;
        Vector tEdge1 = tNode2 - tNode0;
        Vector tEdge2 = tNode3 - tNode0;
        Vector tEdge3 = tNode1 - tNode2;
        Vector tEdge4 = tNode3 - tNode2;
        Vector tEdge5 = tNode3 - tNode1;

        std::vector<Vector> tEdges;

        tEdges.push_back(tEdge0);
        tEdges.push_back(tEdge1);
        tEdges.push_back(tEdge2);
        tEdges.push_back(tEdge3);
        tEdges.push_back(tEdge4);
        tEdges.push_back(tEdge5);

        for(auto tEdge : tEdges)
        {
            if(tEdge.euclideanNorm() < tMinEdgeLength)
                tMinEdgeLength = tEdge.euclideanNorm();
        }
    }

    return tMinEdgeLength;
}

std::vector<int> computeNumElementsInEachDirection(const Vector& aMaxUVWCoords, const Vector& aMinUVWCoords, const double& aTargetEdgeLength)
{
    double tULength = aMaxUVWCoords(0) - aMinUVWCoords(0);
    double tVLength = aMaxUVWCoords(1) - aMinUVWCoords(1);
    double tWLength = aMaxUVWCoords(2) - aMinUVWCoords(2);

    if(aTargetEdgeLength <= 0.0)
        throw(std::domain_error("AMFilterUtilities: target edge length must be greater than zero"));

    if(tULength < aTargetEdgeLength || tVLength < aTargetEdgeLength || tWLength < aTargetEdgeLength)
        throw(std::domain_error("AMFilterUtilities: target edge length is longer than the provided bounding box"));

    std::vector<int> tNumElements;

    tNumElements.push_back((int) tULength/aTargetEdgeLength);
    tNumElements.push_back((int) tVLength/aTargetEdgeLength);
    tNumElements.push_back((int) tWLength/aTargetEdgeLength);

    return tNumElements;
}

Vector computeGridXYZCoordinates(const Vector& aUBasisVector,
                                 const Vector& aVBasisVector,
                                 const Vector& aBuildDirection,
                                 const Vector& aMaxUVWCoords,
                                 const Vector& aMinUVWCoords,
                                 const std::vector<int>& aNumElements,
                                 const std::vector<int>& aIndex)
{
    double tULength = aMaxUVWCoords(0) - aMinUVWCoords(0);
    double tVLength = aMaxUVWCoords(1) - aMinUVWCoords(1);
    double tWLength = aMaxUVWCoords(2) - aMinUVWCoords(2);

    // std::vector<double> tLength;
    // // = {tULength,tVLength,tWLength};
    // tLength.push_back(tULength);
    // tLength.push_back(tVLength);
    // tLength.push_back(tWLength);

    if(tULength < 0 || tVLength < 0 || tWLength < 0)
        throw(std::domain_error("AMFilterUtilities::computeGridXYZCoordinates: Max UVW coordinates expected to be greater than Min UVW coordinates"));

    if(aNumElements[0] <= 0 || aNumElements[1] <= 0 || aNumElements[2] <= 0)
        throw(std::domain_error("AMFilterUtilities::computeGridXYZCoordinates: Number of elements in each direction must be greater than zero"));

    for(int i = 0; i < 3; ++i)
    {
        if(aIndex[i] < 0 || aIndex[i] > aNumElements[i])
            throw(std::out_of_range("AMFilterUtilities::computeGridXYZCoordinates: Index must be between zero and number of elements"));
    }

    Vector tXYZCoordinates({0.0,0.0,0.0});
    Vector tUVWCoordinates({0.0,0.0,0.0});

    std::vector<Vector> tBasis = {aUBasisVector,aVBasisVector,aBuildDirection};
    for(int i = 0; i < 3; ++i)
    {
        // double tUVWCoordinate = aMinUVWCoords(i) + aIndex[i]*tLength[i]/aNumElements[i]; 
        double tUVWCoordinate = aMinUVWCoords(i) + aIndex[i]*10/aNumElements[i]; 
        tUVWCoordinates.set(i,tUVWCoordinate);
    }

    tXYZCoordinates = tUVWCoordinates(0)*aUBasisVector + tUVWCoordinates(1)*aVBasisVector + tUVWCoordinates(2)*aBuildDirection;

    return tXYZCoordinates;
}

void checkTet(const std::vector<int>& aTet, const int& aNumNodes)
{
    if(aTet.size() != 4)
        throw(std::domain_error("AMFilterUtilities::pointInTetrahedron: Expected tetrahedron to contain 4 vertices"));

    auto tTemp = aTet;

    std::sort(tTemp.begin(),tTemp.end());
    auto tLast = std::unique(tTemp.begin(),tTemp.end());
    
    if(tLast != tTemp.end())
        throw(std::domain_error("AMFilterUtilities::pointInTetrahedron: repeated node index"));

    auto tMaxIterator = std::max_element(tTemp.begin(),tTemp.end());
    auto tMinIterator = std::min_element(tTemp.begin(),tTemp.end());

    if(*tMinIterator < 0 || *tMaxIterator >= (int) aNumNodes)
        throw(std::out_of_range("AMFilterUtilities::pointInTetrahedron: node index out of range"));
}

bool AMFilterUtilities::isPointInTetrahedron(const std::vector<int>& aTet, const Vector& aPoint) const
{
    checkTet(aTet, mCoordinates.size());

    return sameSide(aTet[0], aTet[1], aTet[2], aTet[3], aPoint) &&
           sameSide(aTet[1], aTet[2], aTet[3], aTet[0], aPoint) &&
           sameSide(aTet[2], aTet[3], aTet[0], aTet[1], aPoint) &&
           sameSide(aTet[3], aTet[0], aTet[1], aTet[2], aPoint);
}


std::vector<double> AMFilterUtilities::computeBarycentricCoordinates(const std::vector<int>& aTet, const Vector& aPoint) const
{
    checkTet(aTet, mCoordinates.size());

    Vector tR1(mCoordinates[aTet[0]]);
    Vector tR2(mCoordinates[aTet[1]]);
    Vector tR3(mCoordinates[aTet[2]]);
    Vector tR4(mCoordinates[aTet[3]]);

    Vector tRightHandSide = aPoint - tR4;

    // matrix
    Vector tColumn1({tR1(0) - tR4(0), tR1(1) - tR4(1), tR1(2) - tR4(2)});
    Vector tColumn2({tR2(0) - tR4(0), tR2(1) - tR4(1), tR2(2) - tR4(2)});
    Vector tColumn3({tR3(0) - tR4(0), tR3(1) - tR4(1), tR3(2) - tR4(2)});

    // use Cramers rule to solve the system
    double tDeterminant = determinant3X3(tColumn1,tColumn2,tColumn3);

    if(fabs(tDeterminant) < 1e-12)
        throw(std::domain_error("AMFilterUtilities::computeBarycentricCoordinates: Tetrahedron is singular"));

    double tInverseDeterminant = 1.0/tDeterminant;

    double tBary1 = determinant3X3(tRightHandSide,tColumn2,tColumn3)*tInverseDeterminant;
    double tBary2 = determinant3X3(tColumn1,tRightHandSide,tColumn3)*tInverseDeterminant;
    double tBary3 = determinant3X3(tColumn1,tColumn2,tRightHandSide)*tInverseDeterminant;
    double tBary4 = 1 - tBary1 - tBary2 - tBary3;

    std::vector<double> tBarycentricCoordinates({tBary1,tBary2,tBary3,tBary4});

    return tBarycentricCoordinates;
}

bool AMFilterUtilities::sameSide(const int& v1, const int& v2, const int& v3, const int& v4, const Vector& aPoint) const
{
    Vector tVec1(mCoordinates[v1]);
    Vector tVec2(mCoordinates[v2]);
    Vector tVec3(mCoordinates[v3]);
    Vector tVec4(mCoordinates[v4]);

    Vector tNormal = cross_product(tVec2 - tVec1, tVec3 - tVec1);

    double tDot1 = dot_product(tNormal, tVec4 - tVec1);
    double tDot2 = dot_product(tNormal, aPoint - tVec1);

    return tDot1*tDot2 > 0;
}

double determinant3X3(const Vector& aRow1,
                      const Vector& aRow2,
                      const Vector& aRow3)
{
    // rows may actually be columns instead because the
    // determinant of a matrix is equal to the determinant
    // of its transpose
    
    double tTerm1 = aRow1(0)*(aRow2(1)*aRow3(2) - aRow2(2)*aRow3(1));
    double tTerm2 = aRow1(1)*(aRow2(0)*aRow3(2) - aRow2(2)*aRow3(0));
    double tTerm3 = aRow1(2)*(aRow2(0)*aRow3(1) - aRow2(1)*aRow3(0));

    return tTerm1 - tTerm2 + tTerm3;
}

void AMFilterUtilities::checkInput() const
{
    if(mConnectivity.size() == 0 || mCoordinates.size() < 4)
        throw(std::domain_error("AMFilterUtilities expected at least one tetrahedron in mesh"));

    // if(mBaseLayer.size() == 0)
    //     throw(std::domain_error("AMFilterUtilities at least one node must be specified on the base layer"));

    for(auto tCoord: mCoordinates)
    {
        if(tCoord.size() != 3u)
            throw(std::domain_error("AMFilterUtilities expected 3 dimensional coordinates"));
    }

    int tMaxNodeID = 0;
    int tMinNodeID = 0;
    for(auto tElement : mConnectivity)
    {
        for(auto tNodeID : tElement)
        {
            if(tNodeID > tMaxNodeID)
            {
                tMaxNodeID = tNodeID;
            }
            if(tNodeID < tMinNodeID)
            {
                tMinNodeID = tNodeID;
            }
        }

        if(tElement.size() != 4u)
            throw(std::domain_error("AMFilterUtilities expected tetrahedral elements"));
    }

    // for(auto tNodeID : mBaseLayer)
    // {
    //     if(tNodeID > tMaxNodeID)
    //     {
    //         tMaxNodeID = tNodeID;
    //     }
    //     if(tNodeID < tMinNodeID)
    //     {
    //         tMinNodeID = tNodeID;
    //     }
    // }

    if(tMinNodeID < 0 || tMaxNodeID >= (int) mCoordinates.size())
    {
        std::string tError = std::string("Node IDs must be between zero and ") + std::to_string(mCoordinates.size() - 1);
        throw(std::out_of_range(tError));
    }

    // if(mCriticalPrintAngle <= 0 || mCriticalPrintAngle >= M_PI/2)
    //     throw(std::out_of_range("Critical print angle should be between zero and Pi/2"));

    if(fabs(mUBasisVector.euclideanNorm() - 1) > 1e-12 || fabs(mVBasisVector.euclideanNorm() -1) > 1e-12 || fabs(mBuildDirection.euclideanNorm() - 1) > 1e-12)
    {
        throw(std::domain_error("AMFilterUtilities: provided basis not unit length"));
    }

    if(dot_product(mUBasisVector,mVBasisVector) > 1e-12 || dot_product(mUBasisVector,mBuildDirection) > 1e-12 || dot_product(mVBasisVector,mBuildDirection) > 1e-12)
    {
        throw(std::domain_error("AMFilterUtilities: provided basis is not orthogonal"));
    }

    if(dot_product(cross_product(mUBasisVector,mVBasisVector),mBuildDirection) < 0)
    {
        throw(std::domain_error("AMFilterUtilities: provided basis is not positively oriented"));
    }
}

}
