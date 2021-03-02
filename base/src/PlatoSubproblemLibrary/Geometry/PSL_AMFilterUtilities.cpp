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

int getSerializedIndex(const std::vector<int>& aNumElementsInEachDirection, const int& i, const int& j, const int& k)
{
    if(aNumElementsInEachDirection.size() != 3)
    {
        throw(std::domain_error("AMFilterUtilities: Grid must be 3 dimensional"));
    }

    if(aNumElementsInEachDirection[0] <= 0 || aNumElementsInEachDirection[1] <= 0 || aNumElementsInEachDirection[2] <= 0)
    {
        throw(std::domain_error("AMFilterUtilities: Number of elements in each direction must be greater than zero"));
    }

    if(i < 0 || j < 0 || k < 0 || i > aNumElementsInEachDirection[0] || j > aNumElementsInEachDirection[1] || k > aNumElementsInEachDirection[2])
    {
        throw(std::out_of_range("AMFilterUtilities: Index in each direction must be between zero and number of grid points"));
    }

    return i + j*(aNumElementsInEachDirection[0]+1) + k*(aNumElementsInEachDirection[0]+1)*(aNumElementsInEachDirection[1]+1);
}

int getSerializedIndex(const std::vector<int>& aNumElementsInEachDirection, const std::vector<int>& aIndex)
{
    if(aIndex.size() != 3)
    {
        throw(std::domain_error("AMFilterUtilities: Index must have 3 entries"));
    }

    return getSerializedIndex(aNumElementsInEachDirection, aIndex[0], aIndex[1], aIndex[2]);
}

void computeGridXYZCoordinates(const Vector& aUBasisVector,
                                 const Vector& aVBasisVector,
                                 const Vector& aBuildDirection,
                                 const Vector& aMaxUVWCoords,
                                 const Vector& aMinUVWCoords,
                                 const std::vector<int>& aNumElements,
                                 std::vector<Vector>& aXYZCoordinates)
{
    double tULength = aMaxUVWCoords(0) - aMinUVWCoords(0);
    double tVLength = aMaxUVWCoords(1) - aMinUVWCoords(1);
    double tWLength = aMaxUVWCoords(2) - aMinUVWCoords(2);

    std::vector<double> tLength = {tULength,tVLength,tWLength};
    std::vector<Vector> tBasis = {aUBasisVector,aVBasisVector,aBuildDirection};

    if(tULength < 0 || tVLength < 0 || tWLength < 0)
        throw(std::domain_error("AMFilterUtilities::computeGridXYZCoordinates: Max UVW coordinates expected to be greater than Min UVW coordinates"));

    if(aNumElements[0] <= 0 || aNumElements[1] <= 0 || aNumElements[2] <= 0)
        throw(std::domain_error("AMFilterUtilities::computeGridXYZCoordinates: Number of elements in each direction must be greater than zero"));

    aXYZCoordinates.resize((aNumElements[0]+1) * (aNumElements[1]+1) * (aNumElements[2]+1));


    for(int i = 0; i <= aNumElements[0]; ++i)
    {
        for(int j = 0; j <= aNumElements[1]; ++j)
        {
            for(int k = 0; k <= aNumElements[2]; ++k)
            {

                Vector tXYZCoordinates({0.0,0.0,0.0});
                Vector tUVWCoordinates({0.0,0.0,0.0});

                std::vector<int> tIndex = {i,j,k};

                for(int tTempIndex = 0; tTempIndex < 3; ++tTempIndex)
                {
                    double tUVWCoordinate = aMinUVWCoords(tTempIndex) + tIndex[tTempIndex]*tLength[tTempIndex]/aNumElements[tTempIndex]; 
                    tUVWCoordinates.set(tTempIndex,tUVWCoordinate);
                }

                tXYZCoordinates = tUVWCoordinates(0)*aUBasisVector + tUVWCoordinates(1)*aVBasisVector + tUVWCoordinates(2)*aBuildDirection;

                aXYZCoordinates[getSerializedIndex(aNumElements,i,j,k)] = tXYZCoordinates;
            }
        }
    }


    // return tXYZCoordinates;
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

bool isPointInTetrahedron(const std::vector<std::vector<double>>& aCoordinates,const std::vector<int>& aTet, const Vector& aPoint)
{
    checkTet(aTet, aCoordinates.size());

    return sameSide(aCoordinates, aTet[0], aTet[1], aTet[2], aTet[3], aPoint) &&
           sameSide(aCoordinates, aTet[1], aTet[2], aTet[3], aTet[0], aPoint) &&
           sameSide(aCoordinates, aTet[2], aTet[3], aTet[0], aTet[1], aPoint) &&
           sameSide(aCoordinates, aTet[3], aTet[0], aTet[1], aTet[2], aPoint);
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

bool sameSide(const std::vector<std::vector<double>>& aCoordinates, const int& v1, const int& v2, const int& v3, const int& v4, const Vector& aPoint)
{
    Vector tVec1(aCoordinates[v1]);
    Vector tVec2(aCoordinates[v2]);
    Vector tVec3(aCoordinates[v3]);
    Vector tVec4(aCoordinates[v4]);

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
