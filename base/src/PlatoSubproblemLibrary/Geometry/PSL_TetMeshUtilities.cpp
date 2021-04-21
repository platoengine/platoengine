#include <PSL_TetMeshUtilities.hpp>
#include "PSL_FreeHelpers.hpp"
#include <iostream>
#include <limits>
#include <cmath>

namespace PlatoSubproblemLibrary
{

void TetMeshUtilities::computeBoundingBox(const Vector& aUBasisVector,
                                          const Vector& aVBasisVector,
                                          const Vector& aWBasisVector,
                                          Vector& aMaxUVWCoords,
                                          Vector& aMinUVWCoords) const
{
    Vector tCoords(mCoordinates[0]);

    aMaxUVWCoords.set(0, dot_product(tCoords, aUBasisVector));
    aMaxUVWCoords.set(1, dot_product(tCoords, aVBasisVector));
    aMaxUVWCoords.set(2, dot_product(tCoords, aWBasisVector));

    aMinUVWCoords.set(0, dot_product(tCoords, aUBasisVector));
    aMinUVWCoords.set(1, dot_product(tCoords, aVBasisVector));
    aMinUVWCoords.set(2, dot_product(tCoords, aWBasisVector));

    for(auto tNodeCoordinates : mCoordinates)
    {
        Vector tNodeVec(tNodeCoordinates);

        if(dot_product(tNodeVec, aUBasisVector) > aMaxUVWCoords(0))
            aMaxUVWCoords.set(0,dot_product(tNodeVec, aUBasisVector));
        if(dot_product(tNodeVec, aVBasisVector) > aMaxUVWCoords(1))
            aMaxUVWCoords.set(1,dot_product(tNodeVec, aVBasisVector));
        if(dot_product(tNodeVec, aWBasisVector) > aMaxUVWCoords(2))
            aMaxUVWCoords.set(2,dot_product(tNodeVec, aWBasisVector));

        if(dot_product(tNodeVec, aUBasisVector) < aMinUVWCoords(0))
            aMinUVWCoords.set(0,dot_product(tNodeVec, aUBasisVector));
        if(dot_product(tNodeVec, aVBasisVector) < aMinUVWCoords(1))
            aMinUVWCoords.set(1,dot_product(tNodeVec, aVBasisVector));
        if(dot_product(tNodeVec, aWBasisVector) < aMinUVWCoords(2))
            aMinUVWCoords.set(2,dot_product(tNodeVec, aWBasisVector));
    }
}

double TetMeshUtilities::computeMinEdgeLength() const
{
    double tMinEdgeLength = std::numeric_limits<double>::max();

    for(auto tElement : mConnectivity)
    {
        Vector tNode0(mCoordinates[tElement[0]]);
        Vector tNode1(mCoordinates[tElement[1]]);
        Vector tNode2(mCoordinates[tElement[2]]);
        Vector tNode3(mCoordinates[tElement[3]]);

        Vector tEdge0(tNode1);
        tEdge0.subtract(tNode0);
        Vector tEdge1(tNode2);
        tEdge1.subtract(tNode0);
        Vector tEdge2(tNode3);
        tEdge2.subtract(tNode0);
        Vector tEdge3(tNode1);
        tEdge3.subtract(tNode2);
        Vector tEdge4(tNode3);
        tEdge4.subtract(tNode2);
        Vector tEdge5(tNode3);
        tEdge5.subtract(tNode1);

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

void checkTet(const std::vector<int>& aTet, const int& aNumNodes)
{
    if(aTet.size() != 4)
        throw(std::domain_error("TetMeshUtilities::pointInTetrahedron: Expected tetrahedron to contain 4 vertices"));

    auto tTemp = aTet;

    std::sort(tTemp.begin(),tTemp.end());
    auto tLast = std::unique(tTemp.begin(),tTemp.end());
    
    if(tLast != tTemp.end())
        throw(std::domain_error("TetMeshUtilities::pointInTetrahedron: repeated node index"));

    auto tMaxIterator = std::max_element(tTemp.begin(),tTemp.end());
    auto tMinIterator = std::min_element(tTemp.begin(),tTemp.end());

    if(*tMinIterator < 0 || *tMaxIterator >= (int) aNumNodes)
        throw(std::out_of_range("TetMeshUtilities::pointInTetrahedron: node index out of range"));
}

bool isPointInTetrahedron(const std::vector<std::vector<double>>& aCoordinates,const std::vector<int>& aTet, const Vector& aPoint)
{
    checkTet(aTet, aCoordinates.size());

    return sameSide(aCoordinates, aTet[0], aTet[1], aTet[2], aTet[3], aPoint) &&
           sameSide(aCoordinates, aTet[1], aTet[2], aTet[3], aTet[0], aPoint) &&
           sameSide(aCoordinates, aTet[2], aTet[3], aTet[0], aTet[1], aPoint) &&
           sameSide(aCoordinates, aTet[3], aTet[0], aTet[1], aTet[2], aPoint);
}

std::vector<double> TetMeshUtilities::computeBarycentricCoordinates(const std::vector<int>& aTet, const Vector& aPoint) const
{
    checkTet(aTet, mCoordinates.size());

    Vector tR1(mCoordinates[aTet[0]]);
    Vector tR2(mCoordinates[aTet[1]]);
    Vector tR3(mCoordinates[aTet[2]]);
    Vector tR4(mCoordinates[aTet[3]]);

    Vector tRightHandSide(aPoint);
    tRightHandSide.subtract(tR4);

    // matrix
    Vector tColumn1({tR1(0) - tR4(0), tR1(1) - tR4(1), tR1(2) - tR4(2)});
    Vector tColumn2({tR2(0) - tR4(0), tR2(1) - tR4(1), tR2(2) - tR4(2)});
    Vector tColumn3({tR3(0) - tR4(0), tR3(1) - tR4(1), tR3(2) - tR4(2)});

    // use Cramers rule to solve the system
    double tDeterminant = determinant3X3(tColumn1,tColumn2,tColumn3);

    if(fabs(tDeterminant) < 1e-12)
        throw(std::domain_error("TetMeshUtilities::computeBarycentricCoordinates: Tetrahedron is singular"));

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

    tVec2.subtract(tVec1);
    tVec3.subtract(tVec1);
    tVec4.subtract(tVec1);
    Vector tTemp(aPoint);
    tTemp.subtract(tVec1);

    Vector tNormal = cross_product(tVec2, tVec3);

    double tDot1 = dot_product(tNormal, tVec4);
    double tDot2 = dot_product(tNormal, tTemp);

    return tDot1*tDot2 >= 0.0;
}

void TetMeshUtilities::checkInput() const
{
    if(mConnectivity.size() == 0 || mCoordinates.size() < 4)
        throw(std::domain_error("TetMeshUtilities expected at least one tetrahedron in mesh"));

    for(auto tCoord: mCoordinates)
    {
        if(tCoord.size() != 3u)
            throw(std::domain_error("TetMeshUtilities expected 3 dimensional coordinates"));
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
            throw(std::domain_error("TetMeshUtilities expected tetrahedral elements"));
    }

    if(tMinNodeID < 0 || tMaxNodeID >= (int) mCoordinates.size())
    {
        std::string tError = std::string("Node IDs must be between zero and ") + std::to_string(mCoordinates.size() - 1);
        throw(std::out_of_range(tError));
    }
}

}
