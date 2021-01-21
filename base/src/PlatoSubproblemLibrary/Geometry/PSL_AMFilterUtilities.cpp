#include <PSL_AMFilterUtilities.hpp>
#include <iostream>

namespace PlatoSubproblemLibrary
{

void AMFilterUtilities::getBoundingBox(Vector& aMaxUVWCoords, Vector& aMinUVWCoords) const
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

void AMFilterUtilities::checkInput() const
{
    if(mConnectivity.size() == 0 || mCoordinates.size() < 4)
        throw(std::domain_error("AMFilterUtilities expected at least one tetrahedron in mesh"));

    if(mBaseLayer.size() == 0)
        throw(std::domain_error("AMFilterUtilities at least one node must be specified on the base layer"));

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

    for(auto tNodeID : mBaseLayer)
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
