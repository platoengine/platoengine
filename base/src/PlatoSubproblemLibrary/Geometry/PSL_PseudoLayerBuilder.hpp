#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <cassert>
#include <tuple>
#include <set>
#include <map>
#include <PSL_Vector.hpp>
#include <PSL_BoundarySupportPoint.hpp>

namespace PlatoSubproblemLibrary
{

class PseudoLayerBuilder
{
    public:
        PseudoLayerBuilder(const std::vector<std::vector<double>>& aCoordinates,
                           const std::vector<std::vector<int>>& aConnectivity,
                           const double& aCriticalPrintAngle,
                           Vector aBuildDirection,
                           const std::vector<int>& aBaseLayer) 
            :mCoordinates(aCoordinates),
             mConnectivity(aConnectivity),
             mCriticalPrintAngle(aCriticalPrintAngle),
             mBuildDirection(aBuildDirection),
             mBaseLayer(aBaseLayer)
        {
            checkInput();
            mBuildDirection.normalize();
        }

        ~PseudoLayerBuilder(){}

        std::vector<int> orderNodesInBuildDirection() const;
        std::vector<int> setBaseLayerIDToZeroAndOthersToMinusOne() const;

        void computeSupportSetAndCoefficients(std::vector<std::set<BoundarySupportPoint>>& aBoundarySupportSet) const;

        int assignNodeToPseudoLayer(const int& aNode,
                                    const std::vector<int>& aPseudoLayers,
                                    const std::set<BoundarySupportPoint>& aSupportSet) const;

        std::set<BoundarySupportPoint> pruneSupportSet(const int& aNode,
                                                   const std::vector<int>& aPseudoLayers,
                                                   const std::set<BoundarySupportPoint>& aSupportSet) const;

    private:

        double angleObjective(const double& aSuperUnkown,
                              const Vector& aV0,
                              const Vector& aV1,
                              const Vector& aV2) const;

        double angleObjectiveDerivative(const double& aSuperUnkown,
                                        const Vector& aV0,
                                        const Vector& aV1,
                                        const Vector& aV2) const;

        double intersectionResidual(const double& aSuperUnkown,
                                    const Vector& aV0,
                                    const Vector& aV1,
                                    const Vector& aV2) const;

        double intersectionResidualDerivative(const double& aSuperUnkown,
                                              const Vector& aV0,
                                              const Vector& aV1,
                                              const Vector& aV2) const;

        bool criticalWindowIntersectsEdge(const int& aNode,
                                          const int& aNeighbor0,
                                          const int& aNeighbor1,
                                          double& aFirstCoefficient) const;

        bool isNeighborInCriticalWindow(const int& aNode, 
                                        const int& aNeighbor) const;

        bool isPointInCriticalWindow(const int& aNode, const Vector& aCoordinateVec) const;

        std::vector<double> computeSupportCoefficients(const int& aNode, const std::set<int>& aSupportDependencyNodes) const;

        void computeBoundarySupportPointsAndCoefficients(size_t& aLocalIndexOnElement,
                                                         std::vector<int>& aElement,
                                                         std::vector<std::set<BoundarySupportPoint>>& aBoundarySupportSet) const;

        double determineVectorWithMinimalAngleBetweenEdgeAndBuildDirection(const Vector& aV0,
                                                                           const Vector& aV1,
                                                                           const Vector& aV2) const;

        double computeFirstCoefficient(const double& aInitialGuess,
                                       const Vector& aV0,
                                       const Vector& aV1,
                                       const Vector& aV2) const;

        double getInitialGuess(const int& aNode, const int& tNeighbor1, const int& tNeighbor2) const;

        std::set<int> getSupportingNeighbors(const int& aNode, const std::set<BoundarySupportPoint>& aSupportSet) const;
        std::set<int> determineConnectedPseudoLayers(const int& aNode, 
                                                     const std::vector<int>& aPseudoLayers,
                                                     const std::set<int>& aNeighbors) const;

        int determineSupportingPseudoLayer(const int& aNode,
                                           const std::set<int>& aNeighbors,
                                           const std::vector<int>& aPseudoLayers,
                                           const std::set<int>& aConnectedPseudoLayers) const;

        void checkInput() const;

    private:
        const std::vector<std::vector<double>>& mCoordinates;
        const std::vector<std::vector<int>>& mConnectivity;
        const double& mCriticalPrintAngle;
        Vector mBuildDirection;
        const std::vector<int>& mBaseLayer;
};

Vector getVectorToSupportPoint(const BoundarySupportPoint& aSupportPoint,
                                                       const std::vector<std::vector<double>>& aCoordinates);
}
