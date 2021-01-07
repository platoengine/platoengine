#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <cassert>
#include <tuple>
#include <set>
#include <map>
#include <PSL_Vector.hpp>

namespace PlatoSubproblemLibrary
{

// SupportPointData contains the ID of the supported node, and a set of 
// one or two node IDs needed to compute the support point location and density value
using SupportPointData = std::pair<int, std::set<int>>; 

class PseudoLayerBuilder
{
    public:
        PseudoLayerBuilder(const std::vector<std::vector<double>>& aCoordinates,
                           const std::vector<std::vector<int>>& aConnectivity,
                           const double& aCriticalPrintAngle,
                           PlatoSubproblemLibrary::Vector aBuildDirection,
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

        void computeSupportSetAndCoefficients(std::vector<std::set<SupportPointData>>& aBoundarySupportSet,
                                              std::map<SupportPointData,std::vector<double>>& aBoundarySupportCoefficients,
                                              std::map<std::pair<int,int>, std::vector<std::vector<double>>>& aInteriorSupportCoefficients) const;

        int assignNodeToPseudoLayer(const int& aNode,
                                    const std::vector<int>& aPseudoLayers,
                                    const std::set<SupportPointData>& aSupportSet) const;

        std::set<SupportPointData> pruneSupportSet(const int& aNode,
                                                   const std::vector<int>& aPseudoLayers,
                                                   const std::set<SupportPointData>& aSupportSet,
                                                   std::map<SupportPointData, std::vector<double>>& aSupportCoefficients) const;

    private:

        double intersectionResidual(const double& aSuperUnkown,
                                    const PlatoSubproblemLibrary::Vector& aV0,
                                    const PlatoSubproblemLibrary::Vector& aV1,
                                    const PlatoSubproblemLibrary::Vector& aV2) const;

        double intersectionResidualDerivative(const double& aSuperUnkown,
                                              const PlatoSubproblemLibrary::Vector& aV0,
                                              const PlatoSubproblemLibrary::Vector& aV1,
                                              const PlatoSubproblemLibrary::Vector& aV2) const;


        bool isNeighborInCriticalWindow(const int& aNode, 
                                        const int& aNeighbor) const;

        std::vector<double> computeSupportCoefficients(const SupportPointData& aSupportPoint) const;

        void computeBoundarySupportPointsAndCoefficients(size_t& i,
                                                         std::vector<int>& aElement,
                                                         std::vector<std::set<SupportPointData>>& aBoundarySupportSet,
                                                         std::map<SupportPointData,std::vector<double>>& aBoundarySupportCoefficients) const;

        double computeFirstCoefficient(const PlatoSubproblemLibrary::Vector& aV0,
                                       const PlatoSubproblemLibrary::Vector& aV1,
                                       const PlatoSubproblemLibrary::Vector& aV2) const;

        std::set<int> getSupportingNeighbors(const int& aNode, const std::set<SupportPointData>& aSupportSet) const;
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
        PlatoSubproblemLibrary::Vector mBuildDirection;
        const std::vector<int>& mBaseLayer;
};

PlatoSubproblemLibrary::Vector getVectorToSupportPoint(const SupportPointData& aSupportPoint,
                                                       const std::map<PlatoSubproblemLibrary::SupportPointData,std::vector<double>>& aSupportCoefficients,
                                                       const std::vector<std::vector<double>>& aCoordinates);
}
