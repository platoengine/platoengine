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
            // constructNodeToElementsMap();
            mBuildDirection.normalize();
        }

        ~PseudoLayerBuilder(){}

        std::vector<int> orderNodesInBuildDirection() const;
        std::vector<int> setBaseLayerIDToZeroAndOthersToMinusOne() const;

        void computeSupportSetAndCoefficients(std::vector<std::set<SupportPointData>>& aSupportSet,
                                              std::map<SupportPointData,std::vector<double>>& aSupportCoefficients) const;

        int assignNodeToPseudoLayer(const int& aNode,
                                    const std::vector<int>& aPseudoLayers,
                                    const std::set<SupportPointData>& aSupportSet) const;

        std::set<SupportPointData> pruneSupportSet(const int& aNode,
                                                   const std::vector<int>& aPseudoLayers,
                                                   const std::set<SupportPointData>& aSupportSet) const;

        PlatoSubproblemLibrary::Vector getVectorToSupportPoint(const SupportPointData& aSupportPoint,
                                                               const std::map<PlatoSubproblemLibrary::SupportPointData,std::vector<double>>& aSupportCoefficients,
                                                               const std::vector<std::vector<double>>& aCoordinates) const;

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
        // void constructNodeToElementsMap();

    private:
        const std::vector<std::vector<double>>& mCoordinates;
        const std::vector<std::vector<int>>& mConnectivity;
        // std::map<int, std::set<int>> mNodeToElementsMap;
        const double& mCriticalPrintAngle;
        PlatoSubproblemLibrary::Vector mBuildDirection;
        const std::vector<int>& mBaseLayer;
};
}
