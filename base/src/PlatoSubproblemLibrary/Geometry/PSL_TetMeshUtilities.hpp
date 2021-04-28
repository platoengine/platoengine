#pragma once

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <cassert>
#include <tuple>
#include <set>
#include <map>
#include <PSL_Vector.hpp>

#include <ArborX.hpp>
#include <Kokkos_Core.hpp>

using ExecSpace = Kokkos::DefaultExecutionSpace;
using MemSpace = typename ExecSpace::memory_space;
using DeviceType = Kokkos::Device<ExecSpace, MemSpace>;

struct BoundingBoxes
{
  double *d_x0;
  double *d_y0;
  double *d_z0;
  double *d_x1;
  double *d_y1;
  double *d_z1;
  int N;
};

struct Points
{
  double *d_x;
  double *d_y;
  double *d_z;
  int N;
};

namespace ArborX
{
namespace Traits
{
template <>
struct Access<BoundingBoxes, PrimitivesTag>
{
  inline static std::size_t size(BoundingBoxes const &boxes) { return boxes.N; }
  KOKKOS_INLINE_FUNCTION static Box get(BoundingBoxes const &boxes, std::size_t i)
  {
    return {{boxes.d_x0[i], boxes.d_y0[i], boxes.d_z0[i]},
            {boxes.d_x1[i], boxes.d_y1[i], boxes.d_z1[i]}};
  }
  using memory_space = MemSpace;
};

template <>
struct Access<Points, PrimitivesTag>
{
  inline static std::size_t size(Points const &points) { return points.N; }
  KOKKOS_INLINE_FUNCTION static Point get(Points const &points, std::size_t i)
  {
    return {{points.d_x[i], points.d_y[i], points.d_z[i]}};
  }
  using memory_space = MemSpace;
};

template <>
struct Access<Points, PredicatesTag>
{
  inline static std::size_t size(Points const &d) { return d.N; }
  KOKKOS_INLINE_FUNCTION static auto get(Points const &d, std::size_t i)
  {
    return intersects(Point{d.d_x[i], d.d_y[i], d.d_z[i]});
  }
  using memory_space = MemSpace;
};

} // namespace Traits
}

namespace PlatoSubproblemLibrary
{

bool sameSide(const std::vector<std::vector<double>>& aCoordinates, const int& v1, const int& v2, const int& v3, const int& v4, const Vector& aPoint);
bool isPointInTetrahedron(const std::vector<std::vector<double>>& aCoordinates,const std::vector<int>& aTet, const Vector& aPoint);

class TetMeshUtilities
{
    public:
        TetMeshUtilities(const std::vector<std::vector<double>> aCoordinates,
                         const std::vector<std::vector<int>> aConnectivity)
            :mCoordinates(aCoordinates),
             mConnectivity(aConnectivity)
        {
            checkInput();
        }

        ~TetMeshUtilities(){}

        void computeBoundingBox(const Vector& aUBasisVector,
                                const Vector& aVBasisVector,
                                const Vector& aWBasisVector,
                                Vector& aMaxUVWCoords,
                                Vector& aMinUVWCoords) const;

        double computeMinEdgeLength() const;


        std::vector<double> computeBarycentricCoordinates(const std::vector<int>& aTet, const Vector& aPoint) const;

        const std::vector<std::vector<double>>& getCoordinates() const {return mCoordinates;};
        const std::vector<std::vector<int>>& getConnectivity() const {return mConnectivity;};

        /***************************************************************************//**
        * @brief Find id of tet that contains each point 
         * @param [in]  aPoints 
         * @param [out] aContainingTetID

         aContainingTetID is set to -1 if the point is outside the mesh, otherwise it
         is set to the index of the tet in mConnectivity
        *******************************************************************************/
        void
        getTetIDForEachPoint(const std::vector<Vector>& aPoints,
                             std::vector<int>& aContainingTetID) const
        {
            
            constexpr int cDimension = 3;
            int tNumPoints = aPoints.size();
            int tNumTets = mConnectivity.size();
            aContainingTetID.resize(tNumPoints);

            std::vector<std::vector<double>> tMinXYZCoordinates(cDimension, std::vector<double>(tNumTets));
            std::vector<std::vector<double>> tMaxXYZCoordinates(cDimension, std::vector<double>(tNumTets));

            constexpr double cRelativeTol = 1e-2;

            // compute bounding box for each tet element
            
            // set min and max to coordinate values of the first node of the tet
            for(int tTetIndex = 0; tTetIndex < tNumTets; ++tTetIndex)
            {
                auto tTet = mConnectivity[tTetIndex];
                auto tFirstNode = tTet[0];
                for(int tDimIndex = 0; tDimIndex < cDimension; ++tDimIndex)
                {
                    tMaxXYZCoordinates[tDimIndex][tTetIndex] = mCoordinates[tFirstNode][tDimIndex];
                    tMinXYZCoordinates[tDimIndex][tTetIndex] = mCoordinates[tFirstNode][tDimIndex];
                }

                constexpr int cNumNodesPerTet = 4;

                //loop on remaining nodes to find tet bounding box
                for(int tNodeIndex = 1; tNodeIndex < cNumNodesPerTet; ++tNodeIndex)
                {
                    auto tNode = tTet[tNodeIndex];
                    auto tNodeCoordinates = mCoordinates[tNode];
                    for(int tDimIndex = 0; tDimIndex < cDimension; ++tDimIndex)
                    {
                        if(tNodeCoordinates[tDimIndex] > tMaxXYZCoordinates[tDimIndex][tTetIndex])
                        {
                            tMaxXYZCoordinates[tDimIndex][tTetIndex] = tNodeCoordinates[tDimIndex];
                        }
                        else if(tNodeCoordinates[tDimIndex] < tMinXYZCoordinates[tDimIndex][tTetIndex])
                        {
                            tMinXYZCoordinates[tDimIndex][tTetIndex] = tNodeCoordinates[tDimIndex];
                        }
                    }

                }

                for(int tDimIndex = 0; tDimIndex < cDimension; ++tDimIndex)
                {
                    double tLength = tMaxXYZCoordinates[tDimIndex][tTetIndex] - tMinXYZCoordinates[tDimIndex][tTetIndex];
                    tMaxXYZCoordinates[tDimIndex][tTetIndex] += cRelativeTol * tLength;
                    tMinXYZCoordinates[tDimIndex][tTetIndex] -= cRelativeTol * tLength;
                }
            }

            // construct search tree
            ArborX::BVH<DeviceType>
              bvh{BoundingBoxes{tMinXYZCoordinates[0].data(), tMinXYZCoordinates[1].data(), tMinXYZCoordinates[2].data(),
                                tMaxXYZCoordinates[0].data(), tMaxXYZCoordinates[1].data(), tMaxXYZCoordinates[2].data(), tNumTets}};

            
            std::vector<double> tPointsX(tNumPoints);
            std::vector<double> tPointsY(tNumPoints);
            std::vector<double> tPointsZ(tNumPoints);

            for(auto tPointIndex = 0; tPointIndex < tNumPoints; ++tPointIndex)
            {
                auto tPoint = aPoints[tPointIndex];
                tPointsX[tPointIndex] = tPoint.X();
                tPointsY[tPointIndex] = tPoint.Y();
                tPointsZ[tPointIndex] = tPoint.Z();
            }

            Kokkos::View<int*, DeviceType> tIndices("indices", 0), tOffset("offset", 0);
            bvh.query(Points{tPointsX.data(), tPointsY.data(), tPointsZ.data(), tNumPoints}, tIndices, tOffset);

            for(int tPointIndex = 0; tPointIndex < tNumPoints; ++tPointIndex)
            {
                aContainingTetID[tPointIndex] = -1;

                for(int i = tOffset(tPointIndex); i < tOffset(tPointIndex+1); ++i)
                {
                    int tTetIndex = tIndices(i);
                    std::vector<int> tTet = mConnectivity[tTetIndex];
                    Vector tPoint = aPoints[tPointIndex];
                    if(isPointInTetrahedron(mCoordinates,tTet,tPoint))
                    {
                        aContainingTetID[tPointIndex] = tTetIndex;
                    }
                }
            }
        }

    private:

        void checkInput() const;



    private:
        const std::vector<std::vector<double>> mCoordinates;
        const std::vector<std::vector<int>> mConnectivity;
};

}
