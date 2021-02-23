#ifndef PSL_MESHMAP_HPP_
#define PSL_MESHMAP_HPP_

#include <ArborX.hpp>

#include <Kokkos_Core.hpp>
#include <vector>

/***************************************************************************//**
* @brief MeshMap

   This base class contains most of the functionality needed for a MeshMap. The
   MathMap (i.e., Full, SymmetryPlane, etc) is added in the template derived
   class.
*******************************************************************************/
template <typename ScalarT, typename OrdinalT>
class MeshMap
{
    private: 
        const std::vector<std::vector<double>>& mCoordinates;
        const std::vector<std::vector<int>>& mConnectivity;

    public:
        MeshMap(const std::vector<std::vector<ScalarT>>& aCoordinates,
                const std::vector<std::vector<OrdinalT>>& aConnectivity)
            :mCoordinates(aCoordinates),
             mConnectivity(aConnectivity)
    {
        checkInput();
    }

    /***************************************************************************//**
    * @brief Find id of tet that contains each point 
     * @param [in]  aPoints 
     * @param [out] aContainingTetID

     aContainingTetID is set to -1 if the point is outside the mesh, otherwise it
     is set to the index of the tet in mConnectivity
    *******************************************************************************/
    void
    getTetIDForEachPoint(std::vector<std::vector<ScalarT>> aPoints,
                         std::vector<OrdinalT> aContainingTetID)
    {
    //     auto tNElems = aMesh.nelems();
    //     VectorArrayT tMin("min", cSpaceDim, tNElems);
    //     VectorArrayT tMax("max", cSpaceDim, tNElems);

    //     constexpr ScalarT cRelativeTol = 1e-2;

    //     // fill d_* data
    //     auto tCoords = aMesh.coords();
    //     Omega_h::LOs tCells2Nodes = aMesh.ask_elem_verts();
    //     Kokkos::parallel_for(Kokkos::RangePolicy<OrdinalT>(0, tNElems), LAMBDA_EXPRESSION(OrdinalT iCellOrdinal)
    //     {
    //         OrdinalT tNVertsPerElem = cSpaceDim+1;

    //         // set min and max of element bounding box to first node
    //         for(size_t iDim=0; iDim<cSpaceDim; ++iDim)
    //         {
    //             OrdinalT tVertIndex = tCells2Nodes[iCellOrdinal*tNVertsPerElem];
    //             tMin(iDim, iCellOrdinal) = tCoords[tVertIndex*cSpaceDim+iDim];
    //             tMax(iDim, iCellOrdinal) = tCoords[tVertIndex*cSpaceDim+iDim];
    //         }
    //         // loop on remaining nodes to find min
    //         for(OrdinalT iVert=1; iVert<tNVertsPerElem; ++iVert)
    //         {
    //             OrdinalT tVertIndex = tCells2Nodes[iCellOrdinal*tNVertsPerElem + iVert];
    //             for(size_t iDim=0; iDim<cSpaceDim; ++iDim)
    //             {
    //                 if( tMin(iDim, iCellOrdinal) > tCoords[tVertIndex*cSpaceDim+iDim] )
    //                 {
    //                     tMin(iDim, iCellOrdinal) = tCoords[tVertIndex*cSpaceDim+iDim];
    //                 }
    //                 else
    //                 if( tMax(iDim, iCellOrdinal) < tCoords[tVertIndex*cSpaceDim+iDim] )
    //                 {
    //                     tMax(iDim, iCellOrdinal) = tCoords[tVertIndex*cSpaceDim+iDim];
    //                 }
    //             }
    //         }
    //         for(size_t iDim=0; iDim<cSpaceDim; ++iDim)
    //         {
    //             ScalarT tLen = tMax(iDim, iCellOrdinal) - tMin(iDim, iCellOrdinal);
    //             tMax(iDim, iCellOrdinal) += cRelativeTol * tLen;
    //             tMin(iDim, iCellOrdinal) -= cRelativeTol * tLen;
    //         }
    //     }, "element bounding boxes");


    //     auto d_x0 = Kokkos::subview(tMin, (size_t)Dim::X, Kokkos::ALL());
    //     auto d_y0 = Kokkos::subview(tMin, (size_t)Dim::Y, Kokkos::ALL());
    //     auto d_z0 = Kokkos::subview(tMin, (size_t)Dim::Z, Kokkos::ALL());
    //     auto d_x1 = Kokkos::subview(tMax, (size_t)Dim::X, Kokkos::ALL());
    //     auto d_y1 = Kokkos::subview(tMax, (size_t)Dim::Y, Kokkos::ALL());
    //     auto d_z1 = Kokkos::subview(tMax, (size_t)Dim::Z, Kokkos::ALL());

    //     // construct search tree
    //     ArborX::BVH<DeviceType>
    //       bvh{BoundingBoxes{d_x0.data(), d_y0.data(), d_z0.data(),
    //                         d_x1.data(), d_y1.data(), d_z1.data(), tNElems}};

    //     // conduct search for bounding box elements
    //     auto d_x = Kokkos::subview(aMappedLocations, (size_t)Dim::X, Kokkos::ALL());
    //     auto d_y = Kokkos::subview(aMappedLocations, (size_t)Dim::Y, Kokkos::ALL());
    //     auto d_z = Kokkos::subview(aMappedLocations, (size_t)Dim::Z, Kokkos::ALL());

    //     auto tNVerts = aMesh.nverts();
    //     Kokkos::View<int*, DeviceType> tIndices("indices", 0), tOffset("offset", 0);
    //     bvh.query(Points{d_x.data(), d_y.data(), d_z.data(), tNVerts}, tIndices, tOffset);

    //     // loop over indices and find containing element
    //     GetBasis<ScalarT> tGetBasis(aMesh);
    //     Kokkos::parallel_for(Kokkos::RangePolicy<OrdinalT>(0, tNVerts), LAMBDA_EXPRESSION(OrdinalT iNodeOrdinal)
    //     {
    //         ScalarT tBasis[cNVertsPerElem];
    //         aParentElements(iNodeOrdinal) = -1;
    //         if( aLocations(Dim::X, iNodeOrdinal) != aMappedLocations(Dim::X, iNodeOrdinal) ||
    //             aLocations(Dim::Y, iNodeOrdinal) != aMappedLocations(Dim::Y, iNodeOrdinal) ||
    //             aLocations(Dim::Z, iNodeOrdinal) != aMappedLocations(Dim::Z, iNodeOrdinal) )
    //         {
    //             aParentElements(iNodeOrdinal) = -2;
    //             constexpr ScalarT cNotFound = -1e8; // big negative number ensures max min is found
    //             ScalarT tMaxMin = cNotFound;
    //             typename IntegerArrayT::value_type iParent = -2;
    //             for( int iElem=tOffset(iNodeOrdinal); iElem<tOffset(iNodeOrdinal+1); iElem++ )
    //             {
    //                 auto tElem = tIndices(iElem);
    //                 tGetBasis(aMappedLocations, iNodeOrdinal, tElem, tBasis);
    //                 ScalarT tMin = tBasis[0];
    //                 for(OrdinalT iB=1; iB<cNVertsPerElem; iB++)
    //                 {
    //                     if( tBasis[iB] < tMin ) tMin = tBasis[iB];
    //                 }
    //                 if( tMin > cNotFound )
    //                 {
    //                      tMaxMin = tMin;
    //                      iParent = tElem;
    //                 }
    //             }
    //             if( tMaxMin > cNotFound )
    //             {
    //                 aParentElements(iNodeOrdinal) = iParent;
    //             }
    //         }
    //     }, "find parent element");
    }

    private:
        
        void checkInput()
        {
            if(mConnectivity.size() == 0 || mCoordinates.size() < 4)
                throw(std::domain_error("AMFilterUtilities expected at least one tetrahedron in mesh"));

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

            if(tMinNodeID < 0 || tMaxNodeID >= (int) mCoordinates.size())
            {
                std::string tError = std::string("Node IDs must be between zero and ") + std::to_string(mCoordinates.size() - 1);
                throw(std::out_of_range(tError));
            }
        }
};
#endif
