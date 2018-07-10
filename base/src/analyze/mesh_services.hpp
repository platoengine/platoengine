/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
*/

#ifndef MESH_SERVICES
#define MESH_SERVICES

#include "Intrepid_Basis.hpp"
#include "Intrepid_FieldContainer.hpp"
#include "Shards_CellTopology.hpp"
#include "matrix_container.hpp"

class DataMesh;
namespace Plato { class PenaltyModel; }

/******************************************************************************/
class MeshServices
/******************************************************************************/
{
  public:
    MeshServices(DataMesh* dataMesh) : myDataMesh(dataMesh){}

    double getTotalVolume();
    void getCurrentVolume(const DistributedVector& topo,
                          double& volume, DistributedVector& volumeGradient,
                          Plato::PenaltyModel* pModel=nullptr);

    void getRoughness(const DistributedVector& topo,
                      double& roughness, DistributedVector& roughnessGradient,
                      Plato::PenaltyModel* pModel=nullptr);
    void updateLowerBoundsForFixedBlocks(double *toData, const std::vector<int> &aFixedBlocks, const double &aFixedBlockValue);
    void updateLowerBoundsForFixedSidesets(double *toData, const std::vector<int> &aFixedSidesets, const double &aFixedSidesetsValue);
    void updateLowerBoundsForFixedNodesets(double *toData, const std::vector<int> &aFixedNodesets, const double &aFixedNodesetsValue);
    void updateUpperBoundsForFixedBlocks(double *toData, const std::vector<int> &mFixedBlocks, const double &aFixedBlockValue);
    void updateUpperBoundsForFixedSidesets(double *toData, const std::vector<int> &aFixedSidesets, const double &aFixedSidesetsValue);
    void updateUpperBoundsForFixedNodesets(double *toData, const std::vector<int> &aFixedNodesets, const double &aFixedNodesetsValue);

  private:
    DataMesh* myDataMesh;
    //shards::CellTopology **blockTopology;
    //Intrepid::Basis<double, Intrepid::FieldContainer<double> > **blockBasis;
};
/******************************************************************************/
#endif
