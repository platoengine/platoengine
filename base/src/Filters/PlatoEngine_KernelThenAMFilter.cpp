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

#include "PlatoEngine_KernelThenAMFilter.hpp"
#include "PSL_KernelThenAMFilter.hpp"
#include "topological_element.hpp"
#include "XMLG_Macros.hpp"
#include "data_mesh.hpp"

namespace Plato
{

    void KernelThenAMFilter::allocateFilter()
    {
        m_filter =  new PlatoSubproblemLibrary::KernelThenAMFilter(m_authority,
                                                                   m_input_data,
                                                                   m_points,
                                                                   m_parallel_exchanger);
    }

    void KernelThenAMFilter::build(InputData aInputData, MPI_Comm& aLocalComm, DataMesh* aMesh)
    {
        printf("executing KernelThenAMFilter");
        AbstractKernelThenFilter::build(aInputData, aLocalComm, aMesh);
        extractMeshData(aMesh);
        PlatoSubproblemLibrary::KernelThenAMFilter* tFilter = dynamic_cast<PlatoSubproblemLibrary::KernelThenAMFilter*>(m_filter);
        tFilter->setCoordinates(mCoordinates);
        tFilter->setConnectivity(mConnectivity);
        tFilter->buildPseudoLayers();
    }

    void KernelThenAMFilter::extractMeshData(DataMesh* aMesh)
    {
        DataMesh& tMesh = *aMesh;

        int tNumNodes = tMesh.getNumNodes();

        double* tXCoordinates = tMesh.getX();
        double* tYCoordinates = tMesh.getY();
        double* tZCoordinates = tMesh.getZ();

        mCoordinates = std::vector<std::vector<double>>(tNumNodes, std::vector<double>(3));

        for(int i = 0; i < tNumNodes; ++i)
        {
            mCoordinates[i][0] = tXCoordinates[i];
            mCoordinates[i][1] = tYCoordinates[i];
            mCoordinates[i][2] = tZCoordinates[i];
        }

        int tNumElemBlocks = tMesh.getNumElemBlks();

        for(int tBlockIndex = 0; tBlockIndex < tNumElemBlocks; ++tBlockIndex)
        {
            Topological::Element& tElementBlock = *(tMesh.getElemBlk(tBlockIndex));

            // not all blocks will be present on all processors
            if( tElementBlock.getNumElem() == 0 ) continue;

            // don't process 1D blocks.  This prevents including RBAR elements
            if( tElementBlock.getDim() == 1 ) continue;

            if(tElementBlock.getDim() != mDimension)
                THROWERR("Only 3D meshes are currently supported for AM Filter")

            if(tElementBlock.getNnpe() != mNumNodesPerElement)
                THROWERR("Only tetrahedra are currently supported for AM Filter")

            int tNumElementsInThisBlock = tElementBlock.getNumElem();

            for(int tElementInBlockIndex = 0; tElementInBlockIndex < tNumElementsInThisBlock; ++tElementInBlockIndex)
            {
                std::vector<int> tElementConnectivityVector;
                int* tElementConnectivity = tElementBlock.Connect(tElementInBlockIndex);
                for(int tConnectivityIndex = 0; tConnectivityIndex < mNumNodesPerElement; ++tConnectivityIndex)
                {
                    tElementConnectivityVector.push_back(tElementConnectivity[tConnectivityIndex]);
                }
                mConnectivity.push_back(tElementConnectivityVector);
            }
        }
    }
}
