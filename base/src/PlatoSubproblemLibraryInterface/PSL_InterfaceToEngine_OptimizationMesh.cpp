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

#include "PSL_InterfaceToEngine_OptimizationMesh.hpp"

#include "PSL_Abstract_OptimizationMesh.hpp"
#include "PSL_Point.hpp"
#include "data_mesh.hpp"
#include "topological_element.hpp"

#include <cstddef>

namespace Plato
{

InterfaceToEngine_OptimizationMesh::InterfaceToEngine_OptimizationMesh(DataMesh* mesh) :
        PlatoSubproblemLibrary::AbstractInterface::OptimizationMesh(),
        m_mesh(mesh)
{
}

InterfaceToEngine_OptimizationMesh::~InterfaceToEngine_OptimizationMesh()
{
    m_mesh = NULL;
}

PlatoSubproblemLibrary::Point InterfaceToEngine_OptimizationMesh::get_point(const size_t& index)
{
    Real* X = m_mesh->getX();
    const double x = X[index];
    Real* Y = m_mesh->getY();
    const double y = Y[index];
    Real* Z = m_mesh->getZ();
    double z = 0;
    if(Z != NULL)
    {
        z = Z[index];
    }
    PlatoSubproblemLibrary::Point result(index, {x, y, z});
    return result;
}

size_t InterfaceToEngine_OptimizationMesh::get_num_points()
{
    return m_mesh->getNumNodes();
}

size_t InterfaceToEngine_OptimizationMesh::get_num_blocks()
{
    return m_mesh->getNumElemBlks();
}

size_t InterfaceToEngine_OptimizationMesh::get_num_elements(size_t block_index)
{
    return m_mesh->getNumElemInBlk(block_index);
}

std::vector<size_t> InterfaceToEngine_OptimizationMesh::get_nodes_from_element(size_t block_index, size_t element_index)
{
    std::vector<size_t> result;

    // get block
    Topological::Element& elblock = *(m_mesh->getElemBlk(block_index));

    // get element
    int* elemConnect = elblock.Connect(element_index);

    // get nodes
    int numNodesPerElem = elblock.getNnpe();
    for(int inode = 0; inode < numNodesPerElem; inode++)
    {
        result.push_back(elemConnect[inode]);
    }

    return result;
}

bool InterfaceToEngine_OptimizationMesh::is_block_optimizable(size_t block_index)
{
    // TODO: when is this true? when is this false?
    return true;
}

}

