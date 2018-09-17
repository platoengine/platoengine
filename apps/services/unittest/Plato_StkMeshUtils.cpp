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

/*
 * Plato_StkMeshUtils.cpp
 *
 *  Created on: Oct 17, 2017
 */

#ifdef STK_ENABLED

#include <stk_io/StkMeshIoBroker.hpp>
#include <stk_mesh/base/GetEntities.hpp>

#include "Plato_StkMeshUtils.hpp"

namespace Plato
{

// create a comparator so we can sort by the local node id (local node index in old exodus style);
struct LocalIdComparator
{
    LocalIdComparator(const stk::mesh::BulkData & aBulkData) :
            mBulkData(aBulkData)
    {
    }
    bool operator()(stk::mesh::Entity aLHS, stk::mesh::Entity aRHS)
    {
        return mBulkData.local_id(aLHS) < mBulkData.local_id(aRHS);
    }
    const stk::mesh::BulkData & mBulkData;
};

// given a selector, get the node global ids of that set
void getNodesFromSelector(const stk::mesh::BulkData & aBulkData,
                          const stk::mesh::Selector & aNodeSelector, // stk mesh selector for the set of nodes
                          std::vector<int> & aSelectedNodes) // node global ids of the selected nodes
{
    // get the node entities for the selector
    std::vector<stk::mesh::Entity> tNodes;
    stk::mesh::get_selected_entities(aNodeSelector, aBulkData.buckets(stk::topology::NODE_RANK), tNodes);

    // sort nodes using comparitor from above, essentially sorting by "local id" (looks like local node index in exodus language)
    std::sort(tNodes.begin(), tNodes.end(), LocalIdComparator(aBulkData));

    // now put the global ids of these sorted nodes into our output vector
    aSelectedNodes.assign(tNodes.size(), 0);
    for(size_t nodeIndex = 0; nodeIndex < tNodes.size(); nodeIndex++)
    {
        aSelectedNodes[nodeIndex] = aBulkData.identifier(tNodes[nodeIndex]); // get global id for this node
    }
}

void buildCommunicationOwnedAndSharedNodeMap(const stk::mesh::BulkData & aBulkData,
                                             const stk::mesh::MetaData & aMetaData,
                                             std::vector<int> & aLocallyAllGlobalNodeIds)
{
    // get all nodes, both owned and shared
    stk::mesh::Selector tAllLocalSelector = aMetaData.locally_owned_part() | aMetaData.globally_shared_part();
    getNodesFromSelector(aBulkData, tAllLocalSelector, aLocallyAllGlobalNodeIds);
}

void buildCommunicationOwnedNodeMap(const stk::mesh::BulkData & aBulkData,
                                     const stk::mesh::MetaData & aMetaData,
                                     std::vector<int> & aLocallyOwnedGlobalNodeIds)
{
    // get only owned nodes
    stk::mesh::Selector tOwnedLocalSelector = aMetaData.locally_owned_part();
    getNodesFromSelector(aBulkData, tOwnedLocalSelector, aLocallyOwnedGlobalNodeIds);
}

}

#endif // STK_ENABLED
