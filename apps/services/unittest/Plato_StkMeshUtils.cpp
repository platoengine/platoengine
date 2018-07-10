/*
 * Plato_StkMeshUtils.cpp
 *
 *  Created on: Oct 17, 2017
 */

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
