/*
 * Plato_StkMeshUtils.hpp
 *
 *  Created on: Oct 17, 2017
 */

#ifndef PLATO_STKMESHUTILS_HPP_
#define PLATO_STKMESHUTILS_HPP_

#include <vector>

#include <stk_mesh/base/BulkData.hpp>
#include <stk_mesh/base/MetaData.hpp>

namespace Plato
{

// given a selector, get the node global ids of that set
void getNodesFromSelector(const stk::mesh::BulkData & aBulkData,
                          const stk::mesh::Selector & aNodeSelector, // stk mesh selector for the set of nodes
                          std::vector<int> & aSelectedNodes); // node global ids of the selected nodes


void buildCommunicationOwnedAndSharedNodeMap(const stk::mesh::BulkData & aBulkData,
                                             const stk::mesh::MetaData & aMetaData,
                                             std::vector<int> & aLocallyAllGlobalNodeIds);

void buildCommunicationOwnedNodeMap(const stk::mesh::BulkData & aBulkData,
                                     const stk::mesh::MetaData & aMetaData,
                                     std::vector<int> & aLocallyOwnedGlobalNodeIds);

} // namespace Plato

#endif /* PLATO_STKMESHUTILS_HPP_ */
