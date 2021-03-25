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

//-------------------------------------------------------------------------
// Filename      : IVEMeshAPISTK.cpp
//
// Description   : a IVEMeshAPISTK is an implementation of the IVEMeshAPI
//
// Creator       : Brett Clark
//
// Creation Date : 12/8/2014
//
// Owner         : Brett Clark
//-------------------------------------------------------------------------

#include "IVEMeshAPISTK.hpp"
#include <stk_mesh/base/MetaData.hpp>
#include <stk_mesh/base/Field.hpp>
#include <stk_mesh/base/CoordinateSystems.hpp>
#include <stk_util/parallel/ParallelReduce.hpp>
#include <stk_util/parallel/CommSparse.hpp>
#include <stk_io/StkMeshIoBroker.hpp>
#include <stk_mesh/base/Types.hpp>
#include <stk_mesh/base/FieldRestriction.hpp>
#include "Ioss_Region.h"                // for Region, NodeSetContainer, etc


struct less_than_boundary_global_id
{
  bool operator()(const BoundaryNodeInfo& bni1, const BoundaryNodeInfo& bni2) const
  {
    // Currently we can't assume the global id of the min node is 
    // less than the global id of the max node for each BoudnaryNodeInfo.
    // May need to refactor this code later to make it less confusing.
    IVEHandle min1, max1, min2, max2;
    if(bni1.min_node_global_id < bni1.max_node_global_id)
    {
      min1 = bni1.min_node_global_id;
      max1 = bni1.max_node_global_id;
    }
    else
    {
      max1 = bni1.min_node_global_id;
      min1 = bni1.max_node_global_id;
    }
    if(bni2.min_node_global_id < bni2.max_node_global_id)
    {
      min2 = bni2.min_node_global_id;
      max2 = bni2.max_node_global_id;
    }
    else
    {
      max2 = bni2.min_node_global_id;
      min2 = bni2.max_node_global_id;
    }
    if ( min1 != min2 )
    {
      return min1 < min2;
    }
    else
    {
      return max1 < max2;
    }
  }
};

struct less_than_dup_node
{
  bool operator()(const DuplicateNodeInfo& dni1, const DuplicateNodeInfo& dni2) const
  {
    return dni1.existing_node_global_id < dni2.existing_node_global_id;
  }
};

void IVEMeshAPISTK::initialize()
{
  mMetaData = NULL;
  mBulkData = NULL;
  mIoBroker = NULL;
  mFixedTriPart = NULL;
  mOptimizedTriPart = NULL;
  mLocallyOwnedBulk = false;
  mLocallyOwnedMeta = false;
  mIsoField = NULL;
  mFixedTriMap = NULL;
  mOptimizedTriMap = NULL;
  mCoordsField = NULL;
  mNextAvailableElemIndex = 0;
  mNextAvailableNodeIndex = 0;
  mTimeStep = 0;
}

IVEMeshAPISTK::IVEMeshAPISTK(stk::ParallelMachine* comm,
                             stk::mesh::BulkData* bulk_data,
                             stk::mesh::MetaData* meta_data,
                             std::string fieldname) : IVEMeshAPI()
{
  initialize();

  mComm = comm;
  mMetaData = meta_data;
  mBulkData = bulk_data;
  mCoordsField = mMetaData->get_field<stk::mesh::Field<double, stk::mesh::Cartesian> >
                            (stk::topology::NODE_RANK, "coordinates");
  if(!mCoordsField)
  {
    std::cout << "Failed to find nodal coordinate field." << std::endl;
  }
  mIsoField = mMetaData->get_field<stk::mesh::Field<double> >(stk::topology::NODE_RANK, fieldname);
  //mIsoField = mMetaData->get_field<stk::mesh::Field<double, stk::mesh::Cartesian3d> >(stk::topology::NODE_RANK, fieldname);
  if(!mIsoField)
  {
    std::cout << "Failed to find " << fieldname << " nodal variable." << std::endl;
  }
/*
  int num_states = mIsoField->state();
  printf("***********************************************************************\n");
  printf("Num time steps1: %d\n", num_states);
*/
  //mIsoField = mMetaData->get_field<stk::mesh::Field<double> >(stk::topology::NODE_RANK, fieldname);
  mIsoField = &(mIsoField->field_of_state(stk::mesh::StateNew));
  if(!mIsoField)
  {
    std::cout << "Failed to find " << fieldname << " nodal variable." << std::endl;
  }
/*
  printf("Num time steps2: %d\n", mIsoField->state());
  printf("***********************************************************************\n");
  mTimeStep = num_states;  
*/
}

IVEMeshAPISTK::IVEMeshAPISTK(stk::ParallelMachine* comm) : IVEMeshAPI()
{
  initialize();
  mComm = comm;
  mMetaData = new stk::mesh::MetaData(3);
  mLocallyOwnedMeta = true;
  mCoordsField = (stk::mesh::Field<double, stk::mesh::Cartesian>*)(&(mMetaData->
                declare_field<stk::mesh::Field<double, stk::mesh::Cartesian> >
                            (stk::topology::NODE_RANK, "coordinates")));
  if(!mCoordsField)
    std::cout << "Failed to find nodal coordinate field." << std::endl;
  stk::mesh::put_field_on_entire_mesh(*mCoordsField);
}

/*
IVEMeshAPISTK::IVEMeshAPISTK(stk::ParallelMachine* comm) : IVEMeshAPI()
{
  initialize();

  mComm = comm;
  mMetaData = new stk::mesh::MetaData(3);
  mLocallyOwnedMeta = true;
  mCoordsField = (stk::mesh::Field<double, stk::mesh::Cartesian>*)(&(mMetaData->
                declare_field<stk::mesh::Field<double, stk::mesh::Cartesian> >
                            (stk::topology::NODE_RANK, "coordinates")));
  stk::mesh::put_field_on_entire_mesh(*mCoordsField);
  prepare_to_create_tris();
  mMetaData->commit();
  if(!mCoordsField)
  {
    std::cout << "Failed to find nodal coordinate field." << std::endl;
  }
  mBulkData = new stk::mesh::BulkData(*mMetaData, *mComm);
  mLocallyOwnedBulk = true;
  mIoBroker = new stk::io::StkMeshIoBroker(*mComm);
  mIoBroker->set_bulk_data(*mBulkData);
}
*/

IVEMeshAPISTK::~IVEMeshAPISTK()
{
  if ( mLocallyOwnedBulk && mBulkData )
    delete mBulkData;
  if ( mLocallyOwnedMeta && mMetaData )
    delete mMetaData;
  if(mIoBroker)
    delete mIoBroker;
}

IVEHandle IVEMeshAPISTK::get_handle(const stk::mesh::Entity &entity) const
{
  return entity.m_value;
}

stk::mesh::Entity IVEMeshAPISTK::get_stk_entity(const IVEHandle &handle) const
{
  stk::mesh::Entity entity;
  entity.m_value = handle;
  return entity;
}

double IVEMeshAPISTK::get_nodal_iso_field_variable(IVEHandle node) const
{
  stk::mesh::Entity node_entity = get_stk_entity(node);
  double* vals = stk::mesh::field_data(*mIsoField, node_entity);
  return *vals;
}

int IVEMeshAPISTK::element_nodes(IVEHandle elem, IVEHandle nodes[8]) const
{
  stk::mesh::Entity e = get_stk_entity(elem);
  stk::mesh::Entity const *elem_nodes = mBulkData->begin_nodes(e);
  int num_nodes = mBulkData->num_nodes(e);
  for(int i=0; i<num_nodes; ++i)
    nodes[i] = get_handle(elem_nodes[i]);
  return num_nodes;
}

void IVEMeshAPISTK::hex_nodes(IVEHandle hex, IVEHandle nodes[8]) const
{
  stk::mesh::Entity e = get_stk_entity(hex);
  stk::mesh::Entity const *elem_nodes = mBulkData->begin_nodes(e);
  int num_nodes = mBulkData->num_nodes(e);
  for(int i=0; i<num_nodes; ++i)
    nodes[i] = get_handle(elem_nodes[i]);
}

void IVEMeshAPISTK::tet_nodes(IVEHandle tet, IVEHandle nodes[4]) const
{
  stk::mesh::Entity e = get_stk_entity(tet);
  stk::mesh::Entity const *elem_nodes = mBulkData->begin_nodes(e);
  int num_nodes = mBulkData->num_nodes(e);
  for(int i=0; i<num_nodes; ++i)
    nodes[i] = get_handle(elem_nodes[i]);
}

void IVEMeshAPISTK::hex_quad_nodes(IVEHandle hex, int index, IVEHandle nodes[4]) const
{
  stk::mesh::Entity stk_hex = get_stk_entity(hex);
  std::vector<stk::mesh::Entity> face_nodes(4);
  const stk::mesh::Entity* entity_nodes = mBulkData->begin_nodes(stk_hex);
  mBulkData->bucket(stk_hex).topology().face_nodes(entity_nodes, index, face_nodes.begin());
  for(int i=0; i<4; ++i)
    nodes[i] = get_handle(face_nodes[i]);
}

IVEHandle IVEMeshAPISTK::get_connected_hex(IVEHandle hex, IVEHandle n1, IVEHandle n2, IVEHandle n3, IVEHandle n4) const
{
  stk::mesh::Entity stk_hex = get_stk_entity(hex);
  stk::mesh::Entity stk_n1 = get_stk_entity(n1);
  stk::mesh::Entity stk_n2 = get_stk_entity(n2);
  stk::mesh::Entity stk_n3 = get_stk_entity(n3);
  stk::mesh::Entity stk_n4 = get_stk_entity(n4);
  stk::mesh::Entity other_nodes[3];
  other_nodes[0] = stk_n2;
  other_nodes[1] = stk_n3;
  other_nodes[2] = stk_n4;
  stk::mesh::Entity const *node_elems = mBulkData->begin_elements(stk_n1);
  int num_elems = mBulkData->num_elements(stk_n1);
  for(int i=0; i<num_elems; ++i)
  {
    stk::mesh::Entity cur_hex = node_elems[i];
    if(cur_hex != stk_hex)
    {
      stk::mesh::Entity const *hex_nodes = mBulkData->begin_nodes(cur_hex);
      int num_hex_nodes = mBulkData->num_nodes(cur_hex);
      bool found_all = true;
      for(int j=0; found_all && j<3; ++j)
      {    
        stk::mesh::Entity cur_node = other_nodes[j];
        int k;
        for(k=0; k<num_hex_nodes; ++k)
        {    
          if(cur_node == hex_nodes[k])
            break;
        }    
        if(k == num_hex_nodes)
          found_all = false;
      }
      if(found_all)
        return get_handle(cur_hex);
    }
  }
  return 0;
}

IsoVector IVEMeshAPISTK::node_coordinates( IVEHandle node ) const
{
  stk::mesh::Entity stk_node = get_stk_entity(node);
  double* vals = stk::mesh::field_data(*mCoordsField, stk_node);
  return IsoVector(vals[0], vals[1], vals[2]);
}

IVEHandle IVEMeshAPISTK::get_connected_tet(IVEHandle tet, IVEHandle n1, IVEHandle n2, IVEHandle n3) const
{
  stk::mesh::Entity stk_tet = get_stk_entity(tet);
  stk::mesh::Entity stk_n1 = get_stk_entity(n1);
  stk::mesh::Entity stk_n2 = get_stk_entity(n2);
  stk::mesh::Entity stk_n3 = get_stk_entity(n3);
  stk::mesh::Entity other_nodes[2];
  other_nodes[0] = stk_n2;
  other_nodes[1] = stk_n3;
  stk::mesh::Entity const *node_elems = mBulkData->begin_elements(stk_n1);
  int num_elems = mBulkData->num_elements(stk_n1);
  for(int i=0; i<num_elems; ++i)
  {
    stk::mesh::Entity cur_tet = node_elems[i];
    if(cur_tet != stk_tet)
    {
      stk::mesh::Entity const *tet_nodes = mBulkData->begin_nodes(cur_tet);
      int num_tet_nodes = mBulkData->num_nodes(cur_tet);
      bool found_all = true;
      for(int j=0; found_all && j<2; ++j)
      {    
        stk::mesh::Entity cur_node = other_nodes[j];
        int k;
        for(k=0; k<num_tet_nodes; ++k)
        {    
          if(cur_node == tet_nodes[k])
            break;
        }    
        if(k == num_tet_nodes)
          found_all = false;
      }
      if(found_all)
        return get_handle(cur_tet);
    }
  }
  return 0;
}

void IVEMeshAPISTK::calculate_average_edge_length_and_bbox(const std::vector<IVEHandle> &elem_list,
          double &minx, double &miny, double &minz, double &maxx, double &maxy, double &maxz, double &ave_length)
{
  double local_dist_squared_sum = 0.0;
  size_t local_num_elems = elem_list.size();
  minx = miny = minz = 1e8;
  maxx = maxy = maxz = -1e8;
  for(size_t i=0; i<local_num_elems; ++i)
  {
    IVEHandle tmp_list[8];
    size_t num_nodes = element_nodes(elem_list[i], tmp_list);
    IsoVector pos1 = node_coordinates(tmp_list[0]);
    IsoVector pos2 = node_coordinates(tmp_list[1]);
    local_dist_squared_sum += pos1.distance_between_squared(pos2);
    for(size_t j=0; j<num_nodes; ++j)
    {
      IsoVector pos = node_coordinates(tmp_list[j]);
      if(pos.x() < minx)
        minx = pos.x();
      if(pos.y() < miny)
        miny = pos.y();
      if(pos.z() < minz)
        minz = pos.z();
      if(pos.x() > maxx)
        maxx = pos.x();
      if(pos.y() > maxy)
        maxy = pos.y();
      if(pos.z() > maxz)
        maxz = pos.z();
    }
  }

  double global_dist_squared_sum;
  MPI_Allreduce(&local_dist_squared_sum, &global_dist_squared_sum, 1, 
                sierra::MPI::Datatype<double>::type(), MPI_SUM, mBulkData->parallel()); 

  size_t global_num_elems;
  MPI_Allreduce(&local_num_elems, &global_num_elems, 1, 
                sierra::MPI::Datatype<size_t>::type(), MPI_SUM, mBulkData->parallel()); 

  double global_ave_dist_squared = global_dist_squared_sum/(double)global_num_elems;
  ave_length = sqrt(global_ave_dist_squared);

  double mins[3], maxes[3], min_res[3], max_res[3];
  mins[0] = minx;
  mins[1] = miny;
  mins[2] = minz;
  maxes[0] = maxx;
  maxes[1] = maxy;
  maxes[2] = maxz;

  MPI_Allreduce(mins, min_res, 3, 
                sierra::MPI::Datatype<double>::type(), MPI_MIN, mBulkData->parallel()); 
  MPI_Allreduce(maxes, max_res, 3, 
                sierra::MPI::Datatype<double>::type(), MPI_MAX, mBulkData->parallel()); 
  minx = min_res[0];
  miny = min_res[1];
  minz = min_res[2];
  maxx = max_res[0];
  maxy = max_res[1];
  maxz = max_res[2];
}

IVEHandle IVEMeshAPISTK::new_tri(IVEHandle n1, IVEHandle n2, IVEHandle n3, bool is_fixed,
                                 IVEHandle source_elem)
{
 // stk::mesh::Entity stk_source_elem = get_stk_entity(source_elem);
 // IVEHandle source_global_id = mBulkData->identifier(stk_source_elem);
  stk::mesh::Entity stk_n1 = get_stk_entity(n1);
  stk::mesh::Entity stk_n2 = get_stk_entity(n2);
  stk::mesh::Entity stk_n3 = get_stk_entity(n3);
  stk::mesh::EntityId entity_id = get_next_entity_id(stk::topology::ELEM_RANK);  
  stk::mesh::Entity stk_tri;

  if(is_fixed)
    stk_tri = mBulkData->declare_element(entity_id, stk::mesh::ConstPartVector{mFixedTriPart});
  else
      stk_tri = mBulkData->declare_element(entity_id, stk::mesh::ConstPartVector{mOptimizedTriPart});

  mBulkData->declare_relation(stk_tri, stk_n1, 0);
  mBulkData->declare_relation(stk_tri, stk_n2, 1);
  mBulkData->declare_relation(stk_tri, stk_n3, 2);
  return get_handle(stk_tri);
}

IVEHandle IVEMeshAPISTK::get_new_node_id()
{
  return get_next_entity_id(stk::topology::NODE_RANK);  
}

void IVEMeshAPISTK::set_min_node_id(BoundaryNodeInfo &bni, const IVEHandle &n)
{
  bni.min_node = n;
  if(n != 0)
    bni.min_node_global_id = mBulkData->identifier(get_stk_entity(n));
  else
    bni.min_node_global_id = 0;
}

void IVEMeshAPISTK::set_max_node_id(BoundaryNodeInfo &bni, const IVEHandle &n)
{
  bni.max_node = n;
  if(n != 0)
    bni.max_node_global_id = mBulkData->identifier(get_stk_entity(n));
  else
    bni.max_node_global_id = 0;
}

void IVEMeshAPISTK::set_existing_node_id(DuplicateNodeInfo &dni, const IVEHandle &n)
{
  dni.existing_node_local_id = n;
  if(n != 0)
    dni.existing_node_global_id = mBulkData->identifier(get_stk_entity(n));
  else
    dni.existing_node_global_id = 0;
}

void IVEMeshAPISTK::batch_create_edge_boundary_nodes(std::vector<BoundaryNodeInfo> &boundary_info,
                       IVEMeshAPI *existing_mesh)
{
  IVEMeshAPISTK *existing_stk_mesh = (IVEMeshAPISTK*)existing_mesh;
  stk::mesh::BulkData *existing_bulk_data = existing_stk_mesh->bulk_data();
  std::vector<NewNodeData> NewNodeDatas;
  for (unsigned it_req=0; it_req<boundary_info.size(); ++it_req)
  {
    BoundaryNodeInfo &cur_info = boundary_info[it_req];
    stk::mesh::EntityId node1 = cur_info.min_node_global_id;
    stk::mesh::EntityId node2 = cur_info.max_node_global_id;
    NewNodeDatas.push_back(NewNodeData(*existing_bulk_data, *mBulkData, node1, node2));
    NewNodeDatas.back().add_proc_id_pair(existing_bulk_data->parallel_rank(), cur_info.suggested_new_node_id);
    NewNodeDatas.back().calculate_sharing_procs();
  }

  // sort both of our lists so they stay in sync
  std::sort(NewNodeDatas.begin(), NewNodeDatas.end(), NodeDataSorter());
  std::sort(boundary_info.begin(), boundary_info.end(), less_than_boundary_global_id());

  stk::CommSparse comm_spec(mBulkData->parallel());
  for (int phase=0;phase<2;++phase)
  {
    for (size_t edge_index=0;edge_index<NewNodeDatas.size();++edge_index)
    {
      for (size_t proc_index=0;proc_index<NewNodeDatas[edge_index].num_sharing_procs();++proc_index)
      {
        int other_proc = NewNodeDatas[edge_index].sharing_proc(proc_index);
        stk::mesh::EntityId node_1 = NewNodeDatas[edge_index].node1();
        stk::mesh::EntityId node_2 = NewNodeDatas[edge_index].node2();
        stk::mesh::EntityId this_procs_suggested_id = NewNodeDatas[edge_index].suggested_node_id();
        comm_spec.send_buffer(other_proc).pack<stk::mesh::EntityId>(node_1).pack<stk::mesh::EntityId>(node_2);
        comm_spec.send_buffer(other_proc).pack<stk::mesh::EntityId>(this_procs_suggested_id);
      }
    }
    if ( phase == 0 )
    {
      comm_spec.allocate_buffers();
    }
    else
    {
      comm_spec.communicate();
    }
  }

  for(int i = 0; i < mBulkData->parallel_size(); ++i)
  {
    if(i != mBulkData->parallel_rank())
    {
      while(comm_spec.recv_buffer(i).remaining())
      {
        stk::mesh::EntityId node1;
        stk::mesh::EntityId node2;
        stk::mesh::EntityId suggested_node_id;
        comm_spec.recv_buffer(i).unpack<stk::mesh::EntityId>(node1);
        comm_spec.recv_buffer(i).unpack<stk::mesh::EntityId>(node2);
        comm_spec.recv_buffer(i).unpack<stk::mesh::EntityId>(suggested_node_id);

        NewNodeData from_other_proc(*existing_bulk_data, *mBulkData, node1, node2);
        std::vector<NewNodeData>::iterator iter = std::lower_bound(NewNodeDatas.begin(), 
                                                                   NewNodeDatas.end(),
                                                                   from_other_proc, NodeDataSorter());

        if ( iter != NewNodeDatas.end() && *iter == from_other_proc)
        {
          iter->add_proc_id_pair(i, suggested_node_id);
        }
      }
    }
  }

  for (size_t edge_index=0;edge_index<NewNodeDatas.size();++edge_index)
  {
    NewNodeDatas[edge_index].set_node_entity(boundary_info[edge_index].new_node_pos, mCoordsField);
    boundary_info[edge_index].new_node = get_handle(NewNodeDatas[edge_index].m_new_node);
    existing_stk_mesh->transfer_output_fields(boundary_info[edge_index].min_node, 
                                              boundary_info[edge_index].max_node,
                                              boundary_info[edge_index].new_node,
                                              boundary_info[edge_index].mu,
                                              this); 
  }
}

void IVEMeshAPISTK::batch_create_duplicate_nodes(
                   std::vector<DuplicateNodeInfo> &dup_node_infos, 
                   IVEMeshAPI *existing_mesh)
{
  IVEMeshAPISTK *existing_stk_mesh = (IVEMeshAPISTK*)existing_mesh;
  stk::mesh::BulkData *existing_bulk_data = existing_stk_mesh->bulk_data();
  std::vector<NewNodeData> NewNodeDatas;
  for (unsigned it_req=0; it_req<dup_node_infos.size(); ++it_req)
  {
    DuplicateNodeInfo &cur_info = dup_node_infos[it_req];
    stk::mesh::EntityId node = cur_info.existing_node_global_id;
    NewNodeDatas.push_back(NewNodeData(*existing_bulk_data, *mBulkData, node, node));
    NewNodeDatas.back().add_proc_id_pair(existing_bulk_data->parallel_rank(), 
                                     cur_info.suggested_new_node_id);
    NewNodeDatas.back().calculate_sharing_procs();
  }

  // sort both of our lists so they stay in sync
  std::sort(NewNodeDatas.begin(), NewNodeDatas.end(), NodeDataSorter());
  std::sort(dup_node_infos.begin(), dup_node_infos.end(), 
              less_than_dup_node());

  stk::CommSparse comm_spec(mBulkData->parallel());
  for (int phase=0;phase<2;++phase)
  {
    for (size_t node_index=0;node_index<NewNodeDatas.size();++node_index)
    {
      for (size_t proc_index=0;proc_index<NewNodeDatas[node_index].num_sharing_procs();++proc_index)
      {
        int other_proc = NewNodeDatas[node_index].sharing_proc(proc_index);
        stk::mesh::EntityId node_1 = NewNodeDatas[node_index].node1();
        stk::mesh::EntityId this_procs_suggested_id = NewNodeDatas[node_index].suggested_node_id();
        comm_spec.send_buffer(other_proc).pack<stk::mesh::EntityId>(node_1);
        comm_spec.send_buffer(other_proc).pack<stk::mesh::EntityId>(this_procs_suggested_id);
      }
    }
    if ( phase == 0 )
    {
      comm_spec.allocate_buffers();
    }
    else
    {
      comm_spec.communicate();
    }
  }

  for(int i = 0; i < mBulkData->parallel_size(); ++i)
  {
    if(i != mBulkData->parallel_rank())
    {
      while(comm_spec.recv_buffer(i).remaining())
      {
        stk::mesh::EntityId node1;
        stk::mesh::EntityId suggested_node_id;
        comm_spec.recv_buffer(i).unpack<stk::mesh::EntityId>(node1);
        comm_spec.recv_buffer(i).unpack<stk::mesh::EntityId>(suggested_node_id);
        NewNodeData from_other_proc(*existing_bulk_data, *mBulkData, node1, node1);
        std::vector<NewNodeData>::iterator iter = std::lower_bound(
                    NewNodeDatas.begin(), NewNodeDatas.end(), from_other_proc, NodeDataSorter());

        if ( iter != NewNodeDatas.end() && *iter == from_other_proc)
        {
          iter->add_proc_id_pair(i, suggested_node_id);
        }
      }
    }
  }

  for (size_t node_index=0;node_index<NewNodeDatas.size();++node_index)
  {
    NewNodeDatas[node_index].set_node_entity(dup_node_infos[node_index].new_node_pos, 
                mCoordsField);
    dup_node_infos[node_index].new_node = get_handle(NewNodeDatas[node_index].m_new_node);
    // copy output field values from old node to new node
    existing_stk_mesh->copy_node_output_fields(dup_node_infos[node_index].existing_node_local_id, 
                                dup_node_infos[node_index].new_node, this);
  }
}

stk::mesh::Field<double>* IVEMeshAPISTK::get_nodal_field(std::string &name)
{
    for(size_t i=0; i<mNodalFields.size(); ++i)
    {
        if(name == mNodalFields[i]->name())
            return mNodalFields[i];
    }
    return NULL;
}

stk::mesh::Field<double>* IVEMeshAPISTK::get_element_field(std::string &name)
{
    for(size_t i=0; i<mElementFields.size(); ++i)
    {
        if(name == mElementFields[i]->name())
            return mElementFields[i];
    }
    return NULL;
}

void IVEMeshAPISTK::copy_node_output_fields(IVEHandle n1, IVEHandle new_node, IVEMeshAPI *output_mesh_api)
{
    IVEMeshAPISTK *out_api = (IVEMeshAPISTK*)output_mesh_api;
    for(size_t i=0; i<mNodalFields.size(); ++i)
    {
        stk::mesh::Field<double> *cur_field = mNodalFields[i];
        stk::mesh::EntityRank topology_rank = cur_field->entity_rank();
        if(topology_rank == stk::topology::NODE_RANK)
        {
            std::string name = cur_field->name();
            stk::mesh::FieldRestrictionVector rv = cur_field->restrictions();
            int num_per_node = rv[0].num_scalars_per_entity();
            stk::mesh::Entity stk_n1 = get_stk_entity(n1);
            double* vals1 = stk::mesh::field_data(*cur_field, stk_n1);
            if(this == output_mesh_api)
            {
                stk::mesh::Entity stk_new_node = get_stk_entity(new_node);
                double* vals3 = stk::mesh::field_data(*cur_field, stk_new_node);
                for(int j=0; j<num_per_node; ++j)
                    vals3[j] = vals1[j];
            }
            else
            {
                stk::mesh::Field<double> *out_field = out_api->get_nodal_field(name);
                stk::mesh::Entity stk_new_node = out_api->get_stk_entity(new_node);
                double* vals3 = stk::mesh::field_data(*out_field, stk_new_node);
                for(int j=0; j<num_per_node; ++j)
                    vals3[j] = vals1[j];
            }
        }
    }
}

void IVEMeshAPISTK::copy_element_output_fields(IVEHandle e1, IVEHandle e2, 
                                       IVEMeshAPI *output_mesh_api)
{
    IVEMeshAPISTK *out_api = (IVEMeshAPISTK*)output_mesh_api;
    for(size_t i=0; i<mElementFields.size(); ++i)
    {
        stk::mesh::Field<double> *cur_field = mElementFields[i];
        stk::mesh::EntityRank topology_rank = cur_field->entity_rank();
        if(topology_rank == stk::topology::ELEMENT_RANK)
        {
            std::string name = cur_field->name();
            stk::mesh::FieldRestrictionVector rv = cur_field->restrictions();
            int num_per_elem = rv[0].num_scalars_per_entity();
            stk::mesh::Entity stk_e1 = get_stk_entity(e1);
            double* vals1 = stk::mesh::field_data(*cur_field, stk_e1);
            if(this == output_mesh_api)
            {
                stk::mesh::Entity stk_e2 = get_stk_entity(e2);
                double* vals2 = stk::mesh::field_data(*cur_field, stk_e2);
                for(int j=0; j<num_per_elem; ++j)
                    vals2[j] = vals1[j];
            }
            else
            {
                stk::mesh::Field<double> *out_field = out_api->get_element_field(name);
                stk::mesh::Entity stk_e2 = out_api->get_stk_entity(e2);
                double* vals2 = stk::mesh::field_data(*out_field, stk_e2);
                for(int j=0; j<num_per_elem; ++j)
                    vals2[j] = vals1[j];
            }
        }
    }
}

void IVEMeshAPISTK::transfer_output_fields(IVEHandle n1, IVEHandle n2, IVEHandle new_node,
                                           double mu, IVEMeshAPI *output_mesh_api)
{
    IVEMeshAPISTK *out_api = (IVEMeshAPISTK*)output_mesh_api;
    for(size_t i=0; i<mNodalFields.size(); ++i)
    {
        stk::mesh::Field<double> *cur_field = mNodalFields[i];
        stk::mesh::EntityRank topology_rank = cur_field->entity_rank();
        if(topology_rank == stk::topology::NODE_RANK)
        {
            std::string name = cur_field->name();
            stk::mesh::FieldRestrictionVector rv = cur_field->restrictions();
            int num_per_node = rv[0].num_scalars_per_entity();
            stk::mesh::Entity stk_n1 = get_stk_entity(n1);
            stk::mesh::Entity stk_n2 = get_stk_entity(n2);
            double* vals1 = stk::mesh::field_data(*cur_field, stk_n1);
            double* vals2 = stk::mesh::field_data(*cur_field, stk_n2);
            if(this == output_mesh_api)
            {
                stk::mesh::Entity stk_new_node = get_stk_entity(new_node);
                double* vals3 = stk::mesh::field_data(*cur_field, stk_new_node);
                for(int j=0; j<num_per_node; ++j)
                    vals3[j] = vals1[j] + mu * (vals2[j] - vals1[j]);
            }
            else
            {
                stk::mesh::Field<double> *out_field = out_api->get_nodal_field(name);
                stk::mesh::Entity stk_new_node = out_api->get_stk_entity(new_node);
                double* vals3 = stk::mesh::field_data(*out_field, stk_new_node);
                for(int j=0; j<num_per_node; ++j)
                    vals3[j] = vals1[j] + mu * (vals2[j] - vals1[j]);
            }
        }
    }
}

IVEHandle IVEMeshAPISTK::new_node(IsoVector &coordinates)
{
  stk::mesh::EntityId entity_id = get_next_entity_id(stk::topology::NODE_RANK);  
  stk::mesh::Entity stk_node = mBulkData->declare_node(entity_id);
  double* node_coords = stk::mesh::field_data(*mCoordsField, stk_node);
  node_coords[0] = coordinates.x();
  node_coords[1] = coordinates.y();
  node_coords[2] = coordinates.z();
  return get_handle(stk_node);
}

void IVEMeshAPISTK::reserve_new_node_ids(uint64_t num_requested)
{
  mAvailableNodeIds.clear();
  mBulkData->generate_new_ids(stk::topology::NODE_RANK, num_requested, mAvailableNodeIds);
  mNextAvailableNodeIndex = 0;
}

void IVEMeshAPISTK::reserve_new_tri_ids(uint64_t num_requested)
{
  mAvailableElemIds.clear();
  mBulkData->generate_new_ids(stk::topology::ELEM_RANK, num_requested, mAvailableElemIds);
  mNextAvailableElemIndex = 0;
}

stk::mesh::EntityId IVEMeshAPISTK::get_next_entity_id(stk::topology::rank_t rank)
{
  if(rank == stk::topology::NODE_RANK)
  {
    if(mNextAvailableNodeIndex == mAvailableNodeIds.size())
    {
      std::cout << "Error: Ran out of node ids!\n" << std::endl;
    }
    stk::mesh::EntityId next_id = mAvailableNodeIds[mNextAvailableNodeIndex];
    mNextAvailableNodeIndex++;
    return next_id;
  }
  else if(rank == stk::topology::ELEM_RANK)
  {
    if(mNextAvailableElemIndex == mAvailableElemIds.size())
    {
      std::cout << "Error: Ran out of tri ids!\n" << std::endl;
    }
    stk::mesh::EntityId next_id = mAvailableElemIds[mNextAvailableElemIndex];
    mNextAvailableElemIndex++;
    return next_id;
  }
  return 0;
}

void IVEMeshAPISTK::store_tri_to_tet_map_entry(const IVEHandle &tri, const IVEHandle &tet)
{
/*
  CubitTri *ctri = CubitTri::find(tri);
  CubitTet *ctet = CubitTet::find(tet);
  IsoSurfaceTool::tri_to_tet_map[ctri] = ctet;
*/
}

void IVEMeshAPISTK::store_tet_to_tri_map_entry(const IVEHandle &tet, const IVEHandle &tri)
{
/*
  CubitTri *ctri = CubitTri::find(tri);
  CubitTet *ctet = CubitTet::find(tet);
  IsoSurfaceTool::tet_to_tri_map[ctet].push_back(ctri);
*/
}

bool IVEMeshAPISTK::read_exodus_mesh( std::string &meshfile, std::string &fieldname,
                                      std::string &outputFieldsString,
                                      int input_file_is_spread,
                                      int time_step )
{
  mIoBroker->set_option_to_not_collapse_sequenced_fields();
  if(!input_file_is_spread)
    mIoBroker->property_add(Ioss::Property("DECOMPOSITION_METHOD", "RIB"));
  mIoBroker->property_add(Ioss::Property("MAXIMUM_NAME_LENGTH", 256));
  mIoBroker->add_mesh_database(meshfile, "exodus", stk::io::READ_MESH);
  mIoBroker->create_input_mesh();

  mIoBroker->add_all_mesh_fields_as_input_fields();

  get_output_fields(outputFieldsString);

  prepare_to_create_tris();

  mIoBroker->populate_bulk_data();

  mCoordsField = mMetaData->get_field<stk::mesh::Field<double, stk::mesh::Cartesian> >
                            (stk::topology::NODE_RANK, "coordinates");

  if(!mCoordsField)
  {
    std::cout << "Failed to find nodal coordinate field." << std::endl;
    return false;
  }

  int debug_print = 0;
  if(debug_print)
  {
    export_my_mesh();
    return false;
  }

  mIsoField = mMetaData->get_field<stk::mesh::Field<double> >(stk::topology::NODE_RANK, fieldname);
  if(!mIsoField)
  {
    std::cout << "Failed to find " << fieldname << " nodal variable." << std::endl;
    return false;
  }

  if(time_step == -1)
    time_step = mIoBroker->get_input_io_region().get()->get_property("state_count").get_int();
  mIoBroker->read_defined_input_fields(time_step);

  return true;
}

void IVEMeshAPISTK::export_my_mesh()
{
  int p_rank = mBulkData->parallel_rank();
  char filename[1000];
  sprintf(filename, "mesh_proc_%d.txt", p_rank);
  FILE *fp = fopen(filename, "w");
  if(fp)
  {
    fprintf(fp, "Owned Nodes\n");
    stk::mesh::Selector sel(mMetaData->locally_owned_part());
    const stk::mesh::BucketVector &node_buckets = mBulkData->get_buckets(stk::topology::NODE_RANK, sel);
    for(size_t i=0; i<node_buckets.size(); ++i)
    {
      for(size_t j=0; j<node_buckets[i]->size(); ++j)
      {
        stk::mesh::Entity cur_node = (*(node_buckets[i]))[j];
        double* vals = stk::mesh::field_data(*mCoordsField, cur_node);
        unsigned long id = mBulkData->identifier(cur_node);
        fprintf(fp, "%lu %lf %lf %lf\n", id, vals[0], vals[1], vals[2]);
      }
    }
    fprintf(fp, "Shared Nodes\n");
    stk::mesh::Selector sel2(mMetaData->globally_shared_part());
    const stk::mesh::BucketVector &node_buckets2 = mBulkData->get_buckets(stk::topology::NODE_RANK, sel2);
    for(size_t i=0; i<node_buckets2.size(); ++i)
    {
      for(size_t j=0; j<node_buckets2[i]->size(); ++j)
      {
        stk::mesh::Entity cur_node = (*(node_buckets2[i]))[j];
        double* vals = stk::mesh::field_data(*mCoordsField, cur_node);
        unsigned long id = mBulkData->identifier(cur_node);
        fprintf(fp, "%lu %lf %lf %lf\n", id, vals[0], vals[1], vals[2]);
      }
    }
    fclose(fp);
  }
}

void IVEMeshAPISTK::print_boundary_node_info(std::vector<BoundaryNodeInfo> &bni)
{
  int p_rank = mBulkData->parallel_rank();
  char filename[1000];
  sprintf(filename, "boundary_node_info_%d.txt", p_rank);
  FILE *fp = fopen(filename, "w");
  if(fp)
  {
    for(size_t i=0; i<bni.size(); ++i)
    {
      fprintf(fp, "Boundary Info %lu:\n", i+1);
      unsigned long id = mBulkData->identifier(get_stk_entity(bni[i].min_node));
      fprintf(fp, "Min Node: %lu\n", id);
      id = mBulkData->identifier(get_stk_entity(bni[i].max_node));
      fprintf(fp, "Max Node: %lu\n", id);
      id = mBulkData->identifier(get_stk_entity(bni[i].new_node));
      fprintf(fp, "New Node: %lu\n", id);
    }
    fclose(fp);
  }
}

void IVEMeshAPISTK::add_nodal_field(std::string &name)
{
    stk::mesh::Field<double> *cur_field = &mMetaData->declare_field<stk::mesh::Field<double> >(stk::topology::NODE_RANK, name, 1);
    stk::mesh::put_field_on_entire_mesh(*cur_field);
//    stk::io::set_field_role(*cur_field, Ioss::Field::ATTRIBUTE);
    mNodalFields.push_back((stk::mesh::Field<double>*)cur_field);
}

void IVEMeshAPISTK::add_element_field(std::string &name)
{
    stk::mesh::Field<double> *cur_field = &mMetaData->declare_field<stk::mesh::Field<double> >(stk::topology::ELEMENT_RANK, name, 1);
    stk::mesh::put_field_on_entire_mesh(*cur_field);
//    stk::io::set_field_role(*cur_field, Ioss::Field::ATTRIBUTE);
    mElementFields.push_back((stk::mesh::Field<double>*)cur_field);
}

void IVEMeshAPISTK::get_output_fields(std::string &outputFieldsString)
{
  std::string working_string = outputFieldsString; // make a copy since we will be modifying it
  std::vector<std::string> parsed_strings;
  size_t comma_pos = working_string.find(',');
  while(comma_pos != std::string::npos)
  {
    std::string cur_string = working_string.substr(0,comma_pos);
    working_string = working_string.substr(comma_pos+1);
    parsed_strings.push_back(cur_string);
    comma_pos = working_string.find(',');
  }
  parsed_strings.push_back(working_string);
  mNodalFields.clear();
  mElementFields.clear();
  stk::mesh::FieldVector all_fields = mMetaData->get_fields();
  for(size_t i=0; i<parsed_strings.size(); i++)
  {
    for(size_t j=0; j<all_fields.size(); ++j)
    {
      stk::mesh::FieldBase* cur_file_field = all_fields[j];
      std::string file_field_name = cur_file_field->name();
      if(parsed_strings[i] == file_field_name)
      {
        if(cur_file_field->type_is<double>())
        {
          if(cur_file_field->entity_rank() == stk::topology::NODE_RANK)
          { 
            mNodalFields.push_back((stk::mesh::Field<double>*)cur_file_field);
            j = all_fields.size(); // break out
          }
          else if(cur_file_field->entity_rank() == stk::topology::ELEMENT_RANK)
          { 
            mElementFields.push_back((stk::mesh::Field<double>*)cur_file_field);
            j = all_fields.size(); // break out
          }
        }
      }
    }
  }
  if((mNodalFields.size() + mElementFields.size()) != parsed_strings.size())
  {
//    std::cout << "Error: Could not find all of the requested output fields!\n" << std::endl;
  }
}

void IVEMeshAPISTK::prepare_field_data(std::string &outputFieldsString, IVEMeshAPISTK *outAPI)
{
  std::string working_string = outputFieldsString; // make a copy since we will be modifying it
  std::vector<std::string> parsed_strings;
  size_t comma_pos = working_string.find(',');
  while(comma_pos != std::string::npos)
  {
    std::string cur_string = working_string.substr(0,comma_pos);
    working_string = working_string.substr(comma_pos+1);
    parsed_strings.push_back(cur_string);
    comma_pos = working_string.find(',');
  }
  parsed_strings.push_back(working_string);
  mNodalFields.clear();
  mElementFields.clear();
  stk::mesh::FieldVector all_fields = mMetaData->get_fields();
  for(size_t i=0; i<parsed_strings.size(); i++)
  {
    for(size_t j=0; j<all_fields.size(); ++j)
    {
      stk::mesh::FieldBase* cur_file_field = all_fields[j];
      std::string file_field_name = cur_file_field->name();
      if(parsed_strings[i] == file_field_name)
      {
        if(cur_file_field->type_is<double>())
        {
          if(cur_file_field->entity_rank() == stk::topology::NODE_RANK)
          {
            mNodalFields.push_back((stk::mesh::Field<double>*)cur_file_field);
            j = all_fields.size(); // break out
            if(outAPI)
            {
                outAPI->add_nodal_field(file_field_name);
            }
          }
          else if(cur_file_field->entity_rank() == stk::topology::ELEMENT_RANK)
          {
            mElementFields.push_back((stk::mesh::Field<double>*)cur_file_field);
            j = all_fields.size(); // break out
            if(outAPI)
            {
                outAPI->add_element_field(file_field_name);
            }
          }
        }
      }
    }
  }
  if((mNodalFields.size() + mElementFields.size()) != parsed_strings.size())
  {
//    std::cout << "Error: Could not find all of the requested output fields!\n" << std::endl;
  }
  if(outAPI)
      outAPI->prepare_as_destination();
}

void convert_block_names_to_parts(const stk::mesh::MetaData& meta,
                                  const std::vector<std::string>& blocks,
                                  stk::mesh::PartVector& blockParts)
{
  blockParts.clear();
  for(const std::string& block : blocks)
  {
    std::string blockName = "block_" + block;
    stk::mesh::Part* partPtr = meta.get_part(blockName);
    if (partPtr != nullptr)
    { 
      blockParts.push_back(partPtr);
    }
    else
      throw std::runtime_error("Could not find STK part for the block named: " + blockName);
  }
}

void IVEMeshAPISTK::get_fixed_block_nodes(std::vector<IVEHandle> &fixed_block_nodes)
{
  fixed_block_nodes.clear();
  if(mFixedBlocks.size() > 0)
  {
    stk::mesh::PartVector blockParts;
    convert_block_names_to_parts(*mMetaData, mFixedBlocks, blockParts);
    stk::mesh::Selector sel = stk::mesh::selectUnion(blockParts);
    const stk::mesh::BucketVector& fixed_node_buckets = mBulkData->get_buckets(stk::topology::NODE_RANK, sel);
    for(const stk::mesh::Bucket* bucket : fixed_node_buckets)
    {
      for(stk::mesh::Entity node : *bucket)
      {
        fixed_block_nodes.push_back(get_handle(node));
      }
    }
  }
  // uniquify the list
  std::sort(fixed_block_nodes.begin(), fixed_block_nodes.end());
  std::vector<IVEHandle>::iterator it = std::unique(fixed_block_nodes.begin(),fixed_block_nodes.end());
  fixed_block_nodes.resize(it-fixed_block_nodes.begin());
}

void IVEMeshAPISTK::set_fixed_block_ids(std::string fixed_block_string)
{
  mFixedBlocks.clear();
  if(fixed_block_string.length() > 0)
  {
    std::vector<std::string> parsed_strings;
    size_t comma_pos = fixed_block_string.find(',');
    while(comma_pos != std::string::npos)
    {
      std::string cur_string = fixed_block_string.substr(0,comma_pos);
      fixed_block_string = fixed_block_string.substr(comma_pos+1);
      mFixedBlocks.push_back(cur_string);
      comma_pos = fixed_block_string.find(',');
    }
    mFixedBlocks.push_back(fixed_block_string);
  }
}

void IVEMeshAPISTK::prepare_to_create_tris()
{
  mFixedTriPart = &mMetaData->declare_part_with_topology( "FixedTriangles", stk::topology::SHELL_TRI_3 );
  stk::io::put_io_part_attribute(*mFixedTriPart);


  mFixedTriMap = &mMetaData->declare_field<stk::mesh::Field<double> >(stk::topology::ELEMENT_RANK, "FixedTriMap", 1);
  stk::mesh::put_field_on_mesh(*mFixedTriMap, *mFixedTriPart, nullptr);
  stk::io::set_field_role(*mFixedTriMap, Ioss::Field::ATTRIBUTE);

  mOptimizedTriPart = &mMetaData->declare_part_with_topology( "OptimizedTriangles", stk::topology::SHELL_TRI_3 );
  stk::io::put_io_part_attribute(*mOptimizedTriPart);

  mOptimizedTriMap = &mMetaData->declare_field<stk::mesh::Field<double> >(stk::topology::ELEMENT_RANK, "OptimizedTriMap", 1);
  stk::mesh::put_field_on_mesh(*mOptimizedTriMap, *mOptimizedTriPart, nullptr);
  stk::io::set_field_role(*mOptimizedTriMap, Ioss::Field::ATTRIBUTE);

  for(size_t i=0; i<mElementFields.size(); ++i)
  {
    stk::mesh::put_field_on_mesh(*mElementFields[i], *mFixedTriPart, nullptr);
    stk::mesh::put_field_on_mesh(*mElementFields[i], *mOptimizedTriPart, nullptr);
  }
}

void IVEMeshAPISTK::write_exodus_mesh( std::string &meshfile, int output_method, int iso_only )
{
  if(mTimeStep > 0 && mBulkData->parallel_rank() == 0)
  {
    printf("\n########################################################\n");
    printf("Writing results for time step %d.\n", mTimeStep);
    printf("########################################################\n\n");
  }

  if(output_method == 1)  // use Ioss to do concatenation
    mIoBroker->property_add(Ioss::Property("COMPOSE_RESULTS", true));
  mIoBroker->property_add(Ioss::Property("MAXIMUM_NAME_LENGTH", 256));
  size_t fh = mIoBroker->create_output_mesh(meshfile, stk::io::WRITE_RESULTS);
  stk::mesh::Selector sel;
  if(mFixedTriPart != nullptr)
  {
    sel = *mFixedTriPart;
    if(mOptimizedTriPart != nullptr)
      sel |= *mOptimizedTriPart;
    else
      throw std::runtime_error("Could not find STK part for the optimized triangles.");
  }
  else
  {      
    throw std::runtime_error("Could not find STK part for the fixed triangles.");
  }
  if(iso_only)
  {
    mIoBroker->set_subset_selector(fh, sel);
    const stk::mesh::PartVector part_vec = mMetaData->get_mesh_parts();
    for(size_t i=0; i<part_vec.size(); ++i)
    {
      stk::mesh::Part *cur_part = part_vec[i];
      if(cur_part != mFixedTriPart && cur_part != mOptimizedTriPart)
      {
        stk::io::remove_io_part_attribute(*cur_part);
      }
    }
    for(size_t i=0; i<mNodalFields.size(); ++i)
    {
      mIoBroker->add_field(fh, *(mNodalFields[i]));
    }
    for(size_t i=0; i<mElementFields.size(); ++i)
    {
      mIoBroker->add_field(fh, *(mElementFields[i]));
    }
    mIoBroker->begin_output_step(fh, 0);
    mIoBroker->write_defined_output_fields(fh);
    mIoBroker->end_output_step(fh);
  }
  else
  {
    for(size_t i=0; i<mNodalFields.size(); ++i)
    {
      mIoBroker->add_field(fh, *(mNodalFields[i]));
    }
    for(size_t i=0; i<mElementFields.size(); ++i)
    {
      mIoBroker->add_field(fh, *(mElementFields[i]));
    }
    mIoBroker->add_field(fh, *mIsoField);
    mIoBroker->begin_output_step(fh, 0);
    mIoBroker->write_defined_output_fields(fh);
    mIoBroker->end_output_step(fh);
  }
  mIoBroker->write_output_mesh(fh);
}

void IVEMeshAPISTK::get_shared_boundary_nodes(std::set<IVEHandle> &shared_boundary_nodes)
{
  stk::mesh::Selector shared(mMetaData->globally_shared_part());
  const stk::mesh::BucketVector& shared_node_buckets = mBulkData->get_buckets(stk::topology::NODE_RANK,
                                                   shared);
  for(size_t i=0; i<shared_node_buckets.size(); ++i)
  {
    for(size_t j=0; j<shared_node_buckets[i]->size(); ++j)
    {
      stk::mesh::Entity cur_node = (*(shared_node_buckets[i]))[j];
      shared_boundary_nodes.insert(get_handle(cur_node));
    }
  }
}

void IVEMeshAPISTK::get_element_buckets(stk::mesh::Selector& sel, stk::mesh::BucketVector &buckets)
{
  const stk::mesh::BucketVector &elem_buckets = mBulkData->get_buckets(stk::topology::ELEMENT_RANK, sel);

  for ( stk::mesh::BucketVector::const_iterator bucket_iter = elem_buckets.begin();
        bucket_iter != elem_buckets.end();
        ++bucket_iter )
  {
    stk::mesh::Bucket &tmp_bucket = **bucket_iter;
    stk::topology bucket_top = tmp_bucket.topology();
    if(bucket_top == stk::topology::TET_4 ||
       bucket_top == stk::topology::HEX_8)
    {
      buckets.push_back(&tmp_bucket);
    }
  }
}
 
void IVEMeshAPISTK::get_attached_elements(const std::set<IVEHandle> &nodes,
                                          std::vector<IVEHandle> &attached_elements)
{
  // get a set of locally owned elements for filtering the attached elements we return
  std::set<stk::mesh::Entity> owned_elems;
  stk::mesh::Selector sel(mMetaData->locally_owned_part());
  stk::mesh::BucketVector elem_buckets;
  this->get_element_buckets(sel, elem_buckets);

  for(size_t i=0; i<elem_buckets.size(); ++i)
  {
    for(size_t j=0; j<elem_buckets[i]->size(); ++j)
    {
      stk::mesh::Entity cur_elem = (*(elem_buckets[i]))[j];
      owned_elems.insert(cur_elem);
    }
  }

  std::set<IVEHandle>::iterator it = nodes.begin();
  while(it != nodes.end())
  {
    stk::mesh::Entity cur_node = get_stk_entity(*it);
    stk::mesh::Entity const *node_elems = mBulkData->begin_elements(cur_node);
    int num_elems = mBulkData->num_elements(cur_node);
    for(int j=0; j<num_elems; ++j)
    {
      stk::mesh::Entity cur_elem = node_elems[j];
      if(owned_elems.find(cur_elem) != owned_elems.end())
        attached_elements.push_back(get_handle(node_elems[j]));
    }
    ++it;
  }
  std::sort(attached_elements.begin(), attached_elements.end());
  std::vector<IVEHandle>::iterator it2 = std::unique(attached_elements.begin(),attached_elements.end());
  attached_elements.resize(it2-attached_elements.begin());
}

bool IVEMeshAPISTK::prepare_as_source()
{
  if(!mMetaData)
  {
    mMetaData = new stk::mesh::MetaData;
    mLocallyOwnedMeta = true;
  }
  mBulkData = new stk::mesh::BulkData(*mMetaData, *mComm);
  mLocallyOwnedBulk = true;
  mIoBroker = new stk::io::StkMeshIoBroker(*mComm);
  mIoBroker->set_bulk_data(*mBulkData);
  return true;
}

bool IVEMeshAPISTK::prepare_as_destination()
{
  prepare_to_create_tris();
  mMetaData->commit();
  mBulkData = new stk::mesh::BulkData(*mMetaData, *mComm);
  mLocallyOwnedBulk = true;
  mIoBroker = new stk::io::StkMeshIoBroker(*mComm);
  mIoBroker->set_bulk_data(*mBulkData);
  return true;
}






