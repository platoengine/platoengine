//-------------------------------------------------------------------------
// Filename      : PruneMeshAPISTK.cpp
//
// Description   : a PruneMeshAPISTK is an implementation of the PruneMeshAPI
//
// Creator       : Brett Clark
//
// Creation Date : 5/6/2016
//
// Owner         : Brett Clark
//-------------------------------------------------------------------------

#include "PruneMeshAPISTK.hpp"
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
    PruneHandle min1, max1, min2, max2;
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

void PruneMeshAPISTK::initialize()
{
  mMetaData = NULL;
  mBulkData = NULL;
  mIoBroker = NULL;
  mLocallyOwnedBulk = false;
  mLocallyOwnedMeta = false;
  mFixedTriMap = NULL;
  mOptimizedTriMap = NULL;
  mCoordsField = NULL;
  mNextAvailableElemIndex = 0;
  mNextAvailableNodeIndex = 0;
  mTimeStep = 0;
}

PruneMeshAPISTK::PruneMeshAPISTK(stk::ParallelMachine* comm,
                             stk::mesh::BulkData* bulk_data,
                             stk::mesh::MetaData* meta_data,
                             std::string fieldname) : PruneMeshAPI()
{
  initialize();

  mComm = comm;
  mMetaData = meta_data;
  mBulkData = bulk_data;
  mCoordsField = mMetaData->get_field<stk::mesh::Field<double, stk::mesh::Cartesian> >
                            (stk::topology::NODE_RANK, "coordinates");
  if(!mCoordsField)
  {
    if(mBulkData && mBulkData->parallel_rank() == 0)
      std::cout << "Failed to find nodal coordinate field." << std::endl;
  }

  // The fieldname string can contain multiple comma-separated names
  std::string working_string = fieldname; // make a copy since we will be modifying it
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
  for(size_t i=0; i<parsed_strings.size(); ++i)
  {
    stk::mesh::Field<double> *cur_field = mMetaData->get_field<stk::mesh::Field<double> >(
                              stk::topology::NODE_RANK, parsed_strings[i]);
    if(!cur_field)
    {
      if(mBulkData && mBulkData->parallel_rank() == 0)
        std::cout << "Failed to find " << parsed_strings[i] << " nodal variable." << std::endl;
    }
    else
    {
      cur_field = &(cur_field->field_of_state(stk::mesh::StateNew));
      if(!cur_field)
      {
        if(mBulkData && mBulkData->parallel_rank() == 0)
          std::cout << "Failed to find " << parsed_strings[i] << " nodal variable." << std::endl;
      }
      else
      {
        mIsoFields.push_back(cur_field);
      }
    }
  }
}

PruneMeshAPISTK::PruneMeshAPISTK(stk::ParallelMachine* comm) : PruneMeshAPI()
{
  initialize();
  mComm = comm;
}

/*
PruneMeshAPISTK::PruneMeshAPISTK(stk::ParallelMachine* comm) : PruneMeshAPI()
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

PruneMeshAPISTK::~PruneMeshAPISTK()
{
  if ( mLocallyOwnedBulk && mBulkData )
    delete mBulkData;
  if ( mLocallyOwnedMeta && mMetaData )
    delete mMetaData;
  if(mIoBroker)
    delete mIoBroker;
}

PruneHandle PruneMeshAPISTK::get_handle(const stk::mesh::Entity &entity) const
{
  return entity.m_value;
}

stk::mesh::Entity PruneMeshAPISTK::get_stk_entity(const PruneHandle &handle) const
{
  stk::mesh::Entity entity;
  entity.m_value = handle;
  return entity;
}

double PruneMeshAPISTK::get_max_nodal_iso_field_variable(PruneHandle node) const
{
  double max_val = 0.0;
  stk::mesh::Entity node_entity = get_stk_entity(node);
  for(size_t i=0; i<mIsoFields.size(); ++i)
  {
    double* vals = stk::mesh::field_data(*(mIsoFields[i]), node_entity);
    if(*vals > max_val)
      max_val = *vals;
  }
  return max_val;
}

int PruneMeshAPISTK::element_nodes(PruneHandle elem, PruneHandle nodes[8]) const
{
  stk::mesh::Entity e = get_stk_entity(elem);
  stk::mesh::Entity const *elem_nodes = mBulkData->begin_nodes(e);
  int num_nodes = mBulkData->num_nodes(e);
  for(int i=0; i<num_nodes; ++i)
    nodes[i] = get_handle(elem_nodes[i]);
  return num_nodes;
}

void PruneMeshAPISTK::hex_nodes(PruneHandle hex, PruneHandle nodes[8]) const
{
  stk::mesh::Entity e = get_stk_entity(hex);
  stk::mesh::Entity const *elem_nodes = mBulkData->begin_nodes(e);
  int num_nodes = mBulkData->num_nodes(e);
  for(int i=0; i<num_nodes; ++i)
    nodes[i] = get_handle(elem_nodes[i]);
}

void PruneMeshAPISTK::tet_nodes(PruneHandle tet, PruneHandle nodes[4]) const
{
  stk::mesh::Entity e = get_stk_entity(tet);
  stk::mesh::Entity const *elem_nodes = mBulkData->begin_nodes(e);
  int num_nodes = mBulkData->num_nodes(e);
  for(int i=0; i<num_nodes; ++i)
    nodes[i] = get_handle(elem_nodes[i]);
}

void PruneMeshAPISTK::hex_quad_nodes(PruneHandle hex, int index, PruneHandle nodes[4]) const
{
  stk::mesh::Entity stk_hex = get_stk_entity(hex);
  std::vector<stk::mesh::Entity> face_nodes(4);
  const stk::mesh::Entity* entity_nodes = mBulkData->begin_nodes(stk_hex);
  mBulkData->bucket(stk_hex).topology().face_nodes(entity_nodes, index, face_nodes.begin());
  for(int i=0; i<4; ++i)
    nodes[i] = get_handle(face_nodes[i]);
}

PruneHandle PruneMeshAPISTK::get_connected_hex(PruneHandle hex, PruneHandle n1, PruneHandle n2, PruneHandle n3, PruneHandle n4) const
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

int PruneMeshAPISTK::get_connected_hex(stk::mesh::Entity hex, 
                                                     stk::mesh::Entity n1, 
                                                     stk::mesh::Entity n2,  
                                                     stk::mesh::Entity n3, 
                                                     stk::mesh::Entity n4,
                                                     stk::mesh::Entity &connected_hex) const
{
  stk::mesh::Entity other_nodes[3];
  other_nodes[0] = n2;
  other_nodes[1] = n3;
  other_nodes[2] = n4;
  stk::mesh::Entity const *node_elems = mBulkData->begin_elements(n1);
  int num_elems = mBulkData->num_elements(n1);
  for(int i=0; i<num_elems; ++i)
  {
    stk::mesh::Entity cur_hex = node_elems[i];
    if(cur_hex != hex)
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
      {
        connected_hex = cur_hex;
        return 1;
      }
    }
  }
  return 0;
}

int PruneMeshAPISTK::get_connected_elem(stk::mesh::Entity elem, 
           std::vector<stk::mesh::Entity> &face_nodes,
           stk::mesh::Entity &connected_elem) const
{
  std::vector<stk::mesh::Entity> other_nodes;
  std::vector<stk::mesh::Entity>::iterator it = face_nodes.begin();
  ++it; // don't grab the first one
  while(it != face_nodes.end())
  {
    other_nodes.push_back(*it);
    ++it;
  }
  stk::mesh::Entity const *node_elems = mBulkData->begin_elements(face_nodes[0]);
  int num_elems = mBulkData->num_elements(face_nodes[0]);
  for(int i=0; i<num_elems; ++i)
  {
    stk::mesh::Entity cur_elem = node_elems[i];
    if(cur_elem != elem)
    {
      stk::mesh::Entity const *elem_nodes = mBulkData->begin_nodes(cur_elem);
      int num_elem_nodes = mBulkData->num_nodes(cur_elem);
      bool found_all = true;
      for(size_t j=0; found_all && j<other_nodes.size(); ++j)
      {    
        stk::mesh::Entity cur_node = other_nodes[j];
        int k;
        for(k=0; k<num_elem_nodes; ++k)
        {    
          if(cur_node == elem_nodes[k])
            break;
        }    
        if(k == num_elem_nodes)
          found_all = false;
      }
      if(found_all)
      {
        connected_elem = cur_elem;
        return 1;
      }
    }
  }
  return 0;
}

void PruneMeshAPISTK::node_coordinates( PruneHandle node,
                                        double &x,
                                        double &y,
                                        double &z ) 
{
  stk::mesh::Entity stk_node = get_stk_entity(node);
  double* vals = stk::mesh::field_data(*mCoordsField, stk_node);
  x = vals[0];
  y = vals[1];
  z = vals[2];
}

PruneHandle PruneMeshAPISTK::get_connected_tet(PruneHandle tet, PruneHandle n1, PruneHandle n2, PruneHandle n3) const
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

double PruneMeshAPISTK::calculate_average_edge_length(const std::vector<PruneHandle> &elem_list)
{
  double local_dist_squared_sum = 0.0;
  size_t local_num_elems = elem_list.size();
  double x1, y1, z1, x2, y2, z2;
  for(size_t i=0; i<local_num_elems; ++i)
  {
    PruneHandle tmp_list[8];
    element_nodes(elem_list[i], tmp_list);
    node_coordinates(tmp_list[0], x1, y1, z1);
    node_coordinates(tmp_list[1], x2, y2, z2);
    local_dist_squared_sum += (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2);
  }

  double global_dist_squared_sum;
  MPI_Allreduce(&local_dist_squared_sum, &global_dist_squared_sum, 1, 
                sierra::MPI::Datatype<double>::type(), MPI_SUM, mBulkData->parallel()); 

  size_t global_num_elems;
  MPI_Allreduce(&local_num_elems, &global_num_elems, 1, 
                sierra::MPI::Datatype<size_t>::type(), MPI_SUM, mBulkData->parallel()); 

  double global_ave_dist_squared = global_dist_squared_sum/(double)global_num_elems;
  return sqrt(global_ave_dist_squared);
}

void PruneMeshAPISTK::add_element_to_survivor_block(PruneHandle entity)
{
}

PruneHandle PruneMeshAPISTK::new_tri(PruneHandle n1, PruneHandle n2, PruneHandle n3, bool is_fixed,
                                 PruneHandle source_elem)
{
  return 0;
}

PruneHandle PruneMeshAPISTK::get_new_node_id()
{
  return get_next_entity_id(stk::topology::NODE_RANK);  
}

void PruneMeshAPISTK::set_min_node_id(BoundaryNodeInfo &bni, const PruneHandle &n)
{
  bni.min_node = n;
  if(n != 0)
    bni.min_node_global_id = mBulkData->identifier(get_stk_entity(n));
  else
    bni.min_node_global_id = 0;
}

void PruneMeshAPISTK::set_max_node_id(BoundaryNodeInfo &bni, const PruneHandle &n)
{
  bni.max_node = n;
  if(n != 0)
    bni.max_node_global_id = mBulkData->identifier(get_stk_entity(n));
  else
    bni.max_node_global_id = 0;
}

void PruneMeshAPISTK::set_existing_node_id(DuplicateNodeInfo &dni, const PruneHandle &n)
{
  dni.existing_node_local_id = n;
  if(n != 0)
    dni.existing_node_global_id = mBulkData->identifier(get_stk_entity(n));
  else
    dni.existing_node_global_id = 0;
}

void PruneMeshAPISTK::batch_create_edge_boundary_nodes(std::vector<BoundaryNodeInfo> &boundary_info,
                       PruneMeshAPI *existing_mesh)
{
  PruneMeshAPISTK *existing_stk_mesh = (PruneMeshAPISTK*)existing_mesh;
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
    NewNodeDatas[edge_index].set_node_entity(boundary_info[edge_index].new_node_pos[0],
                                             boundary_info[edge_index].new_node_pos[1],
                                             boundary_info[edge_index].new_node_pos[2], 
                                             mCoordsField);
    boundary_info[edge_index].new_node = get_handle(NewNodeDatas[edge_index].m_new_node);
    existing_stk_mesh->transfer_output_fields(boundary_info[edge_index].min_node, 
                                              boundary_info[edge_index].max_node,
                                              boundary_info[edge_index].new_node,
                                              boundary_info[edge_index].mu,
                                              this); 
  }
}

void PruneMeshAPISTK::batch_create_duplicate_nodes(
                   std::vector<DuplicateNodeInfo> &dup_node_infos, 
                   PruneMeshAPI *existing_mesh)
{
  PruneMeshAPISTK *existing_stk_mesh = (PruneMeshAPISTK*)existing_mesh;
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
    NewNodeDatas[node_index].set_node_entity(dup_node_infos[node_index].new_node_pos[0],
                                             dup_node_infos[node_index].new_node_pos[1],
                                             dup_node_infos[node_index].new_node_pos[2], 
                                             mCoordsField);
    dup_node_infos[node_index].new_node = get_handle(NewNodeDatas[node_index].m_new_node);
    // copy output field values from old node to new node
    existing_stk_mesh->copy_node_output_fields(dup_node_infos[node_index].existing_node_local_id, 
                                dup_node_infos[node_index].new_node, this);
  }
}

void PruneMeshAPISTK::copy_node_output_fields(PruneHandle n1, PruneHandle new_node, PruneMeshAPI *output_mesh_api)
{
  for(size_t i=0; i<mNodalFields.size(); ++i)
  {
    stk::mesh::Field<double> *cur_field = mNodalFields[i];
    std::string name = cur_field->name();
    stk::mesh::EntityRank topology_rank = cur_field->entity_rank();
    stk::mesh::FieldRestrictionVector rv = cur_field->restrictions();
    int num_per_node = rv[0].num_scalars_per_entity();
    if(topology_rank == stk::topology::NODE_RANK)
    {
      stk::mesh::Entity stk_n1 = get_stk_entity(n1);
      double* vals1 = stk::mesh::field_data(*cur_field, stk_n1);
      if(this == output_mesh_api)
      {
        stk::mesh::Entity stk_new_node = get_stk_entity(new_node);
        double* vals3 = stk::mesh::field_data(*cur_field, stk_new_node);
        for(int j=0; j<num_per_node; ++j)
          vals3[j] = vals1[j];
      }
    }
    else if(topology_rank == stk::topology::ELEMENT_RANK)
    {
    } 
  }
}

void PruneMeshAPISTK::copy_element_output_fields(PruneHandle e1, PruneHandle e2, 
                                       PruneMeshAPI *output_mesh_api)
{
  for(size_t i=0; i<mElementFields.size(); ++i)
  {
    stk::mesh::Field<double> *cur_field = mElementFields[i];
    std::string name = cur_field->name();
    stk::mesh::EntityRank topology_rank = cur_field->entity_rank();
    stk::mesh::FieldRestrictionVector rv = cur_field->restrictions();
    int num_per_elem = rv[0].num_scalars_per_entity();
    if(topology_rank == stk::topology::ELEMENT_RANK)
    {
      stk::mesh::Entity stk_e1 = get_stk_entity(e1);
      double* vals1 = stk::mesh::field_data(*cur_field, stk_e1);
      if(this == output_mesh_api)
      {
        stk::mesh::Entity stk_e2 = get_stk_entity(e2);
        double* vals2 = stk::mesh::field_data(*cur_field, stk_e2);
        for(int j=0; j<num_per_elem; ++j)
          vals2[j] = vals1[j];
      }
    }
  }
}

void PruneMeshAPISTK::transfer_output_fields(PruneHandle n1, PruneHandle n2, PruneHandle new_node,
                                           double mu, PruneMeshAPI *output_mesh_api)
{
  for(size_t i=0; i<mNodalFields.size(); ++i)
  {
    stk::mesh::Field<double> *cur_field = mNodalFields[i];
    std::string name = cur_field->name();
    stk::mesh::EntityRank topology_rank = cur_field->entity_rank();
    stk::mesh::FieldRestrictionVector rv = cur_field->restrictions();
    int num_per_node = rv[0].num_scalars_per_entity();
    if(topology_rank == stk::topology::NODE_RANK)
    {
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
    }
  }
}

PruneHandle PruneMeshAPISTK::new_node(double &x, double &y, double &z)
{
  stk::mesh::EntityId entity_id = get_next_entity_id(stk::topology::NODE_RANK);  
  stk::mesh::Entity stk_node = mBulkData->declare_node(entity_id);
  double* node_coords = stk::mesh::field_data(*mCoordsField, stk_node);
  node_coords[0] = x;
  node_coords[1] = y;
  node_coords[2] = z;
  return get_handle(stk_node);
}

void PruneMeshAPISTK::reserve_new_node_ids(uint64_t num_requested)
{
  mAvailableNodeIds.clear();
  mBulkData->generate_new_ids(stk::topology::NODE_RANK, num_requested, mAvailableNodeIds);
  mNextAvailableNodeIndex = 0;
}

void PruneMeshAPISTK::reserve_new_tri_ids(uint64_t num_requested)
{
  mAvailableElemIds.clear();
  mBulkData->generate_new_ids(stk::topology::ELEM_RANK, num_requested, mAvailableElemIds);
  mNextAvailableElemIndex = 0;
}

stk::mesh::EntityId PruneMeshAPISTK::get_next_entity_id(stk::topology::rank_t rank)
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

void PruneMeshAPISTK::store_tri_to_tet_map_entry(const PruneHandle &tri, const PruneHandle &tet)
{
/*
  CubitTri *ctri = CubitTri::find(tri);
  CubitTet *ctet = CubitTet::find(tet);
  IsoSurfaceTool::tri_to_tet_map[ctri] = ctet;
*/
}

void PruneMeshAPISTK::store_tet_to_tri_map_entry(const PruneHandle &tet, const PruneHandle &tri)
{
/*
  CubitTri *ctri = CubitTri::find(tri);
  CubitTet *ctet = CubitTet::find(tet);
  IsoSurfaceTool::tet_to_tri_map[ctet].push_back(ctri);
*/
}

bool PruneMeshAPISTK::read_exodus_mesh( std::string &meshfile, std::string &fieldname,
                                      std::string &outputFieldsString,
                                      int input_file_is_spread,
                                      int time_step )
{
  mIoBroker->set_option_to_not_collapse_sequenced_fields();
  if(!input_file_is_spread)
    mIoBroker->property_add(Ioss::Property("DECOMPOSITION_METHOD", "RIB"));
  mIoBroker->add_mesh_database(meshfile, "exodus", stk::io::READ_MESH);
  mIoBroker->create_input_mesh();

  mIoBroker->add_all_mesh_fields_as_input_fields();

  get_output_fields(outputFieldsString);

  prepare_new_block();

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

  // The fieldname string can contain multiple comma-separated names
  std::string working_string = fieldname; // make a copy since we will be modifying it
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
  for(size_t i=0; i<parsed_strings.size(); ++i)
  {
    stk::mesh::Field<double> *cur_field = mMetaData->get_field<stk::mesh::Field<double> >(
                              stk::topology::NODE_RANK, parsed_strings[i]);
    if(!cur_field)
    {
      if(mBulkData && mBulkData->parallel_rank() == 0)
      {
        std::cout << "Failed to find " << parsed_strings[i] << " nodal variable." << std::endl;
        return false;
      }
    }
    else
    {
        mIsoFields.push_back(cur_field);
    }
  }

  if(time_step == -1)
    time_step = mIoBroker->get_input_io_region().get()->get_property("state_count").get_int();
  mIoBroker->read_defined_input_fields(time_step);

  return true;
}

void PruneMeshAPISTK::export_my_mesh()
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

void PruneMeshAPISTK::print_boundary_node_info(std::vector<BoundaryNodeInfo> &bni)
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

void PruneMeshAPISTK::get_output_fields(std::string &outputFieldsString)
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
  if(working_string.size() > 0)
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
    if(bulk_data()->parallel_rank() == 0)
      std::cout << "Error: Could not find all of the requested output fields!\n" << std::endl;
  }
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

void PruneMeshAPISTK::get_fixed_block_nodes(std::vector<PruneHandle> &fixed_block_nodes)
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
  std::vector<PruneHandle>::iterator it = std::unique(fixed_block_nodes.begin(),fixed_block_nodes.end());
  fixed_block_nodes.resize(it-fixed_block_nodes.begin());
}

void PruneMeshAPISTK::set_fixed_block_ids(std::string fixed_block_string)
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

void PruneMeshAPISTK::prepare_new_block()
{
}

void PruneMeshAPISTK::write_exodus_mesh( std::string &meshfile, int concatenate, int iso_only )
{
  if(mTimeStep > 0 && mBulkData->parallel_rank() == 0)
  {
    printf("\n########################################################\n");
    printf("Writing results for time step %d.\n", mTimeStep);
    printf("########################################################\n\n");
  }

  if(concatenate)
    mIoBroker->property_add(Ioss::Property("COMPOSE_RESULTS", true));
  size_t fh = mIoBroker->create_output_mesh(meshfile, stk::io::WRITE_RESULTS);

  // Make sure to write the iso field out.
  for(size_t i=0; i<mIsoFields.size(); ++i)
  {
    mIoBroker->add_field(fh, *(mIsoFields[i])); 
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
  mIoBroker->write_output_mesh(fh);
}

void PruneMeshAPISTK::get_shared_boundary_nodes(std::set<PruneHandle> &shared_boundary_nodes)
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

void PruneMeshAPISTK::get_attached_elements(const std::set<PruneHandle> &nodes,
                                          std::vector<PruneHandle> &attached_elements)
{
  // get a set of locally owned elements for filtering the attached elements we return
  std::set<stk::mesh::Entity> owned_elems;
  stk::mesh::Selector sel(mMetaData->locally_owned_part());
  const stk::mesh::BucketVector &elem_buckets = mBulkData->get_buckets(stk::topology::ELEMENT_RANK, sel);
  for(size_t i=0; i<elem_buckets.size(); ++i)
  {
    for(size_t j=0; j<elem_buckets[i]->size(); ++j)
    {
      stk::mesh::Entity cur_elem = (*(elem_buckets[i]))[j];
      owned_elems.insert(cur_elem);
    }
  }

  std::set<PruneHandle>::iterator it = nodes.begin();
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
  std::vector<PruneHandle>::iterator it2 = std::unique(attached_elements.begin(),attached_elements.end());
  attached_elements.resize(it2-attached_elements.begin());
}

bool PruneMeshAPISTK::prepare_as_source()
{
  mMetaData = new stk::mesh::MetaData;
  mLocallyOwnedMeta = true;
  mBulkData = new stk::mesh::BulkData(*mMetaData, *mComm);
  mLocallyOwnedBulk = true;
  mIoBroker = new stk::io::StkMeshIoBroker(*mComm);
  mIoBroker->set_bulk_data(*mBulkData);
  return true;
}

stk::mesh::EntityId PruneMeshAPISTK::entity_id(PruneHandle &h)
{
  stk::mesh::Entity ent = get_stk_entity(h);
  stk::mesh::EntityId ent_id = mBulkData->identifier(ent);
  return ent_id;
}

/*
bool PruneMeshAPISTK::prepare_as_destination()
{
  mMetaData = new stk::mesh::MetaData(3);
  mLocallyOwnedMeta = true;
  mCoordsField = (stk::mesh::Field<double, stk::mesh::Cartesian>*)(&(mMetaData->
                declare_field<stk::mesh::Field<double, stk::mesh::Cartesian> >
                            (stk::topology::NODE_RANK, "coordinates")));
  if(!mCoordsField)
    std::cout << "Failed to find nodal coordinate field." << std::endl;
  stk::mesh::put_field_on_entire_mesh(*mCoordsField);
  prepare_to_create_tris();
  mMetaData->commit();
  mBulkData = new stk::mesh::BulkData(*mMetaData, *mComm);
  mLocallyOwnedBulk = true;
  mIoBroker = new stk::io::StkMeshIoBroker(*mComm);
  mIoBroker->set_bulk_data(*mBulkData);
  return true;
}
*/






