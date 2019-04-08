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

#include "IVEMeshAPI.hpp"
#include "IsoVolumeExtractionTool.hpp"
#include <algorithm>
#include <iostream>
#include <cmath>

bool boundary_equal(BoundaryNodeInfo bni1, BoundaryNodeInfo bni2)
{
  if(bni1.min_node == bni2.min_node && bni1.max_node == bni2.max_node)
    return true;
  return false;
}

bool boundary_less(BoundaryNodeInfo bni1, BoundaryNodeInfo bni2)
{
  if(bni1.min_node < bni2.min_node)
    return true;
  if(bni1.min_node == bni2.min_node)
  {
    if(bni1.max_node < bni2.max_node)
      return true;
  }
  return false;
}

bool duplicate_less(BoundaryNodeInfo bni1, BoundaryNodeInfo bni2)
{
  if(bni1.min_node < bni2.min_node)
    return true;
  return false;
}

bool duplicate_equal(BoundaryNodeInfo bni1, BoundaryNodeInfo bni2)
{
  if(bni1.min_node == bni2.min_node)
    return true;
  return false;
}

void IsoVolumeExtractionTool::build_iso_volume_tris_from_hexes(const std::vector<IVEHandle> &hex_list,
                                                               const double &iso_value,
                                                               const double &min_edge_length,
                                                               const bool &level_set_data,
                                                               std::vector<IVEHandle> &fixed_tris,
                                                               std::vector<IVEHandle> &optimized_tris,
                                                               IVEMeshAPI *mesh_api,
                                                               IVEMeshAPI *mesh_api_out)
{
  // Distance tolerance is the minimum mesh edge length we want to end up with
  // when we extract the iso surface.  Value range is the max range of values
  // that could be seen in a given element.  For density this range is always
  // from 0 to 1 resulting in a range of 1.0.  For level set data the nodal
  // variables are the distance from the iso surface so the further an element
  // and its nodes are from the iso surface the larger the number will be.
  // For now, the code is assuming a fixed range like we get when the
  // nodal variable data is density. Value tolerance is the tolerance in
  // nodal variable value space that is equivalent to the distance tolerance
  // in real 3D space.
  size_t num_hexes = hex_list.size();
  double value_range = 1.0;
  
  double ave_edge_length, min_x, min_y, min_z, max_x, max_y, max_z;
  mesh_api->calculate_average_edge_length_and_bbox(hex_list, min_x, min_y, min_z, max_x, max_y, max_z, ave_edge_length);
  minx(min_x);
  miny(min_y);
  minz(min_z);
  maxx(max_x);
  maxy(max_y);
  maxz(max_z);
  average_edge_length(ave_edge_length);

  double value_tol;
  if(level_set_data)
    value_tol = min_edge_length;
  else
    value_tol = min_edge_length * value_range / ave_edge_length;
  
  // Adjust nodal variables to snapped positions.  If building this
  // map which is a duplicate of the exodus nodal variable map
  // takes up too much memory we may need to back off and just
  // adjust the nodal values as we process them.  Although that
  // would result in processing/adjusting the same nodes
  // over and over as we process all of the hexes each came from.
  std::map<IVEHandle,double> adjusted_nodal_vars;
  std::vector<IVEHandle> all_nodes(num_hexes*8);
  int cntr = 0;
  for(size_t a=0; a<num_hexes; ++a)
  {
    IVEHandle cur_hex = hex_list[a];
    IVEHandle hex_node_list[8];
    mesh_api->hex_nodes(cur_hex, hex_node_list);
    for(int b=0; b<8; b++)
      all_nodes[cntr++] = hex_node_list[b];
  }
  // Uniquify the list of nodes
  std::sort(all_nodes.begin(), all_nodes.end());
  std::vector<IVEHandle>::iterator it = std::unique(all_nodes.begin(),all_nodes.end());
  all_nodes.resize(it-all_nodes.begin());
  for(size_t a=0; a<all_nodes.size(); a++)
  {
    IVEHandle node = all_nodes[a];
    double val = mesh_api->get_nodal_iso_field_variable(node);
    if(fabs(val-iso_value) < value_tol)
      val = iso_value;
    adjusted_nodal_vars[node] = val;
  }
  
  // Set all nodal densities in fixed blocks to "iso_value"
  std::vector<IVEHandle> fixed_block_nodes;
  mesh_api->get_fixed_block_nodes(fixed_block_nodes);
  size_t num_fixed_block_nodes = fixed_block_nodes.size();
  for(size_t a=0; a < num_fixed_block_nodes; ++a)
  {
    IVEHandle node = fixed_block_nodes[a];
    adjusted_nodal_vars[node] = iso_value;  
  }

  //mesh_api_out->reserve_new_node_ids(10*num_hexes);
  mesh_api_out->reserve_new_node_ids(20*num_hexes);
  mesh_api_out->reserve_new_tri_ids(12*num_hexes);

  std::set<IVEHandle> shared_boundary_nodes;
  mesh_api->get_shared_boundary_nodes(shared_boundary_nodes);

  std::vector<IVEHandle> boundary_elements;
  mesh_api->get_attached_elements(shared_boundary_nodes, boundary_elements);

  std::multimap<IVEHandle, MeshEdge*> edge_map;
  std::map<IVEHandle, IVEHandle> node_map;
  create_nodes_on_processor_boundaries(adjusted_nodal_vars, edge_map, node_map, iso_value, mesh_api, mesh_api_out, true, value_tol,
                                       shared_boundary_nodes, boundary_elements);

  for(size_t a=0; a<num_hexes; a++)
  {
    IVEHandle cur_hex = hex_list[a];
    IVEHandle hex_node_list[8];
    mesh_api->hex_nodes(cur_hex, hex_node_list);
    double vals[8];
    for(int b=0; b<8; b++)
      vals[b] = adjusted_nodal_vars[hex_node_list[b]];
    
    std::set<IVEHandle> dummy_set;
    std::vector<BoundaryNodeInfo> dummy_list;
    create_interior_tris_for_hex(cur_hex, hex_node_list, vals, iso_value, edge_map, node_map, optimized_tris, mesh_api, mesh_api_out, false, dummy_list, dummy_set);
    create_boundary_tris_for_hex(cur_hex, adjusted_nodal_vars, edge_map, node_map, fixed_tris, iso_value, mesh_api, mesh_api_out, false, dummy_list, shared_boundary_nodes);
  }

  for(std::map<IVEHandle, MeshEdge*>::iterator it_del = edge_map.begin(); it_del != edge_map.end(); ++it_del)
  {
      delete it_del->second;
      it_del->second = NULL;
  }
}

void IsoVolumeExtractionTool::build_iso_volume_tris_from_tets(const std::vector<IVEHandle> &tet_list,
                                                              const double &iso_value,
                                                              const double &min_edge_length,
                                                              const bool &level_set_data,
                                                              std::vector<IVEHandle> &fixed_tris,
                                                              std::vector<IVEHandle> &optimized_tris,
                                                              IVEMeshAPI *mesh_api,
                                                              IVEMeshAPI *mesh_api_out)
{
  // Distance tolerance is the minimum mesh edge length we want to end up with
  // when we extract the iso surface.  Value range is the max range of values
  // that could be seen in a given element.  For density this range is always
  // from 0 to 1 resulting in a range of 1.0.  For level set data the nodal
  // variables are the distance from the iso surface so the further an element
  // and its nodes are from the iso surface the larger the number will be.
  // For now, the code is assuming a fixed range like we get when the
  // nodal variable data is density. Value tolerance is the tolerance in
  // nodal variable value space that is equivalent to the distance tolerance
  // in real 3D space.
  size_t num_tets = tet_list.size();
  double value_range = 1.0;
  
  double ave_edge_length, min_x, min_y, min_z, max_x, max_y, max_z;
  mesh_api->calculate_average_edge_length_and_bbox(tet_list, min_x, min_y, min_z, max_x, max_y, max_z, ave_edge_length);
  minx(min_x);
  miny(min_y);
  minz(min_z);
  maxx(max_x);
  maxy(max_y);
  maxz(max_z);
  average_edge_length(ave_edge_length);

  double value_tol;
  if(level_set_data)
    value_tol = min_edge_length;
  else
    value_tol = min_edge_length * value_range / ave_edge_length;

  std::map<IVEHandle, double> nodal_vars;
  std::vector<IVEHandle> all_nodes(num_tets*4);
  size_t cntr = 0;
  for(size_t a=0; a<num_tets; a++)
  {
    IVEHandle cur_tet = tet_list[a];
    IVEHandle tet_node_list[4];
    mesh_api->tet_nodes(cur_tet, tet_node_list);
    for(int b=0; b<4; b++)
      all_nodes[cntr++] = tet_node_list[b];
  }
  // Uniquify the list of nodes
  std::sort(all_nodes.begin(), all_nodes.end());
  std::vector<IVEHandle>::iterator it = std::unique(all_nodes.begin(),all_nodes.end());
  all_nodes.resize(it-all_nodes.begin());
  for(size_t a=0; a<all_nodes.size(); a++)
  {
    IVEHandle node = all_nodes[a];
    double val = mesh_api->get_nodal_iso_field_variable(node);
    nodal_vars[node] = val;
  }

  mesh_api_out->reserve_new_node_ids(6*num_tets);
  mesh_api_out->reserve_new_tri_ids(10*num_tets);

  std::set<IVEHandle> shared_boundary_nodes;
  mesh_api->get_shared_boundary_nodes(shared_boundary_nodes);

  std::vector<IVEHandle> boundary_elements;
  mesh_api->get_attached_elements(shared_boundary_nodes, boundary_elements);

  std::multimap<IVEHandle, MeshEdge*> edge_map;
  std::map<IVEHandle,IVEHandle> node_map;
  create_nodes_on_processor_boundaries(nodal_vars, edge_map, node_map, iso_value, mesh_api, mesh_api_out, false, value_tol,
                                       shared_boundary_nodes, boundary_elements);

  std::set<IVEHandle> dummy_set;
  std::vector<BoundaryNodeInfo> dummy_list1, dummy_list2;
  for (size_t g=0; g<num_tets; g++)
    process_tet(tet_list[g], fixed_tris, optimized_tris, iso_value, value_tol, edge_map, node_map, mesh_api, mesh_api_out, false, dummy_list1, dummy_list2, dummy_set);

  for(std::map<IVEHandle, MeshEdge*>::iterator it_del = edge_map.begin(); it_del != edge_map.end(); ++it_del) {
      delete it_del->second;
      it_del->second = NULL;
  }
}

void IsoVolumeExtractionTool::create_nodes_on_processor_boundaries(std::map<IVEHandle,double> &nodal_vars,
                                                    std::multimap<IVEHandle,MeshEdge*> &edge_map,
                                                    std::map<IVEHandle,IVEHandle> &node_map,
                                                    double iso_value,
                                                    IVEMeshAPI *mesh_api,
                                                    IVEMeshAPI *mesh_api_out,
                                                    bool hex, 
                                                    double value_tol,
                                                    std::set<IVEHandle> &shared_boundary_nodes, 
                                                    std::vector<IVEHandle> &boundary_elements)
{
/*
  if(shared_boundary_nodes.size() == 0)
    return;

  if(boundary_elements.size() == 0)
    return;
*/

  std::vector<BoundaryNodeInfo> edge_node_info, duplicate_node_info;
  for(size_t i=0; i<boundary_elements.size(); ++i)
  {
    IVEHandle cur_elem = boundary_elements[i];
    std::vector<IVEHandle> dummy_tri_list, dummy_tri_list2;
    if(hex)
    {
      IVEHandle node_list[8];
      mesh_api->hex_nodes(cur_elem, node_list);
      double vals[8];
      for(int b=0; b<8; b++)
        vals[b] = nodal_vars[node_list[b]];
      create_interior_tris_for_hex(cur_elem, node_list, vals, iso_value, edge_map, node_map, 
           dummy_tri_list, mesh_api, mesh_api_out, true, edge_node_info, shared_boundary_nodes);
      create_boundary_tris_for_hex(cur_elem, nodal_vars, edge_map, node_map, dummy_tri_list, 
           iso_value, mesh_api, mesh_api_out, true, duplicate_node_info, shared_boundary_nodes);
    }
    else
    {
      IVEHandle node_list[4];
      mesh_api->tet_nodes(cur_elem, node_list);
      process_tet(cur_elem, dummy_tri_list, dummy_tri_list2, iso_value, value_tol, edge_map, 
           node_map, mesh_api, mesh_api_out, true, edge_node_info, duplicate_node_info, 
           shared_boundary_nodes);
    }
  }

  // remove duplicates from edge node info
  std::sort(edge_node_info.begin(), edge_node_info.end(), boundary_less);
  std::vector<BoundaryNodeInfo>::iterator it = std::unique(edge_node_info.begin(), 
                                                           edge_node_info.end(), 
                                                           boundary_equal);
  edge_node_info.resize(it-edge_node_info.begin());

  // remove duplicates from duplicate node info
  std::sort(duplicate_node_info.begin(), duplicate_node_info.end(), duplicate_less);
  it = std::unique(duplicate_node_info.begin(), 
                   duplicate_node_info.end(), 
                   duplicate_equal);
  duplicate_node_info.resize(it-duplicate_node_info.begin());

  // Create data structures for just creating nodes that lie on existing nodes.
  std::vector<DuplicateNodeInfo> duplicate_node_infos;
  std::vector<BoundaryNodeInfo> mid_edge_infos;
  std::set<IVEHandle> nodes_to_duplicate;
  std::vector<BoundaryNodeInfo>::iterator edge_node_info_it = edge_node_info.begin();
  while(edge_node_info_it != edge_node_info.end())
  {
    IVEHandle existing_node_id = edge_node_info_it->existing_node_id;
    if(existing_node_id != 0)
    {
      if(nodes_to_duplicate.find(existing_node_id) == nodes_to_duplicate.end())
      {
        nodes_to_duplicate.insert(existing_node_id);
        DuplicateNodeInfo dni;
        mesh_api->set_existing_node_id(dni, existing_node_id);
        dni.new_node_pos = edge_node_info_it->new_node_pos;
        dni.suggested_new_node_id = edge_node_info_it->suggested_new_node_id;
        duplicate_node_infos.push_back(dni); 
      }
    }
    else
      mid_edge_infos.push_back(*edge_node_info_it);
    edge_node_info_it++;
  }
  std::vector<BoundaryNodeInfo>::iterator dup_node_info_it = duplicate_node_info.begin();
  while(dup_node_info_it != duplicate_node_info.end())
  {
    IVEHandle existing_node_id = dup_node_info_it->existing_node_id;
    if(nodes_to_duplicate.find(existing_node_id) == nodes_to_duplicate.end())
    {
      nodes_to_duplicate.insert(existing_node_id);
      DuplicateNodeInfo dni;
      mesh_api->set_existing_node_id(dni, existing_node_id);
      dni.new_node_pos = dup_node_info_it->new_node_pos;
      dni.suggested_new_node_id = dup_node_info_it->suggested_new_node_id;
      duplicate_node_infos.push_back(dni); 
    }
    dup_node_info_it++;
  }

  mesh_api_out->batch_create_duplicate_nodes(duplicate_node_infos, mesh_api);

  // Add entries to node map for newly created nodes
  std::vector<DuplicateNodeInfo>::iterator dup_it = duplicate_node_infos.begin();
  while(dup_it != duplicate_node_infos.end())
  {
    node_map[dup_it->existing_node_local_id] = dup_it->new_node;
    dup_it++;
  }

  // Create edge_map entries for the new nodes
  for(size_t i=0; i<edge_node_info.size(); ++i)
  {
    BoundaryNodeInfo &bni = edge_node_info[i];
    if(bni.existing_node_id != 0)
    {
      // Find the DuplicateNodeInfo that matches this edge info
      dup_it = duplicate_node_infos.begin();
      while(dup_it != duplicate_node_infos.end())
      {
        if(dup_it->existing_node_local_id == bni.existing_node_id)
        {
          MeshEdge *me = new MeshEdge;
          me->max_id_node = bni.max_node;
          me->mid_node = dup_it->new_node;
          edge_map.insert(std::multimap<IVEHandle,MeshEdge*>::value_type(bni.min_node, me));
          break;
        }
        dup_it++;
      }
    }
  }

  // Now create new nodes that are somewhere in the middle of existing edges.
  mesh_api_out->batch_create_edge_boundary_nodes(mid_edge_infos, mesh_api);
  
  // Create edge_map entries for the new nodes
  for(size_t i=0; i<mid_edge_infos.size(); ++i)
  {
    BoundaryNodeInfo &bni = mid_edge_infos[i];
    MeshEdge *me = new MeshEdge;
    me->max_id_node = bni.max_node;
    me->mid_node = bni.new_node;
    edge_map.insert(std::multimap<IVEHandle,MeshEdge*>::value_type(bni.min_node, me));
  }
}

void IsoVolumeExtractionTool::process_tet(const IVEHandle &tet,
                                          std::vector<IVEHandle> &fixed_tris,
                                          std::vector<IVEHandle> &optimized_tris,
                                          const double &iso_value,
                                          const double &value_tol,
                                          std::multimap<IVEHandle,MeshEdge*> &edge_map,
                                          std::map<IVEHandle,IVEHandle> &node_map,
                                          IVEMeshAPI *mesh_api,
                                          IVEMeshAPI *mesh_api_out,
                                          bool only_create_boundary_node_info,
                                          std::vector<BoundaryNodeInfo> &edge_node_info,
                                          std::vector<BoundaryNodeInfo> &duplicate_node_info,
                                          std::set<IVEHandle> &boundary_nodes)
{
  /* double t5707x = 0.22017025; */
  /* double t5707y = 0.41603475; */
  /* double t5707z = -0.071446; */
	
  /* double t6891x = 0.24403375; */
  /* double t6891y = 0.43941275; */
  /* double t6891z = -0.080284; */


  /* printf("tolerance %f\n",value_tol); */
  /* printf("iso_value %f\n",iso_value); */
  double lower_bound = iso_value - value_tol;
  double upper_bound = iso_value + value_tol;
  IVEHandle tet_nodes[4];
  mesh_api->tet_nodes(tet, tet_nodes);
  std::vector<IVEHandle> tmp_tri_nodes;
  std::vector<IVEHandle> nodes_used_in_tris;
  double vals[4];
  double v0, v1, v2, v3;
  IVEHandle n0 = tet_nodes[0];
  IVEHandle n1 = tet_nodes[1];
  IVEHandle n2 = tet_nodes[2];
  IVEHandle n3 = tet_nodes[3];

  /* IsoVector coords[4]; */
  /* for(int i = 0; i < 4; i++) */
  /*   coords[i] = mesh_api->node_coordinates(tet_nodes[i]); */

  /* IsoVector average = (coords[0] + coords[1] + coords[2] + coords[3])*0.25; */

  /* if(fabs(average.x() - t5707x) < value_tol && fabs(average.y() - t5707y) < value_tol && fabs(average.z() - t5707z) < value_tol) */
  /* { */
  /*   ; */
  /* } */
  /* if(fabs(average.x() - t6891x) < value_tol && fabs(average.y() - t6891y) < value_tol && fabs(average.z() - t6891z) < value_tol) */
  /* { */
  /*   ; */
  /* } */

  // Edge crossing nodes.  They will be indexed in the following
  // way: 0: Edge01, 1: Edge12, 2: Edge20,
  // 3: Edge30, 4: Edge31, 5: Edge32
  IVEHandle ec01 = 0;
  IVEHandle ec12 = 0;
  IVEHandle ec20 = 0;
  IVEHandle ec30 = 0;
  IVEHandle ec31 = 0;
  IVEHandle ec32 = 0;
  for(int i=0; i<4; i++)
    vals[i] = mesh_api->get_nodal_iso_field_variable(tet_nodes[i]);
  
  v0 = vals[0]; v1 = vals[1]; v2 = vals[2]; v3 = vals[3];
  
  BoundaryNodeInfo bni;

  // Look for nodes with iso surface going through them.
  if(v0 <= upper_bound && v0 >= lower_bound)
  {
    if(only_create_boundary_node_info)
    {
      if(boundary_nodes.find(n0) != boundary_nodes.end())
      {
        get_duplicate_node(n0, node_map, mesh_api, mesh_api_out, true, bni);
        duplicate_node_info.push_back(bni);
      }
    }
    else
    {
      IVEHandle new_node_handle = get_duplicate_node(n0, node_map, mesh_api, mesh_api_out, false, bni);
      nodes_used_in_tris.push_back(new_node_handle);
    }
  }
  if(v1 <= upper_bound && v1 >= lower_bound)
  {
    if(only_create_boundary_node_info)
    {
      if(boundary_nodes.find(n1) != boundary_nodes.end())
      {
        get_duplicate_node(n1, node_map, mesh_api, mesh_api_out, true, bni);
        duplicate_node_info.push_back(bni);
      }
    }
    else
    {
      IVEHandle new_node_handle = get_duplicate_node(n1, node_map, mesh_api, mesh_api_out, false, bni);
      nodes_used_in_tris.push_back(new_node_handle);
    }
  }
  if(v2 <= upper_bound && v2 >= lower_bound)
  {
    if(only_create_boundary_node_info)
    {
      if(boundary_nodes.find(n2) != boundary_nodes.end())
      {
        get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, true, bni);
        duplicate_node_info.push_back(bni);
      }
    }
    else
    {
      IVEHandle new_node_handle = get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, false, bni);
      nodes_used_in_tris.push_back(new_node_handle);
    }
  }
  if(v3 <= upper_bound && v3 >= lower_bound)
  {
    if(only_create_boundary_node_info)
    {
      if(boundary_nodes.find(n3) != boundary_nodes.end())
      {
        get_duplicate_node(n3, node_map, mesh_api, mesh_api_out, true, bni);
        duplicate_node_info.push_back(bni);
      }
    }
    else
    {
      IVEHandle new_node_handle = get_duplicate_node(n3, node_map, mesh_api, mesh_api_out, false, bni);
      nodes_used_in_tris.push_back(new_node_handle);
    }
  }
  
  // edge 01
  if((v0 > upper_bound && v1 < lower_bound) || (v0 < lower_bound && v1 > upper_bound))
  {
    if(only_create_boundary_node_info)
    {
      if(get_mid_node(n0, n1, v0, v1, iso_value, edge_map, node_map, mesh_api, mesh_api_out, true, bni))
        edge_node_info.push_back(bni);
    }
    else
    {
      ec01 = get_mid_node(n0, n1, v0, v1, iso_value, edge_map, node_map, mesh_api, mesh_api_out, false, bni);
      nodes_used_in_tris.push_back(ec01);
    }
  }
  // edge 20
  if((v0 > upper_bound && v2 < lower_bound) || (v0 < lower_bound && v2 > upper_bound))
  {
    if(only_create_boundary_node_info)
    {
      if(get_mid_node(n0, n2, v0, v2, iso_value, edge_map, node_map, mesh_api, mesh_api_out, true, bni))
        edge_node_info.push_back(bni);
    }
    else
    {
      ec20 = get_mid_node(n0, n2, v0, v2, iso_value, edge_map, node_map, mesh_api, mesh_api_out, false, bni);
      nodes_used_in_tris.push_back(ec20);
    }
  }
  // edge 30
  if((v0 > upper_bound && v3 < lower_bound) || (v0 < lower_bound && v3 > upper_bound))
  {
    if(only_create_boundary_node_info)
    {
      if(get_mid_node(n0, n3, v0, v3, iso_value, edge_map, node_map, mesh_api, mesh_api_out, true, bni))
        edge_node_info.push_back(bni);
    }
    else
    {
      ec30 = get_mid_node(n0, n3, v0, v3, iso_value, edge_map, node_map, mesh_api, mesh_api_out, false, bni);
      nodes_used_in_tris.push_back(ec30);
    }
  }
  // edge 12
  if((v1 > upper_bound && v2 < lower_bound) || (v1 < lower_bound && v2 > upper_bound))
  {
    if(only_create_boundary_node_info)
    {
      if(get_mid_node(n1, n2, v1, v2, iso_value, edge_map, node_map, mesh_api, mesh_api_out, true, bni))
        edge_node_info.push_back(bni);
    }
    else
    {
      ec12 = get_mid_node(n1, n2, v1, v2, iso_value, edge_map, node_map, mesh_api, mesh_api_out, false, bni);
      nodes_used_in_tris.push_back(ec12);
    }
  }
  // edge 31
  if((v1 > upper_bound && v3 < lower_bound) || (v1 < lower_bound && v3 > upper_bound))
  {
    if(only_create_boundary_node_info)
    {
      if(get_mid_node(n3, n1, v3, v1, iso_value, edge_map, node_map, mesh_api, mesh_api_out, true, bni))
        edge_node_info.push_back(bni);
    }
    else
    {
      ec31 = get_mid_node(n3, n1, v3, v1, iso_value, edge_map, node_map, mesh_api, mesh_api_out, false, bni);
      nodes_used_in_tris.push_back(ec31);
    }
  }
  // edge 32
  if((v2 > upper_bound && v3 < lower_bound) || (v2 < lower_bound && v3 > upper_bound))
  {
    if(only_create_boundary_node_info)
    {
      if(get_mid_node(n3, n2, v3, v2, iso_value, edge_map, node_map, mesh_api, mesh_api_out, true, bni))
        edge_node_info.push_back(bni);
    }
    else
    {
      ec32 = get_mid_node(n3, n2, v3, v2, iso_value, edge_map, node_map, mesh_api, mesh_api_out, false, bni);
      nodes_used_in_tris.push_back(ec32);
    }
  }

  if(!only_create_boundary_node_info)
  {
    if(nodes_used_in_tris.size() == 3)
    {
      IVEHandle new_tri = createOrientedTriInTet(mesh_api, mesh_api_out, tet, nodes_used_in_tris[0], nodes_used_in_tris[1], nodes_used_in_tris[2]);
      mesh_api->copy_element_output_fields(tet, new_tri, mesh_api_out);
      optimized_tris.push_back(new_tri);
      mesh_api_out->store_tri_to_tet_map_entry(new_tri, tet);
      mesh_api_out->store_tet_to_tri_map_entry(tet, new_tri);
    }
    else if(nodes_used_in_tris.size() == 4)
    {
      IVEHandle new_tri = createOrientedTriInTet(mesh_api, mesh_api_out, tet, nodes_used_in_tris[0], nodes_used_in_tris[1], nodes_used_in_tris[2]);
      mesh_api->copy_element_output_fields(tet, new_tri, mesh_api_out);
      optimized_tris.push_back(new_tri);
      mesh_api_out->store_tri_to_tet_map_entry(new_tri, tet);
      mesh_api_out->store_tet_to_tri_map_entry(tet, new_tri);
      IsoVector p0 = mesh_api_out->node_coordinates(nodes_used_in_tris[0]);
      IsoVector p1 = mesh_api_out->node_coordinates(nodes_used_in_tris[1]);
      IsoVector p2 = mesh_api_out->node_coordinates(nodes_used_in_tris[2]);
      IsoVector p3 = mesh_api_out->node_coordinates(nodes_used_in_tris[3]);
      IsoVector vec1 = p0-p3;
      IsoVector vec2 = p1-p3;
      IsoVector vec3 = p2-p3;
      vec1.normalize();
      vec2.normalize();
      vec3.normalize();
      double dot1 = vec1 % vec2;
      double dot2 = vec1 % vec3;
      double dot3 = vec2 % vec3;
      if(dot1 < dot2 && dot1 < dot3)
      {
        new_tri = createOrientedTriInTet(mesh_api, mesh_api_out, tet, nodes_used_in_tris[0], nodes_used_in_tris[1], nodes_used_in_tris[3]);
        mesh_api->copy_element_output_fields(tet, new_tri, mesh_api_out);
        optimized_tris.push_back(new_tri);
        mesh_api_out->store_tri_to_tet_map_entry(new_tri, tet);
        mesh_api_out->store_tet_to_tri_map_entry(tet, new_tri);
      }
      else if(dot2 < dot1 && dot2 < dot3)
      {
        new_tri = createOrientedTriInTet(mesh_api, mesh_api_out, tet, nodes_used_in_tris[0], nodes_used_in_tris[2], nodes_used_in_tris[3]);
        mesh_api->copy_element_output_fields(tet, new_tri, mesh_api_out);
        optimized_tris.push_back(new_tri);
        mesh_api_out->store_tri_to_tet_map_entry(new_tri, tet);
        mesh_api_out->store_tet_to_tri_map_entry(tet, new_tri);
      }
      else if(dot3 < dot1 && dot3 < dot2)
      {
        new_tri = createOrientedTriInTet(mesh_api, mesh_api_out, tet, nodes_used_in_tris[1], nodes_used_in_tris[2], nodes_used_in_tris[3]);
        mesh_api->copy_element_output_fields(tet, new_tri, mesh_api_out);
        optimized_tris.push_back(new_tri);
        mesh_api_out->store_tri_to_tet_map_entry(new_tri, tet);
        mesh_api_out->store_tet_to_tri_map_entry(tet, new_tri);
      }
    }
    else if(nodes_used_in_tris.size() > 0)
    {
      //   PRINT_ERROR("Unhandled number of iso crossing nodes %d in tet %d.\n",
      //               (int)nodes_used_in_tris.size(), tet->id());
    }
  }
  
  //compute outward normals for tet faces
  IsoVector outwardNormal[4];
  IsoVector p0 = mesh_api_out->node_coordinates(tet_nodes[0]);
  IsoVector p1 = mesh_api_out->node_coordinates(tet_nodes[1]);
  IsoVector p2 = mesh_api_out->node_coordinates(tet_nodes[2]);
  IsoVector p3 = mesh_api_out->node_coordinates(tet_nodes[3]);
  
  IsoVector edge01 = p1 - p0;
  IsoVector edge02 = p2 - p0;
  IsoVector edge03 = p3 - p0;
  IsoVector edge12 = p2 - p1;
  IsoVector edge13 = p3 - p1;

  outwardNormal[0] = edge12 * edge13;
  outwardNormal[0].normalize();

  outwardNormal[1] = edge03 * edge02;
  outwardNormal[1].normalize();
  
  outwardNormal[2] = edge01 * edge03;
  outwardNormal[2].normalize();

  outwardNormal[3] = edge02 * edge01;
  outwardNormal[3].normalize();

  // Face 012
  create_boundary_tris_for_tet(tet, n0, n1, n2, v0, v1, v2, ec01, ec12, ec20,
                               fixed_tris, upper_bound, lower_bound, node_map, mesh_api, mesh_api_out,
                               only_create_boundary_node_info, duplicate_node_info, boundary_nodes, outwardNormal[3]);
  // Face 013
  create_boundary_tris_for_tet(tet, n0, n1, n3, v0, v1, v3, ec01, ec31, ec30,
                               fixed_tris, upper_bound, lower_bound, node_map, mesh_api, mesh_api_out,
                               only_create_boundary_node_info, duplicate_node_info, boundary_nodes, outwardNormal[2]);
  // Face 023
  create_boundary_tris_for_tet(tet, n0, n2, n3, v0, v2, v3, ec20, ec32, ec30,
                               fixed_tris, upper_bound, lower_bound, node_map, mesh_api, mesh_api_out,
                               only_create_boundary_node_info, duplicate_node_info, boundary_nodes, outwardNormal[1]);
  // Face 123
  create_boundary_tris_for_tet(tet, n1, n2, n3, v1, v2, v3, ec12, ec32, ec31,
                               fixed_tris, upper_bound, lower_bound, node_map, mesh_api, mesh_api_out,
                               only_create_boundary_node_info, duplicate_node_info, boundary_nodes, outwardNormal[0]);
}

IVEHandle IsoVolumeExtractionTool::createOrientedTriInTet(IVEMeshAPI* mesh_api, IVEMeshAPI* mesh_api_out,
                                                IVEHandle tet, IVEHandle n0, IVEHandle n1, IVEHandle n2)
{
    //initialize tet info
    IVEHandle tet_nodes[4];
    mesh_api->tet_nodes(tet, tet_nodes);

    double vals[4];
    for(int i=0; i<4; i++)
      vals[i] = mesh_api->get_nodal_iso_field_variable(tet_nodes[i]);

    //compute density gradient in tet
    IsoVector basisGrad[4];
    IsoVector p0 = mesh_api_out->node_coordinates(tet_nodes[0]);
    IsoVector p1 = mesh_api_out->node_coordinates(tet_nodes[1]);
    IsoVector p2 = mesh_api_out->node_coordinates(tet_nodes[2]);
    IsoVector p3 = mesh_api_out->node_coordinates(tet_nodes[3]);

    IsoVector edge01 = p1 - p0;
    IsoVector edge02 = p2 - p0;
    IsoVector edge03 = p3 - p0;
    IsoVector edge12 = p2 - p1;
    IsoVector edge13 = p3 - p1;

    basisGrad[3] = edge01 * edge02;
    /* basisGrad[3].normalize(); */

    basisGrad[0] = edge13 * edge12;
    /* basisGrad[0].normalize(); */

    basisGrad[2] = edge03 * edge01;
    /* basisGrad[2].normalize(); */

    basisGrad[1] = edge02 * edge03;
    /* basisGrad[1].normalize(); */

    IsoVector gradient = vals[0]*basisGrad[0] + vals[1]*basisGrad[1] + vals[2]*basisGrad[2] + vals[3]*basisGrad[3];
    gradient.normalize();

    //compute normal to new_tri
    IsoVector tri_node0 = mesh_api_out->node_coordinates(n0);
    IsoVector tri_node1 = mesh_api_out->node_coordinates(n1);
    IsoVector tri_node2 = mesh_api_out->node_coordinates(n2);
    IsoVector vec01 = tri_node1 - tri_node0;
    IsoVector vec02 = tri_node2 - tri_node0;
    IsoVector normal = vec01 * vec02;
    normal.normalize();

    IVEHandle new_tri;
    if(normal % gradient < 0)
    {
        new_tri = mesh_api_out->new_tri(n0, n1, n2, false, tet);
        return new_tri;
    }
    else
    {
        new_tri = mesh_api_out->new_tri(n0, n2, n1, false, tet);
        return new_tri;
    }
}

IVEHandle IsoVolumeExtractionTool::get_duplicate_node(const IVEHandle &old_node,
                                                      std::map<IVEHandle, IVEHandle> &node_map,
                                                      IVEMeshAPI *mesh_api,
                                                      IVEMeshAPI *mesh_api_out,
                                                      bool boundary_info,
                                                      BoundaryNodeInfo &bni)
{
  IVEHandle duplicate_node=0;
  if(boundary_info)
  {
    bni.new_node_pos = mesh_api->node_coordinates(old_node);
    bni.suggested_new_node_id = mesh_api_out->get_new_node_id();
    mesh_api->set_min_node_id(bni, old_node);
    mesh_api->set_max_node_id(bni, old_node);
    bni.mu = 0.0;
    bni.existing_node_id = old_node;
  }
  else
  {
    if(node_map.count(old_node))
      duplicate_node = node_map[old_node];
    else
    {
      IsoVector pos = mesh_api->node_coordinates(old_node);
      duplicate_node = mesh_api_out->new_node(pos);
      mesh_api->transfer_output_fields(old_node, old_node, duplicate_node, 0.0, mesh_api_out);
      node_map[old_node] = duplicate_node;
    }
  }
  return duplicate_node;
}

void IsoVolumeExtractionTool::create_boundary_tris_for_tet(const IVEHandle &cur_tet,
                                                           const IVEHandle &n0,
                                                           const IVEHandle &n1,
                                                           const IVEHandle &n2,
                                                           const double &v0,
                                                           const double &v1,
                                                           const double &v2,
                                                           const IVEHandle &ec01,
                                                           const IVEHandle &ec12,
                                                           const IVEHandle &ec20,
                                                           std::vector<IVEHandle> &fixed_tris,
                                                           const double &upper_bound,
                                                           const double &lower_bound,
                                                           std::map<IVEHandle, IVEHandle> &node_map,
                                                           IVEMeshAPI *mesh_api,
                                                           IVEMeshAPI *mesh_api_out, 
                                                           bool boundary_info,
                                                           std::vector<BoundaryNodeInfo> &duplicate_node_info,
                                                           std::set<IVEHandle> &boundary_nodes,
                                                           IsoVector& outwardNormal)
{
  IVEHandle new_tri;
  BoundaryNodeInfo bni;

  if(v0 > upper_bound || v1 > upper_bound || v2 > upper_bound)
  {
    // Something is positive so we need to do further checking.
    IVEHandle other_tet = mesh_api->get_connected_tet(cur_tet, n0, n1, n2);
    if(!other_tet)
    {
      if(v0 >= lower_bound && v1 >= lower_bound && v2 >= lower_bound)
      {
        if(boundary_info)
        {
          if(boundary_nodes.find(n0) != boundary_nodes.end())
          {
            get_duplicate_node(n0, node_map, mesh_api, mesh_api_out, true, bni);
            duplicate_node_info.push_back(bni);
          }
          if(boundary_nodes.find(n1) != boundary_nodes.end())
          {
            get_duplicate_node(n1, node_map, mesh_api, mesh_api_out, true, bni);
            duplicate_node_info.push_back(bni);
          }
          if(boundary_nodes.find(n2) != boundary_nodes.end())
          {
            get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, true, bni);
            duplicate_node_info.push_back(bni);
          }
        }
        else
        {
          // All >= 0.0: just create one tri for the whole face
          IVEHandle n0_new, n1_new, n2_new;
          n0_new = get_duplicate_node(n0, node_map, mesh_api, mesh_api_out, false, bni);
          n1_new = get_duplicate_node(n1, node_map, mesh_api, mesh_api_out, false, bni);
          n2_new = get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, false, bni);
          new_tri = createOrientedTriOnBoundaryTet(mesh_api, mesh_api_out, cur_tet, n0_new, n1_new, n2_new, outwardNormal);
          mesh_api->copy_element_output_fields(cur_tet, new_tri, mesh_api_out);
          fixed_tris.push_back(new_tri);
          mesh_api_out->store_tri_to_tet_map_entry(new_tri, cur_tet);;
          mesh_api_out->store_tet_to_tri_map_entry(cur_tet, new_tri);;
        }
      }
      else
      {
        // Check if one of the nodes has a zero value.  If so we know that
        // one of the others must be negative and the other positive.
        if(v0 >= lower_bound && v0 <= upper_bound)
        {
          if(v1 > upper_bound)
          {
            if(boundary_info)
            {
              if(boundary_nodes.find(n0) != boundary_nodes.end())
              {
                get_duplicate_node(n0, node_map, mesh_api, mesh_api_out, true, bni);
                duplicate_node_info.push_back(bni);
              }
              if(boundary_nodes.find(n1) != boundary_nodes.end())
              {
                get_duplicate_node(n1, node_map, mesh_api, mesh_api_out, true, bni);
                duplicate_node_info.push_back(bni);
              }
            }
            else
            {
              IVEHandle n0_new, n1_new;
              n0_new = get_duplicate_node(n0, node_map, mesh_api, mesh_api_out, false, bni);
              n1_new = get_duplicate_node(n1, node_map, mesh_api, mesh_api_out, false, bni);
              new_tri = createOrientedTriOnBoundaryTet(mesh_api, mesh_api_out, cur_tet, n0_new, n1_new, ec12, outwardNormal);
              mesh_api->copy_element_output_fields(cur_tet, new_tri, mesh_api_out);
            }
          }
          else
          {
            if(boundary_info)
            {
              if(boundary_nodes.find(n0) != boundary_nodes.end())
              {
                get_duplicate_node(n0, node_map, mesh_api, mesh_api_out, true, bni);
                duplicate_node_info.push_back(bni);
              }
              if(boundary_nodes.find(n2) != boundary_nodes.end())
              {
                get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, true, bni);
                duplicate_node_info.push_back(bni);
              }
            }
            else
            {
              IVEHandle n0_new, n2_new;
              n0_new = get_duplicate_node(n0, node_map, mesh_api, mesh_api_out, false, bni);
              n2_new = get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, false, bni);
              new_tri = createOrientedTriOnBoundaryTet(mesh_api, mesh_api_out, cur_tet, n0_new, n2_new, ec12, outwardNormal);
              mesh_api->copy_element_output_fields(cur_tet, new_tri, mesh_api_out);
            }
          }
          fixed_tris.push_back(new_tri);
          mesh_api_out->store_tri_to_tet_map_entry(new_tri, cur_tet);;
          mesh_api_out->store_tet_to_tri_map_entry(cur_tet, new_tri);;
        }
        else if(v1 >= lower_bound && v1 <= upper_bound)
        {
          if(v0 > upper_bound)
          {
            if(boundary_info)
            {
              if(boundary_nodes.find(n0) != boundary_nodes.end())
              {
                get_duplicate_node(n0, node_map, mesh_api, mesh_api_out, true, bni);
                duplicate_node_info.push_back(bni);
              }
              if(boundary_nodes.find(n1) != boundary_nodes.end())
              {
                get_duplicate_node(n1, node_map, mesh_api, mesh_api_out, true, bni);
                duplicate_node_info.push_back(bni);
              }
            }
            else
            {
              IVEHandle n0_new, n1_new;
              n0_new = get_duplicate_node(n0, node_map, mesh_api, mesh_api_out, false, bni);
              n1_new = get_duplicate_node(n1, node_map, mesh_api, mesh_api_out, false, bni);
              new_tri = createOrientedTriOnBoundaryTet(mesh_api, mesh_api_out, cur_tet, n1_new, n0_new, ec20, outwardNormal);
              mesh_api->copy_element_output_fields(cur_tet, new_tri, mesh_api_out);
            }
          }
          else
          {
            if(boundary_info)
            {
              if(boundary_nodes.find(n2) != boundary_nodes.end())
              {
                get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, true, bni);
                duplicate_node_info.push_back(bni);
              }
              if(boundary_nodes.find(n1) != boundary_nodes.end())
              {
                get_duplicate_node(n1, node_map, mesh_api, mesh_api_out, true, bni);
                duplicate_node_info.push_back(bni);
              }
            }
            else
            {
              IVEHandle n2_new, n1_new;
              n2_new = get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, false, bni);
              n1_new = get_duplicate_node(n1, node_map, mesh_api, mesh_api_out, false, bni);
              new_tri = createOrientedTriOnBoundaryTet(mesh_api, mesh_api_out, cur_tet, n1_new, n2_new, ec20, outwardNormal);
              mesh_api->copy_element_output_fields(cur_tet, new_tri, mesh_api_out);
            }
          }
          fixed_tris.push_back(new_tri);
          mesh_api_out->store_tri_to_tet_map_entry(new_tri, cur_tet);;
          mesh_api_out->store_tet_to_tri_map_entry(cur_tet, new_tri);;
        }
        else if(v2 >= lower_bound && v2 <= upper_bound)
        {
          if(v0 > upper_bound)
          {
            if(boundary_info)
            {
              if(boundary_nodes.find(n2) != boundary_nodes.end())
              {
                get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, true, bni);
                duplicate_node_info.push_back(bni);
              }
              if(boundary_nodes.find(n0) != boundary_nodes.end())
              {
                get_duplicate_node(n0, node_map, mesh_api, mesh_api_out, true, bni);
                duplicate_node_info.push_back(bni);
              }
            }
            else
            {
              IVEHandle n2_new, n0_new;
              n2_new = get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, false, bni);
              n0_new = get_duplicate_node(n0, node_map, mesh_api, mesh_api_out, false, bni);
              new_tri = createOrientedTriOnBoundaryTet(mesh_api, mesh_api_out, cur_tet, n0_new, n2_new, ec01, outwardNormal);
              mesh_api->copy_element_output_fields(cur_tet, new_tri, mesh_api_out);
            }
          }
          else
          {
            if(boundary_info)
            {
              if(boundary_nodes.find(n2) != boundary_nodes.end())
              {
                get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, true, bni);
                duplicate_node_info.push_back(bni);
              }
              if(boundary_nodes.find(n1) != boundary_nodes.end())
              {
                get_duplicate_node(n1, node_map, mesh_api, mesh_api_out, true, bni);
                duplicate_node_info.push_back(bni);
              }
            }
            else
            {
              IVEHandle n2_new, n1_new;
              n2_new = get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, false, bni);
              n1_new = get_duplicate_node(n1, node_map, mesh_api, mesh_api_out, false, bni);
              new_tri = createOrientedTriOnBoundaryTet(mesh_api, mesh_api_out, cur_tet, n1_new, n2_new, ec01, outwardNormal);
              mesh_api->copy_element_output_fields(cur_tet, new_tri, mesh_api_out);
            }
          }
          fixed_tris.push_back(new_tri);
          mesh_api->store_tri_to_tet_map_entry(new_tri, cur_tet);;
          mesh_api->store_tet_to_tri_map_entry(cur_tet, new_tri);;
        }
        else
        {
          // Only two cases left are if we have one positive and two negatives or
          // one negative and two positives.
          std::vector<IVEHandle> node_list;
          if(ec01)
            node_list.push_back(ec01);
          if(ec12)
            node_list.push_back(ec12);
          if(ec20)
            node_list.push_back(ec20);
          if(v0 > upper_bound)
          {
            if(boundary_info)
            {
              if(boundary_nodes.find(n0) != boundary_nodes.end())
              {
                get_duplicate_node(n0, node_map, mesh_api, mesh_api_out, true, bni);
                duplicate_node_info.push_back(bni);
              }
            }
            else
            {
              IVEHandle new_node = get_duplicate_node(n0, node_map, mesh_api, mesh_api_out, false, bni);
              node_list.push_back(new_node);
            }
          }
          if(v1 > upper_bound)
          {
            if(boundary_info)
            {
              if(boundary_nodes.find(n1) != boundary_nodes.end())
              {
                get_duplicate_node(n1, node_map, mesh_api, mesh_api_out, true, bni);
                duplicate_node_info.push_back(bni);
              }
            }
            else
            {
              IVEHandle new_node = get_duplicate_node(n1, node_map, mesh_api, mesh_api_out, false, bni);
              node_list.push_back(new_node);
            }
          }
          if(v2 > upper_bound)
          {
            if(boundary_info)
            {
              if(boundary_nodes.find(n2) != boundary_nodes.end())
              {
                get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, true, bni);
                duplicate_node_info.push_back(bni);
              }
            }
            else
            {
              IVEHandle new_node = get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, false, bni);
              node_list.push_back(new_node);
            }
          }
          
          if(!boundary_info)
          {
            if(node_list.size() == 3)
            {
              new_tri = createOrientedTriOnBoundaryTet(mesh_api, mesh_api_out, cur_tet, node_list[0], node_list[1], node_list[2], outwardNormal);
              mesh_api->copy_element_output_fields(cur_tet, new_tri, mesh_api_out);
              fixed_tris.push_back(new_tri);
              mesh_api_out->store_tri_to_tet_map_entry(new_tri, cur_tet);;
              mesh_api_out->store_tet_to_tri_map_entry(cur_tet, new_tri);;
            }
            else if(node_list.size() == 4)
            {
              new_tri = createOrientedTriOnBoundaryTet(mesh_api, mesh_api_out, cur_tet, node_list[0], node_list[1], node_list[2], outwardNormal);
              mesh_api->copy_element_output_fields(cur_tet, new_tri, mesh_api_out);
              fixed_tris.push_back(new_tri);
              mesh_api_out->store_tri_to_tet_map_entry(new_tri, cur_tet);;
              mesh_api_out->store_tet_to_tri_map_entry(cur_tet, new_tri);;
              IsoVector p0 = mesh_api_out->node_coordinates(node_list[0]);
              IsoVector p1 = mesh_api_out->node_coordinates(node_list[1]);
              IsoVector p2 = mesh_api_out->node_coordinates(node_list[2]);
              IsoVector p3 = mesh_api_out->node_coordinates(node_list[3]);
              IsoVector vec1 = p0 - p3;
              IsoVector vec2 = p1 - p3;
              IsoVector vec3 = p2 - p3;
              vec1.normalize();
              vec2.normalize();
              vec3.normalize();
              double dot1 = vec1 % vec2;
              double dot2 = vec1 % vec3;
              double dot3 = vec2 % vec3;
              if(dot1 < dot2 && dot1 < dot3)
              {
                new_tri = createOrientedTriOnBoundaryTet(mesh_api, mesh_api_out, cur_tet, node_list[0], node_list[1], node_list[3], outwardNormal);
                mesh_api->copy_element_output_fields(cur_tet, new_tri, mesh_api_out);
                fixed_tris.push_back(new_tri);
                mesh_api_out->store_tri_to_tet_map_entry(new_tri, cur_tet);;
                mesh_api_out->store_tet_to_tri_map_entry(cur_tet, new_tri);;
              }
              else if(dot2 < dot1 && dot2 < dot3)
              {
                new_tri = createOrientedTriOnBoundaryTet(mesh_api, mesh_api_out, cur_tet, node_list[0], node_list[2], node_list[3], outwardNormal);
                mesh_api->copy_element_output_fields(cur_tet, new_tri, mesh_api_out);
                fixed_tris.push_back(new_tri);
                mesh_api_out->store_tri_to_tet_map_entry(new_tri, cur_tet);;
                mesh_api_out->store_tet_to_tri_map_entry(cur_tet, new_tri);;
              }
              else if(dot3 < dot1 && dot3 < dot2)
              {
                new_tri = createOrientedTriOnBoundaryTet(mesh_api, mesh_api_out, cur_tet, node_list[1], node_list[2], node_list[3], outwardNormal);
                mesh_api->copy_element_output_fields(cur_tet, new_tri, mesh_api_out);
                fixed_tris.push_back(new_tri);
                mesh_api_out->store_tri_to_tet_map_entry(new_tri, cur_tet);;
                mesh_api_out->store_tet_to_tri_map_entry(cur_tet, new_tri);;
              }
            }
          }
        }
      }
    }
  }
  else if(v0 >= lower_bound && v0 <= upper_bound &&
          v1 >= lower_bound && v1 <= upper_bound &&
          v2 >= lower_bound && v2 <= upper_bound)
  {
    // All 3 values are equal to the iso extraction values 
    // so we need to check for a connected tet and if there
    // isn't one we will create a single tri for the whole face.
/*
    bool all_shared = (boundary_nodes.find(n0) != boundary_nodes.end() &&
                       boundary_nodes.find(n1) != boundary_nodes.end() &&
                       boundary_nodes.find(n2) != boundary_nodes.end());
*/
    IVEHandle other_tet = mesh_api->get_connected_tet(cur_tet, n0, n1, n2);
    if(!other_tet)
    //if(!all_shared && !other_tet)
    {
      if(boundary_info)
      {
        if(boundary_nodes.find(n2) != boundary_nodes.end())
        {
          get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, true, bni);
          duplicate_node_info.push_back(bni);
        }
        if(boundary_nodes.find(n1) != boundary_nodes.end())
        {
          get_duplicate_node(n1, node_map, mesh_api, mesh_api_out, true, bni);
          duplicate_node_info.push_back(bni);
        }
        if(boundary_nodes.find(n0) != boundary_nodes.end())
        {
          get_duplicate_node(n0, node_map, mesh_api, mesh_api_out, true, bni);
          duplicate_node_info.push_back(bni);
        }
      }
      else
      {
        IVEHandle n0_new, n1_new, n2_new;
        n0_new = get_duplicate_node(n0, node_map, mesh_api, mesh_api_out, false, bni);
        n1_new = get_duplicate_node(n1, node_map, mesh_api, mesh_api_out, false, bni);
        n2_new = get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, false, bni);
        new_tri = createOrientedTriOnBoundaryTet(mesh_api, mesh_api_out, cur_tet, n0_new, n1_new, n2_new, outwardNormal);
        mesh_api->copy_element_output_fields(cur_tet, new_tri, mesh_api_out);
        fixed_tris.push_back(new_tri);
        mesh_api_out->store_tri_to_tet_map_entry(new_tri, cur_tet);;
        mesh_api_out->store_tet_to_tri_map_entry(cur_tet, new_tri);;
      }
    }
  }
}

IVEHandle IsoVolumeExtractionTool::createOrientedTriOnBoundaryTet(IVEMeshAPI* mesh_api, IVEMeshAPI* mesh_api_out, IVEHandle tet,
                                         IVEHandle n0, IVEHandle n1, IVEHandle n2, IsoVector& outwardNormal)
{
  IsoVector p0 = mesh_api->node_coordinates(n0);
  IsoVector p1 = mesh_api->node_coordinates(n1);
  IsoVector p2 = mesh_api->node_coordinates(n2);

  IsoVector edge01 = p1 - p0;
  IsoVector edge02 = p2 - p0;

  IsoVector inputNormal = edge01 * edge02;
  inputNormal.normalize();

  IVEHandle new_tri;

  if(inputNormal % outwardNormal > 0)
    new_tri = mesh_api_out->new_tri(n0, n1, n2, true, tet);
  else
    new_tri = mesh_api_out->new_tri(n0, n2, n1, true, tet);

  return new_tri;
}

void IsoVolumeExtractionTool::create_interior_tris_for_hex(IVEHandle hex,
                                                           const IVEHandle hex_nodes[8],
                                                           const double *vals,
                                                           const double &isolevel,
                                                           std::multimap<IVEHandle, MeshEdge*> &edge_map,
                                                           std::map<IVEHandle,IVEHandle> &node_map,
                                                           std::vector<IVEHandle> &optimized_tris,
                                                           IVEMeshAPI *mesh_api,
                                                           IVEMeshAPI *mesh_api_out,
                                                           bool only_create_boundary_node_info,
                                                           std::vector<BoundaryNodeInfo> &boundary_node_info,
                                                           std::set<IVEHandle> &boundary_nodes)
{
  int cubeindex;
  IVEHandle node_list[12];
  
  int edgeTable[256]={
    0x0  , 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
    0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
    0x190, 0x99 , 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
    0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
    0x230, 0x339, 0x33 , 0x13a, 0x636, 0x73f, 0x435, 0x53c,
    0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
    0x3a0, 0x2a9, 0x1a3, 0xaa , 0x7a6, 0x6af, 0x5a5, 0x4ac,
    0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
    0x460, 0x569, 0x663, 0x76a, 0x66 , 0x16f, 0x265, 0x36c,
    0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
    0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff , 0x3f5, 0x2fc,
    0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
    0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55 , 0x15c,
    0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
    0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc ,
    0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
    0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
    0xcc , 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
    0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
    0x15c, 0x55 , 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
    0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
    0x2fc, 0x3f5, 0xff , 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
    0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
    0x36c, 0x265, 0x16f, 0x66 , 0x76a, 0x663, 0x569, 0x460,
    0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
    0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa , 0x1a3, 0x2a9, 0x3a0,
    0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
    0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33 , 0x339, 0x230,
    0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
    0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99 , 0x190,
    0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
    0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0   };
  int triTable[256][16] =
  {{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
    {3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
    {4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
    {9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
    {10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
    {5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
    {8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
    {2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
    {11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
    {5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
    {11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
    {11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
    {9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
    {6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
    {6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
    {8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
    {7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
    {3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
    {9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
    {8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
    {0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
    {6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
    {10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
    {10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
    {0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
    {3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
    {9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
    {8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
    {3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
    {10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
    {10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
    {7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
    {1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
    {11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
    {8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
    {0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
    {7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
    {7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
    {10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
    {0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
    {7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
    {9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
    {6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
    {4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
    {10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
    {8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
    {1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
    {10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
    {10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
    {9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
    {7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
    {3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
    {7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
    {3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
    {6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
    {9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
    {1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
    {4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
    {7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
    {6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
    {0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
    {6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
    {0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
    {11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
    {6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
    {5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
    {9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
    {1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
    {10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
    {0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
    {5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
    {11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
    {9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
    {7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
    {2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
    {9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
    {1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
    {10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
    {2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
    {0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
    {0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
    {9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
    {5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
    {5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
    {9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
    {1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
    {3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
    {4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
    {9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
    {11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
    {2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
    {9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
    {3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
    {1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
    {4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
    {0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
    {1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}};
  
  /*
   Determine the index into the edge table which
   tells us which vertices are inside of the surface
   */
  
  cubeindex = 0;
  if (vals[0] < isolevel) cubeindex |= 1;
  if (vals[1] < isolevel) cubeindex |= 2;
  if (vals[2] < isolevel) cubeindex |= 4;
  if (vals[3] < isolevel) cubeindex |= 8;
  if (vals[4] < isolevel) cubeindex |= 16;
  if (vals[5] < isolevel) cubeindex |= 32;
  if (vals[6] < isolevel) cubeindex |= 64;
  if (vals[7] < isolevel) cubeindex |= 128;
  
  /* Cube is entirely in/out of the surface */
  if (edgeTable[cubeindex] == 0)
    return;

  /* Find the vertices where the surface intersects the cube */
  BoundaryNodeInfo bni;
  if (edgeTable[cubeindex] & 1)
  {
    if(only_create_boundary_node_info)
    {  
      if(boundary_nodes.find(hex_nodes[0]) != boundary_nodes.end() &&
         boundary_nodes.find(hex_nodes[1]) != boundary_nodes.end())
      {
        if(get_mid_node(hex_nodes[0], hex_nodes[1], vals[0], vals[1], isolevel, edge_map, node_map, mesh_api, mesh_api_out, true, bni))
          boundary_node_info.push_back(bni);
      }
    }
    else
      node_list[0] = get_mid_node(hex_nodes[0], hex_nodes[1], vals[0], vals[1], isolevel, edge_map, node_map, mesh_api, mesh_api_out, false, bni);
  }
  if (edgeTable[cubeindex] & 2)
  {
    if(only_create_boundary_node_info)
    {  
      if(boundary_nodes.find(hex_nodes[1]) != boundary_nodes.end() &&
         boundary_nodes.find(hex_nodes[2]) != boundary_nodes.end())
      {
        if(get_mid_node(hex_nodes[1], hex_nodes[2], vals[1], vals[2], isolevel, edge_map, node_map, mesh_api, mesh_api_out, true, bni))
          boundary_node_info.push_back(bni);
      }
    }
    else
      node_list[1] = get_mid_node(hex_nodes[1], hex_nodes[2], vals[1], vals[2], isolevel, edge_map, node_map, mesh_api, mesh_api_out, false, bni);
  }
  if (edgeTable[cubeindex] & 4)
  {
    if(only_create_boundary_node_info)
    {  
      if(boundary_nodes.find(hex_nodes[2]) != boundary_nodes.end() &&
         boundary_nodes.find(hex_nodes[3]) != boundary_nodes.end())
      {
        if(get_mid_node(hex_nodes[2], hex_nodes[3], vals[2], vals[3], isolevel, edge_map, node_map, mesh_api, mesh_api_out, true, bni))
          boundary_node_info.push_back(bni);
      }
    }
    else
      node_list[2] = get_mid_node(hex_nodes[2], hex_nodes[3], vals[2], vals[3], isolevel, edge_map, node_map, mesh_api, mesh_api_out, false, bni);
  }
  if (edgeTable[cubeindex] & 8)
  {
    if(only_create_boundary_node_info)
    {  
      if(boundary_nodes.find(hex_nodes[3]) != boundary_nodes.end() &&
         boundary_nodes.find(hex_nodes[0]) != boundary_nodes.end())
      {
        if(get_mid_node(hex_nodes[3], hex_nodes[0], vals[3], vals[0], isolevel, edge_map, node_map, mesh_api, mesh_api_out, true, bni))
          boundary_node_info.push_back(bni);
      }
    }
    else
      node_list[3] = get_mid_node(hex_nodes[3], hex_nodes[0], vals[3], vals[0], isolevel, edge_map, node_map, mesh_api, mesh_api_out, false, bni);
  }
  if (edgeTable[cubeindex] & 16)
  {
    if(only_create_boundary_node_info)
    {  
      if(boundary_nodes.find(hex_nodes[4]) != boundary_nodes.end() &&
         boundary_nodes.find(hex_nodes[5]) != boundary_nodes.end())
      {
        if(get_mid_node(hex_nodes[4], hex_nodes[5], vals[4], vals[5], isolevel, edge_map, node_map, mesh_api, mesh_api_out, true, bni))
          boundary_node_info.push_back(bni);
      }
    }
    else
      node_list[4] = get_mid_node(hex_nodes[4], hex_nodes[5], vals[4], vals[5], isolevel, edge_map, node_map, mesh_api, mesh_api_out, false, bni);
  }
  if (edgeTable[cubeindex] & 32)
  {
    if(only_create_boundary_node_info)
    {  
      if(boundary_nodes.find(hex_nodes[5]) != boundary_nodes.end() &&
         boundary_nodes.find(hex_nodes[6]) != boundary_nodes.end())
      {
        if(get_mid_node(hex_nodes[5], hex_nodes[6], vals[5], vals[6], isolevel, edge_map, node_map, mesh_api, mesh_api_out, true, bni))
          boundary_node_info.push_back(bni);
      }
    }
    else
      node_list[5] = get_mid_node(hex_nodes[5], hex_nodes[6], vals[5], vals[6], isolevel, edge_map, node_map, mesh_api, mesh_api_out, false, bni);
  }
  if (edgeTable[cubeindex] & 64)
  {
    if(only_create_boundary_node_info)
    {  
      if(boundary_nodes.find(hex_nodes[6]) != boundary_nodes.end() &&
         boundary_nodes.find(hex_nodes[7]) != boundary_nodes.end())
      {
        if(get_mid_node(hex_nodes[6], hex_nodes[7], vals[6], vals[7], isolevel, edge_map, node_map, mesh_api, mesh_api_out, true, bni))
          boundary_node_info.push_back(bni);
      }
    }
    else
      node_list[6] = get_mid_node(hex_nodes[6], hex_nodes[7], vals[6], vals[7], isolevel, edge_map, node_map, mesh_api, mesh_api_out, false, bni);
  }
  if (edgeTable[cubeindex] & 128)
  {
    if(only_create_boundary_node_info)
    {  
      if(boundary_nodes.find(hex_nodes[7]) != boundary_nodes.end() &&
         boundary_nodes.find(hex_nodes[4]) != boundary_nodes.end())
      {
        if(get_mid_node(hex_nodes[7], hex_nodes[4], vals[7], vals[4], isolevel, edge_map, node_map, mesh_api, mesh_api_out, true, bni))
          boundary_node_info.push_back(bni);
      }
    }
    else
      node_list[7] = get_mid_node(hex_nodes[7], hex_nodes[4], vals[7], vals[4], isolevel, edge_map, node_map, mesh_api, mesh_api_out, false, bni);
  }
  if (edgeTable[cubeindex] & 256)
  {
    if(only_create_boundary_node_info)
    {  
      if(boundary_nodes.find(hex_nodes[0]) != boundary_nodes.end() &&
         boundary_nodes.find(hex_nodes[4]) != boundary_nodes.end())
      {
        if(get_mid_node(hex_nodes[0], hex_nodes[4], vals[0], vals[4], isolevel, edge_map, node_map, mesh_api, mesh_api_out, true, bni))
          boundary_node_info.push_back(bni);
      }
    }
    else
      node_list[8] = get_mid_node(hex_nodes[0], hex_nodes[4], vals[0], vals[4], isolevel, edge_map, node_map, mesh_api, mesh_api_out, false, bni);
  }
  if (edgeTable[cubeindex] & 512)
  {
    if(only_create_boundary_node_info)
    {  
      if(boundary_nodes.find(hex_nodes[1]) != boundary_nodes.end() &&
         boundary_nodes.find(hex_nodes[5]) != boundary_nodes.end())
      {
        if(get_mid_node(hex_nodes[1], hex_nodes[5], vals[1], vals[5], isolevel, edge_map, node_map, mesh_api, mesh_api_out, true, bni))
          boundary_node_info.push_back(bni);
      }
    }
    else
      node_list[9] = get_mid_node(hex_nodes[1], hex_nodes[5], vals[1], vals[5], isolevel, edge_map, node_map, mesh_api, mesh_api_out, false, bni);
  }
  if (edgeTable[cubeindex] & 1024)
  {
    if(only_create_boundary_node_info)
    {  
      if(boundary_nodes.find(hex_nodes[2]) != boundary_nodes.end() &&
         boundary_nodes.find(hex_nodes[6]) != boundary_nodes.end())
      {
        if(get_mid_node(hex_nodes[2], hex_nodes[6], vals[2], vals[6], isolevel, edge_map, node_map, mesh_api, mesh_api_out, true, bni))
          boundary_node_info.push_back(bni);
      }
    }
    else
      node_list[10] = get_mid_node(hex_nodes[2], hex_nodes[6], vals[2], vals[6], isolevel, edge_map, node_map, mesh_api, mesh_api_out, false, bni);
  }
  if (edgeTable[cubeindex] & 2048)
  {
    if(only_create_boundary_node_info)
    {  
      if(boundary_nodes.find(hex_nodes[3]) != boundary_nodes.end() &&
         boundary_nodes.find(hex_nodes[7]) != boundary_nodes.end())
      {
        if(get_mid_node(hex_nodes[3], hex_nodes[7], vals[3], vals[7], isolevel, edge_map, node_map, mesh_api, mesh_api_out, true, bni))
          boundary_node_info.push_back(bni);
      }
    }
    else
      node_list[11] = get_mid_node(hex_nodes[3], hex_nodes[7], vals[3], vals[7], isolevel, edge_map, node_map, mesh_api, mesh_api_out, false, bni);
  }

  if(only_create_boundary_node_info)
    return;
  
  /* Create the triangle */
  for (int i=0; triTable[cubeindex][i]!=-1; i+=3)
  {
    IVEHandle n1 = node_list[triTable[cubeindex][i  ]];
    IVEHandle n2 = node_list[triTable[cubeindex][i+1]];
    IVEHandle n3 = node_list[triTable[cubeindex][i+2]];
    // Check for duplicate nodes and remove tris as necessary.
    if(n1 != n2 && n2 != n3 && n1 != n3)
    {
      IVEHandle new_tri = mesh_api_out->new_tri(n1, n2, n3, false, hex);
      mesh_api->copy_element_output_fields(hex, new_tri, mesh_api_out);
      optimized_tris.push_back(new_tri);
    }
  }
}

void IsoVolumeExtractionTool::create_boundary_tris_for_hex(const IVEHandle &cur_hex,
                                                           std::map<IVEHandle,double> &nodal_var_map,
                                                           std::multimap<IVEHandle,MeshEdge*> &edge_map,
                                                           std::map<IVEHandle,IVEHandle> &node_map,
                                                           std::vector<IVEHandle> &fixed_tris,
                                                           const double &iso_level,
                                                           IVEMeshAPI *mesh_api,
                                                           IVEMeshAPI *mesh_api_out,
                                                           bool only_create_boundary_info,
                                                           std::vector<BoundaryNodeInfo> &boundary_node_info,
                                                           std::set<IVEHandle> &boundary_nodes)
{
  for(int i=0; i<6; i++)
  {
    IVEHandle face_nodes[4];
    mesh_api->hex_quad_nodes(cur_hex, i, face_nodes);

    if(!mesh_api->get_connected_hex(cur_hex,
                                    face_nodes[0],
                                    face_nodes[1],
                                    face_nodes[2],
                                    face_nodes[3]))
    {
      double vals[4];
      for(int j=0; j<4; j++)
        vals[j] = nodal_var_map[face_nodes[j]];
      IVEHandle n1 = face_nodes[0];
      double val1 = vals[0];
      std::vector<IVEHandle> node_list;
      BoundaryNodeInfo bni;
      for(int j=0; j<4; j++)
      {
        IVEHandle n2 = face_nodes[(j+1)%4];
        double val2 = vals[(j+1)%4];
        if(val1 > iso_level)
        {
          if(val2 > iso_level)
          {
            if(only_create_boundary_info)
            {  
              if(boundary_nodes.find(n2) != boundary_nodes.end())
              {
                get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, true, bni);
                boundary_node_info.push_back(bni);
              }
            }
            else
            {
              IVEHandle new_node = get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, false, bni);
              node_list.push_back(new_node);
            }
          }
          else if (val2 < iso_level)
          {
            if(only_create_boundary_info)
            {  
/*
              if(boundary_nodes.find(n2) != boundary_nodes.end() &&
                 boundary_nodes.find(n1) != boundary_nodes.end())
              {
              }
*/
            }
            else
            {
              node_list.push_back(get_mid_node(n1, n2, val1, val2, iso_level, edge_map, 
                                            node_map, mesh_api, mesh_api_out, false, bni));
            }
          }
          else
          {
            if(only_create_boundary_info)
            {  
              if(boundary_nodes.find(n2) != boundary_nodes.end())
              {
                get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, true, bni);
                boundary_node_info.push_back(bni);
              }
            }
            else
            {
              IVEHandle new_node = get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, false, bni);
              node_list.push_back(new_node);
            }
          }
        }
        else if (val1 < iso_level)
        {
          if(val2 > iso_level)
          {
            if(only_create_boundary_info)
            {  
              if(boundary_nodes.find(n2) != boundary_nodes.end())
              {
                get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, true, bni);
                boundary_node_info.push_back(bni);
              }
            }
            else
            {
              node_list.push_back(get_mid_node(n1, n2, val1, val2, iso_level, 
                      edge_map, node_map, mesh_api, mesh_api_out, false, bni));
              IVEHandle new_node = get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, false, bni);
              node_list.push_back(new_node);
            }
          }
          else if (val2 < iso_level)
          {
          }
          else
          {
            if(only_create_boundary_info)
            {  
              if(boundary_nodes.find(n2) != boundary_nodes.end())
              {
                get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, true, bni);
                boundary_node_info.push_back(bni);
              }
            }
            else
            {
              IVEHandle new_node = get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, false, bni);
              node_list.push_back(new_node);
            }
          }
        }
        else
        {
          if(val2 > iso_level)
          {
            if(only_create_boundary_info)
            {  
              if(boundary_nodes.find(n2) != boundary_nodes.end())
              {
                get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, true, bni);
                boundary_node_info.push_back(bni);
              }
            }
            else
            {
              IVEHandle new_node = get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, false, bni);
              node_list.push_back(new_node);
            }
          }
          else if (val2 < iso_level)
          {
          }
          else
          {
            if(only_create_boundary_info)
            {  
              if(boundary_nodes.find(n2) != boundary_nodes.end())
              {
                get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, true, bni);
                boundary_node_info.push_back(bni);
              }
            }
            else
            {
              IVEHandle new_node = get_duplicate_node(n2, node_map, mesh_api, mesh_api_out, false, bni);
              node_list.push_back(new_node);
            }
          }
        }
        n1 = n2;
        val1 = val2;
      }
      
      if(!only_create_boundary_info)
      {
        int iter_count = node_list.size() - 1;
        for(int j=2; j<=iter_count; j++)
        {
          IVEHandle tmp_n3 = node_list[j];
          IVEHandle tmp_n2 = node_list[j-1];
          IVEHandle tmp_n1 = node_list[0];
          IVEHandle new_tri = mesh_api_out->new_tri(tmp_n1, tmp_n2, tmp_n3, true, cur_hex);
          mesh_api->copy_element_output_fields(cur_hex, new_tri, mesh_api_out);
          fixed_tris.push_back(new_tri);
        }
      }
    }
  }
}

// This function assumes that any nodal values that are "very close"
// to the iso value have already been "snapped" to be exactly
// on the iso value.
IVEHandle IsoVolumeExtractionTool::get_mid_node(const IVEHandle &n1,
                                              const IVEHandle &n2,
                                              const double &val1,
                                              const double &val2,
                                              const double &iso_val,
                                              std::multimap<IVEHandle, MeshEdge*> &edge_map,
                                              std::map<IVEHandle, IVEHandle> &node_map,
                                              IVEMeshAPI *mesh_api,
                                              IVEMeshAPI *mesh_api_out,
                                              bool boundary_info,
                                              BoundaryNodeInfo &bni)
{
  IVEHandle ret = 0;
  IVEHandle zero = 0;
  if(boundary_info)
  {
    if(val1 == iso_val)
    {
      bni.existing_node_id = n1;
      bni.new_node_pos = mesh_api->node_coordinates(n1);
      bni.suggested_new_node_id = mesh_api_out->get_new_node_id();
      if(n1 < n2)
      {
        bni.mu = 0.0;
        mesh_api->set_min_node_id(bni, n1);
        mesh_api->set_max_node_id(bni, n2);
      }
      else
      {
        bni.mu = 1.0;
        mesh_api->set_min_node_id(bni, n2);
        mesh_api->set_max_node_id(bni, n1);
      }
      return 1;
    }
    else if(val2 == iso_val)
    {
      bni.existing_node_id = n2;
      bni.new_node_pos = mesh_api->node_coordinates(n2);
      bni.suggested_new_node_id = mesh_api_out->get_new_node_id();
      if(n1 < n2)
      {
        bni.mu = 1.0;
        mesh_api->set_min_node_id(bni, n1);
        mesh_api->set_max_node_id(bni, n2);
      }
      else
      {
        bni.mu = 0.0;
        mesh_api->set_min_node_id(bni, n2);
        mesh_api->set_max_node_id(bni, n1);
      }
      return 1;
    }
    else if(fabs(val1-val2) < 1e-6)
    {
      bni.existing_node_id = 0;
      mesh_api->set_min_node_id(bni, zero);
      mesh_api->set_max_node_id(bni, zero);
      return 0;
    }
    
/*
    if(val1 == iso_val || val2 == iso_val || fabs(val1-val2) < 1e-6)
    {
      mesh_api->set_min_node_id(bni, zero);
      mesh_api->set_max_node_id(bni, zero);
      return 0;
    }
*/
    bni.existing_node_id = 0;
    const IsoVector &p1 = mesh_api->node_coordinates(n1);
    const IsoVector &p2 = mesh_api->node_coordinates(n2);
    double mu = (iso_val - val1) / (val2 - val1);
    bni.new_node_pos = p1 + mu * (p2-p1);
    bni.suggested_new_node_id = mesh_api_out->get_new_node_id();
    if(n1 < n2)
    {
      mesh_api->set_min_node_id(bni, n1);
      mesh_api->set_max_node_id(bni, n2);
      bni.mu = mu;
    }
    else
    {
      mesh_api->set_min_node_id(bni, n2);
      mesh_api->set_max_node_id(bni, n1);
      bni.mu = 1.0-mu;
    }
    return 1;
  }
  else
  {
    MeshEdge *me = find_mesh_edge(n1, n2, edge_map);
    if(me)
      ret = me->mid_node;
    else
    {
      if(val1 == iso_val)
      {
        IVEHandle new_node;
        if(node_map.count(n1))
          new_node = node_map[n1];
        else
        {
          IsoVector pos = mesh_api->node_coordinates(n1);
          new_node = mesh_api_out->new_node(pos);
          mesh_api->transfer_output_fields(n1, n1, new_node, 0.0, mesh_api_out);
          node_map[n1] = new_node;
        }
        me = new MeshEdge;
        int min_id;
        if(n1 < n2)
        {
          me->max_id_node = n2;
          min_id = n1;
        }
        else
        {
          me->max_id_node = n1;
          min_id = n2;
        }
        me->mid_node = new_node;
        edge_map.insert(std::multimap<IVEHandle,MeshEdge*>::value_type(min_id, me));
        ret = new_node;
      }
      else if(val2 == iso_val)
      {
        IVEHandle new_node;
        if(node_map.count(n2))
          new_node = node_map[n2];
        else
        {
          IsoVector pos = mesh_api->node_coordinates(n2);
          new_node = mesh_api_out->new_node(pos);
          mesh_api->transfer_output_fields(n2, n2, new_node, 0.0, mesh_api_out);
          node_map[n2] = new_node;
        }
        me = new MeshEdge;
        int min_id;
        if(n1 < n2)
        {
          me->max_id_node = n2;
          min_id = n1;
        }
        else
        {
          me->max_id_node = n1;
          min_id = n2;
        }
        me->mid_node = new_node;
        edge_map.insert(std::multimap<IVEHandle,MeshEdge*>::value_type(min_id, me));
        ret = new_node;
      }
      else if(fabs(val1-val2) < 1e-6)
        ret = 0;
/*
      if(val1 == iso_val)
        ret = n1;
      else if(val2 == iso_val)
        ret = n2;
      else if(fabs(val1-val2) < 1e-6)
        ret = -1;
*/
      else
      {
        const IsoVector &p1 = mesh_api->node_coordinates(n1);
        const IsoVector &p2 = mesh_api->node_coordinates(n2);
        double mu = (iso_val - val1) / (val2 - val1);
        IsoVector new_pos = p1 + mu * (p2-p1);
        IVEHandle new_node = mesh_api_out->new_node(new_pos);
        mesh_api->transfer_output_fields(n1, n2, new_node, mu, mesh_api_out);
        me = new MeshEdge;
        int min_id;
        if(n1 < n2)
        {
          me->max_id_node = n2;
          min_id = n1;
        }
        else
        {
          me->max_id_node = n1;
          min_id = n2;
        }
        me->mid_node = new_node;
        edge_map.insert(std::multimap<IVEHandle,MeshEdge*>::value_type(min_id, me));
        ret = new_node;
      }
    }
  }
  return ret;
}

MeshEdge* IsoVolumeExtractionTool::find_mesh_edge(const IVEHandle &n1,
                                                  const IVEHandle &n2,
                                                  std::multimap<IVEHandle,MeshEdge*> &edge_map)
{
  size_t min_id = n1;
  IVEHandle other_node = n2;
  if(n2 < min_id)
  {
    min_id = n2;
    other_node = n1;
  }
  std::multimap<IVEHandle,MeshEdge*>::iterator itlow, ithi, it;
  itlow = edge_map.lower_bound(min_id);
  ithi = edge_map.upper_bound(min_id);
  for(it=itlow; it!=ithi; ++it)
  {
    if(it->second->max_id_node == other_node)
      return it->second;
  }
  return NULL;
}




