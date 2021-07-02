/*
 * PerceptPruneTool.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: tzirkle
 */

#include "PruneMeshAPI.hpp"
#include "PruneMeshAPISTK.hpp"
#include "PerceptPruneTool.hpp"
#include <stk_util/parallel/ParallelReduce.hpp>
#include <stk_util/parallel/CommSparse.hpp>
#include <stk_mesh/base/MetaData.hpp>
#include <algorithm>

#include <STKMeshTransferSetup.hpp>
#include <StringFunction.hpp>
#include <FieldFunction.hpp>
#include <Percept.hpp>
#include <Util.hpp>
#include <ExceptionWatch.hpp>
#include <GeometryVerifier.hpp>
#include <StringFunction.hpp>
#include <FieldFunction.hpp>
#include <ConstantFunction.hpp>
#include <PerceptMesh.hpp>
#include "MeshManager.hpp"

void PerceptPruneTool::prune_percept_mesh(const std::vector<PruneHandle> &elem_list,
                               std::set<PruneHandle> &elems_to_keep,
                               int num_buffer_layers,
                               int allow_nonmanifold_connections,
                               int remove_islands,
                               MeshManager &tMeshManager)
{
  percept::PerceptMesh * mesh_api = &tMeshManager.get_output_percept();

  int num_elems = elem_list.size();
  std::map<stk::mesh::EntityId, int> node_vals, elem_vals;


  ////////////////////////////////////////////////
  // Calculate sharing maps for this processor
  ////////////////////////////////////////////////

  std::vector<proc_node_map> procs;
  calculate_percept_sharing_maps(procs, tMeshManager);

  ////////////////////////////////////////////////
  // Find elems to keep
  ////////////////////////////////////////////////

  find_initial_set_of_percept_elems_to_keep(elem_list, num_buffer_layers,
          tMeshManager, node_vals, elem_vals, procs);

  ////////////////////////////////////////////////
  // Do initial communication of boundary info
  ////////////////////////////////////////////////

  communicate_percept_boundary_info(procs, node_vals, elem_vals, mesh_api);

  if(remove_islands)
  {
    ////////////////////////////////////////////////
    // Make sure we have one connected set of
    // elements.  Otherwise the physics may bomb.
    ////////////////////////////////////////////////

    std::set<stk::mesh::EntityId> elem_set;
    for(int u=0; u<num_elems; ++u)
    {
      PruneHandle ph = elem_list[u];
//      stk::mesh::EntityId ent_id = mesh_api->entity_id(ph);
      stk::mesh::EntityId ent_id = tMeshManager.entity_id(ph);
      elem_set.insert(ent_id);
    }
    ensure_connected_set_of_percept_elements(elem_set, mesh_api,
            node_vals, elem_vals, procs);
  }

  if(!allow_nonmanifold_connections)
  {
    ////////////////////////////////////////////////
    // Set up initial list of elems to check for
    // nonmanifold connections.
    ////////////////////////////////////////////////

    std::vector<stk::mesh::Entity> elems_to_check_in, elems_to_check_out;
    for(int i=0; i<num_elems; ++i)
    {
      PruneHandle cur_elem = elem_list[i];
//      stk::mesh::Entity elem_ent = mesh_api->get_stk_entity(cur_elem);
      stk::mesh::Entity elem_ent = tMeshManager.get_stk_entity(cur_elem);
//      stk::mesh::EntityId elem_id = mesh_api->entity_id(cur_elem);
      stk::mesh::EntityId elem_id = tMeshManager.entity_id(cur_elem);
      if(elem_vals[elem_id])
        elems_to_check_in.push_back(elem_ent);
    }

    ////////////////////////////////////////////////
    // Loop until all non-manifold cases are resolved
    ////////////////////////////////////////////////
    if(mesh_api->get_bulk_data()->parallel_rank() == 0)
    {
      std::cout << std::endl;
      std::cout << "Looking for non-manifold connections. " << std::endl;
      std::cout << std::endl;
    }

    int cntr=1;
    while(nonmanifold_percept_connection_exists(node_vals, elem_vals, elems_to_check_in,
                elems_to_check_out, mesh_api,tMeshManager))
    {
      if(mesh_api->get_bulk_data()->parallel_rank() == 0)
      {
        std::cout << "Found non-manifold connections on pass " << cntr << "." << std::endl;
      }
      cntr++;
      communicate_percept_boundary_info(procs, node_vals, elem_vals, mesh_api);
      elems_to_check_in = elems_to_check_out;
      elems_to_check_out.clear();
    }
    if(mesh_api->get_bulk_data()->parallel_rank() == 0)
      std::cout << std::endl;
  }

  ////////////////////////////////////////////////
  // Make the final list of elems to keep
  ////////////////////////////////////////////////

  for(int i=0; i<num_elems; ++i)
  {
    PruneHandle cur_elem = elem_list[i];
//    stk::mesh::EntityId elem_id = mesh_api->entity_id(cur_elem);
    stk::mesh::EntityId elem_id = tMeshManager.entity_id(cur_elem);
    if(elem_vals[elem_id])
      elems_to_keep.insert(cur_elem);
  }

  int my_num_elems_to_keep, total_num_elems_to_keep;
  my_num_elems_to_keep = elems_to_keep.size();
  MPI_Reduce(&my_num_elems_to_keep,
                &total_num_elems_to_keep, 1,
                sierra::MPI::Datatype<int>::type(),
                MPI_SUM, 0, mesh_api->get_bulk_data()->parallel());
  if(mesh_api->get_bulk_data()->parallel_rank() == 0)
  {
    printf("\nTotal # of elements after pruning: %d\n\n", total_num_elems_to_keep);
  }
}

void PerceptPruneTool::grow_percept_smallest_group(
                       std::vector<elem_group*> &groups,
                       const std::set<stk::mesh::EntityId> &elem_set,
                       percept::PerceptMesh *mesh_api,
                       std::map<stk::mesh::EntityId, int> &node_vals,
                       std::map<stk::mesh::EntityId, int> &elem_vals,
                       std::vector<proc_node_map> &procs,
                       std::set<stk::mesh::EntityId> &unique_group_ids)
{
  // First we need to decide what the smallest group is
  int num_unique_ids = unique_group_ids.size();
  int global_array[num_unique_ids];
  int local_array[num_unique_ids];

  for(int i=0; i<num_unique_ids; ++i)
  {
    global_array[i] = 0;
    local_array[i] = 0;
  }
  int cntr=0;
  std::set<stk::mesh::EntityId>::iterator it = unique_group_ids.begin();
  while(it != unique_group_ids.end())
  {
    for(size_t i=0; i<groups.size(); ++i)
    {
      if(groups[i]->id == *it)
      {
        // Find just the locally owned elems in this group
        int count = 0;
        std::set<stk::mesh::EntityId>::iterator elem_it = groups[i]->elems.begin();
        while(elem_it != groups[i]->elems.end())
        {
          if(elem_set.find(*elem_it) != elem_set.end())
            ++count;
          ++elem_it;
        }
        local_array[cntr] = count;
        break;
      }
    }
    cntr++;
    ++it;
  }

  MPI_Allreduce(local_array,
                global_array, num_unique_ids,
                sierra::MPI::Datatype<int>::type(),
                MPI_SUM, mesh_api->get_bulk_data()->parallel());

  int smallest_index = 0;
  int smallest_count = global_array[0];
  for(int i=1; i<num_unique_ids; ++i)
  {
    if(smallest_count > global_array[i])
    {
      smallest_count = global_array[i];
      smallest_index = i;
    }
  }
  it = unique_group_ids.begin();
  for(int i=0; i<smallest_index; ++i)
    ++it;
  stk::mesh::EntityId smallest_group = *it;

  for(size_t i=0; i<groups.size(); ++i)
  {
    if(groups[i]->id == smallest_group)
    {
      it = groups[i]->elems.begin();
      while(it != groups[i]->elems.end())
      {
        stk::mesh::EntityId cur_elem_id = *it;
        stk::mesh::Entity cur_elem = mesh_api->get_bulk_data()->get_entity(
                 stk::topology::ELEMENT_RANK, cur_elem_id);
        stk::mesh::Entity const *elem_nodes = mesh_api->get_bulk_data()->begin_nodes(cur_elem);
        int num_nodes = mesh_api->get_bulk_data()->num_nodes(cur_elem);
        for(int j=0; j<num_nodes; ++j)
        {
          stk::mesh::Entity cur_node = elem_nodes[j];
          stk::mesh::Entity const *node_elems = mesh_api->
                            get_bulk_data()->begin_elements(cur_node);
          int num_elems = mesh_api->get_bulk_data()->num_elements(cur_node);
          for(int k=0; k<num_elems; ++k)
          {
            stk::mesh::Entity cur_node_elem = node_elems[k];
            stk::mesh::EntityId cur_node_elem_id = mesh_api->get_bulk_data()->identifier(cur_node_elem);
            elem_vals[cur_node_elem_id] = 1;
            stk::mesh::Entity const *elem_nodes2 =
                         mesh_api->get_bulk_data()->begin_nodes(cur_node_elem);
            int num_nodes2 = mesh_api->get_bulk_data()->num_nodes(cur_node_elem);
            for(int m=0; m<num_nodes2; ++m)
            {
              stk::mesh::Entity cur_node2 = elem_nodes2[m];
              stk::mesh::EntityId nid = mesh_api->get_bulk_data()->identifier(cur_node2);
              node_vals[nid] = 1;
            }
          }
        }
        ++it;
      }
      break;
    }
  }
}

void PerceptPruneTool::find_percept_local_groups(
                       const std::set<stk::mesh::EntityId> &elem_set,
                       std::map<stk::mesh::EntityId, int> &elem_vals,
                       std::vector<elem_group*> &groups,
                       percept::PerceptMesh *mesh_api)
{
  bool done = false;
  std::map<stk::mesh::EntityId, int> processed_map;

  // Initialize processed map to zeros
  std::set<stk::mesh::EntityId>::iterator set_it = elem_set.begin();
  while(set_it != elem_set.end())
  {
    processed_map[*set_it] = 0;
    ++set_it;
  }

  while(!done)
  {
    done = true;
    stk::mesh::EntityId start_elem = 0;
    // Look for a flagged elem that has not been processed.
    set_it = elem_set.begin();
    while(set_it != elem_set.end())
    {
      if(elem_vals[*set_it] && !processed_map[*set_it])
      {
        done = false;
        start_elem = *set_it;
        break;
      }
      ++set_it;
    }
    if(start_elem)
    {
      elem_group *new_group = new elem_group;
      groups.push_back(new_group);
      std::set<stk::mesh::EntityId> elems_to_process;
      elems_to_process.insert(start_elem);
      while(!elems_to_process.empty())
      {
        // get the last entry in the set in case it is more efficient
        // to erase this one
        std::set<stk::mesh::EntityId>::iterator it = elems_to_process.end();
        --it;
        stk::mesh::EntityId cur_elem_id = *it;
        processed_map[cur_elem_id] = 1;
        elems_to_process.erase(it);
        new_group->elems.insert(cur_elem_id);
        stk::mesh::Entity cur_elem = mesh_api->get_bulk_data()->get_entity(
                               stk::topology::ELEMENT_RANK, cur_elem_id);
        stk::mesh::Entity const *nodes = mesh_api->get_bulk_data()->begin_nodes(cur_elem);
        int num_nodes = mesh_api->get_bulk_data()->num_nodes(cur_elem);

        for(int i=0; i<num_nodes; ++i)
        {
          stk::mesh::Entity node_entity = nodes[i];
          stk::mesh::Entity const *node_elems = mesh_api->
                            get_bulk_data()->begin_elements(node_entity);
          int num_elems = mesh_api->get_bulk_data()->num_elements(node_entity);
          for(int j=0; j<num_elems; ++j)
          {
            stk::mesh::Entity cur_node_elem = node_elems[j];
            stk::mesh::EntityId elem_id = mesh_api->get_bulk_data()->identifier(cur_node_elem);
            // Make sure hex is locally owned
            if(cur_node_elem != cur_elem )
            {
              if(elem_vals.count(elem_id) && elem_vals[elem_id] &&
                   !processed_map[elem_id])
              {
                elems_to_process.insert(elem_id);
              }
            }
          }
        }
      }
      // Now find the lowest global element id in this group and set the group id to it
      std::set<stk::mesh::EntityId>::iterator it = new_group->elems.begin();
      stk::mesh::EntityId min_id = *it;
      ++it;
      while(it != new_group->elems.end())
      {
        stk::mesh::EntityId cur_id = *it;
        if(min_id > cur_id)
          min_id = cur_id;
        ++it;
      }
      new_group->id = min_id;
    }
  }
}


void PerceptPruneTool::find_percept_local_equivs(std::vector<proc_node_map> &procs,
              std::map<stk::mesh::EntityId,std::set<stk::mesh::EntityId> > &my_equivs,
              std::vector<elem_group*> &groups,
              percept::PerceptMesh *mesh_api)
{
  if(procs.size() > 0)
  {
    stk::CommSparse comm_spec(mesh_api->get_bulk_data()->parallel());
    for(int phase=0; phase<2; ++phase)
    {
      for(size_t i=0; i<procs.size(); ++i)
      {
        int other_proc = procs[i].processor;
        // pack the number of elems
        comm_spec.send_buffer(other_proc).pack<size_t>(procs[i].elements.size());
        for(size_t j=0; j<procs[i].elements.size(); ++j)
        {
          // pack the element id and then the group id for this element
          comm_spec.send_buffer(other_proc).pack<stk::mesh::EntityId>
                  (procs[i].elements[j]);
          stk::mesh::EntityId group_id = 0;
          for(size_t k=0; k<groups.size(); ++k)
          {
            if(groups[k]->elems.find(procs[i].elements[j]) != groups[k]->elems.end())
            {
              group_id = groups[k]->id;
              break;
            }
          }
          comm_spec.send_buffer(other_proc).pack<int>(group_id);
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

    for(size_t i=0; i<procs.size(); ++i)
    {
      int other_proc = procs[i].processor;
      // get the number of elements from the sending proc
      size_t num_elems;
      comm_spec.recv_buffer(other_proc).unpack<size_t>(num_elems);
      for(size_t j=0; j<num_elems; ++j)
      {
        stk::mesh::EntityId elem_id;
        int group_id;
        comm_spec.recv_buffer(other_proc).unpack<stk::mesh::EntityId>(elem_id);
        comm_spec.recv_buffer(other_proc).unpack<int>(group_id);
        if(group_id > 0)
        {
          for(size_t k=0; k<groups.size(); ++k)
          {
            if(groups[k]->elems.find(elem_id) != groups[k]->elems.end())
            {
              if(groups[k]->id != (stk::mesh::EntityId)group_id)
              {
                if(!my_equivs.count(groups[k]->id))
                {
                  std::set<stk::mesh::EntityId> new_set;
                  new_set.insert(group_id);
                  my_equivs[groups[k]->id] = new_set;
                }
                else
                {
                  my_equivs[groups[k]->id].insert(group_id);
                }
              }
            }
          }
        }
      }
    }
  }
}

void PerceptPruneTool::get_percept_global_equiv_info(
              std::map<stk::mesh::EntityId, std::set<stk::mesh::EntityId> > &local_equivs,
              std::map<stk::mesh::EntityId, std::set<stk::mesh::EntityId> > &global_equivs,
              percept::PerceptMesh *mesh_api)
{
  // Determine how much info each processor will send.
  int num_procs = mesh_api->get_bulk_data()->parallel_size();
  int my_rank = mesh_api->get_bulk_data()->parallel_rank();
  int global_counts [num_procs];
  int local_counts[num_procs];
  for(int i=0; i<num_procs; ++i)
  {
    global_counts[i] = 0;
    local_counts[i] = 0;
  }
  int cnt = 0;
  std::map<stk::mesh::EntityId,std::set<stk::mesh::EntityId> >::iterator it = local_equivs.begin();
  while(it != local_equivs.end())
  {
    cnt += (2 * (it->second.size()));
    ++it;
  }

  local_counts[my_rank] = cnt;

  MPI_Allreduce(local_counts,
                global_counts, num_procs,
                sierra::MPI::Datatype<int>::type(),
                MPI_MAX, mesh_api->get_bulk_data()->parallel());

  // Now get the equiv info from each processor.
  int total_num = 0;
  int my_start_index = 0;
  for(int i=0; i<num_procs; ++i)
  {
    total_num += global_counts[i];
    if(i < my_rank)
      my_start_index += global_counts[i];
  }

  int global_equiv_pairs[total_num];
  int local_equiv_pairs[total_num];
  for(int i=0; i<total_num; ++i)
  {
    global_equiv_pairs[i] = 0;
    local_equiv_pairs[i] = 0;
  }
  it = local_equivs.begin();
  while(it != local_equivs.end())
  {
    std::set<stk::mesh::EntityId>::iterator it2 = it->second.begin();
    while(it2 != it->second.end())
    {
      local_equiv_pairs[my_start_index] = it->first;
      my_start_index++;
      local_equiv_pairs[my_start_index] = *it2;
      my_start_index++;
      ++it2;
    }
    ++it;
  }

  MPI_Allreduce(local_equiv_pairs,
                global_equiv_pairs, total_num,
                sierra::MPI::Datatype<int>::type(),
                MPI_MAX, mesh_api->get_bulk_data()->parallel());

  // Now we should have a list of all of the pairs of
  // groups ids that are equivalent
  int num_pairs = total_num/2;
  for(int i=0; i<num_pairs; ++i)
  {
    stk::mesh::EntityId id1 = global_equiv_pairs[2*i];
    stk::mesh::EntityId id2 = global_equiv_pairs[2*i+1];
    if(!global_equivs.count(id1))
    {
      std::set<stk::mesh::EntityId> new_set;
      new_set.insert(id2);
      global_equivs[id1] = new_set;
    }
    else
    {
      global_equivs[id1].insert(id2);
    }
  }
}

void PerceptPruneTool::recursively_consolidate_groups(stk::mesh::EntityId key,
                             std::map<stk::mesh::EntityId,
                             std::set<stk::mesh::EntityId> > &global_equivs,
                             std::set<stk::mesh::EntityId> &new_set)
{
  if(global_equivs.count(key))
  {
    std::set<stk::mesh::EntityId> copy = global_equivs[key];
    global_equivs.erase(key);
    std::set<stk::mesh::EntityId>::iterator it = copy.begin();
    while(it != copy.end())
    {
      new_set.insert(*it);
      recursively_consolidate_groups(*it, global_equivs, new_set);
      ++it;
    }
  }
}

void PerceptPruneTool::consolidate_percept_groups(
              std::map<stk::mesh::EntityId,
              std::set<stk::mesh::EntityId> > &global_equivs,
              std::vector<elem_group*> &groups,
              percept::PerceptMesh *mesh_api)
{
  std::vector<std::set<stk::mesh::EntityId> > unique_group_sets;
  while(!global_equivs.empty())
  {
    std::map<stk::mesh::EntityId, std::set<stk::mesh::EntityId> >::iterator it = global_equivs.begin();
    std::set<stk::mesh::EntityId> new_set;
    new_set.insert(it->first);
    recursively_consolidate_groups(it->first, global_equivs, new_set);
    unique_group_sets.push_back(new_set);
  }

  // Use global equivalencing to find the lowest group ids for my groups.
  for(size_t i=0; i<groups.size(); ++i)
  {
    stk::mesh::EntityId cur_id = groups[i]->id;
    for(size_t j=0; j<unique_group_sets.size(); ++j)
    {
      if(unique_group_sets[j].find(cur_id) != unique_group_sets[j].end())
      {
        groups[i]->id = *(unique_group_sets[j].begin());
        break;
      }
    }
  }

  // If I have any groups with the same ids consolidate them.
  size_t num_groups = groups.size();
  for(size_t i=0; i<num_groups; ++i)
  {
    stk::mesh::EntityId id1 = groups[i]->id;
    for(size_t j=i+1; j<num_groups; ++j)
    {
      stk::mesh::EntityId id2 = groups[j]->id;
      if(id1 == id2)
      {
        std::set<stk::mesh::EntityId>::iterator it = groups[j]->elems.begin();
        while(it != groups[j]->elems.end())
        {
          groups[i]->elems.insert(*it);
          ++it;
        }
        delete groups[j];
        for(size_t k=j; k<num_groups-1; k++)
          groups[k] = groups[k+1];
        std::vector<elem_group*>::iterator it2 = groups.end();
        --it2;
        groups.erase(it2);
        num_groups--;
        j--;
      }
    }
  }
}


int PerceptPruneTool::number_of_percept_groups(
                       std::vector<elem_group*> &groups,
                       const std::set<stk::mesh::EntityId> &elem_set,
                       percept::PerceptMesh *mesh_api,
                       std::map<stk::mesh::EntityId, int> &node_vals,
                       std::map<stk::mesh::EntityId, int> &elem_vals,
                       std::vector<proc_node_map> &procs,
                       std::set<stk::mesh::EntityId> &unique_group_ids)
{
  int num_groups = 0;

  // Find local groups and assign ids by lowest global element id.
  find_percept_local_groups(elem_set, elem_vals, groups, mesh_api);

  // Find group equivalences that I know about by communicating with my
  // neighbors.  A group equivalence is simply a group on a neighbor
  // that is connected with one of my groups.  At this point they will
  // have different ids but we will know that the ids should be equivalent
  // and will send that info out to all processors.
  std::map<stk::mesh::EntityId, std::set<stk::mesh::EntityId> > my_equivs;
  find_percept_local_equivs(procs, my_equivs, groups, mesh_api);

  std::map<stk::mesh::EntityId, std::set<stk::mesh::EntityId> > global_equivs;
  get_percept_global_equiv_info(my_equivs, global_equivs, mesh_api);

  consolidate_percept_groups(global_equivs, groups, mesh_api);

  // Communicate globally to determine how many total groups we have.

  // Determine how many group ids each processor will send.
  int num_procs = mesh_api->get_bulk_data()->parallel_size();
  int my_rank = mesh_api->get_bulk_data()->parallel_rank();
  int global_group_counts [num_procs];
  int local_group_counts[num_procs];
  for(int i=0; i<num_procs; ++i)
  {
    global_group_counts[i] = 0;
    local_group_counts[i] = 0;
  }
  local_group_counts[my_rank] = groups.size();

  MPI_Allreduce(local_group_counts,
                global_group_counts, num_procs ,
                sierra::MPI::Datatype<int>::type(),
                MPI_MAX, mesh_api->get_bulk_data()->parallel());

  // Now get the groups ids from each processor.
  int total_num_ids = 0;
  int my_start_index = 0;
  for(int i=0; i<num_procs; ++i)
  {
    total_num_ids += global_group_counts[i];
    if(i < my_rank)
      my_start_index += global_group_counts[i];
  }

  int global_id_list[total_num_ids];
  int local_id_list[total_num_ids];
  for(int i=0; i<total_num_ids; ++i)
  {
    global_id_list[i] = 0;
    local_id_list[i] = 0;
  }
  for(size_t i=0; i<groups.size(); ++i)
    local_id_list[my_start_index+i] = groups[i]->id;

  MPI_Allreduce(local_id_list,
                global_id_list, total_num_ids,
                sierra::MPI::Datatype<int>::type(),
                MPI_MAX, mesh_api->get_bulk_data()->parallel());

  // Now we should have a list of all of the groups ids
  // known by each processor.  Loop through and see how
  // many unique ids we have.
  for(int i=0; i<total_num_ids; ++i)
    unique_group_ids.insert(global_id_list[i]);

  num_groups = unique_group_ids.size();

  return num_groups;
}

void PerceptPruneTool::ensure_connected_set_of_percept_elements(
                       const std::set<stk::mesh::EntityId> &elem_set,
                       percept::PerceptMesh *mesh_api,
                       std::map<stk::mesh::EntityId, int> &node_vals,
                       std::map<stk::mesh::EntityId, int> &elem_vals,
                       std::vector<proc_node_map> &procs)
{
  if(mesh_api->get_bulk_data()->parallel_rank() == 0)
  {
    std::cout << std::endl;
    std::cout << "Ensuring that the pruned mesh is not disjoint." << std::endl;
    std::cout << std::endl;
  }

  std::vector<elem_group*> my_groups;
  // Do analysis to see if we have more than one set of hexes to keep.
  // If we have more than one we will grow the smallest one until
  // it connects with another group and continue this process until
  // we only have one connected group.
  std::set<stk::mesh::EntityId> unique_group_ids;
  while(number_of_percept_groups(my_groups, elem_set, mesh_api, node_vals,
                         elem_vals, procs, unique_group_ids) > 1)
  {
    if(mesh_api->get_bulk_data()->parallel_rank() == 0)
    {
      std::cout << "Found " << unique_group_ids.size() << " disjoint regions of mesh." << std::endl;
      std::cout << "Growing the smallest region." << std::endl;
    }

    grow_percept_smallest_group(my_groups, elem_set, mesh_api, node_vals,
                         elem_vals, procs, unique_group_ids);

    // Let the other procs know what values have been set on processor boundaries.
    communicate_percept_boundary_elem_info(procs, elem_vals, mesh_api);

    // Cean out data structures we created on this last iteration.
    while(my_groups.size())
    {
      delete my_groups.back();
      my_groups.pop_back();
    }
    unique_group_ids.clear();
  }
  while(my_groups.size())
  {
    delete my_groups.back();
    my_groups.pop_back();
  }
  if(mesh_api->get_bulk_data()->parallel_rank() == 0)
  {
    std::cout << std::endl;
    std::cout << "Returning 1 non-disjoint mesh region."  << std::endl;
  }
}

int PerceptPruneTool::nonmanifold_percept_connection_exists(
            std::map<stk::mesh::EntityId, int> &node_vals,
            std::map<stk::mesh::EntityId, int> &elem_vals,
            std::vector<stk::mesh::Entity> &elems_to_check_in,
            std::vector<stk::mesh::Entity> &elems_to_check_out,
            percept::PerceptMesh *mesh_api,MeshManager &tMeshManager)
{
  int local_has_nonmanifold_connection = 0;

  std::map<stk::mesh::Entity, int> node_flags;
  // Set flags for all nodes in elems to be processed to 0 (not processed yet).
  // Actually, I am changing this so that only owned or shared nodes
  // are processed.  Others are just set to 1 (so we won't
  // process them).
  stk::mesh::Selector sel(mesh_api->get_fem_meta_data()->locally_owned_part() |
                          mesh_api->get_fem_meta_data()->globally_shared_part());
  std::set<stk::mesh::Entity> owned_or_shared_nodes;
  const stk::mesh::BucketVector &node_buckets = mesh_api->get_bulk_data()->get_buckets(stk::topology::NODE_RANK, sel);
  for(size_t i=0; i<node_buckets.size(); ++i)
  {
    for(size_t j=0; j<node_buckets[i]->size(); ++j)
    {
      stk::mesh::Entity cur_node = (*(node_buckets[i]))[j];
      owned_or_shared_nodes.insert(cur_node);
    }
  }

  for(size_t i=0; i<elems_to_check_in.size(); ++i)
  {
    stk::mesh::Entity cur_elem = elems_to_check_in[i];
    stk::mesh::Entity const *elem_nodes = mesh_api->get_bulk_data()->begin_nodes(cur_elem);
    int num_nodes = mesh_api->get_bulk_data()->num_nodes(cur_elem);
    for(int j=0; j<num_nodes; ++j)
    {
      stk::mesh::Entity cur_node = elem_nodes[j];
      if(owned_or_shared_nodes.find(cur_node) != owned_or_shared_nodes.end())
        node_flags[cur_node] = 0;
      else
        node_flags[cur_node] = 1;
    }
  }

  // Start main processing loop
  for(size_t i=0; i<elems_to_check_in.size(); ++i)
  {
    stk::mesh::Entity cur_elem_to_check = elems_to_check_in[i];
    stk::mesh::Entity const *elem_nodes = mesh_api->get_bulk_data()->begin_nodes(cur_elem_to_check);
    int num_nodes = mesh_api->get_bulk_data()->num_nodes(cur_elem_to_check);
    for(int j=0; j<num_nodes; ++j)
    {
      stk::mesh::Entity cur_node = elem_nodes[j];
      // If this node has not been processed yet...
      if(!node_flags[cur_node])
      {
        std::vector<stk::mesh::Entity> processed_elems, elems_to_process, connected_elems;
        node_flags[cur_node] = 1;
        stk::mesh::Entity const *node_elems = mesh_api->
                            get_bulk_data()->begin_elements(cur_node);
        int num_elems = mesh_api->get_bulk_data()->num_elements(cur_node);
        std::vector<stk::mesh::Entity> elems_to_keep;
        // Prune list of elems on this node to only include those
        // that are in the part of the mesh we are keeping.
        for(int k=0; k<num_elems; ++k)
        {
          stk::mesh::Entity tmp_cur_elem = node_elems[k];
          stk::mesh::EntityId elem_id = mesh_api->get_bulk_data()->identifier(tmp_cur_elem);
          // If this elem is in the area to keep...
          if(elem_vals[elem_id])
            elems_to_keep.push_back(tmp_cur_elem);
        }
        elems_to_process.push_back(cur_elem_to_check);
        connected_elems.push_back(cur_elem_to_check);
        while(elems_to_process.size() > 0 && connected_elems.size() < elems_to_keep.size())
        {
          stk::mesh::Entity cur_node_elem = elems_to_process.back();
          elems_to_process.pop_back();
          processed_elems.push_back(cur_node_elem);
          stk::topology elem_top = mesh_api->get_bulk_data()->bucket(cur_node_elem).topology();
          int num_faces = elem_top.num_faces();
          for(int face = 0; face < num_faces; ++face)
          {
            stk::topology face_top = elem_top.face_topology(face);
            int num_face_nodes = face_top.num_nodes();
            // Get the nodes on this face of the elem
            std::vector<stk::mesh::Entity> face_nodes(num_face_nodes);
            const stk::mesh::Entity* entity_nodes = mesh_api->get_bulk_data()->begin_nodes(cur_node_elem);
            elem_top.face_nodes(entity_nodes, face, face_nodes.begin());
            if(std::find(face_nodes.begin(), face_nodes.end(), cur_node) != face_nodes.end())
            {
              stk::mesh::Entity connected_elem;
//              int found_connected_elem = mesh_api->get_connected_elem(cur_node_elem,
//                           face_nodes, connected_elem);
              int found_connected_elem = tMeshManager.get_connected_elem(cur_node_elem,
                                         face_nodes, connected_elem);
              if(found_connected_elem)
              {
                stk::mesh::EntityId conn_id = mesh_api->get_bulk_data()->identifier(connected_elem);
                if(elem_vals[conn_id])
                {
                  if(std::find(connected_elems.begin(), connected_elems.end(), connected_elem) ==
                      connected_elems.end())
                  {
                    connected_elems.push_back(connected_elem);
                  }
                  if(std::find(processed_elems.begin(), processed_elems.end(), connected_elem) ==
                      processed_elems.end())
                  {
                    elems_to_process.push_back(connected_elem);
                  }
                }
              }
            }
          }
        }
        if(connected_elems.size() < elems_to_keep.size())
        {
          local_has_nonmanifold_connection = 1;
          // Add all elems hooked to this node to the area to keep.
          for(int k=0; k<num_elems; ++k)
          {
            stk::mesh::Entity cur_elem = node_elems[k];
            elems_to_check_out.push_back(cur_elem);
          }
        }
      }
    }
  }

  // Uniquify elems_to_check_out
  std::sort(elems_to_check_out.begin(), elems_to_check_out.end());
  std::vector<stk::mesh::Entity>::iterator it =
          std::unique(elems_to_check_out.begin(), elems_to_check_out.end());
  elems_to_check_out.resize(it-elems_to_check_out.begin());

  // Mark all of the new elems and their nodes with 1
  for(size_t k=0; k<elems_to_check_out.size(); ++k)
  {
    stk::mesh::Entity cur_elem = elems_to_check_out[k];
    stk::mesh::EntityId elem_id = mesh_api->get_bulk_data()->identifier(cur_elem);
    elem_vals[elem_id] = 1;
    stk::mesh::Entity const *elem_nodes = mesh_api->get_bulk_data()->begin_nodes(cur_elem);
    int num_nodes = mesh_api->get_bulk_data()->num_nodes(cur_elem);
    for(int j=0; j<num_nodes; ++j)
    {
      stk::mesh::Entity cur_node = elem_nodes[j];
      stk::mesh::EntityId node_id = mesh_api->get_bulk_data()->identifier(cur_node);
      node_vals[node_id] = 1;
    }
  }

  int global_has_nonmanifold_connection = 0;

  MPI_Allreduce(&local_has_nonmanifold_connection,
                &global_has_nonmanifold_connection, 1,
                sierra::MPI::Datatype<int>::type(),
                MPI_MAX, mesh_api->get_bulk_data()->parallel());

  return global_has_nonmanifold_connection;
}

void PerceptPruneTool::communicate_percept_boundary_elem_info(std::vector<proc_node_map> &procs,
                               std::map<stk::mesh::EntityId,int> &elem_vals,
                               percept::PerceptMesh *mesh_api)
{
  if(procs.size() > 0)
  {
    stk::CommSparse comm_spec(mesh_api->get_bulk_data()->parallel());
    for(int phase=0; phase<2; ++phase)
    {
      for(size_t i=0; i<procs.size(); ++i)
      {
        int other_proc = procs[i].processor;
        // pack the number of elems
        comm_spec.send_buffer(other_proc).pack<size_t>(procs[i].elements.size());
        for(size_t j=0; j<procs[i].elements.size(); ++j)
        {
          // pack the element id and then the value for this element
          comm_spec.send_buffer(other_proc).pack<stk::mesh::EntityId>
                  (procs[i].elements[j]);
          comm_spec.send_buffer(other_proc).pack<int>
                  (elem_vals[procs[i].elements[j]]);
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

    for(size_t i=0; i<procs.size(); ++i)
    {
      int other_proc = procs[i].processor;
      // get the number of elements from the sending proc
      size_t num_elems;
      comm_spec.recv_buffer(other_proc).unpack<size_t>(num_elems);
      for(size_t j=0; j<num_elems; ++j)
      {
        stk::mesh::EntityId elem_id;
        int elem_val;
        comm_spec.recv_buffer(other_proc).unpack<stk::mesh::EntityId>(elem_id);
        comm_spec.recv_buffer(other_proc).unpack<int>(elem_val);
        if(elem_vals.count(elem_id) == 0 || elem_val > elem_vals[elem_id])
          elem_vals[elem_id] = elem_val;
      }
    }
  }
}


void PerceptPruneTool::communicate_percept_boundary_info(std::vector<proc_node_map> &procs,
                               std::map<stk::mesh::EntityId,int> &node_vals,
                               std::map<stk::mesh::EntityId,int> &elem_vals,
                               percept::PerceptMesh *mesh_api)
{
  if(procs.size() > 0)
  {
    stk::CommSparse comm_spec(mesh_api->get_bulk_data()->parallel());
    for(int phase=0; phase<2; ++phase)
    {
      for(size_t i=0; i<procs.size(); ++i)
      {
        int other_proc = procs[i].processor;
        // pack the number of nodes
        comm_spec.send_buffer(other_proc).pack<size_t>(procs[i].nodes.size());
        for(size_t j=0; j<procs[i].nodes.size(); ++j)
        {
          // pack the node id and then the value for this node
          comm_spec.send_buffer(other_proc).pack<stk::mesh::EntityId>
                  (procs[i].nodes[j]);
          comm_spec.send_buffer(other_proc).pack<int>
                  (node_vals[procs[i].nodes[j]]);
        }
        // pack the number of elems
        comm_spec.send_buffer(other_proc).pack<size_t>(procs[i].elements.size());
        for(size_t j=0; j<procs[i].elements.size(); ++j)
        {
          // pack the element id and then the value for this element
          comm_spec.send_buffer(other_proc).pack<stk::mesh::EntityId>
                  (procs[i].elements[j]);
          comm_spec.send_buffer(other_proc).pack<int>
                  (elem_vals[procs[i].elements[j]]);
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

    for(size_t i=0; i<procs.size(); ++i)
    {
      int other_proc = procs[i].processor;
      // get the number of nodes from the sending proc
      size_t num_nodes;
      comm_spec.recv_buffer(other_proc).unpack<size_t>(num_nodes);
      for(size_t j=0; j<num_nodes; ++j)
      {
        stk::mesh::EntityId node_id;
        int node_val;
        comm_spec.recv_buffer(other_proc).unpack<stk::mesh::EntityId>(node_id);
        comm_spec.recv_buffer(other_proc).unpack<int>(node_val);
        if(node_vals.count(node_id) == 0 || node_val > node_vals[node_id])
          node_vals[node_id] = node_val;
      }
      // get the number of elements from the sending proc
      size_t num_elems;
      comm_spec.recv_buffer(other_proc).unpack<size_t>(num_elems);
      for(size_t j=0; j<num_elems; ++j)
      {
        stk::mesh::EntityId elem_id;
        int elem_val;
        comm_spec.recv_buffer(other_proc).unpack<stk::mesh::EntityId>(elem_id);
        comm_spec.recv_buffer(other_proc).unpack<int>(elem_val);
        if(elem_vals.count(elem_id) == 0 || elem_val > elem_vals[elem_id])
          elem_vals[elem_id] = elem_val;
      }
    }
  }
}

void PerceptPruneTool::calculate_percept_sharing_maps(std::vector<proc_node_map> &procs,
                                       MeshManager &tMeshManager)
{

   stk::mesh::BulkData & tBulkData = tMeshManager.get_output_bulk_data();
   percept::PerceptMesh & mesh_api = tMeshManager.get_output_percept();
  // Find which owned elems are on the boundary
  std::set<PruneHandle> shared_boundary_nodes;
//  mesh_api->get_shared_boundary_nodes(shared_boundary_nodes);
  tMeshManager.get_shared_boundary_nodes(shared_boundary_nodes);
  std::set<PruneHandle>::iterator it = shared_boundary_nodes.begin();
  while(it != shared_boundary_nodes.end())
  {
    PruneHandle cur_boundary_node = *it;
//    stk::mesh::Entity node_entity = mesh_api->get_stk_entity(cur_boundary_node);
    stk::mesh::Entity node_entity = tMeshManager.get_stk_entity(cur_boundary_node);
//    stk::mesh::EntityId node_id = mesh_api->entity_id(cur_boundary_node);
    stk::mesh::EntityId node_id = tMeshManager.entity_id(cur_boundary_node);
    stk::mesh::EntityKey key(stk::topology::NODE_RANK, node_id);
    std::vector<int> sharingProcs;
//    mesh_api->get_bulk_data()->comm_shared_procs(key, sharingProcs);
    tBulkData.comm_shared_procs(key,sharingProcs);
    for(size_t i=0; i<sharingProcs.size(); ++i)
    {
      int other_proc_id = sharingProcs[i];
      // if this is not me...
      if(other_proc_id != mesh_api.get_bulk_data()->parallel_rank())
      {
        size_t j = 0;
        for(; j<procs.size(); ++j)
        {
          if(procs[j].processor == other_proc_id)
          {
            procs[j].nodes.push_back(node_id);
            stk::mesh::Entity const *node_elems = mesh_api.
                            get_bulk_data()->begin_elements(node_entity);
            int num_elems = mesh_api.get_bulk_data()->num_elements(node_entity);
            for(int k=0; k<num_elems; ++k)
            {
              stk::mesh::Entity cur_elem = node_elems[k];
           //   if(owned_elems.find(cur_elem) != owned_elems.end())
                procs[j].elements.push_back(mesh_api.get_bulk_data()->identifier(cur_elem));
            }
            break;
          }
        }
        // if we don't have an entry for this proc yet...
        if(j == procs.size())
        {
          proc_node_map new_proc;
          new_proc.processor = other_proc_id;
          new_proc.nodes.push_back(node_id);
          stk::mesh::Entity const *node_elems = mesh_api.
                          get_bulk_data()->begin_elements(node_entity);
          int num_elems = mesh_api.get_bulk_data()->num_elements(node_entity);
          for(int k=0; k<num_elems; ++k)
          {
            stk::mesh::Entity cur_elem = node_elems[k];
          //  if(owned_elems.find(cur_elem) != owned_elems.end())
              new_proc.elements.push_back(mesh_api.get_bulk_data()->identifier(cur_elem));
          }
          procs.push_back(new_proc);
        }
      }
    }
    // Uniquify the element lists in the processor maps
    for(size_t j=0; j<procs.size(); ++j)
    {
      std::sort(procs[j].elements.begin(), procs[j].elements.end());
      std::vector<stk::mesh::EntityId>::iterator it2 =
          std::unique(procs[j].elements.begin(), procs[j].elements.end());
      procs[j].elements.resize(it2-procs[j].elements.begin());
    }

    ++it;
  }
}

void PerceptPruneTool::find_initial_set_of_percept_elems_to_keep(
                       const std::vector<PruneHandle> &elem_list,
                       int num_buffer_layers,
                       MeshManager &tMeshManager,
                       std::map<stk::mesh::EntityId, int> &node_vals,
                       std::map<stk::mesh::EntityId, int> &elem_vals,
                       std::vector<proc_node_map> &procs)
{
  int num_elems = elem_list.size();

  percept::PerceptMesh * mesh_api = &tMeshManager.get_output_percept();

  // Set all element marks to 0
  for(int i=0; i<num_elems; ++i)
  {
    PruneHandle cur_elem = elem_list[i];
//    stk::mesh::EntityId cur_elem_id = mesh_api->entity_id(cur_elem);
    stk::mesh::EntityId cur_elem_id = tMeshManager.entity_id(cur_elem);
    elem_vals[cur_elem_id] = 0;
  }

  PruneHandle tmp_elem_node_list[8];
//  int num_nodes_per_elem = mesh_api->element_nodes(elem_list[0], tmp_elem_node_list);
  int num_nodes_per_elem = tMeshManager.element_nodes(elem_list[0], tmp_elem_node_list);

  // Create a map of all of the nodes and their field values.
  // We will manipulate these values during the algorithm
  // so we don't want to work on the direct field.
  std::vector<PruneHandle> all_nodes(num_elems*num_nodes_per_elem);
  size_t cntr = 0;
  for(int i=0; i<num_elems; ++i)
  {
    PruneHandle cur_elem = elem_list[i];
    PruneHandle elem_node_list[8];
//    int num_nodes = mesh_api->element_nodes(cur_elem, elem_node_list);
    int num_nodes = tMeshManager.element_nodes(cur_elem, elem_node_list);
    for(int j=0; j<num_nodes; j++)
      all_nodes[cntr++] = elem_node_list[j];
  }
  // Uniquify the list of nodes
  std::sort(all_nodes.begin(), all_nodes.end());
  std::vector<PruneHandle>::iterator it = std::unique(all_nodes.begin(),all_nodes.end());
  all_nodes.resize(it-all_nodes.begin());
  // Get the nodal variables and put them in the map.
  for(size_t i=0; i<all_nodes.size(); i++)
  {
//    stk::mesh::EntityId cur_node_id = mesh_api->entity_id(all_nodes[i]);
    stk::mesh::EntityId cur_node_id = tMeshManager.entity_id(all_nodes[i]);
//    double val = mesh_api->get_max_nodal_iso_field_variable(all_nodes[i]);
    double val = tMeshManager.get_max_nodal_iso_field_variable(all_nodes[i]);
    if(val > 0.5)
      node_vals[cur_node_id] = 1;
    else
      node_vals[cur_node_id] = 0;
  }

  // Start the loop for finding elements to keep.
  for(int i=-1; i<num_buffer_layers; ++i)
  {
    // Let the other procs know what values have been set on processor boundaries.
    communicate_percept_boundary_info(procs, node_vals, elem_vals, mesh_api);

    std::vector<PruneHandle> local_elements_to_keep;
    // Get all of the elems with at least one node with a value of 1
    for(int j=0; j<num_elems; ++j)
    {
      PruneHandle cur_elem = elem_list[j];
//      stk::mesh::EntityId cur_elem_id = mesh_api->entity_id(cur_elem);
      stk::mesh::EntityId cur_elem_id = tMeshManager.entity_id(cur_elem);
      if(!elem_vals[cur_elem_id])
      {
        PruneHandle elem_node_list[8];
//        int num_nodes = mesh_api->element_nodes(cur_elem, elem_node_list);
        int num_nodes = tMeshManager.element_nodes(cur_elem, elem_node_list);
        for(int k=0; k<num_nodes; ++k)
        {
//          stk::mesh::EntityId cur_node_id = mesh_api->entity_id(elem_node_list[k]);
            stk::mesh::EntityId cur_node_id =tMeshManager.entity_id(elem_node_list[k]);
          if(node_vals[cur_node_id])
          {
            local_elements_to_keep.push_back(cur_elem);
            break;
          }
        }
      }
    }

    // Mark all of the elems we have found thus far and
    // set the nodal values for these elems to be 1.
    for(size_t j=0; j<local_elements_to_keep.size(); ++j)
    {
      PruneHandle cur_elem = local_elements_to_keep[j];
      PruneHandle elem_node_list[8];
//      int num_nodes = mesh_api->element_nodes(cur_elem, elem_node_list);
      int num_nodes = tMeshManager.element_nodes(cur_elem, elem_node_list);
      for(int k=0; k<num_nodes; ++k)
      {
//        stk::mesh::EntityId id = mesh_api->entity_id(elem_node_list[k]);
          stk::mesh::EntityId id = tMeshManager.entity_id(elem_node_list[k]);
        node_vals[id] = 1;
      }
//      stk::mesh::EntityId elem_id = mesh_api->entity_id(cur_elem);
      stk::mesh::EntityId elem_id = tMeshManager.entity_id(cur_elem);
      elem_vals[elem_id] = 1;
    }
  }
}




