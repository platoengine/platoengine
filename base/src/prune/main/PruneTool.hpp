#ifndef PRUNETOOL_HPP
#define PRUNETOOL_HPP

#include <vector>
#include <map>
#include <set>
#include "PruneMeshAPI.hpp"
#include <stk_mesh/base/Types.hpp>


struct proc_node_map
{
  int processor;
  std::vector<stk::mesh::EntityId> nodes;
  std::vector<stk::mesh::EntityId> elements;
};

struct elem_group
{
  std::set<stk::mesh::EntityId> elems;
  stk::mesh::EntityId id;
};

class PruneTool
{
private:
  void recursively_consolidate_groups(stk::mesh::EntityId key,
                             std::map<stk::mesh::EntityId, std::set<stk::mesh::EntityId> > &global_equivs,
                             std::set<stk::mesh::EntityId> &new_set);
  void consolidate_groups(
              std::map<stk::mesh::EntityId, std::set<stk::mesh::EntityId> > &global_equivs,
              std::vector<elem_group*> &groups,
              PruneMeshAPISTK *mesh_api);
  void get_global_equiv_info(
              std::map<stk::mesh::EntityId, std::set<stk::mesh::EntityId> > &local_equivs,
              std::map<stk::mesh::EntityId, std::set<stk::mesh::EntityId> > &global_equivs,
              PruneMeshAPISTK *mesh_api);
  void find_local_equivs(std::vector<proc_node_map> &procs,
                       std::map<stk::mesh::EntityId,std::set<stk::mesh::EntityId> > &my_equivs, 
                       std::vector<elem_group*> &groups,
                       PruneMeshAPISTK *mesh_api);
  void grow_smallest_group(
                       std::vector<elem_group*> &groups,
                       const std::set<stk::mesh::EntityId> &elem_set,
                       PruneMeshAPISTK *mesh_api,
                       std::map<stk::mesh::EntityId, int> &node_vals,
                       std::map<stk::mesh::EntityId, int> &elem_vals,
                       std::vector<proc_node_map> &procs,
                       std::set<stk::mesh::EntityId> &unique_group_ids);
  void find_local_groups(
                       const std::set<stk::mesh::EntityId> &elem_set,
                       std::map<stk::mesh::EntityId, int> &elem_vals,
                       std::vector<elem_group*> &groups,
                       PruneMeshAPISTK *mesh_api);
  int number_of_groups(
                       std::vector<elem_group*> &groups,
                       const std::set<stk::mesh::EntityId> &elem_set,
                       PruneMeshAPISTK *mesh_api,
                       std::map<stk::mesh::EntityId, int> &node_vals,
                       std::map<stk::mesh::EntityId, int> &elem_vals,
                       std::vector<proc_node_map> &procs, 
                       std::set<stk::mesh::EntityId> &unique_group_ids);
  int nonmanifold_connection_exists(
                       std::map<stk::mesh::EntityId, int> &node_vals,
                       std::map<stk::mesh::EntityId, int> &elem_vals,
                       std::vector<stk::mesh::Entity> &hexes_to_check_in,
                       std::vector<stk::mesh::Entity> &hexes_to_check_out,
                       PruneMeshAPISTK *mesh_api);
  void communicate_boundary_elem_info(
                       std::vector<proc_node_map> &procs,
                       std::map<stk::mesh::EntityId,int> &elem_vals,
                       PruneMeshAPISTK *mesh_api);
  void communicate_boundary_info(
                       std::vector<proc_node_map> &procs,
                       std::map<stk::mesh::EntityId,int> &node_vals,
                       std::map<stk::mesh::EntityId,int> &elem_vals,
                       PruneMeshAPISTK *mesh_api);
  void calculate_sharing_maps(
                       std::vector<proc_node_map> &procs,
                       PruneMeshAPISTK *mesh_api);
  void ensure_connected_set_of_elements(
                       const std::set<stk::mesh::EntityId> &elem_set,
                       PruneMeshAPISTK *mesh_api,
                       std::map<stk::mesh::EntityId, int> &node_vals,
                       std::map<stk::mesh::EntityId, int> &elem_vals,
                       std::vector<proc_node_map> &procs);
  void find_initial_set_of_elems_to_keep(
                       const std::vector<PruneHandle> &elem_list,
                       int num_buffer_layers,
                       PruneMeshAPISTK *mesh_api,
                       std::map<stk::mesh::EntityId, int> &node_vals,
                       std::map<stk::mesh::EntityId, int> &elem_vals,
                       std::vector<proc_node_map> &procs);

public:
  void prune_mesh( const std::vector<PruneHandle> &elem_list,
                       std::set<PruneHandle> &elems_to_keep,
                       int num_buffer_layers,
                       int allow_nonmanifold_connections,
                       int remove_islands,
                       PruneMeshAPISTK *mesh_api);
};

#endif

