/*
 * PerceptPruneTool.hpp
 *
 *  Created on: Aug 1, 2017
 *      Author: tzirkle
 */

#ifndef ISO_PRUNE_MAIN_PERCEPTPRUNETOOL_HPP_
#define ISO_PRUNE_MAIN_PERCEPTPRUNETOOL_HPP_

#include <vector>
#include <map>
#include <set>
#include "PruneMeshAPI.hpp"
#include <stk_mesh/base/Types.hpp>

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

class PerceptPruneTool
{
private:
  void recursively_consolidate_groups(stk::mesh::EntityId key,
                             std::map<stk::mesh::EntityId, std::set<stk::mesh::EntityId> > &global_equivs,
                             std::set<stk::mesh::EntityId> &new_set);
  void consolidate_percept_groups(
                std::map<stk::mesh::EntityId,
                std::set<stk::mesh::EntityId> > &global_equivs,
                std::vector<elem_group*> &groups,
                percept::PerceptMesh *mesh_api);
  void get_percept_global_equiv_info(
                std::map<stk::mesh::EntityId, std::set<stk::mesh::EntityId> > &local_equivs,
                std::map<stk::mesh::EntityId, std::set<stk::mesh::EntityId> > &global_equivs,
                percept::PerceptMesh *mesh_api);
  void find_percept_local_equivs(std::vector<proc_node_map> &procs,
                std::map<stk::mesh::EntityId,std::set<stk::mesh::EntityId> > &my_equivs,
                std::vector<elem_group*> &groups,
                percept::PerceptMesh *mesh_api);
  void grow_percept_smallest_group(
                         std::vector<elem_group*> &groups,
                         const std::set<stk::mesh::EntityId> &elem_set,
                         percept::PerceptMesh *mesh_api,
                         std::map<stk::mesh::EntityId, int> &node_vals,
                         std::map<stk::mesh::EntityId, int> &elem_vals,
                         std::vector<proc_node_map> &procs,
                         std::set<stk::mesh::EntityId> &unique_group_ids);
  void find_percept_local_groups(
                         const std::set<stk::mesh::EntityId> &elem_set,
                         std::map<stk::mesh::EntityId, int> &elem_vals,
                         std::vector<elem_group*> &groups,
                         percept::PerceptMesh *mesh_api);
  int number_of_percept_groups(
                         std::vector<elem_group*> &groups,
                         const std::set<stk::mesh::EntityId> &elem_set,
                         percept::PerceptMesh *mesh_api,
                         std::map<stk::mesh::EntityId, int> &node_vals,
                         std::map<stk::mesh::EntityId, int> &elem_vals,
                         std::vector<proc_node_map> &procs,
                         std::set<stk::mesh::EntityId> &unique_group_ids);
  int nonmanifold_percept_connection_exists(
              std::map<stk::mesh::EntityId, int> &node_vals,
              std::map<stk::mesh::EntityId, int> &elem_vals,
              std::vector<stk::mesh::Entity> &elems_to_check_in,
              std::vector<stk::mesh::Entity> &elems_to_check_out,
              percept::PerceptMesh *mesh_api, MeshManager &tMeshManager);
  void communicate_percept_boundary_elem_info(std::vector<proc_node_map> &procs,
                                 std::map<stk::mesh::EntityId,int> &elem_vals,
                                 percept::PerceptMesh *mesh_api);
  void communicate_percept_boundary_info(std::vector<proc_node_map> &procs,
                                 std::map<stk::mesh::EntityId,int> &node_vals,
                                 std::map<stk::mesh::EntityId,int> &elem_vals,
                                 percept::PerceptMesh *mesh_api);
  void calculate_percept_sharing_maps(
                       std::vector<proc_node_map> &procs,
                       MeshManager &tMeshManager);
  void ensure_connected_set_of_percept_elements(
                         const std::set<stk::mesh::EntityId> &elem_set,
                         percept::PerceptMesh *mesh_api,
                         std::map<stk::mesh::EntityId, int> &node_vals,
                         std::map<stk::mesh::EntityId, int> &elem_vals,
                         std::vector<proc_node_map> &procs);
  void find_initial_set_of_percept_elems_to_keep(
                         const std::vector<PruneHandle> &elem_list,
                         int num_buffer_layers,
                         MeshManager &tMeshManager,
                         std::map<stk::mesh::EntityId, int> &node_vals,
                         std::map<stk::mesh::EntityId, int> &elem_vals,
                         std::vector<proc_node_map> &procs);

public:
  void prune_percept_mesh(const std::vector<PruneHandle> &elem_list,
                       std::set<PruneHandle> &elems_to_keep,
                       int num_buffer_layers,
                       int allow_nonmanifold_connections,
                       int remove_islands,
                       MeshManager &tMeshManager);
};



#endif /* ISO_PRUNE_MAIN_PERCEPTPRUNETOOL_HPP_ */
