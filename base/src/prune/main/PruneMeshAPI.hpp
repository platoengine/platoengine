//-------------------------------------------------------------------------
// Filename      : PruneMeshAPI.hpp
//
// Description   : a PruneMeshAPI is the base class for a generic mesh api
//                 used for the PruneTool
//
// Creator       : Brett Clark
//
// Creation Date : 5/6/2016
//
// Owner         : Brett Clark
//-------------------------------------------------------------------------

#ifndef PruneMeshAPI_____HPP
#define PruneMeshAPI_____HPP

#include <stdint.h>
#include <set>
#include <vector>
#include <map>

typedef uint64_t PruneHandle;

struct DuplicateNodeInfo
{
  PruneHandle existing_node_local_id;
  PruneHandle existing_node_global_id;
  PruneHandle suggested_new_node_id;
  double new_node_pos[3];
  PruneHandle new_node;
};

struct BoundaryNodeInfo
{
  PruneHandle existing_node_id;
  PruneHandle min_node, max_node;
  PruneHandle suggested_new_node_id;
  double new_node_pos[3];
  PruneHandle new_node;
  PruneHandle min_node_global_id, max_node_global_id;
  double mu; // fraction from min_node to max_node where iso surface intersects edge
};

class PruneMeshAPI
{
private:
public:

  // Default Constructor
  PruneMeshAPI(){}
  virtual ~PruneMeshAPI(){}

  // functions derived classes must implement
  virtual void transfer_output_fields(PruneHandle n1, PruneHandle n2, PruneHandle new_node,
                        double mu, PruneMeshAPI *output_mesh_api) = 0;
  virtual void copy_node_output_fields(PruneHandle n1, PruneHandle new_node, PruneMeshAPI *output_mesh_api) = 0;
  virtual void copy_element_output_fields(PruneHandle e1, PruneHandle e2, PruneMeshAPI *output_mesh_api) = 0;
  virtual PruneHandle new_node( double &x, double &y, double &z ) = 0;
  virtual void get_fixed_block_nodes(std::vector<PruneHandle> &fixed_block_nodes) = 0;
  virtual PruneHandle get_new_node_id() = 0;
  virtual PruneHandle new_tri( PruneHandle n1, PruneHandle n2, PruneHandle n3, bool is_fixed,
                             PruneHandle source_elem ) = 0;
  virtual int element_nodes(PruneHandle elem, PruneHandle nodes[8]) const = 0;
  virtual void hex_nodes(PruneHandle hex, PruneHandle nodes[8]) const = 0;
  virtual void tet_nodes(PruneHandle tet, PruneHandle nodes[4]) const = 0;
  virtual void hex_quad_nodes(PruneHandle hex, int index, PruneHandle nodes[4]) const = 0;
  virtual PruneHandle get_connected_hex(PruneHandle hex, PruneHandle n1, PruneHandle n2, PruneHandle n3, PruneHandle n4) const = 0;
  virtual PruneHandle get_connected_tet(PruneHandle tet, PruneHandle n1, PruneHandle n2, PruneHandle n3) const = 0;
  virtual void node_coordinates( PruneHandle node, double &x, double &y, double &z ) = 0;
  virtual double get_max_nodal_iso_field_variable(PruneHandle node) const = 0;
  virtual void store_tri_to_tet_map_entry(const PruneHandle &tri, const PruneHandle &tet) = 0;
  virtual void store_tet_to_tri_map_entry(const PruneHandle &tet, const PruneHandle &tri) = 0;
  virtual void get_shared_boundary_nodes(std::set<PruneHandle> &shared_boundary_nodes) = 0;
  virtual void get_attached_elements(const std::set<PruneHandle> &nodes,
                                     std::vector<PruneHandle> &attached_elements) = 0;
  virtual void batch_create_edge_boundary_nodes(std::vector<BoundaryNodeInfo> &boundary_info,
                          PruneMeshAPI *existing_mesh) = 0;
  virtual void batch_create_duplicate_nodes(std::vector<DuplicateNodeInfo> &dup_node_infos, 
                                            PruneMeshAPI *existing_mesh) = 0;
  virtual void print_boundary_node_info(std::vector<BoundaryNodeInfo> &bni) = 0;
  virtual void reserve_new_node_ids(uint64_t num_requested) = 0;
  virtual void reserve_new_tri_ids(uint64_t num_requested) = 0;
  virtual double calculate_average_edge_length(const std::vector<PruneHandle> &elem_list) = 0;
  virtual void set_min_node_id(BoundaryNodeInfo &bni, const PruneHandle &n) = 0;
  virtual void set_max_node_id(BoundaryNodeInfo &bni, const PruneHandle &n) = 0;
  virtual void set_existing_node_id(DuplicateNodeInfo &dni, const PruneHandle &n) = 0;
  virtual void add_element_to_survivor_block(PruneHandle entity) = 0;
};

#endif // PruneMeshAPI_____HPP
