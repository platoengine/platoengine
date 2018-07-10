#ifndef ISOVOLUMEEXTRACTIONTOOL_HPP
#define ISOVOLUMEEXTRACTIONTOOL_HPP

#include <vector>
#include <map>
#include <set>
#include "IVEMeshAPI.hpp"

struct MeshEdge
{
  IVEHandle mid_node;
  IVEHandle max_id_node;
};

class IsoVolumeExtractionTool
{
private:

      double mMinx, mMiny, mMinz, mMaxx, mMaxy, mMaxz;
      double mAverageEdgeLength;
  void create_nodes_on_processor_boundaries(std::map<IVEHandle,double> &nodal_vars,
                                                    std::multimap<IVEHandle,MeshEdge*> &edge_map,
                                                    std::map<IVEHandle,IVEHandle> &node_map,
                                                    double iso_value,
                                                    IVEMeshAPI *mesh_api,
                                                    IVEMeshAPI *mesh_api_out,
                                                    bool hex,
                                                    double value_tol,
                                                    std::set<IVEHandle> &shared_boundary_nodes,
                                                    std::vector<IVEHandle> &boundary_elements);
  void create_interior_tris_for_hex(IVEHandle hex,
                                    const IVEHandle hex_nodes[8],
                                    const double *vals,
                                    const double &isolevel,
                                    std::multimap<IVEHandle, MeshEdge*> &edge_map,
                                    std::map<IVEHandle,IVEHandle> &node_map,
                                    std::vector<IVEHandle> &optimized_tris,
                                    IVEMeshAPI *mesh_api,
                                    IVEMeshAPI *mesh_api_out,
                                    bool boundary_info,
                                    std::vector<BoundaryNodeInfo> &boundary_node_info,
                                    std::set<IVEHandle> &boundary_nodes);
  void create_boundary_tris_for_hex(const IVEHandle &cur_hex,
                                    std::map<IVEHandle,double> &nodal_var_map,
                                    std::multimap<IVEHandle,MeshEdge*> &edge_map,
                                    std::map<IVEHandle,IVEHandle> &node_map,
                                    std::vector<IVEHandle> &fixed_tris,
                                    const double &iso_level,
                                    IVEMeshAPI *mesh_api,
                                    IVEMeshAPI *mesh_api_out,
                                    bool boundary_info,
                                    std::vector<BoundaryNodeInfo> &boundary_node_info,
                                    std::set<IVEHandle> &boundary_nodes);
  void create_boundary_tris_for_tet(const IVEHandle &cur_tet,
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
                                    const double &zero_tol,
                                    const double &neg_zero_tol,
                                    std::map<IVEHandle, IVEHandle> &node_map,
                                    IVEMeshAPI *mesh_api,
                                    IVEMeshAPI *mesh_api_out,     
                                    bool boundary_info,
                                    std::vector<BoundaryNodeInfo> &duplicate_node_info,
                                    std::set<IVEHandle> &boundary_nodes);
  IVEHandle get_duplicate_node(const IVEHandle &old_node,
                               std::map<IVEHandle, IVEHandle> &node_map,
                               IVEMeshAPI *mesh_api,
                               IVEMeshAPI *mesh_api_out,
                               bool boundary_info,
                               BoundaryNodeInfo &bni);
  IVEHandle get_mid_node(const IVEHandle &n1,
                       const IVEHandle &n2,
                       const double &val1,
                       const double &val2,
                       const double &iso_val,
                       std::multimap<IVEHandle, MeshEdge*> &edge_map,
                       std::map<IVEHandle, IVEHandle> &node_map,
                       IVEMeshAPI *mesh_api,
                       IVEMeshAPI *mesh_api_out,
                       bool boundary_info,
                       BoundaryNodeInfo &bni);
  MeshEdge* find_mesh_edge(const IVEHandle &n1,
                           const IVEHandle &n2,
                           std::multimap<IVEHandle,MeshEdge*> &edge_map);
  void process_tet(const IVEHandle &tet,
                   std::vector<IVEHandle> &fixed_tris,
                   std::vector<IVEHandle> &optimized_tris,
                   const double &iso_value,
                   const double &value_tol,
                   std::multimap<IVEHandle,MeshEdge*> &edge_map,
                   std::map<IVEHandle,IVEHandle> &node_map,
                   IVEMeshAPI *mesh_api,
                   IVEMeshAPI *mesh_api_out,
                   bool boundary_info,
                   std::vector<BoundaryNodeInfo> &edge_node_info,
                   std::vector<BoundaryNodeInfo> &duplicate_node_info,
                   std::set<IVEHandle> &boundary_nodes);
public:
      double minx() { return mMinx; }
      double miny() { return mMiny; }
      double minz() { return mMinz; }
      double maxx() { return mMaxx; }
      double maxy() { return mMaxy; }
      double maxz() { return mMaxz; }
      double average_edge_length() { return mAverageEdgeLength; }
      void minx(double val) { mMinx = val; }
      void miny(double val) { mMiny = val; }
      void minz(double val) { mMinz = val; }
      void maxx(double val) { mMaxx = val; }
      void maxy(double val) { mMaxy = val; }
      void maxz(double val) { mMaxz = val; }
      void average_edge_length(double average_edge_length) { mAverageEdgeLength = average_edge_length; }
  void build_iso_volume_tris_from_hexes(const std::vector<IVEHandle> &elem_list,
                                        const double &iso_value,
                                        const double &min_edge_length,
                                        const bool &level_set_data,
                                        std::vector<IVEHandle> &fixed_tris,
                                        std::vector<IVEHandle> &optimized_tris,
                                        IVEMeshAPI *mesh_api,
                                        IVEMeshAPI *mesh_api_out);
  void build_iso_volume_tris_from_tets(const std::vector<IVEHandle> &elem_list,
                                       const double &iso_value,
                                       const double &min_edge_length,
                                       const bool &level_set_data,
                                       std::vector<IVEHandle> &fixed_tris,
                                       std::vector<IVEHandle> &optimized_tris,
                                       IVEMeshAPI *mesh_api,
                                       IVEMeshAPI *mesh_api_out);
};

#endif

