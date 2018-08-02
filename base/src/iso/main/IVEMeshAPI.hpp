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
// Filename      : IVEMeshAPI.hpp
//
// Description   : a IVEMeshAPI is the base class for a generic mesh api
//                 used for the IsoVolumeExtractTool
//
// Creator       : Brett Clark
//
// Creation Date : 12/8/2014
//
// Owner         : Brett Clark
//-------------------------------------------------------------------------

#ifndef IVEMeshAPI_____HPP
#define IVEMeshAPI_____HPP

#include <stdint.h>
#include <set>
#include <vector>
#include <map>
#include "IsoVector.hpp"

typedef uint64_t IVEHandle;

struct DuplicateNodeInfo
{
  IVEHandle existing_node_local_id;
  IVEHandle existing_node_global_id;
  IVEHandle suggested_new_node_id;
  IsoVector new_node_pos;
  IVEHandle new_node;
};

struct BoundaryNodeInfo
{
  IVEHandle existing_node_id;
  IVEHandle min_node, max_node;
  IVEHandle suggested_new_node_id;
  IsoVector new_node_pos;
  IVEHandle new_node;
  IVEHandle min_node_global_id, max_node_global_id;
  double mu; // fraction from min_node to max_node where iso surface intersects edge
};

class IVEMeshAPI
{
private:
public:

  // Default Constructor
  IVEMeshAPI(){}
  virtual ~IVEMeshAPI(){}

  // functions derived classes must implement
  virtual void transfer_output_fields(IVEHandle n1, IVEHandle n2, IVEHandle new_node,
                        double mu, IVEMeshAPI *output_mesh_api) = 0;
  virtual void copy_node_output_fields(IVEHandle n1, IVEHandle new_node, IVEMeshAPI *output_mesh_api) = 0;
  virtual void copy_element_output_fields(IVEHandle e1, IVEHandle e2, IVEMeshAPI *output_mesh_api) = 0;
  virtual IVEHandle new_node( IsoVector &coordinates ) = 0;
  virtual void get_fixed_block_nodes(std::vector<IVEHandle> &fixed_block_nodes) = 0;
  virtual IVEHandle get_new_node_id() = 0;
  virtual IVEHandle new_tri( IVEHandle n1, IVEHandle n2, IVEHandle n3, bool is_fixed,
                             IVEHandle source_elem ) = 0;
  virtual int element_nodes(IVEHandle elem, IVEHandle nodes[8]) const = 0;
  virtual void hex_nodes(IVEHandle hex, IVEHandle nodes[8]) const = 0;
  virtual void tet_nodes(IVEHandle tet, IVEHandle nodes[4]) const = 0;
  virtual void hex_quad_nodes(IVEHandle hex, int index, IVEHandle nodes[4]) const = 0;
  virtual IVEHandle get_connected_hex(IVEHandle hex, IVEHandle n1, IVEHandle n2, IVEHandle n3, IVEHandle n4) const = 0;
  virtual IVEHandle get_connected_tet(IVEHandle tet, IVEHandle n1, IVEHandle n2, IVEHandle n3) const = 0;
  virtual IsoVector node_coordinates( IVEHandle node ) const = 0;
  virtual double get_nodal_iso_field_variable(IVEHandle node) const = 0;
  virtual void store_tri_to_tet_map_entry(const IVEHandle &tri, const IVEHandle &tet) = 0;
  virtual void store_tet_to_tri_map_entry(const IVEHandle &tet, const IVEHandle &tri) = 0;
  virtual void get_shared_boundary_nodes(std::set<IVEHandle> &shared_boundary_nodes) = 0;
  virtual void get_attached_elements(const std::set<IVEHandle> &nodes,
                                     std::vector<IVEHandle> &attached_elements) = 0;
  virtual void batch_create_edge_boundary_nodes(std::vector<BoundaryNodeInfo> &boundary_info,
                          IVEMeshAPI *existing_mesh) = 0;
  virtual void batch_create_duplicate_nodes(std::vector<DuplicateNodeInfo> &dup_node_infos, 
                                            IVEMeshAPI *existing_mesh) = 0;
  virtual void print_boundary_node_info(std::vector<BoundaryNodeInfo> &bni) = 0;
  virtual void reserve_new_node_ids(uint64_t num_requested) = 0;
  virtual void reserve_new_tri_ids(uint64_t num_requested) = 0;
  virtual void calculate_average_edge_length_and_bbox(const std::vector<IVEHandle> &elem_list,
          double &minx, double &miny, double &minz, double &maxx, double &maxy, double &maxz, double &ave_length) = 0;
  virtual void set_min_node_id(BoundaryNodeInfo &bni, const IVEHandle &n) = 0;
  virtual void set_max_node_id(BoundaryNodeInfo &bni, const IVEHandle &n) = 0;
  virtual void set_existing_node_id(DuplicateNodeInfo &dni, const IVEHandle &n) = 0;
};

#endif // IVEMeshAPI_____HPP
