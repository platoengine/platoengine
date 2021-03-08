//-------------------------------------------------------------------------
// Filename      : PruneMeshAPISTK.hpp
//
// Description   : a PruneMeshAPISTK is an implementation of PruneMeshAPI for STK  mesh entities
//
// Creator       : Brett Clark
//
// Creation Date : 5/6/2016
//
// Owner         : Brett Clark
//-------------------------------------------------------------------------

#ifndef PruneMeshAPISTK_____HPP
#define PruneMeshAPISTK_____HPP

#include "PruneMeshAPI.hpp"
#include <vector>
#include <map>
#include <stk_mesh/base/BulkData.hpp>
#include <stk_mesh/base/CoordinateSystems.hpp>
#include <stk_io/StkMeshIoBroker.hpp>
#include <stk_mesh/base/Field.hpp>

struct NewNodeData
{
  stk::mesh::EntityId m_node1;
  stk::mesh::EntityId m_node2;
  stk::mesh::Entity m_new_node;
  std::vector<int> m_sharing_procs;
  std::vector<std::pair<int, stk::mesh::EntityId> > m_id_proc_pairs_from_all_procs;
  stk::mesh::BulkData *m_old_mesh;
  stk::mesh::BulkData *m_new_mesh;
  bool m_id_procs_pairs_have_been_sorted;

  NewNodeData(stk::mesh::BulkData& oldBulkData, 
              stk::mesh::BulkData& newBulkData, 
              stk::mesh::EntityId nodeA, 
              stk::mesh::EntityId nodeB, 
              stk::mesh::Entity **entity_place_holder=NULL)
              : m_node1(0), 
                m_node2(0), 
                m_sharing_procs(), 
                m_id_proc_pairs_from_all_procs(), 
                m_old_mesh(&oldBulkData),
                m_new_mesh(&newBulkData),
                m_id_procs_pairs_have_been_sorted(false) 
  {
    // If this represents a node that is inbetween two other nodes on an edge...
    if(nodeA != nodeB)
    {
      if ( nodeA < nodeB )
      {
        m_node1 = nodeA;
        m_node2 = nodeB;
      }
      else
      {
        m_node1 = nodeB;
        m_node2 = nodeA;
      }
    }
    // If this represents a node that will be a duplicate of an existing node...
    else
    {
      m_node1 = m_node2 = nodeA;
    }
  }
  
  void add_proc_id_pair(int proc_id, stk::mesh::EntityId id)
  {
    m_id_proc_pairs_from_all_procs.push_back(std::make_pair(proc_id, id));
  }

  void calculate_sharing_procs()
  {
    if(this->node1() != this->node2())
    {
      std::vector<int> sharingProcsA;
      stk::mesh::EntityKey keyA(stk::topology::NODE_RANK, m_node1);
      m_old_mesh->comm_shared_procs(keyA, sharingProcsA);

      std::vector<int> sharingProcsB;
      stk::mesh::EntityKey keyB(stk::topology::NODE_RANK, m_node2);
      m_old_mesh->comm_shared_procs(keyB, sharingProcsB);

      std::sort(sharingProcsA.begin(), sharingProcsA.end());
      std::sort(sharingProcsB.begin(), sharingProcsB.end());

      std::set_intersection(sharingProcsA.begin(),sharingProcsA.end(),sharingProcsB.begin(),sharingProcsB.end(),std::back_inserter(m_sharing_procs));
    }
    else
    {
      stk::mesh::EntityKey keyA(stk::topology::NODE_RANK, m_node1);
      m_old_mesh->comm_shared_procs(keyA, m_sharing_procs);
    }
  }

  size_t num_sharing_procs() const
  {
    return m_sharing_procs.size();
  }

  int sharing_proc(int index) const
  {
    return m_sharing_procs[index];
  }
  
  stk::mesh::EntityId suggested_node_id() const
  {
    return m_id_proc_pairs_from_all_procs[0].second;
  }

  stk::mesh::EntityId node1() const
  {
    return m_node1;
  }

  stk::mesh::EntityId node2() const
  {
    return m_node2;
  }

  void sort_id_proc_pairs()
  {
    m_id_procs_pairs_have_been_sorted = true;
    std::sort(m_id_proc_pairs_from_all_procs.begin(), m_id_proc_pairs_from_all_procs.end());
  }

  stk::mesh::EntityId get_id_for_node() const
  {
    return m_id_proc_pairs_from_all_procs[0].second;
  }

  void set_node_entity(double &x, double &y, double &z, 
                       stk::mesh::Field<double, stk::mesh::Cartesian> *field)
  {
    this->sort_id_proc_pairs();
    stk::mesh::EntityId id_for_node = this->get_id_for_node();
    m_new_node = m_new_mesh->declare_node(id_for_node);
    double* node_coords = stk::mesh::field_data(*field, m_new_node);
    node_coords[0] = x;
    node_coords[1] = y;
    node_coords[2] = z;
    for (size_t i=0;i<m_id_proc_pairs_from_all_procs.size();++i)
    {
      if ( m_id_proc_pairs_from_all_procs[i].first != m_new_mesh->parallel_rank() )
      {
        m_new_mesh->add_node_sharing(m_new_node, m_id_proc_pairs_from_all_procs[i].first);
      }
    }
  }

  bool operator==(const NewNodeData& otherNodeData) const
  {
    if(this->node1() != this->node2())
    {
      if ( this->node1() == otherNodeData.node1() &&
           this->node2() == otherNodeData.node2() )
      {
        return true;
      }
    }
    else
    {
      if ( this->node1() == otherNodeData.node1())
        return true;
    }
    return false;
  }
};

struct NodeDataSorter
{
  bool operator()(const NewNodeData& data1, const NewNodeData& data2) const
  {
    if(data1.node1() != data1.node2()) // if this is between two nodes on an edge
    {
      if ( data1.node1() != data2.node1() )
      {
        return data1.node1() < data2.node1();
      }
      else
      {
        return data1.node2() < data2.node2();
      }
    }
    else // this is just a new node being created with the same location as an existing node
    {
      return data1.node1() < data2.node1();
    }
  }
};

class PruneMeshAPISTK : public PruneMeshAPI
{
  
private:
  stk::mesh::BulkData *mBulkData;
  stk::mesh::MetaData *mMetaData;
  stk::io::StkMeshIoBroker *mIoBroker;
  stk::ParallelMachine *mComm;
  bool mLocallyOwnedBulk;
  bool mLocallyOwnedMeta;
  stk::mesh::Field<double, stk::mesh::Cartesian> *mCoordsField;
  std::vector<stk::mesh::Field<double> *> mIsoFields;
  std::vector<stk::mesh::Field<double>*> mNodalFields;
  std::vector<stk::mesh::Field<double>*> mElementFields;
  std::vector<std::string> mFixedBlocks;
  stk::mesh::Field<double> *mFixedTriMap, *mOptimizedTriMap;
  std::vector<stk::mesh::EntityId> mAvailableElemIds;
  std::vector<stk::mesh::EntityId> mAvailableNodeIds;
  size_t mNextAvailableNodeIndex;
  size_t mNextAvailableElemIndex;
  int mTimeStep;

public:

  // Constructors
  PruneMeshAPISTK(stk::ParallelMachine* comm,
                stk::mesh::BulkData* bulk_data,
                stk::mesh::MetaData* meta_data,
                std::string fieldname);
  PruneMeshAPISTK(stk::ParallelMachine* comm);

  // Destructor
  virtual ~PruneMeshAPISTK();

  // functions from base class
  virtual void transfer_output_fields(PruneHandle n1, PruneHandle n2, PruneHandle new_node,
                   double mu, PruneMeshAPI *output_mesh_api);
  virtual void copy_node_output_fields(PruneHandle n1, PruneHandle new_node, PruneMeshAPI *output_mesh_api);
  virtual void copy_element_output_fields(PruneHandle e1, PruneHandle e2, PruneMeshAPI *output_mesh_api);
  virtual PruneHandle new_node( double &x, double &y, double &z );
  virtual void get_fixed_block_nodes(std::vector<PruneHandle> &fixed_block_nodes);
  virtual PruneHandle get_new_node_id();
  virtual PruneHandle new_tri( PruneHandle n1, PruneHandle n2, PruneHandle n3, bool is_fixed, PruneHandle source_elem );
  virtual int element_nodes(PruneHandle elem, PruneHandle nodes[8]) const;
  virtual void hex_nodes(PruneHandle hex, PruneHandle nodes[8]) const;
  virtual void tet_nodes(PruneHandle tet, PruneHandle nodes[4]) const;
  virtual void hex_quad_nodes(PruneHandle hex, int index, PruneHandle nodes[4]) const;
  virtual PruneHandle get_connected_hex(PruneHandle hex, PruneHandle n1, PruneHandle n2, PruneHandle n3, PruneHandle n4) const;
  virtual PruneHandle get_connected_tet(PruneHandle tet, PruneHandle n1, PruneHandle n2, PruneHandle n3) const;
  virtual void node_coordinates( PruneHandle node, double &x, double &y, double &z );
  virtual double get_max_nodal_iso_field_variable(PruneHandle node) const;
  virtual void store_tri_to_tet_map_entry(const PruneHandle &tri, const PruneHandle &tet);
  virtual void store_tet_to_tri_map_entry(const PruneHandle &tet, const PruneHandle &tri);
  virtual void get_shared_boundary_nodes(std::set<PruneHandle> &shared_boundary_nodes);
  virtual void get_attached_elements(const std::set<PruneHandle> &nodes,
                                     std::vector<PruneHandle> &attached_elements);
  virtual void batch_create_edge_boundary_nodes(std::vector<BoundaryNodeInfo> &boundary_info,
                                          PruneMeshAPI *existing_mesh);
  virtual void batch_create_duplicate_nodes(std::vector<DuplicateNodeInfo> &dup_node_infos, 
                                            PruneMeshAPI *existing_mesh);
  virtual void print_boundary_node_info(std::vector<BoundaryNodeInfo> &bni);
  virtual void reserve_new_node_ids(uint64_t num_requested);
  virtual void reserve_new_tri_ids(uint64_t num_requested);
  virtual double calculate_average_edge_length(const std::vector<PruneHandle> &elem_list);
  virtual void set_min_node_id(BoundaryNodeInfo &bni, const PruneHandle &n);
  virtual void set_max_node_id(BoundaryNodeInfo &bni, const PruneHandle &n);
  virtual void set_existing_node_id(DuplicateNodeInfo &dni, const PruneHandle &n);
  virtual void add_element_to_survivor_block(PruneHandle entity);
  
  // functions local to this derived class
  int get_connected_elem(stk::mesh::Entity elem, 
           std::vector<stk::mesh::Entity> &face_nodes,
           stk::mesh::Entity &connected_elem) const;
  int get_connected_hex(stk::mesh::Entity hex, 
                                      stk::mesh::Entity n1, 
                                      stk::mesh::Entity n2,  
                                      stk::mesh::Entity n3, 
                                      stk::mesh::Entity n4,
                                      stk::mesh::Entity &connected_hex) const;
  void set_fixed_block_ids(std::string fixed_block_string);
  bool prepare_as_source();
  bool prepare_as_destination();
  void initialize();
  void export_my_mesh();
  stk::mesh::EntityId get_next_entity_id(stk::topology::rank_t rank);
  stk::mesh::EntityId entity_id(PruneHandle &h);
  void prepare_new_block();
  void get_output_fields(std::string &outputFieldsString);
  stk::mesh::MetaData* meta_data() { return mMetaData; }
  stk::mesh::BulkData* bulk_data() { return mBulkData; }
  bool read_exodus_mesh(std::string &meshfile, std::string &fieldname, 
                        std::string &outputFieldsString,
                        int input_file_is_spread, int time_step);
  void write_exodus_mesh(std::string &meshfile, int concatenate, int iso_only);
  void set_comm(stk::ParallelMachine* comm) { mComm = comm; }
  stk::ParallelMachine* get_comm() { return mComm; }
  void set_bulk_data_ptr(stk::mesh::BulkData *bp) { mBulkData = bp; }
  void set_meta_data_ptr(stk::mesh::MetaData *mp) { mMetaData = mp; }
  PruneHandle get_handle(const stk::mesh::Entity &entity) const;
  stk::mesh::Entity get_stk_entity(const PruneHandle &handle) const;
  int time_step() { return mTimeStep; };
  void time_step(int val) { mTimeStep = val; };
};

#endif // PruneMeshAPISTK_____HPP



