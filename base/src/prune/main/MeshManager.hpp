/*
 * MeshManager.hpp
 *
 *  Created on: Jul 28, 2017
 *      Author: tzirkle
 */

#ifndef ISO_PRUNE_MAIN_MESHMANAGER_HPP_
#define ISO_PRUNE_MAIN_MESHMANAGER_HPP_

#include <PerceptMesh.hpp>
#include "Teuchos_CommandLineProcessor.hpp"
#include "PruneMeshAPISTK.hpp"

using namespace percept;

class MeshManager
{
public:
           MeshManager();

           void setup_mesh(int argc, char *argv[]);

           stk::mesh::BulkData& get_input_bulk_data();

           stk::mesh::BulkData& get_output_bulk_data();

           stk::mesh::MetaData& get_output_meta_data();

           stk::mesh::Field<double,stk::mesh::Cartesian3d>* get_input_coordinate_field();

           stk::mesh::Field<double,stk::mesh::Cartesian3d>* get_output_coordinate_field();

           stk::mesh::FieldBase* get_input_transfer_field();

           stk::mesh::FieldBase* get_output_transfer_field();

           PerceptMesh &get_output_percept();

           void write_mesh();

           void define_iso_field();

           void reopen_percept_mesh();

           std::string get_transfer_field_name();

           int get_refines();

           int get_buffer_layers();

           int get_prune_flag();
           int get_transfer_flag();

           stk::ParallelMachine* get_communicator();

           stk::ParallelMachine get_u_communicator();

           void define_iso_fields();

           double get_max_nodal_iso_field_variable(PruneHandle node) const;

           stk::mesh::Entity get_stk_entity(const PruneHandle &handle) const;

           int get_connected_elem(stk::mesh::Entity elem,
                          std::vector<stk::mesh::Entity> &face_nodes,
                          stk::mesh::Entity &connected_elem);

           void get_shared_boundary_nodes(std::set<PruneHandle> &shared_boundary_nodes);

           stk::mesh::EntityId entity_id(PruneHandle &h);

           PruneHandle get_handle(const stk::mesh::Entity &entity) const;

           int element_nodes(PruneHandle elem, PruneHandle nodes[8]);


private:
        std::string mMeshWithVariable;
        std::string mMeshToBePruned;
        std::string mResultMesh;
        std::string mTransferFieldName;

        int mNumberOfRefines;
        int mNumberOfBufferLayers;
        int mPruneFlag;
        int mTransferFlag;

        stk::ParallelMachine mComm;

        PerceptMesh mPerceptMeshIn;
        PerceptMesh mPerceptMeshOut;

        std::vector<stk::mesh::Field<double> *> m_isoFields;

        bool read_command_line( int argc, char *argv[]);

        void build_stk_mesh();
};



#endif /* ISO_PRUNE_MAIN_MESHMANAGER_HPP_ */
