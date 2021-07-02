/*
 * MeshManager.cpp
 *
 *  Created on: Jul 28, 2017
 *      Author: tzirkle
 */

#include "MeshManager.hpp"

#include <PerceptMesh.hpp>
#include "PruneMeshAPISTK.hpp"
#include "PruneMeshAPI.hpp"


using namespace percept;
        MeshManager::MeshManager():
            mComm(MPI_COMM_WORLD),
            mPerceptMeshIn(percept::PerceptMesh(3u)),
            mPerceptMeshOut(percept::PerceptMesh(3u))

        {
            mPruneFlag = 1;
            mTransferFlag = 1;
        }

        void MeshManager::setup_mesh(int argc, char *argv[])
        {
            read_command_line(argc, argv);
            if(stk::parallel_machine_rank(mComm) == 0)
                std::cout << "Mesh Name: " << mMeshWithVariable << std::endl;
            build_stk_mesh();
        }

        stk::mesh::BulkData& MeshManager::get_input_bulk_data()
        {
            return *mPerceptMeshIn.get_bulk_data();
        }

        stk::mesh::BulkData& MeshManager::get_output_bulk_data()
        {
            return *mPerceptMeshOut.get_bulk_data();
        }

        stk::mesh::MetaData& MeshManager::get_output_meta_data()
        {
            return *mPerceptMeshOut.get_fem_meta_data();
        }

        stk::mesh::Field<double,stk::mesh::Cartesian3d>* MeshManager::get_input_coordinate_field()
        {
            return mPerceptMeshIn.get_fem_meta_data()->get_field<stk::mesh::Field<double,stk::mesh::Cartesian3d>>(stk::topology::NODE_RANK,"coordinates");
        }

        stk::mesh::Field<double,stk::mesh::Cartesian3d>* MeshManager::get_output_coordinate_field()
        {
            return mPerceptMeshOut.get_fem_meta_data()->get_field<stk::mesh::Field<double,stk::mesh::Cartesian3d>>(stk::topology::NODE_RANK,"coordinates");
        }

        stk::mesh::FieldBase* MeshManager::get_input_transfer_field()
        {
            return mPerceptMeshIn.get_fem_meta_data()->get_field<stk::mesh::Field<double>>(stk::topology::NODE_RANK,mTransferFieldName);
        }

        stk::mesh::FieldBase* MeshManager::get_output_transfer_field()
        {
            return mPerceptMeshOut.get_fem_meta_data()->get_field<stk::mesh::Field<double>>(stk::topology::NODE_RANK,mTransferFieldName);
        }


        PerceptMesh & MeshManager::get_output_percept()
        {
            return mPerceptMeshOut;
        }

        void MeshManager::write_mesh()
        {
            if(stk::parallel_machine_rank(mComm) == 0)
                std::cout<<"Writing output mesh.."<<std::endl;

            mPerceptMeshOut.save_as(mResultMesh);

            if(stk::parallel_machine_rank(mComm) == 0)
                std::cout<<"Done writing output mesh.."<<std::endl;
        }



        void MeshManager::reopen_percept_mesh()
        {
            mPerceptMeshOut.reopen();
        }

        std::string MeshManager::get_transfer_field_name()
        {
            return mTransferFieldName;
        }

        int MeshManager::get_refines()
        {
            return mNumberOfRefines;
        }

        int MeshManager::get_buffer_layers()
        {
            return mNumberOfBufferLayers;
        }

        int MeshManager::get_prune_flag()
        {
            return mPruneFlag;
        }

        int MeshManager::get_transfer_flag()
        {
            return mTransferFlag;
        }

        stk::ParallelMachine* MeshManager::get_communicator()
        {
            return &mComm;
        }

        stk::ParallelMachine MeshManager::get_u_communicator()
        {
            return mComm;
        }

        void MeshManager::define_iso_fields()
        {
            m_isoFields.clear();


           stk::mesh::Field<double> *cur_field = mPerceptMeshOut.get_fem_meta_data()->get_field<stk::mesh::Field<double> >(
                                              stk::topology::NODE_RANK, mTransferFieldName);

            m_isoFields.push_back(cur_field);
        }


        double MeshManager::get_max_nodal_iso_field_variable(PruneHandle node) const
        {
            double max_val = 0.0;
            stk::mesh::Entity node_entity = get_stk_entity(node);
            for(size_t i=0; i<m_isoFields.size(); ++i)
            {
              double* vals = stk::mesh::field_data(*(m_isoFields[i]), node_entity);
              if(*vals > max_val)
                max_val = *vals;
            }
            return max_val;
        }

        stk::mesh::Entity MeshManager::get_stk_entity(const PruneHandle &handle) const
            {
              stk::mesh::Entity entity;
              entity.m_value = handle;
              return entity;
            }

        int MeshManager::get_connected_elem(stk::mesh::Entity elem,
                       std::vector<stk::mesh::Entity> &face_nodes,
                       stk::mesh::Entity &connected_elem)

            {
              std::vector<stk::mesh::Entity> other_nodes;
              std::vector<stk::mesh::Entity>::iterator it = face_nodes.begin();

              ++it; // don't grab the first one
              while(it != face_nodes.end())
              {
                other_nodes.push_back(*it);
                ++it;
              }


              stk::mesh::Entity const *node_elems =mPerceptMeshOut.get_bulk_data()->begin_elements(face_nodes[0]);
              int num_elems = mPerceptMeshOut.get_bulk_data()->num_elements(face_nodes[0]);
              for(int i=0; i<num_elems; ++i)
              {
                stk::mesh::Entity cur_elem = node_elems[i];
                if(cur_elem != elem)
                {
                  stk::mesh::Entity const *elem_nodes = mPerceptMeshOut.get_bulk_data()->begin_nodes(cur_elem);
                  int num_elem_nodes = mPerceptMeshOut.get_bulk_data()->num_nodes(cur_elem);
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

        void MeshManager::get_shared_boundary_nodes(std::set<PruneHandle> &shared_boundary_nodes)
        {
          stk::mesh::Selector shared(mPerceptMeshOut.get_fem_meta_data()->globally_shared_part());
          const stk::mesh::BucketVector& shared_node_buckets = mPerceptMeshOut.get_bulk_data()->get_buckets(stk::topology::NODE_RANK,
                                                           shared);
          for(size_t i=0; i<shared_node_buckets.size(); ++i)
          {
            for(size_t j=0; j<shared_node_buckets[i]->size(); ++j)
            {
              stk::mesh::Entity cur_node = (*(shared_node_buckets[i]))[j];
              shared_boundary_nodes.insert(cur_node.m_value);
            }
          }
        }

        stk::mesh::EntityId MeshManager::entity_id(PruneHandle &h)
        {
          stk::mesh::Entity ent = get_stk_entity(h);
          stk::mesh::EntityId ent_id = mPerceptMeshOut.get_bulk_data()->identifier(ent);
          return ent_id;
        }

        PruneHandle MeshManager::get_handle(const stk::mesh::Entity &entity) const
        {
          return entity.m_value;
        }

        int MeshManager::element_nodes(PruneHandle elem, PruneHandle nodes[8])
        {
          stk::mesh::Entity e = get_stk_entity(elem);
          stk::mesh::Entity const *elem_nodes = mPerceptMeshOut.get_bulk_data()->begin_nodes(e);
          int num_nodes = mPerceptMeshOut.get_bulk_data()->num_nodes(e);
          for(int i=0; i<num_nodes; ++i)
            nodes[i] = get_handle(elem_nodes[i]);
          return num_nodes;
        }


        bool MeshManager::read_command_line( int argc, char *argv[])
        {
          Teuchos::CommandLineProcessor clp;
          clp.setDocString("extract_iso options");

          clp.setOption("mesh_with_variable",  &mMeshWithVariable, "mesh with result variables from previuos run (ExodusII).", false );
          clp.setOption("mesh_to_be_pruned",  &mMeshToBePruned, "mesh to be pruned and refined (ExodusII).", true );
          clp.setOption("result_mesh",&mResultMesh,"filename for result mesh (ExodusII).",true);
          clp.setOption("field_name",  &mTransferFieldName, "field defining level set data.", false );
          clp.setOption("number_of_refines", &mNumberOfRefines,"number of desired refines.",false);
          clp.setOption("number_of_buffer_layers",&mNumberOfBufferLayers,"number of desired buffer layers.",false);
          clp.setOption("prune_mesh",&mPruneFlag,"whether to prune the mesh.",false);

          Teuchos::CommandLineProcessor::EParseCommandLineReturn parseReturn =
                             Teuchos::CommandLineProcessor::PARSE_SUCCESSFUL;
          try
          {
            parseReturn = clp.parse( argc, argv );
          }
          catch (std::exception& exc)
          {
              if(stk::parallel_machine_rank(mComm) == 0)
                  std::cout << "Failed to parse the command line arguments." << std::endl;
              return false;
          }

          if ( parseReturn == Teuchos::CommandLineProcessor::PARSE_SUCCESSFUL )
            return true;

          if(stk::parallel_machine_rank(mComm) == 0)
              std::cout << "Failed to parse the command line arguments." << std::endl;
          return false;
        }

        void MeshManager::build_stk_mesh()
        {
            if(stk::parallel_machine_rank(mComm) == 0)
                std::cout<<"Reading in input Mesh..."<<std::endl;

            if(mMeshWithVariable != "")
            {
                mPerceptMeshIn.open(mMeshWithVariable);
                mPerceptMeshIn.commit();
                int tTimeStep = mPerceptMeshIn.get_database_time_step_count();
                mPerceptMeshIn.read_database_at_step(tTimeStep);
            }
            else
                mTransferFlag = 0;

            mPerceptMeshOut.open(mMeshToBePruned);
            if(mTransferFieldName != "")
                mPerceptMeshOut.add_field(mTransferFieldName,stk::topology::NODE_RANK);
            mPerceptMeshOut.commit();

        }



