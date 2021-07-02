/*
 * PerceptPrune.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: tzirkle
 */


#include "PerceptPrune.hpp"
#include "PruneMeshAPI.hpp"
#include "PruneMeshAPISTK.hpp"
#include "PerceptPruneTool.hpp"

#include <stk_io/StkMeshIoBroker.hpp>
#include <stk_io/IossBridge.hpp>
#include <stk_mesh/base/CreateFaces.hpp>
#include <stk_mesh/base/CreateEdges.hpp>
#include <stk_mesh/base/MetaData.hpp>
#include <stk_mesh/base/DestroyElements.hpp>

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

#include <Ioss_SubSystem.h>

#include <mpi.h>

//useless

namespace prune
{

PerceptPrune::PerceptPrune() :
                mMeshIn(),
                mMeshOut(),
                mFieldName(),
                mOutputFieldsString(),
                mFixedBlocksString(),
                mOutputFieldNames(),
                mIsoValue(-1),
                mNumBufferLayers(-1),
                mMinEdgeLength(-1),
                mReadSpreadFile(-1),
                mRemoveIslands(-1),
                mConcatenateResults(-1),
                mAllowNonmanifoldConnections(-1),
                mIsoOnly(-1)
{
}

PerceptPrune::~PerceptPrune()
{
}

bool PerceptPrune::import(int argc, char **argv,
                          std::string fieldName,
                          std::string outputFieldsString,
                          double minEdgeLength,
                          double isoValue,
                          int concatenateResults,
                          int isoOnly,
                          int readSpreadFile,
                          int allowNonmanifoldConnections,
                          int numberOfBufferLayers)
{
    mFieldName = fieldName;
    mMinEdgeLength = minEdgeLength;
    mIsoValue = isoValue;
    mConcatenateResults = concatenateResults;
    mAllowNonmanifoldConnections = allowNonmanifoldConnections;
    mIsoOnly = isoOnly;
    mReadSpreadFile = readSpreadFile;
    mOutputFieldsString = outputFieldsString;
    mNumBufferLayers = numberOfBufferLayers;
    //  mCleanUpOrphanNodes = 1;
    mRemoveIslands = 1;

    return true;
}


bool PerceptPrune::run_percept_mesh_stand_alone(MeshManager & aMeshManager)
{
    bool return_val;
    return_val = run_percept_mesh_private_stand_alone(aMeshManager);
    return return_val;
}

bool PerceptPrune::run_percept_mesh_private_stand_alone(MeshManager & aMeshManager)
{
    bool return_val = true;

    percept::PerceptMesh & meshIn = aMeshManager.get_output_percept();

    if(meshIn.get_bulk_data()->parallel_rank() == 0)
        std::cout << "Pruning mesh. " << std::endl;

    stk::mesh::Selector myselector = meshIn.get_fem_meta_data()->locally_owned_part();
    stk::mesh::BucketVector const &elem_buckets = meshIn.get_bulk_data()->get_buckets(
            stk::topology::ELEM_RANK, myselector );

    if(elem_buckets.size() == 0)
    {
        std::cout << "Failed to find hexes or tets to prune." << std::endl;
        //    return false;
    }

    // Get the total number of elements
    int total_num_elems=0;
    for ( stk::mesh::BucketVector::const_iterator bucket_iter = elem_buckets.begin();
            bucket_iter != elem_buckets.end();
            ++bucket_iter )
    {
        stk::mesh::Bucket &tmp_bucket = **bucket_iter;
        stk::topology bucket_top = tmp_bucket.topology();
        if(bucket_top == stk::topology::HEX_8 ||
                bucket_top == stk::topology::TET_4)
        {
            total_num_elems += tmp_bucket.size();
        }
    }

    // Get the handles for all of the elements
    size_t cntr=0;
    std::vector<PruneHandle> elem_handle_list(total_num_elems);
    for ( stk::mesh::BucketVector::const_iterator bucket_iter = elem_buckets.begin();
            bucket_iter != elem_buckets.end();
            ++bucket_iter )
    {
        stk::mesh::Bucket &tmp_bucket = **bucket_iter;
        stk::topology bucket_top = tmp_bucket.topology();
        if(bucket_top == stk::topology::HEX_8 ||
                bucket_top == stk::topology::TET_4)
        {
            size_t num_elems = tmp_bucket.size();
            for (size_t i=0; i<num_elems; ++i)
            {
                stk::mesh::Entity cur_elem = tmp_bucket[i];
                //      elem_handle_list[cntr] = meshIn.get_handle(cur_elem);
                elem_handle_list[cntr] = cur_elem.m_value;
                ++cntr;
            }
        }
    }

    PerceptPruneTool pruner;

    std::set<PruneHandle> elems_to_keep;
    // Call the element specific functions for pruning the mesh
    pruner.prune_percept_mesh(elem_handle_list, elems_to_keep, mNumBufferLayers, mAllowNonmanifoldConnections, mRemoveIslands, aMeshManager);

    stk::mesh::EntityVector ent_vec;
    // Delete elements that were pruned out.
    for(size_t i=0; i<elem_handle_list.size(); ++i)
    {
        PruneHandle cur_elem = elem_handle_list[i];
        if(elems_to_keep.find(cur_elem) == elems_to_keep.end())
        {
            //      stk::mesh::Entity cur_ent = meshIn.get_stk_entity(cur_elem);
            stk::mesh::Entity cur_ent = aMeshManager.get_stk_entity(cur_elem);
            ent_vec.push_back(cur_ent);
        }
    }

    // This function calls BulkData::modification_begin/end.
    destroy_elements(*(meshIn.get_bulk_data()), ent_vec);

    // We also need to destroy any RBAR elements that are no longer
    // hooked to a hex.
    std::vector<stk::mesh::Entity> beams_to_delete;
    myselector = meshIn.get_fem_meta_data()->locally_owned_part();
    stk::mesh::BucketVector const &beam_buckets = meshIn.get_bulk_data()->get_buckets(
            stk::topology::ELEM_RANK, myselector );
    stk::topology bucket_top;
    for ( stk::mesh::BucketVector::const_iterator bucket_iter = beam_buckets.begin();
            bucket_iter != beam_buckets.end();
            ++bucket_iter )
    {
        stk::mesh::Bucket &tmp_bucket = **bucket_iter;
        bucket_top = tmp_bucket.topology();
        if(bucket_top == stk::topology::BEAM_2 ||
                bucket_top == stk::topology::BEAM_3 ||
                bucket_top == stk::topology::LINE_2_1D ||
                bucket_top == stk::topology::LINE_3_1D)
        {
            size_t num_beam_elems = tmp_bucket.size();
            for (size_t i=0; i<num_beam_elems; ++i)
            {
                stk::mesh::Entity cur_beam = tmp_bucket[i];
                stk::mesh::Entity const *elem_nodes = meshIn.get_bulk_data()->begin_nodes(cur_beam);
                int num_nodes = meshIn.get_bulk_data()->num_nodes(cur_beam);
                bool connected_to_elem = false;
                for(int j=0; j<num_nodes && !connected_to_elem; ++j)
                {
                    stk::mesh::Entity const *node_elems = meshIn.
                            get_bulk_data()->begin_elements(elem_nodes[j]);
                    int num_elems = meshIn.get_bulk_data()->num_elements(elem_nodes[j]);
                    std::vector<stk::mesh::Entity> node_hexes;
                    for(int k=0; k<num_elems && !connected_to_elem; ++k)
                    {
                        stk::mesh::Entity cur_elem = node_elems[k];
                        stk::topology cur_top = meshIn.get_bulk_data()->bucket(cur_elem).topology();
                        if(cur_top == stk::topology::HEX_8 ||
                                cur_top == stk::topology::TET_4)
                            connected_to_elem = true;
                    }
                }
                if(!connected_to_elem)
                    beams_to_delete.push_back(cur_beam);
            }
        }
    }

    if(beams_to_delete.size() > 0)
        destroy_elements(*(meshIn.get_bulk_data()), beams_to_delete);

    if(meshIn.get_bulk_data()->parallel_rank() == 0)
    {
        std::cout << "Done pruning mesh. " << std::endl;
    }

    return return_val;
}


} //namespace prune


