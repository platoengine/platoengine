#include "STKPrune.hpp"
#include "PruneMeshAPI.hpp"
#include "PruneMeshAPISTK.hpp"
#include "PruneTool.hpp"

#include <stk_io/StkMeshIoBroker.hpp>
#include <stk_io/IossBridge.hpp>
#include <stk_mesh/base/CreateFaces.hpp>
#include <stk_mesh/base/CreateEdges.hpp>
#include <stk_mesh/base/MetaData.hpp>
#include <stk_mesh/base/DestroyElements.hpp>

#include <Ioss_SubSystem.h>

namespace prune
{

STKPrune::~STKPrune()
{
  if(mMeshAPIOut && mMeshAPIOut != mMeshAPIIn)
    delete mMeshAPIOut;
  if(mMeshAPIIn)
    delete mMeshAPIIn;
}

bool STKPrune::init_single_mesh_apis()
{

  mMeshAPIIn = new PruneMeshAPISTK(mComm);
  mMeshAPIIn->prepare_as_source();

  mMeshAPIIn->set_fixed_block_ids(mFixedBlocksString);

  if(!mMeshAPIIn->read_exodus_mesh(mMeshIn, mFieldName, mOutputFieldsString, mReadSpreadFile, mTimeStep))
    return false;

  mMeshAPIOut = mMeshAPIIn;

  return true;
}

bool STKPrune::create_mesh_apis_stand_alone(int argc, char **argv,
                             std::string meshIn,
                             std::string meshOut,
                             std::string fieldName,
                             std::string outputFieldsString,
                             double minEdgeLength,
                             double isoValue,
                             int concatenateResults,
                             int isoOnly,
                             int readSpreadFile,
                             int allowNonmanifoldConnections)
{
  mMeshIn = meshIn;
  mMeshOut = meshOut;
  mFieldName = fieldName;
  mMeshAPIIn = NULL;
  mMeshAPIOut = NULL;
  mMinEdgeLength = minEdgeLength;
  mIsoValue = isoValue;
  mConcatenateResults = concatenateResults;
  mAllowNonmanifoldConnections = allowNonmanifoldConnections;
  mIsoOnly = isoOnly;
  mReadSpreadFile = readSpreadFile;
  mOutputFieldsString = outputFieldsString;
  mNumBufferLayers = 0;
//  mCleanUpOrphanNodes = 1;
  mRemoveIslands = 1;

  stk::ParallelMachine *comm = new stk::ParallelMachine(stk::parallel_machine_init(&argc, &argv));
  if(!comm)
  {
    std::cout << "Failed to initialize the parallel machine." << std::endl;
    return false;
  }
  mComm = comm;
  mTimeStep = -1;
  if ( !read_command_line( argc, argv ) )
    return false;

  if(!init_single_mesh_apis())
  {
    return false;
  }

  return true;
}

bool STKPrune::run_stand_alone()
{
  bool return_val = true;
  try
  {
    return_val = run_private_stand_alone();
  }
  catch (std::exception& exc)
  {
    delete mMeshAPIIn;
    mMeshAPIIn = mMeshAPIOut = NULL;
    stk::parallel_machine_finalize();
    return false;
  }

  if(mMeshAPIIn->bulk_data()->parallel_rank() == 0)
  {
    if ( return_val )
      std::cout << "Exiting Successfully " << std::endl;
    else
      std::cout << "Exiting with Failure " << std::endl;
  }

  delete mMeshAPIIn;
  mMeshAPIIn = mMeshAPIOut = NULL;
  stk::parallel_machine_finalize();

  return return_val;
}

bool STKPrune::run_private_stand_alone()
{
  bool return_val = true;

  if(mMeshAPIIn->bulk_data()->parallel_rank() == 0)
    std::cout << "Pruning mesh. " << std::endl;

  stk::mesh::Selector myselector = mMeshAPIIn->meta_data()->locally_owned_part();
  stk::mesh::BucketVector const &elem_buckets = mMeshAPIIn->bulk_data()->get_buckets(
                   stk::topology::ELEM_RANK, myselector );

  if(elem_buckets.size() == 0)
  {
    std::cout << "Failed to find hexes or tets to prune." << std::endl;
    return false;
  }

  // Get the total number of elements
  int total_num_elems=0;
  for ( stk::mesh::BucketVector::const_iterator bucket_iter = elem_buckets.begin();
          bucket_iter != elem_buckets.end();
          ++bucket_iter )
  {
      stk::mesh::Bucket &tmp_bucket = **bucket_iter;
      total_num_elems += tmp_bucket.size();
  }

  // Get the handles for all of the elements
  size_t cntr=0;
  std::vector<PruneHandle> elem_handle_list(total_num_elems);
  for ( stk::mesh::BucketVector::const_iterator bucket_iter = elem_buckets.begin();
          bucket_iter != elem_buckets.end();
          ++bucket_iter )
  {
      stk::mesh::Bucket &tmp_bucket = **bucket_iter;
      size_t num_elems = tmp_bucket.size();
      for (size_t i=0; i<num_elems; ++i)
      {
          stk::mesh::Entity cur_elem = tmp_bucket[i];
          elem_handle_list[cntr] = mMeshAPIIn->get_handle(cur_elem);
          ++cntr;
      }
  }

  PruneTool pruner;

  std::set<PruneHandle> elems_to_keep;
  // Call the element specific functions for pruning the mesh 
  if(elem_handle_list.size() > 0)
    pruner.prune_mesh(elem_handle_list, elems_to_keep, mNumBufferLayers, mAllowNonmanifoldConnections, mRemoveIslands, mMeshAPIIn);

  stk::mesh::EntityVector ent_vec;
  // Delete elements that were pruned out.
  for(size_t i=0; i<elem_handle_list.size(); ++i)
  {
    PruneHandle cur_elem = elem_handle_list[i];
    if(elems_to_keep.find(cur_elem) == elems_to_keep.end())
    {
      stk::mesh::Entity cur_ent = mMeshAPIIn->get_stk_entity(cur_elem);
      ent_vec.push_back(cur_ent);
    }
  }

  // This function calls BulkData::modification_begin/end.
  destroy_elements(*(mMeshAPIIn->bulk_data()), ent_vec);

  if(mMeshAPIIn->bulk_data()->parallel_rank() == 0)
  {
    std::cout << "Done pruning mesh. " << std::endl;
    std::cout << "Writing the new mesh to Exodus: " << mMeshOut << std::endl;
  }

  mMeshAPIOut->write_exodus_mesh(mMeshOut, mConcatenateResults, mIsoOnly);

  return return_val;
}

bool STKPrune::read_command_line( int argc, char *argv[])
{
  Teuchos::CommandLineProcessor clp;
  clp.setDocString("extract_iso options");

  clp.setOption("in_mesh",  &mMeshIn, "in mesh file (ExodusII).", true );
  clp.setOption("out_mesh",  &mMeshOut, "out mesh file (ExodusII).", false );
  clp.setOption("field_name",  &mFieldName, "field defining level set data.", false );
  clp.setOption("number_buffer_layers",  &mNumBufferLayers, "Number of buffer layers to add.", false );
  clp.setOption("concatenate",  &mConcatenateResults, "specify whether to concatenate resulting mesh files.", false );
  clp.setOption("allow_nonmanifold_connections",  &mAllowNonmanifoldConnections, "specify whether to allow nonmanifold connections.", false );
  clp.setOption("read_spread_file",  &mReadSpreadFile, "specify whether input is already decomposed.", false );
//  clp.setOption("clean_up_orphan_nodes",  &mCleanUpOrphanNodes, "specify whether to manually clean up orphan nodes.", false );
  clp.setOption("remove_islands",  &mRemoveIslands, "specify whether to remove diconnected islands of mesh", false );
  clp.setOption("time_step",  &mTimeStep, "specify the time step to be read from the file.", false );
  clp.setOption("output_fields",  &mOutputFieldsString, "specify the fields (commma separated, no spaces) to output in the output mesh.", false );
  clp.setOption("fixed_blocks",  &mFixedBlocksString, "specify the blocks that are fixed (commma separated, no spaces).", false );

  Teuchos::CommandLineProcessor::EParseCommandLineReturn parseReturn =
                     Teuchos::CommandLineProcessor::PARSE_SUCCESSFUL;
  try
  {
    parseReturn = clp.parse( argc, argv );
  }
  catch (std::exception& exc)
  {
    std::cout << "Failed to parse the command line arguments." << std::endl;
    return false;
  }

  if ( parseReturn == Teuchos::CommandLineProcessor::PARSE_SUCCESSFUL )
    return true;

  std::cout << "Failed to parse the command line arguments." << std::endl;
  return false;
}

} //namespace prune



