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

#include "STKExtract.hpp"
#include "IVEMeshAPI.hpp"
#include "IVEMeshAPISTK.hpp"
#include "IsoVolumeExtractionTool.hpp"

#include <stk_io/StkMeshIoBroker.hpp>
#include <stk_io/IossBridge.hpp>
#include <stk_mesh/base/CreateFaces.hpp>
#include <stk_mesh/base/CreateEdges.hpp>
#include <stk_mesh/base/MetaData.hpp>

#include <Ioss_SubSystem.h>

namespace iso
{

STKExtract::~STKExtract()
{
  if(mMeshAPIOut && mMeshAPIOut != mMeshAPIIn)
    delete mMeshAPIOut;
  if(mMeshAPIIn)
    delete mMeshAPIIn;
}

bool STKExtract::init_single_mesh_apis()
{

  mMeshAPIIn = new IVEMeshAPISTK(mComm);
  mMeshAPIIn->prepare_as_source();

  mMeshAPIIn->set_fixed_block_ids(mFixedBlocksString);

  if(!mMeshAPIIn->read_exodus_mesh(mMeshIn, mFieldName, mOutputFieldsString, mReadSpreadFile, mTimeStep))
    return false;

  mMeshAPIOut = mMeshAPIIn;

  return true;
}

bool STKExtract::create_mesh_apis_stand_alone(int argc, char **argv,
                             std::string meshIn,
                             std::string meshOut,
                             std::string fieldName,
                             std::string outputFieldsString,
                             double minEdgeLength,
                             double isoValue,
                             int levelSetData,
                             int outputMethod,
                             int isoOnly,
                             int readSpreadFile)
{
  mMeshIn = meshIn;
  mMeshOut = meshOut;
  mFieldName = fieldName;
  mMeshAPIIn = NULL;
  mMeshAPIOut = NULL;
  mMinEdgeLength = minEdgeLength;
  mIsoValue = isoValue;
  mLevelSetData = levelSetData;
  mOutputMethod = outputMethod;
  mIsoOnly = isoOnly;
  mReadSpreadFile = readSpreadFile;
  mOutputFieldsString = outputFieldsString;

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

  if(mOutputSTL && !mIsoOnly)
      throw(std::runtime_error("Error: STL can only be output if --iso_only=1"));

  if(!init_single_mesh_apis())
  {
    return false;
  }

  return true;
}

bool STKExtract::create_mesh_apis_read_from_file(stk::ParallelMachine *comm,
                             std::string meshIn,
                             std::string meshOut,
                             std::string fieldName,
                             std::string outputFieldsString,
                             std::vector<std::string> requestedFormats,
                             double minEdgeLength,
                             double isoValue,
                             int levelSetData,
                             int outputMethod,
                             int isoOnly,
                             int readSpreadFile,
                             int timeStep)
{
  mComm = comm;
  mMeshIn = meshIn;
  mMeshOut = meshOut;
  mFieldName = fieldName;
  mMeshAPIIn = NULL;
  mMeshAPIOut = NULL;
  mMinEdgeLength = minEdgeLength;
  mIsoValue = isoValue;
  mLevelSetData = levelSetData;
  mOutputMethod = outputMethod;
  mIsoOnly = isoOnly;
  mReadSpreadFile = readSpreadFile;
  mTimeStep = timeStep;
  mOutputFieldsString = outputFieldsString;
  mRequestedFormats = requestedFormats;

  if(!init_single_mesh_apis())
  {
    return false;
  }

  return true;
}

bool STKExtract::create_mesh_apis_with_existing_stk_mesh(stk::ParallelMachine *comm,
                                                         const stk::mesh::BulkData *bulkData,
                                                         const stk::mesh::MetaData *metaData,
                                                         std::string meshIn,
                                                         std::string meshOut,
                                                         std::string fieldName,
                                                         double minEdgeLength,
                                                         double isoValue,
                                                         int levelSetData,
                                                         int outputMethod,
                                                         int isoOnly,
                                                         int readSpreadFile,
                                                         std::string outputFieldsString)
{
  mMeshIn = meshIn;
  mMeshOut = meshOut;
  mFieldName = fieldName;
  mMeshAPIIn = NULL;
  mMeshAPIOut = NULL;
  mMinEdgeLength = minEdgeLength;
  mIsoValue = isoValue;
  mLevelSetData = levelSetData;
  mOutputMethod = outputMethod;
  mIsoOnly = isoOnly;
  mReadSpreadFile = readSpreadFile;
  mOutputFieldsString = outputFieldsString;

  // Create non-const pointers for input bulk and meta datas
  stk::mesh::BulkData *_bulkData = (stk::mesh::BulkData*)bulkData;
  stk::mesh::MetaData *_metaData = (stk::mesh::MetaData*)metaData;

  // Create the mesh api object for the existing mesh
  mMeshAPIIn = new IVEMeshAPISTK(comm, _bulkData, _metaData, mFieldName);
  // Create an api for the output mesh
  mMeshAPIOut = new IVEMeshAPISTK(comm);
  // Get pointers to the fields that we will want to read from this mesh and
  // write out to the new mesh
  mMeshAPIIn->prepare_field_data(outputFieldsString, mMeshAPIOut);

  return true;
}

bool STKExtract::run_extraction(int iteration, int num_materials)
{
    if(mMeshOut == "")
    {
        char iter_string[100];
        sprintf(iter_string, "%d", iteration);
        mMeshOut = "Iteration";
        if(iteration < 10)
            mMeshOut += "00";
        else if(iteration < 100)
            mMeshOut += "0";
        mMeshOut += iter_string;
        mMeshOut += ".exo";
    }

    mMeshAPIIn->time_step(iteration);
    mMeshAPIOut->time_step(iteration);

    stk::mesh::Selector myselector = mMeshAPIIn->meta_data()->locally_owned_part();
    stk::mesh::BucketVector elem_buckets;
    mMeshAPIIn->get_element_buckets(myselector, elem_buckets);

    bool is_hex = true;
    if(elem_buckets.size() > 0)
    {
        stk::mesh::Bucket &bucket = *(elem_buckets[0]);
        stk::topology bucket_top = bucket.topology();
        if(bucket_top == stk::topology::TET_4)
        {
            is_hex = false;
        }
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
    std::vector<IVEHandle> elem_handle_list(total_num_elems);
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

    bool level_set_data = mLevelSetData;
    std::vector<IVEHandle> fixed_tri_list, optimized_tri_list;
    IsoVolumeExtractionTool ive;

    mMeshAPIOut->bulk_data()->modification_begin();
    // Call the element specific functions for extracting the iso volume
    if(is_hex)
    {
        ive.build_iso_volume_tris_from_hexes(elem_handle_list, mIsoValue, mMinEdgeLength,
                                             level_set_data, fixed_tri_list, optimized_tri_list, mMeshAPIIn, mMeshAPIOut);
    }
    else
    {
        ive.build_iso_volume_tris_from_tets(elem_handle_list, mIsoValue, mMinEdgeLength,
                                            level_set_data, fixed_tri_list, optimized_tri_list, mMeshAPIIn, mMeshAPIOut);
    }

    minx(ive.minx());
    miny(ive.miny());
    minz(ive.minz());
    maxx(ive.maxx());
    maxy(ive.maxy());
    maxz(ive.maxz());
    average_edge_length(ive.average_edge_length());

    mMeshAPIOut->bulk_data()->modification_end();

    
    bool hasMultipleMaterials = (num_materials > 1);
    bool requestedSTL = std::count(mRequestedFormats.begin(), mRequestedFormats.end(), "STL");
    if( hasMultipleMaterials || requestedSTL )
    {
        // Strip off the ".exo" at the end and replace it with ".stl"
        std::string stl_filename = mMeshOut.substr(0, mMeshOut.length()-4);;
        stl_filename += ".stl.";
        int num_procs = mMeshAPIOut->bulk_data()->parallel_size();
        int my_rank = mMeshAPIOut->bulk_data()->parallel_rank();
        char num_procs_str[100], my_rank_str[100];;
        sprintf(num_procs_str, "%d", num_procs);
        sprintf(my_rank_str, "%d", my_rank);
        stl_filename += num_procs_str;
        stl_filename += ".";
        int num_chars_in_num_procs = strlen(num_procs_str);
        int num_chars_in_my_rank = strlen(my_rank_str);
        for(int f=num_chars_in_my_rank; f<num_chars_in_num_procs; ++f)
            stl_filename += "0";
        stl_filename += my_rank_str;

        FILE *stl_fp = fopen(stl_filename.c_str(), "wb");
        if(stl_fp)
        {
            // write the header
            char header[80];
            for(int i = 0; i < 80; i++)
                header[i] = '-';
            fwrite(header, sizeof(char), 80, stl_fp);

            int num_tris = fixed_tri_list.size() + optimized_tri_list.size();
            int num_bytes = sizeof(int);
            fwrite(&num_tris, num_bytes, 1, stl_fp);
            write_tris_to_stl(stl_fp, fixed_tri_list);
            write_tris_to_stl(stl_fp, optimized_tri_list);
            fclose(stl_fp);
        }
    }

    mMeshAPIOut->write_exodus_mesh(mMeshOut, mOutputMethod, mIsoOnly);

    MPI_Barrier(mMeshAPIOut->bulk_data()->parallel());

    int num_procs = mMeshAPIOut->bulk_data()->parallel_size();
    // run epu and then delete temp files if neededs
    if(mMeshAPIOut->bulk_data()->parallel_rank() == 0)
    {
        if( hasMultipleMaterials || requestedSTL )
        {
            concatenate_stl_files(mMeshOut);
            if(num_procs > 1)
            {
                std::string stl_filename = mMeshOut.substr(0, mMeshOut.length()-4);;
                stl_filename += ".stl";
                std::string command = "rm ";
                command += stl_filename;
                command += ".*";
                printf("\nExecuting system call: %s\n", command.c_str());
                system(command.c_str());
            }
        }

        std::string tmp_file = mMeshOut;
        tmp_file += ".";
        char tmp_str[100];
        sprintf(tmp_str, "%d", num_procs);
        int num_chars = strlen(tmp_str);
        tmp_file += tmp_str;
        std::string save_str = tmp_file;
        tmp_file += ".";
        for(int i=0; i<num_chars; ++i)
            tmp_file += "0";

        if(num_procs > 1)
        {
            if(mOutputMethod == 2) // use epu to concatenate results
            {
                std::string command = "epu -auto " + tmp_file + " > epu.log";
                printf("\nExecuting system call: %s\n", command.c_str());
                system(command.c_str());

                command = "rm ";
                command += save_str;
                command += ".*";
                printf("\nExecuting system call: %s\n", command.c_str());
                system(command.c_str());
            }
        }
    }

    MPI_Barrier(mMeshAPIOut->bulk_data()->parallel());

    return true;
}

void STKExtract::concatenate_stl_files(std::string &filename)
{
    // Strip off the ".exo" at the end and replace it with ".stl"
    std::string stl_filename = filename.substr(0, filename.length()-4);;
    stl_filename += ".stl";

    // Start the new file that will conatin the
    // concatenated results.
    FILE *stl_fp = fopen(stl_filename.c_str(), "wb");  
    if(stl_fp)
    {
      // write the header
      char header[80];
      for(int i = 0; i < 80; i++)
        header[i] = '-';
      fwrite(header, sizeof(char), 80, stl_fp);

      // Save the position where we will write the number of facets.
      long num_facets_position;  
      num_facets_position = ftell(stl_fp);

      // Write a temporary value in for the number of tris.
      // We will come back and change it later.
      int num_tris = 0;
      int num_bytes = sizeof(int);
      fwrite(&num_tris, num_bytes, 1, stl_fp);

      // read all of the individual files and write their
      // tris to this new file.
      num_tris = 0;
      int num_procs = mMeshAPIOut->bulk_data()->parallel_size();
      char num_procs_string[11], cur_rank_string[11];
      sprintf(num_procs_string, "%d", num_procs);
      int num_chars_in_num_procs = strlen(num_procs_string);
      for(int i=0; i<num_procs; ++i)
      {
        std::string cur_filename = stl_filename;
        cur_filename += ".";
        cur_filename += num_procs_string;
        cur_filename += ".";

        sprintf(cur_rank_string, "%d", i);
        int num_chars_in_cur_rank = strlen(cur_rank_string);

        for(int w=num_chars_in_cur_rank; w<num_chars_in_num_procs; ++w)
          cur_filename += "0";
        cur_filename += cur_rank_string;

        FILE *cur_fp = fopen(cur_filename.c_str(), "rb");  
        if(cur_fp)
        {
          // Go past the header
          fseek(cur_fp, 80, SEEK_SET);

          int cur_num_tri;
          // read the number of tris
          fread(&cur_num_tri, num_bytes, 1, cur_fp);

          // increment the global num tris
          num_tris += cur_num_tri;
 
          // could read much more at once to optimized this
          for(int k=0; k<cur_num_tri; ++k)
          {
            float cur[12];
            // Read in all the data for this tri
            fread(cur, sizeof(float), 12, cur_fp);

            // read in the attribute at the end
            unsigned short attrib = 0;
            fread(&attrib, sizeof(unsigned short), 1, cur_fp);

            // write the tri info out into the concatenated file
            fwrite(cur, sizeof(float), 12, stl_fp);
            fwrite(&attrib, sizeof(unsigned short), 1, stl_fp);
          }

          fclose(cur_fp);
        }
      }

      //get the current position of file pointer
      long current_location = ftell(stl_fp);
      
      // write the number of facets now
      fseek(stl_fp, num_facets_position, SEEK_SET);
      fwrite(&num_tris, sizeof(int), 1, stl_fp);
      
      //put file pointer back where you found it
      fseek(stl_fp, current_location, SEEK_SET);

      fclose(stl_fp);
    }
}

void STKExtract::write_tris_to_stl(FILE *fp, std::vector<IVEHandle> &tri_list)
{
  for(size_t i=0; i<tri_list.size(); ++i)
  {
    IVEHandle cur_tri = tri_list[i];

    IVEHandle nodes[8];
    mMeshAPIOut->element_nodes(cur_tri, nodes);

    // get the coords for all of the vertices
    IsoVector verts[3];
    for(size_t j=0; j<3; ++j)
    {
      verts[j] = mMeshAPIOut->node_coordinates(nodes[j]);
    }

    // Calculate the normal
    IsoVector norm = (verts[1] - verts[0]) * (verts[2] - verts[1]);

    // write normal
    float float_val = norm.x(); 
    fwrite(&float_val, sizeof(float), 1, fp);
    float_val = norm.y(); 
    fwrite(&float_val, sizeof(float), 1, fp);
    float_val = norm.z(); 
    fwrite(&float_val, sizeof(float), 1, fp);

    // write the coords
    for(size_t j=0; j<3; ++j)
    {
      float_val = verts[j].x(); 
      fwrite(&float_val, sizeof(float), 1, fp);
      float_val = verts[j].y(); 
      fwrite(&float_val, sizeof(float), 1, fp);
      float_val = verts[j].z(); 
      fwrite(&float_val, sizeof(float), 1, fp);
    }

    // write the attribute
    unsigned short short_val = 0;
    fwrite(&short_val, sizeof(unsigned short), 1, fp);
  }
}

bool STKExtract::run_stand_alone()
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

  delete mMeshAPIIn;
  mMeshAPIIn = mMeshAPIOut = NULL;
  stk::parallel_machine_finalize();

  if ( return_val )
    std::cout << "Exiting Successfully " << std::endl;
  else
    std::cout << "Exiting with Failure " << std::endl;

  return return_val;
}

bool STKExtract::run_private_stand_alone()
{
  bool return_val = true;

  std::cout << "Extracting iso-volume. " << std::endl;

  stk::mesh::Selector myselector = mMeshAPIIn->meta_data()->locally_owned_part();
  stk::mesh::BucketVector elem_buckets;
  mMeshAPIIn->get_element_buckets(myselector, elem_buckets);

  bool is_hex = true;
  if(elem_buckets.size() > 0)
  {
    stk::mesh::Bucket &bucket = *(elem_buckets[0]);
    stk::topology bucket_top = bucket.topology();
    if(bucket_top == stk::topology::TET_4)
    {
      is_hex = false;
    }
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
  std::vector<IVEHandle> elem_handle_list(total_num_elems);
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

  bool level_set_data = mLevelSetData;
  std::vector<IVEHandle> fixed_tri_list, optimized_tri_list;
  IsoVolumeExtractionTool ive;

  mMeshAPIOut->bulk_data()->modification_begin();

  // Call the element specific functions for extracting the iso volume 
  if(is_hex)
  {
    ive.build_iso_volume_tris_from_hexes(elem_handle_list, mIsoValue, mMinEdgeLength,
                 level_set_data, fixed_tri_list, optimized_tri_list, mMeshAPIIn, mMeshAPIOut);
  }
  else
  {
    ive.build_iso_volume_tris_from_tets(elem_handle_list, mIsoValue, mMinEdgeLength,
                 level_set_data, fixed_tri_list, optimized_tri_list, mMeshAPIIn, mMeshAPIOut);
  }


  mMeshAPIOut->bulk_data()->modification_end();

  std::cout << "Done extracting iso-volume. " << std::endl;

  if(mOutputSTL)
  {
        std::cout << "Writing the new mesh to STL: " << mMeshOut << std::endl;
      FILE *stl_fp = fopen(mMeshOut.c_str(), "wb");
      if(stl_fp)
      {
          // write the header
          char header[80];
          for(int i = 0; i < 80; i++)
              header[i] = '-';
          fwrite(header, sizeof(char), 80, stl_fp);

          int num_tris = fixed_tri_list.size() + optimized_tri_list.size();
          int num_bytes = sizeof(int);
          fwrite(&num_tris, num_bytes, 1, stl_fp);
          write_tris_to_stl(stl_fp, fixed_tri_list);
          write_tris_to_stl(stl_fp, optimized_tri_list);
          fclose(stl_fp);
      }
  }

  else
  {
      std::cout << "Writing the new mesh to Exodus: " << mMeshOut << std::endl;
      mMeshAPIOut->write_exodus_mesh(mMeshOut, mOutputMethod, mIsoOnly);
  }

  return return_val;
}

bool STKExtract::read_command_line( int argc, char *argv[])
{
  Teuchos::CommandLineProcessor clp;
  clp.setDocString("extract_iso options");

  clp.setOption("in_mesh",  &mMeshIn, "in mesh file (ExodusII).", true );
  clp.setOption("out_mesh",  &mMeshOut, "out mesh file (ExodusII).", false );
  clp.setOption("field_name",  &mFieldName, "field defining level set data.", false );
  clp.setOption("iso_value",  &mIsoValue, "iso value at which to extract.", false );
  clp.setOption("level_set_data",  &mLevelSetData, "specify whether data is level set data or density data.", false );
  clp.setOption("output_method",  &mOutputMethod, "Specify whether/how to concatenate resulting mesh files. (0=dont concatenate, 1=Parallel write with Ioss, 2=Use epu", false );
  clp.setOption("iso_only",  &mIsoOnly, "specify whether to only output the iso volume triangles.", false );
  clp.setOption("min_edge_length",  &mMinEdgeLength, "min edge length that will be generated during extraction.", false );
  clp.setOption("read_spread_file",  &mReadSpreadFile, "specify whether input is already decomposed.", false );
  clp.setOption("time_step",  &mTimeStep, "specify the time step to be read from the file.", false );
  clp.setOption("output_fields",  &mOutputFieldsString, "specify the fields (commma separated, no spaces) to output in the output mesh.", false );
  clp.setOption("fixed_blocks",  &mFixedBlocksString, "specify the blocks that are fixed (commma separated, no spaces).", false );
  clp.setOption("output_stl",  &mOutputSTL, "specify output in stl format rather than exodus. Can only be used if --iso_only=1", false );

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

} //namespace iso



