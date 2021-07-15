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

#ifndef extract_STKExtract_hpp
#define extract_STKExtract_hpp

#include <stdexcept>
#include <sstream>
#include <vector>
#include <iostream>

#include <stk_mesh/base/Field.hpp>
#include <stk_mesh/base/CoordinateSystems.hpp>

#include "Teuchos_CommandLineProcessor.hpp"
#include "IVEMeshAPISTK.hpp"

  namespace stk
  {
    namespace io
    {
      class StkMeshIoBroker;
    }
  }

  namespace iso
  {

    class STKExtract
    {
    public:
      STKExtract() : 
         mAvailableFormats({"EXODUS","STL"}),
         mMeshAPIIn(nullptr),
         mMeshAPIOut(nullptr) {}
      ~STKExtract();
      bool create_mesh_apis_read_from_file(stk::ParallelMachine *comm,
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
                             int timeStep);
      bool create_mesh_apis_stand_alone(int argc, char **argv,
                             std::string meshIn,
                             std::string meshOut,
                             std::string fieldName,
                             std::string outputFieldsString,
                             double minEdgeLength,              
                             double isoValue,
                             int levelSetData,
                             int outputMethod,
                             int isoOnly,
                             int readSpreadFile);
      bool create_mesh_apis_with_existing_stk_mesh(stk::ParallelMachine *comm,
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
                             std::string outputFieldsString);
      bool run_stand_alone();
      bool run_extraction(int iteration, int num_materials);
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

      std::vector<std::string> availableFormats() { return mAvailableFormats; }

    private:
      void concatenate_stl_files(std::string &filename);
      void write_tris_to_stl(FILE *fp, std::vector<IVEHandle> &tri_list);
      bool run_private_stand_alone();
      bool read_command_line(int argc, char *argv[]);
      bool init_single_mesh_apis();

      double mMinx, mMiny, mMinz, mMaxx, mMaxy, mMaxz;
      double mAverageEdgeLength;
      stk::ParallelMachine *mComm;
      std::string mMeshIn;
      std::string mMeshOut;
      std::string mFieldName;
      std::string mOutputFieldsString;
      std::string mFixedBlocksString;
      std::vector<std::string> mOutputFieldNames;
      std::vector<std::string> mRequestedFormats;
      std::vector<std::string> mAvailableFormats;
      double mIsoValue;
      double mMinEdgeLength;
      int mLevelSetData;
      int mReadSpreadFile;
      int mOutputMethod;
      int mIsoOnly;
      int mTimeStep;
      int mOutputSTL;
      IVEMeshAPISTK *mMeshAPIIn;
      IVEMeshAPISTK *mMeshAPIOut;
    };

      
  }//namespace iso

#endif
