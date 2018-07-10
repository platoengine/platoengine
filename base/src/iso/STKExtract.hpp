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
      STKExtract(){}
      ~STKExtract();
      bool create_mesh_apis_read_from_file(stk::ParallelMachine *comm,
                             std::string meshIn,
                             std::string meshOut,
                             std::string fieldName,
                             std::string outputFieldsString,
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
      double mIsoValue;
      double mMinEdgeLength;
      int mLevelSetData;
      int mReadSpreadFile;
      int mOutputMethod;
      int mIsoOnly;
      int mTimeStep;
      IVEMeshAPISTK *mMeshAPIIn;
      IVEMeshAPISTK *mMeshAPIOut;
    };

      
  }//namespace iso

#endif
