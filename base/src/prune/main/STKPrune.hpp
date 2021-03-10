#ifndef prune_STKPrune_hpp
#define prune_STKPrune_hpp

#include <stdexcept>
#include <sstream>
#include <vector>
#include <iostream>

#include <stk_mesh/base/Field.hpp>
#include <stk_mesh/base/CoordinateSystems.hpp>

#include "Teuchos_CommandLineProcessor.hpp"
#include "PruneMeshAPISTK.hpp"

  namespace stk
  {
    namespace io
    {
      class StkMeshIoBroker;
    }
  }

  namespace prune
  {

    class STKPrune
    {
    public:
      STKPrune(){}
      ~STKPrune();
      bool create_mesh_apis_stand_alone(int argc, char **argv,
                             std::string meshIn,
                             std::string meshOut,
                             std::string fieldName,
                             std::string outputFieldsString,
                             double minEdgeLength,              
                             double isoValue,
                             int concatenateResults,
                             int isoOnly,
                             int readSpreadFile,
                             int allowNonmanifoldConnections);
      bool run_stand_alone();

    private:
      bool run_private_stand_alone();
      bool read_command_line(int argc, char *argv[]);
      bool init_single_mesh_apis();

      stk::ParallelMachine *mComm;
      std::string mMeshIn;
      std::string mMeshOut;
      std::string mFieldName;
      std::string mOutputFieldsString;
      std::string mFixedBlocksString;
      std::vector<std::string> mOutputFieldNames;
      double mIsoValue;
      int mNumBufferLayers;
      double mMinEdgeLength;
      int mReadSpreadFile;
//      int mCleanUpOrphanNodes;
      int mRemoveIslands;
      int mConcatenateResults;
      int mAllowNonmanifoldConnections;
      int mIsoOnly;
      int mTimeStep;
      PruneMeshAPISTK *mMeshAPIIn;
      PruneMeshAPISTK *mMeshAPIOut;
    };

      
  }//namespace prune

#endif
