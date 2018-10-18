/*------------------------------------------------------------------------*/
/*                 Copyright 2010, 2011 Sandia Corporation.                     */
/*  Under terms of Contract DE-AC04-94AL85000, there is a non-exclusive   */
/*  license for use of this work by or on behalf of the U.S. Government.  */
/*  Export of this program may require a license from the                 */
/*  United States Government.                                             */
/*------------------------------------------------------------------------*/
#ifndef support_structure_SupportStructure_hpp
#define support_structure_SupportStructure_hpp

#include <stdexcept>
#include <sstream>
#include <vector>
#include <iostream>

#include <stk_mesh/base/Field.hpp>
#include <stk_mesh/base/CoordinateSystems.hpp>

#include "Teuchos_CommandLineProcessor.hpp"
#include "MeshWrapper.hpp"

namespace stk
{
namespace io
{
class StkMeshIoBroker;
}
}

namespace plato
{

namespace support_structure
{

struct proc_node_map
{
    int processor;
    std::vector<uint64_t> nodes;
};

class SupportStructure
{
public:
    SupportStructure(){}
    ~SupportStructure();
    bool createMeshAPIsStandAlone(int argc, char **argv);
    bool run();

private:
    bool runPrivate();
    bool readCommandLine(int argc, char *argv[]);
    bool initSingleMeshAPIs();
    void calculateSharingMaps(std::vector<proc_node_map> &aProcs,
                              MeshWrapper *aMesh);
    bool nodeNeedsSupport(stk::mesh::Entity &aCurNode, uint64_t &aNewSupportNodeLocalId,
                          std::set<uint64_t> &aExistingSupportMaterialNodes);
    void communicateBoundaryInfo(std::vector<proc_node_map> &procs,
                                 std::set<uint64_t> &aSupportNodes,
                                 std::set<uint64_t> &aAllSupportNodes,
                                 std::vector<uint64_t> &aNodesToCheck);

    stk::ParallelMachine *mComm;
    std::string mMeshIn;
    std::string mMeshOut;
    std::string mDesignFieldName;
    std::string mOutputFields;
    std::string mBuildPlateNormalString;
    double mDesignFieldThresholdValue;
    double mBuildPlateNormal[3];
    int mReadSpreadFile;
    int mConcatenateResults;
    int mTimeStep;
    MeshWrapper *mSTKMeshIn;
    MeshWrapper *mSTKMeshOut;
};


}//namespace support_structure
}//namespace plato

#endif
