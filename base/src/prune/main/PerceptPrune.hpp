/*
 * PerceptPrune.hpp
 *
 *  Created on: Aug 1, 2017
 *      Author: tzirkle
 */

#ifndef ISO_PRUNE_MAIN_PERCEPTPRUNE_HPP_
#define ISO_PRUNE_MAIN_PERCEPTPRUNE_HPP_

#include <sstream>
#include <vector>
#include <iostream>

#include <stk_mesh/base/Field.hpp>
#include <stk_mesh/base/CoordinateSystems.hpp>

#include "Teuchos_CommandLineProcessor.hpp"
#include "PruneMeshAPISTK.hpp"

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
#include "MeshManager.hpp"

namespace stk
{

namespace io { class StkMeshIoBroker; }

}

namespace prune
{

class PerceptPrune
{
public:
    PerceptPrune();
    ~PerceptPrune();
    bool import(int argc,
                char **argv,
                std::string fieldName,
                std::string outputFieldsString,
                double minEdgeLength,
                double isoValue,
                int concatenateResults,
                int isoOnly,
                int readSpreadFile,
                int allowNonmanifoldConnections,
                int numberOfBufferLayers);
    bool run_percept_mesh_stand_alone(MeshManager & aMeshManager);

private:
    bool run_percept_mesh_private_stand_alone(MeshManager & aMeshManager);

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
    int mRemoveIslands;
    int mConcatenateResults;
    int mAllowNonmanifoldConnections;
    int mIsoOnly;
};

} //namespace prune

#endif /* ISO_PRUNE_MAIN_PERCEPTPRUNE_HPP_ */
