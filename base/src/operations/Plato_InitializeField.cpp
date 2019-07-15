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

/*
 * Plato_InitializeField.cpp
 *
 *  Created on: Jun 29, 2019
 */

#include <cmath>

#ifdef STK_ENABLED
#include "stk_mesh/base/MetaData.hpp"
#include <stk_mesh/base/Field.hpp>
#include <stk_io/StkMeshIoBroker.hpp>
#include <stk_mesh/base/CoordinateSystems.hpp>
#endif

#include "lightmp.hpp"
#include "matrix_container.hpp"

#include "PlatoApp.hpp"
#include "Plato_Parser.hpp"
#include "Plato_InputData.hpp"
#include "Plato_Exceptions.hpp"
#include "Plato_InitializeField.hpp"
#include "Plato_OperationsUtilities.hpp"

namespace Plato
{

void InitializeField::getArguments(std::vector<Plato::LocalArg> & aLocalArgs)
{
    aLocalArgs.push_back(Plato::LocalArg(mOutputLayout, mOutputFieldName));
}

InitializeField::InitializeField(PlatoApp* aPlatoApp, Plato::InputData & aNode) :
        Plato::LocalOp(aPlatoApp),
        mOutputFieldName("Initialized Field")
{
    mStringMethod = Plato::Get::String(aNode, "Method");

    Plato::InputData tMethodNode = Plato::Get::InputData(aNode, mStringMethod);
    if(mStringMethod == "Uniform")
    {
        mUniformValue = Plato::Get::Double(tMethodNode, "Value");
    }
    else if(mStringMethod == "FromFile")
    {
        mFileName = Plato::Get::String(tMethodNode, "Name");
        mVariableName = Plato::Get::String(tMethodNode, "VariableName");
        mIteration = Plato::Get::Int(tMethodNode, "Iteration");

        // Copy all of the platomain.exo.* files to
        // platomain_restart.exo.* because platomain.exo
        // will get overwritten for the next run. I am doing
        // this in code to try to avoid making system calls.
        int tCommSize = 0;
        MPI_Comm_size(mPlatoApp->getComm(), &tCommSize);

        for(int tRankIndex = 0; tRankIndex < tCommSize; tRankIndex++)
        {
            char tInFilename[200];
            char tOutFilename[200];
            if(tCommSize == 1)
            {
                sprintf(tInFilename, "platomain.exo.1.0");
                sprintf(tOutFilename, "platomain_restart.exo");
            }
            else
            {
                sprintf(tInFilename, "platomain.exo.%d.%d", tCommSize, tRankIndex);
                sprintf(tOutFilename, "platomain_restart.exo.%d.%d", tCommSize, tRankIndex);
            }

            int tBufferSize = 1024;
            char tBuffer[tBufferSize];
            FILE *tInFile, *tOutFile;
            size_t tNumRead;

            tInFile = fopen(tInFilename, "rb");
            tOutFile = fopen(tOutFilename, "wb");
            if(tInFile && tOutFile)
            {
                while((tNumRead = fread(tBuffer, 1, tBufferSize, tInFile)) != 0)
                {
                    fwrite(tBuffer, 1, tNumRead, tOutFile);
                }
                fclose(tInFile);
                fclose(tOutFile);
            }
        }
    }
    else if(mStringMethod == "SwissCheeseLevelSet")
    {
        mFileName = Plato::Get::String(tMethodNode, "Name");
        mCreateSpheres = true;
        mCreateSpheres = Plato::Get::Bool(tMethodNode, "CreateSpheres");
        mSphereSpacingX = Plato::Get::String(tMethodNode, "SphereSpacingX");
        mSphereSpacingY = Plato::Get::String(tMethodNode, "SphereSpacingY");
        mSphereSpacingZ = Plato::Get::String(tMethodNode, "SphereSpacingZ");
        mSphereRadius = Plato::Get::String(tMethodNode, "SphereRadius");
        mSpherePackingFactor = Plato::Get::String(tMethodNode, "SpherePackingFactor");
        mFileName = Plato::Get::String(tMethodNode, "BackgroundMeshName");

        if(mFileName.empty())
        {
            std::cout << "\n\n********************************************************************************\n";
            std::cout << "ERROR: No BackgroundMeshName was specified in the SwissCheeseLevelSet section.\n";
            std::cout << "********************************************************************************\n\n";
        }
        mLevelSetNodesets = Plato::Get::Ints(tMethodNode, "NodeSet");
    }
    else if(mStringMethod == "PrimitivesLevelSet")
    {
        mFileName = Plato::Get::String(tMethodNode, "BackgroundMeshName");

        if(mFileName.empty())
        {
            std::cout << "\n\n********************************************************************************\n";
            std::cout << "ERROR: No BackgroundMeshName was specified in the SwissCheeseLevelSet section.\n";
            std::cout << "********************************************************************************\n\n";
        }
        auto tNodes = tMethodNode.getByName<Plato::InputData>("MaterialBox");
        for(auto tNode = tNodes.begin(); tNode != tNodes.end(); ++ tNode)
        {
            std::string tTempString = Plato::Get::String(*tNode, "MinCoords");
            if(tTempString != "")
            {
                int tPosition = tTempString.find(" ");
                mMinCoords[0] = atof(tTempString.substr(0, tPosition).c_str());
                tTempString = tTempString.substr(tPosition + 1);
                tPosition = tTempString.find(" ");
                mMinCoords[1] = atof(tTempString.substr(0, tPosition).c_str());
                tTempString = tTempString.substr(tPosition + 1);
                tPosition = tTempString.find(" ");
                mMinCoords[2] = atof(tTempString.substr(0, tPosition).c_str());
            }
            tTempString = Plato::Get::String(*tNode, "MaxCoords");
            if(tTempString != "")
            {
                int tPosition = tTempString.find(" ");
                mMaxCoords[0] = atof(tTempString.substr(0, tPosition).c_str());
                tTempString = tTempString.substr(tPosition + 1);
                tPosition = tTempString.find(" ");
                mMaxCoords[1] = atof(tTempString.substr(0, tPosition).c_str());
                tTempString = tTempString.substr(tPosition + 1);
                tPosition = tTempString.find(" ");
                mMaxCoords[2] = atof(tTempString.substr(0, tPosition).c_str());
            }
        }
    }
    else if(mStringMethod == "FromFieldOnInputMesh")
    {
        mVariableName = Plato::Get::String(tMethodNode, "VariableName");
    }

    Plato::InputData tOutputNode = Plato::Get::InputData(aNode, "Output");
    mOutputLayout = Plato::getLayout(tOutputNode, Plato::data::layout_t::SCALAR_FIELD);
}

/******************************************************************************/
void InitializeField::operator()()
/******************************************************************************/
{
    double* tToData;

    if(mOutputLayout == Plato::data::layout_t::SCALAR_FIELD)
    {
        auto& tField = *(mPlatoApp->getNodeField(mOutputFieldName));
        tField.ExtractView(&tToData);
        int tDataLength = tField.MyLength();

        if(mStringMethod == "Uniform")
        {
            for(int tIndex = 0; tIndex < tDataLength; tIndex++)
            {
                tToData[tIndex] = mUniformValue;
            }
        }
        else if(mStringMethod == "FromFile")
        {
            std::vector<double> tValues;
            getInitialValuesForRestart(tField, tValues);
            for(int tIndex = 0; tIndex < tDataLength; tIndex++)
            {
                tToData[tIndex] = tValues[tIndex];
            }
        }
        else if(mStringMethod == "SwissCheeseLevelSet")
        {
            std::vector<double> tValues;
            getInitialValuesForSwissCheeseLevelSet(tField, tValues);
            for(int tIndex = 0; tIndex < tDataLength; tIndex++)
            {
                tToData[tIndex] = tValues[tIndex];
            }
        }
        else if(mStringMethod == "PrimitivesLevelSet")
        {
            std::vector<double> tValues;
            getInitialValuesForPrimitivesLevelSet(tField, tValues);
            for(int tIndex = 0; tIndex < tDataLength; tIndex++)
            {
                tToData[tIndex] = tValues[tIndex];
            }
        }
        else if(mStringMethod == "FromFieldOnInputMesh")
        {
            mPlatoApp->getLightMP()->getMesh()->readNodePlot(tToData, mVariableName);
        }
    }
    else if(mOutputLayout == Plato::data::layout_t::ELEMENT_FIELD)
    {
        auto tFieldIndex = mPlatoApp->getElementField(mOutputFieldName);
        auto tDataContainer = mPlatoApp->getLightMP()->getDataContainer();
        tDataContainer->getVariable(tFieldIndex, tToData);
        int tDataLength = mPlatoApp->getLightMP()->getMesh()->getNumElems();

        if(mStringMethod == "Uniform")
        {
            for(int tIndex = 0; tIndex < tDataLength; tIndex++)
            {
                tToData[tIndex] = mUniformValue;
            }
        }
        else if(mStringMethod == "FromFile")
        {
            throw Plato::ParsingException("InitializeField is not implemented for ELEMENT_FIELD layout.");
        }

    }
}

/******************************************************************************/
double InitializeField::evaluateSwissCheeseLevelSet(const double &aX,
                                                    const double &aY,
                                                    const double &aZ,
                                                    std::vector<double> aLowerCoordBoundsOfDomain,
                                                    std::vector<double> aUpperCoordBoundsOfDomain,
                                                    double aAverageElemLength)
/******************************************************************************/
{
    bool tDoSimpleBoundsOnly = false;
    if(tDoSimpleBoundsOnly)
    {
        double tRelativeDistance = 1e-1 * (aUpperCoordBoundsOfDomain[1] - aLowerCoordBoundsOfDomain[1]);
        double tLevelSetTop = aY - (aUpperCoordBoundsOfDomain[1] - tRelativeDistance);
        double tLevelSetBottom = (aLowerCoordBoundsOfDomain[1] + tRelativeDistance) - aY;
        double tLevelSetValue = std::max(tLevelSetTop, tLevelSetBottom);

        double tRelativeValueX = aUpperCoordBoundsOfDomain[0] - aLowerCoordBoundsOfDomain[0];
        tRelativeValueX *= 1.e-3;
        if((std::abs(aLowerCoordBoundsOfDomain[0] - aX) < tRelativeValueX)
                || (std::abs(aUpperCoordBoundsOfDomain[0] - aX) < tRelativeValueX))
        {
            tLevelSetValue = -4.5;
        }

        return -tLevelSetValue;
    }
    // get characteristic lengths
    const size_t tNUM_DIMENSIONS = 3;
    std::vector<double> tDomainLengths(tNUM_DIMENSIONS, 0.0);
    for(size_t tDim = 0; tDim < tNUM_DIMENSIONS; tDim++)
    {
        tDomainLengths[tDim] = aUpperCoordBoundsOfDomain[tDim] - aLowerCoordBoundsOfDomain[tDim];
    }

    // figure out a size and number of spheres to lay down
    // assume we should have at least 4 or 5 elements per sphere
    double tSphereRadius = aAverageElemLength * 1.5;
    double tPackingFactor = 4.0;
    if(mSphereRadius != "")
    {
        tSphereRadius = atof(mSphereRadius.c_str());
    }
    if(mSpherePackingFactor != "")
    {
        tPackingFactor = atof(mSpherePackingFactor.c_str());
    }

    std::vector<size_t> tNumSpheresInDimension(tNUM_DIMENSIONS, 0);
    std::vector<double> tSphereSpacing(tNUM_DIMENSIONS, 0);

    for(size_t tDim = 0; tDim < tNUM_DIMENSIONS; ++ tDim)
    {
        // try to compute a spacing that leaves one sphere's width between each sphere, but err on the side of fewer spheres in each dimension
        tNumSpheresInDimension[tDim] = std::floor(tDomainLengths[tDim] / (tPackingFactor * tSphereRadius));
        if(tNumSpheresInDimension[tDim] < 1)
        {
            tNumSpheresInDimension[tDim] = 1;
        }

        // now that we have an actual number in each dimension, find the actual spacing between sphere centers
        tSphereSpacing[tDim] = tDomainLengths[tDim] / static_cast<double>(tNumSpheresInDimension[tDim]);
    }

    /* this kind of brakes things right now*/
    if(mSphereSpacingX != "")
    {
        tSphereSpacing[0] = atof(mSphereSpacingX.c_str());
    }
    if(mSphereSpacingY != "")
    {
        tSphereSpacing[1] = atof(mSphereSpacingY.c_str());
    }
    if(mSphereSpacingZ != "")
    {
        tSphereSpacing[2] = atof(mSphereSpacingZ.c_str());
    }

    assert(tNUM_DIMENSIONS == 3); // lazy, wrote 3 nested for loops for 3 dimensions
    double tLevelSetValOut = 2;
    // now for each sphere, compute the levelset, take the min value
    for(size_t tSphereOneIndex = 0; tSphereOneIndex < tNumSpheresInDimension[0]; ++ tSphereOneIndex)
    {
        for(size_t tSphereTwoIndex = 0; tSphereTwoIndex < tNumSpheresInDimension[1]; ++ tSphereTwoIndex)
        {
            for(size_t tSphereThreeIndex = 0; tSphereThreeIndex < tNumSpheresInDimension[2]; ++ tSphereThreeIndex)
            {
                // compute center of this sphere
                double tCenterCoordX = (0.5 + static_cast<double>(tSphereOneIndex)) * tSphereSpacing[0] + aLowerCoordBoundsOfDomain[0];
                double tCenterCoordY = (0.5 + static_cast<double>(tSphereTwoIndex)) * tSphereSpacing[1] + aLowerCoordBoundsOfDomain[1];
                double tCenterCoordZ = (0.5 + static_cast<double>(tSphereThreeIndex)) * tSphereSpacing[2] + aLowerCoordBoundsOfDomain[2];

                // calculate the distance from the sphere center to the input point
                double tRadiusSquared = (aX - tCenterCoordX) * (aX - tCenterCoordX) + (aY - tCenterCoordY) * (aY - tCenterCoordY) + (aZ - tCenterCoordZ) * (aZ - tCenterCoordZ);
                double tRadius = std::sqrt(tRadiusSquared);

                // now the levelset value is just the radius - teh sphere's radius, resulting in negative tValues inside the sphere, positive outside, zero on the boundary
                double tLevelSetValue = tRadius - tSphereRadius;

                // if this is the first sphere just take the resulting levelset value, if we are a subsequent sphere, take this sphere's levelset if it is less than any previous value (
                if((tSphereOneIndex == 0) && (tSphereTwoIndex == 0) && (tSphereThreeIndex == 0))
                {
                    tLevelSetValOut = tLevelSetValue;
                }
                else if(tLevelSetValue < tLevelSetValOut)
                {
                    tLevelSetValOut = tLevelSetValue;
                }
            }
        }
    }

    return tLevelSetValOut;
}

/******************************************************************************/
void InitializeField::getInitialValuesForSwissCheeseLevelSet(const DistributedVector &field, std::vector<double> &aValues)
/******************************************************************************/
{
#ifdef STK_ENABLED
    stk::io::StkMeshIoBroker *tBroker = new stk::io::StkMeshIoBroker(mPlatoApp->getComm());
    stk::mesh::MetaData *tMetaData = new stk::mesh::MetaData;
    stk::mesh::BulkData *tBulkData = new stk::mesh::BulkData(*tMetaData, mPlatoApp->getComm());
    tBroker->set_bulk_data(*tBulkData);

    tBroker->set_option_to_not_collapse_sequenced_fields();
    tBroker->property_add(Ioss::Property("LOWER_CASE_VARIABLE_NAMES", "OFF"));
    tBroker->add_mesh_database(mFileName, "exodus", stk::io::READ_MESH);
    tBroker->create_input_mesh();

    stk::mesh::Field<double> &tTempField = tMetaData->declare_field<stk::mesh::Field<double>>(stk::topology::NODE_RANK, "swiss", 1);

    std::vector<double> tTempFieldVals(2541, 0);
    stk::mesh::put_field_on_mesh(tTempField, tMetaData->universal_part(), tTempFieldVals.data());

    tBroker->populate_bulk_data();

    stk::mesh::Field<double, stk::mesh::Cartesian> *tCoordsField = tMetaData->get_field<stk::mesh::Field<double, stk::mesh::Cartesian> >
    (stk::topology::NODE_RANK, "coordinates");

    std::vector<stk::mesh::Entity> tNodes;
    tBulkData->get_entities(stk::topology::NODE_RANK, tMetaData->universal_part(), tNodes);

    double tMinX, tMaxX;
    double tMinY, tMaxY;
    double tMinZ, tMaxZ;
    double* coords = stk::mesh::field_data(*tCoordsField, tNodes[0]);
    tMinX = tMaxX = coords[0];
    tMinY = tMaxY = coords[1];
    tMinZ = tMaxZ = coords[2];
    for(size_t inode=1; inode<tNodes.size(); inode++)
    {
        coords = stk::mesh::field_data(*tCoordsField, tNodes[inode]);
        if(coords[0] < tMinX)
        tMinX = coords[0];
        else if(coords[0] > tMaxX)
        tMaxX = coords[0];
        if(coords[1] < tMinY)
        tMinY = coords[1];
        else if(coords[1] > tMaxY)
        tMaxY = coords[1];
        if(coords[2] < tMinZ)
        tMinZ = coords[2];
        else if(coords[2] > tMaxZ)
        tMaxZ = coords[2];
    }
    float tSendBuffer[3], tReceiveBuffer[3];
    tSendBuffer[0] = tMinX;
    tSendBuffer[1] = tMinY;
    tSendBuffer[2] = tMinZ;
    MPI_Allreduce(&tSendBuffer, &tReceiveBuffer, 3, MPI_FLOAT, MPI_MIN, mPlatoApp->getComm());
    tMinX = tReceiveBuffer[0];
    tMinY = tReceiveBuffer[1];
    tMinZ = tReceiveBuffer[2];
    tSendBuffer[0] = tMaxX;
    tSendBuffer[1] = tMaxY;
    tSendBuffer[2] = tMaxZ;
    MPI_Allreduce(&tSendBuffer, &tReceiveBuffer, 3, MPI_FLOAT, MPI_MAX, mPlatoApp->getComm());
    tMaxX = tReceiveBuffer[0];
    tMaxY = tReceiveBuffer[1];
    tMaxZ = tReceiveBuffer[2];

    stk::mesh::Selector tMySelector = tMetaData->locally_owned_part();
    const stk::mesh::BucketVector &tElemBuckets = tBulkData->get_buckets(stk::topology::ELEMENT_RANK, tMySelector);

    double tLocalDistanceSquared = 0;
    int tCenter = 0;
    for ( stk::mesh::BucketVector::const_iterator tBucketIteration = tElemBuckets.begin(); tBucketIteration != tElemBuckets.end(); ++tBucketIteration )
    {
        stk::mesh::Bucket &tTempBucket = **tBucketIteration;
        stk::topology tBucketTop = tTempBucket.topology();
        if(tBucketTop == stk::topology::TET_4 ||
                tBucketTop == stk::topology::HEX_8)
        {
            size_t tNumElems = tTempBucket.size();
            for (size_t tIndex=0; tIndex<tNumElems; ++tIndex)
            {
                stk::mesh::Entity cur_elem = tTempBucket[tIndex];
                stk::mesh::Entity const *elem_nodes = tBulkData->begin_nodes(cur_elem);
                double* coords1 = stk::mesh::field_data(*tCoordsField, elem_nodes[0]);
                double* coords2 = stk::mesh::field_data(*tCoordsField, elem_nodes[1]);
                double tDistanceSquared = (coords1[0]-coords2[0])*(coords1[0]-coords2[0]) +
                (coords1[1]-coords2[1])*(coords1[1]-coords2[1]) +
                (coords1[2]-coords2[2])*(coords1[2]-coords2[2]);
                tLocalDistanceSquared += tDistanceSquared;
                tCenter++;
            }
        }
    }
    tLocalDistanceSquared = tLocalDistanceSquared/((double)tCenter);
    int tNumProcs;
    MPI_Comm_size(mPlatoApp->getComm(), &tNumProcs);
    tSendBuffer[0] = tLocalDistanceSquared;
    MPI_Allreduce(&tSendBuffer, &tReceiveBuffer, 1, MPI_FLOAT, MPI_SUM, mPlatoApp->getComm());
    double tGlobalAverageDistance = sqrt(tReceiveBuffer[0]/((double)tNumProcs));

    // Get the ids of all of the tNodes that are in nodesets to be included as level sets.
    std::set<int> tNodeSetGlobalIds;
    const stk::mesh::PartVector tParts = tMetaData->get_mesh_parts();
    for(size_t tIndex=0; tIndex<tParts.size(); ++tIndex)
    {
        stk::mesh::Part *tCurPart = tParts[tIndex];
        stk::mesh::EntityRank tCurRank = tCurPart->primary_entity_rank();
        if(tCurRank == 0)
        {
            int tPartId = tCurPart->id();
            if(std::find(mLevelSetNodesets.begin(), mLevelSetNodesets.end(), tPartId) != mLevelSetNodesets.end())
            {
                tNodes.clear();
                tBulkData->get_entities(stk::topology::NODE_RANK, *tCurPart, tNodes);
                for(size_t b=0; b<tNodes.size(); ++b)
                {
                    tNodeSetGlobalIds.insert(tBulkData->identifier(tNodes[b]));
                }
            }
        }
    }

    std::vector<double> tLowerCoordBoundsOfDomain =
    { tMinX, tMinY, tMinZ};
    std::vector<double> tUpperCoordBoundsOfDomain =
    { tMaxX, tMaxY, tMaxZ};

    int tLength = field.MyLength();
    for(int tIndex=0; tIndex<tLength; ++tIndex)
    {
        int tGlobalID = field.getAssemblyEpetraVector()->Map().GID(tIndex);
        stk::mesh::Entity tEntity = tBulkData->get_entity(stk::topology::NODE_RANK, tGlobalID);
        double* tCoords = stk::mesh::field_data(*tCoordsField, tEntity);
        double tAverageElemLength=tGlobalAverageDistance;
        double tVal;
        if(tNodeSetGlobalIds.find(tGlobalID) != tNodeSetGlobalIds.end())
        {
            tVal = 1.0;
        }
        else
        {
            if(mCreateSpheres)
            {
                tVal = -1.0 * evaluateSwissCheeseLevelSet(tCoords[0], tCoords[1], tCoords[2],
                        tLowerCoordBoundsOfDomain,
                        tUpperCoordBoundsOfDomain,
                        tAverageElemLength);
            }
            else
            tVal = -1.0;
        }
        double* tValues2 = stk::mesh::field_data(tTempField, tEntity);
        tValues2[0] = tVal;
        aValues.push_back(tVal);
    }

    delete tBulkData;
    delete tMetaData;
    delete tBroker;
#else
    throw Plato::LogicException("Functionality not available.  Recompile with STK enabled.");
#endif // STK_ENABLED
}

/******************************************************************************/
void InitializeField::getInitialValuesForPrimitivesLevelSet(const DistributedVector &field, std::vector<double> &tValues)
/******************************************************************************/
{
#ifdef STK_ENABLED
    stk::io::StkMeshIoBroker *tBroker = new stk::io::StkMeshIoBroker(mPlatoApp->getComm());
    stk::mesh::MetaData *tMetaData = new stk::mesh::MetaData;
    stk::mesh::BulkData *tBulkData = new stk::mesh::BulkData(*tMetaData, mPlatoApp->getComm());
    tBroker->set_bulk_data(*tBulkData);

    tBroker->set_option_to_not_collapse_sequenced_fields();
    tBroker->property_add(Ioss::Property("LOWER_CASE_VARIABLE_NAMES", "OFF"));
    tBroker->add_mesh_database(mFileName, "exodus", stk::io::READ_MESH);
    tBroker->create_input_mesh();

    tBroker->populate_bulk_data();

    stk::mesh::Field<double, stk::mesh::Cartesian> *tCoordsField =
            tMetaData->get_field<stk::mesh::Field<double, stk::mesh::Cartesian>>(stk::topology::NODE_RANK, "coordinates");

    // Hard code 4 plane tValues (brick)
    // double tPlanes[6][3] = {{-5.25,.1875,.1875},{-5.25,.1875,.1875},{-5.25,-.1875,-.1875},{-5.25,-.1875,-.1875},{-5.25,.1875,.1875},{-.25,-.1875,-.1875}};
    double tPlanes[6][3] =
    {
        { mMinCoords[0], mMaxCoords[1], mMaxCoords[2] },
        { mMinCoords[0], mMaxCoords[1], mMaxCoords[2] },
        { mMinCoords[0], mMinCoords[1], mMinCoords[2] },
        { mMinCoords[0], mMinCoords[1], mMinCoords[2] },
        { mMinCoords[0], mMaxCoords[1], mMaxCoords[2] },
        { mMaxCoords[0], mMinCoords[1], mMinCoords[2] }};
    double n[6][3] =
    { { 0, -1, 0},
      { 0, 0, -1},
      { 0, 1, 0 },
      { 0, 0, 1 },
      { 1, 0, 0 },
      { -1,0, 0 }};

    int tLength = field.MyLength();
    for(int tIndex=0; tIndex<tLength; ++tIndex)
    {
        int tGlobalID = field.getAssemblyEpetraVector()->Map().GID(tIndex);
        stk::mesh::Entity tEntity = tBulkData->get_entity(stk::topology::NODE_RANK, tGlobalID);
        double* tCurrentCoords = stk::mesh::field_data(*tCoordsField, tEntity);
        double tAllDots[6];
        bool tAreAllDotsPositive = true;
        for(int j=0; j<6; j++)
        {
            double tVector[3];
            for(int k=0; k<3; k++)
            {
                tVector[k] = tCurrentCoords[k]-tPlanes[j][k];
            }
            double tDot = 0.0;
            for(int k=0; k<3; k++)
            {
                tDot += tVector[k]*n[j][k];
            }
            tAllDots[j] = tDot;
            if(tDot < 0.0)
            {
                tAreAllDotsPositive = false;
            }
        }
        if(tAreAllDotsPositive == true) // point is inside brick
        {
            double tSmallestDot = 9999999.;
            // find smallest tDot and use as value
            for(int j=0; j<6; j++)
            {
                if(tAllDots[j] < tSmallestDot)
                {
                    tSmallestDot = tAllDots[j];
                }
            }
            tValues.push_back(-1.0*tSmallestDot);
        }
        else
        {
            // for all the planes this had a negative dot with project
            // the point to that plane.  This will give us the point on
            // the brick closest to the original point and then we can caluculate
            // a distance.
            double tNewCoords[3] = { tCurrentCoords[0], tCurrentCoords[1], tCurrentCoords[2] };
            for(int j=0; j<6; j++)
            {
                if(tAllDots[j] < 0.0)
                {
                    for(int k=0; k<3; ++k)
                    {
                        tNewCoords[k] -= tAllDots[j] * n[j][k];
                    }
                }
            }
            double tDistance = sqrt((tCurrentCoords[0]-tNewCoords[0])*(tCurrentCoords[0]-tNewCoords[0]) +
                    (tCurrentCoords[1]-tNewCoords[1])*(tCurrentCoords[1]-tNewCoords[1]) +
                    (tCurrentCoords[2]-tNewCoords[2])*(tCurrentCoords[2]-tNewCoords[2]));
            tValues.push_back(tDistance);
        }
    }

    delete tBulkData;
    delete tMetaData;
    delete tBroker;
#else
    throw Plato::LogicException("Functionality not available.  Recompile with STK enabled.");
#endif // STK_ENABLED
}

void InitializeField::getInitialValuesForRestart(const DistributedVector &field, std::vector<double> &aValues)
{
#ifdef STK_ENABLED
    bool IsInputFileSpread = true;

    stk::io::StkMeshIoBroker *tBroker = new stk::io::StkMeshIoBroker(mPlatoApp->getComm());
    stk::mesh::MetaData *tMetaData = new stk::mesh::MetaData;
    stk::mesh::BulkData *tBulkData = new stk::mesh::BulkData(*tMetaData, mPlatoApp->getComm());
    tBroker->set_bulk_data(*tBulkData);

    tBroker->set_option_to_not_collapse_sequenced_fields();
    if(!IsInputFileSpread)
    {
        tBroker->property_add(Ioss::Property("DECOMPOSITION_METHOD", "RIB"));
    }
    tBroker->property_add(Ioss::Property("LOWER_CASE_VARIABLE_NAMES", "OFF"));
    tBroker->add_mesh_database(mFileName, "exodus", stk::io::READ_MESH);
    tBroker->create_input_mesh();

    tBroker->add_all_mesh_fields_as_input_fields();

    tBroker->populate_bulk_data();
    stk::mesh::Field<double> *tIsoField;

    tIsoField = tMetaData->get_field<stk::mesh::Field<double> >(stk::topology::NODE_RANK, mVariableName);

    if(mIteration == -1)
    {
        mIteration = tBroker->get_input_io_region().get()->get_property("state_count").get_int();
    }
    tBroker->read_defined_input_fields(mIteration);

    // This code right here makes the assumption that the mesh we are reading the tValues (restart mesh) from has the exact
    // same decomposition as the mesh we are using as input to this plato run (the input mesh).  It is also
    // taking advantage of the fact that when we load the result mesh into a stk_mesh the field has a local
    // ordering that is the same as the local ordering of the field we are loading--which ordering came from
    // the reading in of the input mesh into LightMP.
    int tLength = field.MyLength();
    for(int tIndex=0; tIndex<tLength; ++tIndex)
    {
        int tGlobalID = field.getAssemblyEpetraVector()->Map().GID(tIndex);
        stk::mesh::Entity tEntity = tBulkData->get_entity(stk::topology::NODE_RANK, tGlobalID);
        double* tValues = stk::mesh::field_data(*tIsoField, tEntity);
        aValues.push_back(*tValues);
    }

    delete tBulkData;
    delete tMetaData;
    delete tBroker;

#else
    throw Plato::LogicException("Functionality not available.  Recompile with STK enabled.");
#endif // STK_ENABLED
}

}

