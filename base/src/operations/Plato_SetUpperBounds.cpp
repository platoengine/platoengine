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
 * Plato_SetUpperBounds.cpp
 *
 *  Created on: Jun 28, 2019
 */

#include "PlatoApp.hpp"
#include "Plato_Parser.hpp"
#include "Plato_InputData.hpp"
#include "Plato_Exceptions.hpp"
#include "Plato_SharedData.hpp"
#include "Plato_SetUpperBounds.hpp"
#include "Plato_OperationsUtilities.hpp"

namespace Plato
{

SetUpperBounds::SetUpperBounds(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        Plato::LocalOp(aPlatoApp)
{
    this->parseOperationArguments(aNode);
    this->parseFixedBlocks(aNode);
    this->parseEntitySets(aNode);
    this->parseMemberData(aNode);
}

void SetUpperBounds::operator()()
{
    // Get the output field
    double* tToData;
    if(mOutputLayout == Plato::data::layout_t::SCALAR_FIELD)
    {
        auto& tOutputField = *(mPlatoApp->getNodeField(mOutputArgumentName));
        tOutputField.ExtractView(&tToData);
        mUpperBoundVectorLength = tOutputField.MyLength();
    }
    else if(mOutputLayout == Plato::data::layout_t::ELEMENT_FIELD)
    {
        throw Plato::ParsingException("ELEMENT_FIELD not implemented for SetUpperBounds operation");
    }
    else if(mOutputLayout == Plato::data::layout_t::SCALAR)
    {
        auto tOutputScalar = mPlatoApp->getValue(mOutputArgumentName);
        mUpperBoundVectorLength = mOutputSize;
        tOutputScalar->resize(mUpperBoundVectorLength);
        tToData = tOutputScalar->data();
    }

    this->initializeUpperBoundVector(tToData);
    if( !mFixedBlockMetadata.mBlockIDs.empty() )
    {
        this->updateUpperBoundsBasedOnFixedEntitiesForDBTOP(tToData);
        this->updateUpperBoundsBasedOnFixedEntitiesForLSTOP(tToData);
    }
}

void SetUpperBounds::updateUpperBoundsBasedOnFixedEntitiesForDBTOP(double* aToData)
{
    auto tIsFluidMaterialUseCase = mMaterialUseCase == "fluid";
    auto tIsDensityBasedTopologyOptimizationProblem = mDiscretization == "density" && mOutputLayout == Plato::data::layout_t::SCALAR_FIELD;
    if (tIsDensityBasedTopologyOptimizationProblem && tIsFluidMaterialUseCase)
    {
        auto tSolidFixedBlocksMetadata = Plato::FixedBlock::get_fixed_solid_blocks_metadata(mFixedBlockMetadata);
        if( !tSolidFixedBlocksMetadata.mBlockIDs.empty() )
        {
            this->updateUpperBoundsForDensityProblems(tSolidFixedBlocksMetadata, aToData);
        }
    }
}

void SetUpperBounds::updateUpperBoundsBasedOnFixedEntitiesForLSTOP(double* aToData)
{
    auto tIsLevelsetTopologyOptimizationProblem = mDiscretization == "levelset" && mOutputLayout == Plato::data::layout_t::SCALAR_FIELD;
    if (tIsLevelsetTopologyOptimizationProblem)
    {
        double tValue = -0.001;
        mPlatoApp->getMeshServices()->updateBoundsForFixedBlocks(aToData, mFixedBlockMetadata.mBlockIDs, tValue);
        mPlatoApp->getMeshServices()->updateBoundsForFixedSidesets(aToData, mFixedBlockMetadata.mSidesetIDs, tValue);
        mPlatoApp->getMeshServices()->updateBoundsForFixedNodesets(aToData, mFixedBlockMetadata.mNodesetIDs, tValue);
    }
}

void SetUpperBounds::initializeUpperBoundVector(double* aToData)
{
    // Get incoming global Upper bound specified by user
    std::vector<double>* tInData = mPlatoApp->getValue(mInputArgumentName);
    double tUpperBoundIn = (*tInData)[0];
    mFixedBlockMetadata.mOptimizationBlockValue = tUpperBoundIn;

    // Set specified value for the user
    for(int tIndex = 0; tIndex < mUpperBoundVectorLength; tIndex++)
    {
        aToData[tIndex] = tUpperBoundIn;
    }
}

void SetUpperBounds::updateUpperBoundsForDensityProblems
(const Plato::FixedBlock::Metadata& aMetadata, double* aToData)
{
        LightMP* tLightMP = mPlatoApp->getLightMP();
        const int tDofsPerNode_1D = 1;
        SystemContainer* tSysGraph_1D = new SystemContainer(tLightMP->getMesh(), tDofsPerNode_1D, tLightMP->getInput());
        std::vector<VarIndex> tSingleValue(1u);
        DataContainer* tDataContainer = tLightMP->getDataContainer();
        bool tPlottable = true;
        tSingleValue[0] = tDataContainer->registerVariable(RealType, "lowerBoundWorking", NODE, !tPlottable);
        DistributedVector* tDistributedVector = new DistributedVector(tSysGraph_1D, tSingleValue);

        double tEntitySetsBoundaryValue = 0.4999;
        mPlatoApp->getMeshServices()->updateBoundsForFixedBlocks(aToData, aMetadata, *tDistributedVector);
        mPlatoApp->getMeshServices()->updateBoundsForFixedSidesets(aToData, aMetadata.mSidesetIDs, tEntitySetsBoundaryValue);
        mPlatoApp->getMeshServices()->updateBoundsForFixedNodesets(aToData, aMetadata.mNodesetIDs, tEntitySetsBoundaryValue);

        delete tDistributedVector;
        delete tSysGraph_1D;
}

void SetUpperBounds::getArguments(std::vector<Plato::LocalArg> & aLocalArgs)
{
    aLocalArgs.push_back(Plato::LocalArg(Plato::data::layout_t::SCALAR, mInputArgumentName));
    aLocalArgs.push_back(Plato::LocalArg(mOutputLayout, mOutputArgumentName, mOutputSize));
}

void SetUpperBounds::parseEntitySets(Plato::InputData& aNode)
{
    auto tFixedSidesetsNode = Plato::Get::InputData(aNode, "FixedSidesets");
    mFixedBlockMetadata.mSidesetIDs = Plato::Get::Ints(tFixedSidesetsNode, "Index");

    auto tFixedNodesetsNode = Plato::Get::InputData(aNode, "FixedNodesets");
    mFixedBlockMetadata.mNodesetIDs = Plato::Get::Ints(tFixedNodesetsNode, "Index");
}

void SetUpperBounds::parseFixedBlocks(Plato::InputData& aNode)
{
    for(auto tFixedBlock : aNode.getByName<Plato::InputData>("FixedBlocks"))
    {
        auto tIndexString = Plato::Get::String(tFixedBlock, "Index");
        if (tIndexString.empty())
            { continue; /* break from current iteration, continue with the next iteration in the loop. */ }

        auto tIndex = Plato::Get::Int(tFixedBlock, "Index");
        mFixedBlockMetadata.mBlockIDs.push_back(tIndex);
        auto tDomainValue = Plato::Get::Double(tFixedBlock, "DomainValue");
        mFixedBlockMetadata.mDomainValues.push_back(tDomainValue);
        auto tBoundaryValue = Plato::Get::Double(tFixedBlock, "BoundaryValue");
        mFixedBlockMetadata.mBoundaryValues.push_back(tBoundaryValue);
        auto tMaterialState = Plato::Get::String(tFixedBlock, "MaterialState");
        mFixedBlockMetadata.mMaterialStates.push_back(tMaterialState);
    }
}

void SetUpperBounds::parseMemberData
(Plato::InputData& aNode)
{
    if( !mFixedBlockMetadata.mBlockIDs.empty() )
    {
        mMaterialUseCase = Plato::Parse::keyword(aNode, "UseCase", "solid");
    }
    mDiscretization = Plato::Parse::keyword(aNode, "Discretization", "density");
}

void SetUpperBounds::parseOperationArguments(Plato::InputData& aNode)
{
    mInputArgumentName = "Upper Bound Value";
    auto tInputData = Plato::Get::InputData(aNode, "Input");
    mInputArgumentName = Plato::Get::String(tInputData, "ArgumentName");

    mOutputArgumentName = "Upper Bound Vector";
    auto tOutputNode = Plato::Get::InputData(aNode, "Output");
    mOutputLayout = Plato::getLayout(tOutputNode, Plato::data::layout_t::SCALAR_FIELD);
    mOutputSize = Plato::Get::Int(tOutputNode, "Size");
}

}
// namespace Plato
