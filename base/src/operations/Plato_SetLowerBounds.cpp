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
 * Plato_SetLowerBounds.cpp
 *
 *  Created on: Jun 28, 2019
 */

#include "PlatoApp.hpp"
#include "Plato_Macros.hpp"
#include "Plato_Parser.hpp"
#include "Plato_InputData.hpp"
#include "Plato_Exceptions.hpp"
#include "Plato_SharedData.hpp"
#include "Plato_SetLowerBounds.hpp"
#include "Plato_OperationsUtilities.hpp"

namespace Plato
{

SetLowerBounds::SetLowerBounds(PlatoApp* p, Plato::InputData& aNode) :
    Plato::LocalOp(p)
{
    this->parseOperationArguments(aNode);
    this->parseFixedBlocks(aNode);
    this->parseEntitySets(aNode);
    this->parseMemberData(aNode);
}

void SetLowerBounds::getArguments(std::vector<Plato::LocalArg> & aLocalArgs)
{
    aLocalArgs.push_back(Plato::LocalArg(Plato::data::layout_t::SCALAR, mInputArgumentName));
    aLocalArgs.push_back(Plato::LocalArg(mOutputLayout, mOutputArgumentName, mOutputSize));
}

void SetLowerBounds::operator()()
{
    // Get the output field
    double* tToData;
    if(mOutputLayout == Plato::data::layout_t::SCALAR_FIELD)
    {
        auto& tOutputField = *(mPlatoApp->getNodeField(mOutputArgumentName));
        tOutputField.ExtractView(&tToData);
        mLowerBoundVectorLength = tOutputField.MyLength();
    }
    else if(mOutputLayout == Plato::data::layout_t::ELEMENT_FIELD)
    {
        throw Plato::ParsingException("ELEMENT_FIELD not implemented for SetLowerBounds operation");
    }
    else if(mOutputLayout == Plato::data::layout_t::SCALAR)
    {
        auto tOutputScalar = mPlatoApp->getValue(mOutputArgumentName);
        mLowerBoundVectorLength = mOutputSize;
        tOutputScalar->resize(mLowerBoundVectorLength);
        tToData = tOutputScalar->data();
    }

    this->initializeLowerBoundVector(tToData);
    if( !mFixedBlockMetadata.mBlockIDs.empty() )
    {
        this->updateLowerBoundsBasedOnFixedEntitiesForDBTOP(tToData); 
    }

}

void SetLowerBounds::updateLowerBoundsBasedOnFixedEntitiesForDBTOP(double* aToData)
{
    if(mDiscretization == "density" && mOutputLayout == Plato::data::layout_t::SCALAR_FIELD)
    {
        if( mMaterialUseCase == "solid" )
        {
            this->updateLowerBoundsForDensityProblems(mFixedBlockMetadata, aToData);
        }
        else
        {
            auto tFluidFixedBlocksMetadata = Plato::FixedBlock::get_fixed_fluid_blocks_metadata(mFixedBlockMetadata);
	    if( !tFluidFixedBlocksMetadata.mBlockIDs.empty() )
	    {
                this->updateLowerBoundsForDensityProblems(tFluidFixedBlocksMetadata, aToData);
	    }
        }
    }
}

void SetLowerBounds::updateLowerBoundsForDensityProblems
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

        double tEntitySetsBoundaryValue = .5001;
        mPlatoApp->getMeshServices()->updateBoundsForFixedBlocks(aToData, aMetadata, *tDistributedVector);
        mPlatoApp->getMeshServices()->updateBoundsForFixedSidesets(aToData, aMetadata.mSidesetIDs, tEntitySetsBoundaryValue);
        mPlatoApp->getMeshServices()->updateBoundsForFixedNodesets(aToData, aMetadata.mNodesetIDs, tEntitySetsBoundaryValue);

        delete tDistributedVector;
        delete tSysGraph_1D;
}

void SetLowerBounds::initializeLowerBoundVector(double* aToData)
{
    // Get incoming global lower bound specified by user
    std::vector<double>* tInData = mPlatoApp->getValue(mInputArgumentName);
    double tLowerBoundIn = (*tInData)[0];
    mFixedBlockMetadata.mOptimizationBlockValue = tLowerBoundIn;

    // Set specified value for the user
    for(int tIndex = 0; tIndex < mLowerBoundVectorLength; tIndex++)
    {
        aToData[tIndex] = tLowerBoundIn;
    }   
}

void SetLowerBounds::parseEntitySets(Plato::InputData& aNode)
{
    auto tFixedSidesetsNode = Plato::Get::InputData(aNode, "FixedSidesets");
    mFixedBlockMetadata.mSidesetIDs = Plato::Get::Ints(tFixedSidesetsNode, "Index");

    auto tFixedNodesetsNode = Plato::Get::InputData(aNode, "FixedNodesets");
    mFixedBlockMetadata.mNodesetIDs = Plato::Get::Ints(tFixedNodesetsNode, "Index");
}

void SetLowerBounds::setBlockIndex(Plato::InputData& aNode)
{
    auto tIndex = Plato::Get::Int(aNode, "Index");
    mFixedBlockMetadata.mBlockIDs.push_back(tIndex);
}

void SetLowerBounds::setDomainValue(Plato::InputData& aNode)
{
    auto tDomainValueString = Plato::Get::String(aNode, "DomainValue");
    if( tDomainValueString.empty() )
    {
        mFixedBlockMetadata.mDomainValues.push_back(1.0); // default: assumes all fixed block are in solid material state
    }
    else
    {
        auto tDomainValue = Plato::Get::Double(aNode, "DomainValue");
        mFixedBlockMetadata.mDomainValues.push_back(tDomainValue);
    }
}

void SetLowerBounds::setBoundaryValue(Plato::InputData &aNode)
{
    auto tBoundaryValueString = Plato::Get::String(aNode, "BoundaryValue");
    if (tBoundaryValueString.empty())
    {
        mFixedBlockMetadata.mBoundaryValues.push_back(0.5001); // default: assumes all fixed block are in solid material state
    }
    else
    {
        auto tBoundaryValue = Plato::Get::Double(aNode, "BoundaryValue");
        mFixedBlockMetadata.mBoundaryValues.push_back(tBoundaryValue);
    }
}

void SetLowerBounds::setMaterialState(Plato::InputData &aNode)
{
    auto tMaterialState = Plato::Get::String(aNode, "MaterialState");
    tMaterialState = tMaterialState.empty() ? "solid" : tMaterialState; // default: assumes all fixed block are in solid material state
    mFixedBlockMetadata.mMaterialStates.push_back(tMaterialState);
}

void SetLowerBounds::parseFixedBlocks(Plato::InputData& aNode)
{
    for(auto& tFixedBlock : aNode.getByName<Plato::InputData>("FixedBlocks"))
    {
        auto tIndexString = Plato::Get::String(tFixedBlock, "Index");
        if (tIndexString.empty())
            { continue; /* break from current iteration, continue with the next iteration in the loop. */ }
        
        this->setBlockIndex(tFixedBlock);
        this->setDomainValue(tFixedBlock);
        this->setBoundaryValue(tFixedBlock);
        this->setMaterialState(tFixedBlock);
    }
}

void SetLowerBounds::parseMemberData
(Plato::InputData& aNode)
{
    if( !mFixedBlockMetadata.mBlockIDs.empty() )
    {
        mMaterialUseCase = Plato::Parse::keyword(aNode, "UseCase", "solid");
    }
    mDiscretization = Plato::Parse::keyword(aNode, "Discretization", "density");
}

void SetLowerBounds::parseOperationArguments(Plato::InputData& aNode)
{
    mInputArgumentName = "Lower Bound Value";
    auto tInputData = Plato::Get::InputData(aNode, "Input");
    mInputArgumentName = Plato::Get::String(tInputData, "ArgumentName");

    mOutputArgumentName = "Lower Bound Vector";
    auto tOutputNode = Plato::Get::InputData(aNode, "Output");
    mOutputLayout = Plato::getLayout(tOutputNode, Plato::data::layout_t::SCALAR_FIELD);
    mOutputSize = Plato::Get::Int(tOutputNode, "Size");
}

}
// namespace Plato
