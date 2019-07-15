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
    mInputName = "Lower Bound Value";
    auto tInputData = Plato::Get::InputData(aNode, "Input");
    mInputName = Plato::Get::String(tInputData, "ArgumentName");

    mOutputName = "Lower Bound Vector";
    auto tOutputNode = Plato::Get::InputData(aNode, "Output");
    mOutputLayout = Plato::getLayout(tOutputNode, Plato::data::layout_t::SCALAR_FIELD);
    mOutputSize = Plato::Get::Int(tOutputNode, "Size");

    auto tFixedBlocksNode = Plato::Get::InputData(aNode, "FixedBlocks");
    mFixedBlocks = Plato::Get::Ints(tFixedBlocksNode, "Index");

    auto tFixedSidesetsNode = Plato::Get::InputData(aNode, "FixedSidesets");
    mFixedSidesets = Plato::Get::Ints(tFixedSidesetsNode, "Index");

    auto tFixedNodesetsNode = Plato::Get::InputData(aNode, "FixedNodesets");
    mFixedNodesets = Plato::Get::Ints(tFixedNodesetsNode, "Index");
    mDiscretization = Plato::Get::String(aNode, "Discretization");
}

void SetLowerBounds::getArguments(std::vector<Plato::LocalArg> & aLocalArgs)
{
    aLocalArgs.push_back(Plato::LocalArg(Plato::data::layout_t::SCALAR, mInputName));
    aLocalArgs.push_back(Plato::LocalArg(mOutputLayout, mOutputName, mOutputSize));
}

void SetLowerBounds::operator()()
{
    // Get the output field
    double* tToData;
    int tDataLength = 0;

    if(mOutputLayout == Plato::data::layout_t::SCALAR_FIELD)
    {
        auto& tOutputField = *(mPlatoApp->getNodeField(mOutputName));
        tOutputField.ExtractView(&tToData);
        tDataLength = tOutputField.MyLength();
    }
    else if(mOutputLayout == Plato::data::layout_t::ELEMENT_FIELD)
    {
        throw Plato::ParsingException("ELEMENT_FIELD not implemented for SetLowerBounds operation");
    }
    else if(mOutputLayout == Plato::data::layout_t::SCALAR)
    {
        auto tOutputScalar = mPlatoApp->getValue(mOutputName);
        tDataLength = mOutputSize;
        tOutputScalar->resize(tDataLength);
        tToData = tOutputScalar->data();
    }

    // Get incoming global lower bound specified by user
    std::vector<double>* tInData = mPlatoApp->getValue(mInputName);
    double tLowerBoundIn = (*tInData)[0];

    // Set specified value for the user
    for(int tIndex = 0; tIndex < tDataLength; tIndex++)
    {
        tToData[tIndex] = tLowerBoundIn;
    }

    // Now update values based on fixed entities
    if(mDiscretization == "density" && mOutputLayout == Plato::data::layout_t::SCALAR_FIELD)
    {
        LightMP* tLightMP = mPlatoApp->getLightMP();
        const int tDofsPerNode_1D = 1;
        SystemContainer* tSysGraph_1D = new SystemContainer(tLightMP->getMesh(), tDofsPerNode_1D, tLightMP->getInput());
        std::vector<VarIndex> tSingleValue(1u);
        DataContainer* tDataContainer = tLightMP->getDataContainer();
        bool tPlottable = true;
        tSingleValue[0] = tDataContainer->registerVariable(RealType, "lowerBoundWorking", NODE, !tPlottable);
        DistributedVector* tDistributedVector = new DistributedVector(tSysGraph_1D, tSingleValue);

        double tBoundaryValue = .5001;
        double tUpperValue = 1.;
        mPlatoApp->getMeshServices()->updateLowerBoundsForFixedBlocks(tToData, mFixedBlocks, tLowerBoundIn, tBoundaryValue, tUpperValue, *tDistributedVector);
        mPlatoApp->getMeshServices()->updateLowerBoundsForFixedSidesets(tToData, mFixedSidesets, tBoundaryValue);
        mPlatoApp->getMeshServices()->updateLowerBoundsForFixedNodesets(tToData, mFixedNodesets, tBoundaryValue);

        delete tDistributedVector;
        delete tSysGraph_1D;
    }
}

}
// namespace Plato
