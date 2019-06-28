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
    mInputName = "Upper Bound Value";
    auto tInputData = Plato::Get::InputData(aNode, "Input");
    mInputName = Plato::Get::String(tInputData, "ArgumentName");

    mOutputName = "Upper Bound Vector";
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

void SetUpperBounds::operator()()
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
        throw Plato::ParsingException("ELEMENT_FIELD not implemented for SetUpperBounds operation");
    }
    else if(mOutputLayout == Plato::data::layout_t::SCALAR)
    {
        auto tOutputScalar = mPlatoApp->getValue(mOutputName);
        tDataLength = mOutputSize;
        tOutputScalar->resize(tDataLength);
        tToData = tOutputScalar->data();
    }

    // Get incoming global Upper bound specified by user
    std::vector<double>* tInData = mPlatoApp->getValue(mInputName);
    double tUpperBoundIn = (*tInData)[0];

    // Set specified value for the user
    for(int tIndex = 0; tIndex < tDataLength; tIndex++)
    {
        tToData[tIndex] = tUpperBoundIn;
    }
    // Now update values based on fixed entities
    if(mDiscretization == "levelset" && mOutputLayout == Plato::data::layout_t::SCALAR_FIELD)
    {
        double tValue = -0.001;
        mPlatoApp->getMeshServices()->updateUpperBoundsForFixedBlocks(tToData, mFixedBlocks, tValue);
        mPlatoApp->getMeshServices()->updateUpperBoundsForFixedSidesets(tToData, mFixedSidesets, tValue);
        mPlatoApp->getMeshServices()->updateUpperBoundsForFixedNodesets(tToData, mFixedNodesets, tValue);
    }
}

void SetUpperBounds::getArguments(std::vector<Plato::LocalArg> & aLocalArgs)
{
    aLocalArgs.push_back(Plato::LocalArg(Plato::data::layout_t::SCALAR, mInputName));
    aLocalArgs.push_back(Plato::LocalArg(mOutputLayout, mOutputName, mOutputSize));
}

}
// namespace Plato
