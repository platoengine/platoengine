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
