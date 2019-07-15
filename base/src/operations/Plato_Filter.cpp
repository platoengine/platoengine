/*
 * Plato_Filter.cpp
 *
 *  Created on: Jun 28, 2019
 */

#include "PlatoApp.hpp"
#include "Plato_Parser.hpp"
#include "Plato_Filter.hpp"
#include "Plato_InputData.hpp"
#include "PlatoEngine_AbstractFilter.hpp"

namespace Plato
{

Filter::Filter(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        Plato::LocalOp(aPlatoApp),
        mFilter(),
        mInputToFilterName(),
        mInputBaseFieldName(),
        mOutputFromFilterName(),
        mIsGradient()
{
    // retrieve filter
    mFilter = mPlatoApp->getFilter();

    // decide names differently
    mIsGradient = Plato::Get::Bool(aNode, "Gradient");
    if(mIsGradient == true)
    {
        mInputToFilterName = "Gradient";
        mInputBaseFieldName = "Field";
        mOutputFromFilterName = "Filtered Gradient";
    }
    else
    {
        mInputToFilterName = "Field";
        mInputBaseFieldName = "";
        mOutputFromFilterName = "Filtered Field";
    }
}

Filter::~Filter()
{
    mFilter = nullptr;
}

void Filter::operator()()
{
    if(mPlatoApp->getTimersTree())
    {
        mPlatoApp->getTimersTree()->begin_partition(Plato::timer_partition_t::timer_partition_t::filter);
    }

    // get input data
    auto tInfield = mPlatoApp->getNodeField(mInputToFilterName);
    Real* tInputField;
    tInfield->ExtractView(&tInputField);
    auto tOutfield = mPlatoApp->getNodeField(mOutputFromFilterName);
    Real* tOutputField;
    tOutfield->ExtractView(&tOutputField);

    // copy input field to output
    const int tLength = tInfield->MyLength();
    std::copy(tInputField, tInputField + tLength, tOutputField);

    if(mIsGradient == true)
    {
        // get base field for gradient application
        auto tBasefield = mPlatoApp->getNodeField(mInputBaseFieldName);
        Real* tBaseField;
        tBasefield->ExtractView(&tBaseField);

        mFilter->apply_on_gradient(tLength, tBaseField, tOutputField);
    }
    else
    {
        mFilter->apply_on_field(tLength, tOutputField);
    }

    if(mPlatoApp->getTimersTree())
    {
        mPlatoApp->getTimersTree()->end_partition();
    }
}

void Filter::getArguments(std::vector<Plato::LocalArg>& aLocalArgs)
{
    aLocalArgs.push_back(Plato::LocalArg
        { Plato::data::layout_t::SCALAR_FIELD, mInputToFilterName });
    if(!mInputBaseFieldName.empty())
    {
        aLocalArgs.push_back(Plato::LocalArg
            { Plato::data::layout_t::SCALAR_FIELD, mInputBaseFieldName });
    }
    aLocalArgs.push_back(Plato::LocalArg
        { Plato::data::layout_t::SCALAR_FIELD, mOutputFromFilterName });
}

}
// namespace Plato
