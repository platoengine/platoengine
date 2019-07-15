/*
 * Plato_InitializeValues.cpp
 *
 *  Created on: Jun 30, 2019
 */

#include "PlatoApp.hpp"
#include "Plato_Parser.hpp"
#include "Plato_InputData.hpp"
#include "Plato_InitializeValues.hpp"

namespace Plato
{

InitializeValues::InitializeValues(PlatoApp* aPlatoAppp, Plato::InputData& aNode) :
        Plato::LocalOp(aPlatoAppp),
        mValuesName("Values")
{
    mValue = Plato::Get::Double(aNode, "InitialValue");
}

void InitializeValues::operator()()
{
    // Output: Values

    std::vector<double>* tData = mPlatoApp->getValue(mValuesName);
    (*tData)[0] = mValue;
}

void InitializeValues::getArguments(std::vector<Plato::LocalArg>& aLocalArgs)
{
    aLocalArgs.push_back(Plato::LocalArg
        { Plato::data::layout_t::SCALAR, mValuesName,/*length=*/1 });
}

}
// namespace Plato

