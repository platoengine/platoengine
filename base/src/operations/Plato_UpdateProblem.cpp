/*
 * Plato_UpdateProblem.cpp
 *
 *  Created on: Jun 28, 2019
 */

#include "PlatoApp.hpp"
#include "Plato_InputData.hpp"
#include "Plato_UpdateProblem.hpp"
#include "PlatoEngine_AbstractFilter.hpp"

namespace Plato
{

UpdateProblem::UpdateProblem(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        Plato::LocalOp(aPlatoApp)
{
}

UpdateProblem::~UpdateProblem()
{
}

void UpdateProblem::operator()()
{
    // update filter
    Plato::AbstractFilter* tFilter = mPlatoApp->getFilter();
    tFilter->advance_continuation();

    // update other portions of the problem here (if necessary)
}

void UpdateProblem::getArguments(std::vector<Plato::LocalArg>& aLocalArgs)
{
}

}
// namespace Plato
