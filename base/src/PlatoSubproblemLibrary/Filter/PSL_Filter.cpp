// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Filter.hpp"

#include "PSL_Abstract_ParallelVector.hpp"

#include <iostream>

namespace PlatoSubproblemLibrary
{

Filter::Filter()
{
}

Filter::~Filter()
{
}

void Filter::advance_continuation()
{
    // intentionally do nothing
}

void Filter::additive_advance_continuation()
{
    // intentionally do nothing
}

}
