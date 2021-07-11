// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_ProjectionHeavisideFilter.hpp"
#include "PSL_FreeHelpers.hpp"


namespace PlatoSubproblemLibrary
{

double ProjectionHeavisideFilter::projection_apply(const double& beta, const double& input)
{
  return heaviside_apply(beta,input);
}
double ProjectionHeavisideFilter::projection_gradient(const double& beta, const double& input)
{
  return heaviside_gradient(beta,input);
}

}
