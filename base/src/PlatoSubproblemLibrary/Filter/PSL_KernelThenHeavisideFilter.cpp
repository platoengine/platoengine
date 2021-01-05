// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_KernelThenHeavisideFilter.hpp"
#include "PSL_FreeHelpers.hpp"


namespace PlatoSubproblemLibrary
{

double KernelThenHeavisideFilter::projection_apply(const double& beta, const double& input) const
{
  return heaviside_apply(beta,input);
}
double KernelThenHeavisideFilter::projection_gradient(const double& beta, const double& input) const
{
  return heaviside_gradient(beta,input);
}

}
