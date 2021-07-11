// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_ProjectionTANHFilter.hpp"
#include "PSL_FreeHelpers.hpp"


namespace PlatoSubproblemLibrary
{

double ProjectionTANHFilter::projection_apply(const double& beta, const double& input)
{
  return tanh_apply(beta,input);
}
double ProjectionTANHFilter::projection_gradient(const double& beta, const double& input)
{
  return tanh_gradient(beta,input);
}


}
