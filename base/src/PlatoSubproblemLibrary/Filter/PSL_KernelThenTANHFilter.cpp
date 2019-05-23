// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_KernelThenTANHFilter.hpp"
#include "PSL_FreeHelpers.hpp"


namespace PlatoSubproblemLibrary
{

double KernelThenTANHFilter::projection_apply(const double& beta, const double& input)
{
  tanh_apply(beta,input);  
}
double KernelThenTANHFilter::projection_gradient(const double& beta, const double& input)
{
  tanh_gradient(beta,input); 
}


}
