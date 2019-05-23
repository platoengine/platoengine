// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

/* Class: Kernel then tanh filter for density method topology optimization.
*
* Smooth a field, and then apply a tanh "projection". This accomplishes the length scale
* of the kernel, but retains a somewhat crisp zero-one design.
*/

#include "PSL_Filter.hpp"
#include "PSL_AbstractKernelThenFilter.hpp"
#include "PSL_ParameterDataEnums.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{

namespace AbstractInterface
{
class PointCloud;
class ParallelVector;
class ParallelExchanger;
}
class ParameterData;
class AbstractAuthority;
class KernelFilter;

class KernelThenTANHFilter : public AbstractKernelThenFilter
{
public:
    KernelThenTANHFilter(AbstractAuthority* authority,
                              ParameterData* data,
                              AbstractInterface::PointCloud* points,
                              AbstractInterface::ParallelExchanger* exchanger)
                            : AbstractKernelThenFilter(authority, data, points, exchanger)
{
}

    virtual ~KernelThenTANHFilter(){}

private:

    double projection_apply(const double& beta, const double& input);
    double projection_gradient(const double& beta, const double& input);

};

}
