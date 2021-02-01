// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

/* Class: Kernel then heaviside filter for density method topology optimization.
*
* Smooth a field, and then apply a heaviside projection. This accomplishes the length scale
* of the kernel, but retains a somewhat crisp zero-one design.
*/

#include "PSL_Filter.hpp"
#include "PSL_AbstractKernelThenProjection.hpp"
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

class KernelThenHeavisideFilter : public AbstractKernelThenProjection
{
public:
    KernelThenHeavisideFilter(AbstractAuthority* authority,
                              ParameterData* data,
                              AbstractInterface::PointCloud* points,
                              AbstractInterface::ParallelExchanger* exchanger)
                            : AbstractKernelThenProjection(authority, data, points, exchanger)
{
}

    virtual ~KernelThenHeavisideFilter(){}

private:

    void projection_apply(const double& beta, AbstractInterface::ParallelVector* field) const;
    void projection_gradient(const double& beta, AbstractInterface::ParallelVector* const field, AbstractInterface::ParallelVector* gradient) const;

};

}
