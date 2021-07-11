// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

/* Class: Projection heaviside filter for density method topology optimization.
*
* Smooth a field, and then apply a heaviside projection. This accomplishes the length scale
* of the filter, but retains a somewhat crisp zero-one design.
*/

#include "PSL_Filter.hpp"
#include "PSL_AbstractProjectionFilter.hpp"
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

class ProjectionHeavisideFilter : public AbstractProjectionFilter
{
public:
    ProjectionHeavisideFilter(AbstractAuthority* authority,
                              ParameterData* data,
                              AbstractInterface::PointCloud* points,
                              AbstractInterface::ParallelExchanger* exchanger)
                            : AbstractProjectionFilter(authority, data, points, exchanger)
{
}

    virtual ~ProjectionHeavisideFilter(){}

private:

    double projection_apply(const double& beta, const double& input);
    double projection_gradient(const double& beta, const double& input);

};

}
