// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

/* Class: Abstract class to generalize the application of a "projection" after applying the Kernel Filter*/

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

class AbstractKernelThenProjection : public AbstractKernelThenFilter
{
public:
    AbstractKernelThenProjection(AbstractAuthority* authority,
                                 ParameterData* data,
                                 AbstractInterface::PointCloud* points,
                                 AbstractInterface::ParallelExchanger* exchanger);
    virtual ~AbstractKernelThenProjection();

    // Filter operations
    virtual void build();
    virtual void advance_continuation();
    virtual void additive_advance_continuation();

private:

    bool m_built;
    bool m_announce_radius;
    AbstractAuthority* m_authority;
    ParameterData* m_input_data;
    AbstractInterface::PointCloud* m_original_points;
    AbstractInterface::ParallelExchanger* m_parallel_exchanger;
    KernelFilter* m_kernel;
    double m_current_heaviside_parameter;
    double m_heaviside_parameter_continuation_scale;
    double m_max_heaviside_parameter;

    void check_input_data();

    virtual void internal_apply(AbstractInterface::ParallelVector* field);
    virtual void internal_gradient(AbstractInterface::ParallelVector* const field, AbstractInterface::ParallelVector* gradient) const;

    virtual void projection_apply(const double& beta, AbstractInterface::ParallelVector* field) const = 0;
    virtual void projection_gradient(const double& beta, AbstractInterface::ParallelVector* const field, AbstractInterface::ParallelVector* gradient) const = 0;
};

}
