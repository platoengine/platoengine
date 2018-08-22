// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

/* Class: overhang filter for density method topology optimization. NOT YET IMPLEMENTED.
*
* Intended to loosely enforce an overhang constraint amongst the design variable locations
* of a density field. Main functions are build and apply. Apply either applies on a field,
* or applies on a gradient.
*/

#include "PSL_Filter.hpp"
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
class PointCloud;

class OverhangFilter : public Filter
{
public:
    OverhangFilter(AbstractAuthority* authority,
                   ParameterData* data,
                   AbstractInterface::PointCloud* points,
                   AbstractInterface::ParallelExchanger* exchanger);
    virtual ~OverhangFilter();

    void set_authority(AbstractAuthority* authority);
    void set_input_data(ParameterData* data);
    void set_points(AbstractInterface::PointCloud* points);
    void set_parallel_exchanger(AbstractInterface::ParallelExchanger* exchanger);
    void announce_radius();

    // Filter operations
    virtual void build();
    virtual void apply(AbstractInterface::ParallelVector* field);
    virtual void apply(AbstractInterface::ParallelVector* base_field, AbstractInterface::ParallelVector* gradient);
    virtual void advance_continuation();

private:

    void compute_overhang_bias(PointCloud* kernel_points);
    void internal_apply(const std::vector<double>& input,
                        std::vector<double>& inner_preactivation,
                        std::vector<double>& inner_postactivation,
                        std::vector<double>& outer_preactivation);

    bool m_built;
    bool m_announce_radius;
    AbstractAuthority* m_authority;
    ParameterData* m_input_data;
    AbstractInterface::PointCloud* m_original_points;
    AbstractInterface::ParallelExchanger* m_parallel_exchanger;
    KernelFilter* m_smoothing_kernel;
    KernelFilter* m_overhang_kernel;
    double m_max_overhang_angle;
    double m_overhang_projection_angle_fraction;
    double m_overhang_projection_radius_fraction;
    double m_build_direction_x;
    double m_build_direction_y;
    double m_build_direction_z;
    double m_current_heaviside_parameter;
    double m_heaviside_parameter_continuation_scale;
    double m_max_heaviside_parameter;
    double m_overhang_fractional_threshold;
    std::vector<double> m_overhang_bias;

    void check_input_data();
};

}
