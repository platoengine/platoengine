// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{
namespace example
{

#define PSL_KERNEL_STRUCTPARAMETERDATA(_type, _name) \
    bool has_##_name; _type _name;

struct Kernel_StructParameterData
{
    PSL_KERNEL_STRUCTPARAMETERDATA(double, scale)
    PSL_KERNEL_STRUCTPARAMETERDATA(double, absolute)
    PSL_KERNEL_STRUCTPARAMETERDATA(int, iterations)
    PSL_KERNEL_STRUCTPARAMETERDATA(double, symPlane_X)
    PSL_KERNEL_STRUCTPARAMETERDATA(double, symPlane_Y)
    PSL_KERNEL_STRUCTPARAMETERDATA(double, symPlane_Z)
    PSL_KERNEL_STRUCTPARAMETERDATA(double, penalty)
    PSL_KERNEL_STRUCTPARAMETERDATA(double, node_resolution_tolerance)
    PSL_KERNEL_STRUCTPARAMETERDATA(spatial_searcher_t::spatial_searcher_t, spatial_searcher)
    PSL_KERNEL_STRUCTPARAMETERDATA(normalization_t::normalization_t, normalization)
    PSL_KERNEL_STRUCTPARAMETERDATA(reproduction_level_t::reproduction_level_t, reproduction)
    PSL_KERNEL_STRUCTPARAMETERDATA(matrix_assembly_agent_t::matrix_assembly_agent_t, matrix_assembly_agent)
    PSL_KERNEL_STRUCTPARAMETERDATA(symmetry_plane_agent_t::symmetry_plane_agent_t, symmetry_plane_agent)
    PSL_KERNEL_STRUCTPARAMETERDATA(mesh_scale_agent_t::mesh_scale_agent_t, mesh_scale_agent)
    PSL_KERNEL_STRUCTPARAMETERDATA(point_ghosting_agent_t::point_ghosting_agent_t, point_ghosting_agent)
    PSL_KERNEL_STRUCTPARAMETERDATA(bounded_support_function_t::bounded_support_function_t, bounded_support_function)
};

}
}
