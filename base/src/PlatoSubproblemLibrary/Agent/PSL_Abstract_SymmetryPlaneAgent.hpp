// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_ParameterDataEnums.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class PointCloud;
}
class PointCloud;
class ParameterData;
class AbstractAuthority;

class Abstract_SymmetryPlaneAgent
{
public:
    Abstract_SymmetryPlaneAgent(symmetry_plane_agent_t::symmetry_plane_agent_t type,
                                AbstractAuthority* authority);
    virtual ~Abstract_SymmetryPlaneAgent();

    // mirror points by symmetry plane, store mapping internally
    virtual PointCloud* build_kernel_points(double support_distance,
                                            double point_resolution_tolerance,
                                            ParameterData* input_data,
                                            AbstractInterface::PointCloud* all_points,
                                            const std::vector<size_t>& indexes_of_local_points) = 0;
    // expand field data to mirrored points
    virtual std::vector<double> expand_with_symmetry_points(const std::vector<double>& contracted_data_vector) = 0;
    // contract field data by mirrored points
    virtual std::vector<double> contract_by_symmetry_points(const std::vector<double>& expanded_data_vector) = 0;

protected:
    symmetry_plane_agent_t::symmetry_plane_agent_t m_type;
    AbstractAuthority* m_authority;

};

}
