// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_ParameterDataEnums.hpp"
#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class PointCloud;

class Abstract_PointGhostingAgent
{
public:
    Abstract_PointGhostingAgent(point_ghosting_agent_t::point_ghosting_agent_t type,
                                AbstractAuthority* authority);
    virtual ~Abstract_PointGhostingAgent();

    virtual void share(double support_distance,
                       PointCloud* local_kernel_points,
                       std::vector<PointCloud*>& nonlocal_kernel_points,
                       std::vector<size_t>& processor_neighbors_below,
                       std::vector<size_t>& processor_neighbors_above) = 0;

protected:
    point_ghosting_agent_t::point_ghosting_agent_t m_type;
    AbstractAuthority* m_authority;

};

}
