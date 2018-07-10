// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_ParameterDataEnums.hpp"
#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class AxisAlignedBoundingBox;
class PointCloud;

class RegionOfInterestGhostingAgent
{
public:
    RegionOfInterestGhostingAgent(AbstractAuthority* authority);
    ~RegionOfInterestGhostingAgent();

    PointCloud* share(double spatial_tolerance,
                      const std::vector<AxisAlignedBoundingBox>& local_regions_of_interest,
                      PointCloud* locally_indexed_local_nodes,
                      PointCloud* globally_indexed_local_nodes,
                      const std::vector<bool>& is_local_point_of_interest);

protected:
    AbstractAuthority* m_authority;

};

}
