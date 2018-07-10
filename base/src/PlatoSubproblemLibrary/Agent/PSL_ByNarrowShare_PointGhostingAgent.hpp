// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_PointGhostingAgent.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class OverlapSearcher;
}
class AbstractAuthority;
class ParameterData;
class PointCloud;
class AxisAlignedBoundingBox;

class ByNarrowShare_PointGhostingAgent : public Abstract_PointGhostingAgent
{
public:
    ByNarrowShare_PointGhostingAgent(AbstractAuthority* authority);
    virtual ~ByNarrowShare_PointGhostingAgent();

    virtual void share(double support_distance,
                       PointCloud* local_kernel_points,
                       std::vector<PointCloud*>& nonlocal_kernel_points,
                       std::vector<size_t>& processor_neighbors_below,
                       std::vector<size_t>& processor_neighbors_above);

protected:
    void determine_processor_bounds_and_neighbors(PointCloud* local_kernel_points,
                                                  std::vector<AxisAlignedBoundingBox>& processor_bounds,
                                                  std::vector<size_t>& processor_neighbors_below,
                                                  std::vector<size_t>& processor_neighbors_above);
    void share_points_with_below_processor(const std::vector<size_t>& processor_neighbors_below,
                                           std::vector<AxisAlignedBoundingBox>& processor_bounds,
                                           PointCloud* local_kernel_points,
                                           std::vector<PointCloud*>& nonlocal_kernel_points);
    void share_points_with_above_processor(const std::vector<size_t>& processor_neighbors_above,
                                           std::vector<AxisAlignedBoundingBox>& processor_bounds,
                                           PointCloud* local_kernel_points,
                                           std::vector<PointCloud*>& nonlocal_kernel_points);
    void share_points_with_processor_send(size_t other_proc_id,
                                          std::vector<AxisAlignedBoundingBox>& processor_bounds,
                                          PointCloud* local_kernel_points);
    void share_points_with_processor_receive(size_t other_proc_id, std::vector<PointCloud*>& nonlocal_kernel_points);

    AbstractInterface::OverlapSearcher* m_overlap_searcher;
    double m_support_distance;

};

}
