// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

#include <map>
#include <vector>
#include <utility>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class PointCloud;
}
class AbstractAuthority;
class NodeSet;
class Point;
class AxisAlignedBoundingBox;
class PointCloud;

class NodeSetMatcher
{
public:
    NodeSetMatcher(AbstractAuthority* authority);
    ~NodeSetMatcher();

    void set_spatial_tolerance(double spatial_tolerance);
    void infer_spatial_tolerance(AbstractInterface::PointCloud* nodes);
    double get_spatial_tolerance();

    // only expose globals
    void register_pair(NodeSet* source, NodeSet* destination, Point* vector);
    void match(AbstractInterface::PointCloud* nodes, const std::vector<int>& local_to_global);
    void get_final_pairs(int source_NodeSet_index,
                         int destination_NodeSet_index,
                         std::vector<std::pair<int, int> >& source_and_destination_pairs);

private:

    void build_local_interest(AbstractInterface::PointCloud* nodes,
                              std::vector<AxisAlignedBoundingBox>& local_regions_of_interest,
                              std::vector<bool>& local_points_of_interest);
    void build_source_and_destination_pairs(AbstractInterface::PointCloud* nodes,
                                            const std::vector<int>& local_to_global,
                                            PointCloud* global_points_of_interest, const std::map<int, int>& global_to_local);

    AbstractAuthority* m_authority;
    double m_spatial_tolerance;

    // register data
    std::vector<std::vector<int> > m_sources;
    std::vector<std::vector<int> > m_destinations;
    std::vector<Point*> m_vectors;

    // final data
    std::map<std::pair<int, int>,int> m_nodeset_pairs_to_index;
    std::vector<std::vector<std::pair<int,int> > > m_source_and_destination_pairs;

};
}
