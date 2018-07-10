// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#include "PSL_Abstract_PointCloud.hpp"

#include "PSL_PointCloud.hpp"
#include "PSL_Point.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{

namespace AbstractInterface
{

PointCloud::PointCloud()
{
}

PointCloud::~PointCloud()
{
}

PlatoSubproblemLibrary::PointCloud* PointCloud::build_point_cloud(std::vector<bool>& selected_nodes)
{
    const size_t num_nodes = selected_nodes.size();

    // count selected
    size_t num_selected = 0u;
    for(size_t node = 0u; node < num_nodes; node++)
    {
        num_selected += selected_nodes[node];
    }

    // allocate
    std::vector<PlatoSubproblemLibrary::Point> points(num_selected);

    // fill
    size_t selected_counter = 0u;
    for(size_t node = 0u; node < num_nodes; node++)
    {
        if(selected_nodes[node])
        {
            points[selected_counter++] = this->get_point(node);
        }
    }

    // build
    PlatoSubproblemLibrary::PointCloud* result = new PlatoSubproblemLibrary::PointCloud;
    result->assign(points);

    return result;
}

}

}
