// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_Selector.hpp"

#include "PSL_NodeSet.hpp"
#include "PSL_Abstract_PointCloud.hpp"
#include "PSL_Point.hpp"

#include <cstddef>
#include <cmath>

namespace PlatoSubproblemLibrary
{

Selector::Selector() :
        m_tolerance(1e-6),
        m_dimension(-1),
        m_value(0.)
{
}
Selector::~Selector()
{
}

void Selector::set_selection_tolerance(double tol)
{
    m_tolerance = tol;
}
void Selector::set_selection_criteria_x(double value)
{
    m_dimension = 0;
    m_value = value;
}
void Selector::set_selection_criteria_y(double value)
{
    m_dimension = 1;
    m_value = value;
}
void Selector::set_selection_criteria_z(double value)
{
    m_dimension = 2;
    m_value = value;
}

void Selector::select(AbstractInterface::PointCloud* nodes, NodeSet& result)
{
    if(m_dimension != 0 && m_dimension != 1 && m_dimension != 2)
    {
        return;
    }

    // for each node
    size_t num_nodes = nodes->get_num_points();
    for(size_t node_index = 0u; node_index < num_nodes; node_index++)
    {
        Point node = nodes->get_point(node_index);

        // if within tolerance, select
        if(fabs(node(m_dimension) - m_value) <= m_tolerance)
        {
            result.insert(node.get_index());
        }
    }
}

}
