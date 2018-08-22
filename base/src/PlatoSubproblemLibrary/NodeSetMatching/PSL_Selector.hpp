// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class PointCloud;
}
class NodeSet;

class Selector
{
public:
    Selector();
    ~Selector();

    void set_selection_tolerance(double tol);
    void set_selection_criteria_x(double value);
    void set_selection_criteria_y(double value);
    void set_selection_criteria_z(double value);

    void select(AbstractInterface::PointCloud* nodes, NodeSet& result);

private:
    double m_tolerance;
    short m_dimension;
    double m_value;
};

}
