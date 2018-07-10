// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_MeshScaleAgent.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class PointCloud;
}
class AbstractAuthority;

class ByOptimizedElementSide_MeshScaleAgent : public Abstract_MeshScaleAgent
{
public:
    ByOptimizedElementSide_MeshScaleAgent(AbstractAuthority* authority,
                                          AbstractInterface::PointCloud* points);
    virtual ~ByOptimizedElementSide_MeshScaleAgent();

    virtual double get_mesh_minimum_scale();
    virtual double get_mesh_average_scale();
    virtual double get_mesh_maximum_scale();

protected:
    void calculate_mesh_scales();
    std::vector<bool> get_is_optimizable();

    bool m_calculated_mesh_scales;
    double m_mesh_minimum_scale;
    double m_mesh_average_scale;
    double m_mesh_maximum_scale;

};

}
