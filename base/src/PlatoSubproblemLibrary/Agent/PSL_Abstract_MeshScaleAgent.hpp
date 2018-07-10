// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class PointCloud;
}
class AbstractAuthority;

class Abstract_MeshScaleAgent
{
public:
    Abstract_MeshScaleAgent(mesh_scale_agent_t::mesh_scale_agent_t type,
                            AbstractAuthority* authority,
                            AbstractInterface::PointCloud* points);
    virtual ~Abstract_MeshScaleAgent();

    virtual double get_mesh_minimum_scale() = 0;
    virtual double get_mesh_average_scale() = 0;
    virtual double get_mesh_maximum_scale() = 0;

protected:

    mesh_scale_agent_t::mesh_scale_agent_t m_type;
    AbstractAuthority* m_authority;
    AbstractInterface::PointCloud* m_points;
};

}
