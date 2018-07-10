// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Abstract_MeshScaleAgent.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_AbstractAuthority.hpp"

namespace PlatoSubproblemLibrary
{

Abstract_MeshScaleAgent::Abstract_MeshScaleAgent(mesh_scale_agent_t::mesh_scale_agent_t type,
                                                 AbstractAuthority* authority,
                                                 AbstractInterface::PointCloud* points) :
        m_type(type),
        m_authority(authority),
        m_points(points)
{
}

Abstract_MeshScaleAgent::~Abstract_MeshScaleAgent()
{
}

}
