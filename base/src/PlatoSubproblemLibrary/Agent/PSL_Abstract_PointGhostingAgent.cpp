// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#include "PSL_Abstract_PointGhostingAgent.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_AbstractAuthority.hpp"

namespace PlatoSubproblemLibrary
{

Abstract_PointGhostingAgent::Abstract_PointGhostingAgent(point_ghosting_agent_t::point_ghosting_agent_t type,
                                                         AbstractAuthority* authority) :
        m_type(type),
        m_authority(authority)
{

}

Abstract_PointGhostingAgent::~Abstract_PointGhostingAgent()
{
    m_authority = NULL;
}

}
