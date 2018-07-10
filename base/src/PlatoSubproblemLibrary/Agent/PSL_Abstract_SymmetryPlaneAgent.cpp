// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Abstract_SymmetryPlaneAgent.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_Abstract_PointCloud.hpp"
#include "PSL_PointCloud.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{

Abstract_SymmetryPlaneAgent::Abstract_SymmetryPlaneAgent(symmetry_plane_agent_t::symmetry_plane_agent_t type,
                                                         AbstractAuthority* authority) :
        m_type(type),
        m_authority(authority)
{

}

Abstract_SymmetryPlaneAgent::~Abstract_SymmetryPlaneAgent()
{

}

}
