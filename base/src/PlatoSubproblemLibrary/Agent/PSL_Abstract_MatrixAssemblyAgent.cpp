// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Abstract_MatrixAssemblyAgent.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_AbstractAuthority.hpp"

namespace PlatoSubproblemLibrary
{

Abstract_MatrixAssemblyAgent::Abstract_MatrixAssemblyAgent(matrix_assembly_agent_t::matrix_assembly_agent_t type,
                                                           AbstractAuthority* authority) :
        m_type(type),
        m_authority(authority)
{

}

Abstract_MatrixAssemblyAgent::~Abstract_MatrixAssemblyAgent()
{

}

}
