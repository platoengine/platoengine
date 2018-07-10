// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Abstract_MatrixNormalizationAgent.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_ParameterData.hpp"

namespace PlatoSubproblemLibrary
{

Abstract_MatrixNormalizationAgent::Abstract_MatrixNormalizationAgent(matrix_normalization_agent_t::matrix_normalization_agent_t type,
                                                                     AbstractAuthority* authority,
                                                                     ParameterData* data) :
        m_type(type),
        m_authority(authority),
        m_input_data(data)
{

}

Abstract_MatrixNormalizationAgent::~Abstract_MatrixNormalizationAgent()
{

}

}
