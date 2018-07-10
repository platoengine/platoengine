#include "PSL_Classifier.hpp"

#include "PSL_DataSequence.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Preprocessor.hpp"
#include "PSL_MultistagePreprocessor.hpp"
#include "PSL_ParameterData.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{

Classifier::Classifier(AbstractAuthority* authority) :
        DataFlow(authority),
        m_preprocessor_enum(),
        m_preprocessor(NULL)
{
}
Classifier::~Classifier()
{
    safe_free(m_preprocessor);
}

void Classifier::allocate_preprocessor(ParameterData* parameters)
{
    safe_free(m_preprocessor);
    m_preprocessor = new MultistagePreprocessor(m_authority, parameters->get_preprocessor(), m_data_sequence);
    m_preprocessor->allocate_data();
}
void Classifier::unpack_preprocessor(const std::vector<preprocessor_t::preprocessor_t>& types)
{
    safe_free(m_preprocessor);
    m_preprocessor = new MultistagePreprocessor(m_authority, types, m_data_sequence);
    m_preprocessor->unpack_data();
}

}

