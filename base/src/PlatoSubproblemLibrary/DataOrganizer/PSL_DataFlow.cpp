#include "PSL_DataFlow.hpp"

#include "PSL_AbstractAuthority.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_DataSequence.hpp"
#include "PSL_FreeHelpers.hpp"

namespace PlatoSubproblemLibrary
{

DataFlow::DataFlow(AbstractAuthority* authority) :
        m_data_sequence(new DataSequence(authority, data_flow_t::data_flow_t::inert_data_flow)),
        m_authority(authority),
        m_should_free_data_sequence(true)
{
}
DataFlow::~DataFlow()
{
    if(m_should_free_data_sequence)
    {
        safe_free(m_data_sequence);
    }
}

void DataFlow::set_data_sequence(DataSequence* data_sequence, const bool& should_free)
{
    if(m_should_free_data_sequence)
    {
        safe_free(m_data_sequence);
    }
    m_should_free_data_sequence = should_free;
    m_data_sequence = data_sequence;
}
DataSequence* DataFlow::get_data_sequence()
{
    return m_data_sequence;
}

void DataFlow::unpack_data()
{
}
void DataFlow::allocate_data()
{
}

}
