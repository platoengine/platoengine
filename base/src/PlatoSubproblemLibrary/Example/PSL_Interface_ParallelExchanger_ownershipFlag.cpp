// PlatoSubproblemLibraryVersion(6): a stand-alone library for the kernel filter for plato.
#include "PSL_Interface_ParallelExchanger_ownershipFlag.hpp"

#include "PSL_Abstract_ParallelExchanger_Managed.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_ParallelExchanger_ownershipFlag::Interface_ParallelExchanger_ownershipFlag(AbstractAuthority* authority) :
        AbstractInterface::ParallelExchanger_Managed(authority),
        m_is_owned()
{
}

Interface_ParallelExchanger_ownershipFlag::~Interface_ParallelExchanger_ownershipFlag()
{
    m_is_owned.clear();
}

void Interface_ParallelExchanger_ownershipFlag::put_is_owned(const std::vector<bool>& is_owned)
{
    m_is_owned = is_owned;
}

void Interface_ParallelExchanger_ownershipFlag::build()
{
    const size_t num_locations = m_is_owned.size();

    // count local contracted
    size_t num_contracted = 0;
    for(size_t local_index = 0u; local_index < num_locations; local_index++)
    {
        num_contracted += m_is_owned[local_index];
    }

    // allocate
    m_contracted_to_local.resize(num_contracted, 0u);

    // fill
    size_t contracted_counter = 0;
    for(size_t local_index = 0u; local_index < num_locations; local_index++)
    {
        if(m_is_owned[local_index])
        {
            m_contracted_to_local[contracted_counter++] = local_index;
        }
    }

    // no communication
    size_t size = m_authority->mpi_wrapper->get_size();
    m_local_index_to_send.resize(size);
    m_local_index_to_recv.resize(size);
}

}
}
