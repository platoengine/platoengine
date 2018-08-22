// PlatoSubproblemLibraryVersion(6): a stand-alone library for the kernel filter for plato.
#include "PSL_Interface_ParallelExchanger_ownershipAndGlobal.hpp"

#include "PSL_Abstract_ParallelVector.hpp"
#include "PSL_Interface_ParallelExchanger_localAndNonlocal.hpp"
#include "PSL_Abstract_ParallelExchanger_Managed.hpp"
#include "PSL_Abstract_ParallelExchanger.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Interface_ParallelExchanger_global.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cstddef>
#include <algorithm>
#include <cassert>
#include <utility>
#include <map>

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_ParallelExchanger_ownershipAndGlobal::Interface_ParallelExchanger_ownershipAndGlobal(AbstractAuthority* authority) :
        Interface_ParallelExchanger_global(authority),
        m_is_owned()
{
}

Interface_ParallelExchanger_ownershipAndGlobal::~Interface_ParallelExchanger_ownershipAndGlobal()
{
    m_is_owned.clear();
}

void Interface_ParallelExchanger_ownershipAndGlobal::put_is_owned(const std::vector<bool>& is_owned)
{
    m_is_owned.assign(is_owned.begin(), is_owned.end());
}

void Interface_ParallelExchanger_ownershipAndGlobal::build_node_ownership(std::vector<int>& node_ownership)
{
    // node_ownership:
    // if <0, prepare to receive from owner (owning_proc = size + ownership)
    // if 0, owned exclusively on this proc
    // if 1, owned and exists on other procs, prepare to send to non-owners

    const size_t rank = m_authority->mpi_wrapper->get_rank();
    const size_t size = m_authority->mpi_wrapper->get_size();
    const int yet_unknown_owner = -int(size) - 1;

    // build global to local map and initialize node_ownership
    std::map<size_t, size_t> global_to_local;
    const size_t num_local = m_globals.size();
    for(size_t local_index = 0u; local_index < num_local; local_index++)
    {
        const size_t this_global_index = m_globals[local_index];
        global_to_local[this_global_index] = local_index;

        if(!m_is_owned[local_index])
        {
            // owner is not yet known
            node_ownership[local_index] = yet_unknown_owner;
        }
    }

    for(size_t this_other_proc = 0u; this_other_proc < size; this_other_proc++)
    {
        const size_t num_shared_nodes = m_processor_then_vector_of_local_and_nonlocal_pairs[this_other_proc].size();
        if(num_shared_nodes > 0u)
        {
            // code assumption : of course node exists on this proc
            assert(this_other_proc != rank);

            bool send_first = (this_other_proc < rank);

            // order shared nodes by globals
            std::vector<std::pair<size_t, size_t> > global_and_local_pair(num_shared_nodes);
            for(size_t shared_index = 0u; shared_index < num_shared_nodes; shared_index++)
            {
                const size_t this_local = m_processor_then_vector_of_local_and_nonlocal_pairs[this_other_proc][shared_index].first;
                const size_t this_global = m_globals[this_local];

                global_and_local_pair[shared_index] = std::make_pair(this_global, this_local);

                // if owned and shared, set node_ownership
                if(m_is_owned[this_local])
                {
                    node_ownership[this_local] = 1;
                }
            }
            std::sort(global_and_local_pair.begin(), global_and_local_pair.end());

            // build ownership indexed by globals
            std::vector<int> this_proc_is_owned(num_shared_nodes, 0);
            for(size_t shared_index = 0u; shared_index < num_shared_nodes; shared_index++)
            {
                const size_t this_local = global_and_local_pair[shared_index].second;

                this_proc_is_owned[shared_index] = m_is_owned[this_local];
            }

            std::vector<int> other_proc_is_owned(num_shared_nodes, 0);
            if(send_first)
            {
                m_authority->mpi_wrapper->send(this_other_proc, this_proc_is_owned);
                m_authority->mpi_wrapper->receive(this_other_proc, other_proc_is_owned);
            }
            else
            {
                m_authority->mpi_wrapper->receive(this_other_proc, other_proc_is_owned);
                m_authority->mpi_wrapper->send(this_other_proc, this_proc_is_owned);
            }

            // if owned on other processor, set node_ownership
            for(size_t shared_index = 0u; shared_index < num_shared_nodes; shared_index++)
            {
                // if owned on other processor
                if(other_proc_is_owned[shared_index])
                {
                    const size_t this_local = global_and_local_pair[shared_index].second;

                    if(node_ownership[this_local] != yet_unknown_owner)
                    {
                        m_authority->utilities->fatal_error("Interface_ParallelExchanger_ownershipAndGlobal: ambiguous parallel information. Aborting. \n\n");
                    }

                    node_ownership[this_local] = -int(size) + int(this_other_proc);
                }
            }
        }
    }

    // all node_ownership should be resolved
    for(size_t local_index = 0u; local_index < num_local; local_index++)
    {
        if(node_ownership[local_index] == yet_unknown_owner)
        {
            m_authority->utilities->fatal_error("Interface_ParallelExchanger_ownershipAndGlobal: incomplete specification of parallel information. Aborting. \n\n");
        }
    }
}

}
}
