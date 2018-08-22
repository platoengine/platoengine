// PlatoSubproblemLibraryVersion(6): a stand-alone library for the kernel filter for plato.
#include "PSL_Interface_ParallelExchanger_localAndNonlocal.hpp"

#include "PSL_Abstract_ParallelVector.hpp"
#include "PSL_Abstract_ParallelExchanger_Managed.hpp"
#include "PSL_Abstract_ParallelExchanger.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cstddef>
#include <algorithm>
#include <cassert>

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_ParallelExchanger_localAndNonlocal::Interface_ParallelExchanger_localAndNonlocal(AbstractAuthority* authority) :
        AbstractInterface::ParallelExchanger_Managed(authority),
        m_processor_then_vector_of_local_and_nonlocal_pairs(),
        m_num_locations()
{
}

Interface_ParallelExchanger_localAndNonlocal::~Interface_ParallelExchanger_localAndNonlocal()
{
    m_processor_then_vector_of_local_and_nonlocal_pairs.clear();
}

void Interface_ParallelExchanger_localAndNonlocal::put_shared_pairs(std::vector<std::vector<std::pair<size_t, size_t> > >& shared_local_and_nonlocal_pairs)
{
    // receive local and nonlocal shared pairs
    m_processor_then_vector_of_local_and_nonlocal_pairs.assign(shared_local_and_nonlocal_pairs.begin(),
                                                               shared_local_and_nonlocal_pairs.end());
}

void Interface_ParallelExchanger_localAndNonlocal::put_num_local_locations(size_t num_locations)
{
    m_num_locations = num_locations;
}

void Interface_ParallelExchanger_localAndNonlocal::build()
{
    // builds m_contracted_to_local, m_local_index_to_send, and m_local_index_to_recv
    size_t size = m_authority->mpi_wrapper->get_size();
    m_local_index_to_send.resize(size);
    m_local_index_to_recv.resize(size);

    std::vector<int> node_ownership(m_num_locations, 0);
    // if <0, prepare to receive from owner (owning_proc = size + ownership)
    // if 0, owned exclusively on this proc
    // if 1, owned and exists on other procs, prepare to send to non-owners

    build_node_ownership(node_ownership);

    // count local contracted
    size_t num_contracted = 0;
    for(size_t local_index = 0u; local_index < m_num_locations; local_index++)
    {
        num_contracted += (node_ownership[local_index] >= 0);
    }

    // allocate
    m_contracted_to_local.resize(num_contracted, 0u);

    // fill
    size_t contracted_counter = 0;
    for(size_t local_index = 0u; local_index < m_num_locations; local_index++)
    {
        if(node_ownership[local_index] >= 0)
        {
            m_contracted_to_local[contracted_counter++] = local_index;
        }
    }

    // count number to send/recv for each other processor
    std::vector<size_t> num_to_send(size, 0);
    std::vector<size_t> num_to_recv(size, 0);
    for(size_t this_other_proc = 0u; this_other_proc < size; this_other_proc++)
    {
        const size_t num_paired_nodes = m_processor_then_vector_of_local_and_nonlocal_pairs[this_other_proc].size();
        for(size_t index = 0; index < num_paired_nodes; index++)
        {
            const size_t this_local_node_index = m_processor_then_vector_of_local_and_nonlocal_pairs[this_other_proc][index].first;
            const int this_node_ownership = node_ownership[this_local_node_index];

            // if node_ownership == 0, do nothing; not planning to send or recv

            if(this_node_ownership < 0)
            {
                // if owns this node
                if(size_t(this_node_ownership + size) == this_other_proc)
                {
                    // count receiving this node from other proc
                    num_to_recv[this_other_proc]++;
                }
            }
            else // this_node_ownership == 1
            {
                // count sending this node to other proc
                num_to_send[this_other_proc]++;
            }
        }
    }

    // allocate
    for(size_t proc = 0; proc < size; proc++)
    {
        m_local_index_to_recv[proc].resize(num_to_recv[proc]);
        m_local_index_to_send[proc].resize(num_to_send[proc]);
    }

    // fill m_local_index_to_send
    std::vector<size_t> to_send_counters(size, 0u);
    for(size_t other_proc = 0u; other_proc < size; other_proc++)
    {
        // sort
        std::sort(m_processor_then_vector_of_local_and_nonlocal_pairs[other_proc].begin(),
                  m_processor_then_vector_of_local_and_nonlocal_pairs[other_proc].end());

        // fill sending indexes
        const size_t num_paired_nodes = m_processor_then_vector_of_local_and_nonlocal_pairs[other_proc].size();
        for(size_t index = 0; index < num_paired_nodes; index++)
        {
            const size_t this_local_node_index = m_processor_then_vector_of_local_and_nonlocal_pairs[other_proc][index].first;
            const int this_node_ownership = node_ownership[this_local_node_index];

            if(this_node_ownership > 0)
            {
                // sending this node to other proc
                m_local_index_to_send[other_proc][to_send_counters[other_proc]++] = this_local_node_index;
            }
        }
    }

    // fill m_local_index_to_recv
    std::vector<size_t> to_recv_counters(size, 0u);
    for(size_t other_proc = 0u; other_proc < size; other_proc++)
    {
        // swap local and nonlocal
        const size_t num_paired_nodes = m_processor_then_vector_of_local_and_nonlocal_pairs[other_proc].size();
        std::vector<std::pair<size_t, size_t> > nonlocal_and_local_pairs(num_paired_nodes);
        for(size_t index = 0; index < num_paired_nodes; index++)
        {
            nonlocal_and_local_pairs[index] =
                    std::make_pair(m_processor_then_vector_of_local_and_nonlocal_pairs[other_proc][index].second,
                                   m_processor_then_vector_of_local_and_nonlocal_pairs[other_proc][index].first);
        }

        // sort
        std::sort(nonlocal_and_local_pairs.begin(), nonlocal_and_local_pairs.end());
        for(size_t index = 0; index < num_paired_nodes; index++)
        {
            const size_t this_local_node_index = nonlocal_and_local_pairs[index].second;
            const int this_node_ownership = node_ownership[this_local_node_index];

            // if node_ownership == 0, do nothing; not planning to send or recv

            if(this_node_ownership < 0)
            {
                // if owns this node
                if(size_t(this_node_ownership + size) == other_proc)
                {
                    // receiving this node from other proc
                    m_local_index_to_recv[other_proc][to_recv_counters[other_proc]++] = this_local_node_index;
                }
            }
        }
    }

    m_processor_then_vector_of_local_and_nonlocal_pairs.clear();
}

void Interface_ParallelExchanger_localAndNonlocal::build_node_ownership(std::vector<int>& node_ownership)
{
    // if <0, exists on a lower proc ; prepare to receive from lowest proc (owning_proc = size + ownership)
    // if 0, owned exclusively on this proc
    // if 1, exists exclusively on this proc and upper procs ; prepare to send to upper procs

    const size_t rank = m_authority->mpi_wrapper->get_rank();
    const size_t size = m_authority->mpi_wrapper->get_size();

    for(size_t this_other_proc = 0u; this_other_proc < size; this_other_proc++)
    {
        const size_t num_shared_nodes = m_processor_then_vector_of_local_and_nonlocal_pairs[this_other_proc].size();
        for(size_t index = 0; index < num_shared_nodes; index++)
        {
            const size_t this_local_node_index = m_processor_then_vector_of_local_and_nonlocal_pairs[this_other_proc][index].first;

            // code assumption : of course node exists on this proc
            assert(this_other_proc != rank);

            if(this_other_proc < rank)
            {
                // this node exists on a lower proc
                node_ownership[this_local_node_index] = std::min(node_ownership[this_local_node_index],
                                                                 -int(size) + int(this_other_proc));
            }
            else // this_other_proc > rank
            {
                // if node_ownership < 0, do nothing; plan to receive this node
                // if node_ownership == 1, do nothing; already aware this node exists exclusively on this proc and upper procs
                if(node_ownership[this_local_node_index] == 0)
                {
                    // exists exclusively on this proc and upper procs
                    node_ownership[this_local_node_index] = 1;
                }
            }
        }
    }
}

}
}

