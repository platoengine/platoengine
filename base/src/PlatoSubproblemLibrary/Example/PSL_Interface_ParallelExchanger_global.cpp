// PlatoSubproblemLibraryVersion(6): a stand-alone library for the kernel filter for plato.
#include "PSL_Interface_ParallelExchanger_global.hpp"

#include "PSL_Abstract_ParallelVector.hpp"
#include "PSL_Interface_ParallelExchanger_localAndNonlocal.hpp"
#include "PSL_Abstract_ParallelExchanger_Managed.hpp"
#include "PSL_Abstract_ParallelExchanger.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cstddef>
#include <algorithm>
#include <cassert>
#include <utility>

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_ParallelExchanger_global::Interface_ParallelExchanger_global(AbstractAuthority* authority) :
        Interface_ParallelExchanger_localAndNonlocal(authority),
        m_globals()
{
}

Interface_ParallelExchanger_global::~Interface_ParallelExchanger_global()
{
    m_globals.clear();
}

void Interface_ParallelExchanger_global::put_globals(const std::vector<size_t>& globals)
{
    m_globals.assign(globals.begin(), globals.end());
}

void Interface_ParallelExchanger_global::build()
{
    // put num local locations
    const size_t num_local = m_globals.size();
    put_num_local_locations(num_local);

    std::vector<std::pair<size_t, size_t> > global_and_local_pair(num_local);
    for(size_t local_index = 0u; local_index < num_local; local_index++)
    {
        global_and_local_pair[local_index] = std::make_pair(m_globals[local_index], local_index);
    }
    std::sort(global_and_local_pair.begin(), global_and_local_pair.end());

    // get minimums and maximums
    size_t mpi_size = m_authority->mpi_wrapper->get_size();
    const int global_minimum_index_on_local = global_and_local_pair.front().first;
    std::vector<int> global_minimum_indexes(mpi_size, 0);
    m_authority->mpi_wrapper->all_gather(global_minimum_index_on_local, global_minimum_indexes);
    const int global_maximum_index_on_local = global_and_local_pair.back().first;
    std::vector<int> global_maximum_indexes(mpi_size, 0);
    m_authority->mpi_wrapper->all_gather(global_maximum_index_on_local, global_maximum_indexes);

    // allocate
    std::vector<std::vector<std::pair<size_t, size_t> > > shared_local_and_nonlocal_pairs(mpi_size);

    // communicate when globals intersect
    size_t mpi_rank = m_authority->mpi_wrapper->get_rank();
    for(size_t proc = 0u; proc < mpi_size; proc++)
    {
        // do not communicate with yourself
        if(proc == mpi_rank)
        {
            continue;
        }

        // no need to communicate
        const bool intersects = intersect(global_minimum_index_on_local,
                                          global_maximum_index_on_local,
                                          global_minimum_indexes[proc],
                                          global_maximum_indexes[proc]);
        if(!intersects)
        {
            continue;
        }

        // choose which processor does the matching for the two
        const bool send_first = (proc < mpi_rank);
        if(send_first)
        {
            build_send_first(num_local,
                             proc,
                             global_minimum_indexes,
                             global_maximum_indexes,
                             global_and_local_pair,
                             shared_local_and_nonlocal_pairs);
        }
        else
        {
            build_receive_first(num_local,
                                proc,
                                global_and_local_pair,
                                shared_local_and_nonlocal_pairs);
        }
    }

    // put shared pairs
    put_shared_pairs(shared_local_and_nonlocal_pairs);

    // build as local and nonlocal pairs
    Interface_ParallelExchanger_localAndNonlocal::build();

    m_globals.clear();
}

void Interface_ParallelExchanger_global::build_send_first(size_t num_local,
                                                          size_t proc,
                                                          const std::vector<int>& global_minimum_indexes,
                                                          const std::vector<int>& global_maximum_indexes,
                                                          const std::vector<std::pair<size_t, size_t> >& global_and_local_pair,
                                                          std::vector<std::vector<std::pair<size_t, size_t> > >& shared_local_and_nonlocal_pairs)
{
    // count for sending
    size_t num_send = 0u;
    for(size_t local_index = 0u; local_index < num_local; local_index++)
    {
        const int this_global = m_globals[local_index];
        num_send += (global_minimum_indexes[proc] <= this_global) && (this_global <= global_maximum_indexes[proc]);
    }

    // allocate for sending
    m_authority->mpi_wrapper->send(proc, (int)num_send);
    std::vector<int> local_index_send(num_send);
    std::vector<int> global_index_send(num_send);

    // fill for sending
    size_t send_counter = 0u;
    for(size_t local_index = 0u; local_index < num_local; local_index++)
    {
        const int this_global = global_and_local_pair[local_index].first;

        if((global_minimum_indexes[proc] <= this_global) && (this_global <= global_maximum_indexes[proc]))
        {
            const int this_local = global_and_local_pair[local_index].second;

            local_index_send[send_counter] = this_local;
            global_index_send[send_counter] = this_global;
            send_counter++;
        }
    }

    // send pairs
    m_authority->mpi_wrapper->send(proc, local_index_send);
    m_authority->mpi_wrapper->send(proc, global_index_send);

    // count for receiving
    int num_pairs = 0;
    m_authority->mpi_wrapper->receive(proc, num_pairs);

    // allocate for receiving
    std::vector<int> local_indexes_recv(num_pairs);
    std::vector<int> nonlocal_indexes_recv(num_pairs);

    // fill by receiving
    m_authority->mpi_wrapper->receive(proc, local_indexes_recv);
    m_authority->mpi_wrapper->receive(proc, nonlocal_indexes_recv);

    shared_local_and_nonlocal_pairs[proc].resize(num_pairs);
    for(int pair = 0u; pair < num_pairs; pair++)
    {
        const size_t this_local = local_indexes_recv[pair];
        const size_t this_nonlocal = nonlocal_indexes_recv[pair];
        shared_local_and_nonlocal_pairs[proc][pair] = std::make_pair(this_local, this_nonlocal);
    }
}

void Interface_ParallelExchanger_global::build_receive_first(size_t num_local,
                                                             size_t proc,
                                                             const std::vector<std::pair<size_t, size_t> >& global_and_local_pair,
                                                             std::vector<std::vector<std::pair<size_t, size_t> > >& shared_local_and_nonlocal_pairs)
{
    // receive count
    int num_recv = 0;
    m_authority->mpi_wrapper->receive(proc, num_recv);

    // allocate for receive
    std::vector<int> nonloal_index_recv(num_recv);
    std::vector<int> global_index_recv(num_recv);

    // receive
    m_authority->mpi_wrapper->receive(proc, nonloal_index_recv);
    m_authority->mpi_wrapper->receive(proc, global_index_recv);

    size_t num_pairs = 0u;
    std::vector<int> nonlocal_indexes_send;
    std::vector<int> local_indexes_send;

    // build pairs
    for(size_t repeat = 0u; repeat < 2u; repeat++)
    {
        size_t local_index = 0u;
        size_t nonlocal_index = 0u;

        // linear matching search
        const size_t local_index_max = num_local;
        const size_t nonlocal_index_max = num_recv;
        while(local_index != local_index_max && nonlocal_index != nonlocal_index_max)
        {
            const size_t global_from_local = global_and_local_pair[local_index].first;
            const size_t global_from_nonlocal = global_index_recv[nonlocal_index];

            if(global_from_local < global_from_nonlocal)
            {
                // advance
                local_index++;
            }
            else if(global_from_nonlocal < global_from_local)
            {
                // advance
                nonlocal_index++;
            }
            else
            {
                if(repeat == 1u)
                {
                    // fill
                    const size_t this_local_index = global_and_local_pair[local_index].second;
                    const size_t this_nonlocal_index = nonloal_index_recv[nonlocal_index];

                    shared_local_and_nonlocal_pairs[proc][num_pairs] = std::make_pair(this_local_index, this_nonlocal_index);
                    nonlocal_indexes_send[num_pairs] = this_nonlocal_index;
                    local_indexes_send[num_pairs] = this_local_index;
                }
                num_pairs++;
                local_index++;
                nonlocal_index++;
            }
        }

        if(repeat == 0)
        {
            m_authority->mpi_wrapper->send(proc, (int)num_pairs);

            // allocate
            shared_local_and_nonlocal_pairs[proc].resize(num_pairs);
            nonlocal_indexes_send.resize(num_pairs);
            local_indexes_send.resize(num_pairs);
            num_pairs = 0u;
        }
    }

    // send
    m_authority->mpi_wrapper->send(proc, nonlocal_indexes_send);
    m_authority->mpi_wrapper->send(proc, local_indexes_send);
}

bool Interface_ParallelExchanger_global::intersect(int a_lower, int a_upper, int b_lower, int b_upper)
{
    return (b_lower <= a_upper) && (a_lower <= b_upper);
}

}
}
