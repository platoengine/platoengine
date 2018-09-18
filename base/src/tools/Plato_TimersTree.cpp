/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
*/

#include "Plato_TimersTree.hpp"

#include "Plato_FreeFunctions.hpp"

#include <mpi.h>                // for MPI_Wtime
#include <algorithm>            // for fill
#include <cstdlib>              // for size_t, NULL
#include <iomanip>              // for operator<<, setw
#include <iostream>             // for operator<<, basic_ostream, char_traits
#include <memory>               // for allocator_traits<>::value_type

namespace Plato
{

TimersTree::TimersTree(const MPI_Comm& aLocalComm,
                       const int num_keys) :
        mLocalComm(aLocalComm),
        m_num_keys(num_keys),
        m_accumulated_times_by_key(num_keys + 1, 0.0),
        m_num_entrances_by_key(num_keys + 1, 0u),
        m_stack_of_keys_and_times(),
        m_consider_partition(num_keys + 1, true),
        m_just_incrementing(false)
{
    int rank = -1;
    MPI_Comm_rank(mLocalComm, &rank);
    if(rank == 0)
    {
        this->begin_partition(m_num_keys);
    }
}

TimersTree::~TimersTree()
{
}

bool TimersTree::begin_partition(const int partition)
{
    // only proceed on rank 0
    int rank = -1;
    MPI_Comm_rank(mLocalComm, &rank);
    if(rank != 0)
    {
        return false;
    }

    if (partition > m_num_keys) {
        return false;
    }

    m_stack_of_keys_and_times.push_back(std::make_pair(partition, -MPI_Wtime()));
    if(m_just_incrementing)
    {
        m_stack_of_keys_and_times.back().second = 0.0;
    }

    m_num_entrances_by_key[partition]++;
    return true;
}

bool TimersTree::end_partition()
{
    // only proceed on rank 0
     int rank = -1;
     MPI_Comm_rank(mLocalComm, &rank);
     if(rank != 0)
     {
         return false;
     }

    if(m_stack_of_keys_and_times.size() == 0)
    {
        return false;
    }

    // get completed partition
    double accumulated_time = m_stack_of_keys_and_times.back().second + MPI_Wtime();
    if(m_just_incrementing)
    {
        accumulated_time = m_stack_of_keys_and_times.back().second;
    }
    const int completed_partition = m_stack_of_keys_and_times.back().first;

    // pop stack
    m_stack_of_keys_and_times.pop_back();

    // if considering this partition, make contributions
    if(m_consider_partition[completed_partition])
    {
        // update accumulated time in total
        m_accumulated_times_by_key[completed_partition] += accumulated_time;

        // update accumulated time in stack
        if(!m_just_incrementing)
        {
            const int stack_size = m_stack_of_keys_and_times.size();
            for(int stack_index = 0; stack_index < stack_size; stack_index++)
            {
                m_stack_of_keys_and_times[stack_index].second -= accumulated_time;
            }
        }
    }

    return true;
}

bool TimersTree::print_results()
{
    // only proceed on rank 0
     int rank = -1;
     MPI_Comm_rank(mLocalComm, &rank);
     if(rank != 0)
     {
         return false;
     }

    // expect stack to just contain ending key
    if(m_stack_of_keys_and_times.size() != 1)
    {
        return false;
    }
    else if(m_stack_of_keys_and_times[0].first != m_num_keys)
    {
        return false;
    }

    // close the ending key
    if(!this->end_partition())
    {
        return false; // reaching this line should be impossible
    }

    const double total_accumulated_time = free_sum(m_accumulated_times_by_key);
    const double nonzero_total_accumulated_time = (total_accumulated_time <= 0.0 ? 1.0 : total_accumulated_time);

    const int partition_width = 50;
    const int time_width = 15;
    const int percentage_width = 15;
    const int entrances_width = 15;

    std::cout << "Plato Timers:" << std::endl;
    std::cout << std::left << std::setw(partition_width) << "partition" << "|"
              << std::setw(time_width) << "seconds" << "|"
              << std::setw(percentage_width) << "percentage" << "|"
              << std::setw(entrances_width) << "entrances" << std::endl;

    for(int partition_index = 0; partition_index <= m_num_keys; partition_index++)
    {
        const double this_percentage = 100.0 * m_accumulated_times_by_key[partition_index] / nonzero_total_accumulated_time;

        const char* this_partition_name = this->get_string_from_partition_enum(partition_index);
        if(!this_partition_name)
        {
            return false; // acquisition of partition name failed
        }

        std::cout << std::left << std::setw(partition_width) << this_partition_name << "|"
                  << std::setw(time_width) << m_accumulated_times_by_key[partition_index] << "|"
                  << std::setw(percentage_width) << this_percentage << "|"
                  << std::setw(entrances_width) << m_num_entrances_by_key[partition_index] << std::endl;
    }

    return true;
}

void TimersTree::unit_testing_incrament()
{
    // only proceed on rank 0
     int rank = -1;
     MPI_Comm_rank(mLocalComm, &rank);
     if(rank != 0)
     {
         return;
     }

    if(!m_just_incrementing)
    {
        std::fill(m_accumulated_times_by_key.begin(), m_accumulated_times_by_key.end(), 0.0);
        const int stack_size = m_stack_of_keys_and_times.size();
        for(int stack_index = 0; stack_index < stack_size; stack_index++)
        {
            m_stack_of_keys_and_times[stack_index].second = 0.0;
        }

        m_just_incrementing = true;
    }

    m_stack_of_keys_and_times.back().second++;
}

std::vector<double> TimersTree::unit_testing_get_timers() const
{
    // only proceed on rank 0
    int rank = -1;
    MPI_Comm_rank(mLocalComm, &rank);
    if(rank != 0)
    {
        std::vector<double> empty_result;
        return empty_result;
    }

    std::vector<double> result(m_accumulated_times_by_key.begin(), m_accumulated_times_by_key.end());
    return result;
}

std::vector<size_t> TimersTree::unit_testing_get_entrances() const
{
    // only proceed on rank 0
    int rank = -1;
    MPI_Comm_rank(mLocalComm, &rank);
    if(rank != 0)
    {
        std::vector<size_t> empty_result;
        return empty_result;
    }

    std::vector<size_t> result(m_num_entrances_by_key.begin(), m_num_entrances_by_key.end());
    return result;
}

#define PLATO_TIMERSTREE_PARTITION_CASE(name) \
        case timer_partition_t::timer_partition_t::name: \
        { \
            return #name; \
        } \

const char* TimersTree::get_string_from_partition_enum(const int partition) const
{
    switch(partition)
    {
        PLATO_TIMERSTREE_PARTITION_CASE(optimizer)
        PLATO_TIMERSTREE_PARTITION_CASE(filter)
        PLATO_TIMERSTREE_PARTITION_CASE(mesh_services)
        PLATO_TIMERSTREE_PARTITION_CASE(file_input_output)
        PLATO_TIMERSTREE_PARTITION_CASE(physics_compute)
        PLATO_TIMERSTREE_PARTITION_CASE(aggregator)
        case timer_partition_t::timer_partition_t::TOTAL_NUM_KEYS:
        {
            return "uncategorized";
        }
        default:
        {
            break;
        }
    }

    std::cout << __func__ << ": could not match partition (" << partition << "); displayed time should not be trusted. " << std::endl;

    return NULL;
}

}
