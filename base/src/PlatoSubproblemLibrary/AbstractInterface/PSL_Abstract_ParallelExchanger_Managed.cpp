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

// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_Abstract_ParallelExchanger_Managed.hpp"

#include "PSL_Abstract_ParallelVector.hpp"
#include "PSL_Abstract_ParallelExchanger.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cstddef>
#include <algorithm>
#include <cassert>
#include <iostream>
#include <cmath>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{

ParallelExchanger_Managed::ParallelExchanger_Managed(AbstractAuthority* authority) :
        AbstractInterface::ParallelExchanger(authority),
        m_contracted_to_local(),
        m_local_index_to_send(),
        m_local_index_to_recv()
{
}

ParallelExchanger_Managed::~ParallelExchanger_Managed()
{
    m_contracted_to_local.clear();
    m_local_index_to_recv.clear();
    m_local_index_to_send.clear();
}

std::vector<size_t> ParallelExchanger_Managed::get_local_contracted_indexes()
{
    // get indexes of local points that are locally owned
    return m_contracted_to_local;
}

std::vector<double> ParallelExchanger_Managed::get_contraction_to_local_indexes(AbstractInterface::ParallelVector* input_data_vector)
{
    // convert data vector in parallel format to vector of values for locally owned points
    const size_t num_contracted = m_contracted_to_local.size();
    std::vector<double> contraction_result(num_contracted, 0.);
    for(size_t contracted_index = 0u; contracted_index < num_contracted; contracted_index++)
    {
        const size_t local_index = m_contracted_to_local[contracted_index];
        contraction_result[contracted_index] = input_data_vector->get_value(local_index);
    }

    return contraction_result;
}

void ParallelExchanger_Managed::get_expansion_to_parallel_vector(const std::vector<double>& input_data_vector,
                                                                  AbstractInterface::ParallelVector* output_data_vector)
{
    // communicate between processors to expand the locally owned data to parallel format with some values shared on processors

    // do local mapping
    const size_t num_contracted = m_contracted_to_local.size();
    for(size_t contracted_index = 0u; contracted_index < num_contracted; contracted_index++)
    {
        const size_t local_index = m_contracted_to_local[contracted_index];
        output_data_vector->set_value(local_index, input_data_vector[contracted_index]);
    }

    // do communication
    size_t size = m_authority->mpi_wrapper->get_size();

    // receive from lower procs
    for(size_t proc_ = 0; proc_ < size; proc_++)
    {
        const size_t num_to_recv = m_local_index_to_recv[proc_].size();

        if(num_to_recv > 0)
        {
            std::vector<double> field_values_to_recv(num_to_recv, 0.0);

            m_authority->mpi_wrapper->receive(proc_, field_values_to_recv);

            for(size_t recv_index = 0; recv_index < num_to_recv; recv_index++)
            {
                const size_t index_to_recv_to = m_local_index_to_recv[proc_][recv_index];
                const double field_value_recvd = field_values_to_recv[recv_index];
                output_data_vector->set_value(index_to_recv_to, field_value_recvd);
            }
        }
    }

    // send to upper procs
    for(size_t proc_ = 0; proc_ < size; proc_++)
    {
        const size_t num_to_send = m_local_index_to_send[proc_].size();

        if(num_to_send > 0)
        {
            std::vector<double> field_values_to_send(num_to_send, 0.0);

            for(size_t send_index = 0; send_index < num_to_send; send_index++)
            {
                const size_t index_to_send = m_local_index_to_send[proc_][send_index];
                const double field_value_to_send = output_data_vector->get_value(index_to_send);
                field_values_to_send[send_index] = field_value_to_send;
            }

            m_authority->mpi_wrapper->send(proc_, field_values_to_send);
        }
    }
}

double ParallelExchanger_Managed::get_maximum_absolute_parallel_error(ParallelVector* input_data_vector)
{
    // determine maximum absolute parallel error

    // determine original
    const size_t original_length = input_data_vector->get_length();
    std::vector<double> original(original_length, 0.);
    for(size_t o = 0u; o < original_length; o++)
    {
        original[o] = input_data_vector->get_value(o);
    }

    // contract and expand
    get_expansion_to_parallel_vector(get_contraction_to_local_indexes(input_data_vector), input_data_vector);

    // local error
    double local_max_absolute_error = 0.;
    for(size_t o = 0u; o < original_length; o++)
    {
        const double this_error = fabs(input_data_vector->get_value(o) - original[o]);
        local_max_absolute_error = std::max(local_max_absolute_error, this_error);
    }
    double global_max_absolute_error = 0.;
    m_authority->mpi_wrapper->all_reduce_max(local_max_absolute_error, global_max_absolute_error);

    return global_max_absolute_error;
}

}
}

