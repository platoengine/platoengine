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

// PlatoSubproblemLibraryVersion(6): a stand-alone library for the kernel filter for plato.
#include "PSL_Interface_MpiWrapper.hpp"

#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Implementation_MpiWrapper.hpp"
#include "PSL_Interface_MpiWrapper.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"

#include <mpi.h>
#include <vector>
#include <cstddef>
#include <sstream>

//#define PSL_INTERFACE_MPIWRAPPER_VERBOSE

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_MpiWrapper::Interface_MpiWrapper(AbstractInterface::GlobalUtilities* utilities, MPI_Comm* comm) :
        AbstractInterface::MpiWrapper(utilities),
        m_comm(comm)
{
}
Interface_MpiWrapper::~Interface_MpiWrapper()
{
}

size_t Interface_MpiWrapper::get_rank()
{
    return example::get_rank(*m_comm);
}
size_t Interface_MpiWrapper::get_size()
{
    return example::get_size(*m_comm);
}

double Interface_MpiWrapper::get_time()
{
    return MPI_Wtime();
}

void Interface_MpiWrapper::send(size_t target_rank, std::vector<int>& send_vector)
{
#ifdef PSL_INTERFACE_MPIWRAPPER_VERBOSE
    std::stringstream stream;
    stream << "send::from,to,size,type:" << get_rank() << "," << target_rank << "," << send_vector.size() << ",int\n";
    m_utilities->print(stream.str());
#endif
    example::send(*m_comm, target_rank, send_vector);
}
void Interface_MpiWrapper::send(size_t target_rank, std::vector<float>& send_vector)
{
#ifdef PSL_INTERFACE_MPIWRAPPER_VERBOSE
    std::stringstream stream;
    stream << "send::from,to,size,type:" << get_rank() << "," << target_rank << "," << send_vector.size() << ",float\n";
    m_utilities->print(stream.str());
#endif
    example::send(*m_comm, target_rank, send_vector);
}
void Interface_MpiWrapper::send(size_t target_rank, std::vector<double>& send_vector)
{
#ifdef PSL_INTERFACE_MPIWRAPPER_VERBOSE
    std::stringstream stream;
    stream << "send::from,to,size,type:" << get_rank() << "," << target_rank << "," << send_vector.size() << ",double\n";
    m_utilities->print(stream.str());
#endif
    example::send(*m_comm, target_rank, send_vector);
}

void Interface_MpiWrapper::receive(size_t source_rank, std::vector<int>& send_vector)
{
#ifdef PSL_INTERFACE_MPIWRAPPER_VERBOSE
    std::stringstream stream;
    stream << "recv::from,to,size,type:" << source_rank << "," << get_rank() << "," << send_vector.size() << ",int\n";
    m_utilities->print(stream.str());
#endif
    example::receive(*m_comm, source_rank, send_vector);
}
void Interface_MpiWrapper::receive(size_t source_rank, std::vector<float>& send_vector)
{
#ifdef PSL_INTERFACE_MPIWRAPPER_VERBOSE
    std::stringstream stream;
    stream << "recv::from,to,size,type:" << source_rank << "," << get_rank() << "," << send_vector.size() << ",float\n";
    m_utilities->print(stream.str());
#endif
    example::receive(*m_comm, source_rank, send_vector);
}
void Interface_MpiWrapper::receive(size_t source_rank, std::vector<double>& send_vector)
{
#ifdef PSL_INTERFACE_MPIWRAPPER_VERBOSE
    std::stringstream stream;
    stream << "recv::from,to,size,type:" << source_rank << "," << get_rank() << "," << send_vector.size() << ",double\n";
    m_utilities->print(stream.str());
#endif
    example::receive(*m_comm, source_rank, send_vector);
}

void Interface_MpiWrapper::all_gather(std::vector<int>& local_portion, std::vector<int>& global_portion)
{
    example::all_gather(*m_comm, local_portion, global_portion);
}
void Interface_MpiWrapper::all_gather(std::vector<float>& local_portion, std::vector<float>& global_portion)
{
    example::all_gather(*m_comm, local_portion, global_portion);
}
void Interface_MpiWrapper::all_gather(std::vector<double>& local_portion, std::vector<double>& global_portion)
{
    example::all_gather(*m_comm, local_portion, global_portion);
}

void Interface_MpiWrapper::all_reduce_min(std::vector<int>& local_contribution, std::vector<int>& global_result)
{
    example::all_reduce_min(*m_comm, local_contribution, global_result);
}
void Interface_MpiWrapper::all_reduce_min(std::vector<float>& local_contribution, std::vector<float>& global_result)
{
    example::all_reduce_min(*m_comm, local_contribution, global_result);
}
void Interface_MpiWrapper::all_reduce_min(std::vector<double>& local_contribution, std::vector<double>& global_result)
{
    example::all_reduce_min(*m_comm, local_contribution, global_result);
}

void Interface_MpiWrapper::all_reduce_sum(std::vector<int>& local_contribution, std::vector<int>& global_result)
{
    example::all_reduce_sum(*m_comm, local_contribution, global_result);
}
void Interface_MpiWrapper::all_reduce_sum(std::vector<float>& local_contribution, std::vector<float>& global_result)
{
    example::all_reduce_sum(*m_comm, local_contribution, global_result);
}
void Interface_MpiWrapper::all_reduce_sum(std::vector<double>& local_contribution, std::vector<double>& global_result)
{
    example::all_reduce_sum(*m_comm, local_contribution, global_result);
}

void Interface_MpiWrapper::all_reduce_max(std::vector<int>& local_contribution, std::vector<int>& global_result)
{
    example::all_reduce_max(*m_comm, local_contribution, global_result);
}
void Interface_MpiWrapper::all_reduce_max(std::vector<float>& local_contribution, std::vector<float>& global_result)
{
    example::all_reduce_max(*m_comm, local_contribution, global_result);
}
void Interface_MpiWrapper::all_reduce_max(std::vector<double>& local_contribution, std::vector<double>& global_result)
{
    example::all_reduce_max(*m_comm, local_contribution, global_result);
}

void Interface_MpiWrapper::broadcast(size_t source_rank, std::vector<int>& broadcast_vector)
{
    example::broadcast(*m_comm, source_rank, broadcast_vector);
}
void Interface_MpiWrapper::broadcast(size_t source_rank, std::vector<float>& broadcast_vector)
{
    example::broadcast(*m_comm, source_rank, broadcast_vector);
}
void Interface_MpiWrapper::broadcast(size_t source_rank, std::vector<double>& broadcast_vector)
{
    example::broadcast(*m_comm, source_rank, broadcast_vector);
}

}
}
