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
#pragma once

#include "PSL_Abstract_MpiWrapper.hpp"

#include <vector>
#include <cstddef>
#include <mpi.h>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class GlobalUtilities;
}

namespace example
{

class Interface_MpiWrapper : public AbstractInterface::MpiWrapper
{
public:
    Interface_MpiWrapper(AbstractInterface::GlobalUtilities* utilities, MPI_Comm* comm);
    virtual ~Interface_MpiWrapper();

    virtual size_t get_rank();
    virtual size_t get_size();

    virtual double get_time();

    virtual void send(size_t target_rank, std::vector<int>& send_vector);
    virtual void send(size_t target_rank, std::vector<float>& send_vector);
    virtual void send(size_t target_rank, std::vector<double>& send_vector);

    virtual void receive(size_t source_rank, std::vector<int>& send_vector);
    virtual void receive(size_t source_rank, std::vector<float>& send_vector);
    virtual void receive(size_t source_rank, std::vector<double>& send_vector);

    virtual void all_gather(std::vector<int>& local_portion, std::vector<int>& global_portion);
    virtual void all_gather(std::vector<float>& local_portion, std::vector<float>& global_portion);
    virtual void all_gather(std::vector<double>& local_portion, std::vector<double>& global_portion);

    virtual void all_reduce_min(std::vector<int>& local_contribution, std::vector<int>& global);
    virtual void all_reduce_min(std::vector<float>& local_contribution, std::vector<float>& global);
    virtual void all_reduce_min(std::vector<double>& local_contribution, std::vector<double>& global);

    virtual void all_reduce_sum(std::vector<int>& local_contribution, std::vector<int>& global_result);
    virtual void all_reduce_sum(std::vector<float>& local_contribution, std::vector<float>& global_result);
    virtual void all_reduce_sum(std::vector<double>& local_contribution, std::vector<double>& global_result);

    virtual void all_reduce_max(std::vector<int>& local_contribution, std::vector<int>& global);
    virtual void all_reduce_max(std::vector<float>& local_contribution, std::vector<float>& global);
    virtual void all_reduce_max(std::vector<double>& local_contribution, std::vector<double>& global);

    virtual void broadcast(size_t source_rank, std::vector<int>& broadcast_vector);
    virtual void broadcast(size_t source_rank, std::vector<float>& broadcast_vector);
    virtual void broadcast(size_t source_rank, std::vector<double>& broadcast_vector);

protected:
    MPI_Comm* m_comm;

};

}
}
