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
#include "PSL_Implementation_MpiWrapper.hpp"

#include <mpi.h>
#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace example
{

size_t get_rank(MPI_Comm& comm)
{
    int rank = 0;
    MPI_Comm_rank(comm, &rank);
    return rank;
}
size_t get_size(MPI_Comm& comm)
{
    int size = 0;
    MPI_Comm_size(comm, &size);
    return size;
}

// int MPI_Send (void* message, int count, MPI_Datatype datatype, int dest, int tag, MPI_Comm comm);
void send(MPI_Comm& comm, size_t target_rank, std::vector<int>& send_vector)
{
    const size_t size = send_vector.size();
    MPI_Send(send_vector.data(), size, MPI_INT, target_rank, 0, comm);
}
void send(MPI_Comm& comm, size_t target_rank, std::vector<float>& send_vector)
{
    const size_t size = send_vector.size();
    MPI_Send(send_vector.data(), size, MPI_FLOAT, target_rank, 0, comm);
}
void send(MPI_Comm& comm, size_t target_rank, std::vector<double>& send_vector)
{
    const size_t size = send_vector.size();
    MPI_Send(send_vector.data(), size, MPI_DOUBLE, target_rank, 0, comm);
}

// int MPI_Recv (void* message, int count, MPI_Datatype datatype, int source, int tag, MPI_Comm comm, MPI_Status* status);
void receive(MPI_Comm& comm, size_t source_rank, std::vector<int>& receive_vector)
{
    MPI_Status status;
    const size_t size = receive_vector.size();
    MPI_Recv(receive_vector.data(), size, MPI_INT, source_rank, 0, comm, &status);
}
void receive(MPI_Comm& comm, size_t source_rank, std::vector<float>& receive_vector)
{
    MPI_Status status;
    const size_t size = receive_vector.size();
    MPI_Recv(receive_vector.data(), size, MPI_FLOAT, source_rank, 0, comm, &status);
}
void receive(MPI_Comm& comm, size_t source_rank, std::vector<double>& receive_vector)
{
    MPI_Status status;
    const size_t size = receive_vector.size();
    MPI_Recv(receive_vector.data(), size, MPI_DOUBLE, source_rank, 0, comm, &status);
}

//int MPI_Allgather ( void *sendbuf, int sendcount, MPI_Datatype sendtype,
//                    void *recvbuf, int recvcount, MPI_Datatype recvtype,
//                    MPI_Comm comm );
void all_gather(MPI_Comm& comm, std::vector<int>& local_portion, std::vector<int>& global_portion)
{
    const size_t send_size = local_portion.size();
    MPI_Allgather(local_portion.data(), send_size, MPI_INT, global_portion.data(), send_size, MPI_INT, comm);
}
void all_gather(MPI_Comm& comm, std::vector<float>& local_portion, std::vector<float>& global_portion)
{
    const size_t send_size = local_portion.size();
    MPI_Allgather(local_portion.data(), send_size, MPI_FLOAT, global_portion.data(), send_size, MPI_FLOAT, comm);
}
void all_gather(MPI_Comm& comm, std::vector<double>& local_portion, std::vector<double>& global_portion)
{
    const size_t send_size = local_portion.size();
    MPI_Allgather(local_portion.data(), send_size, MPI_DOUBLE, global_portion.data(), send_size, MPI_DOUBLE, comm);
}

// int MPI_Allreduce ( void *sendbuf, void *recvbuf, int count,
//                     MPI_Datatype datatype, MPI_Op op, MPI_Comm comm );
void all_reduce_min(MPI_Comm& comm, std::vector<int>& local_contribution, std::vector<int>& global_result)
{
    const size_t contribution_size = local_contribution.size();
    MPI_Allreduce(local_contribution.data(), global_result.data(), contribution_size, MPI_INT, MPI_MIN, comm);
}
void all_reduce_min(MPI_Comm& comm, std::vector<float>& local_contribution, std::vector<float>& global_result)
{
    const size_t contribution_size = local_contribution.size();
    MPI_Allreduce(local_contribution.data(), global_result.data(), contribution_size, MPI_FLOAT, MPI_MIN, comm);
}
void all_reduce_min(MPI_Comm& comm, std::vector<double>& local_contribution, std::vector<double>& global_result)
{
    const size_t contribution_size = local_contribution.size();
    MPI_Allreduce(local_contribution.data(), global_result.data(), contribution_size, MPI_DOUBLE, MPI_MIN, comm);
}

// int MPI_Allreduce ( void *sendbuf, void *recvbuf, int count,
//                     MPI_Datatype datatype, MPI_Op op, MPI_Comm comm );
void all_reduce_sum(MPI_Comm& comm, std::vector<int>& local_contribution, std::vector<int>& global_result)
{
    const size_t contribution_size = local_contribution.size();
    MPI_Allreduce(local_contribution.data(), global_result.data(), contribution_size, MPI_INT, MPI_SUM, comm);
}
void all_reduce_sum(MPI_Comm& comm, std::vector<float>& local_contribution, std::vector<float>& global_result)
{
    const size_t contribution_size = local_contribution.size();
    MPI_Allreduce(local_contribution.data(), global_result.data(), contribution_size, MPI_FLOAT, MPI_SUM, comm);
}
void all_reduce_sum(MPI_Comm& comm, std::vector<double>& local_contribution, std::vector<double>& global_result)
{
    const size_t contribution_size = local_contribution.size();
    MPI_Allreduce(local_contribution.data(), global_result.data(), contribution_size, MPI_DOUBLE, MPI_SUM, comm);
}

// int MPI_Allreduce ( void *sendbuf, void *recvbuf, int count,
//                     MPI_Datatype datatype, MPI_Op op, MPI_Comm comm );
void all_reduce_max(MPI_Comm& comm, std::vector<int>& local_contribution, std::vector<int>& global_result)
{
    const size_t contribution_size = local_contribution.size();
    MPI_Allreduce(local_contribution.data(), global_result.data(), contribution_size, MPI_INT, MPI_MAX, comm);
}
void all_reduce_max(MPI_Comm& comm, std::vector<float>& local_contribution, std::vector<float>& global_result)
{
    const size_t contribution_size = local_contribution.size();
    MPI_Allreduce(local_contribution.data(), global_result.data(), contribution_size, MPI_FLOAT, MPI_MAX, comm);
}
void all_reduce_max(MPI_Comm& comm, std::vector<double>& local_contribution, std::vector<double>& global_result)
{
    const size_t contribution_size = local_contribution.size();
    MPI_Allreduce(local_contribution.data(), global_result.data(), contribution_size, MPI_DOUBLE, MPI_MAX, comm);
}

// int MPI_Bcast ( void *buffer, int count, MPI_Datatype datatype, int root,
//                 MPI_Comm comm );
void broadcast(MPI_Comm& comm, size_t source_rank, std::vector<int>& broadcast_vector)
{
    const size_t broadcast_size = broadcast_vector.size();
    MPI_Bcast(broadcast_vector.data(), broadcast_size, MPI_INT, source_rank, comm);
}
void broadcast(MPI_Comm& comm, size_t source_rank, std::vector<float>& broadcast_vector)
{
    const size_t broadcast_size = broadcast_vector.size();
    MPI_Bcast(broadcast_vector.data(), broadcast_size, MPI_FLOAT, source_rank, comm);
}
void broadcast(MPI_Comm& comm, size_t source_rank, std::vector<double>& broadcast_vector)
{
    const size_t broadcast_size = broadcast_vector.size();
    MPI_Bcast(broadcast_vector.data(), broadcast_size, MPI_DOUBLE, source_rank, comm);
}

}
}
