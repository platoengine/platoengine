// PlatoSubproblemLibraryVersion(6): a stand-alone library for the kernel filter for plato.
#pragma once

#include <mpi.h>
#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace example
{

size_t get_rank(MPI_Comm& comm);
size_t get_size(MPI_Comm& comm);

void send(MPI_Comm& comm, size_t target_rank, std::vector<int>& send_vector);
void send(MPI_Comm& comm, size_t target_rank, std::vector<float>& send_vector);
void send(MPI_Comm& comm, size_t target_rank, std::vector<double>& send_vector);

void receive(MPI_Comm& comm, size_t source_rank, std::vector<int>& receive_vector);
void receive(MPI_Comm& comm, size_t source_rank, std::vector<float>& receive_vector);
void receive(MPI_Comm& comm, size_t source_rank, std::vector<double>& receive_vector);

void all_gather(MPI_Comm& comm, std::vector<int>& local_portion, std::vector<int>& global_portion);
void all_gather(MPI_Comm& comm, std::vector<float>& local_portion, std::vector<float>& global_portion);
void all_gather(MPI_Comm& comm, std::vector<double>& local_portion, std::vector<double>& global_portion);

void all_reduce_min(MPI_Comm& comm, std::vector<int>& local_contribution, std::vector<int>& global_result);
void all_reduce_min(MPI_Comm& comm, std::vector<float>& local_contribution, std::vector<float>& global_result);
void all_reduce_min(MPI_Comm& comm, std::vector<double>& local_contribution, std::vector<double>& global_result);

void all_reduce_sum(MPI_Comm& comm, std::vector<int>& local_contribution, std::vector<int>& global_result);
void all_reduce_sum(MPI_Comm& comm, std::vector<float>& local_contribution, std::vector<float>& global_result);
void all_reduce_sum(MPI_Comm& comm, std::vector<double>& local_contribution, std::vector<double>& global_result);

void all_reduce_max(MPI_Comm& comm, std::vector<int>& local_contribution, std::vector<int>& global_result);
void all_reduce_max(MPI_Comm& comm, std::vector<float>& local_contribution, std::vector<float>& global_result);
void all_reduce_max(MPI_Comm& comm, std::vector<double>& local_contribution, std::vector<double>& global_result);

void broadcast(MPI_Comm& comm, size_t source_rank, std::vector<int>& broadcast_vector);
void broadcast(MPI_Comm& comm, size_t source_rank, std::vector<float>& broadcast_vector);
void broadcast(MPI_Comm& comm, size_t source_rank, std::vector<double>& broadcast_vector);

}
}
