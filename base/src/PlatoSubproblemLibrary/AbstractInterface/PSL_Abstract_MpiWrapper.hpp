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
#pragma once

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
class PointCloud;
class AxisAlignedBoundingBox;

namespace AbstractInterface
{
class GlobalUtilities;

class MpiWrapper
{
public:
    MpiWrapper(GlobalUtilities* utilities);
    virtual ~MpiWrapper();

    GlobalUtilities* get_utilities();
    bool is_root();

    virtual size_t get_rank() = 0;
    virtual size_t get_size() = 0;
    virtual double get_time() = 0;

    virtual void send(size_t target_rank, std::vector<int>& send_vector) = 0;
    virtual void send(size_t target_rank, std::vector<float>& send_vector) = 0;
    virtual void send(size_t target_rank, std::vector<double>& send_vector) = 0;
    void send(size_t target_rank, int send_data);
    void send(size_t target_rank, float send_data);
    void send(size_t target_rank, double send_data);

    virtual void receive(size_t source_rank, std::vector<int>& recv_vector) = 0;
    virtual void receive(size_t source_rank, std::vector<float>& recv_vector) = 0;
    virtual void receive(size_t source_rank, std::vector<double>& recv_vector) = 0;
    void receive(size_t source_rank, int& recv_data);
    void receive(size_t source_rank, float& recv_data);
    void receive(size_t source_rank, double& recv_data);

    virtual void all_gather(std::vector<int>& local_portion, std::vector<int>& global_portion) = 0;
    virtual void all_gather(std::vector<float>& local_portion, std::vector<float>& global_portion) = 0;
    virtual void all_gather(std::vector<double>& local_portion, std::vector<double>& global_portion) = 0;
    void all_gather(int local_portion, std::vector<int>& global_portion);
    void all_gather(float local_portion, std::vector<float>& global_portion);
    void all_gather(double local_portion, std::vector<double>& global_portion);

    virtual void all_reduce_min(std::vector<int>& local_contribution, std::vector<int>& global) = 0;
    virtual void all_reduce_min(std::vector<float>& local_contribution, std::vector<float>& global) = 0;
    virtual void all_reduce_min(std::vector<double>& local_contribution, std::vector<double>& global) = 0;
    void all_reduce_min(int& local_contribution, int& global);
    void all_reduce_min(float& local_contribution, float& global);
    void all_reduce_min(double& local_contribution, double& global);

    virtual void all_reduce_sum(std::vector<int>& local_contribution, std::vector<int>& global) = 0;
    virtual void all_reduce_sum(std::vector<float>& local_contribution, std::vector<float>& global) = 0;
    virtual void all_reduce_sum(std::vector<double>& local_contribution, std::vector<double>& global) = 0;
    void all_reduce_sum(int& local_contribution, int& global);
    void all_reduce_sum(float& local_contribution, float& global);
    void all_reduce_sum(double& local_contribution, double& global);

    virtual void all_reduce_max(std::vector<int>& local_contribution, std::vector<int>& global) = 0;
    virtual void all_reduce_max(std::vector<float>& local_contribution, std::vector<float>& global) = 0;
    virtual void all_reduce_max(std::vector<double>& local_contribution, std::vector<double>& global) = 0;
    void all_reduce_max(int& local_contribution, int& global);
    void all_reduce_max(float& local_contribution, float& global);
    void all_reduce_max(double& local_contribution, double& global);

    virtual void broadcast(size_t source_rank, std::vector<int>& broadcast_vector) = 0;
    virtual void broadcast(size_t source_rank, std::vector<float>& broadcast_vector) = 0;
    virtual void broadcast(size_t source_rank, std::vector<double>& broadcast_vector) = 0;
    void broadcast(size_t source_rank, int& broadcast_data);
    void broadcast(size_t source_rank, float& broadcast_data);
    void broadcast(size_t source_rank, double& broadcast_data);

    void send_point_cloud(size_t target_rank, PlatoSubproblemLibrary::PointCloud* points);
    PlatoSubproblemLibrary::PointCloud* receive_point_cloud(size_t source_rank);
    void receive_to_point_cloud(size_t source_rank, PlatoSubproblemLibrary::PointCloud* points);
    void all_gather(AxisAlignedBoundingBox& local_portion, std::vector<AxisAlignedBoundingBox>& global_portion);
    void send(size_t target_rank, const AxisAlignedBoundingBox& box);
    void receive(size_t source_rank, AxisAlignedBoundingBox& box);

protected:
    GlobalUtilities* m_utilities;

};

}

}
