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
#include "PSL_Abstract_MpiWrapper.hpp"

#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_PointCloud.hpp"
#include "PSL_Point.hpp"
#include "PSL_AxisAlignedBoundingBox.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{

namespace AbstractInterface
{

MpiWrapper::MpiWrapper(GlobalUtilities* utilities) :
        m_utilities(utilities)
{
}

MpiWrapper::~MpiWrapper()
{
}

GlobalUtilities* MpiWrapper::get_utilities()
{
    return m_utilities;
}

bool MpiWrapper::is_root()
{
    return get_rank() == 0u;
}

void MpiWrapper::send(size_t target_rank, int send_data)
{
    std::vector<int> send_vector(1u, send_data);
    this->send(target_rank, send_vector);
}
void MpiWrapper::send(size_t target_rank, float send_data)
{
    std::vector<float> send_vector(1u, send_data);
    this->send(target_rank, send_vector);
}
void MpiWrapper::send(size_t target_rank, double send_data)
{
    std::vector<double> send_vector(1u, send_data);
    this->send(target_rank, send_vector);
}

void MpiWrapper::receive(size_t source_rank, int& recv_data)
{
    std::vector<int> recv_vector(1u, 0);
    this->receive(source_rank, recv_vector);
    recv_data = recv_vector[0];
}
void MpiWrapper::receive(size_t source_rank, float& recv_data)
{
    std::vector<float> recv_vector(1u, 0);
    this->receive(source_rank, recv_vector);
    recv_data = recv_vector[0];
}
void MpiWrapper::receive(size_t source_rank, double& recv_data)
{
    std::vector<double> recv_vector(1u, 0);
    this->receive(source_rank, recv_vector);
    recv_data = recv_vector[0];
}

void MpiWrapper::all_gather(int local_portion, std::vector<int>& global_portion)
{
    std::vector<int> local_v(1u, local_portion);
    this->all_gather(local_v, global_portion);
}
void MpiWrapper::all_gather(float local_portion, std::vector<float>& global_portion)
{
    std::vector<float> local_v(1u, local_portion);
    this->all_gather(local_v, global_portion);
}
void MpiWrapper::all_gather(double local_portion, std::vector<double>& global_portion)
{
    std::vector<double> local_v(1u, local_portion);
    this->all_gather(local_v, global_portion);
}

void MpiWrapper::all_reduce_min(int& local_contribution, int& global)
{
    std::vector<int> local_v(1u, local_contribution);
    std::vector<int> global_v(1u, 0);
    this->all_reduce_min(local_v, global_v);
    global = global_v[0];
}
void MpiWrapper::all_reduce_min(float& local_contribution, float& global)
{
    std::vector<float> local_v(1u, local_contribution);
    std::vector<float> global_v(1u, 0);
    this->all_reduce_min(local_v, global_v);
    global = global_v[0];
}
void MpiWrapper::all_reduce_min(double& local_contribution, double& global)
{
    std::vector<double> local_v(1u, local_contribution);
    std::vector<double> global_v(1u, 0);
    this->all_reduce_min(local_v, global_v);
    global = global_v[0];
}

void MpiWrapper::all_reduce_sum(int& local_contribution, int& global)
{
    std::vector<int> local_v(1u, local_contribution);
    std::vector<int> global_v(1u, 0);
    this->all_reduce_sum(local_v, global_v);
    global = global_v[0];
}
void MpiWrapper::all_reduce_sum(float& local_contribution, float& global)
{
    std::vector<float> local_v(1u, local_contribution);
    std::vector<float> global_v(1u, 0);
    this->all_reduce_sum(local_v, global_v);
    global = global_v[0];
}
void MpiWrapper::all_reduce_sum(double& local_contribution, double& global)
{
    std::vector<double> local_v(1u, local_contribution);
    std::vector<double> global_v(1u, 0);
    this->all_reduce_sum(local_v, global_v);
    global = global_v[0];
}

void MpiWrapper::all_reduce_max(int& local_contribution, int& global)
{
    std::vector<int> local_v(1u, local_contribution);
    std::vector<int> global_v(1u, 0);
    this->all_reduce_max(local_v, global_v);
    global = global_v[0];
}
void MpiWrapper::all_reduce_max(float& local_contribution, float& global)
{
    std::vector<float> local_v(1u, local_contribution);
    std::vector<float> global_v(1u, 0);
    this->all_reduce_max(local_v, global_v);
    global = global_v[0];
}
void MpiWrapper::all_reduce_max(double& local_contribution, double& global)
{
    std::vector<double> local_v(1u, local_contribution);
    std::vector<double> global_v(1u, 0);
    this->all_reduce_max(local_v, global_v);
    global = global_v[0];
}

void MpiWrapper::broadcast(size_t source_rank, int& broadcast_data)
{
    std::vector<int> broadcast_vector(1u, broadcast_data);
    broadcast(source_rank, broadcast_vector);
    broadcast_data = broadcast_vector[0];
}
void MpiWrapper::broadcast(size_t source_rank, float& broadcast_data)
{
    std::vector<float> broadcast_vector(1u, broadcast_data);
    broadcast(source_rank, broadcast_vector);
    broadcast_data = broadcast_vector[0];
}
void MpiWrapper::broadcast(size_t source_rank, double& broadcast_data)
{
    std::vector<double> broadcast_vector(1u, broadcast_data);
    broadcast(source_rank, broadcast_vector);
    broadcast_data = broadcast_vector[0];
}

void MpiWrapper::send_point_cloud(size_t target_rank, PlatoSubproblemLibrary::PointCloud* points)
{
    // handle null pointer
    if(!points)
    {
        const size_t num_points = 0u;
        this->send(target_rank, (int)num_points);
        return;
    }

    // send number of points
    const size_t num_points = points->get_num_points();
    this->send(target_rank, (int)num_points);

    // send points
    for(size_t point_index = 0u; point_index < num_points; point_index++) {
        Point* point = points->get_point(point_index);
        std::vector<double> point_data;
        point->get_data(point_data);
        this->send(target_rank, (int)point->get_index());
        this->send(target_rank, (int)point_data.size());
        this->send(target_rank, point_data);
    }
}

PlatoSubproblemLibrary::PointCloud* MpiWrapper::receive_point_cloud(size_t source_rank)
{
    // receive number of points
    int int_num_points = 0;
    this->receive(source_rank, int_num_points);
    const size_t num_points = int_num_points;

    // allocate
    PlatoSubproblemLibrary::PointCloud* result = new PlatoSubproblemLibrary::PointCloud;
    std::vector<Point> points(num_points);

    // receive point data
    for(size_t i = 0u; i < num_points; i++)
    {
        int index_of_point = 0;
        this->receive(source_rank, index_of_point);
        int point_dimension = 0;
        this->receive(source_rank, point_dimension);
        std::vector<double> point_data(point_dimension);
        this->receive(source_rank, point_data);
        points[i].set(index_of_point, point_data);
    }

    // fill point data
    result->assign(points);
    return result;
}

void MpiWrapper::receive_to_point_cloud(size_t source_rank, PlatoSubproblemLibrary::PointCloud* points)
{
    // receive number of points
    int num_points = 0;
    this->receive(source_rank, num_points);

    // receive point data
    for(int i = 0; i < num_points; i++)
    {
        int index_of_point = 0;
        this->receive(source_rank, index_of_point);
        int point_dimension = 0;
        this->receive(source_rank, point_dimension);
        std::vector<double> point_data(point_dimension);
        this->receive(source_rank, point_data);
        points->push_back(Point(index_of_point, point_data));
    }
}

void MpiWrapper::all_gather(AxisAlignedBoundingBox& local_portion, std::vector<AxisAlignedBoundingBox>& global_portion)
{
    // allocate
    const size_t mpi_size = get_size();
    const size_t num_floats = 6u;
    std::vector<float> global_float_values(mpi_size * num_floats);

    // set local
    std::vector<float> local_float_values(num_floats);
    local_float_values[0] = local_portion.get_x_min();
    local_float_values[1] = local_portion.get_x_max();
    local_float_values[2] = local_portion.get_y_min();
    local_float_values[3] = local_portion.get_y_max();
    local_float_values[4] = local_portion.get_z_min();
    local_float_values[5] = local_portion.get_z_max();

    // communicate floats
    all_gather(local_float_values, global_float_values);

    // communicate ids
    std::vector<int> global_ids(mpi_size);
    all_gather(local_portion.get_id(), global_ids);

    // copy into global portion
    for(size_t proc = 0u; proc < mpi_size; proc++)
    {
        global_portion[proc].set_x_min(global_float_values[proc * num_floats + 0u]);
        global_portion[proc].set_x_max(global_float_values[proc * num_floats + 1u]);
        global_portion[proc].set_y_min(global_float_values[proc * num_floats + 2u]);
        global_portion[proc].set_y_max(global_float_values[proc * num_floats + 3u]);
        global_portion[proc].set_z_min(global_float_values[proc * num_floats + 4u]);
        global_portion[proc].set_z_max(global_float_values[proc * num_floats + 5u]);
        global_portion[proc].set_id(global_ids[proc]);
    }
}

void MpiWrapper::send(size_t target_rank, const AxisAlignedBoundingBox& box)
{
    const size_t num_floats = 6u;
    std::vector<float> float_values(num_floats);
    float_values[0] = box.get_x_min();
    float_values[1] = box.get_x_max();
    float_values[2] = box.get_y_min();
    float_values[3] = box.get_y_max();
    float_values[4] = box.get_z_min();
    float_values[5] = box.get_z_max();

    // send
    send(target_rank, float_values);
    send(target_rank, box.get_id());
}

void MpiWrapper::receive(size_t source_rank, AxisAlignedBoundingBox& box)
{
    const size_t num_floats = 6u;
    std::vector<float> float_values(num_floats);
    int id = -1;

    // receive
    receive(source_rank, float_values);
    receive(source_rank, id);

    // transfer
    box.set_x_min(float_values[0]);
    box.set_x_max(float_values[1]);
    box.set_y_min(float_values[2]);
    box.set_y_max(float_values[3]);
    box.set_z_min(float_values[4]);
    box.set_z_max(float_values[5]);
    box.set_id(id);
}

}

}
