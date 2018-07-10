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

    virtual GlobalUtilities* get_utilities();

    virtual bool is_root();
    virtual size_t get_rank() = 0;
    virtual size_t get_size() = 0;

    virtual double get_time() = 0;

    virtual void send(size_t target_rank, std::vector<int>& send_vector) = 0;
    virtual void send(size_t target_rank, std::vector<float>& send_vector) = 0;
    virtual void send(size_t target_rank, std::vector<double>& send_vector) = 0;
    virtual void send(size_t target_rank, int send_data);
    virtual void send(size_t target_rank, float send_data);
    virtual void send(size_t target_rank, double send_data);

    virtual void receive(size_t source_rank, std::vector<int>& recv_vector) = 0;
    virtual void receive(size_t source_rank, std::vector<float>& recv_vector) = 0;
    virtual void receive(size_t source_rank, std::vector<double>& recv_vector) = 0;
    virtual void receive(size_t source_rank, int& recv_data);
    virtual void receive(size_t source_rank, float& recv_data);
    virtual void receive(size_t source_rank, double& recv_data);

    virtual void all_gather(std::vector<int>& local_portion, std::vector<int>& global_portion) = 0;
    virtual void all_gather(std::vector<float>& local_portion, std::vector<float>& global_portion) = 0;
    virtual void all_gather(std::vector<double>& local_portion, std::vector<double>& global_portion) = 0;
    virtual void all_gather(int local_portion, std::vector<int>& global_portion);
    virtual void all_gather(float local_portion, std::vector<float>& global_portion);
    virtual void all_gather(double local_portion, std::vector<double>& global_portion);

    virtual void all_reduce_min(std::vector<int>& local_contribution, std::vector<int>& global) = 0;
    virtual void all_reduce_min(std::vector<float>& local_contribution, std::vector<float>& global) = 0;
    virtual void all_reduce_min(std::vector<double>& local_contribution, std::vector<double>& global) = 0;
    virtual void all_reduce_min(int& local_contribution, int& global);
    virtual void all_reduce_min(float& local_contribution, float& global);
    virtual void all_reduce_min(double& local_contribution, double& global);

    virtual void all_reduce_sum(std::vector<int>& local_contribution, std::vector<int>& global) = 0;
    virtual void all_reduce_sum(std::vector<float>& local_contribution, std::vector<float>& global) = 0;
    virtual void all_reduce_sum(std::vector<double>& local_contribution, std::vector<double>& global) = 0;
    virtual void all_reduce_sum(int& local_contribution, int& global);
    virtual void all_reduce_sum(float& local_contribution, float& global);
    virtual void all_reduce_sum(double& local_contribution, double& global);

    virtual void all_reduce_max(std::vector<int>& local_contribution, std::vector<int>& global) = 0;
    virtual void all_reduce_max(std::vector<float>& local_contribution, std::vector<float>& global) = 0;
    virtual void all_reduce_max(std::vector<double>& local_contribution, std::vector<double>& global) = 0;
    virtual void all_reduce_max(int& local_contribution, int& global);
    virtual void all_reduce_max(float& local_contribution, float& global);
    virtual void all_reduce_max(double& local_contribution, double& global);

    virtual void broadcast(size_t source_rank, std::vector<int>& broadcast_vector) = 0;
    virtual void broadcast(size_t source_rank, std::vector<float>& broadcast_vector) = 0;
    virtual void broadcast(size_t source_rank, std::vector<double>& broadcast_vector) = 0;
    virtual void broadcast(size_t source_rank, int& broadcast_data);
    virtual void broadcast(size_t source_rank, float& broadcast_data);
    virtual void broadcast(size_t source_rank, double& broadcast_data);

    virtual void send_point_cloud(size_t target_rank, PlatoSubproblemLibrary::PointCloud* points);
    virtual PlatoSubproblemLibrary::PointCloud* receive_point_cloud(size_t source_rank);
    virtual void receive_to_point_cloud(size_t source_rank, PlatoSubproblemLibrary::PointCloud* points);
    virtual void all_gather(AxisAlignedBoundingBox& local_portion, std::vector<AxisAlignedBoundingBox>& global_portion);
    virtual void send(size_t target_rank, const AxisAlignedBoundingBox& box);
    virtual void receive(size_t source_rank, AxisAlignedBoundingBox& box);

protected:
    GlobalUtilities* m_utilities;

};

}

}
