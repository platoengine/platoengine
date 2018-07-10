#pragma once

#include "PSL_VectorKernel.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

class CrossVectorKernel : public VectorKernel
{
public:
    CrossVectorKernel(int projection_power);
    virtual ~CrossVectorKernel();

    virtual int projection_length(const int& source_length);
    virtual void project(const std::vector<double>& source, std::vector<double>& destination);

protected:
    void recursive_for_loop(const int& num_for_loops,
                            const int& begin_index,
                            const std::vector<double>& source,
                            const double& input_partial_product,
                            std::vector<double>& destination,
                            int& counter);

    int m_projection_power;

};

}

