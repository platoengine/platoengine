#pragma once

#include "PSL_VectorKernel.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

class PureVectorKernel : public VectorKernel
{
public:
    PureVectorKernel(int projection_power);
    virtual ~PureVectorKernel();

    virtual int projection_length(const int& source_length);
    virtual void project(const std::vector<double>& source, std::vector<double>& destination);

protected:
    int m_projection_power;

};

}

