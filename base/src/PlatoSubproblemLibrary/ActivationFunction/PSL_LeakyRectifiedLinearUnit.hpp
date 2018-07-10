#pragma once

#include "PSL_ActivationFunction.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

class LeakyRectifiedLinearUnit : public ActivationFunction
{
public:
    LeakyRectifiedLinearUnit(double leaky_constant);
    virtual ~LeakyRectifiedLinearUnit();

    virtual void activate(const double& input, double& output, double& output_gradient);

protected:
    double m_leaky_constant;

};

}
