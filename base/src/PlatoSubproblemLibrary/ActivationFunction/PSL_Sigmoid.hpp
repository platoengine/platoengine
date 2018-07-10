#pragma once

#include "PSL_ActivationFunction.hpp"

namespace PlatoSubproblemLibrary
{

class Sigmoid : public ActivationFunction
{
public:
    Sigmoid();
    virtual ~Sigmoid();

    virtual void activate(const double& input, double& output, double& output_gradient);

protected:

};

}
