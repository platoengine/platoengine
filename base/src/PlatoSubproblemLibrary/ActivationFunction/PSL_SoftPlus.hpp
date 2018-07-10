#pragma once

#include "PSL_ActivationFunction.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

class SoftPlus : public ActivationFunction
{
public:
    SoftPlus();
    virtual ~SoftPlus();

    virtual void activate(const double& input, double& output, double& output_gradient);

protected:

};

}
