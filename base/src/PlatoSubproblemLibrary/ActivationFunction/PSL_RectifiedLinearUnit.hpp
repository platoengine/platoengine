#pragma once

#include "PSL_ActivationFunction.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

class RectifiedLinearUnit : public ActivationFunction
{
public:
    RectifiedLinearUnit();
    virtual ~RectifiedLinearUnit();

    virtual void activate(const double& input, double& output, double& output_gradient);

protected:

};

}
