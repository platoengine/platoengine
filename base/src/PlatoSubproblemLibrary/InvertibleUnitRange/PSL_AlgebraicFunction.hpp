#pragma once

#include "PSL_InvertibleUnitRange.hpp"

namespace PlatoSubproblemLibrary
{

class AlgebraicFunction : public InvertibleUnitRange
{
public:
    AlgebraicFunction(const double& negative_scale, const double& positive_scale);
    AlgebraicFunction(const double& scale);
    virtual ~AlgebraicFunction();

    virtual double evaluate(const double& x);
    virtual double invert(const double& y);

protected:

};

}

