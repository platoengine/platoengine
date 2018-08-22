#pragma once

#include "PSL_InvertibleUnitRange.hpp"

namespace PlatoSubproblemLibrary
{

class LogisticFunction : public InvertibleUnitRange
{
public:
    LogisticFunction(const double& negative_scale, const double& positive_scale);
    LogisticFunction(const double& scale);
    virtual ~LogisticFunction();

    virtual double evaluate(const double& x);
    virtual double invert(const double& y);

protected:

};

}

