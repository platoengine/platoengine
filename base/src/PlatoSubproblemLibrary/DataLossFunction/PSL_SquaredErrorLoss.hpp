#pragma once

#include "PSL_DataLossFunction.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

class SquaredErrorLoss : public DataLossFunction
{
public:
    SquaredErrorLoss();
    virtual ~SquaredErrorLoss();

    virtual double compute_loss(const std::vector<double>& computed,
                                const std::vector<double>& true_,
                                std::vector<double>& gradient);

protected:

};

}
