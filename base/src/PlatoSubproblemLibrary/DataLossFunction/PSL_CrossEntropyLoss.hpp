#pragma once

#include "PSL_DataLossFunction.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

class CrossEntropyLoss : public DataLossFunction
{
public:
    CrossEntropyLoss();
    virtual ~CrossEntropyLoss();

    virtual double compute_loss(const std::vector<double>& computed,
                                const std::vector<double>& true_,
                                std::vector<double>& gradient);

protected:

};

}
