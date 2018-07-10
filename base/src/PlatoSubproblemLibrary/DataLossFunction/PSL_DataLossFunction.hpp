#pragma once

#include "PSL_ParameterDataEnums.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

class DataLossFunction
{
public:
    DataLossFunction(data_loss_function_t::data_loss_function_t type);
    virtual ~DataLossFunction();

    virtual double compute_loss(const std::vector<double>& computed,
                              const std::vector<double>& true_,
                              std::vector<double>& gradient) = 0;

protected:
    data_loss_function_t::data_loss_function_t m_type;

};

}
