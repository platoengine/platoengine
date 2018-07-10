#include "PSL_SquaredErrorLoss.hpp"

#include "PSL_ParameterDataEnums.hpp"

#include <cstddef>
#include <cassert>

namespace PlatoSubproblemLibrary
{

SquaredErrorLoss::SquaredErrorLoss() :
        DataLossFunction(data_loss_function_t::data_loss_function_t::squared_error_loss)
{
}
SquaredErrorLoss::~SquaredErrorLoss()
{
}

double SquaredErrorLoss::compute_loss(const std::vector<double>& computed,
                                      const std::vector<double>& true_,
                                      std::vector<double>& gradient)
{
    const size_t length = computed.size();
    assert(true_.size() == length);
    gradient.resize(length);

    // compute result = 0.5 * norm(computed-true_,2)^2
    double result = 0.;
    for(size_t i = 0u; i < length; i++)
    {
        gradient[i] = computed[i] - true_[i];
        result += gradient[i] * gradient[i];
    }

    return result / 2.;
}

}
