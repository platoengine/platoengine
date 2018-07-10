#include "PSL_CrossEntropyLoss.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_FreeHelpers.hpp"

#include <cstddef>
#include <cassert>
#include <cmath>
#include <math.h>

namespace PlatoSubproblemLibrary
{

CrossEntropyLoss::CrossEntropyLoss() :
        DataLossFunction(data_loss_function_t::data_loss_function_t::cross_entropy_loss)
{
}
CrossEntropyLoss::~CrossEntropyLoss()
{
}

double CrossEntropyLoss::compute_loss(const std::vector<double>& computed,
                                      const std::vector<double>& true_,
                                      std::vector<double>& gradient)
{
    const size_t length = computed.size();
    assert(true_.size() == length);
    gradient.resize(length);

    // compute sums
    const double max_computed = max(computed);
    double t_sum = 0.;
    double o_sum = 0.;
    for(size_t i = 0u; i < length; i++)
    {
        const double this_exp = exp(computed[i] - max_computed);
        t_sum += true_[i] * this_exp;
        o_sum += this_exp;
    }

    // compute loss
    for(size_t i = 0u; i < length; i++)
    {
        const double this_exp = exp(computed[i] - max_computed);
        gradient[i] = -(true_[i]*this_exp/t_sum) + (this_exp/o_sum);
    }

    return -log(t_sum) + log(o_sum);
}

}
