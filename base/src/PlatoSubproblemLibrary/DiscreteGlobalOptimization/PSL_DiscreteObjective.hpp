#pragma once

#include <vector>

namespace PlatoSubproblemLibrary
{

class DiscreteObjective
{
public:
    DiscreteObjective();
    virtual ~DiscreteObjective();

    virtual void get_domain(std::vector<double>& inclusive_lower,
                            std::vector<double>& inclusive_upper,
                            std::vector<int>& num_values) = 0;
    virtual double evaluate(const std::vector<double>& parameters) = 0;

protected:

};

}
