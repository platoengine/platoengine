#pragma once

#include "PSL_DiscreteObjective.hpp"
#include "PSL_FindDataset.hpp"

#include <vector>
#include <string>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class ParameterData;
class FindDataset;

class CrossValidationErrorDiscreteObjective : public DiscreteObjective
{
public:
    CrossValidationErrorDiscreteObjective(AbstractAuthority* authority, const std::vector<datasets_t::datasets_t>& dataset);
    virtual ~CrossValidationErrorDiscreteObjective();

    virtual void get_domain(std::vector<double>& inclusive_lower,
                            std::vector<double>& inclusive_upper,
                            std::vector<int>& num_values) = 0;
    virtual double evaluate(const std::vector<double>& parameters);

protected:
    virtual void get_parameters(const std::vector<double>& parameters, ParameterData* parameter_data) = 0;
    virtual double compute_result(const std::vector<double>& results) = 0;

    AbstractAuthority* m_authority;
    std::vector<std::string> m_archive_filename;
    FindDataset m_finder;

};

}
