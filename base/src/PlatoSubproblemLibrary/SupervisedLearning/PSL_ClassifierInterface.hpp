#pragma once

#include <vector>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class ParameterData;

class ClassifierInterface
{
public:
    ClassifierInterface(AbstractAuthority* authority);
    virtual ~ClassifierInterface();

    virtual void initialize(ParameterData* parameters,
                            int num_scalars,
                            const std::vector<int>& enum_sizes,
                            int output_enum_size) = 0;

    virtual int predict_classification(const std::vector<double>& input_scalars, const std::vector<int>& input_enums) = 0;

    virtual void store_accurate_classification(const std::vector<double>& input_scalars,
                                               const std::vector<int>& input_enums,
                                               int accurate_output) = 0;

protected:
    AbstractAuthority* m_authority;

};

}
