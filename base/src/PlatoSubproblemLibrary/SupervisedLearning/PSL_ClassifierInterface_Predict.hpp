#pragma once

#include "PSL_ClassifierInterface.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class ParameterData;

class ClassifierInterface_Predict : public ClassifierInterface
{
public:
    ClassifierInterface_Predict(AbstractAuthority* authority);
    virtual ~ClassifierInterface_Predict();

    virtual void initialize(ParameterData* parameters,
                            int num_scalars,
                            const std::vector<int>& enum_sizes,
                            int output_enum_size);

    virtual int predict_classification(const std::vector<double>& input_scalars, const std::vector<int>& input_enums);

    virtual void store_accurate_classification(const std::vector<double>& input_scalars,
                                               const std::vector<int>& input_enums,
                                               int accurate_output);

protected:
};

}
