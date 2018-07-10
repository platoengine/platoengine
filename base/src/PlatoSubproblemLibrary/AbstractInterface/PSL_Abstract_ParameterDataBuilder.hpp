#pragma once

#define PSL_STRUCT_TO_PARAMETER_DATA_BUILDER(_input_data, _output_data, _field) \
    if(_input_data->has_##_field) { \
        _output_data->set_##_field(_input_data->_field); \
    }

namespace PlatoSubproblemLibrary
{
class ParameterData;

namespace AbstractInterface
{

class ParameterDataBuilder
{
public:
    ParameterDataBuilder();
    virtual ~ParameterDataBuilder();

    virtual ParameterData* build() = 0;

private:
};

}

}
