#pragma once

#include "PSL_Implementation_NeuralNetwork_StructParameterData.hpp"
#include "PSL_Abstract_ParameterDataBuilder.hpp"

namespace PlatoSubproblemLibrary
{
class ParameterData;

namespace example
{
struct NeuralNetwork_StructParameterData;

class Interface_NeuralNetwork_StructParameterDataBuilder : public AbstractInterface::ParameterDataBuilder
{
public:

    Interface_NeuralNetwork_StructParameterDataBuilder();
    virtual ~Interface_NeuralNetwork_StructParameterDataBuilder();

    void set_data(example::NeuralNetwork_StructParameterData* data);
    virtual ParameterData* build();

private:

    example::NeuralNetwork_StructParameterData* m_data;
};

}
}
