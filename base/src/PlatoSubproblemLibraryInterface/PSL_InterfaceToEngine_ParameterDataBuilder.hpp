#pragma once

#include "PSL_Abstract_ParameterDataBuilder.hpp"
#include "Plato_InputData.hpp"

namespace PlatoSubproblemLibrary
{
class ParameterData;
}

namespace Plato
{

class InterfaceToEngine_ParameterDataBuilder : public PlatoSubproblemLibrary::AbstractInterface::ParameterDataBuilder
{
public:
    InterfaceToEngine_ParameterDataBuilder(InputData interface);
    virtual ~InterfaceToEngine_ParameterDataBuilder();

    virtual PlatoSubproblemLibrary::ParameterData* build();

private:
    Plato::InputData m_inputData;

};

}
