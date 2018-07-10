#include "PSL_DataLossFunction.hpp"

#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{

DataLossFunction::DataLossFunction(data_loss_function_t::data_loss_function_t type) :
        m_type(type)
{
}
DataLossFunction::~DataLossFunction()
{
}

}
