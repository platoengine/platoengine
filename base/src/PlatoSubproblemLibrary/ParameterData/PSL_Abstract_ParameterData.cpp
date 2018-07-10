#include "PSL_Abstract_ParameterData.hpp"

#include <map>
#include <vector>

namespace PlatoSubproblemLibrary
{

Abstract_ParameterData::Abstract_ParameterData()
{
}

Abstract_ParameterData::~Abstract_ParameterData()
{
}

template<>
void Abstract_ParameterData::set_token<bool>(int _key, const bool& _input)
{
    m_bool[_key] = _input;
}

template<>
void Abstract_ParameterData::set_token<int>(int _key, const int& _input)
{
    m_int[_key] = _input;
}

template<>
void Abstract_ParameterData::set_token<double>(int _key, const double& _input)
{
    m_double[_key] = _input;
}

template<>
void Abstract_ParameterData::set_token<std::vector<int> >(int _key, const std::vector<int>& _input)
{
    m_vector_of_ints[_key] = _input;
}

template<>
void Abstract_ParameterData::set_token<std::vector<double> >(int _key, const std::vector<double>& _input)
{
    m_vector_of_doubles[_key] = _input;
}

}
