#pragma once

#include "PSL_FreeHelpers.hpp"
#include <map>
#include <vector>
#include <string>

namespace PlatoSubproblemLibrary
{

#define PSL_PARAMETER_DATA_VECTOR_ENUM(_class_tokens_t, _enum_tokens_t, _token) \
    void set_##_token(const std::vector<_enum_tokens_t::_enum_tokens_t>& _input) \
    { \
        transfer_vector_types(m_vector_of_ints[_class_tokens_t::_token], _input); \
    } \
    void set_##_token(const _enum_tokens_t::_enum_tokens_t& _input) \
    { \
        set_##_token(std::vector<_enum_tokens_t::_enum_tokens_t>(1, _input)); \
    } \
    std::vector<_enum_tokens_t::_enum_tokens_t> get_##_token() \
    { \
        std::vector<_enum_tokens_t::_enum_tokens_t> result; \
        transfer_vector_types(result, m_vector_of_ints.find(_class_tokens_t::_token)->second); \
        return result; \
    } \
    bool didUserInput_##_token() \
    { \
        return (m_vector_of_ints.find(_class_tokens_t::_token) != m_vector_of_ints.end()); \
    }

#define PSL_PARAMETER_DATA_ENUM(_class_tokens_t, _enum_tokens_t, _token) \
    void set_##_token(_enum_tokens_t::_enum_tokens_t _input) \
    { \
        m_int[_class_tokens_t::_token] = _input; \
    } \
    _enum_tokens_t::_enum_tokens_t get_##_token() \
    { \
        return (_enum_tokens_t::_enum_tokens_t) m_int.find(_class_tokens_t::_token)->second; \
    } \
    bool didUserInput_##_token() \
    { \
        return (m_int.find(_class_tokens_t::_token) != m_int.end()); \
    }

    // plain old data of int, double, or bool
#define PSL_PARAMETER_DATA_POD_LONG(_class_tokens_t, _shorthand_type, _formal_type, _token) \
    void set_##_token(_formal_type _input) \
    { \
        m_##_shorthand_type[_class_tokens_t::_token] = _input; \
    } \
    _formal_type get_##_token() \
    { \
        return m_##_shorthand_type.find(_class_tokens_t::_token)->second; \
    } \
    bool didUserInput_##_token() \
    { \
        return (m_##_shorthand_type.find(_class_tokens_t::_token) != m_##_shorthand_type.end()); \
    }
#define PSL_PARAMETER_DATA_POD(__class_tokens_t, __type, __token) \
    PSL_PARAMETER_DATA_POD_LONG(__class_tokens_t, __type, __type, __token)

    // vector of int or double types
#define PSL_PARAMETER_DATA_VECTOR(_class_tokens_t, _type, _token) \
    void set_##_token(const std::vector<_type>& _input) \
    { \
        m_vector_of_##_type##s[_class_tokens_t::_token] = _input; \
    } \
    std::vector<_type> get_##_token() \
    { \
        std::vector<_type> result = m_vector_of_##_type##s.find(_class_tokens_t::_token)->second; \
        return result; \
    } \
    bool didUserInput_##_token() \
    { \
        return (m_vector_of_##_type##s.find(_class_tokens_t::_token) != m_vector_of_##_type##s.end()); \
    }

class Abstract_ParameterData
{
public:
    Abstract_ParameterData();
    virtual ~Abstract_ParameterData();

    template<typename _type>
    void set_token(int _key, const _type& _input);

protected:

    std::map<int, bool> m_bool;
    std::map<int, int> m_int;
    std::map<int, double> m_double;
    std::map<int, std::string> m_string;
    std::map<int, std::vector<int> > m_vector_of_ints;
    std::map<int, std::vector<double> > m_vector_of_doubles;
};

}
