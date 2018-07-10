#include "Plato_InputData.hpp"

namespace Plato {
InputData::InputData() : m_name("Input Data") { init(); }

InputData::InputData(std::string name) : m_name(name) { init(); }

std::string InputData::name()  const { return m_name; }
bool        InputData::empty() const { return m_parameters->empty(); }

void InputData::init()
  {
    m_parameters = std::make_shared<std::multimap<std::string, Plato::any>>(std::multimap<std::string, Plato::any>());
  }

#ifndef USE_CXX_14
  bool operator==(const InputData& l, const InputData& r)
  {
    return std::tie(l.m_name,l.m_parameters) == std::tie(r.m_name, r.m_parameters);
  }
  std::ostream& operator<<(std::ostream& os, const InputData& v)
  {
    os << v.m_name;
    return os;
  }
#endif


}
