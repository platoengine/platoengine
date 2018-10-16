/*
 * Cogent_ParameterFunction.hpp
 *
 *  Created on: Nov 22, 2016
 *      Author: jrobbin
 */

#ifndef COGENT_PARAMETERFUNCTION_HPP_
#define COGENT_PARAMETERFUNCTION_HPP_

#include <RTC_FunctionRTC.hh>

#include "Cogent_Types.hpp"

namespace Cogent {

class ParameterFunction {
public:
    ParameterFunction(std::vector<std::string> a_Xvals, std::string a_func, RealType epsilon=1e-6);
    virtual ~ParameterFunction();
 
    void update(std::vector<RealType> P);

    template <typename T> T getValue();

    friend std::ostream& operator<<(std::ostream& out, const Cogent::ParameterFunction& pf);

private:
    void getDeriv(std::vector<RealType> P, RealType* dvdp);
    RealType getValue(std::vector<RealType> P);
    PG_RuntimeCompiler::Function m_function;

    std::vector<std::string> m_xVarNames;
    std::string m_strFunc;
    const RealType c_epsilon;
    const int c_numVars;

    RealType m_currentValue_RealType;
    std::vector<RealType> m_currentDeriv_RealType;

    DFadType m_currentValue_DFadType;
  
};
} /* namespace Cogent */

#endif /* COGENT_PARAMETERFUNCTION_HPP_ */
