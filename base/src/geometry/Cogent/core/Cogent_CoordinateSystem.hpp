/*
 * Cogent_CoordinateSystem.hpp
 *
 *  Created on: July 11, 2017
 *      Author: jrobbin
 */

#ifndef COGENT_COORDINATE_SYSTEM_HPP_
#define COGENT_COORDINATE_SYSTEM_HPP_

#include <Teuchos_ParameterList.hpp>

#include "Cogent_Types.hpp"
#include "Cogent_ParameterFunction.hpp"

namespace Cogent {

class CoordinateSystem {
public:
  CoordinateSystem(const Teuchos::ParameterList& params,
                   const std::vector<std::string>& parameterNames);
  CoordinateSystem(const Teuchos::ParameterList& params,
                   const std::vector<std::string>& parameterNames, 
                   const Teuchos::RCP<CoordinateSystem> coordSys);
  ~CoordinateSystem(){}
  void update(const std::vector<RealType>& P);

  template <typename T>
  std::vector<T> localFromGlobal(std::vector<T> X);

  bool operator==(const CoordinateSystem& rhs);

protected:

  void
  parse(const std::vector<std::string>& parameterNames, 
        const Teuchos::ParameterList& params);

  std::vector<std::string> m_stringP;
  std::vector<Teuchos::RCP<Cogent::ParameterFunction>> m_P;

  std::vector<std::vector<std::string>> m_stringBasis;
  std::vector<std::vector<Teuchos::RCP<Cogent::ParameterFunction>>> m_Basis;

  Teuchos::RCP<CoordinateSystem> m_modelCoordinateSystem;

  const int c_numDim;

};

template <typename T>
std::vector<T> 
CoordinateSystem::
localFromGlobal(std::vector<T> X)
{
  if(!(m_modelCoordinateSystem.is_null()))
    X = m_modelCoordinateSystem->localFromGlobal(X);

  std::vector<T> Xret(c_numDim);

  // subtract origin
  for(int iDim=0; iDim<c_numDim; iDim++)
    X[iDim] -= m_P[iDim]->getValue<T>();

  // times basis transpose
  for(int iDim=0; iDim<c_numDim; iDim++){
    Xret[iDim] = T(0.0);
    for(int jDim=0; jDim<c_numDim; jDim++){
      Xret[iDim] += (m_Basis[jDim][iDim]->getValue<T>())*X[jDim];
    }
  }

  return Xret;
}

} /* namespace Cogent */

#endif /* COGENT_COORDINATE_SYSTEM_HPP_ */
