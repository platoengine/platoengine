/*
 * Plato_PenaltyModel.cpp
 *
 *  Created on: April 19, 2017
 *
 *
 *  NOTE: This class is a placeholder till the TopOpt source is pulled in.
 *
 */

#include "Plato_PenaltyModel.hpp"
#include "Plato_Parser.hpp"
#include <math.h>

namespace Plato {

/******************************************************************************/
SIMP::SIMP(const Plato::InputData& input)
/******************************************************************************/
{
  m_penaltyExponent = Get::Double(input,"PenaltyExponent");
  m_minimumValue    = Get::Double(input,"MinimumValue");
}

/******************************************************************************/
double SIMP::eval(double x)
/******************************************************************************/
{
  if (x != 0.0)
    return m_minimumValue+(1.0-m_minimumValue)*pow(x,m_penaltyExponent);
  else
    return m_minimumValue;
}

/******************************************************************************/
double SIMP::grad(double x)
/******************************************************************************/
{
  if (x != 0.0)
    return m_minimumValue+
           (1.0-m_minimumValue)*m_penaltyExponent*pow(x,m_penaltyExponent-1.0);
  else
    return m_minimumValue;

}

/******************************************************************************/
PenaltyModel* PenaltyModelFactory::create(Plato::InputData& aInputData)
/******************************************************************************/
{
  std::string strModel = Get::String(aInputData,"PenaltyModel");

  auto tInputData = Get::InputData(aInputData, strModel);

  PenaltyModel* pModel = nullptr;

  if(strModel == "SIMP"){
    pModel = new SIMP(tInputData);
  }

  return pModel;

}

} /* namespace Plato */
