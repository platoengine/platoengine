/*
 * Plato_PenaltyModel.hpp
 *
 *  Created on: April 19, 2017
 *
 *
 *  NOTE: This class is a placeholder till the TopOpt source is pulled in.
 *
 */

#ifndef SRC_PENALTYMODEL_HPP_
#define SRC_PENALTYMODEL_HPP_

#include "Plato_Parser.hpp"

namespace Plato {

  class PenaltyModel {
    public:
      PenaltyModel(){}
      virtual ~PenaltyModel(){}
      virtual double eval(double x)=0;
      virtual double grad(double x)=0;
  };

  class SIMP : public PenaltyModel {
    public:
      SIMP(const Plato::InputData& input);
      virtual ~SIMP(){}
      virtual double eval(double x);
      virtual double grad(double x);
    private:
      double m_penaltyExponent;
      double m_minimumValue;
  };

  class PenaltyModelFactory {
    public:
      PenaltyModel* create(Plato::InputData& input);
  };

} /* namespace Plato */

#endif /* SRC_PENALTYMODEL_HPP_ */
