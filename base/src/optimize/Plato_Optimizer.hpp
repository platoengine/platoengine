/*
 * Plato_Optimizer.hpp
 *
 *  Created on: April 19, 2017
 *
 */

#ifndef SRC_OPTIMIZER_HPP_
#define SRC_OPTIMIZER_HPP_

#include "Plato_Convergence.hpp"

namespace Plato {

  class Interface;

  using std::string;
  using std::vector;

/**
 * This class provides an interface between PlatoEngine and the 
 * hosted codes. 
 */

//!  Base class defining optimizer interface
/*!
*/
class Optimizer {
public:
  Optimizer(Interface* interface);
  virtual ~Optimizer();

  /** this function must not throw an exception.  use m_interface->registerException(). **/
  virtual void initialize()=0;

  /** this function must not throw an exception.  use m_interface->registerException(). **/
  virtual void optimize()=0;

  virtual void finalize();

protected:
  
  double computeNorm(const double* p, int n);
  double computeDiffNorm(const double* p, const double* p_last, int n, bool printResult);

  Interface* m_interface;

  struct Target {
    Target(Plato::InputData& node);
    Target(){}
    string valueName;
    double currentValue=0.0;
    double previousValue=0.0;
    double targetValue=0.0;
    string referenceValueName;
    double referenceValue=1.0;
    string  gradientName;
    double* gradient=nullptr;
    double tolerance=1e-3;
    bool isLinear=true;
    bool isEquality=true;
    bool isRelative=true;
  };

  vector<Target> m_constraints;
  Target m_objective;

  ConvergenceTest* convergenceChecker;
 
  double m_upperBound, m_lowerBound;

  std::string m_outputStage;
  std::string m_optimizationDOFsName;
  std::string m_initializationStage;
};


} // end Plato namespace

#endif
