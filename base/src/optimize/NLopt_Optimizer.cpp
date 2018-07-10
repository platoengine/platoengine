/*
 * NLopt_Optimizer.cpp
 *
 *  Created on: April 19, 2017
 *
 */

#ifdef ENABLE_NLOPT

#include <algorithm>
#include <Teuchos_TestForException.hpp>

#include "communicator.hpp"
#include "NLopt_Optimizer.hpp"
#include "Plato_Interface.hpp"

extern Communicator WorldComm;

namespace Plato {

/**********************************************************************/
NLopt_Optimizer::NLopt_Optimizer(Interface* interface) :
Optimizer(interface)
/**********************************************************************/
{ 
   auto tInputData = interface->getInputData();
   if( tInputData.size<Plato::InputData>("Optimizer") )
   {
      auto tOptNode = tInputData.get<Plato::InputData>("Optimizer");
      if( tOptNode.size<Plato::InputData>("NLopt") 
      {
         auto tNLoptNode = tOptNode.get<Plato::InputData>("NLopt");
         m_optMethod = Parse::getString(NLoptNode, "Method");
      }
   }
}

/******************************************************************************/
NLopt_Optimizer::~NLopt_Optimizer()
/******************************************************************************/
{
}

/******************************************************************************/
void
NLopt_Optimizer::initialize()
/******************************************************************************/
{
  TEUCHOS_TEST_FOR_EXCEPTION (
    m_interface == NULL, Teuchos::Exceptions::InvalidParameter, std::endl
    << "Error! Optimizer requires valid Solver Interface" << std::endl);

  TEUCHOS_TEST_FOR_EXCEPTION (
    (WorldComm.GetSize() != 1) && (WorldComm.getPID()==0), 
    Teuchos::Exceptions::InvalidParameter, std::endl
    << "Error! NLopt package doesn't work in parallel.  Use OC package." << std::endl);
  TEUCHOS_TEST_FOR_EXCEPT ( (WorldComm.GetSize() != 1) );
  
  m_numOptDofs = m_interface->size(m_optimizationDOFsName);
 
  if( m_optMethod == "MMA" )
    opt = nlopt_create(NLOPT_LD_MMA, m_numOptDofs);
  else
  if( m_optMethod == "CCSA" )
    opt = nlopt_create(NLOPT_LD_CCSAQ, m_numOptDofs);
  else
  if( m_optMethod == "SLSQP" )
    opt = nlopt_create(NLOPT_LD_SLSQP, m_numOptDofs);
  else
    TEUCHOS_TEST_FOR_EXCEPTION(
      true, Teuchos::Exceptions::InvalidParameter, std::endl 
      << "Error!  Optimization method: " << m_optMethod << " Unknown!" << std::endl 
      << "Valid options are (MMA, CCSA, SLSQP)" << std::endl);

  
  // set bounds
  m_lowerBounds = std::vector<double>(m_numOptDofs,m_lowerBound);
  m_upperBounds = std::vector<double>(m_numOptDofs,m_upperBound);
  nlopt_set_lower_bounds(opt, m_lowerBounds.data());
  nlopt_set_upper_bounds(opt, m_upperBounds.data());
//  nlopt_set_lower_bounds(opt, &m_lowerBound);
// nlopt_set_upper_bounds(opt, &m_upperBound);

  // set objective function
  nlopt_set_min_objective(opt, this->evaluate, this);
  
  // set stop criteria
  nlopt_set_xtol_rel(opt, 1e-9);  // don't converge based on this.  I.e., use convergenceChecker.
  nlopt_set_maxeval(opt, 10);  // TODO

  p      = new double[m_numOptDofs];
  p_last = new double[m_numOptDofs];
  x_ref  = new double[m_numOptDofs];

  std::fill_n(p,      m_numOptDofs, 0.0);
  std::fill_n(p_last, m_numOptDofs, 0.0);
  std::fill_n(x_ref,  m_numOptDofs, 0.0);

  m_objective.gradient      = new double[m_numOptDofs];
  m_constraints[0].gradient = new double[m_numOptDofs];

  std::fill_n(m_objective.gradient,      m_numOptDofs, 0.0);
  std::fill_n(m_constraints[0].gradient, m_numOptDofs, 0.0);

  Teuchos::ParameterList initArgs;
  initArgs.set(m_optimizationDOFsName, p);
  m_interface->compute(m_initializationStage, initArgs);

  Target& con = m_constraints[0];
  double tolerance = con.tolerance;
  if( con.isRelative ){
    Teuchos::ParameterList args;
    args.set(con.referenceValueName, &con.referenceValue);
    m_interface->compute(con.referenceValueName, args);
    tolerance *= con.referenceValue;
  }
  nlopt_add_inequality_constraint(opt, this->constraint, this, tolerance);
}

#define ATO_XTOL_REACHED 104

/******************************************************************************/
void
NLopt_Optimizer::optimize()
/******************************************************************************/
{

  double minf;
  int errorcode = nlopt_optimize(opt, p, &minf);

  if( errorcode < 0 ){
    TEUCHOS_TEST_FOR_EXCEPTION(
      true, Teuchos::Exceptions::InvalidParameter, std::endl 
      << "Error!  Optimization failed with errorcode " << errorcode << std::endl);
  }

  return;
}

/******************************************************************************/
double 
NLopt_Optimizer::evaluate_backend( unsigned int n, const double* x, double* grad )
/******************************************************************************/
{
   
  std::memcpy((void*)p, (void*)x, m_numOptDofs*sizeof(double));
  bool changed = isChanged(p);

  if( changed ){
    std::memcpy((void*)p_last, (void*)p, m_numOptDofs*sizeof(double));
    m_objective.previousValue = m_objective.currentValue;

    std::vector<std::string> operations;

    Teuchos::ParameterList args;
    args.set(m_optimizationDOFsName, p);
    args.set(m_objective.valueName, &m_objective.currentValue);
    args.set(m_objective.gradientName, m_objective.gradient);
    operations.push_back(m_objective.valueName);

    // if any of the constraints are nonlinear, update them at the same time
    // as the objective.  What if the objective is linear?
    Target& con = m_constraints[0];
//    if(con.isLinear == false){
    args.set(con.valueName, &con.currentValue);
    args.set(con.gradientName, con.gradient);
    operations.push_back(con.valueName);
 //   }

    operations.push_back(m_outputStage);
  
    m_interface->compute(operations, args);
  }

  std::memcpy((void*)grad, (void*)m_objective.gradient, m_numOptDofs*sizeof(double));

  if(WorldComm.getPID()==0){
    std::cout << "**************************************************************" << std::endl;
    std::cout << "       objective: " << m_objective.currentValue << std::endl;
    std::cout << "      constraint: " << m_constraints[0].currentValue << std::endl;
    std::cout << "**************************************************************" << std::endl;
  }

  return m_objective.currentValue;
}

/******************************************************************************/
double 
NLopt_Optimizer::evaluate( unsigned int n, const double* x,
                           double* grad, void* data)
/******************************************************************************/
{
  NLopt_Optimizer* NLopt = reinterpret_cast<NLopt_Optimizer*>(data);
  return NLopt->evaluate_backend(n, x, grad);
}

/******************************************************************************/
double 
NLopt_Optimizer::constraint_backend( unsigned int n, const double* x, double* grad )
/******************************************************************************/
{
  std::memcpy((void*)p, (void*)x, m_numOptDofs*sizeof(double));
  bool changed = isChanged(p);

  Target& con = m_constraints[0];
  if( changed ){
    Teuchos::ParameterList args;
    args.set(m_optimizationDOFsName, p);
    args.set(con.valueName, &con.currentValue);
    args.set(con.gradientName, con.gradient);
    m_interface->compute(con.valueName, args);
  }

  std::memcpy((void*)grad, (void*)con.gradient, m_numOptDofs*sizeof(double));

  double targetValue = con.targetValue;
  if(m_constraints[0].isRelative) targetValue *= con.referenceValue;
  if(WorldComm.getPID()==0){
    std::cout << "************************************************************************" << std::endl;
    std::cout << "  Optimizer:  computed constraint is: " << con.currentValue << std::endl;
    std::cout << "  Optimizer:    target constraint is: " << targetValue << std::endl;
    std::cout << "************************************************************************" << std::endl;
  }

  return con.currentValue - targetValue;
}

/******************************************************************************/
double 
NLopt_Optimizer::constraint( unsigned int n, const double* x,
                             double* grad, void* data)
/******************************************************************************/
{
  NLopt_Optimizer* NLopt = reinterpret_cast<NLopt_Optimizer*>(data);
  return NLopt->constraint_backend(n, x, grad);
}

/******************************************************************************/
bool
NLopt_Optimizer::isChanged(const double* x)
/******************************************************************************/
{
  for(int i=0; i<m_numOptDofs; i++){
    if( x[i] != x_ref[i] ){
      std::memcpy((void*)x_ref, (void*)x, m_numOptDofs*sizeof(double));
      return true;
    }
  }
  return false;
}

/******************************************************************************/
void NLopt_Optimizer::finalize() { Optimizer::finalize(); }
/******************************************************************************/

}
#endif
