/*
 * OC_Optimizer.cpp
 *
 *  Created on: April 19, 2017
 *
 */

#include <algorithm>
#include <Teuchos_TestForException.hpp>

#include "communicator.hpp"
#include "OC_Optimizer.hpp"
#include "Plato_Interface.hpp"
#include "Plato_Convergence.hpp"

extern Communicator WorldComm;

namespace Plato {

/**********************************************************************/
OC_Optimizer::OC_Optimizer(Interface* interface) :
Optimizer(interface)
/**********************************************************************/
{ 

  zeroSet();

  auto tInputData = interface->getInputData(); 
  if( tInputData.size<Plato::InputData>("Optimizer") )
  {
    auto tOptNode = tInputData.get<Plato::InputData>("Optimizer");
    if( tOptNode.size<Plato::InputData>("OC") )
    {
      auto tOCNode = tOptNode.get<Plato::InputData>("OC");
      m_moveLimit         = Get::Double(tOCNode,"MoveLimiter");
      m_stabExponent      = Get::Double(tOCNode,"StabilizationParameter");
      m_useNewtonSearch   = Get::Bool(tOCNode,"UseNewtonSearch");
      m_constraintMaxIter = Get::Int(tOCNode,"ConstraintMaxIterations");
    } else 
    TEUCHOS_TEST_FOR_EXCEPTION(
      true, Teuchos::Exceptions::InvalidParameter, std::endl 
      << "Error! Missing 'OC' input block." << std::endl);
  }
}

/******************************************************************************/
void 
OC_Optimizer::zeroSet()
/******************************************************************************/
{
  p = nullptr;
  p_last = nullptr;
};

/******************************************************************************/
OC_Optimizer::~OC_Optimizer()
/******************************************************************************/
{
  if( p      ) delete [] p;
  if( p_last ) delete [] p_last;
  
  if( m_objective.gradient ) delete [] m_objective.gradient;
  for( auto& constraint : m_constraints){
    if( constraint.gradient ) delete [] constraint.gradient;
  }
}

/******************************************************************************/
double
Optimizer::computeNorm(const double* p, int n)
/******************************************************************************/
{
  double norm = 0.0;
  for(int i=0; i<n; i++){
    norm += p[i]*p[i];
  }
  norm = WorldComm.globalSum(norm);
  norm = (norm > 0.0) ? sqrt(norm) : 0.0;
  return norm;
}


/******************************************************************************/
double
Optimizer::computeDiffNorm(const double* p, const double* p_last, int n, bool printResult)
/******************************************************************************/
{
  double norm = 0.0;
  for(int i=0; i<n; i++){
    norm += pow(p[i]-p_last[i],2);
  }
  norm = WorldComm.globalSum(norm);
  norm = (norm > 0.0) ? sqrt(norm) : 0.0;
  if(printResult && WorldComm.getPID()==0){
    std::cout << "************************************************************************" << std::endl;
    std::cout << "  Optimizer:  computed diffnorm is: " << norm << std::endl;
    std::cout << "************************************************************************" << std::endl;
  }
  return norm;
}

/******************************************************************************/
void
OC_Optimizer::initialize()
/******************************************************************************/
{
  TEUCHOS_TEST_FOR_EXCEPTION (
    m_interface == nullptr, Teuchos::Exceptions::InvalidParameter, 
    std::endl << "Error! Optimizer requires valid Plato::Interface" << std::endl);

  m_numOptDofs = m_interface->size(m_optimizationDOFsName);

  p      = new double[m_numOptDofs];
  p_last = new double[m_numOptDofs];

  std::fill_n(p,      m_numOptDofs, 0.0);
  std::fill_n(p_last, m_numOptDofs, 0.0);

  m_objective.gradient      = new double[m_numOptDofs];
  m_constraints[0].gradient = new double[m_numOptDofs];

  std::fill_n(m_objective.gradient,      m_numOptDofs, 0.0);
  std::fill_n(m_constraints[0].gradient, m_numOptDofs, 0.0);

  Teuchos::ParameterList initArgs;
  initArgs.set(m_optimizationDOFsName, p        );
  m_interface->compute(m_initializationStage, initArgs);
  for(int i=0; i<m_numOptDofs; i++) p_last[i] = p[i];

  // compute reference measure
  Target& con = m_constraints[0];
  Teuchos::ParameterList conArgs;
  conArgs.set(con.referenceValueName, &con.referenceValue);
  m_interface->compute(con.referenceValueName, conArgs);

  // compute initial values
  Teuchos::ParameterList args;
  
  std::vector<std::string> operations;
  args.set(m_optimizationDOFsName, p);
  args.set(m_objective.valueName, &m_objective.currentValue);
  args.set(m_objective.gradientName, m_objective.gradient);
  args.set(con.valueName, &con.currentValue);
  args.set(con.gradientName, con.gradient);
  operations.push_back(m_objective.valueName);
  operations.push_back(con.valueName);
  m_interface->compute(operations, args);

  double pnorm = computeNorm(p, m_numOptDofs);
  convergenceChecker->initNorm(m_objective.currentValue, pnorm);

}


/******************************************************************************/
void
OC_Optimizer::optimize()
/******************************************************************************/
{
  int iter=0;
  Target& con = m_constraints[0];
  bool optimization_converged = false;
  while(!optimization_converged) {

    m_objective.previousValue = m_objective.currentValue;
    m_constraints[0].previousValue = m_constraints[0].currentValue;

    Teuchos::ParameterList args;
    std::vector<std::string> operations;
    args.set(m_optimizationDOFsName, p);
    args.set(m_objective.valueName, &m_objective.currentValue);
    args.set(m_objective.gradientName, m_objective.gradient);
    args.set(con.valueName, &con.currentValue);
    args.set(con.gradientName, con.gradient);
    operations.push_back(m_objective.valueName);
    operations.push_back(con.valueName);
    operations.push_back(m_outputStage);
    m_interface->compute(operations, args); // p, f, dfdp, g

    for(int i=0; i<m_numOptDofs; i++) p_last[i] = p[i];

    computeUpdatedTopology();

    if(WorldComm.getPID()==0.0){
      std::cout << "************************************************************************" << std::endl;
      std::cout << "** Optimization Status Check *******************************************" << std::endl;
      std::cout << "Status: Objective = " << m_objective.currentValue << std::endl;
    }

    double delta_f = m_objective.currentValue - m_objective.previousValue;
    double delta_p = computeDiffNorm(p, p_last, m_numOptDofs, /*result to cout*/ false);
    optimization_converged = convergenceChecker->isConverged(delta_f, delta_p, iter, WorldComm.getPID());

    iter++;
  }

  return;
}


/******************************************************************************/
void
OC_Optimizer::computeUpdatedTopology()
/******************************************************************************/
{

  double vmid, v1=0.0, v2=0.0;
  double residRatio = 0.0;
  int niters=0;

  double* dfdp = m_objective.gradient;
  double* dmdp = m_constraints[0].gradient;

  double dfdp_tot = 0.0, dmdp_tot = 0.0;
  for(int i=0; i<m_numOptDofs; i++) {
    dfdp_tot += dfdp[i];
    dmdp_tot += dmdp[i];
  }
  dfdp_tot = WorldComm.globalSum(dfdp_tot);
  dmdp_tot = WorldComm.globalSum(dmdp_tot);

  v2 = -1000.0* dfdp_tot / dmdp_tot;

  if(WorldComm.getPID()==0){
    std::cout << "Measure enforcement: Target = " << m_constraints[0].targetValue <<  std::endl;
    std::cout << "Measure enforcement: Beginning search with recursive bisection." <<  std::endl;
  }

  bool converged = false;
  Target& con = m_constraints[0];
  do {
    vmid = (v2+v1)/2.0;

    // update topology
    updateTopology(dfdp, dmdp, vmid);

    // compute new measure
    if( m_useNewtonSearch ){
      double prevResidual = con.currentValue - con.targetValue*con.referenceValue;
      Teuchos::ParameterList args;
      args.set(m_optimizationDOFsName, p);
      args.set(con.valueName, &con.currentValue);
      args.set(con.gradientName, con.gradient);
      std::vector<std::string> operations;
      operations.push_back(con.valueName);
      m_interface->compute(operations, args);
      double newResidual = con.currentValue - con.targetValue*con.referenceValue;
      if( newResidual > 0.0 ){
        residRatio = newResidual/prevResidual;
        v1 = vmid;
        niters++;
        break;
      } else v2 = vmid;
    } else {
      Teuchos::ParameterList args;
      args.set(m_optimizationDOFsName, p);
      args.set(con.valueName, &con.currentValue);
      args.set(con.gradientName, con.gradient);
      std::vector<std::string> operations;
      operations.push_back(con.valueName);
      m_interface->compute(operations, args);
      double newResidual = con.currentValue - con.targetValue*con.referenceValue;
      if( newResidual > 0.0 ){
        v1 = vmid;
      } else v2 = vmid;
    }
    niters++;

    if(WorldComm.getPID()==0){
      double resid = (con.currentValue - con.targetValue*con.referenceValue)/con.referenceValue;
      std::cout << "Measure enforcement (iteration " << niters 
                << "): Residual = " << resid << ", Bisection bounds: " 
                << v1 << ", " << v2 << std::endl;
    }

  } while ( niters < m_constraintMaxIter && 
            fabs(con.currentValue - con.targetValue*con.referenceValue) > con.tolerance*con.referenceValue );


  if(m_useNewtonSearch){

    if(WorldComm.getPID()==0){
      std::cout << "Measure enforcement: Bounds found.  Switching to Newton search." << std::endl;
    }

    int newtonMaxIters = niters + 10;
    double lambda = (residRatio*v2 - v1)/(residRatio-1.0);
    double epsilon = lambda*1e-5;
    if( lambda > 0.0 ) do {
      updateTopology(dfdp, dmdp, lambda);
      // compute new measure
      Teuchos::ParameterList args0;
      args0.set(m_optimizationDOFsName, p);
      args0.set(con.valueName, &con.currentValue);
      args0.set(con.gradientName, con.gradient);
      std::vector<std::string> operations0;
      operations0.push_back(con.valueName);
      m_interface->compute(operations0, args0);
      double f0 = con.currentValue - con.targetValue*con.referenceValue;
  
      if(WorldComm.getPID()==0){
        std::cout << "Measure enforcement (iteration " << niters 
                  << "): Residual = " << f0/con.referenceValue << ", Bisection bounds: " 
                  << v1 << ", " << v2 << std::endl;
      }
  
      if( fabs(f0) < con.tolerance*con.referenceValue ){
        converged = true;
        break;
      }
  
      double plambda = lambda+epsilon;
      updateTopology(dfdp, dmdp, lambda);
      // compute new measure
      Teuchos::ParameterList args1;
      args1.set(m_optimizationDOFsName, p);
      args1.set(con.valueName, &con.currentValue);
      args1.set(con.gradientName, con.gradient);
      std::vector<std::string> operations1;
      operations1.push_back(con.valueName);
      m_interface->compute(operations1, args1);
      double f1 = con.currentValue - con.targetValue*con.referenceValue;
  
      if( f1-f0 == 0.0 ) break;
      lambda -= epsilon*f0/(f1-f0);
  
      niters++;
    } while ( niters < newtonMaxIters );
  
    if(!converged){
      if(WorldComm.getPID()==0){
        std::cout << "Measure enforcement: Newton search failed.  Switching back to recursive bisection." << std::endl;
      }
  
      niters = 0;
      do {
        vmid = (v2+v1)/2.0;
    
        updateTopology(dfdp, dmdp, vmid);
    
        // compute new measure
        Teuchos::ParameterList args;
        args.set(m_optimizationDOFsName, p);
        args.set(con.valueName, &con.currentValue);
        args.set(con.gradientName, con.gradient);
        std::vector<std::string> operations;
        operations.push_back(con.valueName);
        m_interface->compute(operations, args);
        double newResidual = con.currentValue - con.targetValue*con.referenceValue;
        if( newResidual > 0.0 ){
          v1 = vmid;
        } else v2 = vmid;
        niters++;
    
        if(WorldComm.getPID()==0){
          double resid = (con.currentValue - con.targetValue*con.referenceValue)/con.referenceValue;
          std::cout << "Measure enforcement (iteration " << niters 
                    << "): Residual = " << resid << ", Bisection bounds: " 
                    << v1 << ", " << v2 << std::endl;
        }
      } while ( niters < m_constraintMaxIter && 
                fabs(con.currentValue - con.targetValue*con.referenceValue) > con.tolerance*con.referenceValue );
    }

  }

  if( fabs(con.currentValue - con.targetValue*con.referenceValue) > con.tolerance*con.referenceValue ){
    LogicException le("Enforcement of measure constraint failed:  Exceeded max iterations");
    m_interface->registerException(le);
  }
}

/******************************************************************************/
inline void 
OC_Optimizer::updateTopology(double* dfdp, double* dmdp, double lambda)
/******************************************************************************/
{
  for(int i=0; i<m_numOptDofs; i++) {
    double be = 0.0;
    be = -dfdp[i]/dmdp[i]/lambda;
    double p_old = p_last[i];
    double offset = 0.01*(m_upperBound - m_lowerBound) - m_lowerBound;
    double sign = (be > 0.0) ? 1 : -1;
    be = (be > 0.0) ? be : -be;
    double p_new = (p_old+offset)*pow(be,m_stabExponent)-offset;
    // limit change
    double dval = p_new - p_old;
    if( fabs(dval) > m_moveLimit) p_new = p_old+fabs(dval)/dval*m_moveLimit;
    // enforce limits
    if( p_new < m_lowerBound ) p_new = m_lowerBound;
    if( p_new > m_upperBound ) p_new = m_upperBound;
    p[i] = p_new;
  }
}

/******************************************************************************/
void OC_Optimizer::finalize() { Optimizer::finalize(); }
/******************************************************************************/


}
