/*
 * Plato_Optimizer.cpp
 *
 *  Created on: April 19, 2017
 *
 */

#include <cstdlib>
#include <stdlib.h>

#include "Plato_Optimizer.hpp"
#include "Plato_Parser.hpp"
#include "Plato_Interface.hpp"
#include "Plato_Exceptions.hpp"

namespace Plato {


/******************************************************************************/
Optimizer::Target::Target(Plato::InputData& node)
/******************************************************************************/
{
    valueName           = Get::String(node, "ValueName");
    gradientName        = Get::String(node, "GradientName");
    referenceValueName  = Get::String(node, "ReferenceValueName");
    targetValue         = Get::Double(node, "TargetValue");
    tolerance           = Get::Double(node, "Tolerance");
    isLinear            = Get::Bool(node,   "Linear");
    isEquality          = Get::Bool(node,   "Equality");

    isRelative = (referenceValueName != "");
}

/******************************************************************************/
Optimizer::Optimizer(Interface* interface) :
        m_interface(interface),
        convergenceChecker(NULL)
/******************************************************************************/
{

  auto inputData = interface->getInputData();

  if( inputData.size<Plato::InputData>("Optimizer") == 0 ){
     Plato::ParsingException tParsingException("Plato::Optimizer: missing 'Optimizer' definitions");
     interface->registerException(tParsingException);
  } else
  if( inputData.size<Plato::InputData>("Optimizer") > 1 )
  {
     Plato::ParsingException tParsingException("Plato::Optimizer: multiple 'Optimizer' definitions");
     interface->registerException(tParsingException);
  }

  auto optNode = inputData.get<Plato::InputData>("Optimizer");

  Plato::InputData outputNode = optNode.get<Plato::InputData>("Output");
  m_outputStage = Get::String(outputNode, "OutputStage");

  Plato::InputData optVarNode = optNode.get<Plato::InputData>("OptimizationVariables");
  m_optimizationDOFsName = Get::String(optVarNode, "ValueName");
  m_initializationStage = Get::String(optVarNode, "InitializationStage");

  // TODO: parse multiple constraints
  if( optNode.size<Plato::InputData>("Constraint") ){
    Plato::InputData conNode = optNode.get<Plato::InputData>("Constraint");
    Target newConstraint(conNode);
    m_constraints.push_back(newConstraint);
  } else {
    throw ParsingException("No Constraint defined.");
  }  

  if( optNode.size<Plato::InputData>("Objective") ){
    Plato::InputData objNode = optNode.get<Plato::InputData>("Objective");
    Target newObjective(objNode);
    m_objective = newObjective;
  } else {
    throw ParsingException("No Objective defined.");
  }  

  // parse bound constraints
  if( optNode.size<Plato::InputData>("BoundConstraint") ){
    Plato::InputData bconNode = optNode.get<Plato::InputData>("BoundConstraint");
    m_upperBound = Plato::Get::Double(bconNode,"Upper");
    m_lowerBound = Plato::Get::Double(bconNode,"Lower");
  } else {
    m_upperBound = 1.0;
    m_lowerBound = 0.0;
  }
 

  // create convergence checker
  if( optNode.size<Plato::InputData>("Convergence") ){
    Plato::InputData checkerNode = optNode.get<Plato::InputData>("Convergence");
    try {
      convergenceChecker = new ConvergenceTest(checkerNode);
    } catch(...) {interface->Catch();}
  }

}

/******************************************************************************/
Optimizer::~Optimizer()
/******************************************************************************/
{
    if(convergenceChecker)
    {
        delete convergenceChecker;
        convergenceChecker = NULL;
    }
}

/******************************************************************************/
void Optimizer::finalize()
/******************************************************************************/
{
    m_interface->getStage("Terminate");
}

} /* namespace Plato */

