/*
 * Plato_Optimizer.cpp
 *
 *  Created on: April 19, 2017
 *
 */

/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
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

