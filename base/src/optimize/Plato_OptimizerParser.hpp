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

/*
 * Plato_OptimizerParser.hpp
 *
 *  Created on: Sep 1, 2019
 */

#pragma once

#include <limits>

#include "Plato_Parser.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Optimizer parser parent class
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class OptimizerParser
{
public:
    /******************************************************************************//**
     * @brief Constructor
    **********************************************************************************/
    OptimizerParser()
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~OptimizerParser()
    {
    }

    /******************************************************************************//**
     * @brief Return gradient free objective function stage name
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return gradient free objective function stage name
    **********************************************************************************/
    std::string getObjectiveStageName(const Plato::InputData & aOptimizerNode) const
    {
        std::string tOutput("ObjFuncEval");
        if(aOptimizerNode.size<Plato::InputData>("Objective"))
        {
            Plato::InputData tObjectiveNode = aOptimizerNode.get<Plato::InputData>("Objective");
            tOutput = Plato::Get::String(tObjectiveNode, "ValueStageName");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Return list of gradient free constraint stage names
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return list of gradient free constraint stage names
    **********************************************************************************/
    std::vector<std::string> getConstraintStageNames(const Plato::InputData & aOptimizerNode) const
    {
        std::vector<std::string> tNames;
        auto tAllNodes = aOptimizerNode.getByName<Plato::InputData>("Constraint");
        for(auto tNode = tAllNodes.begin(); tNode != tAllNodes.end(); ++tNode)
        {
            std::string tMyStageName = Plato::Get::String(*tNode, "ValueStageName");
            tNames.push_back(tMyStageName);
        }
        return (tNames);
    }

    /******************************************************************************//**
     * @brief Return list of constraint reference values
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return list of constraint reference values
    **********************************************************************************/
    std::vector<ScalarType> getConstraintReferenceValues(const Plato::InputData & aOptimizerNode) const
    {
        std::vector<ScalarType> tOutput;
        auto tAllNodes = aOptimizerNode.getByName<Plato::InputData>("Constraint");
        for(auto tNode = tAllNodes.begin(); tNode != tAllNodes.end(); ++tNode)
        {
            ScalarType tMyReferenceValue = Plato::Get::Double(*tNode, "ReferenceValue");
            tMyReferenceValue = tMyReferenceValue <= static_cast<ScalarType>(0.0) ? static_cast<ScalarType>(1.0) : tMyReferenceValue;
            tOutput.push_back(tMyReferenceValue);
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Return list of constraint target values
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return list of constraint target values
    **********************************************************************************/
    std::vector<ScalarType> getConstraintTargetValues(const Plato::InputData & aOptimizerNode) const
    {
        std::vector<ScalarType> tOutput;
        auto tAllNodes = aOptimizerNode.getByName<Plato::InputData>("Constraint");
        for(auto tNode = tAllNodes.begin(); tNode != tAllNodes.end(); ++tNode)
        {
            ScalarType tValue = Plato::Get::Double(*tNode, "TargetValue");
            if(std::abs(tValue) <= std::numeric_limits<ScalarType>::epsilon())
            {
                tValue = Plato::Get::Double(*tNode, "NormalizedTargetValue");
            }
            tOutput.push_back(tValue);
        }
        return (tOutput);
    }

private:
    OptimizerParser(const Plato::OptimizerParser<ScalarType, OrdinalType>&);
    Plato::OptimizerParser<ScalarType, OrdinalType> & operator=(const Plato::OptimizerParser<ScalarType, OrdinalType>&);
};
// class OptimizerParser

}
// namespace Plato
