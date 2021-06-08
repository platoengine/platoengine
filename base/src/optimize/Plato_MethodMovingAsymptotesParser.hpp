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
 * Plato_MethodMovingAsymptotesParser.hpp
 *
 *  Created on: Sep 1, 2019
 */

#pragma once

#include "Plato_OptimizerParser.hpp"
#include "Plato_OptimizersIO_Utilities.hpp"
#include "Plato_MethodMovingAsymptotesIO_Data.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Method of Moving Asymptotes (MMA) algorithm parser
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class MethodMovingAsymptotesParser : public Plato::OptimizerParser<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
    **********************************************************************************/
    MethodMovingAsymptotesParser()
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~MethodMovingAsymptotesParser()
    {
    }

    /******************************************************************************//**
     * @brief Parse options for MMA algorithm
     * @param [in] aOptimizerNode data structure with algorithmic options
     * @param [out] aData data structure with MMA algorithmic options
    **********************************************************************************/
    void parse(const Plato::InputData & aOptimizerNode, Plato::AlgorithmInputsMMA<ScalarType, OrdinalType> & aData)
    {
        this->parseOptions(aOptimizerNode, aData);
    }

private:
    /******************************************************************************//**
     * @brief Parse optimizer options for bound constrained PSO algorithm
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @param [out] aData data structure with bound constrained PSO algorithm options
    **********************************************************************************/
    void parseOptions(const Plato::InputData & aOptimizerNode, Plato::AlgorithmInputsMMA<ScalarType, OrdinalType> & aData)
    {
        if(aOptimizerNode.size<Plato::InputData>("Options"))
        {
            Plato::InputData tOptionsNode = aOptimizerNode.get<Plato::InputData>("Options");
            aData.mMemorySpace = this->memorySpace(tOptionsNode);
            aData.mPrintMMADiagnostics = this->outputDiagnostics(tOptionsNode);
            aData.mPrintAugLagSubProbDiagnostics = this->outputSubProblemDiagnostics(tOptionsNode);

            aData.mUpdateFrequency = this->updateFrequency(tOptionsNode);
            aData.mNumControlVectors = this->numControlVectors(tOptionsNode);

            // core mma parameters
            aData.mMoveLimit = this->moveLimit(tOptionsNode);
            aData.mAsymptoteExpansion = this->asymptoteExpansion(tOptionsNode);
            aData.mMaxNumSolverIter = this->maxNumOuterIterations(tOptionsNode);
            aData.mOptimalityTolerance = this->optimalityTolerance(tOptionsNode);
            aData.mFeasibilityTolerance = this->feasibilityTolerance(tOptionsNode);
            aData.mAsymptoteContraction = this->asymptoteContraction(tOptionsNode);
            aData.mInitialAymptoteScaling = this->initialAymptoteScaling(tOptionsNode);
            aData.mControlStagnationTolerance = this->controlStagnationTolerance(tOptionsNode);
            aData.mObjectiveStagnationTolerance = this->objectiveStagnationTolerance(tOptionsNode);

            // core augmented Lagrangian subproblem parameters
            aData.mMaxNumSubProblemIter = this->maxNumSubProblemIter(tOptionsNode);
            aData.mMaxNumTrustRegionIter = this->maxNumTrustRegionIter(tOptionsNode);
            aData.mInitialAugLagPenalty = this->augLagSubProblemInitialPenalty(tOptionsNode);
            aData.mSubProblemBoundsScaling = this->augLagSubProblemBoundsScaling(tOptionsNode);
            aData.mAugLagSubProbPenaltyMultiplier = this->augLagSubProbPenaltyMultiplier(tOptionsNode);
            aData.mConstraintNormalizationMultipliers = this->constraintNormalizationMultipliers(tOptionsNode);
            aData.mAugLagSubProbFeasibilityTolerance = this->augLagSubProblemFeasibilityTolerance(tOptionsNode);
        }
    }

    /******************************************************************************//**
     * @brief Parse output diagnostics keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return output diagnostic flag, default = true
    **********************************************************************************/
    bool outputDiagnostics(const Plato::InputData & aOptionsNode)
    {
        bool tOuput = true;
        if(aOptionsNode.size<std::string>("OutputDiagnosticsToFile"))
        {
            tOuput = Plato::Get::Bool(aOptionsNode, "OutputDiagnosticsToFile");
        }
        return (tOuput);
    }

    /******************************************************************************//**
     * @brief Parse output diagnostic keyword for augmented Lagrangian subproblem.
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return output diagnostic flag, default = false
    **********************************************************************************/
    bool outputSubProblemDiagnostics(const Plato::InputData & aOptionsNode)
    {
        bool tOuput = false;
        if(aOptionsNode.size<std::string>("OutputSubProblemDiagnostics"))
        {
            tOuput = Plato::Get::Bool(aOptionsNode, "OutputSubProblemDiagnostics");
        }
        return (tOuput);
    }

    /******************************************************************************//**
     * @brief Parse memory space keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return memory space, default = HOST
    **********************************************************************************/
    Plato::MemorySpace::type_t memorySpace(const Plato::InputData & aOptionsNode)
    {
        std::string tInput("HOST");
        if(aOptionsNode.size<std::string>("MemorySpace"))
        {
            tInput = Plato::Get::String(aOptionsNode, "MemorySpace", true);
        }
        Plato::MemorySpace::type_t tOutput = Plato::get_memory_space(tInput);
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse number of control vectors keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return number of control vectors, default = 1
    **********************************************************************************/
    OrdinalType numControlVectors(const Plato::InputData & aOptionsNode)
    {
        OrdinalType tOutput = 1;
        if(aOptionsNode.size<std::string>("NumControlVectors"))
        {
            tOutput = Plato::Get::Int(aOptionsNode, "NumControlVectors");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse update frequency keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return update frequency, default = 0
    **********************************************************************************/
    OrdinalType updateFrequency(const Plato::InputData & aOptionsNode)
    {
        OrdinalType tOutput = 0;
        if(aOptionsNode.size<std::string>("UpdateFrequency"))
        {
            tOutput = Plato::Get::Int(aOptionsNode, "UpdateFrequency");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse maximum number of MMA subproblem iterations keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return maximum number of MMA subproblem iterations, default = 100
    **********************************************************************************/
    OrdinalType maxNumSubProblemIter(const Plato::InputData & aOptionsNode)
    {
        OrdinalType tOutput = 25;
        if(aOptionsNode.size<std::string>("MaxNumSubProblemIter"))
        {
            tOutput = Plato::Get::Int(aOptionsNode, "MaxNumSubProblemIter");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse maximum number of trust region iterations keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return maximum number of trust region iterations, default = 50
    **********************************************************************************/
    OrdinalType maxNumTrustRegionIter(const Plato::InputData & aOptionsNode)
    {
        OrdinalType tOutput = 50;
        if(aOptionsNode.size<std::string>("MaxNumTrustRegionIter"))
        {
            tOutput = Plato::Get::Int(aOptionsNode, "MaxNumTrustRegionIter");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse maximum number of outer iterations keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return maximum number of outer iterations, default = 500
    **********************************************************************************/
    OrdinalType maxNumOuterIterations(const Plato::InputData & aOptionsNode)
    {
        OrdinalType tOutput = 500;
        if(aOptionsNode.size<std::string>("MaxNumOuterIterations"))
        {
            tOutput = Plato::Get::Int(aOptionsNode, "MaxNumOuterIterations");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse move limit keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return maximum move limit, default = 500
    **********************************************************************************/
    ScalarType moveLimit(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 0.5;
        if(aOptionsNode.size<std::string>("MoveLimit"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "MoveLimit");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse initial augmented Lagrangian penalty keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return initial augmented Lagrangian penalty, default = 0.0015
    **********************************************************************************/
    ScalarType augLagSubProblemInitialPenalty(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 0.0015;
        if(aOptionsNode.size<std::string>("SubProblemInitialPenalty"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "SubProblemInitialPenalty");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse augmented Lagrangian penalty penalty multiplier keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return augmented Lagrangian penalty penalty multiplier, default = 1.025
    **********************************************************************************/
    ScalarType augLagSubProbPenaltyMultiplier(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1.025;
        if(aOptionsNode.size<std::string>("SubProblemPenaltyMultiplier"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "SubProblemPenaltyMultiplier");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse asymptotes expansion keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return initial asymptotes expansion, default = 1.2
    **********************************************************************************/
    ScalarType asymptoteExpansion(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1.2;
        if(aOptionsNode.size<std::string>("AsymptoteExpansion"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "AsymptoteExpansion");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse asymptotes contraction keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return initial asymptotes contraction, default = 0.7
    **********************************************************************************/
    ScalarType asymptoteContraction(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 0.7;
        if(aOptionsNode.size<std::string>("AsymptoteContraction"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "AsymptoteContraction");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse asymptotes contraction keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return initial asymptotes contraction, default = 0.5
    **********************************************************************************/
    ScalarType initialAymptoteScaling(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 0.5;
        if(aOptionsNode.size<std::string>("InitialAymptoteScaling"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "InitialAymptoteScaling");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse sub problem bound scaling keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return sub problem bound scaling, default = 0.1
    **********************************************************************************/
    ScalarType augLagSubProblemBoundsScaling(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 0.1;
        if(aOptionsNode.size<std::string>("SubProblemBoundsScaling"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "SubProblemBoundsScaling");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse constraint normalization multipliers keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return standard vector of constraint normalization multiplier, default = 0.1
    **********************************************************************************/
    std::vector<ScalarType> constraintNormalizationMultipliers(const Plato::InputData & aOptionsNode)
    {
        std::vector<ScalarType> tOutput = Plato::Get::Doubles(aOptionsNode, "ConstraintNormalizationMultipliers");
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse optimality tolerance keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return optimality tolerance, default = 1e-6
    **********************************************************************************/
    ScalarType optimalityTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1e-6;
        if(aOptionsNode.size<std::string>("OptimalityTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "OptimalityTolerance");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse feasibility tolerance keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return feasibility tolerance, default = 1e-4
    **********************************************************************************/
    ScalarType feasibilityTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1e-4;
        if(aOptionsNode.size<std::string>("FeasibilityTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "FeasibilityTolerance");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse feasibility tolerance keyword for augmented Lagrangian subproblem.
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return feasibility tolerance, default = 1e-8
    **********************************************************************************/
    ScalarType augLagSubProblemFeasibilityTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1e-8;
        if(aOptionsNode.size<std::string>("SubProblemFeasibilityTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "SubProblemFeasibilityTolerance");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse control stagnation tolerance keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return control stagnation tolerance, default = 1e-6
    **********************************************************************************/
    ScalarType controlStagnationTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1e-6;
        if(aOptionsNode.size<std::string>("ControlStagnationTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "ControlStagnationTolerance");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse objective stagnation tolerance keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @return objective stagnation tolerance, default = 1e-8
    **********************************************************************************/
    ScalarType objectiveStagnationTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1e-8;
        if(aOptionsNode.size<std::string>("ObjectiveStagnationTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "ObjectiveStagnationTolerance");
        }
        return (tOutput);
    }

private:
    MethodMovingAsymptotesParser(const Plato::MethodMovingAsymptotesParser<ScalarType, OrdinalType>&);
    Plato::MethodMovingAsymptotesParser<ScalarType, OrdinalType> & operator=(const Plato::MethodMovingAsymptotesParser<ScalarType, OrdinalType>&);
};
// class MethodMovingAsymptotesParser

}
// namespace Plato

