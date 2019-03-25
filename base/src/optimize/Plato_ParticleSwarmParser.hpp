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
 * Plato_ParticleSwarmParser.hpp
 *
 *  Created on: Jan 24, 2019
 */

#pragma once

#include <sstream>
#include <algorithm>

#include "Plato_Parser.hpp"
#include "Plato_InputData.hpp"
#include "Plato_ParticleSwarmIO_BCPSO.hpp"
#include "Plato_ParticleSwarmIO_ALPSO.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Parser for Particle Swarm Optimization (PSO) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class ParticleSwarmParser
{
public:
    /******************************************************************************//**
     * @brief Constructor
    **********************************************************************************/
    ParticleSwarmParser()
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~ParticleSwarmParser()
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
            const ScalarType tMyTargeteValue = Plato::Get::Double(*tNode, "TargetValue");
            tOutput.push_back(tMyTargeteValue);
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse options for bound constrained PSO algorithm
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @param [out] aData data structure with bound constrained PSO algorithm options
    **********************************************************************************/
    void parse(const Plato::InputData & aOptimizerNode, Plato::InputDataBCPSO<ScalarType, OrdinalType> & aData)
    {
        this->parseOptions(aOptimizerNode, aData);
        if(aOptimizerNode.size<Plato::InputData>("Output"))
        {
            Plato::InputData tOutputNode = aOptimizerNode.get<Plato::InputData>("Output");
            aData.mOutputStageName = this->outputStageName(tOutputNode);
        }
    }

    /******************************************************************************//**
     * @brief Parse options for augmented Lagrangian PSO algorithm
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @param [out] aData data structure with augmented Lagrangian PSO algorithm options
    **********************************************************************************/
    void parse(const Plato::InputData & aOptimizerNode, Plato::InputDataALPSO<ScalarType, OrdinalType> & aData)
    {
        this->parseOptions(aOptimizerNode, aData);
        this->parseConstraintTypes(aOptimizerNode, aData);
        if(aOptimizerNode.size<Plato::InputData>("Output"))
        {
            Plato::InputData tOutputNode = aOptimizerNode.get<Plato::InputData>("Output");
            aData.mOutputStageName = this->outputStageName(tOutputNode);
        }
    }

private:
    /******************************************************************************//**
     * @brief Parse optimizer options for augmented Lagrangian PSO algorithm
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @param [out] aData data structure with augmented Lagrangian PSO algorithm options
    **********************************************************************************/
    void parseOptions(const Plato::InputData & aOptimizerNode, Plato::InputDataALPSO<ScalarType, OrdinalType> & aData)
    {
        if(aOptimizerNode.size<Plato::InputData>("Options"))
        {
            Plato::InputData tOptionsNode = aOptimizerNode.get<Plato::InputData>("Options");
            aData.mOutputSolution = this->outputSolution(tOptionsNode);
            aData.mOutputDiagnostics = this->outputDiagnostics(tOptionsNode);
            aData.mOutputParticleDiagnostics = this->outputParticleDiagnostics(tOptionsNode);
            aData.mDisableStdDevStoppingTol = this->disableStdDevStoppingTolerance(tOptionsNode);

            aData.mNumParticles = this->numParticles(tOptionsNode);
            aData.mMaxNumOuterIter = this->maxNumOuterIterations(tOptionsNode);
            aData.mMaxNumInnerIter = this->maxNumInnerIterations(tOptionsNode);
            aData.mMaxNumConsecutiveFailures = this->maxNumConsecutiveFailures(tOptionsNode);
            aData.mMaxNumConsecutiveSuccesses = this->maxNumConsecutiveSuccesses(tOptionsNode);

            aData.mTimeStep = this->particleVelocityTimeStep(tOptionsNode);
            aData.mInertiaMultiplier = this->inertiaMultiplier(tOptionsNode);
            aData.mSocialBehaviorMultiplier = this->socialBehaviorMultiplier(tOptionsNode);
            aData.mMeanBestAugLagFuncTolerance = this->meanBestAugLagFuncTolerance(tOptionsNode);
            aData.mPenaltyExpansionMultiplier = this->penaltyExpansionMultiplier(tOptionsNode);
            aData.mCognitiveBehaviorMultiplier = this->cognitiveBehaviorMultiplier(tOptionsNode);
            aData.mPenaltyMultiplierUpperBound = this->penaltyMultiplierUpperBound(tOptionsNode);
            aData.mPenaltyContractionMultiplier = this->penaltyContractionMultiplier(tOptionsNode);
            aData.mStdDevBestAugLagFuncTolerance = this->stdDevBestAugLagFuncTolerance(tOptionsNode);
            aData.mGlobalBestAugLagFuncTolerance = this->globalBestAugLagFuncTolerance(tOptionsNode);
            aData.mTrustRegionMultiplierTolerance = this->trustRegionMultiplierTolerance(tOptionsNode);
            aData.mTrustRegionExpansionMultiplier = this->trustRegionExpansionMultiplier(tOptionsNode);
            aData.mFeasibilityInexactnessTolerance = this->feasibilityInexactnessTolerance(tOptionsNode);
            aData.mTrustRegionContractionMultiplier = this->trustRegionContractionMultiplier(tOptionsNode);
        }
    }

    /******************************************************************************//**
     * @brief Parse optimizer options for bound constrained PSO algorithm
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @param [out] aData data structure with bound constrained PSO algorithm options
    **********************************************************************************/
    void parseOptions(const Plato::InputData & aOptimizerNode, Plato::InputDataBCPSO<ScalarType, OrdinalType> & aData)
    {
        if(aOptimizerNode.size<Plato::InputData>("Options"))
        {
            Plato::InputData tOptionsNode = aOptimizerNode.get<Plato::InputData>("Options");
            aData.mOutputSolution = this->outputSolution(tOptionsNode);
            aData.mOutputDiagnostics = this->outputDiagnostics(tOptionsNode);
            aData.mOutputParticleDiagnostics = this->outputParticleDiagnostics(tOptionsNode);
            aData.mDisableStdDevStoppingTol = this->disableStdDevStoppingTolerance(tOptionsNode);

            aData.mNumParticles = this->numParticles(tOptionsNode);
            aData.mMaxNumIterations = this->maxNumOuterIterations(tOptionsNode);
            aData.mMaxNumConsecutiveFailures = this->maxNumConsecutiveFailures(tOptionsNode);
            aData.mMaxNumConsecutiveSuccesses = this->maxNumConsecutiveSuccesses(tOptionsNode);

            aData.mTimeStep = this->particleVelocityTimeStep(tOptionsNode);
            aData.mInertiaMultiplier = this->inertiaMultiplier(tOptionsNode);
            aData.mSocialBehaviorMultiplier = this->socialBehaviorMultiplier(tOptionsNode);
            aData.mMeanBestObjFuncTolerance = this->meanBestObjFuncTolerance(tOptionsNode);
            aData.mGlobalBestObjFuncTolerance = this->globalBestObjFuncTolerance(tOptionsNode);
            aData.mStdDevBestObjFuncTolerance = this->stdDevBestObjFuncTolerance(tOptionsNode);
            aData.mTrustRegionMultiplierTolerance = this->trustRegionMultiplierTolerance(tOptionsNode);
            aData.mCognitiveBehaviorMultiplier = this->cognitiveBehaviorMultiplier(tOptionsNode);
            aData.mTrustRegionExpansionMultiplier = this->trustRegionExpansionMultiplier(tOptionsNode);
            aData.mTrustRegionContractionMultiplier = this->trustRegionContractionMultiplier(tOptionsNode);
        }
    }

    /******************************************************************************//**
     * @brief Transform string with constraint type into Plato::particle_swarm::constraint_t type
     * @param [in] aType constraint type string
     * @return constraint type, default = INEQUALITY
    **********************************************************************************/
    Plato::particle_swarm::constraint_t type(const std::string & aType)
    {
        std::string tCopy = aType;
        std::transform(tCopy.begin(), tCopy.end(), tCopy.begin(), ::tolower);
        Plato::particle_swarm::constraint_t tOutput;
        if(tCopy.compare("equality") == 0)
        {
            tOutput = Plato::particle_swarm::EQUALITY;
        }
        else if(tCopy.compare("inequality") == 0)
        {
            tOutput = Plato::particle_swarm::INEQUALITY;
        }
        else
        {
            tOutput = Plato::particle_swarm::INEQUALITY;
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse constraint type, options 1) EQUALITY or 2) INEQUALITY
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @param [out] aData data structure with augmented Lagrangian PSO algorithm options
    **********************************************************************************/
    void parseConstraintTypes(const Plato::InputData & aOptimizerNode,
                              Plato::InputDataALPSO<ScalarType, OrdinalType> & aData)
    {
        auto tConstraints = aOptimizerNode.getByName<Plato::InputData>("Constraint");
        for(auto tMyConstraint = tConstraints.begin(); tMyConstraint != tConstraints.end(); ++ tMyConstraint)
        {
            std::string tType;
            Plato::particle_swarm::constraint_t tMyType;
            try { tType = tMyConstraint->get<std::string>("Type"); }
            catch(int aValue) { /* USE DEFAULT VALUE = INEQUALITY */ }
            tMyType = this->type(tType);
            aData.mConstraintTypes.push_back(tMyType);
        }
    }

    /******************************************************************************//**
     * @brief Parse output diagnostics keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
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
     * @brief Parse output algorithm diagnostics keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    bool outputParticleDiagnostics(const Plato::InputData & aOptionsNode)
    {
        bool tOuput = false;
        if(aOptionsNode.size<std::string>("OutputParticleDiagnosticsToFile"))
        {
            tOuput = Plato::Get::Bool(aOptionsNode, "OutputParticleDiagnosticsToFile");
        }
        return (tOuput);
    }

    /******************************************************************************//**
     * @brief Parse output solution keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    bool outputSolution(const Plato::InputData & aOptionsNode)
    {
        bool tOuput = false;
        if(aOptionsNode.size<std::string>("OutputSolution"))
        {
            tOuput = Plato::Get::Bool(aOptionsNode, "OutputSolution");
        }
        return (tOuput);
    }

    /******************************************************************************//**
     * @brief Parse disable standard deviation stopping tolerance keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    bool disableStdDevStoppingTolerance(const Plato::InputData & aOptionsNode)
    {
        bool tOuput = false;
        if(aOptionsNode.size<std::string>("DisableStdDevStoppingTolerance"))
        {
            tOuput = Plato::Get::Bool(aOptionsNode, "DisableStdDevStoppingTolerance");
        }
        return (tOuput);
    }

    /******************************************************************************//**
     * @brief Parse number of particles
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    OrdinalType numParticles(const Plato::InputData & aOptionsNode)
    {
        OrdinalType tOutput = 10;
        if(aOptionsNode.size<std::string>("NumParticles"))
        {
            tOutput = Plato::Get::Int(aOptionsNode, "NumParticles");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse maximum number of outer iterations keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    OrdinalType maxNumOuterIterations(const Plato::InputData & aOptionsNode)
    {
        OrdinalType tOutput = 1e3;
        if(aOptionsNode.size<std::string>("MaxNumOuterIterations"))
        {
            tOutput = Plato::Get::Int(aOptionsNode, "MaxNumOuterIterations");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse maximum number of inner iterations keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    OrdinalType maxNumInnerIterations(const Plato::InputData & aOptionsNode)
    {
        OrdinalType tOutput = 10;
        if(aOptionsNode.size<std::string>("MaxNumInnerIterations"))
        {
            tOutput = Plato::Get::Int(aOptionsNode, "MaxNumInnerIterations");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse maximum number of consecutive failures keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    OrdinalType maxNumConsecutiveFailures(const Plato::InputData & aOptionsNode)
    {
        OrdinalType tOutput = 10;
        if(aOptionsNode.size<std::string>("MaxNumConsecutiveFailures"))
        {
            tOutput = Plato::Get::Int(aOptionsNode, "MaxNumConsecutiveFailures");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse maximum number of consecutive successes keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    OrdinalType maxNumConsecutiveSuccesses(const Plato::InputData & aOptionsNode)
    {
        OrdinalType tOutput = 10;
        if(aOptionsNode.size<std::string>("MaxNumConsecutiveSuccesses"))
        {
            tOutput = Plato::Get::Int(aOptionsNode, "MaxNumConsecutiveSuccesses");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse particle's velocity time step keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType particleVelocityTimeStep(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1.0;
        if(aOptionsNode.size<std::string>("ParticleVelocityTimeStep"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "ParticleVelocityTimeStep");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse upper bound on penalty multiplier keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType penaltyMultiplierUpperBound(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 100;
        if(aOptionsNode.size<std::string>("PenaltyMultiplierUpperBound"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "PenaltyMultiplierUpperBound");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse expansion multiplier keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType penaltyExpansionMultiplier(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 2.0;
        if(aOptionsNode.size<std::string>("PenaltyExpansionMultiplier"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "PenaltyExpansionMultiplier");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse contraction multiplier keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType penaltyContractionMultiplier(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 0.5;
        if(aOptionsNode.size<std::string>("PenaltyContractionMultiplier"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "PenaltyContractionMultiplier");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse tolerance on the global best objective function
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType globalBestObjFuncTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1e-10;
        if(aOptionsNode.size<std::string>("GlobalBestObjFuncTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "GlobalBestObjFuncTolerance");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse tolerance on the mean of the best objective function values
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType meanBestObjFuncTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 5e-4;
        if(aOptionsNode.size<std::string>("MeanBestObjFuncTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "MeanBestObjFuncTolerance");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse tolerance on the standard deviation of the best objective function values
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType stdDevBestObjFuncTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1e-6;
        if(aOptionsNode.size<std::string>("StdDevBestObjFuncTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "StdDevBestObjFuncTolerance");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse lower tolerance on trust region multiplier
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType trustRegionMultiplierTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1e-8;
        if(aOptionsNode.size<std::string>("TrustRegionMultiplierTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "TrustRegionMultiplierTolerance");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse tolerance on the mean of the best augmented Lagrangian function values
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType meanBestAugLagFuncTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 5e-4;
        if(aOptionsNode.size<std::string>("MeanBestAugLagFuncTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "MeanBestAugLagFuncTolerance");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse tolerance on the standard deviation of the best augmented Lagrangian function values
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType stdDevBestAugLagFuncTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1e-6;
        if(aOptionsNode.size<std::string>("StdDevBestAugLagFuncTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "StdDevBestAugLagFuncTolerance");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse tolerance on the global best augmented Lagrangian function
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType globalBestAugLagFuncTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1e-10;
        if(aOptionsNode.size<std::string>("GlobalBestAugLagFuncTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "GlobalBestAugLagFuncTolerance");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse tolerance on the global best augmented Lagrangian function
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType inertiaMultiplier(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 0.9;
        if(aOptionsNode.size<std::string>("InertiaMultiplier"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "InertiaMultiplier");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse social behavior multiplier keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType socialBehaviorMultiplier(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 0.8;
        if(aOptionsNode.size<std::string>("SocialBehaviorMultiplier"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "SocialBehaviorMultiplier");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse cognitive behavior multiplier keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType cognitiveBehaviorMultiplier(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 0.8;
        if(aOptionsNode.size<std::string>("CognitiveBehaviorMultiplier"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "CognitiveBehaviorMultiplier");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse trust region expansion multiplier keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType trustRegionExpansionMultiplier(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 4;
        if(aOptionsNode.size<std::string>("TrustRegionExpansionMultiplier"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "TrustRegionExpansionMultiplier");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse trust region contraction multiplier keyword
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType trustRegionContractionMultiplier(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 0.75;
        if(aOptionsNode.size<std::string>("TrustRegionContractionMultiplier"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "TrustRegionContractionMultiplier");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse tolerance on feasibility inexactness
     * @param [in] aOptimizerNode data structure with optimization related input options
    **********************************************************************************/
    ScalarType feasibilityInexactnessTolerance(const Plato::InputData & aOptionsNode)
    {
        ScalarType tOutput = 1e-4;
        if(aOptionsNode.size<std::string>("FeasibilityInexactnessTolerance"))
        {
            tOutput = Plato::Get::Double(aOptionsNode, "FeasibilityInexactnessTolerance");
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Parse output stage name
     * @param [in] aOutputNode output stage database
    **********************************************************************************/
    std::string outputStageName(const Plato::InputData & aOutputNode)
    {
        std::string tOutput;
        if(aOutputNode.size<std::string>("OutputStage"))
        {
            tOutput = Plato::Get::String(aOutputNode, "OutputStage");
        }
        return (tOutput);
    }

private:
    ParticleSwarmParser(const Plato::ParticleSwarmParser<ScalarType, OrdinalType>&);
    Plato::ParticleSwarmParser<ScalarType, OrdinalType> & operator=(const Plato::ParticleSwarmParser<ScalarType, OrdinalType>&);
};
// class ParticleSwarmParser

} // namespace Plato
