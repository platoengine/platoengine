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
     * @brief Parse options for bound constrained PSO algorithm
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @param [out] aData data structure with bound constrained PSO algorithm options
    **********************************************************************************/
    void parse(const Plato::InputData & aOptimizerNode, Plato::InputDataBCPSO<ScalarType, OrdinalType> & aData)
    {
        if(aOptimizerNode.size<Plato::InputData>("Options"))
        {
            Plato::InputData tOptionsNode = aOptimizerNode.get<Plato::InputData>("Options");
            aData.mOutputDiagnostics = this->outputDiagnostics(tOptionsNode);

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
            aData.mCognitiveBehaviorMultiplier = this->cognitiveBehaviorMultiplier(tOptionsNode);
            aData.mTrustRegionExpansionMultiplier = this->trustRegionExpansionMultiplier(tOptionsNode);
            aData.mTrustRegionContractionMultiplier = this->trustRegionContractionMultiplier(tOptionsNode);
        }
    }

    /******************************************************************************//**
     * @brief Parse options for augmented Lagrangian PSO algorithm
     * @param [in] aOptimizerNode data structure with optimization related input options
     * @param [out] aData data structure with augmented Lagrangian PSO algorithm options
    **********************************************************************************/
    void parse(const Plato::InputData & aOptimizerNode, Plato::InputDataALPSO<ScalarType, OrdinalType> & aData)
    {
        if(aOptimizerNode.size<Plato::InputData>("Options"))
        {
            Plato::InputData tOptionsNode = aOptimizerNode.get<Plato::InputData>("Options");
            aData.mOutputDiagnostics = this->outputDiagnostics(tOptionsNode);

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
            aData.mTrustRegionExpansionMultiplier = this->trustRegionExpansionMultiplier(tOptionsNode);
            aData.mFeasibilityInexactnessTolerance = this->feasibilityInexactnessTolerance(tOptionsNode);
            aData.mTrustRegionContractionMultiplier = this->trustRegionContractionMultiplier(tOptionsNode);
        }
    }

private:
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
        OrdinalType tOutput = 5;
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

private:
    ParticleSwarmParser(const Plato::ParticleSwarmParser<ScalarType, OrdinalType>&);
    Plato::ParticleSwarmParser<ScalarType, OrdinalType> & operator=(const Plato::ParticleSwarmParser<ScalarType, OrdinalType>&);
};
// class ParticleSwarmParser

} // namespace Plato
