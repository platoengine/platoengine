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
 * Plato_ParticleSwarmOperations.hpp
 *
 *  Created on: Jan 24, 2019
 */

#pragma once

#include <cmath>
#include <random>

#include "Plato_DataFactory.hpp"
#include "Plato_ParticleSwarmDataMng.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Core operations for the Particle Swarm Optimization (PSO) algorithms
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class ParticleSwarmOperations
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aFactory data factory for class member data
     **********************************************************************************/
    explicit ParticleSwarmOperations(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aFactory) :
        mNumConsecutiveFailures(0),
        mNumConsecutiveSuccesses(0),
        mMaxNumConsecutiveFailures(10),
        mMaxNumConsecutiveSuccesses(10),
        mRandomNumMultiplier(0.2),
        mInertiaMultiplier(0.9),
        mSocialBehaviorMultiplier(0.8),
        mCognitiveBehaviorMultiplier(0.8),
        mTrustRegionMultiplier(1),
        mTrustRegionExpansionMultiplier(4.0),
        mTrustRegionContractionMultiplier(0.75),
        mTrialControl(),
        mTrialVelocity()
    {
        this->initialize(*aFactory);
    }

    /******************************************************************************//**
     * @brief Destructor
     **********************************************************************************/
    virtual ~ParticleSwarmOperations()
    {
    }

    /******************************************************************************//**
     * @brief Return inertia multiplier
     * @return inertia multiplier
     **********************************************************************************/
    ScalarType getInertiaMultiplier() const
    {
        return (mInertiaMultiplier);
    }

    /******************************************************************************//**
     * @brief Return random number multiplier used to update the particle positions.
     * The random number multiplier is used to find an unique particle.
     * @return random number multiplier
     **********************************************************************************/
    ScalarType getRandomNumMultiplier() const
    {
        return (mRandomNumMultiplier);
    }

    /******************************************************************************//**
     * @brief Return trust region multiplier
     * @return trust region multiplier
     **********************************************************************************/
    ScalarType getTrustRegionMultiplier() const
    {
        return (mTrustRegionMultiplier);
    }

    /******************************************************************************//**
     * @brief Set inertia multiplier
     * @param [in] aInput inertia multiplier
     **********************************************************************************/
    void setInertiaMultiplier(const ScalarType &aInput)
    {
        mInertiaMultiplier = aInput;
    }

    /******************************************************************************//**
     * @brief Set random number multiplier used to update the particle positions.
     * The random number multiplier is used to find an unique particle.
     * @param [in] aInput random number multiplier
     **********************************************************************************/
    void setRandomNumMultiplier(const ScalarType &aInput)
    {
        mRandomNumMultiplier = aInput;
    }

    /******************************************************************************//**
     * @brief Set cognitive behavior multiplier
     * @param [in] aInput cognitive behavior multiplier
     **********************************************************************************/
    void setCognitiveBehaviorMultiplier(const ScalarType &aInput)
    {
        mCognitiveBehaviorMultiplier = aInput;
    }

    /******************************************************************************//**
     * @brief Set social behavior multiplier
     * @param [in] aInput social behavior multiplier
     **********************************************************************************/
    void setSocialBehaviorMultiplier(const ScalarType &aInput)
    {
        mSocialBehaviorMultiplier = aInput;
    }

    /******************************************************************************//**
     * @brief Set trust region expansion multiplier
     * @param [in] aInput trust region expansion multiplier
     **********************************************************************************/
    void setTrustRegionExpansionMultiplier(const ScalarType &aInput)
    {
        mTrustRegionExpansionMultiplier = aInput;
    }

    /******************************************************************************//**
     * @brief Set trust region contraction multiplier
     * @param [in] aInput trust region contraction multiplier
     **********************************************************************************/
    void setTrustRegionContractionMultiplier(const ScalarType &aInput)
    {
        mTrustRegionContractionMultiplier = aInput;
    }

    /******************************************************************************//**
     * @brief Set number of consecutive failures. A failure is defined as \f$ f(x_i)
     *        = f(x_{i-1}) \f$, where i denotes the optimization iteration and \f$ f
     *        \f$ is the objective function.
     * @param [in] aInput number of consecutive failures
     **********************************************************************************/
    void setNumConsecutiveFailures(const OrdinalType &aInput)
    {
        mNumConsecutiveFailures = aInput;
    }

    /******************************************************************************//**
     * @brief Set number of consecutive successes. A failure is defined as \f$ f(x_i)
     *        < f(x_{i-1}) \f$, where i denotes the optimization iteration and \f$ f
     *        \f$ is the objective function.
     * @param [in] aInput number of consecutive successes
     **********************************************************************************/
    void setNumConsecutiveSuccesses(const OrdinalType &aInput)
    {
        mNumConsecutiveSuccesses = aInput;
    }

    /******************************************************************************//**
     * @brief Set maximum number of consecutive failures. A failure is defined as
     *        \f$ f(x_i) = f(x_{i-1}) \f$, where i denotes the optimization iteration
     *        and \f$ f \f$ is the objective function.
     * @param [in] aInput maximum number of consecutive failures
     **********************************************************************************/
    void setMaxNumConsecutiveFailures(const OrdinalType &aInput)
    {
        mMaxNumConsecutiveFailures = aInput;
    }

    /******************************************************************************//**
     * @brief Set maximum number of consecutive successes. A failure is defined as
     *        \f$ f(x_i) = f(x_{i-1}) \f$, where i denotes the optimization iteration
     *        and \f$ f \f$ is the objective function.
     * @param [in] aInput maximum number of consecutive successes
     **********************************************************************************/
    void setMaxNumConsecutiveSuccesses(const OrdinalType &aInput)
    {
        mMaxNumConsecutiveSuccesses = aInput;
    }

    /******************************************************************************//**
     * @brief Check if inertia multiplier is between theoretical bounds
     **********************************************************************************/
    bool checkInertiaMultiplier()
    {
        const ScalarType tUpperBound = 1.0;
        const ScalarType tLowerBound = (static_cast<ScalarType>(0.5) * (mCognitiveBehaviorMultiplier + mSocialBehaviorMultiplier))
            - static_cast<ScalarType>(1.0);
        const bool tMultiplierOutsideBounds = (mInertiaMultiplier < tLowerBound) || (mInertiaMultiplier > tUpperBound);
        if (tMultiplierOutsideBounds == true)
        {
            // use default values
            mInertiaMultiplier = 0.9;
            mSocialBehaviorMultiplier = 0.8;
            mCognitiveBehaviorMultiplier = 0.8;
        }
        return (tMultiplierOutsideBounds);
    }

    /******************************************************************************//**
     * @brief Update trust region multiplier
     **********************************************************************************/
    void updateTrustRegionMultiplier()
    {
        if (mNumConsecutiveSuccesses >= mMaxNumConsecutiveSuccesses)
        {
            mTrustRegionMultiplier *= mTrustRegionExpansionMultiplier;
        }
        else if (mNumConsecutiveFailures >= mMaxNumConsecutiveFailures)
        {
            mTrustRegionMultiplier *= mTrustRegionContractionMultiplier;
        }
    }

    /******************************************************************************//**
     * @brief Update particle velocities and find global best particle velocity
     * @param [in] aDataMng PSO data manager
     **********************************************************************************/
    void updateParticleVelocities(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> &aDataMng)
    {
        aDataMng.cachePreviousVelocities();
        std::uniform_real_distribution<ScalarType> tDistribution(0.0 /* lower bound */, 1.0 /* upper bound */);

        const OrdinalType tMyProcID = aDataMng.getCommWrapper().myProcID();
        const OrdinalType tCurrentGlobalBestParticleRank = aDataMng.getCurrentGlobalBestParticleRank();
        const OrdinalType tCurrentGlobalBestParticleIndex = aDataMng.getCurrentGlobalBestParticleIndex();

        const OrdinalType tNumParticles = aDataMng.getNumParticles();
        for (OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            if ((tIndex == tCurrentGlobalBestParticleIndex) && (tMyProcID == tCurrentGlobalBestParticleRank))
            {
                this->updateGlobalBestParticleVelocity(tDistribution, aDataMng);
            }
            else
            {
                this->updateParticleVelocity(tIndex, tDistribution, aDataMng);
            }
        }
    }

    /******************************************************************************//**
     * @brief Update particle positions and find global best particle position
     * @param [in] aDataMng PSO data manager
     **********************************************************************************/
    void updateParticlePositions(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> &aDataMng)
    {
        const OrdinalType tMyProcID = aDataMng.getCommWrapper().myProcID();
        const OrdinalType tCurrentGlobalBestParticleRank = aDataMng.getCurrentGlobalBestParticleRank();
        const OrdinalType tCurrentGlobalBestParticleIndex = aDataMng.getCurrentGlobalBestParticleIndex();

        const OrdinalType tNumParticles = aDataMng.getNumParticles();
        for (OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            if ((tIndex == tCurrentGlobalBestParticleIndex) && (tMyProcID == tCurrentGlobalBestParticleRank))
            {
                this->findUniqueGlobalBestParticlePositions(aDataMng);
            }
            else
            {
                this->findUniqueParticlePositions(tIndex, aDataMng);
            }
        }
    }

    /******************************************************************************//**
     * @brief Check success rate: i.e. did the PSO algorithm find a new minimum in the last iteration?
     * @param [in] aDataMng PSO data manager
     **********************************************************************************/
    void checkGlobalBestParticleUpdateSuccessRate(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> &aDataMng)
    {
        const ScalarType tCurrentGlobalBestObjFunValue = aDataMng.getCurrentGlobalBestObjFuncValue();
        const ScalarType tPreviousGlobalBestObjFunValue = aDataMng.getPreviousGlobalBestObjFuncValue();

        if (tCurrentGlobalBestObjFunValue < tPreviousGlobalBestObjFunValue)
        {
            mNumConsecutiveSuccesses++;
            mNumConsecutiveFailures = 0;
        }
        else
        {
            mNumConsecutiveFailures++;
            mNumConsecutiveSuccesses = 0;
        }
    }

private:
    /******************************************************************************//**
     * @brief Allocate class member containers
     * @param [in] aFactory PSO data factory
     **********************************************************************************/
    void initialize(const Plato::DataFactory<ScalarType, OrdinalType> &aFactory)
    {
        const OrdinalType tPARTICLE_INDEX = 0;
        mTrialControl = aFactory.control(tPARTICLE_INDEX).create();
        mTrialVelocity = aFactory.control(tPARTICLE_INDEX).create();
    }

    /******************************************************************************//**
     * @brief Update particle velocity
     * @param [in] aParticleIndex particle index
     * @param [in] aDistribution uniform distribution sample generator
     * @param [in] aDataMng PSO data manager
     **********************************************************************************/
    void updateParticleVelocity(const OrdinalType &aParticleIndex,
                                std::uniform_real_distribution<ScalarType> &aDistribution,
                                Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> &aDataMng)
    {
        const Plato::Vector<ScalarType, OrdinalType> &tPreviousVel = aDataMng.getPreviousVelocity(aParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> &tCurrentParticle = aDataMng.getCurrentParticle(aParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> &tGlobalBestParticlePosition = aDataMng.getGlobalBestParticlePosition();
        const Plato::Vector<ScalarType, OrdinalType> &tBestParticlePosition = aDataMng.getBestParticlePosition(aParticleIndex);

        const OrdinalType tNumControls = tPreviousVel.size();
        for (OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            // inertia contribution
            const ScalarType tInertiaValue = mInertiaMultiplier * tPreviousVel[tIndex];
            // cognitive behavior contribution
            const ScalarType tRandomNumOne = aDistribution(mGenerator);
            const ScalarType tCognitiveMultiplier = tRandomNumOne * mCognitiveBehaviorMultiplier;
            const ScalarType tCognitiveValue = tCognitiveMultiplier * (tBestParticlePosition[tIndex] - tCurrentParticle[tIndex]);
            // social behavior contribution
            const ScalarType tRandomNumTwo = aDistribution(mGenerator);
            const ScalarType tSocialMultiplier = tRandomNumTwo * mSocialBehaviorMultiplier;
            const ScalarType tSocialValue = tSocialMultiplier * (tGlobalBestParticlePosition[tIndex] - tCurrentParticle[tIndex]);
            // set new velocity
            (*mTrialVelocity)[tIndex] = tInertiaValue + tCognitiveValue + tSocialValue;
        }

        aDataMng.setCurrentVelocity(aParticleIndex, *mTrialVelocity);
    }

    /******************************************************************************//**
     * @brief Update global best particle velocity
     * @param [in] aDistribution uniform distribution sample generator
     * @param [in] aDataMng PSO data manager
     **********************************************************************************/
    void updateGlobalBestParticleVelocity(std::uniform_real_distribution<ScalarType> &aDistribution,
                                          Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> &aDataMng)
    {
        const OrdinalType tCurrentGlobalBestParticleIndex = aDataMng.getCurrentGlobalBestParticleIndex();
        const Plato::Vector<ScalarType, OrdinalType> &tPreviousVel = aDataMng.getPreviousVelocity(tCurrentGlobalBestParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> &tGlobalBestParticlePosition = aDataMng.getGlobalBestParticlePosition();
        const Plato::Vector<ScalarType, OrdinalType> &tCurrentParticlePosition = aDataMng.getCurrentParticle(tCurrentGlobalBestParticleIndex);

        const OrdinalType tNumControls = tPreviousVel.size();
        for (OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            const ScalarType tRandomNum = aDistribution(mGenerator);
            const ScalarType tStochasticTrustRegionMultiplier = mTrustRegionMultiplier * (static_cast<ScalarType>(1) - static_cast<ScalarType>(2) * tRandomNum);
            (*mTrialVelocity)[tIndex] = (static_cast<ScalarType>(-1) * tCurrentParticlePosition[tIndex]) + tGlobalBestParticlePosition[tIndex]
                + (mInertiaMultiplier * tPreviousVel[tIndex]) + tStochasticTrustRegionMultiplier;
        }

        aDataMng.setCurrentVelocity(tCurrentGlobalBestParticleIndex, *mTrialVelocity);
    }

    /******************************************************************************//**
     * @brief Update particle position. The new particle is required to be unique.
     * @param [in] aParticleIndex particle index
     * @param [in] aDataMng PSO data manager
     **********************************************************************************/
    void findUniqueParticlePositions(const OrdinalType &aParticleIndex,
                                     Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> &aDataMng)
    {
        const ScalarType tTimeStep = aDataMng.getTimeStep();
        const Plato::Vector<ScalarType, OrdinalType> &tLowerBounds = aDataMng.getLowerBounds();
        const Plato::Vector<ScalarType, OrdinalType> &tUpperBounds = aDataMng.getUpperBounds();
        const Plato::Vector<ScalarType, OrdinalType> &tParticleVel = aDataMng.getCurrentVelocity(aParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> &tParticlePosition = aDataMng.getCurrentParticle(aParticleIndex);
        mTrialControl->update(static_cast<ScalarType>(1), tParticlePosition, static_cast<ScalarType>(0));

        const OrdinalType tNumControls = tParticleVel.size();
        for (OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            (*mTrialControl)[tIndex] = (*mTrialControl)[tIndex] + (tTimeStep * tParticleVel[tIndex]);
            (*mTrialControl)[tIndex] = std::max((*mTrialControl)[tIndex], tLowerBounds[tIndex]);
            (*mTrialControl)[tIndex] = std::min((*mTrialControl)[tIndex], tUpperBounds[tIndex]);
        }

        this->findUniqueParticleAndUpdate(aParticleIndex, aDataMng);
    }

    /******************************************************************************//**
     * @brief Update global best particle position. The new particle is required to be unique.
     * @param [in] aDataMng PSO data manager
     **********************************************************************************/
    void findUniqueGlobalBestParticlePositions(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> &aDataMng)
    {
        const Plato::Vector<ScalarType, OrdinalType> &tLowerBounds = aDataMng.getLowerBounds();
        const Plato::Vector<ScalarType, OrdinalType> &tUpperBounds = aDataMng.getUpperBounds();
        const OrdinalType tCurrentGlobalBestParticleIndex = aDataMng.getCurrentGlobalBestParticleIndex();
        const Plato::Vector<ScalarType, OrdinalType> &tGlobalBestParticleVel = aDataMng.getCurrentVelocity(tCurrentGlobalBestParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> &tGlobalBestParticlePosition = aDataMng.getGlobalBestParticlePosition();

        std::uniform_real_distribution<ScalarType> tDistribution(0.0 /* lower bound */, 1.0 /* upper bound */);
        const ScalarType tRandomNum = tDistribution(mGenerator);
        const ScalarType tStochasticTrustRegionMultiplier = mTrustRegionMultiplier * (static_cast<ScalarType>(1) - static_cast<ScalarType>(2) * tRandomNum);

        const OrdinalType tNumControls = tGlobalBestParticlePosition.size();
        for (OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            (*mTrialControl)[tIndex] = tGlobalBestParticlePosition[tIndex] + (mInertiaMultiplier * tGlobalBestParticleVel[tIndex])
                + tStochasticTrustRegionMultiplier;
            (*mTrialControl)[tIndex] = std::max((*mTrialControl)[tIndex], tLowerBounds[tIndex]);
            (*mTrialControl)[tIndex] = std::min((*mTrialControl)[tIndex], tUpperBounds[tIndex]);
        }

        this->findUniqueParticleAndUpdate(tCurrentGlobalBestParticleIndex, aDataMng);
    }

    /******************************************************************************//**
     * @brief Update particle position, finds an unique particle if the current trial
     * particle is not unique.
     * @param [in] aParticleIndex index of particle to update
     * @param [in/out] aDataMng PSO data manager
     **********************************************************************************/
    void findUniqueParticleAndUpdate(const OrdinalType &aParticleIndex,
                                     Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> &aDataMng)
    {

        bool tIsParticleUnique = aDataMng.isParticleUnique(*mTrialControl);
        if (tIsParticleUnique == true)
        {
            aDataMng.setCurrentParticle(aParticleIndex, *mTrialControl);
        }
        else
        {
            this->computeUniqueParticle(aParticleIndex, aDataMng);
        }
    }

    /******************************************************************************//**
     * @brief Compute an unique particle that satisfies bounds.
     * @param [in] aParticleIndex index of particle to update
     * @param [in/out] aDataMng PSO data manager
     **********************************************************************************/
    void computeUniqueParticle(const OrdinalType &aParticleIndex,
                               Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> &aDataMng)
    {
        this->computeRandomStep(aDataMng.getLowerBounds(), aDataMng.getUpperBounds(), *mTrialControl);
        const Plato::Vector<ScalarType, OrdinalType> &tGlobalBestParticlePosition = aDataMng.getGlobalBestParticlePosition();
        mTrialControl->update(static_cast<ScalarType>(1), tGlobalBestParticlePosition, static_cast<ScalarType>(1));
        const OrdinalType tNumControls = tGlobalBestParticlePosition.size();
        for (OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            (*mTrialControl)[tIndex] = std::max((*mTrialControl)[tIndex], aDataMng.getLowerBounds()[tIndex]);
            (*mTrialControl)[tIndex] = std::min((*mTrialControl)[tIndex], aDataMng.getUpperBounds()[tIndex]);
        }
        aDataMng.setCurrentParticle(aParticleIndex, *mTrialControl);
    }

    /******************************************************************************//**
     * @brief Compute random step
     * @param [in] aLowerBounds lower bounds
     * @param [in] aUpperBounds upper bounds
     * @param [in/out] aOutput random step
     **********************************************************************************/
    void computeRandomStep(const Plato::Vector<ScalarType, OrdinalType> & aLowerBounds,
                           const Plato::Vector<ScalarType, OrdinalType> & aUpperBounds,
                           Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {
        std::uniform_real_distribution<ScalarType> tDistribution(0.0 /* lower bound */, 1.0 /* upper bound */);
        aOutput.update(static_cast<ScalarType>(1), aLowerBounds, static_cast<ScalarType>(0));
        aOutput.update(static_cast<ScalarType>(-1), aUpperBounds, static_cast<ScalarType>(1));

        const OrdinalType tNumControls = aOutput.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            const ScalarType tRandomNum = tDistribution(mGenerator) * mRandomNumMultiplier;
            aOutput[tIndex] *= tRandomNum;
        }
    }

private:
    std::default_random_engine mGenerator; /*!< random number generator */

    OrdinalType mNumConsecutiveFailures; /*!< number of consecutive failures, \f$ F(x_i) = F(x_{i-1}) \f$ */
    OrdinalType mNumConsecutiveSuccesses; /*!< number of consecutive successes, \f$ F(x_i) < F(x_{i-1}) \f$ */
    OrdinalType mMaxNumConsecutiveFailures; /*!< maximum number of consecutive failures, \f$ F(x_i) = F(x_{i-1}) \f$ */
    OrdinalType mMaxNumConsecutiveSuccesses; /*!< maximum number of consecutive successes, \f$ F(x_i) < F(x_{i-1}) \f$ */

    ScalarType mRandomNumMultiplier; /*!< random number multiplier (used to find unique particle position - see function findUniqueParticleAndUpdate) */
    ScalarType mInertiaMultiplier; /*!< inertia multiplier */
    ScalarType mSocialBehaviorMultiplier; /*!< social behavior multiplier */
    ScalarType mCognitiveBehaviorMultiplier; /*!< cognitive behavior multiplier */

    ScalarType mTrustRegionMultiplier; /*!< trust region multiplier */
    ScalarType mTrustRegionExpansionMultiplier; /*!< trust region expansion multiplier */
    ScalarType mTrustRegionContractionMultiplier; /*!< trust region contraction multiplier */

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mTrialControl; /*!< trial control vector */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mTrialVelocity; /*!< trial velocity vector */

private:
    ParticleSwarmOperations(const Plato::ParticleSwarmOperations<ScalarType, OrdinalType>&);
    Plato::ParticleSwarmOperations<ScalarType, OrdinalType>& operator=(const Plato::ParticleSwarmOperations<ScalarType, OrdinalType>&);
};
// class ParticleSwarmOperations

}// namespace Plato
