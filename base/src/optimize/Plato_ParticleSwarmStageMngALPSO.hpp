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
 * Plato_ParticleSwarmStageMngALPSO.hpp
 *
 *  Created on: Jan 23, 2019
 */

#pragma once

#include "Plato_DataFactory.hpp"
#include "Plato_GradFreeCriterion.hpp"
#include "Plato_GradFreeCriteriaList.hpp"
#include "Plato_ParticleSwarmDataMng.hpp"
#include "Plato_ParticleSwarmStageMng.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Augmented Lagrangian Particle Swarm Optimization (ALPSO) algorithm stage
 *        managers. The ALPSO stage coordinates calls to grad-free criteria (e.g.
 *        objective functions).
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class ParticleSwarmStageMngALPSO : public Plato::ParticleSwarmStageMng<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aFactory data factory
     * @param [in] aObjective gradient free objective function interface
     * @param [in] aConstraints list of gradient free constraint interfaces
    **********************************************************************************/
    explicit ParticleSwarmStageMngALPSO(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                                            const std::shared_ptr<Plato::GradFreeCriterion<ScalarType, OrdinalType>> & aObjective,
                                            const std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> & aConstraints) :
            mNumAugLagFuncEval(0),
            mMeanCurrentBestObjFuncValue(0),
            mStdDevCurrentBestObjFuncValue(0),
            mCurrentGlobalBestObjFuncValue(std::numeric_limits<ScalarType>::max()),
            mCurrentGlobalBestAugLagFuncValue(std::numeric_limits<ScalarType>::max()),
            mPenaltyExpansionMultiplier(2),
            mPenaltyMultiplierUpperBound(100),
            mPenaltyContractionMultiplier(0.5),
            mFeasibilityInexactnessTolerance(1e-4),
            mCriteriaWorkVec(aFactory->objective().create()),
            mCurrentObjFuncValues(aFactory->objective().create()),
            mCurrentBestObjFuncValues(aFactory->objective().create()),
            mPreviousBestObjFuncValues(aFactory->objective().create()),
            mCurrentAugLagFuncValues(aFactory->objective().create()),
            mCurrentBestAugLagFuncValues(aFactory->objective().create()),
            mMeanBestConstraintValues(std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(aConstraints->size())),
            mCurrentFeasibilityMeasure(aFactory->objective().create()),
            mPreviousFeasibilityMeasure(aFactory->objective().create()),
            mStdDevBestConstraintValues(std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(aConstraints->size())),
            mMeanCurrentPenaltyMultipliers(std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(aConstraints->size())),
            mStdDevCurrentPenaltyMultipliers(std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(aConstraints->size())),
            mMeanCurrentLagrangeMultipliers(std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(aConstraints->size())),
            mStdDevCurrentLagrangeMultipliers(std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(aConstraints->size())),
            mCurrentGlobalBestConstraintValues(std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(aConstraints->size())),
            mCurrentConstraintValues(aFactory->dual().create()),
            mCurrentPenaltyMultipliers(aFactory->dual().create()),
            mCurrentLagrangeMultipliers(aFactory->dual().create()),
            mCurrentBestConstraintValues(aFactory->dual().create()),
            mPreviousBestConstraintValues(aFactory->dual().create()),
            mObjective(aObjective),
            mConstraints(aConstraints),
            mCriteriaReductions(aFactory->getObjFuncReductionOperations().create())
    {
        this->initialize();
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~ParticleSwarmStageMngALPSO()
    {
    }

    /******************************************************************************//**
     * @brief Set expansion multiplier for penalty multipliers
     * @param [in] aInput expansion multiplier
    **********************************************************************************/
    void setPenaltyExpansionMultiplier(const ScalarType & aInput)
    {
        mPenaltyExpansionMultiplier = aInput;
    }

    /******************************************************************************//**
     * @brief Set contraction multiplier for penalty multipliers
     * @param [in] aInput contraction multiplier
    **********************************************************************************/
    void setPenaltyContractionMultiplier(const ScalarType & aInput)
    {
        mPenaltyContractionMultiplier = aInput;
    }

    /******************************************************************************//**
     * @brief Set upper bound on penalty multipliers
     * @param [in] aInput upper bound
    **********************************************************************************/
    void setPenaltyMultiplierUpperBound(const ScalarType & aInput)
    {
        mPenaltyMultiplierUpperBound = aInput;
    }

    /******************************************************************************//**
     * @brief Set feasibility inexactness tolerance
     * @param [in] aInput tolerance
    **********************************************************************************/
    void setFeasibilityInexactnessTolerance(const ScalarType & aInput)
    {
        mFeasibilityInexactnessTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Return number of constraints
     * @return number of constraints
    **********************************************************************************/
    OrdinalType getNumConstraints() const
    {
        return (mConstraints->size());
    }

    /******************************************************************************//**
     * @brief Return number of augmented Lagrangian function evaluations
     * @return number of function evaluations
    **********************************************************************************/
    OrdinalType getNumAugLagFuncEvaluations() const
    {
        return (mNumAugLagFuncEval);
    }

    /******************************************************************************//**
     * @brief Return mean of current best objective function values
     * @return mean value
    **********************************************************************************/
    ScalarType getMeanCurrentBestObjFuncValues() const
    {
        return (mMeanCurrentBestObjFuncValue);
    }

    /******************************************************************************//**
     * @brief Return standard deviation of current best objective function values
     * @return standard deviation value
    **********************************************************************************/
    ScalarType getStdDevCurrentBestObjFuncValues() const
    {
        return (mStdDevCurrentBestObjFuncValue);
    }

    /******************************************************************************//**
     * @brief Return current global best objective function value
     * @return global best objective function value
    **********************************************************************************/
    ScalarType getCurrentGlobalBestObjFuncValue() const
    {
        return (mCurrentGlobalBestObjFuncValue);
    }

    /******************************************************************************//**
     * @brief Return current objective function value
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getCurrentObjFuncValue(const OrdinalType & aIndex) const
    {
        assert(mCurrentObjFuncValues.get() != nullptr);
        return((*mCurrentObjFuncValues)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Return current best objective function value
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getCurrentBestObjFuncValue(const OrdinalType & aIndex) const
    {
        assert(mCurrentBestObjFuncValues.get() != nullptr);
        return((*mCurrentBestObjFuncValues)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Return previous best objective function value
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getPreviousBestObjFuncValue(const OrdinalType & aIndex) const
    {
        assert(mPreviousBestObjFuncValues.get() != nullptr);
        return((*mPreviousBestObjFuncValues)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Return mean of current penalty multipliers
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getMeanCurrentPenaltyMultipliers(const OrdinalType & aInput) const
    {
        assert(aInput < mMeanCurrentPenaltyMultipliers->size());
        return ((*mMeanCurrentPenaltyMultipliers)[aInput]);
    }

    /******************************************************************************//**
     * @brief Return standard deviation of current penalty multipliers
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getStdDevCurrentPenaltyMultipliers(const OrdinalType & aInput) const
    {
        assert(aInput < mStdDevCurrentPenaltyMultipliers->size());
        return ((*mStdDevCurrentPenaltyMultipliers)[aInput]);
    }

    /******************************************************************************//**
     * @brief Return mean of current Lagrange multipliers
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getMeanCurrentLagrangeMultipliers(const OrdinalType & aInput) const
    {
        assert(aInput < mMeanCurrentLagrangeMultipliers->size());
        return ((*mMeanCurrentLagrangeMultipliers)[aInput]);
    }

    /******************************************************************************//**
     * @brief Return standard deviation of current Lagrange multipliers
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getStdDevCurrentLagrangeMultipliers(const OrdinalType & aInput) const
    {
        assert(aInput < mStdDevCurrentLagrangeMultipliers->size());
        return ((*mStdDevCurrentLagrangeMultipliers)[aInput]);
    }

    /******************************************************************************//**
     * @brief Return mean of current best constraint values
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getMeanCurrentBestConstraintValues(const OrdinalType & aInput) const
    {
        assert(aInput < mMeanBestConstraintValues->size());
        return ((*mMeanBestConstraintValues)[aInput]);
    }

    /******************************************************************************//**
     * @brief Return standard deviation of current cconstraint values
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getStdDevCurrentBestConstraintValues(const OrdinalType & aInput) const
    {
        assert(aInput < mStdDevBestConstraintValues->size());
        return ((*mStdDevBestConstraintValues)[aInput]);
    }

    /******************************************************************************//**
     * @brief Return current global constraint values
     * @param [in] aIndex particle index
    **********************************************************************************/
    ScalarType getCurrentGlobalBestConstraintValue(const OrdinalType & aInput) const
    {
        assert(aInput < mCurrentGlobalBestConstraintValues->size());
        return ((*mCurrentGlobalBestConstraintValues)[aInput]);
    }

    /******************************************************************************//**
     * @brief Return penalty multiplier
     * @param [in] aConstraintIndex constraint index
     * @param [in] aParticleIndex particle index
    **********************************************************************************/
    ScalarType getPenaltyMultiplier(const OrdinalType & aConstraintIndex, const OrdinalType & aParticleIndex) const
    {
        assert(mCurrentPenaltyMultipliers.get() != nullptr);
        assert(aConstraintIndex < mCurrentPenaltyMultipliers->getNumVectors());
        return((*mCurrentPenaltyMultipliers)(aConstraintIndex, aParticleIndex));
    }

    /******************************************************************************//**
     * @brief Return Lagrange multiplier
     * @param [in] aConstraintIndex constraint index
     * @param [in] aParticleIndex particle index
    **********************************************************************************/
    ScalarType getLagrangeMultiplier(const OrdinalType & aConstraintIndex, const OrdinalType & aParticleIndex) const
    {
        assert(mCurrentLagrangeMultipliers.get() != nullptr);
        assert(aConstraintIndex < mCurrentLagrangeMultipliers->getNumVectors());
        return((*mCurrentLagrangeMultipliers)(aConstraintIndex, aParticleIndex));
    }

    /******************************************************************************//**
     * @brief Return current constraint value
     * @param [in] aConstraintIndex constraint index
     * @param [in] aParticleIndex particle index
    **********************************************************************************/
    ScalarType getCurrentConstraintValue(const OrdinalType & aConstraintIndex, const OrdinalType & aParticleIndex) const
    {
        assert(mCurrentConstraintValues.get() != nullptr);
        assert(aConstraintIndex < mCurrentConstraintValues->getNumVectors());
        return((*mCurrentConstraintValues)(aConstraintIndex, aParticleIndex));
    }

    /******************************************************************************//**
     * @brief Return current best constraint value
     * @param [in] aConstraintIndex constraint index
     * @param [in] aParticleIndex particle index
    **********************************************************************************/
    ScalarType getCurrentBestConstraintValue(const OrdinalType & aConstraintIndex, const OrdinalType & aParticleIndex) const
    {
        assert(mCurrentBestConstraintValues.get() != nullptr);
        assert(aConstraintIndex < mCurrentBestConstraintValues->getNumVectors());
        return((*mCurrentBestConstraintValues)(aConstraintIndex, aParticleIndex));
    }

    /******************************************************************************//**
     * @brief Return previous best constraint value
     * @param [in] aConstraintIndex constraint index
     * @param [in] aParticleIndex particle index
    **********************************************************************************/
    ScalarType getPreviousBestConstraintValue(const OrdinalType & aConstraintIndex, const OrdinalType & aParticleIndex) const
    {
        assert(mPreviousBestConstraintValues.get() != nullptr);
        assert(aConstraintIndex < mPreviousBestConstraintValues->getNumVectors());
        return((*mPreviousBestConstraintValues)(aConstraintIndex, aParticleIndex));
    }

    /******************************************************************************//**
     * @brief Return 1D container of current best constraint means
     * @return const reference to the 1D container of current best constraint means
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getMeanCurrentBestConstraintValues() const
    {
        assert(mMeanBestConstraintValues.get() != nullptr);
        return(*mMeanBestConstraintValues);
    }

    /******************************************************************************//**
     * @brief Return 1D container of current best constraint standard deviations
     * @return const reference of the 1D container of current best constraint standard deviations
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getStdDevCurrentBestConstraintValues() const
    {
        assert(mStdDevBestConstraintValues.get() != nullptr);
        return(*mStdDevBestConstraintValues);
    }

    /******************************************************************************//**
     * @brief Return 1D container of current global best constraint values
     * @return const reference to the 1D container of current global best constraint values
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentGlobalBestConstraintValues() const
    {
        assert(mCurrentGlobalBestConstraintValues.get() != nullptr);
        return(*mCurrentGlobalBestConstraintValues);
    }

    /******************************************************************************//**
     * @brief Return 1D container of current objective function values
     * @return const reference to the 1D container of current objective function values
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentObjFuncValues() const
    {
        assert(mCurrentObjFuncValues.get() != nullptr);
        return(*mCurrentObjFuncValues);
    }

    /******************************************************************************//**
     * @brief Return 1D container of previous objective function values
     * @return const reference to the 1D container of previous objective function values
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getPreviousBestObjFuncValues() const
    {
        assert(mPreviousBestObjFuncValues.get() != nullptr);
        return(*mPreviousBestObjFuncValues);
    }

    /******************************************************************************//**
     * @brief Return 1D container of current best objective function values
     * @return const reference to the 1D container of current best objective function values
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentBestObjFuncValues() const
    {
        assert(mCurrentBestObjFuncValues.get() != nullptr);
        return(*mCurrentBestObjFuncValues);
    }

    /******************************************************************************//**
     * @brief Return 1D container of current constraint values
     * @param [in] aIndex constraint index
     * @return const reference to the 1D container of current constraint values
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentConstraintValues(const OrdinalType & aIndex) const
    {
        assert(mCurrentConstraintValues.get() != nullptr);
        assert(aIndex < mCurrentConstraintValues->getNumVectors());
        return((*mCurrentConstraintValues)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Return 1D container of current best constraint values
     * @param [in] aIndex constraint index
     * @return const reference to the 1D container of current best constraint values
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentBestConstraintValues(const OrdinalType & aIndex) const
    {
        assert(mCurrentBestConstraintValues.get() != nullptr);
        assert(aIndex < mCurrentBestConstraintValues->getNumVectors());
        return((*mCurrentBestConstraintValues)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Return 1D container of previous best constraint values
     * @param [in] aIndex constraint index
     * @return const reference to the 1D container of previous best constraint values
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getPreviousBestConstraintValues(const OrdinalType & aIndex) const
    {
        assert(mPreviousBestConstraintValues.get() != nullptr);
        assert(aIndex < mPreviousBestConstraintValues->getNumVectors());
        return((*mPreviousBestConstraintValues)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Return 1D container of current penalty multipliers
     * @param [in] aIndex constraint index
     * @return const reference to the 1D container of current penalty multipliers
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getPenaltyMultipliers(const OrdinalType & aIndex) const
    {
        assert(mCurrentPenaltyMultipliers.get() != nullptr);
        assert(aIndex < mCurrentPenaltyMultipliers->getNumVectors());
        return((*mCurrentPenaltyMultipliers)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Return 1D container of current Lagrange multipliers
     * @param [in] aIndex constraint index
     * @return const reference to the 1D container of current Lagrange multipliers
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getLagrangeMultipliers(const OrdinalType & aIndex) const
    {
        assert(mCurrentLagrangeMultipliers.get() != nullptr);
        assert(aIndex < mCurrentLagrangeMultipliers->getNumVectors());
        return((*mCurrentLagrangeMultipliers)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Find current best particle positions
     * @param [in] aDataMng PSO data manager
    **********************************************************************************/
    void findBestParticlePositions(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        aDataMng.cacheGlobalBestObjFunValue();
        this->computeFeasibilityMeasure();
        this->updateBestParticlesData(aDataMng);
        aDataMng.findGlobalBestParticle();
    }

    /******************************************************************************//**
     * @brief Evaluate augmented Lagrangian objective function
     * @param [in] aDataMng PSO data manager
    **********************************************************************************/
    void evaluateObjective(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        this->cacheCriteriaValues();
        // Evaluate objective function
        mNumAugLagFuncEval += aDataMng.getNumParticles();
        mCurrentObjFuncValues->fill(static_cast<ScalarType>(0));
        const Plato::MultiVector<ScalarType, OrdinalType> & tParticles = aDataMng.getCurrentParticles();
        mObjective->value(tParticles, *mCurrentObjFuncValues);
        // Evaluate constraints
        this->evaluateConstraints(tParticles);
        // Evaluate augmented Lagrangian
        mCurrentAugLagFuncValues->update(static_cast<ScalarType>(1), *mCurrentObjFuncValues, static_cast<ScalarType>(0));
        this->evaluateAugmentedLagrangian();
        aDataMng.setCurrentObjFuncValues(*mCurrentAugLagFuncValues);
    }

    /******************************************************************************//**
     * @brief Update current set of Lagrange multipliers
    **********************************************************************************/
    void updateLagrangeMultipliers()
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tPenaltyValues = (*mCurrentPenaltyMultipliers)[tConstraintIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tBestConstraintValues = (*mCurrentBestConstraintValues)[tConstraintIndex];
            Plato::Vector<ScalarType, OrdinalType> & tLagrangeMultipliers = (*mCurrentLagrangeMultipliers)[tConstraintIndex];

            const OrdinalType tNumParticles = tBestConstraintValues.size();
            for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
            {
                const ScalarType tLagMultiplierOverPenalty = -tLagrangeMultipliers[tParticleIndex]
                        / (static_cast<ScalarType>(2) * tPenaltyValues[tParticleIndex]);
                const ScalarType tSuggestedConstraintValue = std::max(tBestConstraintValues[tParticleIndex], tLagMultiplierOverPenalty);
                ScalarType tNewLagMultiplier = tLagrangeMultipliers[tParticleIndex] +
                        (static_cast<ScalarType>(2) * tPenaltyValues[tParticleIndex] * tSuggestedConstraintValue);
                tNewLagMultiplier = std::max(tNewLagMultiplier, static_cast<ScalarType>(0));
                tLagrangeMultipliers[tParticleIndex] = tNewLagMultiplier;
            }
        }
    }

    /******************************************************************************//**
     * @brief Update current set of penalty multipliers
    **********************************************************************************/
    void updatePenaltyMultipliers()
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tCurrentBestConstraint = (*mCurrentBestConstraintValues)[tConstraintIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tPreviousBestConstraint = (*mPreviousBestConstraintValues)[tConstraintIndex];
            Plato::Vector<ScalarType, OrdinalType> & tPenaltyMultipliers = (*mCurrentPenaltyMultipliers)[tConstraintIndex];
            Plato::Vector<ScalarType, OrdinalType> & tLagrangeMultipliers = (*mCurrentLagrangeMultipliers)[tConstraintIndex];

            const OrdinalType tNumParticles = tPenaltyMultipliers.size();
            for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
            {
                const bool tIsConstraintGreaterThanPrevious = tCurrentBestConstraint[tParticleIndex] > tPreviousBestConstraint[tParticleIndex];
                const bool tIsConstraintGreaterThanFeasibilityTol = tCurrentBestConstraint[tParticleIndex] > mFeasibilityInexactnessTolerance;
                tPenaltyMultipliers[tParticleIndex] = tIsConstraintGreaterThanPrevious || tIsConstraintGreaterThanFeasibilityTol ?
                        mPenaltyExpansionMultiplier * tPenaltyMultipliers[tParticleIndex] : tPenaltyMultipliers[tParticleIndex];

                const bool tIsConstraintLessThanFeasibilityTol = tCurrentBestConstraint[tParticleIndex] <= mFeasibilityInexactnessTolerance;
                tPenaltyMultipliers[tParticleIndex] = tIsConstraintLessThanFeasibilityTol ?
                        mPenaltyContractionMultiplier * tPenaltyMultipliers[tParticleIndex] : tPenaltyMultipliers[tParticleIndex];

                const ScalarType tLagrangeMultiplierOverFeasibilityTolerance =
                        tLagrangeMultipliers[tParticleIndex] / mFeasibilityInexactnessTolerance;
                const ScalarType tLowerBound = static_cast<ScalarType>(0.5) *
                        std::pow(tLagrangeMultiplierOverFeasibilityTolerance, static_cast<ScalarType>(0.5));
                tPenaltyMultipliers[tParticleIndex] = std::max(tPenaltyMultipliers[tParticleIndex], tLowerBound);
                tPenaltyMultipliers[tParticleIndex] = std::min(tPenaltyMultipliers[tParticleIndex], mPenaltyMultiplierUpperBound);
            }
        }
    }

    /******************************************************************************//**
     * @brief Compute objective and constraint statistics
    **********************************************************************************/
    void computeCriteriaStatistics()
    {
        this->computeObjFuncStatistics();
        this->computeDualDataStatistics();
        this->computeConstraintStatistics();
    }

    /******************************************************************************//**
     * @brief Re-set elements in objective and constraint containers to initial guess
    **********************************************************************************/
    void restart()
    {
        this->initializeObjFuncData();
        this->initializeConstraintData();
    }

private:
    /******************************************************************************//**
     * @brief Initialize objective and constraint class member containers
    **********************************************************************************/
    void initialize()
    {
        this->initializeObjFuncData();
        this->initializeMultipliersData();
        this->initializeConstraintData();
    }

    /******************************************************************************//**
     * @brief Initialize elements in objective function containers to initial guess
    **********************************************************************************/
    void initializeObjFuncData()
    {
        mCurrentObjFuncValues->fill(std::numeric_limits<ScalarType>::max());
        mCurrentBestObjFuncValues->fill(std::numeric_limits<ScalarType>::max());
        mCurrentAugLagFuncValues->fill(std::numeric_limits<ScalarType>::max());
        mCurrentBestAugLagFuncValues->fill(std::numeric_limits<ScalarType>::max());
    }

    /******************************************************************************//**
     * @brief Initialize elements in constraint containers to initial guess
    **********************************************************************************/
    void initializeConstraintData()
    {
        mCurrentFeasibilityMeasure->fill(std::numeric_limits<ScalarType>::max());
        mPreviousFeasibilityMeasure->fill(std::numeric_limits<ScalarType>::max());
        mCurrentGlobalBestConstraintValues->fill(std::numeric_limits<ScalarType>::max());

        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            (*mCurrentConstraintValues)[tIndex].fill(std::numeric_limits<ScalarType>::max());
            (*mCurrentBestConstraintValues)[tIndex].fill(std::numeric_limits<ScalarType>::max());
        }
    }

    /******************************************************************************//**
     * @brief Allocate penalty and Lagrange multipliers containers
    ***********************************************************************************/
    void initializeMultipliersData()
    {
        Plato::fill(static_cast<ScalarType>(1), *mCurrentPenaltyMultipliers);
        Plato::fill(static_cast<ScalarType>(0), *mCurrentLagrangeMultipliers);
    }

    /******************************************************************************//**
     * @brief Cache current constraint and objective values
    ***********************************************************************************/
    void cacheCriteriaValues()
    {
        mPreviousBestObjFuncValues->update(static_cast<ScalarType>(1), *mCurrentBestObjFuncValues, static_cast<ScalarType>(0));
        Plato::update(static_cast<ScalarType>(1), *mCurrentBestConstraintValues, static_cast<ScalarType>(0), *mPreviousBestConstraintValues);
    }

    /******************************************************************************//**
     * @brief Compute feasibility measure \f$ \max(h_i)\ \forall i=1,\dots,N_c\f$,
     *        where \f$ h \f$ denotes the constraint and \f$ N_c \f$ is the total
     *        number of constraints. The feasibility measure is computed for each
     *        particle.
    ***********************************************************************************/
    void computeFeasibilityMeasure()
    {
        mPreviousFeasibilityMeasure->update(static_cast<ScalarType>(1), *mCurrentFeasibilityMeasure, static_cast<ScalarType>(0));
        mCurrentFeasibilityMeasure->fill(static_cast<ScalarType>(0));

        const OrdinalType tNumConstraints = mConstraints->size();
        const OrdinalType tNumParticles = mCurrentBestObjFuncValues->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyConstraint = (*mCurrentConstraintValues)[tConstraintIndex];
            mCriteriaWorkVec->update(static_cast<ScalarType>(1), tMyConstraint, static_cast<ScalarType>(0));
            mCriteriaWorkVec->modulus();
            for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
            {
                const ScalarType tMyConstraintValue = (*mCriteriaWorkVec)[tParticleIndex];
                const ScalarType tMyFeasibilityMeasure = (*mCurrentFeasibilityMeasure)[tParticleIndex];
                (*mCurrentFeasibilityMeasure)[tParticleIndex] = std::max(tMyConstraintValue, tMyFeasibilityMeasure);
            }
        }
    }

    /******************************************************************************//**
     * @brief Update best particle positions and criteria values
     * @param [in,out] aDataMng PSO algorithm data manager
    ***********************************************************************************/
    void updateBestParticlesData(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const OrdinalType tNumParticles = aDataMng.getNumParticles();
        for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
        {
            const bool tAugLagFuncReduced =
                    (*mCurrentAugLagFuncValues)[tParticleIndex] < (*mCurrentBestAugLagFuncValues)[tParticleIndex];
            const bool tFeasibilityImproved =
                    (*mCurrentFeasibilityMeasure)[tParticleIndex] < (*mPreviousFeasibilityMeasure)[tParticleIndex];
            if(tAugLagFuncReduced == true && tFeasibilityImproved == true)
            //if(tAugLagFuncReduced == true)
            {
                const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticle = aDataMng.getCurrentParticle(tParticleIndex);
                aDataMng.setBestParticlePosition(tParticleIndex, tCurrentParticle);
                this->updateBestCriteriaValues(tParticleIndex, aDataMng);
            }
        }
    }

    /******************************************************************************//**
     * @brief Update best particle objective and constraint value containers
     * @param [in] aParticleIndex particle index
     * @param [in] aDataMng PSO algorithm data manager
    ***********************************************************************************/
    void updateBestCriteriaValues(const OrdinalType & aParticleIndex,
                                  Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        (*mCurrentBestAugLagFuncValues)[aParticleIndex] = (*mCurrentAugLagFuncValues)[aParticleIndex];
        aDataMng.setCurrentBestObjFuncValue(aParticleIndex, (*mCurrentBestAugLagFuncValues)[aParticleIndex]);
        (*mCurrentBestObjFuncValues)[aParticleIndex] = (*mCurrentObjFuncValues)[aParticleIndex];
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            (*mCurrentBestConstraintValues)(tConstraintIndex, aParticleIndex) =
                    (*mCurrentConstraintValues)(tConstraintIndex, aParticleIndex);
        }
    }

    /******************************************************************************//**
     * @brief Compute current best objective function statistics
    ***********************************************************************************/
    void computeObjFuncStatistics()
    {
        mMeanCurrentBestObjFuncValue = Plato::mean(*mCriteriaReductions, *mCurrentBestObjFuncValues);
        mStdDevCurrentBestObjFuncValue =
                Plato::standard_deviation(mMeanCurrentBestObjFuncValue, *mCurrentBestObjFuncValues, *mCriteriaReductions);
        const ScalarType tValue = mCriteriaReductions->min(*mCurrentBestObjFuncValues);
        const bool tFoundNewGlobalBest = tValue < mCurrentGlobalBestObjFuncValue;
        mCurrentGlobalBestObjFuncValue = tFoundNewGlobalBest == true ? tValue : mCurrentGlobalBestObjFuncValue;
    }

    /******************************************************************************//**
     * @brief Compute current penalty and Lagrange multipliers statistics
    ***********************************************************************************/
    void computeDualDataStatistics()
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyPenaltyMultipliers = (*mCurrentPenaltyMultipliers)[tIndex];
            ScalarType tMean = Plato::mean(*mCriteriaReductions, tMyPenaltyMultipliers);
            (*mMeanCurrentPenaltyMultipliers)[tIndex] = tMean;
            (*mStdDevCurrentPenaltyMultipliers)[tIndex] =
                    Plato::standard_deviation(tMean, tMyPenaltyMultipliers, *mCriteriaReductions);

            const Plato::Vector<ScalarType, OrdinalType> & tMyLagrangeMultipliers = (*mCurrentPenaltyMultipliers)[tIndex];
            tMean = Plato::mean(*mCriteriaReductions, tMyLagrangeMultipliers);
            (*mMeanCurrentLagrangeMultipliers)[tIndex] = tMean;
            (*mStdDevCurrentLagrangeMultipliers)[tIndex] =
                    Plato::standard_deviation(tMean, tMyLagrangeMultipliers, *mCriteriaReductions);
        }
    }

    /******************************************************************************//**
     * @brief Compute current best constraint statistics
    ***********************************************************************************/
    void computeConstraintStatistics()
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            Plato::Vector<ScalarType, OrdinalType> & tMyBestConstraintValues = (*mCurrentBestConstraintValues)[tIndex];
            const ScalarType tMean = Plato::mean(*mCriteriaReductions, tMyBestConstraintValues);
            (*mMeanBestConstraintValues)[tIndex] = tMean;
            (*mStdDevBestConstraintValues)[tIndex] =
                    Plato::standard_deviation(tMean, tMyBestConstraintValues, *mCriteriaReductions);

            const ScalarType tValue = mCriteriaReductions->min(tMyBestConstraintValues);
            const bool tFoundNewGlobalBest = tValue < (*mCurrentGlobalBestConstraintValues)[tIndex];
            (*mCurrentGlobalBestConstraintValues)[tIndex] =
                    tFoundNewGlobalBest == true ? tValue : (*mCurrentGlobalBestConstraintValues)[tIndex];
        }
    }

    /******************************************************************************//**
     * @brief Evaluate inequality constraints
     * @param [in] aControl control multi-vector
    **********************************************************************************/
    void evaluateConstraints(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            Plato::GradFreeCriterion<ScalarType, OrdinalType> & tConstraint = (*mConstraints)[tIndex];
            Plato::Vector<ScalarType, OrdinalType> & tMyValues = (*mCurrentConstraintValues)[tIndex];
            tMyValues.fill(static_cast<ScalarType>(0));
            tConstraint.value(aControl, tMyValues);
        }
    }

    /******************************************************************************//**
     * @brief Evaluate augmented Lagrangian criterion:
     *
     *        \f$ \ell(z, \lambda, \mu) = f(z) + sum_{i=1}^{N}\lambda_i \theta_i(z)
     *        + sum_{i=1}^{N}\mu_i \theta_i^2(z) \f$, where \f$ \theta_i =
     *        \max( h_i(x), \frac{-\lambda_i}{2\mu_i} ) \f$
     *
     * @param [in,out] aOutput augmented Lagrangian values
    **********************************************************************************/
    void evaluateAugmentedLagrangian()
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tConstraintValues = (*mCurrentConstraintValues)[tConstraintIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tPenaltyMultipliers = (*mCurrentPenaltyMultipliers)[tConstraintIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tLagrangeMultipliers = (*mCurrentLagrangeMultipliers)[tConstraintIndex];

            const OrdinalType tNumParticles = mCurrentObjFuncValues->size();
            for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
            {
                const ScalarType tLagMultiplierOverPenalty = -tLagrangeMultipliers[tParticleIndex]
                        / (static_cast<ScalarType>(2) * tPenaltyMultipliers[tParticleIndex]);
                ScalarType tSuggestedConstraintValue = std::max(tConstraintValues[tParticleIndex], tLagMultiplierOverPenalty);
                const bool tIsConstraintFeasible = tSuggestedConstraintValue < static_cast<ScalarType>(0);
                tSuggestedConstraintValue = tIsConstraintFeasible == true ? static_cast<ScalarType>(0) : tSuggestedConstraintValue;
                const ScalarType tLagrangeMultipliersTimesConstraint = tLagrangeMultipliers[tParticleIndex] * tSuggestedConstraintValue;
                const ScalarType tConstraintTimesConstraint = tSuggestedConstraintValue * tSuggestedConstraintValue;
                const ScalarType tMyConstraintContribution = tLagrangeMultipliersTimesConstraint
                        + tPenaltyMultipliers[tParticleIndex] * tConstraintTimesConstraint;
                (*mCurrentAugLagFuncValues)[tParticleIndex] += tMyConstraintContribution;
            }
        }
    }

private:
    OrdinalType mNumAugLagFuncEval; /*!< number of augmented Lagrangian function evaluations */

    ScalarType mMeanCurrentBestObjFuncValue; /*!< mean of current best objective function values */
    ScalarType mStdDevCurrentBestObjFuncValue; /*!< standard deviation of current best objective function values */
    ScalarType mCurrentGlobalBestObjFuncValue; /*!< current global best objective function value */
    ScalarType mCurrentGlobalBestAugLagFuncValue; /*!< current global best augmented Lagrangian function value */

    ScalarType mPenaltyExpansionMultiplier; /*!< expansion multiplier for penalty multipliers */
    ScalarType mPenaltyMultiplierUpperBound; /*!< upper bound on penalty multipliers */
    ScalarType mPenaltyContractionMultiplier; /*!< contraction multiplier for penalty multipliers */
    ScalarType mFeasibilityInexactnessTolerance; /*!< feasibility inexactness tolerance */

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCriteriaWorkVec; /*!< feasibility inexactness tolerance */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentObjFuncValues; /*!< current objective function values */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentBestObjFuncValues; /*!< current best objective function values */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mPreviousBestObjFuncValues; /*!< previous best objective function values */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentAugLagFuncValues; /*!< current augmented Lagrangian function values */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentBestAugLagFuncValues; /*!< current best augmented Lagrangian function values */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mMeanBestConstraintValues; /*!< mean of current best constraint values */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentFeasibilityMeasure; /*!< current feasibility measure */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mPreviousFeasibilityMeasure; /*!< previous feasibility measure */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mStdDevBestConstraintValues; /*!< standard deviation of current best constraint values */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mMeanCurrentPenaltyMultipliers; /*!< mean of current penalty multipliers */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mStdDevCurrentPenaltyMultipliers; /*!< standard deviation of current penalty multipliers */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mMeanCurrentLagrangeMultipliers; /*!< mean of current Lagrange multipliers */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mStdDevCurrentLagrangeMultipliers; /*!< standard deviation of current Lagrange multipliers */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentGlobalBestConstraintValues;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentConstraintValues; /*!< current constraint values */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentPenaltyMultipliers; /*!< current penalty multipliers */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentLagrangeMultipliers; /*!< current Lagrange multipliers */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentBestConstraintValues; /*!< current best constraint values */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousBestConstraintValues; /*!< previous best constraint values */

    std::shared_ptr<Plato::GradFreeCriterion<ScalarType, OrdinalType>> mObjective; /*!< objective criterion interface */
    std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> mConstraints; /*!< constraint criterion interface */
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mCriteriaReductions; /*!< interface to parallel reduction operations associated with a criterion */

private:
    ParticleSwarmStageMngALPSO(const Plato::ParticleSwarmStageMngALPSO<ScalarType, OrdinalType>&);
    Plato::ParticleSwarmStageMngALPSO<ScalarType, OrdinalType> & operator=(const Plato::ParticleSwarmStageMngALPSO<ScalarType, OrdinalType>&);
};
// class ParticleSwarmStageMngALPSO

} // namespace Plato
