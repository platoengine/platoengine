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
 * Plato_ParticleSwarmDataMng.hpp
 *
 *  Created on: Jan 23, 2019
*/

#pragma once

#include <limits>
#include <memory>
#include <random>
#include <cassert>

#include "Plato_DataFactory.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_StatisticsUtils.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Data manager for all particle swarm optimization algorithms
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class ParticleSwarmDataMng
{
public:
    /******************************************************************************//**
     * @brief Constructor
    **********************************************************************************/
    explicit ParticleSwarmDataMng(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory) :
            mGlobalNumParticles(0),
            mCurrentGlobalBestParticleRank(0),
            mCurrentGlobalBestParticleIndex(0),
            mTimeStep(1),
            mMeanCurrentBestObjFunValue(std::numeric_limits<ScalarType>::max()),
            mMeanPreviousBestObjFunValue(std::numeric_limits<ScalarType>::max()),
            mStdDevCurrentBestObjFunValue(std::numeric_limits<ScalarType>::max()),
            mCurrentGlobalBestObjFuncValue(std::numeric_limits<ScalarType>::max()),
            mPreviousGlobalBestObjFuncValue(std::numeric_limits<ScalarType>::max()),
            mCurrentObjFuncValues(aFactory->objective().create()),
            mCurrentBestObjFuncValues(aFactory->objective().create()),
            mWorkVector(),
            mLowerBounds(),
            mUpperBounds(),
            mMeanCurrentBestParticlePositions(),
            mStdDevCurrentBestParticlePositions(),
            mGlobalBestParticlePositions(),
            mCurrentParticles(aFactory->control().create()),
            mCurrentVelocities(aFactory->control().create()),
            mPreviousVelocities(aFactory->control().create()),
            mCurrentBestParticlePositions(aFactory->control().create()),
            mCommWrapper(aFactory->getCommWrapper().create()),
            mCriteriaReductions(aFactory->getObjFuncReductionOperations().create())
    {
        this->initialize();
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~ParticleSwarmDataMng()
    {
    }

    /******************************************************************************//**
     * @brief Returns the local number of particles
     * @return local number of particles
    **********************************************************************************/
    OrdinalType getNumParticles() const
    {
        assert(static_cast<OrdinalType>(mCurrentParticles.use_count()) > static_cast<OrdinalType>(0));
        return (mCurrentParticles->getNumVectors());
    }

    /******************************************************************************//**
     * @brief Returns the rank that owns the current global best particle
     * @return rank that owns the current global best particle
    **********************************************************************************/
    OrdinalType getCurrentGlobalBestParticleRank() const
    {
        return (mCurrentGlobalBestParticleRank);
    }

    /******************************************************************************//**
     * @brief Returns particle index associated with the current global best particle
     * @return particle index
    **********************************************************************************/
    OrdinalType getCurrentGlobalBestParticleIndex() const
    {
        return (mCurrentGlobalBestParticleIndex);
    }

    /******************************************************************************//**
     * @brief Returns time step used to update particles' velocities
     * @return time step
    **********************************************************************************/
    ScalarType getTimeStep() const
    {
        return (mTimeStep);
    }

    /******************************************************************************//**
     * @brief Returns the mean of the current set of best objective function values
     * @return current mean value
    **********************************************************************************/
    ScalarType getMeanCurrentBestObjFuncValues() const
    {
        return (mMeanCurrentBestObjFunValue);
    }

    /******************************************************************************//**
     * @brief Returns the mean of the previous set of best objective function values
     * @return previous mean value
    **********************************************************************************/
    ScalarType getMeanPreviousBestObjFuncValues() const
    {
        return (mMeanPreviousBestObjFunValue);
    }

    /******************************************************************************//**
     * @brief Returns the standard deviation of the current set of best objective function values
     * @return current standard deviation value
    **********************************************************************************/
    ScalarType getStdDevCurrentBestObjFuncValues() const
    {
        return (mStdDevCurrentBestObjFunValue);
    }

    /******************************************************************************//**
     * @brief Returns the current global best objective function value
     * @return current global best objective function value
    **********************************************************************************/
    ScalarType getCurrentGlobalBestObjFuncValue() const
    {
        return (mCurrentGlobalBestObjFuncValue);
    }

    /******************************************************************************//**
     * @brief Returns the previous global best objective function value
     * @return previous global best objective function value
    **********************************************************************************/
    ScalarType getPreviousGlobalBestObjFuncValue() const
    {
        return (mPreviousGlobalBestObjFuncValue);
    }

    /******************************************************************************//**
     * @brief Returns current objective function value at this particle position
     * @param [in] aIndex particle position
     * @return current objective function value at this particle position
    **********************************************************************************/
    ScalarType getCurrentObjFuncValue(const OrdinalType & aIndex) const
    {
        assert(mCurrentObjFuncValues.get() != nullptr);
        return ((*mCurrentObjFuncValues)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Returns 1D container of current objective function values
     * @return const reference of the 1D container of current objective function values
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentObjFuncValues() const
    {
        assert(mCurrentObjFuncValues.get() != nullptr);
        return(*mCurrentObjFuncValues);
    }

    /******************************************************************************//**
     * @brief Returns 1D container of current best objective function values
     * @return const reference of the 1D container of current best objective function values
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentBestObjFuncValues() const
    {
        assert(mCurrentBestObjFuncValues.get() != nullptr);
        return(*mCurrentBestObjFuncValues);
    }

    /******************************************************************************//**
     * @brief Returns 1D container of particles' lower bounds
     * @return const reference of the 1D container of particles' lower bounds
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getLowerBounds() const
    {
        assert(static_cast<OrdinalType>(mLowerBounds.use_count()) > static_cast<OrdinalType>(0));
        return (*mLowerBounds);
    }

    /******************************************************************************//**
     * @brief Returns 1D container of particles' upper bounds
     * @return const reference of the 1D container of particles' upper bounds
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getUpperBounds() const
    {
        assert(static_cast<OrdinalType>(mUpperBounds.use_count()) > static_cast<OrdinalType>(0));
        return (*mUpperBounds);
    }

    /******************************************************************************//**
     * @brief Returns 1D container of mean particle positions
     * @return const reference of the 1D container of mean particle positions
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getMeanParticlePositions() const
    {
        assert(static_cast<OrdinalType>(mMeanCurrentBestParticlePositions.use_count()) > static_cast<OrdinalType>(0));
        return (*mMeanCurrentBestParticlePositions);
    }

    /******************************************************************************//**
     * @brief Returns 1D container with standard deviation for each particle position
     * @return const reference of the 1D container with standard deviation for each particle position
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getStdDevParticlePositions() const
    {
        assert(static_cast<OrdinalType>(mStdDevCurrentBestParticlePositions.use_count()) > static_cast<OrdinalType>(0));
        return (*mStdDevCurrentBestParticlePositions);
    }

    /******************************************************************************//**
     * @brief Returns the global best particle positions
     * @return const reference of the 1D container of global best particles positions
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getGlobalBestParticlePosition() const
    {
        assert(static_cast<OrdinalType>(mGlobalBestParticlePositions.use_count()) > static_cast<OrdinalType>(0));
        return (*mGlobalBestParticlePositions);
    }

    /******************************************************************************//**
     * @brief Returns 1D container of current particle positions at this location
     * @param [in] aIndex particle location
     * @return const reference of the 1D container of current particle positions at this location
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentParticle(const OrdinalType & aIndex) const
    {
        assert(static_cast<OrdinalType>(mCurrentParticles.use_count()) > static_cast<OrdinalType>(0));
        return ((*mCurrentParticles)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Returns 1D container of current particle velocities at this location
     * @param [in] aIndex particle location
     * @return const reference of the 1D container of current particle velocities at this location
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentVelocity(const OrdinalType & aIndex) const
    {
        assert(static_cast<OrdinalType>(mCurrentVelocities.use_count()) > static_cast<OrdinalType>(0));
        return ((*mCurrentVelocities)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Returns 1D container of previous particle velocities at this location
     * @param [in] aIndex particle location
     * @return const reference of the 1D container of previous particle velocities at this location
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getPreviousVelocity(const OrdinalType & aIndex) const
    {
        assert(static_cast<OrdinalType>(mPreviousVelocities.use_count()) > static_cast<OrdinalType>(0));
        return ((*mPreviousVelocities)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Returns 1D container with the current best particle positions at this particle index
     * @param [in] aIndex particle index
     * @return const reference to 1D container of current best particle positions
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getBestParticlePosition(const OrdinalType & aIndex) const
    {
        assert(static_cast<OrdinalType>(mCurrentBestParticlePositions.use_count()) > static_cast<OrdinalType>(0));
        return ((*mCurrentBestParticlePositions)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Returns 2D container of current particle positions
     * @return const reference of the 2D container of current particle positions
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentParticles() const
    {
        assert(static_cast<OrdinalType>(mCurrentParticles.use_count()) > static_cast<OrdinalType>(0));
        return (*mCurrentParticles);
    }

    /******************************************************************************//**
     * @brief Returns 2D container of current particle velocities
     * @return const reference of the 2D container of current particle velocities
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentVelocities() const
    {
        assert(static_cast<OrdinalType>(mCurrentVelocities.use_count()) > static_cast<OrdinalType>(0));
        return (*mCurrentVelocities);
    }

    /******************************************************************************//**
     * @brief Returns 2D container of previous particle velocities
     * @return const reference of the 2D container of previous particle velocities
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & getPreviousVelocities() const
    {
        assert(static_cast<OrdinalType>(mPreviousVelocities.use_count()) > static_cast<OrdinalType>(0));
        return (*mPreviousVelocities);
    }

    /******************************************************************************//**
     * @brief Set rank that owns the current global best particle
     * @param [in] aInput rank that owns the current global best particle
    **********************************************************************************/
    void setCurrentGlobalBestParticleRank(const OrdinalType & aInput)
    {
        mCurrentGlobalBestParticleRank = aInput;
    }

    /******************************************************************************//**
     * @brief Sets particle index associated with the current global best particle
     * @param [in] aInput particle index
    **********************************************************************************/
    void setCurrentGlobalBestParticleIndex(const OrdinalType & aInput)
    {
        mCurrentGlobalBestParticleIndex = aInput;
    }

    /******************************************************************************//**
     * @brief Sets time step used to update particle velocities
     * @param [in] aInput time step
    **********************************************************************************/
    void setTimeStep(const ScalarType & aInput)
    {
        mTimeStep = aInput;
    }

    /******************************************************************************//**
     * @brief Set current global best objective function value
     * @param [in] aInput objective function value
    **********************************************************************************/
    void setCurrentGlobalBestObjFunValue(const ScalarType & aInput)
    {
        mCurrentGlobalBestObjFuncValue = aInput;
    }

    /******************************************************************************//**
     * @brief Set current objective function values
     * @param [in] aInput 1D container of current objective function values
    **********************************************************************************/
    void setCurrentObjFuncValues(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        mCurrentObjFuncValues->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    /******************************************************************************//**
     * @brief Set current best objective function value for this particle
     * @param [in] aIndex particle index
     * @param [in] aInput current best objective function values
    **********************************************************************************/
    void setCurrentBestObjFuncValue(const OrdinalType & aIndex, const ScalarType & aInput)
    {
        (*mCurrentBestObjFuncValues)[aIndex] = aInput;
    }

    /******************************************************************************//**
     * @brief Set current best objective function values
     * @param [in] aInput 1D container of current best objective function values
    **********************************************************************************/
    void setCurrentBestObjFuncValues(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        mCurrentBestObjFuncValues->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    /******************************************************************************//**
     * @brief Set all the elements in the 2D container of current particles to this scalar value
     * @param [in] aInput scalar value
    **********************************************************************************/
    void setCurrentParticle(const ScalarType & aInput)
    {
        assert(static_cast<OrdinalType>(mCurrentParticles.use_count()) > static_cast<OrdinalType>(0));
        Plato::fill(aInput, *mCurrentParticles);
    }

    /******************************************************************************//**
     * @brief Set all the elements in the 1D container of this current particle to this scalar value.
     * @param [in] aIndex this particle index
     * @param [in] aInput this scalar value
    **********************************************************************************/
    void setCurrentParticle(const OrdinalType & aIndex, const ScalarType & aInput)
    {
        assert(static_cast<OrdinalType>(mCurrentParticles.use_count()) > static_cast<OrdinalType>(0));
        (*mCurrentParticles)[aIndex].fill(aInput);
    }

    /******************************************************************************//**
     * @brief Set this current particle to input 1D container
     * @param [in] aIndex this particle index
     * @param [in] aParticle 1D container of current particles
    **********************************************************************************/
    void setCurrentParticle(const OrdinalType & aIndex, const Plato::Vector<ScalarType, OrdinalType> & aParticle)
    {
        assert(static_cast<OrdinalType>(mCurrentParticles.use_count()) > static_cast<OrdinalType>(0));
        assert(aParticle.size() > static_cast<OrdinalType>(0));
        (*mCurrentParticles)[aIndex].update(static_cast<ScalarType>(1), aParticle, static_cast<ScalarType>(0));
    }

    /******************************************************************************//**
     * @brief Set Set 2D container of current particles to input 2D container
     * @param [in] aParticles 2D container of current particles
    **********************************************************************************/
    void setCurrentParticles(const Plato::MultiVector<ScalarType, OrdinalType> & aParticles)
    {
        assert(aParticles.getNumVectors() > static_cast<OrdinalType>(0));
        assert(static_cast<OrdinalType>(mCurrentParticles.use_count()) > static_cast<OrdinalType>(0));
        Plato::update(static_cast<ScalarType>(1), aParticles, static_cast<ScalarType>(0), *mCurrentParticles);
    }

    /******************************************************************************//**
     * @brief Set this current particle velocities to input 1D container
     * @param [in] aIndex this particle index
     * @param [in] aParticleVel 1D container of current particle velocities
    **********************************************************************************/
    void setCurrentVelocity(const OrdinalType & aIndex, const Plato::Vector<ScalarType, OrdinalType> & aParticleVel)
    {
        assert(static_cast<OrdinalType>(mCurrentVelocities.use_count()) > static_cast<OrdinalType>(0));
        assert(aParticleVel.size() > static_cast<OrdinalType>(0));
        (*mCurrentVelocities)[aIndex].update(static_cast<ScalarType>(1), aParticleVel, static_cast<ScalarType>(0));
    }

    /******************************************************************************//**
     * @brief Set 2D container of current particle velocities to input 2D container
     * @param [in] aInput 2D container of current particle velocities
    **********************************************************************************/
    void setCurrentVelocities(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() > static_cast<OrdinalType>(0));
        assert(static_cast<OrdinalType>(mCurrentVelocities.use_count()) > static_cast<OrdinalType>(0));
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mCurrentVelocities);
    }

    /******************************************************************************//**
     * @brief Set 2D container of previous particle velocities to input 2D container
     * @param [in] aInput 2D container of previous particle velocities
    **********************************************************************************/
    void setPreviousVelocities(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() > static_cast<OrdinalType>(0));
        assert(static_cast<OrdinalType>(mPreviousVelocities.use_count()) > static_cast<OrdinalType>(0));
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mPreviousVelocities);
    }

    /******************************************************************************//**
     * @brief Set all the elements in the 1D container of lower bounds to input scalar
     * @param [in] aInput input scalar
    **********************************************************************************/
    void setLowerBounds(const ScalarType & aInput)
    {
        assert(static_cast<OrdinalType>(mLowerBounds.use_count()) > static_cast<OrdinalType>(0));
        mLowerBounds->fill(aInput);
    }

    /******************************************************************************//**
     * @brief Set 1D container of particles' lower bounds
     * @param [in] aInput 1D container of particles' lower bounds
    **********************************************************************************/
    void setLowerBounds(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(static_cast<OrdinalType>(mLowerBounds.use_count()) > static_cast<OrdinalType>(0));
        assert(aInput.size() > static_cast<OrdinalType>(0));
        mLowerBounds->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    /******************************************************************************//**
     * @brief Set all the elements in the 1D container of upper bounds to input scalar
     * @param [in] aInput input scalar
    **********************************************************************************/
    void setUpperBounds(const ScalarType & aInput)
    {
        assert(static_cast<OrdinalType>(mUpperBounds.use_count()) > static_cast<OrdinalType>(0));
        mUpperBounds->fill(aInput);
    }

    /******************************************************************************//**
     * @brief Set 1D container of particles' upper bounds
     * @param [in] aInput 1D container of particles' upper bounds
    **********************************************************************************/
    void setUpperBounds(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(static_cast<OrdinalType>(mUpperBounds.use_count()) > static_cast<OrdinalType>(0));
        assert(aInput.size() > static_cast<OrdinalType>(0));
        mUpperBounds->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    /******************************************************************************//**
     * @brief Set 1D container of global best particle positions
     * @param [in] aInput 1D container of global best particle positions
    **********************************************************************************/
    void setGlobalBestParticlePosition(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        assert(static_cast<OrdinalType>(mGlobalBestParticlePositions.use_count()) > static_cast<OrdinalType>(0));
        mGlobalBestParticlePositions->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    /******************************************************************************//**
     * @brief Set 1D container of current best positions for this particle
     * @param [in] aIndex particle index
     * @param [in] aInput 1D container of best particle positions
    **********************************************************************************/
    void setBestParticlePosition(const OrdinalType & aIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput) const
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        assert(static_cast<OrdinalType>(mCurrentBestParticlePositions.use_count()) > static_cast<OrdinalType>(0));
        (*mCurrentBestParticlePositions)[aIndex].update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    /******************************************************************************//**
     * @brief Set 2D container of current best particle positions
     * @param [in] aInput 2D container of current best particle positions
    **********************************************************************************/
    void setBestParticlePositions(const Plato::MultiVector<ScalarType, OrdinalType> & aInput) const
    {
        assert(aInput.getNumVectors() > static_cast<OrdinalType>(0));
        assert(static_cast<OrdinalType>(mCurrentBestParticlePositions.use_count()) > static_cast<OrdinalType>(0));
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mCurrentBestParticlePositions);
    }

    /******************************************************************************//**
     * @brief Cache current global best objective function value
    **********************************************************************************/
    void cacheGlobalBestObjFunValue()
    {
        mPreviousGlobalBestObjFuncValue = mCurrentGlobalBestObjFuncValue;
    }

    /******************************************************************************//**
     * @brief Set initial particle positions (i.e. initial guess)
    **********************************************************************************/
    void setInitialParticles()
    {
        std::default_random_engine tGenerator;
        std::uniform_real_distribution<ScalarType> tDistribution(0.0 /* lower bound */, 1.0 /* upper bound */);

        const OrdinalType tNumParticles = this->getNumParticles();
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            mWorkVector->update(static_cast<ScalarType>(1), *mUpperBounds, static_cast<ScalarType>(0));
            mWorkVector->update(static_cast<ScalarType>(-1), *mLowerBounds, static_cast<ScalarType>(1));

            ScalarType tMyRandomNum = tDistribution(tGenerator);
            (*mCurrentParticles)[tIndex].update(static_cast<ScalarType>(1), *mLowerBounds, static_cast<ScalarType>(0));
            (*mCurrentParticles)[tIndex].update(tMyRandomNum, *mWorkVector, static_cast<ScalarType>(1));
            (*mCurrentVelocities)[tIndex].update(static_cast<ScalarType>(1), (*mCurrentParticles)[tIndex], static_cast<ScalarType>(0));
        }
    }

    /******************************************************************************//**
     * @brief Cache current particle velocities
    **********************************************************************************/
    void cachePreviousVelocities()
    {
        Plato::update(static_cast<ScalarType>(1), *mCurrentVelocities, static_cast<ScalarType>(0), *mPreviousVelocities);
    }

    /******************************************************************************//**
     * @brief Find global best particle positions
    **********************************************************************************/
    void findGlobalBestParticle()
    {
        Plato::ReductionOutputs<ScalarType, OrdinalType> tOutput;
        mCriteriaReductions->minloc(*mCurrentObjFuncValues, tOutput);
        const bool tFoundNewGlobalBest = tOutput.mOutputValue < mCurrentGlobalBestObjFuncValue;
        if(tFoundNewGlobalBest == true)
        {
            mCurrentGlobalBestObjFuncValue = tOutput.mOutputValue;
            mCurrentGlobalBestParticleRank = tOutput.mOutputRank;
            mCurrentGlobalBestParticleIndex = tOutput.mOutputIndex;
            const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticle = this->getCurrentParticle(tOutput.mOutputIndex);
            this->setGlobalBestParticlePosition(tCurrentParticle);
        }
    }

    /******************************************************************************//**
     * @brief Update current best particle positions and objective function values
    **********************************************************************************/
    void updateBestParticlesData()
    {
        const OrdinalType tNumParticles = this->getNumParticles();
        const Plato::Vector<ScalarType, OrdinalType> & tCurrentObjFuncValues = this->getCurrentObjFuncValues();
        const Plato::Vector<ScalarType, OrdinalType> & tCurrentBestObjFuncValues = this->getCurrentBestObjFuncValues();
        for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
        {
            const ScalarType tCurrentObjFuncVal = tCurrentObjFuncValues[tParticleIndex];
            if(tCurrentObjFuncVal < tCurrentBestObjFuncValues[tParticleIndex])
            {
                const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticle = this->getCurrentParticle(tParticleIndex);
                this->setBestParticlePosition(tParticleIndex, tCurrentParticle);
                this->setCurrentBestObjFuncValue(tParticleIndex, tCurrentObjFuncVal);
            }
        }
    }

    /******************************************************************************//**
     * @brief Compute current best particle positions statistics
    **********************************************************************************/
    void computeCurrentBestParticlesStatistics()
    {
        this->computeMeanBestParticlePositions();
        this->computeStdDevBestParticlePositions();
    }

    /******************************************************************************//**
     * @brief Compute current best objective function values statistics
    **********************************************************************************/
    void computeCurrentBestObjFuncStatistics()
    {
        mMeanPreviousBestObjFunValue = mMeanCurrentBestObjFunValue;
        mMeanCurrentBestObjFunValue = Plato::mean(*mCriteriaReductions, *mCurrentBestObjFuncValues);
        mStdDevCurrentBestObjFunValue = Plato::standard_deviation(mMeanCurrentBestObjFunValue, *mCurrentBestObjFuncValues, *mCriteriaReductions);
    }

    /******************************************************************************//**
     * @brief Return a const reference to the distributed memory communication wrapper
     * @return const reference to the distributed memory communication wrapper
    **********************************************************************************/
    const Plato::CommWrapper& getCommWrapper() const
    {
        return (mCommWrapper.operator*());
    }

private:
    /******************************************************************************//**
     * @brief Initialize/Allocate class member data
    **********************************************************************************/
    void initialize()
    {
        const OrdinalType tPARTICLE_INDEX = 0;
        const Plato::Vector<ScalarType, OrdinalType> & tMyParticle = (*mCurrentParticles)[tPARTICLE_INDEX];
        mWorkVector = tMyParticle.create();
        mLowerBounds = tMyParticle.create();
        mUpperBounds = tMyParticle.create();
        mMeanCurrentBestParticlePositions = tMyParticle.create();
        mStdDevCurrentBestParticlePositions = tMyParticle.create();

        mCurrentObjFuncValues->fill(std::numeric_limits<ScalarType>::max());
        mCurrentBestObjFuncValues->fill(std::numeric_limits<ScalarType>::max());

        mGlobalBestParticlePositions = tMyParticle.create();
        mGlobalBestParticlePositions->fill(std::numeric_limits<ScalarType>::max());
        Plato::fill(std::numeric_limits<ScalarType>::max(), *mCurrentBestParticlePositions);

        const OrdinalType tLength = 1;
        Plato::StandardVector<ScalarType, OrdinalType> tWork(tLength, this->getNumParticles());
        mGlobalNumParticles = mCriteriaReductions->sum(tWork);
    }

    /******************************************************************************//**
     * @brief Compute the mean of the current best particle positions
    **********************************************************************************/
    void computeMeanBestParticlePositions()
    {
        /* local sum */
        mWorkVector->fill(static_cast<ScalarType>(0));
        const OrdinalType tLocalNumParticles = this->getNumParticles();
        for(OrdinalType tParticleIndex = 0; tParticleIndex < tLocalNumParticles; tParticleIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyParticle = (*mCurrentBestParticlePositions)[tParticleIndex];
            const OrdinalType tMyParticleDims = tMyParticle.size();
            for(OrdinalType tDim = 0; tDim < tMyParticleDims; tDim++)
            {
                (*mWorkVector)[tDim] += tMyParticle[tDim];
            }
        }

        /* global sum */
        const OrdinalType tLength = 1;
        const OrdinalType tVECTOR_INDEX = 0;
        Plato::StandardVector<ScalarType, OrdinalType> tWork(tLength);
        const OrdinalType tNumDims = mWorkVector->size();
        for(OrdinalType tDim = 0; tDim < tNumDims; tDim++)
        {
            tWork[tVECTOR_INDEX] = (*mWorkVector)[tDim];
            (*mMeanCurrentBestParticlePositions)[tDim] = mCriteriaReductions->sum(tWork);
        }
        const ScalarType tMultiplier = static_cast<ScalarType>(1) / mGlobalNumParticles;
        mMeanCurrentBestParticlePositions->scale(tMultiplier);
    }

    /******************************************************************************//**
     * @brief Compute the standard deviation of the current best particle positions
    **********************************************************************************/
    void computeStdDevBestParticlePositions()
    {
        /* local operations */
        mWorkVector->fill(static_cast<ScalarType>(0));
        const OrdinalType tLocalNumParticles = this->getNumParticles();
        const OrdinalType tNumControls = mStdDevCurrentBestParticlePositions->size();
        for(OrdinalType tParticleIndex = 0; tParticleIndex < tLocalNumParticles; tParticleIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyBestParticlePosition = (*mCurrentBestParticlePositions)[tParticleIndex];
            for(OrdinalType tDim = 0; tDim < tNumControls; tDim++)
            {
                const ScalarType tMisfit = tMyBestParticlePosition[tDim] - (*mMeanCurrentBestParticlePositions)[tDim];
                (*mWorkVector)[tDim] += tMisfit * tMisfit;
            }
        }

        /* global operations */
        const OrdinalType tLength = 1;
        const OrdinalType tVECTOR_INDEX = 0;
        Plato::StandardVector<ScalarType, OrdinalType> tWork(tLength);
        for(OrdinalType tDim = 0; tDim < tNumControls; tDim++)
        {
            tWork[tVECTOR_INDEX] = (*mWorkVector)[tDim];
            const ScalarType tGlobalBestParticlePositionMinusMean = mCriteriaReductions->sum(tWork);
            const ScalarType tValue = tGlobalBestParticlePositionMinusMean / static_cast<ScalarType>(mGlobalNumParticles - 1);
            (*mStdDevCurrentBestParticlePositions)[tDim] = std::pow(tValue, static_cast<ScalarType>(0.5));
        }
    }

private:
    OrdinalType mGlobalNumParticles; /*!< global number of particles */
    OrdinalType mCurrentGlobalBestParticleRank; /*!< rank associated with global best particle */
    OrdinalType mCurrentGlobalBestParticleIndex; /*!< particle index associated with global best particle */

    ScalarType mTimeStep; /*!< time step needed to compute particle velocities */
    ScalarType mMeanCurrentBestObjFunValue; /*!< mean of current best objective function values */
    ScalarType mMeanPreviousBestObjFunValue; /*!< mean of previous best objective function values */
    ScalarType mStdDevCurrentBestObjFunValue; /*!< standard deviation of current best objective function values */
    ScalarType mCurrentGlobalBestObjFuncValue; /*!< current global best objective function value */
    ScalarType mPreviousGlobalBestObjFuncValue; /*!< previous global best objective function value */

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentObjFuncValues; /*!< current objective function values */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentBestObjFuncValues; /*!< current best objective function values */

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mWorkVector; /*!< particle work vector */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mLowerBounds; /*!< particle lower bounds */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mUpperBounds; /*!< particle upper bounds */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mMeanCurrentBestParticlePositions; /*!< mean of current best particle positions */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mStdDevCurrentBestParticlePositions; /*!< standard deviation of current best particle positions */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mGlobalBestParticlePositions; /*!< global best particle positions */

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentParticles; /*!< current particle positions */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentVelocities; /*!< current particle velocities */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousVelocities; /*!< previous particle velocities */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentBestParticlePositions; /*!< current best particle positions */

    std::shared_ptr<Plato::CommWrapper> mCommWrapper; /*!< parallel communicator wrapper */
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mCriteriaReductions; /*!< reduction operations for criteria (e.g. objective function) containers */

private:
    ParticleSwarmDataMng(const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType>&);
    Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & operator=(const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType>&);
};
// class ParticleSwarmDataMng

} // namespace Plato
