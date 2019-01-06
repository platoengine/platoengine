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
 //S
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
 * Plato_Test_ParticleSwarm.cpp
 *
 *  Created on: Dec 26, 2018
 */

#include "gtest/gtest.h"

#include <cmath>
#include <limits>
#include <random>
#include <iomanip>
#include <fstream>

#include "Plato_DataFactory.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_UnitTestUtils.hpp"

namespace Plato
{

struct particle_swarm
{
    enum stop_t
    {
        DID_NOT_CONVERGE = 1,
        MAX_NUMBER_ITERATIONS = 2,
        TRUE_OBJECTIVE_TOLERANCE = 3,
        MEAN_OBJECTIVE_TOLERANCE = 4,
    };
};

/******************************************************************************//**
 * @brief Diagnostic data for the Particle Swarm Optimization (PSO) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct OutputDataPSO
{
    OrdinalType mNumIter;  /*!< number of outer iterations */
    OrdinalType mObjFuncCount;  /*!< number of objective function evaluations */

    ScalarType mBestObjFuncValue;  /*!< best - objective function value */
    ScalarType mMeanObjFuncValue;  /*!< mean - objective function value */
    ScalarType mStdDevObjFuncValue;  /*!< standard deviation - objective function value */
    ScalarType mTrustRegionMultiplier;  /*!< trust region multiplier */
};
// struct OutputDataPSO

namespace pso
{

/******************************************************************************//**
 * @brief Check if output file is open. An error is thrown if output file is not open.
 * @param [in] aOutputFile output file
**********************************************************************************/
template<typename Type>
void is_file_open(const Type & aOutputFile)
{
    try
    {
        if(aOutputFile.is_open() == false)
        {
            throw std::invalid_argument("\n\n ******** MESSAGE: OUTPUT FILE IS NOT OPEN. ABORT! ******** \n\n");
        }
    }
    catch(const std::invalid_argument & tError)
    {
        throw tError;
    }
}

/******************************************************************************//**
 * @brief Output a brief sentence explaining why the optimizer stopped.
 * @param [in] aStopCriterion stopping criterion flag
 * @param [in,out] aOutput string with brief description
 **********************************************************************************/
inline void get_stop_criterion(const Plato::particle_swarm::stop_t & aCriterion, std::string & aOutput)
{
    aOutput.clear();
    switch(aCriterion)
    {
        case Plato::particle_swarm::stop_t::DID_NOT_CONVERGE:
        {
            aOutput = "\n\n****** Optimization algorithm did not converge. ******\n\n";
            break;
        }
        case Plato::particle_swarm::stop_t::MAX_NUMBER_ITERATIONS:
        {
            aOutput = "\n\n****** Optimization stopping due to exceeding maximum number of iterations. ******\n\n";
            break;
        }
        case Plato::particle_swarm::stop_t::TRUE_OBJECTIVE_TOLERANCE:
        {
            aOutput = "\n\n****** Optimization stopping due to global best objective function tolerance being met. ******\n\n";
            break;
        }
        case Plato::particle_swarm::stop_t::MEAN_OBJECTIVE_TOLERANCE:
        {
            aOutput = "\n\n****** Optimization stopping due to mean objective function tolerance being met. ******\n\n";
            break;
        }
    }
}

/******************************************************************************//**
 * @brief Print header for Kelley-Sachs-Bound-Constrained (KSBC) diagnostics file
 * @param [in,out] aOutputFile output file
 * @param [in] aPrint flag use to enable/disable output (default = disabled)
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void print_pso_diagnostics_header(const Plato::OutputDataPSO<ScalarType, OrdinalType>& aData,
                                         std::ofstream& aOutputFile,
                                         bool aPrint = false)
{
    try
    {
        Plato::pso::is_file_open(aOutputFile);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {

        std::ostringstream tMessage;
        tMessage << "\n\n ******** ERROR IN FILE: " << __FILE__ << ", FUNCTION: " << __PRETTY_FUNCTION__
        << ", LINE: " << __LINE__ << " ******** \n\n";
        tMessage << tErrorMsg.what();
        if(aPrint == true)
        {
            std::cout << tMessage.str().c_str() << std::flush;
        }
        throw std::invalid_argument(tMessage.str().c_str());
    }

    aOutputFile << std::scientific << std::setprecision(6) << std::right << "Iter" << std::setw(10) << "F-count"
            << std::setw(15) << "Best(F)" << std::setw(15) << "Mean(F)" << std::setw(15) << "StdDev(F)" << std::setw(15)
            << "TR-Radius" << "\n" << std::flush;
}

/******************************************************************************//**
 * @brief Print diagnostics for Kelley-Sachs-Bound-Constrained (KSBC) algorithm
 * @param [in] aData diagnostic data for Kelley-Sachs-Bound-Constrained algorithm
 * @param [in,out] aOutputFile output file
 * @param [in] aPrint flag use to enable/disable output (default = disabled)
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void print_pso_diagnostics(const Plato::OutputDataPSO<ScalarType, OrdinalType>& aData,
                                  std::ofstream& aOutputFile,
                                  bool aPrint = false)
{
    try
    {
        Plato::pso::is_file_open(aOutputFile);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        std::ostringstream tMessage;
        tMessage << "\n\n ******** ERROR IN FILE: " << __FILE__ << ", FUNCTION: " << __PRETTY_FUNCTION__ << ", LINE: "
                << __LINE__ << " ******** \n\n";
        tMessage << tErrorMsg.what();
        if(aPrint == true)
        {
            std::cout << tMessage.str().c_str() << std::flush;
        }
        throw std::invalid_argument(tMessage.str().c_str());
    }

    aOutputFile << std::scientific << std::setprecision(6) << std::right << aData.mNumIter << std::setw(10)
            << aData.mObjFuncCount << std::setw(20) << aData.mBestObjFuncValue << std::setw(15)
            << aData.mMeanObjFuncValue << std::setw(15) << aData.mStdDevObjFuncValue << std::setw(15)
            << aData.mTrustRegionMultiplier << "\n" << std::flush;
}

} // namespace pso

template<typename ScalarType, typename OrdinalType = size_t>
class GradFreeCriteria
{
public:
    virtual ~GradFreeCriteria()
    {
    }

    /******************************************************************************//**
     * @brief Evaluates general criterion
     * @param [in] aControl multi-vector of control variables (i.e. optimization variables)
     * @param [out] aOutput criterion value for each control vector
     **********************************************************************************/
    virtual void value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                       Plato::Vector<ScalarType, OrdinalType> & aOutput) = 0;
};

template<typename ScalarType, typename OrdinalType = size_t>
class GradFreeRosenbrock : public Plato::GradFreeCriteria<ScalarType, OrdinalType>
{
public:
    GradFreeRosenbrock()
    {
    }

    virtual ~GradFreeRosenbrock()
    {
    }

    /******************************************************************************//**
     * @brief Evaluate Rosenbrock criterion
     * @param [in] aControl multi-vector of control variables (i.e. optimization variables)
     * @param [out] aOutput criterion value for each control vector
     **********************************************************************************/
    void value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
               Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {
        const OrdinalType tNumParticles = aControl.getNumVectors();
        assert(tNumParticles > static_cast<OrdinalType>(0));
        assert(aOutput.size() == tNumParticles);
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            aOutput[tIndex] = this->evaluate(aControl[tIndex]);
        }
    }

private:
    /******************************************************************************//**
     * @brief Evaluate function: 100 * \left(x_2 - x_1^2\right)^2 + \left(1 - x_1\right)^2
     * @param [in] aControl vector of control variables (i.e. optimization variables)
     * @return Rosenbrock function evaluation
     **********************************************************************************/
    ScalarType evaluate(const Plato::Vector<ScalarType, OrdinalType> & aControl)
    {
        const ScalarType tOutput = static_cast<ScalarType>(100.)
                * std::pow((aControl[1] - aControl[0] * aControl[0]), static_cast<ScalarType>(2))
                + std::pow(static_cast<ScalarType>(1) - aControl[0], static_cast<ScalarType>(2));
        return (tOutput);
    }
};

template<typename ScalarType, typename OrdinalType = size_t>
class ParticleSwarmDataMng
{
public:
    explicit ParticleSwarmDataMng(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory) :
            mTimeStep(1),
            mLowerBounds(),
            mUpperBounds(),
            mMeanParticlePosition(),
            mStdDevParticlePosition(),
            mGlobalBestParticlePosition(),
            mCurrentParticles(aFactory->control().create()),
            mCurrentVelocities(aFactory->control().create()),
            mPreviousVelocities(aFactory->control().create()),
            mBestParticlePositions(aFactory->control().create()),
            mCommWrapper(aFactory->getCommWrapper().create())
    {
        this->initialize();
    }

    ~ParticleSwarmDataMng()
    {
    }

    OrdinalType getNumParticles() const
    {
        assert(mCurrentParticles.use_count() > static_cast<OrdinalType>(0));
        return (mCurrentParticles->getNumVectors());
    }

    ScalarType getTimeStep() const
    {
        return (mTimeStep);
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentParticles() const
    {
        assert(mCurrentParticles.use_count() > static_cast<OrdinalType>(0));
        return (*mCurrentParticles);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getCurrentParticle(const OrdinalType & aIndex) const
    {
        assert(mCurrentParticles.use_count() > static_cast<OrdinalType>(0));
        return ((*mCurrentParticles)[aIndex]);
    }

    void setCurrentParticle(const OrdinalType & aIndex, const Plato::Vector<ScalarType, OrdinalType> & aParticle)
    {
        assert(mCurrentParticles.use_count() > static_cast<OrdinalType>(0));
        assert(aParticle.size() > static_cast<OrdinalType>(0));
        (*mCurrentParticles)[aIndex].update(static_cast<ScalarType>(1), aParticle, static_cast<ScalarType>(0));
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentVelocities() const
    {
        assert(mCurrentVelocities.use_count() > static_cast<OrdinalType>(0));
        return (*mCurrentVelocities);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getCurrentVelocity(const OrdinalType & aIndex) const
    {
        assert(mCurrentVelocities.use_count() > static_cast<OrdinalType>(0));
        return ((*mCurrentVelocities)[aIndex]);
    }

    void setCurrentVelocity(const OrdinalType & aIndex, const Plato::Vector<ScalarType, OrdinalType> & aParticleVel)
    {
        assert(mCurrentVelocities.use_count() > static_cast<OrdinalType>(0));
        assert(aParticleVel.size() > static_cast<OrdinalType>(0));
        (*mCurrentVelocities)[aIndex].update(static_cast<ScalarType>(1), aParticleVel, static_cast<ScalarType>(0));
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getPreviousVelocities() const
    {
        assert(mPreviousVelocities.use_count() > static_cast<OrdinalType>(0));
        return (*mPreviousVelocities);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getPreviousVelocity(const OrdinalType & aIndex) const
    {
        assert(mPreviousVelocities.use_count() > static_cast<OrdinalType>(0));
        return ((*mPreviousVelocities)[aIndex]);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getLowerBounds() const
    {
        assert(mLowerBounds.use_count() > static_cast<OrdinalType>(0));
        return (*mLowerBounds);
    }

    void setLowerBounds(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mLowerBounds.use_count() > static_cast<OrdinalType>(0));
        assert(aInput.size() > static_cast<OrdinalType>(0));
        mLowerBounds->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    void setLowerBounds(const ScalarType & aInput)
    {
        assert(mLowerBounds.use_count() > static_cast<OrdinalType>(0));
        mLowerBounds->fill(aInput);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getUpperBounds() const
    {
        assert(mUpperBounds.use_count() > static_cast<OrdinalType>(0));
        return (*mUpperBounds);
    }

    void setUpperBounds(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mUpperBounds.use_count() > static_cast<OrdinalType>(0));
        assert(aInput.size() > static_cast<OrdinalType>(0));
        mUpperBounds->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    void setUpperBounds(const ScalarType & aInput)
    {
        assert(mUpperBounds.use_count() > static_cast<OrdinalType>(0));
        mUpperBounds->fill(aInput);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getParticlePositionMean() const
    {
        assert(mMeanParticlePosition.use_count() > static_cast<OrdinalType>(0));
        return (*mMeanParticlePosition);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getParticlePositionStdDev() const
    {
        assert(mStdDevParticlePosition.use_count() > static_cast<OrdinalType>(0));
        return (*mStdDevParticlePosition);
    }

    const Plato::Vector<ScalarType, OrdinalType> & getGlobalBestParticlePosition() const
    {
        assert(mGlobalBestParticlePosition.use_count() > static_cast<OrdinalType>(0));
        return (*mGlobalBestParticlePosition);
    }

    void setGlobalBestParticlePosition(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        assert(mGlobalBestParticlePosition.use_count() > static_cast<OrdinalType>(0));
        mGlobalBestParticlePosition->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    const Plato::Vector<ScalarType, OrdinalType> & getBestParticlePosition(const OrdinalType & aIndex) const
    {
        assert(mBestParticlePositions.use_count() > static_cast<OrdinalType>(0));
        return ((*mBestParticlePositions)[aIndex]);
    }

    void setBestParticlePosition(const OrdinalType & aIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput) const
    {
        assert(mBestParticlePositions.use_count() > static_cast<OrdinalType>(0));
        (*mBestParticlePositions)[aIndex].update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

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

    void cachePreviousVelocities()
    {
        Plato::update(static_cast<ScalarType>(1), *mCurrentVelocities, static_cast<ScalarType>(0), *mPreviousVelocities);
    }

    /******************************************************************************//**
     * @brief Return a const reference to the distributed memory communication wrapper
     * @return const reference to the distributed memory communication wrapper
    **********************************************************************************/
    const Plato::CommWrapper& getCommWrapper() const
    {
        return (mCommWrapper.operator*());
    }

    void computeBestParticlePositionStatistics()
    {
        this->computeBestParticlePositionMean();
        this->computeBestParticlePositionStdDev();
    }

private:
    void initialize()
    {
        const OrdinalType tPARTICLE_INDEX = 0;
        const Plato::Vector<ScalarType, OrdinalType> & tMyParticle = (*mCurrentParticles)[tPARTICLE_INDEX];
        mWorkVector = tMyParticle.create();
        mLowerBounds = tMyParticle.create();
        mUpperBounds = tMyParticle.create();
        mMeanParticlePosition = tMyParticle.create();
        mStdDevParticlePosition = tMyParticle.create();

        mGlobalBestParticlePosition = tMyParticle.create();
        mGlobalBestParticlePosition->fill(std::numeric_limits<ScalarType>::max());
        Plato::fill(std::numeric_limits<ScalarType>::max(), *mBestParticlePositions);
    }

    void computeBestParticlePositionMean()
    {
        mMeanParticlePosition->fill(static_cast<ScalarType>(0));
        const OrdinalType tNumParticles = this->getNumParticles();
        for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyParticle = (*mBestParticlePositions)[tParticleIndex];
            const OrdinalType tMyParticleDims = tMyParticle.size();
            for(OrdinalType tDim = 0; tDim < tMyParticleDims; tDim++)
            {
                (*mMeanParticlePosition)[tDim] += tMyParticle[tDim] / static_cast<ScalarType>(tNumParticles);
            }
        }
    }

    void computeBestParticlePositionStdDev()
    {
        mStdDevParticlePosition->fill(static_cast<ScalarType>(0));
        const OrdinalType tNumParticles = this->getNumParticles();
        const OrdinalType tNumControls = mStdDevParticlePosition->size();
        for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyBestParticlePosition = (*mBestParticlePositions)[tParticleIndex];
            for(OrdinalType tDim = 0; tDim < tNumControls; tDim++)
            {
                const ScalarType tMisfit = tMyBestParticlePosition[tDim] - (*mMeanParticlePosition)[tDim];
                (*mStdDevParticlePosition)[tDim] += tMisfit * tMisfit;
            }
        }

        for(OrdinalType tDim = 0; tDim < tNumControls; tDim++)
        {
            const ScalarType tValue = (*mStdDevParticlePosition)[tDim] / static_cast<ScalarType>(tNumParticles - 1);
            (*mStdDevParticlePosition)[tDim] = std::pow(tValue, static_cast<ScalarType>(0.5));
        }
    }

private:
    ScalarType mTimeStep;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mWorkVector;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mLowerBounds;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mUpperBounds;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mMeanParticlePosition;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mStdDevParticlePosition;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mGlobalBestParticlePosition;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentParticles;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentVelocities;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousVelocities;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mBestParticlePositions;

    std::shared_ptr<Plato::CommWrapper> mCommWrapper;
};

template<typename ScalarType, typename OrdinalType = size_t>
class ParticleSwarm
{
public:
    ParticleSwarm(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                  const std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> & aObjective) :
            mPrintDiagnostics(false),
            mNumIterations(0),
            mMaxNumIterations(1000),
            mNumConsecutiveFailures(0),
            mNumConsecutiveSuccesses(0),
            mMaxNumConsecutiveFailures(10),
            mMaxNumConsecutiveSuccesses(10),
            mGlobalBestParticleIndex(0),
            mGlobalBestParticleIteration(0),
            mInertiaMultiplier(0.9),
            mSocialBehaviorMultiplier(0.8),
            mCognitiveBehaviorMultiplier(0.8),
            mTrustRegionMultiplier(1),
            mTrustRegionExpansionMultiplier(4.0),
            mTrustRegionContractionMultiplier(0.75),
            mBestObjFuncTolerance(1e-10),
            mMeanObjFuncTolerance(5e-4),
            mBestObjFunValueMean(std::numeric_limits<ScalarType>::max()),
            mBestObjFunValueStdDev(std::numeric_limits<ScalarType>::max()),
            mCurrentGlobalBestObjFunValue(std::numeric_limits<ScalarType>::max()),
            mPreviousGlobalBestObjFunValue(std::numeric_limits<ScalarType>::max()),
            mStopCriterion(Plato::particle_swarm::DID_NOT_CONVERGE),
            mWorkVectorOne(),
            mWorkVectorTwo(),
            mBestObjFuncValues(),
            mCurrentObjFuncValues(),
            mObjective(aObjective),
            mDataMng(std::make_shared<Plato::ParticleSwarmDataMng<ScalarType, OrdinalType>>(aFactory)),
            mReductions(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>())
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~ParticleSwarm()
    {
    }

    /******************************************************************************//**
     * @brief Enable output of diagnostics (i.e. optimization problem status)
    **********************************************************************************/
    void enableDiagnostics()
    {
        mPrintDiagnostics = true;
    }

    void setInertiaMultiplier(const ScalarType & aInput)
    {
        mInertiaMultiplier = aInput;
    }

    void setCognitiveMultiplier(const ScalarType & aInput)
    {
        mCognitiveBehaviorMultiplier = aInput;
    }

    void setSocialBehaviorMultiplier(const ScalarType & aInput)
    {
        mSocialBehaviorMultiplier = aInput;
    }

    OrdinalType getNumIterations() const
    {
        return (mNumIterations);
    }

    void setMaxNumIterations(const OrdinalType & aInput)
    {
        mMaxNumIterations = aInput;
    }

    ScalarType getBestObjFuncValue() const
    {
        return (mCurrentGlobalBestObjFunValue);
    }

    ScalarType getBestObjFuncValueMean() const
    {
        return (mBestObjFunValueMean);
    }

    ScalarType getBestObjFuncValueStdDev() const
    {
        return (mBestObjFunValueStdDev);
    }

    void setUpperBounds(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        mDataMng->setUpperBounds(aInput);
    }

    void setLowerBounds(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        mDataMng->setLowerBounds(aInput);
    }

    void setUpperBounds(const ScalarType & aInput)
    {
        mDataMng->setUpperBounds(aInput);
    }

    void setLowerBounds(const ScalarType & aInput)
    {
        mDataMng->setLowerBounds(aInput);
    }

    const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & getDataMng() const
    {
        return (*mDataMng);
    }

    /******************************************************************************//**
     * @brief Solve particle swarm optimization problem
    **********************************************************************************/
    void solve()
    {
        assert(mDataMng.use_count() > static_cast<OrdinalType>(0));
        assert(mObjective.use_count() > static_cast<OrdinalType>(0));

        this->openOutputFile();
        this->initialize();
        this->checkInertiaMultiplier();
        mDataMng->setInitialParticles();

        mNumIterations = 0;
        while(1)
        {
            mNumIterations++;
            mObjective->value(mDataMng->getCurrentParticles(), *mCurrentObjFuncValues);
            this->findBestParticlePositions();
            this->updateParticleVelocities();
            this->updateParticlePositions();
            this->updateTrustRegionMultiplier();
            this->computeBestObjFunStatistics();
            this->outputDiagnostics();

            if(this->checkStoppingCriteria())
            {
                mDataMng->computeBestParticlePositionStatistics();
                this->outputStoppingCriterion();
                this->closeOutputFile();
                break;
            }
        }
    }

private:
    void initialize()
    {
        const OrdinalType tPARTICLE_INDEX = 0;
        mWorkVectorOne = mDataMng->getCurrentParticle(tPARTICLE_INDEX).create();
        mWorkVectorTwo = mDataMng->getCurrentParticle(tPARTICLE_INDEX).create();

        const OrdinalType tNumParticles = mDataMng->getNumParticles();
        mBestObjFuncValues = std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(tNumParticles);
        mBestObjFuncValues->fill(std::numeric_limits<ScalarType>::max());
        mCurrentObjFuncValues = std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(tNumParticles);
        mCurrentObjFuncValues->fill(std::numeric_limits<ScalarType>::max());
    }

    void updateParticleVelocity(const OrdinalType & aParticleIndex,
                                std::default_random_engine & aGenerator,
                                std::uniform_real_distribution<ScalarType> & aDistribution)
    {
        const Plato::Vector<ScalarType, OrdinalType> & tPreviousVel = mDataMng->getPreviousVelocity(aParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticle = mDataMng->getCurrentParticle(aParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalBestParticlePosition = mDataMng->getGlobalBestParticlePosition();
        const Plato::Vector<ScalarType, OrdinalType> & tBestParticlePosition = mDataMng->getBestParticlePosition(aParticleIndex);

        const OrdinalType tNumControls = tPreviousVel.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            // inertia contribution
            const ScalarType tInertiaValue = mInertiaMultiplier * tPreviousVel[tIndex];
            // cognitive behavior contribution
            const ScalarType tRandomNumOne = aDistribution(aGenerator);
            const ScalarType tCognitiveMultiplier = tRandomNumOne * mCognitiveBehaviorMultiplier;
            const ScalarType tCognitiveValue = tCognitiveMultiplier * (tBestParticlePosition[tIndex] - tCurrentParticle[tIndex]);
            // social behavior contribution
            const ScalarType tRandomNumTwo = aDistribution(aGenerator);
            const ScalarType tSocialMultiplier = tRandomNumTwo * mSocialBehaviorMultiplier;
            const ScalarType tSocialValue = tSocialMultiplier * (tGlobalBestParticlePosition[tIndex] - tCurrentParticle[tIndex]);
            // set new velocity
            (*mWorkVectorOne)[tIndex] = tInertiaValue + tCognitiveValue + tSocialValue;
        }

        mDataMng->setCurrentVelocity(aParticleIndex, *mWorkVectorOne);
    }

    void updateGlobalBestParticleVelocity(std::default_random_engine & aGenerator,
                                          std::uniform_real_distribution<ScalarType> & aDistribution)
    {
        const Plato::Vector<ScalarType, OrdinalType> & tPreviousVel = mDataMng->getPreviousVelocity(mGlobalBestParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalBestParticlePosition = mDataMng->getGlobalBestParticlePosition();
        const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticlePosition = mDataMng->getCurrentParticle(mGlobalBestParticleIndex);

        const OrdinalType tNumControls = tPreviousVel.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            const ScalarType tRandomNum = aDistribution(aGenerator);
            const ScalarType tStochasticTrustRegionMultiplier = mTrustRegionMultiplier
                    * (static_cast<ScalarType>(1) - static_cast<ScalarType>(2) * tRandomNum);
            (*mWorkVectorOne)[tIndex] = (static_cast<ScalarType>(-1) * tCurrentParticlePosition[tIndex]) + tGlobalBestParticlePosition[tIndex]
                    + (mInertiaMultiplier * tPreviousVel[tIndex]) + tStochasticTrustRegionMultiplier;
        }
        mDataMng->setCurrentVelocity(mGlobalBestParticleIndex, *mWorkVectorOne);
    }

    void updateParticleVelocities()
    {
        mDataMng->cachePreviousVelocities();
        std::default_random_engine tGenerator;
        std::uniform_real_distribution<ScalarType> tDistribution(0.0 /* lower bound */, 1.0 /* upper bound */);
        const OrdinalType tNumParticles = mDataMng->getNumParticles();
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            if(tIndex != mGlobalBestParticleIndex)
            {
                this->updateParticleVelocity(tIndex, tGenerator, tDistribution);
            }
            else
            {
                assert(tIndex == mGlobalBestParticleIndex);
                this->updateGlobalBestParticleVelocity(tGenerator, tDistribution);
            }
        }
    }

    void updateParticlePosition(const OrdinalType & aParticleIndex)
    {
        const ScalarType tTimeStep = mDataMng->getTimeStep();
        const Plato::Vector<ScalarType, OrdinalType> & tLowerBounds = mDataMng->getLowerBounds();
        const Plato::Vector<ScalarType, OrdinalType> & tUpperBounds = mDataMng->getUpperBounds();
        const Plato::Vector<ScalarType, OrdinalType> & tParticleVel = mDataMng->getCurrentVelocity(aParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tParticlePosition = mDataMng->getCurrentParticle(aParticleIndex);
        mWorkVectorOne->update(static_cast<ScalarType>(1), tParticlePosition, static_cast<ScalarType>(0));

        const OrdinalType tNumControls = tParticleVel.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            (*mWorkVectorOne)[tIndex] = (*mWorkVectorOne)[tIndex] + (tTimeStep * tParticleVel[tIndex]);
            (*mWorkVectorOne)[tIndex] = std::max((*mWorkVectorOne)[tIndex], tLowerBounds[tIndex]);
            (*mWorkVectorOne)[tIndex] = std::min((*mWorkVectorOne)[tIndex], tUpperBounds[tIndex]);
        }

        mDataMng->setCurrentParticle(aParticleIndex, *mWorkVectorOne);
    }


    void updateGlobalBestParticlePosition()
    {
        const Plato::Vector<ScalarType, OrdinalType> & tLowerBounds = mDataMng->getLowerBounds();
        const Plato::Vector<ScalarType, OrdinalType> & tUpperBounds = mDataMng->getUpperBounds();
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalBestParticleVel = mDataMng->getCurrentVelocity(mGlobalBestParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalBestParticlePosition = mDataMng->getGlobalBestParticlePosition();

        std::default_random_engine tGenerator;
        std::uniform_real_distribution<ScalarType> tDistribution(0.0 /* lower bound */, 1.0 /* upper bound */);
        const ScalarType tRandomNum = tDistribution(tGenerator);
        const ScalarType tStochasticTrustRegionMultiplier = mTrustRegionMultiplier * (static_cast<ScalarType>(1) - static_cast<ScalarType>(2) * tRandomNum);

        const OrdinalType tNumControls = tGlobalBestParticlePosition.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            (*mWorkVectorOne)[tIndex] = tGlobalBestParticlePosition[tIndex]
                    + (mInertiaMultiplier * tGlobalBestParticleVel[tIndex]) + tStochasticTrustRegionMultiplier;
        }

        mDataMng->setCurrentParticle(mGlobalBestParticleIndex, *mWorkVectorOne);
    }

    void updateParticlePositions()
    {
        const OrdinalType tNumParticles = mDataMng->getNumParticles();
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            if(tIndex != mGlobalBestParticleIndex)
            {
                this->updateParticlePosition(tIndex);
            }
            else
            {
                assert(tIndex == mGlobalBestParticleIndex);
                this->updateGlobalBestParticlePosition();
            }
        }
    }

    void checkInertiaMultiplier()
    {
        const ScalarType tUpperBound = 1.0;
        const ScalarType tLowerBound = (static_cast<ScalarType>(0.5)
                * (mCognitiveBehaviorMultiplier + mSocialBehaviorMultiplier)) - static_cast<ScalarType>(1.0);
        const bool tMultiplierOutsideBounds = (mInertiaMultiplier < tLowerBound) || (mInertiaMultiplier > tUpperBound);
        if(tMultiplierOutsideBounds == true)
        {
            // use default values
            mInertiaMultiplier = 0.9;
            mSocialBehaviorMultiplier = 0.8;
            mCognitiveBehaviorMultiplier = 0.8;
        }
    }

    void checkGlobalBestParticleUpdateSuccessRate()
    {
        if(mCurrentGlobalBestObjFunValue < mPreviousGlobalBestObjFunValue)
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

    void findBestParticlePositions()
    {
        mPreviousGlobalBestObjFunValue = mCurrentGlobalBestObjFunValue;
        const OrdinalType tNumParticles = mDataMng->getNumParticles();
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            this->checkBestParticlePosition(tIndex);
            this->checkGlobalBestParticlePosition(tIndex);
        }
        this->checkGlobalBestParticleUpdateSuccessRate();
    }

    bool checkStoppingCriteria()
    {
        bool tStop = false;
        if(mNumIterations >= mMaxNumIterations)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::MAX_NUMBER_ITERATIONS;
        }
        else if(mCurrentGlobalBestObjFunValue < mBestObjFuncTolerance)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::TRUE_OBJECTIVE_TOLERANCE;
        }
        else if(mBestObjFunValueMean < mMeanObjFuncTolerance)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::MEAN_OBJECTIVE_TOLERANCE;
        }

        return (tStop);
    }

    void updateTrustRegionMultiplier()
    {
        if(mNumConsecutiveSuccesses >= mMaxNumConsecutiveSuccesses)
        {
            mTrustRegionMultiplier *= mTrustRegionExpansionMultiplier;
        }
        else if(mNumConsecutiveFailures >= mMaxNumConsecutiveFailures)
        {
            mTrustRegionMultiplier *= mTrustRegionContractionMultiplier;
        }
    }

    void checkBestParticlePosition(const OrdinalType & aParticleIndex)
    {
        if((*mCurrentObjFuncValues)[aParticleIndex] < (*mBestObjFuncValues)[aParticleIndex])
        {
            const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticle = mDataMng->getCurrentParticle(aParticleIndex);
            mDataMng->setBestParticlePosition(aParticleIndex, tCurrentParticle);
            (*mBestObjFuncValues)[aParticleIndex] = (*mCurrentObjFuncValues)[aParticleIndex];
        }
    }

    void checkGlobalBestParticlePosition(const OrdinalType & aParticleIndex)
    {
        const ScalarType tCurrentParticleObjFuncValue = (*mCurrentObjFuncValues)[aParticleIndex];
        if(tCurrentParticleObjFuncValue < mCurrentGlobalBestObjFunValue)
        {
            mCurrentGlobalBestObjFunValue = tCurrentParticleObjFuncValue;
            const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticle = mDataMng->getCurrentParticle(aParticleIndex);
            mDataMng->setGlobalBestParticlePosition(tCurrentParticle);
            mGlobalBestParticleIndex = aParticleIndex;
            mGlobalBestParticleIteration = mNumIterations;
        }
    }

    void computeBestObjFunStatistics()
    {
        this->computeBestObjFunMean();
        this->computeBestObjFunStdDev();
    }

    void computeBestObjFunMean()
    {
        mBestObjFunValueMean = mReductions->sum(*mBestObjFuncValues);
        const OrdinalType tNumParticles = mDataMng->getNumParticles();
        mBestObjFunValueMean = mBestObjFunValueMean / tNumParticles;
    }

    void computeBestObjFunStdDev()
    {
        mBestObjFunValueStdDev = 0;
        const OrdinalType tNumParticles = mDataMng->getNumParticles();
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            const ScalarType tMisfit = (*mBestObjFuncValues)[tIndex] - mBestObjFunValueMean;
            mBestObjFunValueStdDev += tMisfit * tMisfit;
        }
        mBestObjFunValueStdDev = mBestObjFunValueStdDev / (tNumParticles - static_cast<OrdinalType>(1));
        mBestObjFunValueStdDev = std::pow(mBestObjFunValueStdDev, static_cast<ScalarType>(0.5));
    }

    /******************************************************************************//**
     * @brief Open output file (i.e. diagnostics file)
    **********************************************************************************/
    void openOutputFile()
    {
        if(mPrintDiagnostics == true)
        {
            const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
            if(tMyCommWrapper.myProcID() == 0)
            {
                mOutputStream.open("plato_pso_algorithm_diagnostics.txt");
                Plato::pso::print_pso_diagnostics_header(mOutputData, mOutputStream, mPrintDiagnostics);
            }
        }
    }

    /******************************************************************************//**
     * @brief Close output file (i.e. diagnostics file)
    **********************************************************************************/
    void closeOutputFile()
    {
        if(mPrintDiagnostics == true)
        {
            const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
            if(tMyCommWrapper.myProcID() == 0)
            {
                mOutputStream.close();
            }
        }
    }

    /******************************************************************************//**
     * @brief Print stopping criterion into diagnostics file.
    **********************************************************************************/
    void outputStoppingCriterion()
    {
        if(mPrintDiagnostics == true)
        {
            const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
            if(tMyCommWrapper.myProcID() == 0)
            {
                std::string tReason;
                Plato::pso::get_stop_criterion(mStopCriterion, tReason);
                mOutputStream << tReason.c_str();
            }
        }
    }

    /******************************************************************************//**
     * @brief Print diagnostics for Kelley-Sachs bound constrained optimization algorithm
    **********************************************************************************/
    void outputDiagnostics()
    {
        if(mPrintDiagnostics == false)
        {
            return;
        }

        OrdinalType mNumIter;  /*!< number of outer iterations */
        OrdinalType mObjFuncCount;  /*!< number of objective function evaluations */

        ScalarType mTrustRegionRadius;  /*!< trust region radius */
        ScalarType mBestObjFuncValue;  /*!< best - objective function value */
        ScalarType mMeanObjFuncValue;  /*!< mean - objective function value */
        ScalarType mStdDevObjFuncValue;  /*!< standard deviation - objective function value */

        const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
        if(tMyCommWrapper.myProcID() == 0)
        {
            mOutputData.mNumIter = mNumIterations;
            mOutputData.mObjFuncCount = mNumIterations * mDataMng->getNumParticles();
            mOutputData.mBestObjFuncValue = mCurrentGlobalBestObjFunValue;
            mOutputData.mMeanObjFuncValue = mBestObjFunValueMean;
            mOutputData.mStdDevObjFuncValue = mBestObjFunValueStdDev;
            mOutputData.mTrustRegionMultiplier = mTrustRegionMultiplier;

            Plato::pso::print_pso_diagnostics(mOutputData, mOutputStream, mPrintDiagnostics);
        }
    }

private:
    bool mPrintDiagnostics;
    std::ofstream mOutputStream;

    OrdinalType mNumIterations;
    OrdinalType mMaxNumIterations;

    OrdinalType mNumConsecutiveFailures;
    OrdinalType mNumConsecutiveSuccesses;
    OrdinalType mMaxNumConsecutiveFailures;
    OrdinalType mMaxNumConsecutiveSuccesses;

    OrdinalType mGlobalBestParticleIndex;
    OrdinalType mGlobalBestParticleIteration;

    ScalarType mInertiaMultiplier;
    ScalarType mSocialBehaviorMultiplier;
    ScalarType mCognitiveBehaviorMultiplier;

    ScalarType mTrustRegionMultiplier;
    ScalarType mTrustRegionExpansionMultiplier;
    ScalarType mTrustRegionContractionMultiplier;

    ScalarType mBestObjFuncTolerance;
    ScalarType mMeanObjFuncTolerance;
    ScalarType mBestObjFunValueMean;
    ScalarType mBestObjFunValueStdDev;
    ScalarType mCurrentGlobalBestObjFunValue;
    ScalarType mPreviousGlobalBestObjFunValue;

    Plato::particle_swarm::stop_t mStopCriterion;
    Plato::OutputDataPSO<ScalarType, OrdinalType> mOutputData;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mWorkVectorOne;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mWorkVectorTwo;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mBestObjFuncValues;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentObjFuncValues;

    std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> mObjective;
    std::shared_ptr<Plato::ParticleSwarmDataMng<ScalarType, OrdinalType>> mDataMng;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mReductions;
};
// class ParticleSwarm

}// namespace Plato

namespace ParticleSwarmTest
{

TEST(PlatoTest, PSO_DataMng)
{
    // ********* Allocate Core Optimization Data Templates *********
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    tFactory->allocateControl(tNumControls, tNumParticles);

    // TEST setInitialParticles
    Plato::ParticleSwarmDataMng<double> tDataMng(tFactory);
    EXPECT_EQ(20u, tDataMng.getNumParticles());
    Plato::StandardVector<double> tVector(tNumControls);
    tVector.fill(5);
    tDataMng.setUpperBounds(tVector);
    tVector.fill(-5);
    tDataMng.setLowerBounds(tVector);
    tDataMng.setInitialParticles();
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentParticles(), tDataMng.getCurrentVelocities());

    // TEST cachePreviousVelocities
    tDataMng.cachePreviousVelocities();
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentVelocities(), tDataMng.getPreviousVelocities());
}

TEST(PlatoTest, GradFreeRosenbrock)
{
    // ********* Allocate Core Optimization Data Templates *********
    const size_t tNumControls = 2;
    const size_t tNumParticles = 3;
    Plato::StandardMultiVector<double> tControls(tNumParticles, tNumControls);
    tControls[0].fill(2);
    tControls[1].fill(0.95);
    tControls[2].fill(1);

    // TEST OBJECTIVE
    Plato::StandardVector<double> tObjVals(tNumParticles);
    Plato::GradFreeRosenbrock<double> tObjective;
    tObjective.value(tControls, tObjVals);

    const double tTolerance = 1e-6;
    EXPECT_NEAR(401, tObjVals[0], tTolerance);
    EXPECT_NEAR(0.228125, tObjVals[1], tTolerance);
    EXPECT_NEAR(0, tObjVals[2], tTolerance);
}

TEST(PlatoTest, PSO_IsFileOpenExeption)
{
    std::ofstream tFile;
    ASSERT_THROW(Plato::pso::is_file_open(tFile), std::invalid_argument);

    tFile.open("MyFile.txt");
    ASSERT_NO_THROW(Plato::pso::is_file_open(tFile));
    tFile.close();
    std::system("rm -f MyFile.txt");
}

TEST(PlatoTest, PSO_PrintStoppingCriterion)
{
    std::string tDescription;
    Plato::particle_swarm::stop_t tFlag = Plato::particle_swarm::DID_NOT_CONVERGE;
    Plato::pso::get_stop_criterion(tFlag, tDescription);
    std::string tGold("\n\n****** Optimization algorithm did not converge. ******\n\n");
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::particle_swarm::MAX_NUMBER_ITERATIONS;
    Plato::pso::get_stop_criterion(tFlag, tDescription);
    tGold = "\n\n****** Optimization stopping due to exceeding maximum number of iterations. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::particle_swarm::TRUE_OBJECTIVE_TOLERANCE;
    Plato::pso::get_stop_criterion(tFlag, tDescription);
    tGold = "\n\n****** Optimization stopping due to global best objective function tolerance being met. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());

    tFlag = Plato::particle_swarm::MEAN_OBJECTIVE_TOLERANCE;
    Plato::pso::get_stop_criterion(tFlag, tDescription);
    tGold = "\n\n****** Optimization stopping due to mean objective function tolerance being met. ******\n\n";
    ASSERT_STREQ(tDescription.c_str(), tGold.c_str());
}

TEST(PlatoTest, PSO_PrintDiagnostics)
{
    std::ofstream tWriteFile;
    tWriteFile.open("MyFile1.txt");
    Plato::OutputDataPSO<double> tData;
    tData.mNumIter = 1;
    tData.mObjFuncCount = 20;
    tData.mBestObjFuncValue = 4.2321;
    tData.mMeanObjFuncValue = 8.2321;
    tData.mStdDevObjFuncValue = 2.2321;
    tData.mTrustRegionMultiplier = 1.0;
    ASSERT_NO_THROW(Plato::pso::print_pso_diagnostics_header(tData, tWriteFile));
    ASSERT_NO_THROW(Plato::pso::print_pso_diagnostics(tData, tWriteFile));

    tData.mNumIter = 2;
    tData.mObjFuncCount = 40;
    tData.mBestObjFuncValue = 2.2321;
    tData.mMeanObjFuncValue = 7.2321;
    tData.mStdDevObjFuncValue = 2.4321;
    tData.mTrustRegionMultiplier = 1.0;
    ASSERT_NO_THROW(Plato::pso::print_pso_diagnostics(tData, tWriteFile));
    tWriteFile.close();

    std::ifstream tReadFile;
    tReadFile.open("MyFile1.txt");
    std::string tInputString;
    std::stringstream tReadData;
    while(tReadFile >> tInputString)
    {
        tReadData << tInputString.c_str();
    }
    tReadFile.close();
    std::system("rm -f MyFile1.txt");

    std::stringstream tGold;
    tGold << "IterF-countBest(F)Mean(F)StdDev(F)TR-Radius";
    tGold << "1204.232100e+008.232100e+002.232100e+001.000000e+00";
    tGold << "2402.232100e+007.232100e+002.432100e+001.000000e+00";
    ASSERT_STREQ(tReadData.str().c_str(), tGold.str().c_str());
}

TEST(PlatoTest, PSO_Solve)
{
    // ********* Allocate Core Optimization Data Templates *********
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    tFactory->allocateControl(tNumControls, tNumParticles);

    // TEST ALGORITHM
    std::shared_ptr<Plato::GradFreeRosenbrock<double>> tObjective = std::make_shared<Plato::GradFreeRosenbrock<double>>();
    Plato::ParticleSwarm<double> tAlgorithm(tFactory, tObjective);
    tAlgorithm.setLowerBounds(-6);
    tAlgorithm.setUpperBounds(6);
    tAlgorithm.solve();

    const double tTolerance = 1e-4;
    EXPECT_NEAR(0, tAlgorithm.getBestObjFuncValue(), tTolerance);

    std::cout << "NUM ITERATIONS = " << tAlgorithm.getNumIterations() << "\n";
    std::cout << "OBJECTIVE: BEST = " << tAlgorithm.getBestObjFuncValue() << ", MEAN = "
            << tAlgorithm.getBestObjFuncValueMean() << ", STDDEV = " << tAlgorithm.getBestObjFuncValueStdDev() << "\n";
    for(size_t tIndex = 0; tIndex < tNumControls; tIndex++)
    {
        std::cout << "CONTROL[" << tIndex << "]: BEST = "
                << tAlgorithm.getDataMng().getGlobalBestParticlePosition()[tIndex] << ", MEAN = "
                << tAlgorithm.getDataMng().getParticlePositionMean()[tIndex] << ", STDDEV = "
                << tAlgorithm.getDataMng().getParticlePositionStdDev()[tIndex] << "\n";
    }
}

} // ParticleSwarmTest
