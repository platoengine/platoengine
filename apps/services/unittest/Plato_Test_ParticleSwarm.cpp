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

/******************************************************************************//**
 * @brief Generic interface that gives access to the list of criteria
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class GradFreeCriteriaList
{
public:
    /******************************************************************************//**
     * @brief Constructor
    **********************************************************************************/
    GradFreeCriteriaList() :
            mWeights(),
            mList()
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~GradFreeCriteriaList()
    {
    }

    /******************************************************************************//**
     * @brief Return size of list
     * @return size
    **********************************************************************************/
    OrdinalType size() const
    {
        return (mList.size());
    }

    /******************************************************************************//**
     * @brief Initializes criterion list.
     * @param [in] aCriteria list of Plato criterion
    **********************************************************************************/
    void add(const std::vector<std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>>> & aCriteria)
    {
        assert(aCriteria.empty() == false);

        mList.clear();
        const OrdinalType tNumCriteria = aCriteria.size();
        for(OrdinalType tIndex = 0; tIndex < tNumCriteria; tIndex++)
        {
            assert(aCriteria[tIndex] != nullptr);
            mList.push_back(aCriteria[tIndex]);
            mWeights.push_back(static_cast<ScalarType>(1));
        }
    }

    /******************************************************************************//**
     * @brief Adds a new element at the end of the vector, after its current last element.
     * @param [in] aCriterion Plato criterion
     * @param [in] aMyWeight weight for input Plato criterion
    **********************************************************************************/
    void add(const std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> & aCriterion, ScalarType aMyWeight = 1)
    {
        assert(aCriterion != nullptr);
        mList.push_back(aCriterion);
        mWeights.push_back(aMyWeight);
    }

    /******************************************************************************//**
     * @brief Initializes criterion list and corresponding weights.
     * @param [in] aCriteria list of Plato criteria
     * @param [in] aWeights list of weights
    **********************************************************************************/
    void add(const std::vector<std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>>> & aCriteria, const std::vector<ScalarType> & aWeights)
    {
        assert(aCriteria.empty() == false);

        mList.clear();
        const OrdinalType tNumCriteria = aCriteria.size();
        for(OrdinalType tIndex = 0; tIndex < tNumCriteria; tIndex++)
        {
            assert(aCriteria[tIndex] != nullptr);
            mList.push_back(aCriteria[tIndex]);
        }
        assert(aWeights.empty() == false);
        assert(aWeights.size() == aCriteria.size());
        mWeights = aWeights;
    }

    /******************************************************************************//**
     * @brief Returns weight at position aIndex in the list.
     * @param [in] aIndex  Position of an element in the list.
     * @return weight
    **********************************************************************************/
    ScalarType weight(const OrdinalType & aIndex) const
    {
        assert(mWeights.empty() == false);
        assert(aIndex < mWeights.size());
        return (mWeights[aIndex]);
    }

    /******************************************************************************//**
     * @brief Returns a reference to the element at position aIndex in the list.
     * @param [in] aIndex Position of an element in the list.
     * @return The criterion at the specified position in the list.
    **********************************************************************************/
    Plato::GradFreeCriteria<ScalarType, OrdinalType> & operator [](const OrdinalType & aIndex)
    {
        assert(aIndex < mList.size());
        assert(mList[aIndex].get() != nullptr);
        return (mList[aIndex].operator*());
    }

    /******************************************************************************//**
     * @brief Returns a const reference to the element at position aIndex in the list.
     * @param [in] aIndex Position of an element in the list.
     * @return The criterion at the specified position in the list.
    **********************************************************************************/
    const Plato::GradFreeCriteria<ScalarType, OrdinalType> & operator [](const OrdinalType & aIndex) const
    {
        assert(aIndex < mList.size());
        assert(mList[aIndex].get() != nullptr);
        return (mList[aIndex].operator*());
    }

    /******************************************************************************//**
     * @brief Returns a copy of the list of criteria.
     * @return Copy of the list of criteria.
    **********************************************************************************/
    std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> create() const
    {
        assert(this->size() > static_cast<OrdinalType>(0));
        std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> tOutput =
                std::make_shared<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>>();
        const OrdinalType tNumCriterion = this->size();
        for(OrdinalType tIndex = 0; tIndex < tNumCriterion; tIndex++)
        {
            assert(mList[tIndex].get() != nullptr);
            const std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> & tCriterion = mList[tIndex];
            ScalarType tMyWeight = mWeights[tIndex];
            tOutput->add(tCriterion, tMyWeight);
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Returns a const shared pointer reference to the element at position aIndex in the list.
     * @param [in] aIndex Position of an element in the list.
     * @return The criterion at the specified position in the list.
    **********************************************************************************/
    const std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> & ptr(const OrdinalType & aIndex) const
    {
        assert(aIndex < mList.size());
        assert(mList[aIndex].get() != nullptr);
        return(mList[aIndex]);
    }

private:
    std::vector<ScalarType> mWeights; /*!< list of weights */
    std::vector<std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>>> mList; /*!< list of grad-free criteria */

private:
    GradFreeCriteriaList(const Plato::GradFreeCriteriaList<ScalarType, OrdinalType>&);
    Plato::GradFreeCriteriaList<ScalarType, OrdinalType> & operator=(const Plato::GradFreeCriteriaList<ScalarType, OrdinalType>&);
};
// class GradFreeCriteriaList

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

private:
    ParticleSwarmDataMng(const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType>&);
    Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & operator=(const Plato::ParticleSwarmDataMng<ScalarType, OrdinalType>&);
};

template<typename ScalarType, typename OrdinalType = size_t>
class ParticleSwarmStageMng
{
public:
    virtual ~ParticleSwarmStageMng()
    {
    }

    virtual void evaluateObjective(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                   Plato::Vector<ScalarType, OrdinalType> & aOutput) = 0;
};
// class ParticleSwarmStageMng

template<typename ScalarType, typename OrdinalType = size_t>
class BoundConstrainedStageMngPSO : public Plato::ParticleSwarmStageMng<ScalarType, OrdinalType>
{
public:
    explicit BoundConstrainedStageMngPSO(const std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> & aObjective) :
            mObjective(aObjective)
    {
    }

    virtual ~BoundConstrainedStageMngPSO()
    {
    }

    void evaluateObjective(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                           Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {
        mObjective->value(aControl, aOutput);
    }

private:
    std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> mObjective;

private:
    BoundConstrainedStageMngPSO(const Plato::BoundConstrainedStageMngPSO<ScalarType, OrdinalType>&);
    Plato::BoundConstrainedStageMngPSO<ScalarType, OrdinalType> & operator=(const Plato::BoundConstrainedStageMngPSO<ScalarType, OrdinalType>&);
};
// class BoundConstrainedStageMngPSO

template<typename ScalarType, typename OrdinalType = size_t>
class AugmentedLagrangianStageMngPSO : public Plato::ParticleSwarmStageMng<ScalarType, OrdinalType>
{
public:
    explicit AugmentedLagrangianStageMngPSO(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                                            const std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> & aObjective,
                                            const std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> & aConstraints) :
            mPenaltyExpansionMultiplier(2),
            mPenaltyContractionMultiplier(0.5),
            mFeasibilityInexactnessTolerance(1e-4),
            mCurrentObjFuncValues(aFactory->objective().create()),
            mPreviousObjFuncValues(aFactory->objective().create()),
            mPenaltyMultipliers(aFactory->dual().create()),
            mLagrangeMultipliers(aFactory->dual().create()),
            mCurrentConstraintValues(aFactory->dual().create()),
            mPreviousConstraintValues(aFactory->dual().create()),
            mObjective(aObjective),
            mConstraints(aConstraints)
    {
        this->initialize();
    }

    virtual ~AugmentedLagrangianStageMngPSO()
    {
    }

    void setPenaltyExpansionMultiplier(const ScalarType & aInput)
    {
        mPenaltyExpansionMultiplier = aInput;
    }

    void setPenaltyContractionMultiplier(const ScalarType & aInput)
    {
        mPenaltyContractionMultiplier = aInput;
    }

    void setFeasibilityInexactnessTolerance(const ScalarType & aInput)
    {
        mFeasibilityInexactnessTolerance = aInput;
    }

    void evaluateObjective(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                           Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {
        // Evaluate objective function
        mPreviousObjFuncValues->update(static_cast<ScalarType>(1), *mCurrentObjFuncValues, static_cast<ScalarType>(0));
        mCurrentObjFuncValues->fill(static_cast<ScalarType>(0));
        mObjective->value(aControl, *mCurrentObjFuncValues);
        aOutput.update(static_cast<ScalarType>(1), *mCurrentObjFuncValues, static_cast<ScalarType>(0));
        // Evaluate constraints
        this->evaluateConstraints(aControl);
        // Evaluate augmented Lagrangian
        this->evaluateAugmentedLagrangian(aOutput);
    }

    void updateLagrangeMultipliers()
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tPenaltyValues = (*mPenaltyMultipliers)[tConstraintIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tConstraintValues = (*mCurrentConstraintValues)[tConstraintIndex];
            Plato::Vector<ScalarType, OrdinalType> & tLagrangeMultipliers = (*mLagrangeMultipliers)[tConstraintIndex];

            const OrdinalType tNumParticles = tConstraintValues.size();
            for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
            {
                const ScalarType tLagMultiplierOverPenalty = -tLagrangeMultipliers[tParticleIndex]
                        / (static_cast<ScalarType>(2) * tPenaltyValues[tParticleIndex]);
                const ScalarType tConstraintValue = std::max(tConstraintValues[tParticleIndex], tLagMultiplierOverPenalty);
                const ScalarType tNewLagMultiplier = tLagrangeMultipliers[tParticleIndex] +
                        (static_cast<ScalarType>(2) * tPenaltyValues[tParticleIndex] * tConstraintValue);
                tLagrangeMultipliers[tParticleIndex] = tNewLagMultiplier;
            }
        }
    }

    void updatePenaltyMultipliers()
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tCurrentConstraint = (*mCurrentConstraintValues)[tConstraintIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tPreviousConstraint = (*mPreviousConstraintValues)[tConstraintIndex];
            Plato::Vector<ScalarType, OrdinalType> & tPenaltyValues = (*mPenaltyMultipliers)[tConstraintIndex];

            const OrdinalType tNumParticles = tPenaltyValues.size();
            for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
            {
                const bool tIsConstraintGreaterThanPrevious = tCurrentConstraint[tParticleIndex] > tPreviousConstraint[tParticleIndex];
                tPenaltyValues[tParticleIndex] = tIsConstraintGreaterThanPrevious ? mPenaltyExpansionMultiplier * tPenaltyValues[tParticleIndex] : tPenaltyValues[tParticleIndex];
                const bool tIsConstraintGreaterThanFeasibilityTol = tCurrentConstraint[tParticleIndex] > mFeasibilityInexactnessTolerance;
                tPenaltyValues[tParticleIndex] = tIsConstraintGreaterThanFeasibilityTol ? mPenaltyExpansionMultiplier * tPenaltyValues[tParticleIndex] : tPenaltyValues[tParticleIndex];
                const bool tIsConstraintLessThanFeasibilityTol = tCurrentConstraint[tParticleIndex] <= mFeasibilityInexactnessTolerance;
                tPenaltyValues[tParticleIndex] = tIsConstraintLessThanFeasibilityTol ? mPenaltyContractionMultiplier * tPenaltyValues[tParticleIndex] : tPenaltyValues[tParticleIndex];
            }
        }
    }

private:
    void initialize()
    {
        Plato::fill(static_cast<ScalarType>(1), *mPenaltyMultipliers);
        Plato::fill(static_cast<ScalarType>(0), *mLagrangeMultipliers);
    }

    /******************************************************************************//**
     * @brief Evaluate inequality constraints
     * @param [in] aControl control multi-vector
    **********************************************************************************/
    void evaluateConstraints(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        Plato::update(static_cast<ScalarType>(1), *mCurrentConstraintValues, static_cast<ScalarType>(0), *mPreviousConstraintValues);

        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            Plato::GradFreeCriteria<ScalarType, OrdinalType> & tConstraint = (*mConstraints)[tIndex];
            Plato::Vector<ScalarType, OrdinalType> & tMyValues = (*mCurrentConstraintValues)[tIndex];
            tMyValues.fill(static_cast<ScalarType>(0));
            tConstraint.value(aControl, tMyValues);
        }
    }

    /******************************************************************************//**
     * @brief Evaluate augmented Lagrangian criterion:
     * \f$ \ell(z, \lambda, \mu) = f(z) + sum_{i=1}^{N}\lambda_i \theta_i(z) + sum_{i=1}^{N}\mu_i \theta_i^2(z) \f$,
     * where \f$ \theta_i = \max( h_i(x), \frac{-\lambda_i}{2\mu_i} ) \f$
     * @param [in,out] aOutput augmented Lagrangian values
    **********************************************************************************/
    void evaluateAugmentedLagrangian(Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tPenaltyMultipliers = (*mPenaltyMultipliers)[tConstraintIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tLagrangeMultipliers = (*mLagrangeMultipliers)[tConstraintIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tConstraintValues = (*mCurrentConstraintValues)[tConstraintIndex];

            const OrdinalType tNumParticles = tConstraintValues.size();
            for(OrdinalType tParticleIndex = 0; tParticleIndex < tNumParticles; tParticleIndex++)
            {
                const ScalarType tLagMultiplierOverPenalty = -tLagrangeMultipliers[tParticleIndex]
                        / (static_cast<ScalarType>(2) * tPenaltyMultipliers[tParticleIndex]);
                const ScalarType tConstraintValue = std::max(tConstraintValues[tParticleIndex], tLagMultiplierOverPenalty);
                const ScalarType tLagrangeMultipliersTimesConstraint = tLagrangeMultipliers[tParticleIndex] * tConstraintValue;
                const ScalarType tConstraintTimesConstraint = tConstraintValue * tConstraintValue;
                const ScalarType tMyConstraintContribution = tLagrangeMultipliersTimesConstraint
                        + tPenaltyMultipliers[tParticleIndex] * tConstraintTimesConstraint;
                aOutput[tParticleIndex] += tMyConstraintContribution;
            }
        }
    }

private:
    ScalarType mPenaltyExpansionMultiplier;
    ScalarType mPenaltyContractionMultiplier;
    ScalarType mFeasibilityInexactnessTolerance;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentObjFuncValues;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mPreviousObjFuncValues;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPenaltyMultipliers;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mLagrangeMultipliers;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentConstraintValues;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousConstraintValues;

    std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> mObjective;
    std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> mConstraints;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mDualReductionOperations;

private:
    AugmentedLagrangianStageMngPSO(const Plato::AugmentedLagrangianStageMngPSO<ScalarType, OrdinalType>&);
    Plato::AugmentedLagrangianStageMngPSO<ScalarType, OrdinalType> & operator=(const Plato::AugmentedLagrangianStageMngPSO<ScalarType, OrdinalType>&);
};
// class AugmentedLagrangianStageMngPSO

template<typename ScalarType, typename OrdinalType = size_t>
class ParticleSwarmOperations
{
public:
    explicit ParticleSwarmOperations(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory) :
        mNumConsecutiveFailures(0),
        mNumConsecutiveSuccesses(0),
        mMaxNumConsecutiveFailures(10),
        mMaxNumConsecutiveSuccesses(10),
        mGlobalBestParticleIndex(0),
        mInertiaMultiplier(0.9),
        mSocialBehaviorMultiplier(0.8),
        mCognitiveBehaviorMultiplier(0.8),
        mBestObjFunValueMean(std::numeric_limits<ScalarType>::max()),
        mBestObjFunValueStdDev(std::numeric_limits<ScalarType>::max()),
        mCurrentGlobalBestObjFunValue(std::numeric_limits<ScalarType>::max()),
        mPreviousGlobalBestObjFunValue(std::numeric_limits<ScalarType>::max()),
        mTrustRegionMultiplier(1),
        mTrustRegionExpansionMultiplier(4.0),
        mTrustRegionContractionMultiplier(0.75),
        mWorkVector(aFactory->objective().create()),
        mBestObjFuncValues(aFactory->objective().create()),
        mCurrentObjFuncValues(aFactory->objective().create()),
        mReductions(aFactory->getObjFuncReductionOperations().create())
    {
        this->initialize();
    }

    virtual ~ParticleSwarmOperations()
    {
    }

    ScalarType getBestObjFuncValueMean() const
    {
        return (mBestObjFunValueMean);
    }

    ScalarType getBestObjFuncValueStdDev() const
    {
        return (mBestObjFunValueStdDev);
    }

    ScalarType getTrustRegionMultiplier() const
    {
        return (mTrustRegionMultiplier);
    }

    ScalarType getCurrentGlobalBestObjFunValue() const
    {
        return (mCurrentGlobalBestObjFunValue);
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

    void setCurrentGlobalBestObjFunValue(const ScalarType & aInput)
    {
        mCurrentGlobalBestObjFunValue = aInput;
    }

    void setTrustRegionExpansionMultiplier(const ScalarType & aInput)
    {
        mTrustRegionExpansionMultiplier = aInput;
    }

    void setTrustRegionContractionMultiplier(const ScalarType & aInput)
    {
        mTrustRegionContractionMultiplier = aInput;
    }

    void setMaxNumConsecutiveFailures(const OrdinalType & aInput)
    {
        mMaxNumConsecutiveFailures = aInput;
    }

    void setMaxNumConsecutiveSuccesses(const OrdinalType & aInput)
    {
        mMaxNumConsecutiveSuccesses = aInput;
    }

    void setCurrentObjFuncValues(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        mCurrentObjFuncValues->update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
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

    void computeBestObjFunStatistics()
    {
        this->computeBestObjFunMean();
        this->computeBestObjFunStdDev();
    }

    void findBestParticlePositions(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        mPreviousGlobalBestObjFunValue = mCurrentGlobalBestObjFunValue;
        const OrdinalType tNumParticles = aDataMng.getNumParticles();
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            this->checkBestParticlePosition(tIndex, aDataMng);
            this->checkGlobalBestParticlePosition(tIndex, aDataMng);
        }
        this->checkGlobalBestParticleUpdateSuccessRate();
    }

    void updateParticleVelocities(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        aDataMng.cachePreviousVelocities();
        std::default_random_engine tGenerator;
        std::uniform_real_distribution<ScalarType> tDistribution(0.0 /* lower bound */, 1.0 /* upper bound */);
        const OrdinalType tNumParticles = aDataMng.getNumParticles();
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            if(tIndex != mGlobalBestParticleIndex)
            {
                this->updateParticleVelocity(tIndex, tGenerator, tDistribution, aDataMng);
            }
            else
            {
                assert(tIndex == mGlobalBestParticleIndex);
                this->updateGlobalBestParticleVelocity(tGenerator, tDistribution, aDataMng);
            }
        }
    }

    void updateParticlePositions(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const OrdinalType tNumParticles = aDataMng.getNumParticles();
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            if(tIndex != mGlobalBestParticleIndex)
            {
                this->updateParticlePosition(tIndex, aDataMng);
            }
            else
            {
                assert(tIndex == mGlobalBestParticleIndex);
                this->updateGlobalBestParticlePosition(aDataMng);
            }
        }
    }

private:
    void initialize()
    {
        mBestObjFuncValues->fill(std::numeric_limits<ScalarType>::max());
        mCurrentObjFuncValues->fill(std::numeric_limits<ScalarType>::max());
    }

    void computeBestObjFunMean()
    {
        mBestObjFunValueMean = mReductions->sum(*mBestObjFuncValues);
        const OrdinalType tNumParticles = mBestObjFuncValues->size();
        mBestObjFunValueMean = mBestObjFunValueMean / tNumParticles;
    }

    void computeBestObjFunStdDev()
    {
        mBestObjFunValueStdDev = 0;
        const OrdinalType tNumParticles = mBestObjFuncValues->size();
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            const ScalarType tMisfit = (*mBestObjFuncValues)[tIndex] - mBestObjFunValueMean;
            mBestObjFunValueStdDev += tMisfit * tMisfit;
        }
        mBestObjFunValueStdDev = mBestObjFunValueStdDev / (tNumParticles - static_cast<OrdinalType>(1));
        mBestObjFunValueStdDev = std::pow(mBestObjFunValueStdDev, static_cast<ScalarType>(0.5));
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

    void checkBestParticlePosition(const OrdinalType & aParticleIndex,
                                   Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        if((*mCurrentObjFuncValues)[aParticleIndex] < (*mBestObjFuncValues)[aParticleIndex])
        {
            const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticle = aDataMng.getCurrentParticle(aParticleIndex);
            aDataMng.setBestParticlePosition(aParticleIndex, tCurrentParticle);
            (*mBestObjFuncValues)[aParticleIndex] = (*mCurrentObjFuncValues)[aParticleIndex];
        }
    }

    void checkGlobalBestParticlePosition(const OrdinalType & aParticleIndex,
                                         Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const ScalarType tCurrentParticleObjFuncValue = (*mCurrentObjFuncValues)[aParticleIndex];
        if(tCurrentParticleObjFuncValue < mCurrentGlobalBestObjFunValue)
        {
            mCurrentGlobalBestObjFunValue = tCurrentParticleObjFuncValue;
            const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticle = aDataMng.getCurrentParticle(aParticleIndex);
            aDataMng.setGlobalBestParticlePosition(tCurrentParticle);
            mGlobalBestParticleIndex = aParticleIndex;
        }
    }

    void updateParticleVelocity(const OrdinalType & aParticleIndex,
                                std::default_random_engine & aGenerator,
                                std::uniform_real_distribution<ScalarType> & aDistribution,
                                Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const Plato::Vector<ScalarType, OrdinalType> & tPreviousVel = aDataMng.getPreviousVelocity(aParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticle = aDataMng.getCurrentParticle(aParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalBestParticlePosition = aDataMng.getGlobalBestParticlePosition();
        const Plato::Vector<ScalarType, OrdinalType> & tBestParticlePosition = aDataMng.getBestParticlePosition(aParticleIndex);

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
            (*mWorkVector)[tIndex] = tInertiaValue + tCognitiveValue + tSocialValue;
        }

        aDataMng.setCurrentVelocity(aParticleIndex, *mWorkVector);
    }

    void updateGlobalBestParticleVelocity(std::default_random_engine & aGenerator,
                                          std::uniform_real_distribution<ScalarType> & aDistribution,
                                          Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const Plato::Vector<ScalarType, OrdinalType> & tPreviousVel = aDataMng.getPreviousVelocity(mGlobalBestParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalBestParticlePosition = aDataMng.getGlobalBestParticlePosition();
        const Plato::Vector<ScalarType, OrdinalType> & tCurrentParticlePosition = aDataMng.getCurrentParticle(mGlobalBestParticleIndex);

        const OrdinalType tNumControls = tPreviousVel.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            const ScalarType tRandomNum = aDistribution(aGenerator);
            const ScalarType tStochasticTrustRegionMultiplier = mTrustRegionMultiplier
                    * (static_cast<ScalarType>(1) - static_cast<ScalarType>(2) * tRandomNum);
            (*mWorkVector)[tIndex] = (static_cast<ScalarType>(-1) * tCurrentParticlePosition[tIndex]) + tGlobalBestParticlePosition[tIndex]
                    + (mInertiaMultiplier * tPreviousVel[tIndex]) + tStochasticTrustRegionMultiplier;
        }
        aDataMng.setCurrentVelocity(mGlobalBestParticleIndex, *mWorkVector);
    }

    void updateParticlePosition(const OrdinalType & aParticleIndex,
                                Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const ScalarType tTimeStep = aDataMng.getTimeStep();
        const Plato::Vector<ScalarType, OrdinalType> & tLowerBounds = aDataMng.getLowerBounds();
        const Plato::Vector<ScalarType, OrdinalType> & tUpperBounds = aDataMng.getUpperBounds();
        const Plato::Vector<ScalarType, OrdinalType> & tParticleVel = aDataMng.getCurrentVelocity(aParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tParticlePosition = aDataMng.getCurrentParticle(aParticleIndex);
        mWorkVector->update(static_cast<ScalarType>(1), tParticlePosition, static_cast<ScalarType>(0));

        const OrdinalType tNumControls = tParticleVel.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            (*mWorkVector)[tIndex] = (*mWorkVector)[tIndex] + (tTimeStep * tParticleVel[tIndex]);
            (*mWorkVector)[tIndex] = std::max((*mWorkVector)[tIndex], tLowerBounds[tIndex]);
            (*mWorkVector)[tIndex] = std::min((*mWorkVector)[tIndex], tUpperBounds[tIndex]);
        }

        aDataMng.setCurrentParticle(aParticleIndex, *mWorkVector);
    }

    void updateGlobalBestParticlePosition(Plato::ParticleSwarmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const Plato::Vector<ScalarType, OrdinalType> & tLowerBounds = aDataMng.getLowerBounds();
        const Plato::Vector<ScalarType, OrdinalType> & tUpperBounds = aDataMng.getUpperBounds();
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalBestParticleVel = aDataMng.getCurrentVelocity(mGlobalBestParticleIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalBestParticlePosition = aDataMng.getGlobalBestParticlePosition();

        std::default_random_engine tGenerator;
        std::uniform_real_distribution<ScalarType> tDistribution(0.0 /* lower bound */, 1.0 /* upper bound */);
        const ScalarType tRandomNum = tDistribution(tGenerator);
        const ScalarType tStochasticTrustRegionMultiplier = mTrustRegionMultiplier
                * (static_cast<ScalarType>(1) - static_cast<ScalarType>(2) * tRandomNum);

        const OrdinalType tNumControls = tGlobalBestParticlePosition.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            (*mWorkVector)[tIndex] = tGlobalBestParticlePosition[tIndex]
                    + (mInertiaMultiplier * tGlobalBestParticleVel[tIndex]) + tStochasticTrustRegionMultiplier;
        }

        aDataMng.setCurrentParticle(mGlobalBestParticleIndex, *mWorkVector);
    }

private:
    OrdinalType mNumConsecutiveFailures;
    OrdinalType mNumConsecutiveSuccesses;
    OrdinalType mMaxNumConsecutiveFailures;
    OrdinalType mMaxNumConsecutiveSuccesses;

    OrdinalType mGlobalBestParticleIndex;

    ScalarType mInertiaMultiplier;
    ScalarType mSocialBehaviorMultiplier;
    ScalarType mCognitiveBehaviorMultiplier;

    ScalarType mBestObjFunValueMean;
    ScalarType mBestObjFunValueStdDev;

    ScalarType mCurrentGlobalBestObjFunValue;
    ScalarType mPreviousGlobalBestObjFunValue;

    ScalarType mTrustRegionMultiplier;
    ScalarType mTrustRegionExpansionMultiplier;
    ScalarType mTrustRegionContractionMultiplier;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mWorkVector;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mBestObjFuncValues;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentObjFuncValues;

    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mReductions;

private:
    ParticleSwarmOperations(const Plato::ParticleSwarmOperations<ScalarType, OrdinalType>&);
    Plato::ParticleSwarmOperations<ScalarType, OrdinalType> & operator=(const Plato::ParticleSwarmOperations<ScalarType, OrdinalType>&);
};

template<typename ScalarType, typename OrdinalType = size_t>
class BoundConstrainedPSO
{
public:
    BoundConstrainedPSO(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                        const std::shared_ptr<Plato::GradFreeCriteria<ScalarType, OrdinalType>> & aObjective) :
            mPrintDiagnostics(false),
            mNumIterations(0),
            mMaxNumIterations(1000),
            mBestObjFuncTolerance(1e-10),
            mMeanObjFuncTolerance(5e-4),
            mStopCriterion(Plato::particle_swarm::DID_NOT_CONVERGE),
            mWorkVector(aFactory->objective().create()),
            mDataMng(std::make_shared<Plato::ParticleSwarmDataMng<ScalarType, OrdinalType>>(aFactory)),
            mStageMng(std::make_shared<Plato::BoundConstrainedStageMngPSO<ScalarType, OrdinalType>>(aObjective)),
            mOperations(std::make_shared<Plato::ParticleSwarmOperations<ScalarType, OrdinalType>>(aFactory))
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~BoundConstrainedPSO()
    {
    }

    /******************************************************************************//**
     * @brief Enable output of diagnostics (i.e. optimization problem status)
    **********************************************************************************/
    void enableDiagnostics()
    {
        mPrintDiagnostics = true;
    }

    void setMaxNumIterations(const OrdinalType & aInput)
    {
        mMaxNumIterations = aInput;
    }

    void setMaxNumConsecutiveFailures(const OrdinalType & aInput)
    {
        mOperations->setMaxNumConsecutiveFailures(aInput);
    }

    void setMaxNumConsecutiveSuccesses(const OrdinalType & aInput)
    {
        mOperations->setMaxNumConsecutiveSuccesses(aInput);
    }

    void setInertiaMultiplier(const ScalarType & aInput)
    {
        mOperations->setInertiaMultiplier(aInput);
    }

    void setCognitiveMultiplier(const ScalarType & aInput)
    {
        mOperations->setCognitiveMultiplier(aInput);
    }

    void setSocialBehaviorMultiplier(const ScalarType & aInput)
    {
        mOperations->setSocialBehaviorMultiplier(aInput);
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

    OrdinalType getNumIterations() const
    {
        return (mNumIterations);
    }

    ScalarType getBestObjFuncValueMean() const
    {
        return (mOperations->getBestObjFuncValueMean());
    }

    ScalarType getBestObjFuncValueStdDev() const
    {
        return (mOperations->getBestObjFuncValueStdDev());
    }

    ScalarType getBestObjFuncValue() const
    {
        return (mOperations->getCurrentGlobalBestObjFunValue());
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
        assert(mStageMng.use_count() > static_cast<OrdinalType>(0));

        this->openOutputFile();
        mOperations->checkInertiaMultiplier();
        mDataMng->setInitialParticles();

        mNumIterations = 0;
        while(1)
        {
            mNumIterations++;
            mStageMng->evaluateObjective(mDataMng->getCurrentParticles(), *mWorkVector);
            mOperations->setCurrentObjFuncValues(*mWorkVector);
            mOperations->findBestParticlePositions(*mDataMng);
            mOperations->updateParticleVelocities(*mDataMng);
            mOperations->updateParticlePositions(*mDataMng);
            mOperations->updateTrustRegionMultiplier();
            mOperations->computeBestObjFunStatistics();
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
    bool checkStoppingCriteria()
    {
        bool tStop = false;
        const ScalarType tBestObjFunValueMean = mOperations->getBestObjFuncValueMean();
        const ScalarType tCurrentGlobalBestObjFunValue = mOperations->getCurrentGlobalBestObjFunValue();

        if(mNumIterations >= mMaxNumIterations)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::MAX_NUMBER_ITERATIONS;
        }
        else if(tCurrentGlobalBestObjFunValue < mBestObjFuncTolerance)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::TRUE_OBJECTIVE_TOLERANCE;
        }
        else if(tBestObjFunValueMean < mMeanObjFuncTolerance)
        {
            tStop = true;
            mStopCriterion = Plato::particle_swarm::MEAN_OBJECTIVE_TOLERANCE;
        }

        return (tStop);
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

        const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
        if(tMyCommWrapper.myProcID() == 0)
        {
            mOutputData.mNumIter = mNumIterations;
            mOutputData.mObjFuncCount = mNumIterations * mDataMng->getNumParticles();
            mOutputData.mBestObjFuncValue = mOperations->getCurrentGlobalBestObjFunValue();
            mOutputData.mMeanObjFuncValue = mOperations->getBestObjFuncValueMean();
            mOutputData.mStdDevObjFuncValue = mOperations->getBestObjFuncValueStdDev();
            mOutputData.mTrustRegionMultiplier = mOperations->getTrustRegionMultiplier();

            Plato::pso::print_pso_diagnostics(mOutputData, mOutputStream, mPrintDiagnostics);
        }
    }

private:
    bool mPrintDiagnostics;
    std::ofstream mOutputStream;

    OrdinalType mNumIterations;
    OrdinalType mMaxNumIterations;

    ScalarType mBestObjFuncTolerance;
    ScalarType mMeanObjFuncTolerance;

    Plato::particle_swarm::stop_t mStopCriterion;
    Plato::OutputDataPSO<ScalarType, OrdinalType> mOutputData;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mWorkVector;
    std::shared_ptr<Plato::ParticleSwarmDataMng<ScalarType, OrdinalType>> mDataMng;
    std::shared_ptr<Plato::ParticleSwarmStageMng<ScalarType, OrdinalType>> mStageMng;
    std::shared_ptr<Plato::ParticleSwarmOperations<ScalarType, OrdinalType>> mOperations;

private:
    BoundConstrainedPSO(const Plato::BoundConstrainedPSO<ScalarType, OrdinalType>&);
    Plato::BoundConstrainedPSO<ScalarType, OrdinalType> & operator=(const Plato::BoundConstrainedPSO<ScalarType, OrdinalType>&);
};
// class BoundConstrainedPSO

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

TEST(PlatoTest, PSO_BoundConstrainedStageMng)
{
    const size_t tNumControls = 2;
    const size_t tNumParticles = 1;
    Plato::StandardVector<double> tObjFuncValues(tNumParticles);
    Plato::StandardMultiVector<double> tControl(tNumParticles, tNumControls, 2.0 /* initial guess */);

    std::shared_ptr<Plato::GradFreeRosenbrock<double>> tObjective = std::make_shared<Plato::GradFreeRosenbrock<double>>();
    Plato::BoundConstrainedStageMngPSO<double> tStageMng(tObjective);
    tStageMng.evaluateObjective(tControl, tObjFuncValues);

    const double tTolerance = 1e-6;
    EXPECT_NEAR(401, tObjFuncValues[0], tTolerance);
}

TEST(PlatoTest, PSO_Solve)
{
    // ********* Allocate Core Optimization Data Templates *********
    std::shared_ptr<Plato::DataFactory<double>> tFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    const size_t tNumParticles = 20;
    tFactory->allocateObjFuncValues(tNumParticles);
    tFactory->allocateControl(tNumControls, tNumParticles);

    // TEST ALGORITHM
    std::shared_ptr<Plato::GradFreeRosenbrock<double>> tObjective = std::make_shared<Plato::GradFreeRosenbrock<double>>();
    Plato::BoundConstrainedPSO<double> tAlgorithm(tFactory, tObjective);
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
