/*
 * Plato_KelleySachsAlgorithm.hpp
 *
 *  Created on: Oct 21, 2017
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

#ifndef PLATO_KELLEYSACHSALGORITHM_HPP_
#define PLATO_KELLEYSACHSALGORITHM_HPP_

#include <memory>

#include "Plato_Types.hpp"
#include "Plato_Vector.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_KelleySachsStepMng.hpp"
#include "Plato_TrustRegionStageMng.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class KelleySachsAlgorithm
{
public:
    /******************************************************************************//**
     * @brief Default constructor
     * @param [in] aDataFactory data factory
    **********************************************************************************/
    explicit KelleySachsAlgorithm(const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory) :
            mIsPostSmoothingActive(true),
            mMaxNumLineSearchItr(10),
            mNumLineSearchItrDone(0),
            mMaxNumOuterIterations(100),
            mNumOuterIterationsDone(0),
            mGradientTolerance(1e-4),
            mStationarityTolerance(1e-4),
            mPostSmoothingAlphaScale(1e-4),
            mObjectiveStagnationTolerance(1e-8),
            mControlStagnationTolerance(std::numeric_limits<ScalarType>::epsilon()),
            mActualReductionTolerance(1e-8),
            mStoppingCriterion(Plato::algorithm::NOT_CONVERGED),
            mControlWorkVector(aDataFactory.control().create())
    {
    }

    /******************************************************************************//**
     * @brief Default destructor
    **********************************************************************************/
    virtual ~KelleySachsAlgorithm()
    {
    }

    /******************************************************************************//**
     * @brief Disable port smoothing operation
    **********************************************************************************/
    void disablePostSmoothing()
    {
        mIsPostSmoothingActive = false;
    }

    /******************************************************************************//**
     * @brief Check if post smoothing operation is active
     * @return post smoothing operation flag
    **********************************************************************************/
    bool isPostSmoothingActive() const
    {
        return (mIsPostSmoothingActive);
    }
    /******************************************************************************//**
     * @brief Set post smoothing operation scale
     * @param [in] aInput post smoothing operation scale
    **********************************************************************************/
    void setPostSmoothingScale(const ScalarType & aInput)
    {
        mPostSmoothingAlphaScale = aInput;
    }

    /******************************************************************************//**
     * @brief Set stopping tolerance based on norm of the projected gradient.
     * @param [in] aInput stopping tolerance based on norm of the projected gradient
    **********************************************************************************/
    void setGradientTolerance(const ScalarType & aInput)
    {
        mGradientTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set stopping tolerance based on norm of the projected descent direction.
     * @param [in] aInput stopping tolerance based on norm of the projected descent direction
    **********************************************************************************/
    void setStationarityTolerance(const ScalarType & aInput)
    {
        mStationarityTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set stopping tolerance based on the objective function stagnation metric
     * @param [in] aInput stopping tolerance based on the objective function stagnation
    **********************************************************************************/
    void setObjectiveStagnationTolerance(const ScalarType & aInput)
    {
        mObjectiveStagnationTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set stopping tolerance based on the control stagnation metric
     * @param [in] aInput stopping tolerance based on the control stagnation
    **********************************************************************************/
    void setControlStagnationTolerance(const ScalarType & aInput)
    {
        mControlStagnationTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set stopping tolerance based on minimum actual reduction
     * @param [in] aInput stopping tolerance based on minimum actual reduction
    **********************************************************************************/
    void setActualReductionTolerance(const ScalarType & aInput)
    {
        mActualReductionTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set number of line search iterations
     * @param [in] aInput number of line search iterations
    **********************************************************************************/
    void setMaxNumLineSearchIterations(const OrdinalType & aInput)
    {
        mMaxNumLineSearchItr = aInput;
    }

    /******************************************************************************//**
     * @brief Set number of line search iterations (i.e. smoothing iterations) done
     * @param [in] aInput number of line search iterations done
    **********************************************************************************/
    void setNumLineSearchItrDone(const OrdinalType & aInput)
    {
        mNumLineSearchItrDone = aInput;
    }

    /******************************************************************************//**
     * @brief Set number of outer optimization iterations
     * @param [in] aInput number of outer optimization iterations
    **********************************************************************************/
    void setNumIterationsDone(const OrdinalType & aInput)
    {
        mNumOuterIterationsDone = aInput;
    }

    /******************************************************************************//**
     * @brief Set maximum number of outer optimization iterations
     * @param [in] aInput maximum number of outer optimization iterations
    **********************************************************************************/
    void setMaxNumIterations(const OrdinalType & aInput)
    {
        mMaxNumOuterIterations = aInput;
    }

    /******************************************************************************//**
     * @brief Set stopping criterion
     * @param [in] aInput stopping criterion
    **********************************************************************************/
    void setStoppingCriterion(const Plato::algorithm::stop_t & aInput)
    {
        mStoppingCriterion = aInput;
    }

    /******************************************************************************//**
     * @brief Return norm of projected descent direction
     * @return stationarity measure
    **********************************************************************************/
    ScalarType getStationarityMeasure() const
    {
        return (mStationarityMeasure);
    }

    /******************************************************************************//**
     * @brief Return tolerance on the norm of projected gradient
     * @return tolerance on the norm of projected gradient
    **********************************************************************************/
    ScalarType getGradientTolerance() const
    {
        return (mGradientTolerance);
    }

    /******************************************************************************//**
     * @brief Return tolerance on the norm of projected descent direction
     * @return tolerance on the norm of projected descent direction
    **********************************************************************************/
    ScalarType getStationarityTolerance() const
    {
        return (mStationarityTolerance);
    }

    /******************************************************************************//**
     * @brief Return tolerance on the objective stagnation metric
     * @return tolerance on the objective stagnation metric
    **********************************************************************************/
    ScalarType getObjectiveStagnationTolerance() const
    {
        return (mObjectiveStagnationTolerance);
    }

    /******************************************************************************//**
     * @brief Return tolerance on the control stagnation metric
     * @return tolerance on the control stagnation metric
    **********************************************************************************/
    ScalarType getControlStagnationTolerance() const
    {
        return (mControlStagnationTolerance);
    }

    /******************************************************************************//**
     * @brief Return tolerance on the actual reduction
     * @return tolerance on the actual reduction
    **********************************************************************************/
    ScalarType getActualReductionTolerance() const
    {
        return (mActualReductionTolerance);
    }

    /******************************************************************************//**
     * @brief Return maximum number of line search iterations
     * @return maximum number of line search iterations
    **********************************************************************************/
    OrdinalType getMaxNumLineSearchItr() const
    {
        return (mMaxNumLineSearchItr);
    }

    /******************************************************************************//**
     * @brief Return number of line search iterations done
     * @return number of line search iterations done
    **********************************************************************************/
    OrdinalType getNumLineSearchItrDone() const
    {
        return (mNumLineSearchItrDone);
    }

    /******************************************************************************//**
     * @brief Return number of outer optimization iterations
     * @return number of outer optimization iterations
    **********************************************************************************/
    OrdinalType getNumIterationsDone() const
    {
        return (mNumOuterIterationsDone);
    }

    /******************************************************************************//**
     * @brief Return maximum number of outer optimization iterations
     * @return maximum number of outer optimization iterations
    **********************************************************************************/
    OrdinalType getMaxNumIterations() const
    {
        return (mMaxNumOuterIterations);
    }

    /******************************************************************************//**
     * @brief Return stopping criterion
     * @return stopping criterion
    **********************************************************************************/
    Plato::algorithm::stop_t getStoppingCriterion() const
    {
        return (mStoppingCriterion);
    }

    /******************************************************************************//**
     * @brief Return scale factor for backtracking line search routine
     * @param [in] aIteration current line search iteration
     * @param [in] aPreviousScale scale factor for previous line search iteration
     * @return line search scale factor
    **********************************************************************************/
    ScalarType computeLineSearchScale(const OrdinalType & aIteration, const ScalarType & aPreviousScale)
    {
        ScalarType tOuput = 0;
        if(aIteration == 1)
        {
            tOuput = mPostSmoothingAlphaScale;
        }
        else
        {
            ScalarType tBeta = 1e-2;
            tOuput = aPreviousScale * tBeta;
        }
        return (tOuput);
    }

    bool updateControl(const Plato::MultiVector<ScalarType, OrdinalType> & aMidGradient,
                       Plato::KelleySachsStepMng<ScalarType, OrdinalType> & aStepMng,
                       Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng,
                       Plato::TrustRegionStageMng<ScalarType, OrdinalType> & aStageMng)
    {
        bool tControlUpdated = false;

        const ScalarType tMidActualReduction = aStepMng.getActualReduction();
        const ScalarType tMu = static_cast<ScalarType>(1) - static_cast<ScalarType>(1e-4);
        ScalarType tMuTimesMidActualReduction = -tMu * tMidActualReduction;
        const ScalarType tMidObjectiveValue = aStepMng.getMidPointObjectiveFunctionValue();
        const Plato::MultiVector<ScalarType, OrdinalType> & tMidControl = aStepMng.getMidPointControls();
        const Plato::MultiVector<ScalarType, OrdinalType> & tLowerBounds = aDataMng.getControlLowerBounds();
        const Plato::MultiVector<ScalarType, OrdinalType> & tUpperBounds = aDataMng.getControlUpperBounds();

        ScalarType tLambda = 1.;
        OrdinalType tIteration = 1;
        while(tIteration <= mMaxNumLineSearchItr)
        {
            // Compute trial point based on the mid gradient (i.e. mid steepest descent)
            Plato::update(static_cast<ScalarType>(1), tMidControl, static_cast<ScalarType>(0), *mControlWorkVector);
            Plato::update(-tLambda, aMidGradient, static_cast<ScalarType>(1), *mControlWorkVector);
            aDataMng.bounds().project(tLowerBounds, tUpperBounds, *mControlWorkVector);

            // Compute trial objective function
            ScalarType tTolerance = aStepMng.getObjectiveInexactnessTolerance();
            ScalarType tTrialObjectiveValue = aStageMng.evaluateObjective(*mControlWorkVector, tTolerance);
            // Compute actual reduction
            ScalarType tTrialActualReduction = tTrialObjectiveValue - tMidObjectiveValue;
            // Check convergence
            if(tTrialActualReduction < tMuTimesMidActualReduction || tIteration == mMaxNumLineSearchItr)
            {
                tControlUpdated = true;
                aDataMng.setCurrentControl(*mControlWorkVector);
                aDataMng.setCurrentObjectiveFunctionValue(tTrialObjectiveValue);
                break;
            }

            // Compute scaling for next iteration
            tLambda = this->computeLineSearchScale(tIteration, tLambda);
            tIteration++;
        }
        this->setNumLineSearchItrDone(tIteration);

        return (tControlUpdated);
    }

    virtual void solve() = 0;

private:
    bool mIsPostSmoothingActive;

    OrdinalType mMaxNumLineSearchItr;
    OrdinalType mNumLineSearchItrDone;
    OrdinalType mMaxNumOuterIterations;
    OrdinalType mNumOuterIterationsDone;

    ScalarType mGradientTolerance;
    ScalarType mStationarityTolerance;
    ScalarType mPostSmoothingAlphaScale;
    ScalarType mObjectiveStagnationTolerance;
    ScalarType mControlStagnationTolerance;
    ScalarType mStationarityMeasure;
    ScalarType mActualReductionTolerance;

    Plato::algorithm::stop_t mStoppingCriterion;

    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mControlWorkVector;

private:
    KelleySachsAlgorithm(const Plato::KelleySachsAlgorithm<ScalarType, OrdinalType> & aRhs);
    Plato::KelleySachsAlgorithm<ScalarType, OrdinalType> & operator=(const Plato::KelleySachsAlgorithm<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_KELLEYSACHSALGORITHM_HPP_ */
