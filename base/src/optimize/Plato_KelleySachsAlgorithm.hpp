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
    explicit KelleySachsAlgorithm(const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory) :
            mMaxNumUpdates(10),
            mMaxNumOuterIterations(100),
            mNumOuterIterationsDone(0),
            mGradientTolerance(1e-4),
            mStationarityTolerance(1e-4),
            mObjectiveStagnationTolerance(1e-8),
            mControlStagnationTolerance(std::numeric_limits<ScalarType>::epsilon()),
            mActualReductionTolerance(1e-8),
            mStoppingCriterion(Plato::algorithm::NOT_CONVERGED),
            mControlWorkVector(aDataFactory.control().create())
    {
    }
    virtual ~KelleySachsAlgorithm()
    {
    }

    void setGradientTolerance(const ScalarType & aInput)
    {
        mGradientTolerance = aInput;
    }
    void setStationarityTolerance(const ScalarType & aInput)
    {
        mStationarityTolerance = aInput;
    }
    void setObjectiveStagnationTolerance(const ScalarType & aInput)
    {
        mObjectiveStagnationTolerance = aInput;
    }
    void setControlStagnationTolerance(const ScalarType & aInput)
    {
        mControlStagnationTolerance = aInput;
    }
    void setActualReductionTolerance(const ScalarType & aInput)
    {
        mActualReductionTolerance = aInput;
    }

    void setMaxNumUpdates(const OrdinalType & aInput)
    {
        mMaxNumUpdates = aInput;
    }
    void setNumIterationsDone(const OrdinalType & aInput)
    {
        mNumOuterIterationsDone = aInput;
    }
    void setMaxNumIterations(const OrdinalType & aInput)
    {
        mMaxNumOuterIterations = aInput;
    }
    void setStoppingCriterion(const Plato::algorithm::stop_t & aInput)
    {
        mStoppingCriterion = aInput;
    }

    ScalarType getStationarityMeasure() const
    {
        return (mStationarityMeasure);
    }
    ScalarType getGradientTolerance() const
    {
        return (mGradientTolerance);
    }
    ScalarType getStationarityTolerance() const
    {
        return (mStationarityTolerance);
    }
    ScalarType getObjectiveStagnationTolerance() const
    {
        return (mObjectiveStagnationTolerance);
    }
    ScalarType getControlStagnationTolerance() const
    {
        return (mControlStagnationTolerance);
    }
    ScalarType getActualReductionTolerance() const
    {
        return (mActualReductionTolerance);
    }

    OrdinalType getMaxNumUpdates() const
    {
        return (mMaxNumUpdates);
    }
    OrdinalType getNumIterationsDone() const
    {
        return (mNumOuterIterationsDone);
    }
    OrdinalType getMaxNumIterations() const
    {
        return (mMaxNumOuterIterations);
    }
    Plato::algorithm::stop_t getStoppingCriterion() const
    {
        return (mStoppingCriterion);
    }

    bool updateControl(const Plato::MultiVector<ScalarType, OrdinalType> & aMidGradient,
                       Plato::KelleySachsStepMng<ScalarType, OrdinalType> & aStepMng,
                       Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng,
                       Plato::TrustRegionStageMng<ScalarType, OrdinalType> & aStageMng)
    {
        bool tControlUpdated = false;

        ScalarType tXi = 1.;
        ScalarType tBeta = 1e-2;
        ScalarType tAlpha = tBeta;
        ScalarType tMu = static_cast<ScalarType>(1) - static_cast<ScalarType>(1e-4);

        ScalarType tMidActualReduction = aStepMng.getActualReduction();
        ScalarType tMidObjectiveValue = aStepMng.getMidPointObjectiveFunctionValue();
        const Plato::MultiVector<ScalarType, OrdinalType> & tMidControl = aStepMng.getMidPointControls();
        const Plato::MultiVector<ScalarType, OrdinalType> & tLowerBounds = aDataMng.getControlLowerBounds();
        const Plato::MultiVector<ScalarType, OrdinalType> & tUpperBounds = aDataMng.getControlUpperBounds();

        OrdinalType tIteration = 0;
        while(tIteration < mMaxNumUpdates)
        {
            // Compute trial point based on the mid gradient (i.e. mid steepest descent)
            ScalarType tLambda = -tXi / tAlpha;
            Plato::update(static_cast<ScalarType>(1), tMidControl, static_cast<ScalarType>(0), *mControlWorkVector);
            Plato::update(tLambda, aMidGradient, static_cast<ScalarType>(1), *mControlWorkVector);
            aDataMng.bounds().project(tLowerBounds, tUpperBounds, *mControlWorkVector);

            // Compute trial objective function
            ScalarType tTolerance = aStepMng.getObjectiveInexactnessTolerance();
            ScalarType tTrialObjectiveValue = aStageMng.evaluateObjective(*mControlWorkVector, tTolerance);
            // Compute actual reduction
            ScalarType tTrialActualReduction = tTrialObjectiveValue - tMidObjectiveValue;
            // Check convergence
            if(tTrialActualReduction < -tMu * tMidActualReduction)
            {
                tControlUpdated = true;
                aDataMng.setCurrentControl(*mControlWorkVector);
                aDataMng.setCurrentObjectiveFunctionValue(tTrialObjectiveValue);
                break;
            }
            // Compute scaling for next iteration
            if(tIteration == 1)
            {
                tXi = tAlpha;
            }
            else
            {
                tXi = tXi * tBeta;
            }
            tIteration++;
        }

        return (tControlUpdated);
    }

    virtual void solve() = 0;

private:
    OrdinalType mMaxNumUpdates;
    OrdinalType mMaxNumOuterIterations;
    OrdinalType mNumOuterIterationsDone;

    ScalarType mGradientTolerance;
    ScalarType mStationarityTolerance;
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
