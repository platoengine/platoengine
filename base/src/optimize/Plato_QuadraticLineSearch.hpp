/*
 * Plato_QuadraticLineSearch.hpp
 *
 *  Created on: Oct 22, 2017
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

#ifndef PLATO_QUADRATICLINESEARCH_HPP_
#define PLATO_QUADRATICLINESEARCH_HPP_

#include <cmath>
#include <vector>
#include <memory>

#include "Plato_LineSearch.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_StateManager.hpp"
#include "Plato_LinearAlgebra.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class QuadraticLineSearch : public Plato::LineSearch<ScalarType, OrdinalType>
{
public:
    explicit QuadraticLineSearch(const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory) :
            mMaxNumIterations(50),
            mNumIterationsDone(0),
            mStepLowerBound(1e-3),
            mStepUpperBound(0.5),
            mContractionFactor(0.5),
            mInitialTrialStepDotCurrentGradient(0),
            mStepValues(2, static_cast<ScalarType>(0)),
            mTrialControl(aDataFactory.control().create())
    {
    }
    virtual ~QuadraticLineSearch()
    {
    }

    OrdinalType getNumIterationsDone() const
    {
        return (mNumIterationsDone);
    }
    ScalarType getStepValue() const
    {
        return (mStepValues[1]);
    }
    void setMaxNumIterations(const OrdinalType & aInput)
    {
        mMaxNumIterations = aInput;
    }
    void setStepLowerBound(const ScalarType & aInput)
    {
        mStepLowerBound = aInput;
    }
    void setStepUpperBound(const ScalarType & aInput)
    {
        mStepUpperBound = aInput;
    }
    void setContractionFactor(const ScalarType & aInput)
    {
        mContractionFactor = aInput;
    }

    void step(Plato::StateManager<ScalarType, OrdinalType> & aStateMng)
    {
        OrdinalType tSize = 3;
        // tObjectiveFunction[0] = current trial value
        // tObjectiveFunction[1] = old trial value
        // tObjectiveFunction[2] = new trial value
        std::vector<ScalarType> tObjectiveFunction(tSize);
        tObjectiveFunction[0] = aStateMng.getCurrentObjectiveValue();

        ScalarType tNormTrialStep = Plato::dot(aStateMng.getTrialStep(), aStateMng.getTrialStep());
        tNormTrialStep = std::sqrt(tNormTrialStep);
        mStepValues[1] = std::min(static_cast<ScalarType>(1),
                                  static_cast<ScalarType>(100) / (static_cast<ScalarType>(1) + tNormTrialStep));

        Plato::update(static_cast<ScalarType>(1),
                      aStateMng.getCurrentControl(),
                      static_cast<ScalarType>(0),
                      *mTrialControl);
        Plato::update(mStepValues[1], aStateMng.getTrialStep(), static_cast<ScalarType>(1), *mTrialControl);

        tObjectiveFunction[2] = aStateMng.evaluateObjective(*mTrialControl);
        mInitialTrialStepDotCurrentGradient = Plato::dot(aStateMng.getTrialStep(), aStateMng.getCurrentGradient());
        const ScalarType tAlpha = 1e-4;
        ScalarType tTargetObjectiveValue = tObjectiveFunction[0]
                - (tAlpha * mStepValues[1] * mInitialTrialStepDotCurrentGradient);

        mNumIterationsDone = 1;
        while(tObjectiveFunction[2] > tTargetObjectiveValue)
        {
            mStepValues[0] = mStepValues[1];
            ScalarType tStep = this->interpolate(tObjectiveFunction, mStepValues);
            mStepValues[1] = tStep;

            Plato::update(static_cast<ScalarType>(1),
                          aStateMng.getCurrentControl(),
                          static_cast<ScalarType>(0),
                          *mTrialControl);
            Plato::update(mStepValues[1], aStateMng.getTrialStep(), static_cast<ScalarType>(1), *mTrialControl);

            tObjectiveFunction[1] = tObjectiveFunction[2];
            tObjectiveFunction[2] = aStateMng.evaluateObjective(*mTrialControl);

            mNumIterationsDone++;
            if(mNumIterationsDone >= mMaxNumIterations)
            {
                break;
            }
            tTargetObjectiveValue = tObjectiveFunction[0]
                    - (tAlpha * mStepValues[1] * mInitialTrialStepDotCurrentGradient);
        }

        aStateMng.setCurrentObjectiveValue(tObjectiveFunction[2]);
        aStateMng.setCurrentControl(*mTrialControl);
    }

private:
    ScalarType interpolate(const std::vector<ScalarType> & aObjectiveFunction, const std::vector<ScalarType> & aStepValues)
    {
        ScalarType tStepLowerBound = aStepValues[1] * mStepLowerBound;
        ScalarType tStepUpperBound = aStepValues[1] * mStepUpperBound;
        ScalarType tDenominator = static_cast<ScalarType>(2) * aStepValues[1]
                * (aObjectiveFunction[2] - aObjectiveFunction[0] - mInitialTrialStepDotCurrentGradient);

        ScalarType tStep = -mInitialTrialStepDotCurrentGradient / tDenominator;
        if(tStep < tStepLowerBound)
        {
            tStep = tStepLowerBound;
        }
        if(tStep > tStepUpperBound)
        {
            tStep = tStepUpperBound;
        }

        return (tStep);
    }

private:
    OrdinalType mMaxNumIterations;
    OrdinalType mNumIterationsDone;

    ScalarType mStepLowerBound;
    ScalarType mStepUpperBound;
    ScalarType mContractionFactor;
    ScalarType mInitialTrialStepDotCurrentGradient;

    // mStepValues[0] = old trial value
    // mStepValues[1] = new trial value
    std::vector<ScalarType> mStepValues;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTrialControl;

private:
    QuadraticLineSearch(const Plato::QuadraticLineSearch<ScalarType, OrdinalType> & aRhs);
    Plato::QuadraticLineSearch<ScalarType, OrdinalType> & operator=(const Plato::QuadraticLineSearch<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_QUADRATICLINESEARCH_HPP_ */
