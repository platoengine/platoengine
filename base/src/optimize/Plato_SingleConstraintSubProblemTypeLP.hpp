/*
 * Plato_SingleConstraintSubProblemTypeLP.hpp
 *
 *  Created on: Oct 17, 2017
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
// *************************************************************************
//@HEADER
*/

#ifndef PLATO_SINGLECONSTRAINTSUBPROBLEMTYPELP_HPP_
#define PLATO_SINGLECONSTRAINTSUBPROBLEMTYPELP_HPP_

#include <cmath>
#include <memory>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_OptimalityCriteriaDataMng.hpp"
#include "Plato_OptimalityCriteriaSubProblem.hpp"
#include "Plato_OptimalityCriteriaStageMngBase.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class SingleConstraintSubProblemTypeLP : public Plato::OptimalityCriteriaSubProblem<ScalarType,OrdinalType>
{
public:
    explicit SingleConstraintSubProblemTypeLP(Plato::OptimalityCriteriaDataMng<ScalarType, OrdinalType> & aDataMng) :
            mActiveSet(aDataMng.getCurrentControl().create()),
            mPassiveSet(aDataMng.getCurrentControl().create()),
            mWorkControl(aDataMng.getCurrentControl().create())
    {
        this->initialize();
    }
    virtual ~SingleConstraintSubProblemTypeLP()
    {
    }

    void solve(Plato::OptimalityCriteriaDataMng<ScalarType, OrdinalType> & aDataMng,
               Plato::OptimalityCriteriaStageMngBase<ScalarType, OrdinalType> & aStageMng)
    {
        assert(aDataMng.getNumConstraints() == static_cast<OrdinalType>(1));

        const OrdinalType tConstraintIndex = 0;
        ScalarType tDual = this->computeDual(aDataMng);
        aDataMng.setCurrentDual(tConstraintIndex, tDual);

        this->updateControl(tDual, aDataMng);
    }

private:
    void initialize()
    {
        const OrdinalType tVectorIndex = 0;
        mActiveSet->operator [](tVectorIndex).fill(1);
        mPassiveSet->operator [](tVectorIndex).fill(0);
    }
    ScalarType computeDual(Plato::OptimalityCriteriaDataMng<ScalarType, OrdinalType> & aDataMng)
    {

        // Compute linearized constraint, c_0 = g(\bm{x}) + (\frac{\partial{g}}{\partial{x}})^{T}\bm{x}
        const OrdinalType tVectorIndex = 0;
        const Plato::Vector<ScalarType, OrdinalType> & tInequalityGradient = aDataMng.getInequalityGradient(tVectorIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tPreviousControl = aDataMng.getPreviousControl(tVectorIndex);
        ScalarType tLinearizedConstraint = tInequalityGradient.dot(tPreviousControl);
        const OrdinalType tConstraintIndex = 0;
        tLinearizedConstraint += aDataMng.getCurrentConstraintValues(tConstraintIndex);

        /* Compute c_0^{\ast} = c_0 + \sum_{i\in{I}_p}\frac{\partial{g}}{\partial{y}_i}\frac{1}{x_i}, where I_p is the passive set
           and \frac{\partial{g}}{\partial{y}_i} = -x_i^2\frac{\partial{g}}{\partial{x}_i}\ \forall\ i=1\,dots,length(\bm{x})*/
        const Plato::Vector<ScalarType, OrdinalType> & tPassiveSet = mPassiveSet->operator [](tVectorIndex);
        mWorkControl->operator [](tVectorIndex).update(1., tInequalityGradient, 0.);
        mWorkControl->operator [](tVectorIndex).entryWiseProduct(tPassiveSet);
        ScalarType tLinearizedConstraintStar = -(mWorkControl->operator [](tVectorIndex).dot(tPreviousControl));
        tLinearizedConstraintStar += tLinearizedConstraint;

        // Compute Active Inequality Constraint Gradient
        const Plato::Vector<ScalarType, OrdinalType> & tActiveSet = mActiveSet->operator [](tVectorIndex);
        mWorkControl->operator [](tVectorIndex).update(1., tInequalityGradient, 0.);
        mWorkControl->operator [](tVectorIndex).entryWiseProduct(tActiveSet);

        /* Compute Dual, \lambda=\left[\frac{1}{c_0^{ast}}\sum_{i\in{I}_a}\left(-\frac{\partial{f}}{\partial{x_i}}
           \frac{\partial{g}}{\partial{y_i}}\right)^{1/2}\right]^2, where y_i=1/x_i */
        ScalarType tSum = 0;
        OrdinalType tNumControls = tPreviousControl.size();
        const Plato::Vector<ScalarType, OrdinalType> & tActiveInqGradient = mWorkControl->operator [](tVectorIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tObjectiveGradient = aDataMng.getObjectiveGradient(tVectorIndex);
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            ScalarType tValue = std::pow(tPreviousControl[tIndex], static_cast<ScalarType>(2)) * tObjectiveGradient[tIndex] * tActiveInqGradient[tIndex];
            tValue = std::sqrt(tValue);
            tSum += tValue;
        }
        ScalarType tDual = (static_cast<ScalarType>(1) / tLinearizedConstraintStar) * tSum;
        tDual = std::pow(tDual, static_cast<ScalarType>(2));

        return (tDual);
    }
    void updateControl(const ScalarType & aDual, Plato::OptimalityCriteriaDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        OrdinalType tNumControlVectors = aDataMng.getNumControlVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tPreviousControl = aDataMng.getPreviousControl(tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tControlLowerBound = aDataMng.getControlLowerBounds(tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tControlUpperBound = aDataMng.getControlUpperBounds(tVectorIndex);

            const Plato::Vector<ScalarType, OrdinalType> & tObjectiveGradient = aDataMng.getObjectiveGradient(tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tInequalityGradient = aDataMng.getInequalityGradient(tVectorIndex);

            mActiveSet->operator [](tVectorIndex).fill(1);
            mPassiveSet->operator [](tVectorIndex).fill(0);

            ScalarType tDampingPower = 0.5;
            OrdinalType tNumControls = tPreviousControl.size();
            for(OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                ScalarType tTrialControl = (-aDual * tInequalityGradient[tControlIndex])
                        / (tObjectiveGradient[tControlIndex]);
                tTrialControl = -std::pow(tPreviousControl[tControlIndex], static_cast<ScalarType>(2)) * tTrialControl;
                tTrialControl = std::pow(tTrialControl, tDampingPower);
                bool tOutsideBounds = (tControlLowerBound[tControlIndex] >= tTrialControl) || (tControlUpperBound[tControlIndex] <= tTrialControl);
                if(tOutsideBounds == true)
                {
                    mActiveSet->operator ()(tVectorIndex, tControlIndex) = 0;
                    mPassiveSet->operator ()(tVectorIndex, tControlIndex) = 1;
                }
                tTrialControl = tControlLowerBound[tControlIndex] >= tTrialControl ? tControlLowerBound[tControlIndex] : tTrialControl;
                tTrialControl = tControlUpperBound[tControlIndex] <= tTrialControl ? tControlUpperBound[tControlIndex] : tTrialControl;
                mWorkControl->operator ()(tVectorIndex, tControlIndex) = tTrialControl;
            }
            aDataMng.setCurrentControl(tVectorIndex, mWorkControl->operator [](tVectorIndex));
        }
    }

private:
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mActiveSet;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPassiveSet;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mWorkControl;

private:
    SingleConstraintSubProblemTypeLP(const Plato::SingleConstraintSubProblemTypeLP<ScalarType, OrdinalType>&);
    Plato::SingleConstraintSubProblemTypeLP<ScalarType, OrdinalType> & operator=(const Plato::SingleConstraintSubProblemTypeLP<ScalarType, OrdinalType>&);
};

}

#endif /* PLATO_SINGLECONSTRAINTSUBPROBLEMTYPELP_HPP_ */
