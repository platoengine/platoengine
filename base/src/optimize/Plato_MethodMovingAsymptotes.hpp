/*
 * Plato_MethodMovingAsymptotes.hpp
 *
 *  Created on: Nov 4, 2017
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

#ifndef PLATO_METHODMOVINGASYMPTOTES_HPP_
#define PLATO_METHODMOVINGASYMPTOTES_HPP_

#include <memory>

#include "Plato_BoundsBase.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_DualSolverInterface.hpp"
#include "Plato_ConservativeConvexSeparableAppxDataMng.hpp"
#include "Plato_ConservativeConvexSeparableAppxStageMng.hpp"
#include "Plato_ConservativeConvexSeparableApproximation.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class MethodMovingAsymptotes : public Plato::ConservativeConvexSeparableApproximation<ScalarType, OrdinalType>
{
public:
    explicit MethodMovingAsymptotes(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory) :
            mTrialDual(aDataFactory->dual().create()),
            mActiveSet(aDataFactory->control().create()),
            mInactiveSet(aDataFactory->control().create()),
            mTrialControl(aDataFactory->control().create()),
            mConstraintValues(aDataFactory->dual().create()),
            mDualSolver(std::make_shared<Plato::DualSolverInterface<ScalarType, OrdinalType>>(aDataFactory.operator*()))
    {
    }
    virtual ~MethodMovingAsymptotes()
    {
    }

    void solve(Plato::ConservativeConvexSeparableAppxStageMng<ScalarType, OrdinalType> & aStageMng,
               Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        // NOTE: REMBER THAT THE GLOBALIZATION FACTORS FOR BOTH OBJECTIVE AND CONSTRAINTS ARE SET TO ZERO IF USING
        //       THE MMA APPROACH. THE MMA METHOD IS DETECTED AT THE OUTTER LOOP LEVEL (NOT THE SUBPROBLEM LEVEL)
        //       AND THE VALUES SET INSIDE THE INITIALZE FUNCTION IN THE ALGORITHM CLASS. DEFAULT VALUES ARE ONLY
        //       SET FOR THE GCMMA CASE.
        mDualSolver->update(aDataMng);
        mDualSolver->updateObjectiveCoefficients(aDataMng);
        mDualSolver->updateConstraintCoefficients(aDataMng);
        mDualSolver->solve(mTrialDual.operator*(), mTrialControl.operator*());

        const Plato::MultiVector<ScalarType, OrdinalType> & tLowerBounds = aDataMng.getControlLowerBounds();
        const Plato::MultiVector<ScalarType, OrdinalType> & tUpperBounds = aDataMng.getControlUpperBounds();
        aDataMng.bounds().project(tLowerBounds, tUpperBounds, mTrialControl.operator*());
        aDataMng.bounds().computeActiveAndInactiveSets(mTrialControl.operator*(),
                                                       tLowerBounds,
                                                       tUpperBounds,
                                                       mActiveSet.operator*(),
                                                       mInactiveSet.operator*());
        aDataMng.setActiveSet(mActiveSet.operator*());
        aDataMng.setInactiveSet(mInactiveSet.operator*());

        ScalarType tObjectiveFunctionValue = aStageMng.evaluateObjective(mTrialControl.operator*());
        aDataMng.setCurrentObjectiveFunctionValue(tObjectiveFunctionValue);
        aStageMng.evaluateConstraints(mTrialControl.operator*(), mConstraintValues.operator*());
        aDataMng.setCurrentConstraintValues(mConstraintValues.operator*());
        aDataMng.setCurrentControl(mTrialControl.operator*());
        aDataMng.setDual(mTrialDual.operator*());
    }
    void initializeAuxiliaryVariables(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        mDualSolver->initializeAuxiliaryVariables(aDataMng);
    }

private:
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTrialDual;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mActiveSet;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mInactiveSet;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTrialControl;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mConstraintValues;

    std::shared_ptr<Plato::DualProblemSolver<ScalarType, OrdinalType>> mDualSolver;

private:
    MethodMovingAsymptotes(const Plato::MethodMovingAsymptotes<ScalarType, OrdinalType> & aRhs);
    Plato::MethodMovingAsymptotes<ScalarType, OrdinalType> & operator=(const Plato::MethodMovingAsymptotes<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_METHODMOVINGASYMPTOTES_HPP_ */
