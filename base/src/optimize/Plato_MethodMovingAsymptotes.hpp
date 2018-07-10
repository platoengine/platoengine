/*
 * Plato_MethodMovingAsymptotes.hpp
 *
 *  Created on: Nov 4, 2017
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
