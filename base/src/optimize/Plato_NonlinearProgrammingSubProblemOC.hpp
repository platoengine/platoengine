/*
 * Plato_NonlinearProgrammingSubProblemOC.hpp
 *
 *  Created on: Oct 17, 2017
 */

#ifndef PLATO_NONLINEARPROGRAMMINGSUBPROBLEMOC_HPP_
#define PLATO_NONLINEARPROGRAMMINGSUBPROBLEMOC_HPP_

#include <cmath>
#include <memory>

#include "Plato_Vector.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_OptimalityCriteriaDataMng.hpp"
#include "Plato_HostOptimalityCriteriaUpdate.hpp"
#include "Plato_OptimalityCriteriaSubProblem.hpp"
#include "Plato_DeviceOptimalityCriteriaUpdate.hpp"
#include "Plato_OptimalityCriteriaStageMngBase.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class NonlinearProgrammingSubProblemOC : public Plato::OptimalityCriteriaSubProblem<ScalarType,OrdinalType>
{
public:
    explicit NonlinearProgrammingSubProblemOC(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory) :
            mMoveLimit(1.0),
            mScaleFactor(0.01),
            mDampingPower(0.5),
            mDualLowerBound(0),
            mDualUpperBound(1e7),
            mBisectionTolerance(1e-6),
            mInequalityGradientDotDeltaControl(0),
            mWorkControl(aDataFactory->control().create()),
            mUpdateFunctor(nullptr)
    {
        this->initialize(*aDataFactory);
    }
    virtual ~NonlinearProgrammingSubProblemOC()
    {
    }

    ScalarType getMoveLimit() const
    {
        return (mMoveLimit);
    }
    ScalarType getScaleFactor() const
    {
        return (mScaleFactor);
    }
    ScalarType getDampingPower() const
    {
        return (mDampingPower);
    }
    ScalarType getDualLowerBound() const
    {
        return (mDualLowerBound);
    }
    ScalarType getDualUpperBound() const
    {
        return (mDualUpperBound);
    }
    ScalarType getBisectionTolerance() const
    {
        return (mBisectionTolerance);
    }

    void setMoveLimit(const ScalarType & aInput)
    {
        mMoveLimit = aInput;
        mUpdateFunctor->reset(mMoveLimit, mScaleFactor, mDampingPower);
    }
    void setScaleFactor(const ScalarType & aInput)
    {
        mScaleFactor = aInput;
        mUpdateFunctor->reset(mMoveLimit, mScaleFactor, mDampingPower);
    }
    void setDampingPower(const ScalarType & aInput)
    {
        mDampingPower = aInput;
        mUpdateFunctor->reset(mMoveLimit, mScaleFactor, mDampingPower);
    }
    void setDualLowerBound(const ScalarType & aInput)
    {
        mDualLowerBound = aInput;
    }
    void setDualUpperBound(const ScalarType & aInput)
    {
        mDualUpperBound = aInput;
    }
    void setBisectionTolerance(const ScalarType & aInput)
    {
        mBisectionTolerance = aInput;
    }

    void solve(Plato::OptimalityCriteriaDataMng<ScalarType, OrdinalType> & aDataMng,
               Plato::OptimalityCriteriaStageMngBase<ScalarType, OrdinalType> & aStageMng)
    {
        ScalarType tDualLowerBound = this->getDualLowerBound();
        ScalarType tDualUpperBound = this->getDualUpperBound();
        ScalarType tBisectionTolerance = this->getBisectionTolerance();

        const OrdinalType tNumConstraints = aDataMng.getNumConstraints();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            ScalarType tDualMisfit = tDualUpperBound - tDualLowerBound;
            ScalarType tTrialDual = std::numeric_limits<ScalarType>::max();
            while(tDualMisfit >= tBisectionTolerance)
            {
                tTrialDual = static_cast<ScalarType>(0.5) * (tDualUpperBound + tDualLowerBound);
                this->updateControl(tTrialDual, aDataMng);

                const ScalarType tMyCurrentConstraintValue = aDataMng.getCurrentConstraintValues(tConstraintIndex);
                ScalarType tFirstOrderTaylorApproximation = tMyCurrentConstraintValue + mInequalityGradientDotDeltaControl;
                if(tFirstOrderTaylorApproximation > static_cast<ScalarType>(0))
                {
                    tDualLowerBound = tTrialDual;
                }
                else
                {
                    tDualUpperBound = tTrialDual;
                }
                tDualMisfit = tDualUpperBound - tDualLowerBound;
            }
            aDataMng.setCurrentDual(tConstraintIndex, tTrialDual);
        }
    }

private:
    void initialize(const Plato::DataFactory<ScalarType, OrdinalType> & aFactory)
    {
        Plato::MemorySpace::type_t tMemorySpace = aFactory.getMemorySpace();
        switch(tMemorySpace)
        {
            case Plato::MemorySpace::DEVICE:
            {
                mUpdateFunctor =
                        std::make_shared<Plato::DeviceOptimalityCriteriaUpdate<ScalarType, OrdinalType>>(mMoveLimit, mScaleFactor, mDampingPower);
                break;
            }
            default:
            case Plato::MemorySpace::HOST:
            {
                mUpdateFunctor =
                        std::make_shared<Plato::HostOptimalityCriteriaUpdate<ScalarType, OrdinalType>>(mMoveLimit, mScaleFactor, mDampingPower);
                break;
            }
        }
    }

    void updateControl(const ScalarType & aTrialDual, Plato::OptimalityCriteriaDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        mInequalityGradientDotDeltaControl = 0;

        const OrdinalType tNumControlVectors = aDataMng.getNumControlVectors();
        for(OrdinalType tMyVectorIndex = 0; tMyVectorIndex < tNumControlVectors; tMyVectorIndex++)
        {
            Plato::Vector<ScalarType, OrdinalType> & tMyTrialControls = mWorkControl->operator[](tMyVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyPreviousControls = aDataMng.getPreviousControl(tMyVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyControlLowerBounds = aDataMng.getControlLowerBounds(tMyVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyControlUpperBounds = aDataMng.getControlUpperBounds(tMyVectorIndex);

            const Plato::Vector<ScalarType, OrdinalType> & tMyObjectiveGradient = aDataMng.getObjectiveGradient(tMyVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyInequalityGradient = aDataMng.getInequalityGradient(tMyVectorIndex);

            mUpdateFunctor->update(aTrialDual,
                                   tMyControlLowerBounds,
                                   tMyControlUpperBounds,
                                   tMyPreviousControls,
                                   tMyObjectiveGradient,
                                   tMyInequalityGradient,
                                   tMyTrialControls);

            aDataMng.setCurrentControl(tMyVectorIndex, tMyTrialControls);
            /*Compute Delta Control*/
            tMyTrialControls.update(static_cast<ScalarType>(-1), tMyPreviousControls, static_cast<ScalarType>(1));
            mInequalityGradientDotDeltaControl += tMyInequalityGradient.dot(tMyTrialControls);
        }
    }

private:
    ScalarType mMoveLimit;
    ScalarType mScaleFactor;
    ScalarType mDampingPower;
    ScalarType mDualLowerBound;
    ScalarType mDualUpperBound;
    ScalarType mBisectionTolerance;
    ScalarType mInequalityGradientDotDeltaControl;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mWorkControl;
    std::shared_ptr<Plato::OptimalityCriteriaUpdate<ScalarType, OrdinalType>> mUpdateFunctor;

private:
    NonlinearProgrammingSubProblemOC(const Plato::NonlinearProgrammingSubProblemOC<ScalarType, OrdinalType>&);
    Plato::NonlinearProgrammingSubProblemOC<ScalarType, OrdinalType> & operator=(const Plato::NonlinearProgrammingSubProblemOC<ScalarType, OrdinalType>&);
};
// class NonlinearProgrammingSubProblemOC

} // namespace Plato

#endif /* PLATO_NONLINEARPROGRAMMINGSUBPROBLEMOC_HPP_ */
