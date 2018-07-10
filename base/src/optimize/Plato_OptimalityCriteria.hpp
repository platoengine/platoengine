/*
 * Plato_OptimalityCriteria.hpp
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
//
// *************************************************************************
//@HEADER
*/

#ifndef PLATO_OPTIMALITYCRITERIA_HPP_
#define PLATO_OPTIMALITYCRITERIA_HPP_

#include <sstream>
#include <iostream>

#include "Plato_Vector.hpp"
#include "Plato_HostBounds.hpp"
#include "Plato_ErrorChecks.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DeviceBounds.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_OptimalityCriteriaDataMng.hpp"
#include "Plato_OptimalityCriteriaSubProblem.hpp"
#include "Plato_OptimalityCriteriaStageMngBase.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class OptimalityCriteria
{
public:
    OptimalityCriteria(const std::shared_ptr<Plato::OptimalityCriteriaDataMng<ScalarType, OrdinalType>> & aDataMng,
                       const std::shared_ptr<Plato::OptimalityCriteriaStageMngBase<ScalarType, OrdinalType>> & aStageMng,
                       const std::shared_ptr<Plato::OptimalityCriteriaSubProblem<ScalarType, OrdinalType>> & aSubProblem) :
            mPrintDiagnostics(false),
            mOutputStream(),
            mMaxNumIterations(50),
            mNumIterationsDone(0),
            mFeasibilityTolerance(1e-5),
            mControlStagnationTolerance(1e-2),
            mObjectiveGradientTolerance(1e-8),
            mObjectiveStagnationTolerance(1e-5),
            mBounds(nullptr),
            mDataMng(aDataMng),
            mStageMng(aStageMng),
            mSubProblem(aSubProblem)
    {
        this->initialize();
    }
    ~OptimalityCriteria()
    {
    }

    bool printDiagnostics() const
    {
        return (mPrintDiagnostics);
    }

    void enableDiagnostics()
    {
        mPrintDiagnostics = true;
    }

    OrdinalType getNumIterationsDone() const
    {
        return (mNumIterationsDone);
    }
    OrdinalType getMaxNumIterations() const
    {
        return (mMaxNumIterations);
    }
    ScalarType getControlStagnationTolerance() const
    {
        return (mControlStagnationTolerance);
    }
    ScalarType getFeasibilityTolerance() const
    {
        return (mFeasibilityTolerance);
    }
    ScalarType getObjectiveGradientTolerance() const
    {
        return (mObjectiveGradientTolerance);
    }
    ScalarType getObjectiveStagnationTolerance() const
    {
        return (mObjectiveStagnationTolerance);
    }

    void setMaxNumIterations(const OrdinalType & aInput)
    {
        mMaxNumIterations = aInput;
    }
    void setControlStagnationTolerance(const ScalarType & aInput)
    {
        mControlStagnationTolerance = aInput;
    }
    void setFeasibilityTolerance(const ScalarType & aInput)
    {
        mFeasibilityTolerance = aInput;
    }
    void setObjectiveGradientTolerance(const ScalarType & aInput)
    {
        mObjectiveGradientTolerance = aInput;
    }
    void setObjectiveStagnationTolerance(const ScalarType & aInput)
    {
        mObjectiveStagnationTolerance = aInput;
    }

    void gatherOuputStream(std::ostringstream & aOutput)
    {
        aOutput << mOutputStream.str().c_str();
    }

    void solve()
    {
        this->checkInitialGuess();

        mNumIterationsDone = 0;
        while(1)
        {
            mStageMng->update(*mDataMng);

            mDataMng->computeMaxInequalityValue();
            mDataMng->computeNormObjectiveGradient();
            mDataMng->computeControlStagnationMeasure();
            mDataMng->computeObjectiveStagnationMeasure();

            this->printCurrentProgress();
            if(this->stoppingCriteriaSatisfied() == true)
            {
                break;
            }

            this->storeCurrentStageData();
            mSubProblem->solve(*mDataMng, *mStageMng);

            mNumIterationsDone++;
        }
    }

    bool stoppingCriteriaSatisfied()
    {
        bool tStoppingCriterionSatisfied = false;
        ScalarType tMaxInequalityValue = mDataMng->getMaxInequalityValue();
        ScalarType tNormObjectiveGradient = mDataMng->getNormObjectiveGradient();
        ScalarType tControlStagnationMeasure = mDataMng->getControlStagnationMeasure();
        ScalarType tObjectiveStagnationMeasure = mDataMng->getObjectiveStagnationMeasure();

        if(this->getNumIterationsDone() >= this->getMaxNumIterations())
        {
            tStoppingCriterionSatisfied = true;
        }
        else if(tControlStagnationMeasure < this->getControlStagnationTolerance())
        {
            tStoppingCriterionSatisfied = true;
        }
        else if(tObjectiveStagnationMeasure < this->getObjectiveStagnationTolerance())
        {
            tStoppingCriterionSatisfied = true;
        }
        else if(tNormObjectiveGradient < this->getObjectiveGradientTolerance()
                && tMaxInequalityValue < this->getFeasibilityTolerance())
        {
            tStoppingCriterionSatisfied = true;
        }

        return (tStoppingCriterionSatisfied);
    }

    void printCurrentProgress()
    {
        if(this->printDiagnostics() == false)
        {
            return;
        }

        OrdinalType tCurrentNumIterationsDone = this->getNumIterationsDone();

        if(tCurrentNumIterationsDone < static_cast<OrdinalType>(2))
        {
            mOutputStream << " Itr" << std::setw(14) << "   F(x)  " << std::setw(16) << " ||F'(x)||" << std::setw(16)
                    << "   Max(H) " << "\n" << std::flush;
            mOutputStream << "-----" << std::setw(14) << "----------" << std::setw(16) << "-----------" << std::setw(16)
                    << "----------" << "\n" << std::flush;
        }

        ScalarType tObjectiveValue = mDataMng->getCurrentObjectiveValue();
        ScalarType tMaxInequalityValue = mDataMng->getMaxInequalityValue();
        ScalarType tNormObjectiveGradient = mDataMng->getNormObjectiveGradient();
        mOutputStream << std::setw(3) << tCurrentNumIterationsDone << std::setprecision(4) << std::fixed
                << std::scientific << std::setw(16) << tObjectiveValue << std::setw(16) << tNormObjectiveGradient
                << std::setw(16) << tMaxInequalityValue << "\n";
    }
    void storeCurrentStageData()
    {
        const ScalarType tObjectiveValue = mDataMng->getCurrentObjectiveValue();
        mDataMng->setPreviousObjectiveValue(tObjectiveValue);

        const OrdinalType tNumVectors = mDataMng->getNumControlVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tControl = mDataMng->getCurrentControl(tVectorIndex);
            mDataMng->setPreviousControl(tVectorIndex, tControl);
        }

        const OrdinalType tNumConstraints = mDataMng->getNumConstraints();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            ScalarType tMyCurrentConstraintValue = mDataMng->getCurrentConstraintValues(tConstraintIndex);
            mDataMng->setPreviousConstraintValue(tConstraintIndex, tMyCurrentConstraintValue);
        }
    }

private:
    void initialize()
    {
        // Check Bounds
        try
        {
            const Plato::MultiVector<ScalarType, OrdinalType> & tLowerBounds = mDataMng->getControlLowerBounds();
            const Plato::MultiVector<ScalarType, OrdinalType> & tUpperBounds = mDataMng->getControlUpperBounds();
            Plato::error::checkBounds(tLowerBounds, tUpperBounds);
        }
        catch(const std::invalid_argument & tErrorMsg)
        {
            std::cout << tErrorMsg.what() << std::flush;
            std::abort();
        }

        Plato::MemorySpace::type_t tMemorySpace = mDataMng->getMemorySpace();
        switch(tMemorySpace)
        {
            case Plato::MemorySpace::DEVICE:
            {
                mBounds = std::make_shared<Plato::DeviceBounds<ScalarType, OrdinalType>>();
                break;
            }
            default:
            case Plato::MemorySpace::HOST:
            {
                mBounds = std::make_shared<Plato::HostBounds<ScalarType, OrdinalType>>();
                break;
            }
        }
    }
    void checkInitialGuess()
    {
        try
        {
            bool tIsInitialGuessSet = mDataMng->isInitialGuessSet();
            Plato::error::checkInitialGuessIsSet(tIsInitialGuessSet);
        }
        catch(const std::invalid_argument & tErrorMsg)
        {
            std::cout << tErrorMsg.what() << std::flush;
            std::abort();
        }

        try
        {
            const Plato::MultiVector<ScalarType, OrdinalType> & tControl = mDataMng->getCurrentControl();
            const Plato::MultiVector<ScalarType, OrdinalType> & tLowerBounds = mDataMng->getControlLowerBounds();
            const Plato::MultiVector<ScalarType, OrdinalType> & tUpperBounds = mDataMng->getControlUpperBounds();
            Plato::error::checkInitialGuess(tControl, tLowerBounds, tUpperBounds);
        }
        catch(const std::invalid_argument & tErrorMsg)
        {
            std::cout << tErrorMsg.what() << std::flush;
            std::abort();
        }

        const Plato::MultiVector<ScalarType, OrdinalType> & tControl = mDataMng->getCurrentControl();
        const Plato::MultiVector<ScalarType, OrdinalType> & tLowerBounds = mDataMng->getControlLowerBounds();
        const Plato::MultiVector<ScalarType, OrdinalType> & tUpperBounds = mDataMng->getControlUpperBounds();
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tWorkMultiVector = tControl.create();
        Plato::update(static_cast<ScalarType>(1), tControl, static_cast<ScalarType>(0), *tWorkMultiVector);
        mBounds->project(tLowerBounds, tUpperBounds, *tWorkMultiVector);
        mDataMng->setCurrentControl(*tWorkMultiVector);
    }

private:
    bool mPrintDiagnostics;
    std::ostringstream mOutputStream;

    OrdinalType mMaxNumIterations;
    OrdinalType mNumIterationsDone;

    ScalarType mFeasibilityTolerance;
    ScalarType mControlStagnationTolerance;
    ScalarType mObjectiveGradientTolerance;
    ScalarType mObjectiveStagnationTolerance;

    std::shared_ptr<Plato::BoundsBase<ScalarType, OrdinalType>> mBounds;
    std::shared_ptr<Plato::OptimalityCriteriaDataMng<ScalarType, OrdinalType>> mDataMng;
    std::shared_ptr<Plato::OptimalityCriteriaStageMngBase<ScalarType, OrdinalType>> mStageMng;
    std::shared_ptr<Plato::OptimalityCriteriaSubProblem<ScalarType, OrdinalType>> mSubProblem;

private:
    OptimalityCriteria(const Plato::OptimalityCriteria<ScalarType, OrdinalType>&);
    Plato::OptimalityCriteria<ScalarType, OrdinalType> & operator=(const Plato::OptimalityCriteria<ScalarType, OrdinalType>&);
};
// class OptimalityCriteria

} // namespace Plato

#endif /* PLATO_OPTIMALITYCRITERIA_HPP_ */
