/*
 * Plato_Diagnostics.hpp
 *
 *  Created on: Jan 9, 2018
 */

#ifndef PLATO_DIAGNOSTICS_HPP_
#define PLATO_DIAGNOSTICS_HPP_

#include <cmath>
#include <vector>
#include <limits>
#include <memory>
#include <sstream>
#include <cassert>
#include <iomanip>
#include <cstdlib>
#include <iostream>

#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class Diagnostics
{
public:
    Diagnostics() :
        mDidHessianTestPassed(false),
        mDidGradientTestPassed(false),
        mFinalSuperscript(8),
        mInitialSuperscript(1),
        mHessianTestBound(1e-6),
        mGradientTestBound(1e-6),
        mRandomNumLowerBound(0.05),
        mRandomNumUpperBound(0.1)
    {
    }
    ~Diagnostics()
    {
    }

    bool didHessianTestPassed() const
    {
        return (mDidHessianTestPassed);
    }

    bool didGradientTestPassed() const
    {
        return (mDidGradientTestPassed);
    }

    void setHessianTestBound(const ScalarType & aInput)
    {
        mHessianTestBound = aInput;
    }

    void setGradientTestBound(const ScalarType & aInput)
    {
        mGradientTestBound = aInput;
    }

    void setRandomNumberLowerBound(const ScalarType & aInput)
    {
        mRandomNumLowerBound = aInput;
    }

    void setRandomNumberUpperBound(const ScalarType & aInput)
    {
        mRandomNumUpperBound = aInput;
    }

    void setFinalSuperscript(const int & aInput)
    {
        mFinalSuperscript = aInput;
    }

    void setInitialSuperscript(const int & aInput)
    {
        mInitialSuperscript = aInput;
    }

    void checkCriterionGradient(Plato::Criterion<ScalarType, OrdinalType> & aCriterion,
                                Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                std::ostringstream & aOutputMsg,
                                bool aUseInitialGuess = false)
    {
        mDidGradientTestPassed = false;
        aOutputMsg << std::right << std::setw(18) << "\nStep Size" << std::setw(20) << "Grad'*Step" << std::setw(18) << "FD Approx"
                   << std::setw(20) << "abs(Error)" << "\n";

        assert(aControl.getNumVectors() > static_cast<OrdinalType>(0));
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tStep = aControl.create();
        assert(tStep.get() != nullptr);
        // NOTE: Think how to syncronize random number generator if working with owned and shared data
        unsigned int tRANDOM_SEED = 1;
        std::srand(tRANDOM_SEED);
        this->random(mRandomNumLowerBound, mRandomNumUpperBound, *tStep);
        if(aUseInitialGuess == false)
        {
            this->random(mRandomNumLowerBound, mRandomNumUpperBound, aControl);
        }

        aCriterion.value(aControl);
        aCriterion.cacheData();
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tGradient = aControl.create();
        assert(tGradient.get() != nullptr);
        aCriterion.gradient(aControl, *tGradient);

        std::vector<ScalarType> tApproximationErrors;
        const ScalarType tGradientDotStep = Plato::dot(*tGradient, *tStep);
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tWork = aControl.create();
        assert(tWork.get() != nullptr);
        for(int tIndex = mInitialSuperscript; tIndex <= mFinalSuperscript; tIndex++)
        {
            // Compute \hat{x} = x + \epsilon\Delta{x}, where x denotes the control vector and \Delta{x} denotes the step.
            ScalarType tEpsilon = static_cast<ScalarType>(1) /
                    std::pow(static_cast<ScalarType>(10), tIndex);
            Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *tWork);
            Plato::update(tEpsilon, *tStep, static_cast<ScalarType>(1), *tWork);
            ScalarType tObjectiveValueAtPlusEpsilon = aCriterion.value(*tWork);

            // Compute \hat{x} = x - \epsilon\Delta{x}, where x denotes the control vector and \Delta{x} denotes the step.
            Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *tWork);
            ScalarType tMultiplier = static_cast<ScalarType>(-1) * tEpsilon;
            Plato::update(tMultiplier, *tStep, static_cast<ScalarType>(1), *tWork);
            ScalarType tObjectiveValueAtMinusEpsilon = aCriterion.value(*tWork);

            // Compute \hat{x} = x + 2\epsilon\Delta{x}, where x denotes the control vector and \Delta{x} denotes the step.
            Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *tWork);
            tMultiplier = static_cast<ScalarType>(2) * tEpsilon;
            Plato::update(tMultiplier, *tStep, static_cast<ScalarType>(1), *tWork);
            ScalarType tObjectiveValueAtPlusTwoEpsilon = aCriterion.value(*tWork);

            // Compute \hat{x} = x - 2\epsilon\Delta{x}, where x denotes the control vector and \Delta{x} denotes the step.
            Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *tWork);
            tMultiplier = static_cast<ScalarType>(-2) * tEpsilon;
            Plato::update(tMultiplier, *tStep, static_cast<ScalarType>(1), *tWork);
            ScalarType tObjectiveValueAtMinusTwoEpsilon = aCriterion.value(*tWork);

            // Compute objective value approximation via a five point stencil finite difference procedure
            ScalarType tObjectiveAppx = (-tObjectiveValueAtPlusTwoEpsilon
                    + static_cast<ScalarType>(8) * tObjectiveValueAtPlusEpsilon
                    - static_cast<ScalarType>(8) * tObjectiveValueAtMinusEpsilon
                    + tObjectiveValueAtMinusTwoEpsilon) / (static_cast<ScalarType>(12) * tEpsilon);

            ScalarType tAppxError = std::abs(tObjectiveAppx - tGradientDotStep);
            tApproximationErrors.push_back(tAppxError);
            aOutputMsg << std::right << std::scientific << std::setprecision(8) << std::setw(14) << tEpsilon << std::setw(19)
            << tGradientDotStep << std::setw(19) << tObjectiveAppx << std::setw(19) << tAppxError << "\n";
        }

        ScalarType tMinError = *std::min_element(tApproximationErrors.begin(), tApproximationErrors.end());
        mDidGradientTestPassed = tMinError < mGradientTestBound ? true : false;
    }

    void checkCriterionHessian(Plato::Criterion<ScalarType, OrdinalType> & aCriterion,
                               Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                               std::ostringstream & aOutputMsg,
                               bool aUseInitialGuess = false)
    {
        mDidHessianTestPassed = false;
        aOutputMsg << std::right << std::setw(18) << "\nStep Size" << std::setw(20) << "Hess*Step " << std::setw(18) << "FD Approx"
                   << std::setw(20) << "abs(Error)" << "\n";

        assert(aControl.getNumVectors() > static_cast<OrdinalType>(0));
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tStep = aControl.create();
        assert(tStep.get() != nullptr);
        // NOTE: Think how to syncronize random number generator if working with owned and shared data
        unsigned int tRANDOM_SEED = 1;
        std::srand(tRANDOM_SEED);
        this->random(mRandomNumLowerBound, mRandomNumUpperBound, *tStep);
        this->random(mRandomNumLowerBound, mRandomNumUpperBound, aControl);

        // Compute true Hessian times Step and corresponding norm value
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tHessianTimesStep = aControl.create();
        assert(tHessianTimesStep.get() != nullptr);
        aCriterion.hessian(aControl, *tStep, *tHessianTimesStep);
        const ScalarType tNormHesianTimesStep = Plato::norm(*tHessianTimesStep);

        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tGradient = aControl.create();
        assert(tGradient.get() != nullptr);
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tAppxHessianTimesStep = aControl.create();
        assert(tAppxHessianTimesStep.get() != nullptr);

        // Compute 5-point stencil finite difference approximation
        std::vector<ScalarType> tApproximationErrors;
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tWork = aControl.create();
        assert(tWork.get() != nullptr);
        for(int tIndex = mInitialSuperscript; tIndex <= mFinalSuperscript; tIndex++)
        {
            // Compute \hat{x} = x + \epsilon\Delta{x}, where x denotes the control vector and \Delta{x} denotes the step.
            ScalarType tEpsilon = static_cast<ScalarType>(1) /
                    std::pow(static_cast<ScalarType>(10), tIndex);
            Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *tWork);
            Plato::update(tEpsilon, *tStep, static_cast<ScalarType>(1), *tWork);
            this->gradient(*tWork, *tGradient, aCriterion);
            Plato::update(static_cast<ScalarType>(8), *tGradient, static_cast<ScalarType>(0), *tAppxHessianTimesStep);

            // Compute \hat{x} = x - \epsilon\Delta{x}, where x denotes the control vector and \Delta{x} denotes the step.
            Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *tWork);
            ScalarType tMultiplier = static_cast<ScalarType>(-1) * tEpsilon;
            Plato::update(tMultiplier, *tStep, static_cast<ScalarType>(1), *tWork);
            this->gradient(*tWork, *tGradient, aCriterion);
            Plato::update(static_cast<ScalarType>(-8), *tGradient, static_cast<ScalarType>(1), *tAppxHessianTimesStep);

            // Compute \hat{x} = x + 2\epsilon\Delta{x}, where x denotes the control vector and \Delta{x} denotes the step.
            Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *tWork);
            tMultiplier = static_cast<ScalarType>(2) * tEpsilon;
            Plato::update(tMultiplier, *tStep, static_cast<ScalarType>(1), *tWork);
            this->gradient(*tWork, *tGradient, aCriterion);
            Plato::update(static_cast<ScalarType>(-1), *tGradient, static_cast<ScalarType>(1), *tAppxHessianTimesStep);

            // Compute \hat{x} = x - 2\epsilon\Delta{x}, where x denotes the control vector and \Delta{x} denotes the step.
            Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *tWork);
            tMultiplier = static_cast<ScalarType>(-2) * tEpsilon;
            Plato::update(tMultiplier, *tStep, static_cast<ScalarType>(1), *tWork);
            this->gradient(*tWork, *tGradient, aCriterion);
            Plato::update(static_cast<ScalarType>(1), *tGradient, static_cast<ScalarType>(1), *tAppxHessianTimesStep);

            // Comptute \frac{F(x)}{12}, where F(x) denotes the finite difference approximation of \nabla_{x}^{2}f(x)\Delta{x}
            // and f(x) denotes the respective criterion being evaluated/tested.
            tMultiplier = static_cast<ScalarType>(1) / (static_cast<ScalarType>(12) * tEpsilon);
            Plato::scale(tMultiplier, *tAppxHessianTimesStep);
            ScalarType tNormAppxHesianTimesStep = Plato::norm(*tAppxHessianTimesStep);

            // Compute error between true and finite differenced Hessian times step calculation.
            Plato::update(static_cast<ScalarType>(1), *tHessianTimesStep, static_cast<ScalarType>(-1), *tAppxHessianTimesStep);
            ScalarType tNumerator = Plato::norm(*tAppxHessianTimesStep);
            ScalarType tDenominator = std::numeric_limits<ScalarType>::epsilon() + tNormHesianTimesStep;
            ScalarType tAppxError = tNumerator / tDenominator;
            tApproximationErrors.push_back(tAppxError);

            aOutputMsg << std::right << std::scientific << std::setprecision(8) << std::setw(14) << tEpsilon << std::setw(19)
            << tNormHesianTimesStep << std::setw(19) << tNormAppxHesianTimesStep << std::setw(19) << tAppxError << "\n";
        }

        ScalarType tMinError = *std::min_element(tApproximationErrors.begin(), tApproximationErrors.end());
        mDidHessianTestPassed = tMinError < mHessianTestBound ? true : false;
    }

private:
    void random(const ScalarType & aLowerBound, const ScalarType & aUpperBound, Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        const OrdinalType tNumVectors = aInput.getNumVectors();
        assert(tNumVectors > static_cast<ScalarType>(0));
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            Plato::Vector<ScalarType, OrdinalType> & tMyVector = aInput[tVectorIndex];
            const OrdinalType tMyLength = tMyVector.size();
            assert(tMyLength > static_cast<ScalarType>(0));
            for(OrdinalType tElemIndex = 0; tElemIndex < tMyLength; tElemIndex++)
            {
                tMyVector[tElemIndex] = aLowerBound + ((aUpperBound - aLowerBound) * static_cast<ScalarType>(std::rand() / RAND_MAX));
            }
        }
    }
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aGradient,
                  Plato::Criterion<ScalarType, OrdinalType> & aCriterion)
    {
        Plato::fill(static_cast<ScalarType>(0), aGradient);
        aCriterion.value(aControl);
        aCriterion.cacheData();
        aCriterion.gradient(aControl, aGradient);
    }

private:
    bool mDidHessianTestPassed;
    bool mDidGradientTestPassed;

    int mFinalSuperscript;
    int mInitialSuperscript;

    ScalarType mHessianTestBound;
    ScalarType mGradientTestBound;
    ScalarType mRandomNumLowerBound;
    ScalarType mRandomNumUpperBound;

private:
    Diagnostics(const Plato::Diagnostics<ScalarType, OrdinalType> & aRhs);
    Plato::Diagnostics<ScalarType, OrdinalType> & operator=(const Plato::Diagnostics<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_DIAGNOSTICS_HPP_ */
