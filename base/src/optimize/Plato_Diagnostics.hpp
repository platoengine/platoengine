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

/*
 * Plato_Diagnostics.hpp
 *
 *  Created on: Jan 9, 2018
 */

#pragma once

#include <cmath>
#include <vector>
#include <limits>
#include <memory>
#include <sstream>
#include <cassert>
#include <iomanip>
#include <cstdlib>
#include <iostream>

#include "Plato_Macros.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Class use to access the diagnostics tools needed to check the analytical
 * criterion gradients and Hessians.
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class Diagnostics
{
public:
    /******************************************************************************//**
     * @brief Default constructor
    **********************************************************************************/
    Diagnostics() :
        mDidHessianTestPassed(false),
        mDidGradientTestPassed(false),
        mFinalSuperscript(8),
        mInitialSuperscript(1),
        mRandomNumLowerBound(0.05),
        mRandomNumUpperBound(0.1),
        mLowerBoundOnHessDigitAccuracy(5),
        mLowerBoundOnGradDigitAccuracy(5)
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~Diagnostics()
    {
    }

    /******************************************************************************//**
     * @brief Returns flag used to specify if the Hessian test passed
     * @return flag (true = pass & false = did not pass)
    **********************************************************************************/
    bool didHessianTestPassed() const
    {
        return (mDidHessianTestPassed);
    }

    /******************************************************************************//**
     * @brief Returns flag used to specify if the gradient test passed
     * @return flag (true = pass & false = did not pass)
    **********************************************************************************/
    bool didGradientTestPassed() const
    {
        return (mDidGradientTestPassed);
    }

    /******************************************************************************//**
     * @brief Set tolerance on finite difference test measure used to specify if the
     * Hessian test passed.  Default value is set to 1e-5.
     * @param [in] aInput finite difference test measure
    **********************************************************************************/
    void setHessianTestBound(const ScalarType & aInput)
    {
        mLowerBoundOnHessDigitAccuracy = aInput;
    }

    /******************************************************************************//**
     * @brief Set tolerance on finite difference test measure used to specify if
     * gradient test passed.  Default value is set to 1e-5.
     * @param [in] aInput finite difference test measure
    **********************************************************************************/
    void setGradientTestBound(const ScalarType & aInput)
    {
        mLowerBoundOnGradDigitAccuracy = aInput;
    }

    /******************************************************************************//**
     * @brief Set lower bound on random number generator.  Basically, the random number
     * generator will generate number between the random number generator lower and
     * upper bounds.
     * @param [in] aInput lower bound on random numbers
    **********************************************************************************/
    void setRandomNumberLowerBound(const ScalarType & aInput)
    {
        mRandomNumLowerBound = aInput;
    }

    /******************************************************************************//**
     * @brief Set upper bound on random number generator.  Basically, the random number
     * generator will generate number between the random number generator lower and
     * upper bounds.
     * @param [in] aInput upper bound on random numbers
    **********************************************************************************/
    void setRandomNumberUpperBound(const ScalarType & aInput)
    {
        mRandomNumUpperBound = aInput;
    }

    /******************************************************************************//**
     * @brief Set superscript on measure used to compute the final finite difference step.
     * The measure is defined as /f$10^{\alpha}/f$, where $\alpha$ denotes the superscript.
     * @param [in] aInput superscript used to compute the finite difference step
    **********************************************************************************/
    void setFinalSuperscript(const int & aInput)
    {
        mFinalSuperscript = aInput;
    }

    /******************************************************************************//**
     * @brief Set superscript on measure used to compute the initial finite difference step.
     * The measure is defined as /f$10^{\alpha}/f$, where $\alpha$ denotes the superscript.
     * @param [in] aInput superscript used to compute the finite difference step
    **********************************************************************************/
    void setInitialSuperscript(const int & aInput)
    {
        mInitialSuperscript = aInput;
    }

    /******************************************************************************//**
     * @brief Check if criterion's analytical gradient is correctly implemented by the
     * application.  The test is based on a four-point finite difference approximation.
     * @param [in] aCriterion interface to application's criterion
     * @param [in] aControl design variables - automatically initialized in the
     *   function by default.
     * @param [in] aOutputMsg output string stream with diagnostics
     * @param [in] aUseInitialGuess used initial control guess provided by the users -
     *   default = false, i.e. controls are randomly generated.
    **********************************************************************************/
    void checkCriterionGradient
    (Plato::Criterion<ScalarType, OrdinalType> & aCriterion,
     Plato::MultiVector<ScalarType, OrdinalType> & aControl,
     std::ostringstream & aOutputMsg,
     bool aUseInitialGuess = false)
    {
        this->checkDimensions(aControl, "CONTROLS");

        mDidGradientTestPassed = false;
        aOutputMsg << std::right << std::setw(18) << "\nStep Size" << std::setw(20) << "Grad'*Step" << std::setw(18) << "FD Approx"
                   << std::setw(20) << "abs(Error)" << "\n";

        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tStep = aControl.create();
        this->checkDimensions(*tStep, "STEP");

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
        this->checkDimensions(*tGradient, "GRADIENT");
        aCriterion.gradient(aControl, *tGradient);
        this->checkValues(*tGradient, "CRITERION GRADIENT");

        std::vector<ScalarType> tApproximationErrors;
        std::vector<ScalarType> tFiniteDiffApprox;
        const ScalarType tTruthGradientDotStep = Plato::dot(*tGradient, *tStep);
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tWork = aControl.create();
        this->checkDimensions(*tWork, "CONTROL WORK");

        for(int tIndex = mInitialSuperscript; tIndex <= mFinalSuperscript; tIndex++)
        {
            // Compute \hat{x} = x + \epsilon\Delta{x}, where x denotes the control vector and \Delta{x} denotes the step.
            ScalarType tEpsilon = static_cast<ScalarType>(1) /
                    std::pow(static_cast<ScalarType>(10), tIndex);
            Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *tWork);
            Plato::update(tEpsilon, *tStep, static_cast<ScalarType>(1), *tWork);
            ScalarType tObjectiveValueAtPlusEpsilon = aCriterion.value(*tWork);
            this->checkValues(tObjectiveValueAtPlusEpsilon, "CRITERION VALUE");

            // Compute \hat{x} = x - \epsilon\Delta{x}, where x denotes the control vector and \Delta{x} denotes the step.
            Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *tWork);
            ScalarType tMultiplier = static_cast<ScalarType>(-1) * tEpsilon;
            Plato::update(tMultiplier, *tStep, static_cast<ScalarType>(1), *tWork);
            ScalarType tObjectiveValueAtMinusEpsilon = aCriterion.value(*tWork);
            this->checkValues(tObjectiveValueAtMinusEpsilon, "CRITERION VALUE");

            // Compute \hat{x} = x + 2\epsilon\Delta{x}, where x denotes the control vector and \Delta{x} denotes the step.
            Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *tWork);
            tMultiplier = static_cast<ScalarType>(2) * tEpsilon;
            Plato::update(tMultiplier, *tStep, static_cast<ScalarType>(1), *tWork);
            ScalarType tObjectiveValueAtPlusTwoEpsilon = aCriterion.value(*tWork);
            this->checkValues(tObjectiveValueAtPlusTwoEpsilon, "CRITERION VALUE");

            // Compute \hat{x} = x - 2\epsilon\Delta{x}, where x denotes the control vector and \Delta{x} denotes the step.
            Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *tWork);
            tMultiplier = static_cast<ScalarType>(-2) * tEpsilon;
            Plato::update(tMultiplier, *tStep, static_cast<ScalarType>(1), *tWork);
            ScalarType tObjectiveValueAtMinusTwoEpsilon = aCriterion.value(*tWork);
            this->checkValues(tObjectiveValueAtMinusTwoEpsilon, "CRITERION VALUE");

            // Compute objective value approximation via a five point stencil finite difference procedure
            ScalarType tObjectiveAppx = (-tObjectiveValueAtPlusTwoEpsilon
                    + static_cast<ScalarType>(8) * tObjectiveValueAtPlusEpsilon
                    - static_cast<ScalarType>(8) * tObjectiveValueAtMinusEpsilon
                    + tObjectiveValueAtMinusTwoEpsilon) / (static_cast<ScalarType>(12) * tEpsilon);

            ScalarType tAppxError = std::abs(tObjectiveAppx - tTruthGradientDotStep);
            tApproximationErrors.push_back(tAppxError);
            tFiniteDiffApprox.push_back(tObjectiveAppx);
            aOutputMsg << std::right << std::scientific << std::setprecision(8) << std::setw(14) << tEpsilon << std::setw(19)
                << tTruthGradientDotStep << std::setw(19) << tObjectiveAppx << std::setw(19) << tAppxError << "\n";
        }

        auto tMinErrorValueItr = std::min_element(tApproximationErrors.begin(), tApproximationErrors.end());
        auto tPosition = std::distance(tApproximationErrors.begin(), tMinErrorValueItr);
        auto tNumAccurateDigits =
            std::floor( std::log10(std::abs(*tMinErrorValueItr)) - std::log10(std::abs(tFiniteDiffApprox[tPosition])) );
        mDidGradientTestPassed = std::abs(tNumAccurateDigits) > mLowerBoundOnGradDigitAccuracy ? true : false;
    }

    /******************************************************************************//**
     * @brief Check if criterion's analytical Hessian is correctly implemented by the
     * application.  The test is based on a four-point finite difference approximation.
     * @param [in] aCriterion interface to application's criterion
     * @param [in] aControl design variables - automatically initialized in the
     *   function by default.
     * @param [in] aOutputMsg output string stream with diagnostics
     * @param [in] aUseInitialGuess used initial control guess provided by the users -
     *   default = false, i.e. controls are randomly generated.
    **********************************************************************************/
    void checkCriterionHessian
    (Plato::Criterion<ScalarType, OrdinalType> & aCriterion,
     Plato::MultiVector<ScalarType, OrdinalType> & aControl,
     std::ostringstream & aOutputMsg,
     bool aUseInitialGuess = false)
    {
        this->checkDimensions(aControl, "CONTROL");

        mDidHessianTestPassed = false;
        aOutputMsg << std::right << std::setw(18) << "\nStep Size" << std::setw(20) << "Hess*Step "
            << std::setw(18) << "FD Approx" << std::setw(20) << "abs(Error)" << "\n";

        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tStep = aControl.create();
        this->checkDimensions(*tStep, "STEP");
        // NOTE: Think how to syncronize random number generator if working with owned and shared data
        unsigned int tRANDOM_SEED = 1;
        std::srand(tRANDOM_SEED);
        this->random(mRandomNumLowerBound, mRandomNumUpperBound, *tStep);
        this->random(mRandomNumLowerBound, mRandomNumUpperBound, aControl);

        // Compute true Hessian times Step and corresponding norm value
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tHessianTimesStep = aControl.create();
        this->checkDimensions(*tHessianTimesStep, "HESSIAN TIMES STEP");
        aCriterion.hessian(aControl, *tStep, *tHessianTimesStep);
        this->checkValues(*tHessianTimesStep, "CRITERION HESSIAN TIMES STEP");
        const ScalarType tTruthNormHesianTimesStep = Plato::norm(*tHessianTimesStep);

        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tGradient = aControl.create();
        this->checkDimensions(*tGradient, "GRADIENT");
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tAppxHessianTimesStep = aControl.create();
        this->checkDimensions(*tAppxHessianTimesStep, "APPROXIMATION HESSIAN TIMES STEP");

        // Compute 5-point stencil finite difference approximation
        std::vector<ScalarType> tApproximationErrors;
        std::vector<ScalarType> tFiniteDiffApprox;
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tWork = aControl.create();
        this->checkDimensions(*tWork, "CONTROL WORK");
        for(int tIndex = mInitialSuperscript; tIndex <= mFinalSuperscript; tIndex++)
        {
            // Compute \hat{x} = x + \epsilon\Delta{x}, where x denotes the control vector and \Delta{x} denotes the step.
            ScalarType tEpsilon = static_cast<ScalarType>(1) /
                    std::pow(static_cast<ScalarType>(10), tIndex);
            Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *tWork);
            Plato::update(tEpsilon, *tStep, static_cast<ScalarType>(1), *tWork);
            this->gradient(*tWork, *tGradient, aCriterion);
            this->checkValues(*tGradient, "CRITERION GRADIENT");
            Plato::update(static_cast<ScalarType>(8), *tGradient, static_cast<ScalarType>(0), *tAppxHessianTimesStep);

            // Compute \hat{x} = x - \epsilon\Delta{x}, where x denotes the control vector and \Delta{x} denotes the step.
            Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *tWork);
            ScalarType tMultiplier = static_cast<ScalarType>(-1) * tEpsilon;
            Plato::update(tMultiplier, *tStep, static_cast<ScalarType>(1), *tWork);
            this->gradient(*tWork, *tGradient, aCriterion);
            this->checkValues(*tGradient, "CRITERION GRADIENT");
            Plato::update(static_cast<ScalarType>(-8), *tGradient, static_cast<ScalarType>(1), *tAppxHessianTimesStep);

            // Compute \hat{x} = x + 2\epsilon\Delta{x}, where x denotes the control vector and \Delta{x} denotes the step.
            Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *tWork);
            tMultiplier = static_cast<ScalarType>(2) * tEpsilon;
            Plato::update(tMultiplier, *tStep, static_cast<ScalarType>(1), *tWork);
            this->gradient(*tWork, *tGradient, aCriterion);
            this->checkValues(*tGradient, "CRITERION GRADIENT");
            Plato::update(static_cast<ScalarType>(-1), *tGradient, static_cast<ScalarType>(1), *tAppxHessianTimesStep);

            // Compute \hat{x} = x - 2\epsilon\Delta{x}, where x denotes the control vector and \Delta{x} denotes the step.
            Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *tWork);
            tMultiplier = static_cast<ScalarType>(-2) * tEpsilon;
            Plato::update(tMultiplier, *tStep, static_cast<ScalarType>(1), *tWork);
            this->gradient(*tWork, *tGradient, aCriterion);
            this->checkValues(*tGradient, "CRITERION GRADIENT");
            Plato::update(static_cast<ScalarType>(1), *tGradient, static_cast<ScalarType>(1), *tAppxHessianTimesStep);

            // Comptute \frac{F(x)}{12}, where F(x) denotes the finite difference approximation of \nabla_{x}^{2}f(x)\Delta{x}
            // and f(x) denotes the respective criterion being evaluated/tested.
            tMultiplier = static_cast<ScalarType>(1) / (static_cast<ScalarType>(12) * tEpsilon);
            Plato::scale(tMultiplier, *tAppxHessianTimesStep);
            ScalarType tNormAppxHesianTimesStep = Plato::norm(*tAppxHessianTimesStep);
            tFiniteDiffApprox.push_back(tNormAppxHesianTimesStep);

            // Compute error between true and finite differenced Hessian times step calculation.
            Plato::update(static_cast<ScalarType>(1), *tHessianTimesStep, static_cast<ScalarType>(-1), *tAppxHessianTimesStep);
            ScalarType tNumerator = Plato::norm(*tAppxHessianTimesStep);
            ScalarType tDenominator = std::numeric_limits<ScalarType>::epsilon() + tTruthNormHesianTimesStep;
            ScalarType tAppxError = tNumerator / tDenominator;
            tApproximationErrors.push_back(tAppxError);

            aOutputMsg << std::right << std::scientific << std::setprecision(8) << std::setw(14) << tEpsilon << std::setw(19)
            << tTruthNormHesianTimesStep << std::setw(19) << tNormAppxHesianTimesStep << std::setw(19) << tAppxError << "\n";
        }

        auto tMinErrorValueItr = std::min_element(tApproximationErrors.begin(), tApproximationErrors.end());
        auto tPosition = std::distance(tApproximationErrors.begin(), tMinErrorValueItr);
        auto tNumAccurateDigits =
            std::floor( std::log10(std::abs(*tMinErrorValueItr)) - std::log10(std::abs(tFiniteDiffApprox[tPosition])) );
        mDidHessianTestPassed = std::abs(tNumAccurateDigits) > mLowerBoundOnHessDigitAccuracy ? true : false;
    }

private:
    /******************************************************************************//**
     * @brief Fill input 2D container with random numbers between lower and upper bounds.
     * @param [in] aLowerBound lower bound on random numbers
     * @param [in] aUpperBound upper bound on random numbers
     * @param [in/out] aInput 2D container
    **********************************************************************************/
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
                const ScalarType tRand = static_cast<ScalarType>(std::rand()) / static_cast<ScalarType>(RAND_MAX);
                tMyVector[tElemIndex] = aLowerBound + ((aUpperBound - aLowerBound) * tRand);
            }
        }
    }

    /******************************************************************************//**
     * @brief Evaluate criterion's analytical gradient
     * @param [in] aControl 2D container of control, i.e. optimization, variables
     * @param [in/out] aGradient 2D container with analytical gradient values
     * @param [in/out] aCriterion interface to application's criterion
    **********************************************************************************/
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aGradient,
                  Plato::Criterion<ScalarType, OrdinalType> & aCriterion)
    {
        Plato::fill(static_cast<ScalarType>(0), aGradient);
        aCriterion.value(aControl);
        aCriterion.cacheData();
        aCriterion.gradient(aControl, aGradient);
    }

    /******************************************************************************//**
     * @brief Check dimensions of 2D container
     * @param [in] aInput 2D container
     * @param [in] aName container's name
    **********************************************************************************/
    void checkDimensions(const Plato::MultiVector<ScalarType, OrdinalType> & aInput, const std::string & aName)
    {
        if(aInput.getNumVectors() <= static_cast<OrdinalType>(0))
        {
            THROWERR("INPUT CONTAINER WITH NAME = " + aName + "IS EMPTY\n")
        }
    }

    /******************************************************************************//**
     * @brief Check if values of 2D container are finite
     * @param [in] aInput 2D container
     * @param [in] aName container's name
    **********************************************************************************/
    void checkValues(const Plato::MultiVector<ScalarType, OrdinalType> & aInput, const std::string & aName)
    {
        ScalarType tValue = Plato::dot(aInput, aInput);
        if(std::isfinite(tValue) == false)
        {
            THROWERR("INPUT CONTAINER WITH NAME = " + aName + " HAS NONE FINITE NUMBERS\n")
        }
    }

    /******************************************************************************//**
     * @brief Check if the number is finite
     * @param [in] aInput scalar number
     * @param [in] aName scalar name
    **********************************************************************************/
    void checkValues(const ScalarType & aInput, const std::string & aName)
    {
        if(std::isfinite(aInput) == false)
        {
            THROWERR("INPUT SCALAR WITH NAME = " + aName + " HAS NONE FINITE NUMBERS\n")
        }
    }

private:
    bool mDidHessianTestPassed; /*!< flag: true = Hessian check passed & false = Hessian check did not pass */
    bool mDidGradientTestPassed; /*!< flag: true = gradient check passed & false = gradient check did not pass */

    int mFinalSuperscript; /*!< superscript on measure used to compute the final finite difference step */
    int mInitialSuperscript; /*!< superscript on measure used to compute the initial finite difference step */

    ScalarType mRandomNumLowerBound; /*!< lower bound on random number generator */
    ScalarType mRandomNumUpperBound; /*!< upper bound on random number generator */
    ScalarType mLowerBoundOnHessDigitAccuracy; /*!< tolerance on finite difference Hessian test measure - used to specify if test passed */
    ScalarType mLowerBoundOnGradDigitAccuracy; /*!< tolerance on finite difference gradient test measure - used to specify if test passed */

private:
    Diagnostics(const Plato::Diagnostics<ScalarType, OrdinalType> & aRhs);
    Plato::Diagnostics<ScalarType, OrdinalType> & operator=(const Plato::Diagnostics<ScalarType, OrdinalType> & aRhs);
};
// class Diagnostics

}
// namespace Plato

