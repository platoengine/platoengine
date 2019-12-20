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
 * Plato_MethodMovingAsymptotesOperations.hpp
 *
 *  Created on: Sep 1, 2019
 */

#pragma once

#include <cmath>
#include <limits>
#include <memory>

#include "Plato_DataFactory.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_MethodMovingAsymptotesDataMng.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Main operations for the Method Moving Asymptotes (MMA) optimization algorithm
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class MethodMovingAsymptotesOperations
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aDataFactory factory used to allocate internal metadata
     **********************************************************************************/
    explicit MethodMovingAsymptotesOperations(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory) :
        mMoveLimit(0.5),
        mApproxFuncEpsilon(1e-5),
        mAsymptoteExpansion(1.2),
        mAsymptoteContraction(0.7),
        mApproxFuncScalingOne(1.001),
        mApproxFuncScalingTwo(0.001),
        mInitialAymptoteScaling(0.5),
        mSubProblemBoundsScaling(0.1),
        mUpperMinusLowerBounds(aDataFactory->control().create()),
        mCurrentAsymptotesMultipliers(aDataFactory->control().create())
    {
    }

    /******************************************************************************//**
     * @brief Destructor
     **********************************************************************************/
    ~MethodMovingAsymptotesOperations()
    {
    }

    /******************************************************************************//**
     * @brief Set move limit
     * @param [in] aInput move limit
     **********************************************************************************/
    void setMoveLimit(const ScalarType& aInput)
    {
        mMoveLimit = aInput;
    }

    /******************************************************************************//**
     * @brief Set moving asymptotes expansion parameter
     * @param [in] aInput expansion parameter
     **********************************************************************************/
    void setAsymptoteExpansionParameter(const ScalarType& aInput)
    {
        mAsymptoteExpansion = aInput;
    }

    /******************************************************************************//**
     * @brief Set moving asymptotes contraction parameter
     * @param [in] aInput contraction parameter
     **********************************************************************************/
    void setAsymptoteContractionParameter(const ScalarType& aInput)
    {
        mAsymptoteContraction = aInput;
    }

    /******************************************************************************//**
     * @brief Set scaling for initial moving asymptotes
     * @param [in] aInput scaling for initial moving asymptotes
     **********************************************************************************/
    void setInitialAymptoteScaling(const ScalarType& aInput)
    {
        mInitialAymptoteScaling = aInput;
    }

    /******************************************************************************//**
     * @brief Set scaling parameter on upper and lower bounds for subproblem
     * @param [in] aInput scaling parameter on upper and lower bounds for subproblem
     **********************************************************************************/
    void setSubProblemBoundsScaling(const ScalarType& aInput)
    {
        mSubProblemBoundsScaling = aInput;
    }

    /******************************************************************************//**
     * @brief Gather 2D container of upper minus lower bounds
     * @param [in] aData reference to 2D container
     **********************************************************************************/
    void getUpperMinusLowerBounds(Plato::MultiVector<ScalarType, OrdinalType> &aData)
    {
        Plato::update(static_cast<ScalarType>(1), *mUpperMinusLowerBounds, static_cast<ScalarType>(0), aData);
    }

    /******************************************************************************//**
     * @brief Gather 2D container of current moving asymptotes multipliers
     * @param [in] aData reference to 2D container of current moving asymptotes multipliers
     **********************************************************************************/
    void getCurrentAsymptotesMultipliers(Plato::MultiVector<ScalarType, OrdinalType> &aData)
    {
        Plato::update(static_cast<ScalarType>(1), *mCurrentAsymptotesMultipliers, static_cast<ScalarType>(0), aData);
    }

    /******************************************************************************//**
     * @brief Set lower and upper bounds on the optimization variables
     * @param [in] aDataMng MMA data manager interface
     **********************************************************************************/
    void initialize(const Plato::MethodMovingAsymptotesDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tControlUpperBounds = aDataMng.getControlUpperBounds();
        Plato::update(static_cast<ScalarType>(1), tControlUpperBounds, static_cast<ScalarType>(0), *mUpperMinusLowerBounds);
        const Plato::MultiVector<ScalarType, OrdinalType> &tControlLowerBounds = aDataMng.getControlLowerBounds();
        Plato::update(static_cast<ScalarType>(-1), tControlLowerBounds, static_cast<ScalarType>(1), *mUpperMinusLowerBounds);
    }

    /******************************************************************************//**
     * @brief Update initial moving asymptotes
     * @param [in] aDataMng MMA data manager interface
     **********************************************************************************/
    void updateInitialAsymptotes(Plato::MethodMovingAsymptotesDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = aDataMng.getCurrentControls();
        Plato::MultiVector<ScalarType, OrdinalType> &tLowerAsymptotes = aDataMng.getLowerAsymptotes();
        Plato::update(static_cast<ScalarType>(1), tCurrentControls, static_cast<ScalarType>(0), tLowerAsymptotes);
        Plato::update(-mInitialAymptoteScaling, *mUpperMinusLowerBounds, static_cast<ScalarType>(1), tLowerAsymptotes);

        Plato::MultiVector<ScalarType, OrdinalType> &tUpperAsymptotes = aDataMng.getUpperAsymptotes();
        Plato::update(static_cast<ScalarType>(1), tCurrentControls, static_cast<ScalarType>(0), tUpperAsymptotes);
        Plato::update(mInitialAymptoteScaling, *mUpperMinusLowerBounds, static_cast<ScalarType>(1), tUpperAsymptotes);
    }

    /******************************************************************************//**
     * @brief Update current moving asymptotes multipliers
     * @param [in] aDataMng MMA data manager interface
     **********************************************************************************/
    void updateCurrentAsymptotesMultipliers(Plato::MethodMovingAsymptotesDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = aDataMng.getCurrentControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tPreviousControls = aDataMng.getPreviousControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tAntepenultimateControls = aDataMng.getAntepenultimateControls();

        const OrdinalType tNumVectors = tCurrentControls.getNumVectors();
        for (OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const OrdinalType tNumControls = tCurrentControls[tVectorIndex].size();
            for (OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                const ScalarType tMeasure = (tCurrentControls(tVectorIndex, tControlIndex) - tPreviousControls(tVectorIndex, tControlIndex))
                    * (tPreviousControls(tVectorIndex, tControlIndex) - tAntepenultimateControls(tVectorIndex, tControlIndex));
                ScalarType tGammaValue = tMeasure > static_cast<ScalarType>(0) ? mAsymptoteExpansion : mAsymptoteContraction;
                (*mCurrentAsymptotesMultipliers)(tVectorIndex, tControlIndex) =
                        std::abs(tMeasure) <= std::numeric_limits<ScalarType>::min() ? static_cast<ScalarType>(1) : tGammaValue;
            }
        }
    }

    /******************************************************************************//**
     * @brief Update current moving asymptotes
     * @param [in] aDataMng MMA data manager interface
     **********************************************************************************/
    void updateCurrentAsymptotes(Plato::MethodMovingAsymptotesDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        Plato::MultiVector<ScalarType, OrdinalType> &tLowerAsymptotes = aDataMng.getLowerAsymptotes();
        Plato::MultiVector<ScalarType, OrdinalType> &tUpperAsymptotes = aDataMng.getUpperAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = aDataMng.getCurrentControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tPreviousControls = aDataMng.getPreviousControls();

        const OrdinalType tNumVectors = tCurrentControls.getNumVectors();
        for (OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const OrdinalType tNumControls = tCurrentControls[tVectorIndex].size();
            for (OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                tLowerAsymptotes(tVectorIndex, tControlIndex) = tCurrentControls(tVectorIndex, tControlIndex)
                    - ((*mCurrentAsymptotesMultipliers)(tVectorIndex, tControlIndex)
                            * (tPreviousControls(tVectorIndex, tControlIndex) - tLowerAsymptotes(tVectorIndex, tControlIndex)));
                tUpperAsymptotes(tVectorIndex, tControlIndex) = tCurrentControls(tVectorIndex, tControlIndex)
                    + ((*mCurrentAsymptotesMultipliers)(tVectorIndex, tControlIndex)
                            * (tUpperAsymptotes(tVectorIndex, tControlIndex) - tPreviousControls(tVectorIndex, tControlIndex)));
            }
        }
    }

    /******************************************************************************//**
     * @brief Update current objective approximation functions
     * @param [in] aDataMng MMA data manager interface
     **********************************************************************************/
    void updateObjectiveApproximationFunctionData(Plato::MethodMovingAsymptotesDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        ScalarType tNormalizationValue = std::abs(aDataMng.getCurrentObjectiveValue());
        if(tNormalizationValue <= std::numeric_limits<ScalarType>::epsilon())
        {
            tNormalizationValue = 1.0;
        }

        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = aDataMng.getCurrentControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tLowerAsymptotes = aDataMng.getLowerAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tUpperAsymptotes = aDataMng.getUpperAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentObjGrad = aDataMng.getCurrentObjectiveGradient();

        Plato::MultiVector<ScalarType, OrdinalType> &tAppxFunctionP = aDataMng.getObjFuncAppxFunctionP();
        Plato::MultiVector<ScalarType, OrdinalType> &tAppxFunctionQ = aDataMng.getObjFuncAppxFunctionQ();
        this->computeApproxFuncCoefficients(tNormalizationValue,
                                            tCurrentControls,
                                            tLowerAsymptotes,
                                            tUpperAsymptotes,
                                            tCurrentObjGrad,
                                            tAppxFunctionP,
                                            tAppxFunctionQ);
    }

    /******************************************************************************//**
     * @brief Update current constraint approximation functions
     * @param [in] aDataMng MMA data manager interface
     **********************************************************************************/
    void updateConstraintApproximationFunctionsData(Plato::MethodMovingAsymptotesDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = aDataMng.getCurrentControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tLowerAsymptotes = aDataMng.getLowerAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tUpperAsymptotes = aDataMng.getUpperAsymptotes();

        const OrdinalType tNumConstraints = aDataMng.getNumConstraints();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            const ScalarType tConstraintNormalization = aDataMng.getConstraintNormalization(tIndex);
            const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentConstraintGrad = aDataMng.getCurrentConstraintGradient(tIndex);
            Plato::MultiVector<ScalarType, OrdinalType> &tMyAppxFunctionP = aDataMng.getConstraintAppxFunctionP(tIndex);
            Plato::MultiVector<ScalarType, OrdinalType> &tMyAppxFunctionQ = aDataMng.getConstraintAppxFunctionQ(tIndex);
            this->computeApproxFuncCoefficients(tConstraintNormalization,
                                                tCurrentControls,
                                                tLowerAsymptotes,
                                                tUpperAsymptotes,
                                                tCurrentConstraintGrad,
                                                tMyAppxFunctionP,
                                                tMyAppxFunctionQ);
        }
    }

    /******************************************************************************//**
     * @brief Update subproblem upper and lower bounds
     * @param [in] aDataMng MMA data manager interface
     **********************************************************************************/
    void updateSubProblemBounds(Plato::MethodMovingAsymptotesDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = aDataMng.getCurrentControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tLowerAsymptotes = aDataMng.getLowerAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tUpperAsymptotes = aDataMng.getUpperAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tControlLowerBounds = aDataMng.getControlLowerBounds();
        const Plato::MultiVector<ScalarType, OrdinalType> &tControlUpperBounds = aDataMng.getControlUpperBounds();

        Plato::MultiVector<ScalarType, OrdinalType> &tSubProblemLowerBounds = aDataMng.getSubProblemControlLowerBounds();
        Plato::MultiVector<ScalarType, OrdinalType> &tSubProblemUpperBounds = aDataMng.getSubProblemControlUpperBounds();

        std::vector<ScalarType> tCriteria(3/* length */);
        const OrdinalType tNumVectors = tCurrentControls.getNumVectors();
        for (OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const OrdinalType tNumControls = tCurrentControls[tVectorIndex].size();
            for (OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                tCriteria[0] = tControlLowerBounds(tVectorIndex, tControlIndex);
                tCriteria[1] = tLowerAsymptotes(tVectorIndex, tControlIndex)
                    + (mSubProblemBoundsScaling * (tCurrentControls(tVectorIndex, tControlIndex) - tLowerAsymptotes(tVectorIndex, tControlIndex)));
                tCriteria[2] = tCurrentControls(tVectorIndex, tControlIndex) - (mMoveLimit * (*mUpperMinusLowerBounds)(tVectorIndex, tControlIndex));
                tSubProblemLowerBounds(tVectorIndex, tControlIndex) = *std::max_element(tCriteria.begin(), tCriteria.end());

                tCriteria[0] = tControlUpperBounds(tVectorIndex, tControlIndex);
                tCriteria[1] = tUpperAsymptotes(tVectorIndex, tControlIndex)
                    - (mSubProblemBoundsScaling * (tUpperAsymptotes(tVectorIndex, tControlIndex) - tCurrentControls(tVectorIndex, tControlIndex)));
                tCriteria[2] = tCurrentControls(tVectorIndex, tControlIndex) + (mMoveLimit * (*mUpperMinusLowerBounds)(tVectorIndex, tControlIndex));
                tSubProblemUpperBounds(tVectorIndex, tControlIndex) = *std::min_element(tCriteria.begin(), tCriteria.end());

                if(tSubProblemLowerBounds(tVectorIndex, tControlIndex) > tSubProblemUpperBounds(tVectorIndex, tControlIndex))
                {
                    tSubProblemLowerBounds(tVectorIndex, tControlIndex) = tControlLowerBounds(tVectorIndex, tControlIndex);
                    tSubProblemUpperBounds(tVectorIndex, tControlIndex) = tControlUpperBounds(tVectorIndex, tControlIndex);
                }
            }
        }
    }

private:
    /******************************************************************************//**
     * @brief Compute approximation functions
     * @param [in] aNormalization normalization factor
     * @param [in] aCurrentControls 2D container of current optimization variables
     * @param [in] aLowerAsymptotes 2D container of current lower asymptotes
     * @param [in] aUpperAsymptotes 2D container of current upper asymptotes
     * @param [in] aCriterionGrad 2D container of current criterion gradient
     * @param [in] aAppxFunctionP 2D container of the first approximation function values
     * @param [in] aAppxFunctionQ 2D container of the second approximation function values
     **********************************************************************************/
    void computeApproxFuncCoefficients(const ScalarType &aNormalization,
                                       const Plato::MultiVector<ScalarType, OrdinalType> &aCurrentControls,
                                       const Plato::MultiVector<ScalarType, OrdinalType> &aLowerAsymptotes,
                                       const Plato::MultiVector<ScalarType, OrdinalType> &aUpperAsymptotes,
                                       const Plato::MultiVector<ScalarType, OrdinalType> &aCriterionGrad,
                                       Plato::MultiVector<ScalarType, OrdinalType> &aAppxFunctionP,
                                       Plato::MultiVector<ScalarType, OrdinalType> &aAppxFunctionQ)
    {
        const ScalarType tAbsNormalization = std::abs(aNormalization);
        const OrdinalType tNumVectors = aCriterionGrad.getNumVectors();
        for (OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const OrdinalType tNumControls = aCriterionGrad[tVectorIndex].size();
            for (OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                const ScalarType tNormalizedGradValue = aCriterionGrad(tVectorIndex, tControlIndex) / tAbsNormalization;
                const ScalarType tGradValuePlus = std::max(tNormalizedGradValue, static_cast<ScalarType>(0));
                const ScalarType tGradValueMinus = std::max(-tNormalizedGradValue, static_cast<ScalarType>(0));

                aAppxFunctionP(tVectorIndex, tControlIndex) = (mApproxFuncScalingOne * tGradValuePlus) + (mApproxFuncScalingTwo * tGradValueMinus)
                    + ( mApproxFuncEpsilon / ((*mUpperMinusLowerBounds)(tVectorIndex, tControlIndex) + std::numeric_limits<ScalarType>::epsilon()) );
                ScalarType tUpperAsymmMinusCurrentControlSquared = aUpperAsymptotes(tVectorIndex, tControlIndex)
                    - aCurrentControls(tVectorIndex, tControlIndex);
                tUpperAsymmMinusCurrentControlSquared *= tUpperAsymmMinusCurrentControlSquared;
                aAppxFunctionP(tVectorIndex, tControlIndex) *= tUpperAsymmMinusCurrentControlSquared;

                aAppxFunctionQ(tVectorIndex, tControlIndex) = (mApproxFuncScalingTwo * tGradValuePlus) + (mApproxFuncScalingOne * tGradValueMinus)
                    + ( mApproxFuncEpsilon / ((*mUpperMinusLowerBounds)(tVectorIndex, tControlIndex) + std::numeric_limits<ScalarType>::epsilon()) );
                ScalarType tCurrentControlMinusLowerAsymmSquared = aCurrentControls(tVectorIndex, tControlIndex)
                    - aLowerAsymptotes(tVectorIndex, tControlIndex);
                tCurrentControlMinusLowerAsymmSquared *= tCurrentControlMinusLowerAsymmSquared;
                aAppxFunctionQ(tVectorIndex, tControlIndex) *= tCurrentControlMinusLowerAsymmSquared;
            }
        }
    }

private:
    ScalarType mMoveLimit; /*!< move limit */
    ScalarType mApproxFuncEpsilon; /*!< approximation function epsilon */
    ScalarType mAsymptoteExpansion; /*!< moving asymptotes expansion parameter */
    ScalarType mAsymptoteContraction; /*!< moving asymptotes contraction parameter */
    ScalarType mApproxFuncScalingOne; /*!< scaling factor for approximation function one */
    ScalarType mApproxFuncScalingTwo; /*!< scaling factor for approximation function two */
    ScalarType mInitialAymptoteScaling; /*!< scaling factor for initial moving asymptotes */
    ScalarType mSubProblemBoundsScaling; /*!< scaling factor for subproblem upper and lower bounds */

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperMinusLowerBounds; /*!< 2D container */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentAsymptotesMultipliers; /*!< 2D container of current moving asymptotes multipliers */

private:
    MethodMovingAsymptotesOperations(const Plato::MethodMovingAsymptotesOperations<ScalarType, OrdinalType> & aRhs);
    Plato::MethodMovingAsymptotesOperations<ScalarType, OrdinalType> & operator=(const Plato::MethodMovingAsymptotesOperations<ScalarType, OrdinalType> & aRhs);
};
// class MethodMovingAsymptotesOperations

}
// namespace Plato
