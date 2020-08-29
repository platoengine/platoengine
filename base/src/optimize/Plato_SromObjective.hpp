/*
 * Plato_SromObjective.hpp
 *
 *  Created on: Jan 31, 2018
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

#ifndef PLATO_SROMOBJECTIVE_HPP_
#define PLATO_SROMOBJECTIVE_HPP_

#include <cmath>
#include <memory>
#include <cassert>

#include "Plato_Criterion.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_Distribution.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_SromStatistics.hpp"
#include "Plato_StandardVector.hpp"
#include "Plato_StandardMultiVector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class SromObjective : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * \brief Constructor
     * \param [in] aDistribution  interface to probability distribution function
     * \param [in] aMaxNumMoments number of matching statistical moments
     * \param [in] aNumSamples    number of samples
     * \param [in] aRandomVecDim  random vector dimensions
    **********************************************************************************/
    explicit SromObjective
    (const std::shared_ptr<Plato::Distribution<ScalarType, OrdinalType>> & aDistribution,
     const OrdinalType & aMaxNumMoments,
     const OrdinalType & aNumSamples,
     OrdinalType aRandomVecDim = 1) :
            mNumSamples(aNumSamples),
            mMaxNumMoments(aMaxNumMoments),
            mRandomVectorDim(aRandomVecDim),
            mSromSigma(1e-3),
            mSqrtConstant(0),
            mWeightCdfMisfit(1),
            mWeightMomentMisfit(1),
            mWeightCorrelationMisfit(1),
            mSromSigmaTimesSigma(0),
            mFrobeniusNormTruthCorrelationMatrix(1),
            mCumulativeDistributionFunctionError(0),
            mSromMoments(std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(mMaxNumMoments)),
            mTruthMoments(std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(mMaxNumMoments)),
            mSromCDF(std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(mRandomVectorDim, mNumSamples)),
            mTruthCDF(std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(mRandomVectorDim, mNumSamples)),
            mMomentsError(std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(mRandomVectorDim, mMaxNumMoments)),
            mMomentsMisfit(std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(mRandomVectorDim, mMaxNumMoments)),
            mSromCorrelation(std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(mRandomVectorDim, mRandomVectorDim)),
            mTruthCorrelation(std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(mRandomVectorDim, mRandomVectorDim)),
            mDistribution(aDistribution)
    {
        this->setConstants();
        this->setTrueMoments();
        this->setDefaultTruthCorrelationMatrix();
    }

    /******************************************************************************//**
     * \brief Destructor
    **********************************************************************************/
    virtual ~SromObjective() { }

    /******************************************************************************//**
     * \brief Set parameter used to control the degree of smoothness of the SROM CDF
     * \param [in] aInput parameter used to control the degree of smoothness
    **********************************************************************************/
    void setSromSigma(const ScalarType & aInput)
    {
        mSromSigma = aInput;
        this->setConstants(); // Reset constants due to new input
    }

    /******************************************************************************//**
     * \brief Set weight used to ensure CDF error term has the same order of magnitude \n
     * than the other two error terms in the objective function.
     * \param [in] aInput weight
    **********************************************************************************/
    void setCdfMisfitTermWeight(const ScalarType & aInput)
    {
        mWeightCdfMisfit = aInput;
    }

    /******************************************************************************//**
     * \brief Set weight used to ensure moment error term has the same order of magnitude \n
     * than the other two error terms in the objective function.
     * \param [in] aInput weight
    **********************************************************************************/
    void setMomentMisfitTermWeight(const ScalarType & aInput)
    {
        mWeightMomentMisfit = aInput;
    }

    /******************************************************************************//**
     * \brief Set weight used to ensure correlation error term has the same order \n
     * of magnitude than the other two error terms in the objective function.
     * \param [in] aInput weight
    **********************************************************************************/
    void setCorrelationMisfitTermWeight(const ScalarType & aInput)
    {
        mWeightCorrelationMisfit = aInput;
    }

    /******************************************************************************//**
     * \brief Set truth correlation matrix
     * \param [in] aInput truth correlation matrix
    **********************************************************************************/
    void setTruthCorrelationMatrix(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        if(aInput.getNumVectors() != mRandomVectorDim)
        {
            THROWERR(std::string("Dimension mismatch: Number of input random vector dimensions does not match number of class")
                + "member random vector dimensions. Input random vector dimensions is '" + std::to_string(aInput.getNumVectors())
                + "' and number of class member random vector dimensions is '" + std::to_string(mRandomVectorDim) + "'.")
        }
        Plato::update(1.0, aInput, 0.0, *mTruthCorrelation);
        mFrobeniusNormTruthCorrelationMatrix = Plato::norm(*mTruthCorrelation);
    }

    /******************************************************************************//**
     * \brief Return cumulative distribution function (CDF) error
     * \return error between true and SROM CDF
    **********************************************************************************/
    ScalarType getCumulativeDistributionFunctionError() const
    {
        return (mCumulativeDistributionFunctionError);
    }

    /******************************************************************************//**
     * \brief Return number of samples
     * \return number of samples
    **********************************************************************************/
    OrdinalType getNumSamples() const
    {
        return (mNumSamples);
    }

    /******************************************************************************//**
     * \brief Return the maximum number of moments matched through optimization.
     * \return maximum number of moments
    **********************************************************************************/
    OrdinalType getMaxNumMoments() const
    {
        return (mMaxNumMoments);
    }

    /******************************************************************************//**
     * \brief Return moment errors for all random dimension
     * \return multi-vector with moment errors for all random dimension
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getMomentError() const
    {
        return (*mMomentsError);
    }


    /******************************************************************************//**
     * \brief Return moment errors for input random dimension
     * \param [in] aInput random vector dimension
     * \return vector with moment errors for input random dimension
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType>& getMomentError(const OrdinalType& aInput) const
    {
        assert(aInput < mMomentsError->getNumVectors());
        return (mMomentsError->operator[](aInput));
    }

    /******************************************************************************//**
     * \brief Return true moments
     * \return vector with true moments in increasing order, i.e. first element
     *          corresponds to the first moment (i.e. mean) and the last element
     *          corresponds to the last moment
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType>& getTrueMoments() const
    {
        return (*mTruthMoments);
    }

    /******************************************************************************//**
     * \brief Return SROM moments
     * \return vector with SROM moments in increasing order, i.e. first element
     *          corresponds to the first SROM moment approximation (i.e. mean) and
     *          the last element corresponds to the last SROM moment approximation
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType>& getSromMoments() const
    {
        return (*mSromMoments);
    }

    /******************************************************************************//**
     * \brief Return true cumulative distribution function evaluations
     * \return vector with true cumulative distribution function evaluations at the
     *         sample inputs to the objective function (for all random dimensions).
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getTrueCDF() const
    {
        return (*mTruthCDF);
    }

    /******************************************************************************//**
     * \brief Return SROM cumulative distribution function evaluations
     * \return vector with SROM cumulative distribution function evaluations at the
     *         sample inputs to the objective function (for all random dimensions).
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getSromCDF() const
    {
        return (*mSromCDF);
    }

    /******************************************************************************//**
     * \brief Return trust correlation matrix.
     * \return trust correlation matrix.
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getTruthCorrelationMatrix() const
    {
        return (*mTruthCorrelation);
    }

    /******************************************************************************//**
     * \brief Return true cumulative distribution function evaluations
     * \param [in] aInput random vector dimension
     * \return vector with true cumulative distribution function evaluations at the
     *         sample inputs to the objective function (for all random dimensions).
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType>& getTrueCDF(const OrdinalType& aInput) const
    {
        assert(aInput < mTruthCDF->getNumVectors());
        return (mTruthCDF->operator[](aInput));
    }

    /******************************************************************************//**
     * \brief Return SROM cumulative distribution function evaluations
     * \param [in] aInput random vector dimension
     * \return vector with SROM cumulative distribution function evaluations at the
     *         sample inputs to the objective function (for all random dimensions).
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType>& getSromCDF(const OrdinalType& aInput) const
    {
        assert(aInput < mSromCDF->getNumVectors());
        return (mSromCDF->operator[](aInput));
    }

    /******************************************************************************//**
     * \brief Null operation - operation is not needed for SROM optimization problem
    **********************************************************************************/
    void cacheData() { return; }

    /******************************************************************************//**
     * \brief Evaluate SROM objective, defined as:
     *
     * \f$ f(x) = \frac{1}{2}\left( \alpha_{1}\varepsilon_1 + \alpha_{2}\varepsilon_2 + \alpha_{3}\varepsilon_3 \right) \f$, \n
     * where,
     * \f$ \varepsilon_1 = \mbox{moment misfit term\f$
     * \f$ \varepsilon_2 = \mbox{cdf misfit term\f$
     * \f$ \varepsilon_3 = \mbox{correlation misfit term\f$
     *
     * \param [in] aControl optimization variables
     * \return criterion evaluation
    **********************************************************************************/
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        const auto tMomentsMisfit = this->computeMomentsMisfit(aControl);
        const auto tCorrelationMisfit = this->computeCorrelationMisfit(aControl);
        mCumulativeDistributionFunctionError = this->computeCumulativeDistributionFunctionMisfit(aControl);
        const auto tOutput = static_cast<ScalarType>(0.5) * (mWeightCdfMisfit * mCumulativeDistributionFunctionError
            + mWeightMomentMisfit * tMomentsMisfit + mWeightCorrelationMisfit * tCorrelationMisfit);
        return (tOutput);
    }

    /******************************************************************************//**
     * \brief Evaluate SROM gradient
     * \param [in] aControl optimization variables
     * \param [in,out] aOutput SROM gradient
    **********************************************************************************/
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aControl.getNumVectors() == aOutput.getNumVectors());
        this->addGradContributionFromMomentAndCDF(aControl, aOutput);
        auto tAreSamplesCorrelated = mFrobeniusNormTruthCorrelationMatrix != static_cast<ScalarType>(1.0);
        if((mRandomVectorDim > static_cast<OrdinalType>(1)) && tAreSamplesCorrelated)
        {
            this->addGradContributionFromCorrelation(aControl, aOutput);
        }
    }

    /******************************************************************************//**
     * \brief Apply vector to Hessian operator
     * \param [in] aControl optimization variables
     * \param [in] aVector descent direction
     * \param [in,out] aOutput application of vector to Hessian operator
    **********************************************************************************/
    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        Plato::update(static_cast<ScalarType>(1), aVector, static_cast<ScalarType>(0), aOutput);
    }

private:
    /******************************************************************************//**
     * \fn addGradContributionFromCorrelation
     * \brief Add contribution from correlation error term to output gradient.
     * \param [in]   aControl optimization variables
     * \param [out]  aOutput  output gradient
    **********************************************************************************/
    void addGradContributionFromCorrelation
    (const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
     Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        const auto tRandomVecDim = aOutput.getNumVectors() - static_cast<OrdinalType>(1);
        const auto &tProbabilities = aControl[tRandomVecDim];
        auto& tGradProbabilities = aOutput[tRandomVecDim];
        Plato::compute_srom_correlation_matrix(tProbabilities, aControl, *mSromCorrelation);

        auto tNumSamples = this->getNumSamples();
        for(decltype(mRandomVectorDim) tDimIndex = 0; tDimIndex < mRandomVectorDim; tDimIndex++)
        {
            auto& tMyDimGradWrtSamples = aOutput[tDimIndex];
            for(decltype(tNumSamples) tSampleIndex = 0; tSampleIndex < tNumSamples; tSampleIndex++)
            {
                auto tPartialCorrelationWrtSample = this->partialCorrelationErrorWrtSamples(tDimIndex, tSampleIndex, aControl);
                tMyDimGradWrtSamples[tSampleIndex] += mWeightCorrelationMisfit * tPartialCorrelationWrtSample;
            }
        }

        for(decltype(tNumSamples) tSampleIndex = 0; tSampleIndex < tNumSamples; tSampleIndex++)
        {
            auto tPartialCorrelationWrtProbabilities = this->partialCorrelationErrorWrtProbabilities(tSampleIndex, aControl);
            tGradProbabilities[tSampleIndex] += mWeightCorrelationMisfit * tPartialCorrelationWrtProbabilities;
        }
    }

    /******************************************************************************//**
     * \fn partialCorrelationErrorWrtSamples
     * \brief Compute partial derivative of correlation error term with respect to samples.
     * \param [in]  aOuterDimIndex     outer loop random vector dimension index
     * \param [in]  aOuterSampleIndex  outer loop sample index
     * \param [in]  aControl           optimization variables
     * \return partial derivative of correlation error term with respect to samples
    **********************************************************************************/
    ScalarType partialCorrelationErrorWrtSamples
    (const OrdinalType& aOuterDimIndex,
     const OrdinalType& aOuterSampleIndex,
     const Plato::MultiVector<ScalarType, OrdinalType>& aControl)
    {
        ScalarType tOutput = 0;
        const auto &tProbabilities = aControl[mRandomVectorDim];
        for (decltype(mRandomVectorDim) tDimI = 0; tDimI < mRandomVectorDim - static_cast<OrdinalType>(1); tDimI++)
        {
            for (decltype(mRandomVectorDim) tDimJ = tDimI + static_cast<OrdinalType>(1); tDimJ < mRandomVectorDim; tDimJ++)
            {
                auto tConstant = static_cast<ScalarType>(1.0) / (*mTruthCorrelation)(tDimI, tDimJ);
                //Account for Kronecker delta in derivative expression:
                if (tDimI == aOuterDimIndex)
                {
                    tOutput += tConstant * tConstant * ((*mSromCorrelation)(tDimI, tDimJ) - (*mTruthCorrelation)(tDimI, tDimJ))
                        * tProbabilities[aOuterSampleIndex] * aControl(tDimJ, aOuterSampleIndex);
                }
                else if (tDimJ == aOuterDimIndex)
                {
                    tOutput += tConstant * tConstant * ((*mSromCorrelation)(tDimI, tDimJ) - (*mTruthCorrelation)(tDimI, tDimJ))
                        * tProbabilities[aOuterSampleIndex] * aControl(tDimI, aOuterSampleIndex);
                }
            }
        }
        return tOutput;
    }

    /******************************************************************************//**
     * \fn partialCorrelationErrorWrtProbabilities
     * \brief Compute partial derivative of correlation error term with respect to probabilities.
     * \param [in]  aOuterSampleIndex  outer loop sample index
     * \param [in]  aControl           optimization variables
     * \return partial derivative of correlation error term with respect to probabilities
    **********************************************************************************/
    ScalarType partialCorrelationErrorWrtProbabilities
    (const OrdinalType& aOuterSampleIndex,
     const Plato::MultiVector<ScalarType, OrdinalType>& aControl)
    {
        ScalarType tOutput = 0;
        for (decltype(mRandomVectorDim) tDimI = 0; tDimI < mRandomVectorDim - static_cast<OrdinalType>(1); tDimI++)
        {
            for (decltype(mRandomVectorDim) tDimJ = tDimI + static_cast<OrdinalType>(1); tDimJ < mRandomVectorDim; tDimJ++)
            {
                auto tConstant = static_cast<ScalarType>(1.0) / (*mTruthCorrelation)(tDimI, tDimJ);
                tOutput += tConstant * tConstant * ((*mSromCorrelation)(tDimI, tDimJ) - (*mTruthCorrelation)(tDimI, tDimJ))
                    * aControl(tDimI, aOuterSampleIndex) * aControl(tDimJ, aOuterSampleIndex);
            }
        }
        return tOutput;
    }

    /******************************************************************************//**
     * \fn addGradContributionFromMomentAndCDF
     * \brief Add contribution from moment and cumulative distribution function error \n
     * terms to output gradient.
     * \param [in]   aControl optimization variables
     * \param [out]  aOutput  output gradient
    **********************************************************************************/
    void addGradContributionFromMomentAndCDF
    (const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
     Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        const auto& tProbabilities = aControl[mRandomVectorDim];
        auto& tGradientProbabilities = aOutput[mRandomVectorDim];

        auto tNumSamples = this->getNumSamples();
        for(decltype(mRandomVectorDim) tDimIndex = 0; tDimIndex < mRandomVectorDim; tDimIndex++)
        {
            const auto& tMySamples = aControl[tDimIndex];
            auto& tMySamplesGradient = aOutput[tDimIndex];
            auto& tMyMomentMisfit = mMomentsMisfit->operator[](tDimIndex);
            for(decltype(tNumSamples) tSampleIndex = 0; tSampleIndex < tNumSamples; tSampleIndex++)
            {
                // Samples' Gradient
                auto tSample_ij = tMySamples[tSampleIndex];
                auto tProbability_ij = tProbabilities[tSampleIndex];
                auto tPartialCDFwrtSample =
                    this->partialCumulativeDistributionFunctionWrtSamples(tSample_ij, tProbability_ij, tMySamples, tProbabilities);
                auto tPartialMomentWrtSample = this->partialMomentsWrtSamples(tSample_ij, tProbability_ij, tMyMomentMisfit);
                tMySamplesGradient[tSampleIndex] = (mWeightCdfMisfit * tPartialCDFwrtSample)
                    + (mWeightMomentMisfit * tPartialMomentWrtSample);

                // Probabilities' Gradient
                auto tPartialCDFwrtProbability =
                    this->partialCumulativeDistributionFunctionWrtProbabilities(tSample_ij, tMySamples, tProbabilities);
                auto tPartialMomentWrtProbability = this->partialMomentsWrtProbabilities(tSample_ij, tMyMomentMisfit);
                tGradientProbabilities[tSampleIndex] += (mWeightCdfMisfit * tPartialCDFwrtProbability
                    + mWeightMomentMisfit * tPartialMomentWrtProbability);
            }
        }
    }

    /******************************************************************************//**
     * \brief Set SROM constants
    **********************************************************************************/
    void setConstants()
    {
        mSromSigmaTimesSigma = mSromSigma * mSromSigma;
        mSqrtConstant = static_cast<ScalarType>(2) * static_cast<ScalarType>(M_PI) * mSromSigmaTimesSigma;
        mSqrtConstant = std::sqrt(mSqrtConstant);
    }

    /******************************************************************************//**
     * \brief Precompute true moments for target statistics.
    **********************************************************************************/
    void setTrueMoments()
    {
        for(decltype(mMaxNumMoments) tIndex = 0; tIndex < mMaxNumMoments; tIndex++)
        {
            auto tMyOrder = tIndex + static_cast<OrdinalType>(1);
            mTruthMoments->operator[](tIndex) = mDistribution->moment(tMyOrder);
        }
    }

    /******************************************************************************//**
     * \brief Compute partial derivative of moment term with respect to samples.
     * \param [in] aSampleIJ sample IJ
     * \param [in] aProbabilityIJ probability IJ
     * \param [in] aMomentsMisfit moment misfit term
    **********************************************************************************/
    void setDefaultTruthCorrelationMatrix()
    {
        for (decltype(mRandomVectorDim) tDimI = 0; tDimI < mRandomVectorDim; tDimI++)
        {
            (*mTruthCorrelation)(tDimI, tDimI) = 1.0;
        }
    }

    /******************************************************************************//**
     * \brief Compute partial derivative of moment term with respect to samples.
     * \param [in] aSampleIJ sample IJ
     * \param [in] aProbabilityIJ probability IJ
     * \param [in] aMomentsMisfit moment misfit term
    **********************************************************************************/
    ScalarType partialMomentsWrtSamples
    (const ScalarType & aSampleIJ,
     const ScalarType & aProbabilityIJ,
     const Plato::Vector<ScalarType, OrdinalType> & aMomentsMisfit)
    {
        assert(mMaxNumMoments == aMomentsMisfit.size());

        ScalarType tSum = 0;
        for(decltype(mMaxNumMoments) tIndexK = 0; tIndexK < mMaxNumMoments; tIndexK++)
        {
            auto tTrueMomentTimesTrueMoment = mTruthMoments->operator[](tIndexK) * mTruthMoments->operator[](tIndexK);
            auto tConstant = static_cast<ScalarType>(1) / tTrueMomentTimesTrueMoment;
            auto tMomentOrder = tIndexK + static_cast<OrdinalType>(1);
            tSum = tSum + (tConstant * aMomentsMisfit[tIndexK] * tMomentOrder * aProbabilityIJ
                    * std::pow(aSampleIJ, static_cast<ScalarType>(tIndexK)));
        }
        return (tSum);
    }

    /******************************************************************************//**
     * \brief Compute partial derivative of moment term with respect to probabilities.
     * \param [in] aSampleIJ sample IJ
     * \param [in] aMomentsMisfit moment misfit term
    **********************************************************************************/
    ScalarType partialMomentsWrtProbabilities
    (const ScalarType & aSampleIJ,
     const Plato::Vector<ScalarType, OrdinalType> & aMomentsMisfit)
    {
        // Compute sensitivity in dimension k:
        ScalarType tSum = 0;
        auto tNumMoments = aMomentsMisfit.size();
        for(decltype(tNumMoments) tIndexK = 0; tIndexK < tNumMoments; tIndexK++)
        {
            auto tTrueMomentTimesTrueMoment = mTruthMoments->operator[](tIndexK) * mTruthMoments->operator[](tIndexK);
            auto tConstant = static_cast<ScalarType>(1) / tTrueMomentTimesTrueMoment;
            auto tExponent = tIndexK + static_cast<OrdinalType>(1);
            tSum = tSum + (tConstant * aMomentsMisfit[tIndexK] * std::pow(aSampleIJ, tExponent));
        }
        return (tSum);
    }

    /******************************************************************************//**
     * \brief Compute partial derivative of CDF term with respect to samples.
     * \param [in] aSampleIJ sample IJ
     * \param [in] aProbabilityIJ probability IJ
     * \param [in] aSamples trial samples
     * \param [in] aSamples trial probabilities
    **********************************************************************************/
    ScalarType partialCumulativeDistributionFunctionWrtSamples
    (const ScalarType & aSampleIJ,
     const ScalarType & aProbabilityIJ,
     const Plato::Vector<ScalarType, OrdinalType> & aSamples,
     const Plato::Vector<ScalarType, OrdinalType> & aProbabilities)
    {
        ScalarType tSum = 0;
        for(decltype(mNumSamples) tSampleIndexK = 0; tSampleIndexK < mNumSamples; tSampleIndexK++)
        {
            auto tSample_ik = aSamples[tSampleIndexK];
            auto tConstant = aProbabilities[tSampleIndexK] / mSqrtConstant;
            auto tExponent = (static_cast<ScalarType>(-1) / (static_cast<ScalarType>(2) * mSromSigmaTimesSigma))
                    * (aSampleIJ - tSample_ik) * (aSampleIJ - tSample_ik);
            tSum = tSum + (tConstant * std::exp(tExponent));
        }

        const auto tTruePDF = mDistribution->pdf(aSampleIJ);
        const auto tTrueCDF = mDistribution->cdf(aSampleIJ);
        const auto tSromCDF = Plato::compute_srom_cdf<ScalarType, OrdinalType>(aSampleIJ, mSromSigma, aSamples, aProbabilities);
        const auto tMisfitCDF = tSromCDF - tTrueCDF;
        const auto tTermOne = tMisfitCDF * (tSum - tTruePDF);

        ScalarType tTermTwo = 0;
        for(decltype(mNumSamples) tSampleIndexL = 0; tSampleIndexL < mNumSamples; tSampleIndexL++)
        {
            auto tSample_il= aSamples[tSampleIndexL];
            auto tMyTrueCDF = mDistribution->cdf(tSample_il);
            auto tMySromCDF = Plato::compute_srom_cdf<ScalarType, OrdinalType>(tSample_il, mSromSigma, aSamples, aProbabilities);
            auto tMyMisfitCDF = tMySromCDF - tMyTrueCDF;
            auto tConstant = aProbabilityIJ / mSqrtConstant;
            auto tExponent = (static_cast<ScalarType>(-1) / (static_cast<ScalarType>(2) * mSromSigmaTimesSigma))
                * (tSample_il - aSampleIJ) * (tSample_il - aSampleIJ);
            tTermTwo = tTermTwo + (tMyMisfitCDF * tConstant * std::exp(tExponent));
        }

        const auto tOutput = tTermOne - tTermTwo;
        return (tOutput);
    }

    /******************************************************************************//**
     * \brief Compute partial derivative of CDF term with respect to probabilities.
     * \param [in] aSampleIJ sample IJ
     * \param [in] aSamples trial samples
     * \param [in] aSamples trial probabilities
    **********************************************************************************/
    ScalarType
    partialCumulativeDistributionFunctionWrtProbabilities
    (const ScalarType & aSampleIJ,
     const Plato::Vector<ScalarType, OrdinalType> & aSamples,
     const Plato::Vector<ScalarType, OrdinalType> & aProbabilities)
    {
        ScalarType tSum = 0;
        auto tNumSamples = this->getNumSamples();
        for(decltype(tNumSamples) tSampleIndexK = 0; tSampleIndexK < tNumSamples; tSampleIndexK++)
        {
            // Compute different in true/SROM CDFs at x_ij:
            auto tTrueCDF = mDistribution->cdf(aSamples[tSampleIndexK]);
            auto tSromCDF =
                Plato::compute_srom_cdf<ScalarType, OrdinalType>(aSamples[tSampleIndexK], mSromSigma, aSamples, aProbabilities);
            auto tMisfitCDF = tSromCDF - tTrueCDF;
            // Compute CDF derivative term:
            auto tNumerator = aSamples[tSampleIndexK] - aSampleIJ;
            auto tDenominator = std::sqrt(static_cast<ScalarType>(2)) * mSromSigma;
            auto tSensitivity = static_cast<ScalarType>(1) + std::erf(tNumerator / tDenominator);
            // Add contribution to summation in k:
            tSum = tSum + (tMisfitCDF * tSensitivity);
        }
        tSum = static_cast<ScalarType>(0.5) * tSum;
        return (tSum);
    }

    /******************************************************************************//**
     * \brief Compute misfit in moments, i.e. difference between target and SROM moments.
     * \param [in] aControl sample/probability pairs
     *
     * NOTE:
     * Compute misfit in moments up to order q for ith dimension (i.e. i-th random vector
     * dimension). Currently, the random vector dimension is always set to one. Hence,
     * random vector has size one and samples are not correlated.
    **********************************************************************************/
    ScalarType computeCorrelationMisfit(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        ScalarType tCorrelationMisfit = 0;
        auto tRandVecDim = aControl.getNumVectors() - static_cast<OrdinalType>(1);
        auto tAreSamplesCorrelated = mFrobeniusNormTruthCorrelationMatrix != static_cast<ScalarType>(1.0);
        if ((tRandVecDim > static_cast<OrdinalType>(1)) && tAreSamplesCorrelated)
        {
            const auto& tProbabilities = aControl[tRandVecDim];
            Plato::compute_srom_correlation_matrix(tProbabilities, aControl, *mSromCorrelation);
            tCorrelationMisfit = Plato::compute_correlation_misfit(*mSromCorrelation, *mTruthCorrelation);
        }
        return tCorrelationMisfit;
    }

    /******************************************************************************//**
     * \brief Compute misfit in moments, i.e. difference between target and SROM moments.
     * \param [in] aControl sample/probability pairs
     *
     * NOTE:
     * Compute misfit in moments up to order q for ith dimension (i.e. i-th random vector
     * dimension). Currently, the random vector dimension is always set to one. Hence,
     * random vector has size one and samples are not correlated.
    **********************************************************************************/
    ScalarType computeMomentsMisfit(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(mMaxNumMoments == mTruthMoments->size());
        assert(aControl.getNumVectors() >= static_cast<OrdinalType>(2));
        const auto& tProbabilities = aControl[mRandomVectorDim];

        ScalarType tTotalSum = 0;
        for(decltype(mRandomVectorDim) tDimIndex = 0; tDimIndex < mRandomVectorDim; tDimIndex++)
        {
            ScalarType tMomentSum = 0;
            const auto& tMySamples = aControl[tDimIndex];
            auto& tMyMomentError = mMomentsError->operator[](tDimIndex);
            auto& tMyMomentMisfit = mMomentsMisfit->operator[](tDimIndex);
            for(decltype(mMaxNumMoments) tMomentIndex = 0; tMomentIndex < mMaxNumMoments; tMomentIndex++)
            {
                auto tMomentOrder = tMomentIndex + static_cast<OrdinalType>(1);
                mSromMoments->operator[](tMomentIndex) =
                    Plato::compute_raw_moment<ScalarType, OrdinalType>(tMomentOrder, tMySamples, tProbabilities);
                tMyMomentMisfit[tMomentIndex] = mSromMoments->operator[](tMomentIndex) - mTruthMoments->operator[](tMomentIndex);
                auto tValue = tMyMomentMisfit[tMomentIndex] / mTruthMoments->operator[](tMomentIndex);
                tMyMomentError[tMomentIndex] = tValue * tValue;
                tMomentSum = tMomentSum + tMyMomentError[tMomentIndex];
            }
            tTotalSum = tTotalSum + tMomentSum;
        }
        return (tTotalSum);
    }

    /******************************************************************************//**
     * \brief Compute misfit in CDF, i.e. difference between target and SROM CDF.
     * \param [in] aControl sample/probability pairs
     *
     * NOTE:
     * Compute misfit in Cumulative Distribution Function (CDF) for i-th dimension (i.e.
     * i-th random vector dimension). Currently, the random vector dimension is always
     * set to one. Hence, random vector has size one and samples are not correlated.
    **********************************************************************************/
    ScalarType computeCumulativeDistributionFunctionMisfit(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(aControl.getNumVectors() >= static_cast<OrdinalType>(2));
        const auto& tProbabilities = aControl[mRandomVectorDim];

        ScalarType tTotalSum = 0;
        auto tNumSamples = this->getNumSamples();
        for(decltype(mRandomVectorDim) tDimIndex = 0; tDimIndex < mRandomVectorDim; tDimIndex++)
        {
            Plato::Vector<ScalarType, OrdinalType> & tMySromCDF = mSromCDF->operator[](tDimIndex);
            Plato::Vector<ScalarType, OrdinalType> & tMyTrueCDF = mTruthCDF->operator[](tDimIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMySamples = aControl[tDimIndex];

            ScalarType tMyRandomDimError = 0;
            for(decltype(tNumSamples) tSampleIndex = 0; tSampleIndex < tNumSamples; tSampleIndex++)
            {
                auto tSample_ij = tMySamples[tSampleIndex];
                tMyTrueCDF[tSampleIndex] = mDistribution->cdf(tSample_ij);
                tMySromCDF[tSampleIndex] =
                    Plato::compute_srom_cdf<ScalarType, OrdinalType>(tSample_ij, mSromSigma, tMySamples, tProbabilities);
                auto tMisfit = tMySromCDF[tSampleIndex] - tMyTrueCDF[tSampleIndex];
                tMyRandomDimError = tMyRandomDimError + (tMisfit * tMisfit);
            }
            tTotalSum = tTotalSum + tMyRandomDimError;
        }
        return (tTotalSum);
    }

private:
    OrdinalType mNumSamples;
    OrdinalType mMaxNumMoments;
    OrdinalType mRandomVectorDim;

    ScalarType mSromSigma;
    ScalarType mSqrtConstant;
    ScalarType mWeightCdfMisfit;
    ScalarType mWeightMomentMisfit;
    ScalarType mSromSigmaTimesSigma;
    ScalarType mWeightCorrelationMisfit;
    ScalarType mFrobeniusNormTruthCorrelationMatrix;
    ScalarType mCumulativeDistributionFunctionError;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mSromMoments;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mTruthMoments;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mSromCDF;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTruthCDF;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mMomentsError;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mMomentsMisfit;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mSromCorrelation;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTruthCorrelation;

    std::shared_ptr<Plato::Distribution<ScalarType, OrdinalType>> mDistribution;

private:
    SromObjective(const Plato::SromObjective<ScalarType, OrdinalType> & aRhs);
    Plato::SromObjective<ScalarType, OrdinalType> & operator=(const Plato::SromObjective<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_SROMOBJECTIVE_HPP_ */
