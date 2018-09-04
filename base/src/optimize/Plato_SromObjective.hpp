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
    explicit SromObjective(const std::shared_ptr<Plato::Distribution<ScalarType, OrdinalType>> & aDistribution,
                           const OrdinalType & aMaxNumMoments,
                           const OrdinalType & aNumSamples,
                           OrdinalType aNumRandomVecDim = 1) :
            mMaxNumMoments(aMaxNumMoments),
            mNumRandomVecDim(aNumRandomVecDim),
            mSromSigma(1e-3),
            mSqrtConstant(0),
            mWeightCdfMisfit(1/1e2),
            mWeightMomentMisfit(1/1e2),
            mSromSigmaTimesSigma(0),
            mCumulativeDistributionFunctionMisfit(0),
            mSromMoments(std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(mMaxNumMoments)),
            mTrueMoments(std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(mMaxNumMoments)),
            mSromCDF(std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(mNumRandomVecDim, aNumSamples)),
            mTrueCDF(std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(mNumRandomVecDim, aNumSamples)),
            mMomentsMisfit(std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(mNumRandomVecDim, mMaxNumMoments)),
            mDistribution(aDistribution)
    {
        this->setConstants();
        this->setTrueMoments();
    }
    virtual ~SromObjective()
    {
    }

    /******************************************************************************//**
     * @brief Set parameter used to control the degree of smoothness of the SROM CDF
     * @param [in] aInput parameter used to control the degree of smoothness
    **********************************************************************************/
    void setSromSigma(const ScalarType & aInput)
    {
        mSromSigma = aInput;
        this->setConstants(); // Reset constants due to new input
    }

    /******************************************************************************//**
     * @brief Set weights used to ensure CDF error component has similar order of magnitude
     * @param [in] aInput weight
    **********************************************************************************/
    void setCdfMisfitTermWeight(const ScalarType & aInput)
    {
        mWeightCdfMisfit = aInput;
    }

    /******************************************************************************//**
     * @brief Set weights used to ensure moment's error component has similar order of magnitude
     * @param [in] aInput weight
    **********************************************************************************/
    void setMomentMisfitTermWeight(const ScalarType & aInput)
    {
        mWeightMomentMisfit = aInput;
    }

    /******************************************************************************//**
     * @brief Return CDF misfit
     * @return cumulative distribution function (CDF) misfit
    **********************************************************************************/
    ScalarType getCumulativeDistributionFunctionMisfit() const
    {
        return (mCumulativeDistributionFunctionMisfit);
    }

    /******************************************************************************//**
     * @brief Return moment misfit terms for each random dimension
     * @return multi-vector with moment's misfit
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getMomentMisfit() const
    {
        return (*mMomentsMisfit);
    }


    /******************************************************************************//**
     * @brief Return moment misfit terms for a given random dimension
     * @param [in] aInput random vector dimension
     * @return vector with moment's misfit for input random dimension
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType>& getMomentMisfit(const OrdinalType& aInput) const
    {
        assert(aInput < mMomentsMisfit->getNumVectors());
        return (mMomentsMisfit->operator[](aInput));
    }

    /******************************************************************************//**
     * @brief Return true moments
     * @return vector with true moments in increasing order, i.e. first element
     *          corresponds to the first moment (i.e. mean) and the last element
     *          corresponds to the last moment
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType>& getTrueMoments() const
    {
        return (*mTrueMoments);
    }

    /******************************************************************************//**
     * @brief Return true moments
     * @return vector with SROM moments in increasing order, i.e. first element
     *          corresponds to the first SROM moment approximation (i.e. mean) and
     *          the last element corresponds to the last SROM moment approximation
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType>& getSromMoments() const
    {
        return (*mSromMoments);
    }

    /******************************************************************************//**
     * @brief Return true cumulative distribution function evaluations
     * @return vector with true cumulative distribution function evaluations at the
     *         sample inputs to the objective function (for all random dimensions).
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getTrueCDF() const
    {
        return (*mTrueCDF);
    }

    /******************************************************************************//**
     * @brief Return SROM cumulative distribution function evaluations
     * @return vector with SROM cumulative distribution function evaluations at the
     *         sample inputs to the objective function (for all random dimensions).
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getSromCDF() const
    {
        return (*mSromCDF);
    }

    /******************************************************************************//**
     * @brief Return true cumulative distribution function evaluations
     * @param [in] aInput random vector dimension
     * @return vector with true cumulative distribution function evaluations at the
     *         sample inputs to the objective function (for all random dimensions).
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType>& getTrueCDF(const OrdinalType& aInput) const
    {
        return (*mTrueCDF);
    }

    /******************************************************************************//**
     * @brief Return SROM cumulative distribution function evaluations
     * @param [in] aInput random vector dimension
     * @return vector with SROM cumulative distribution function evaluations at the
     *         sample inputs to the objective function (for all random dimensions).
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType>& getSromCDF(const OrdinalType& aInput) const
    {
        return (*mSromCDF);
    }

    /******************************************************************************//**
     * @brief Null operation - operation is not needed for SROM optimization problem
    **********************************************************************************/
    void cacheData()
    {
        return;
    }

    /******************************************************************************//**
     * @brief Evaluate SROM objective
     * @param [in] aControl optimization variables
     * @return criterion evaluation
    **********************************************************************************/
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        // NOTE: CORRELATION TERM IS NOT IMPLEMENTED YET. THIS TERM WILL BE ADDED IN THE FUTURE
        const ScalarType tMomentsMisfit = this->computeMomentsMisfit(aControl);
        mCumulativeDistributionFunctionMisfit = this->computeCumulativeDistributionFunctionMisfit(aControl);
        const ScalarType tOutput = static_cast<ScalarType>(0.5)
                * (mWeightCdfMisfit * mCumulativeDistributionFunctionMisfit + mWeightMomentMisfit * tMomentsMisfit);
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Evaluate SROM gradient
     * @param [in] aControl optimization variables
     * @param [in,out] aOutput SROM gradient
    **********************************************************************************/
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        // NOTE: CORRELATION TERM IS NOT IMPLEMENTED YET. THIS TERM WILL BE ADDED IN THE NEAR FUTURE
        assert(aControl.getNumVectors() == aOutput.getNumVectors());
        assert(aControl.getNumVectors() >= static_cast<OrdinalType>(2));

        const OrdinalType tNumRandomVecDim = aOutput.getNumVectors() - static_cast<OrdinalType>(1);
        assert(tNumRandomVecDim == mNumRandomVecDim);
        const Plato::Vector<ScalarType, OrdinalType> & tProbabilities = aControl[tNumRandomVecDim];
        Plato::Vector<ScalarType, OrdinalType> & tGradientProbabilities = aOutput[tNumRandomVecDim];

        const OrdinalType tNumProbabilities = tProbabilities.size();
        for(OrdinalType tIndexI = 0; tIndexI < tNumRandomVecDim; tIndexI++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMySamples = aControl[tIndexI];
            Plato::Vector<ScalarType, OrdinalType> & tMySamplesGradient = aOutput[tIndexI];
            Plato::Vector<ScalarType, OrdinalType> & tMyMomentMisfit = mMomentsMisfit->operator[](tIndexI);
            for(OrdinalType tIndexJ = 0; tIndexJ < tNumProbabilities; tIndexJ++)
            {
                // Samples' Gradient
                ScalarType tSample_ij = tMySamples[tIndexJ];
                ScalarType tProbability_ij = tProbabilities[tIndexJ];
                ScalarType tPartialCDFwrtSample =
                        this->partialCumulativeDistributionFunctionWrtSamples(tSample_ij, tProbability_ij, tMySamples, tProbabilities);
                ScalarType tPartialMomentWrtSample = this->partialMomentsWrtSamples(tSample_ij, tProbability_ij, tMyMomentMisfit);
                tMySamplesGradient[tIndexJ] = (mWeightCdfMisfit * tPartialCDFwrtSample) +
                        (mWeightMomentMisfit * tPartialMomentWrtSample);

                // Probabilities' Gradient
                ScalarType tPartialCDFwrtProbability =
                        this->partialCumulativeDistributionFunctionWrtProbabilities(tSample_ij, tMySamples, tProbabilities);
                ScalarType tPartialMomentWrtProbability = this->partialMomentsWrtProbabilities(tSample_ij, tMyMomentMisfit);
                tGradientProbabilities[tIndexJ] = tGradientProbabilities[tIndexJ] +
                        (mWeightCdfMisfit * tPartialCDFwrtProbability + mWeightMomentMisfit * tPartialMomentWrtProbability);
            }
        }
    }

    /******************************************************************************//**
     * @brief Apply vector to Hessian operator
     * @param [in] aControl optimization variables
     * @param [in] aVector descent direction
     * @param [in,out] aOutput application of vector to Hessian operator
    **********************************************************************************/
    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        Plato::update(static_cast<ScalarType>(1), aVector, static_cast<ScalarType>(0), aOutput);
    }

private:
    /******************************************************************************//**
     * @brief Set SROM constants
    **********************************************************************************/
    void setConstants()
    {
        mSromSigmaTimesSigma = mSromSigma * mSromSigma;
        mSqrtConstant = static_cast<ScalarType>(2) * static_cast<ScalarType>(M_PI) * mSromSigmaTimesSigma;
        mSqrtConstant = std::sqrt(mSqrtConstant);
    }

    /******************************************************************************//**
     * @brief Precompute true moments for target statistics.
    **********************************************************************************/
    void setTrueMoments()
    {
        for(OrdinalType tIndex = 0; tIndex < mMaxNumMoments; tIndex++)
        {
            OrdinalType tMyOrder = tIndex + static_cast<OrdinalType>(1);
            mTrueMoments->operator[](tIndex) = mDistribution->moment(tMyOrder);
        }
    }

    /******************************************************************************//**
     * @brief Compute partial derivative of moment term with respect to samples.
     * @param [in] aSampleIJ sample IJ
     * @param [in] aProbabilityIJ probability IJ
     * @param [in] aMomentsMisfit moment misfit term
    **********************************************************************************/
    ScalarType partialMomentsWrtSamples(const ScalarType & aSampleIJ,
                                        const ScalarType & aProbabilityIJ,
                                        const Plato::Vector<ScalarType, OrdinalType> & aMomentsMisfit)
    {
        assert(mMaxNumMoments == aMomentsMisfit.size());

        ScalarType tSum = 0;
        for(OrdinalType tIndexK = 0; tIndexK < mMaxNumMoments; tIndexK++)
        {
            ScalarType tTrueMomentTimesTrueMoment = mTrueMoments->operator[](tIndexK) * mTrueMoments->operator[](tIndexK);
            ScalarType tConstant = static_cast<ScalarType>(1) / tTrueMomentTimesTrueMoment;
            ScalarType tMomentOrder = tIndexK + static_cast<OrdinalType>(1);
            tSum = tSum + (tConstant * aMomentsMisfit[tIndexK] * tMomentOrder * aProbabilityIJ
                    * std::pow(aSampleIJ, static_cast<ScalarType>(tIndexK)));
        }
        return (tSum);
    }

    /******************************************************************************//**
     * @brief Compute partial derivative of moment term with respect to probabilities.
     * @param [in] aSampleIJ sample IJ
     * @param [in] aMomentsMisfit moment misfit term
    **********************************************************************************/
    ScalarType partialMomentsWrtProbabilities(const ScalarType & aSampleIJ,
                                              const Plato::Vector<ScalarType, OrdinalType> & aMomentsMisfit)
    {
        // Compute sensitivity in dimension k:
        ScalarType tSum = 0;
        const OrdinalType tNumMoments = aMomentsMisfit.size();
        for(OrdinalType tIndexK = 0; tIndexK < tNumMoments; tIndexK++)
        {
            ScalarType tTrueMomentTimesTrueMoment = mTrueMoments->operator[](tIndexK) * mTrueMoments->operator[](tIndexK);
            ScalarType tConstant = static_cast<ScalarType>(1) / tTrueMomentTimesTrueMoment;
            ScalarType tExponent = tIndexK + static_cast<OrdinalType>(1);
            tSum = tSum + (tConstant * aMomentsMisfit[tIndexK] * std::pow(aSampleIJ, tExponent));
        }
        return (tSum);
    }

    /******************************************************************************//**
     * @brief Compute partial derivative of CDF term with respect to samples.
     * @param [in] aSampleIJ sample IJ
     * @param [in] aProbabilityIJ probability IJ
     * @param [in] aSamples trial samples
     * @param [in] aSamples trial probabilities
    **********************************************************************************/
    ScalarType partialCumulativeDistributionFunctionWrtSamples(const ScalarType & aSampleIJ,
                                                               const ScalarType & aProbabilityIJ,
                                                               const Plato::Vector<ScalarType, OrdinalType> & aSamples,
                                                               const Plato::Vector<ScalarType, OrdinalType> & aProbabilities)
    {
        ScalarType tSum = 0;
        const OrdinalType tNumProbabilities = aProbabilities.size();
        for(OrdinalType tProbIndexK = 0; tProbIndexK < tNumProbabilities; tProbIndexK++)
        {
            ScalarType tSample_ik = aSamples[tProbIndexK];
            ScalarType tConstant = aProbabilities[tProbIndexK] / mSqrtConstant;
            ScalarType tExponent = (static_cast<ScalarType>(-1) / (static_cast<ScalarType>(2) * mSromSigmaTimesSigma))
                    * (aSampleIJ - tSample_ik) * (aSampleIJ - tSample_ik);
            tSum = tSum + (tConstant * std::exp(tExponent));
        }

        const ScalarType tTruePDF = mDistribution->pdf(aSampleIJ);
        const ScalarType tTrueCDF = mDistribution->cdf(aSampleIJ);
        const ScalarType tSromCDF = Plato::compute_srom_cdf<ScalarType, OrdinalType>(aSampleIJ, mSromSigma, aSamples, aProbabilities);
        const ScalarType tMisfitCDF = tSromCDF - tTrueCDF;
        const ScalarType tTermOne = tMisfitCDF * (tSum - tTruePDF);

        ScalarType tTermTwo = 0;
        for(OrdinalType tProbIndexL = 0; tProbIndexL < tNumProbabilities; tProbIndexL++)
        {
            ScalarType tSample_il= aSamples[tProbIndexL];
            ScalarType tMyTrueCDF = mDistribution->cdf(tSample_il);
            ScalarType tMySromCDF = Plato::compute_srom_cdf<ScalarType, OrdinalType>(tSample_il, mSromSigma, aSamples, aProbabilities);
            ScalarType tMyMisfitCDF = tMySromCDF - tMyTrueCDF;
            ScalarType tConstant = aProbabilityIJ / mSqrtConstant;
            ScalarType tExponent = (static_cast<ScalarType>(-1) / (static_cast<ScalarType>(2) * mSromSigmaTimesSigma))
                    * (tSample_il - aSampleIJ) * (tSample_il - aSampleIJ);
            tTermTwo = tTermTwo + (tMyMisfitCDF * tConstant * std::exp(tExponent));
        }

        const ScalarType tOutput = tTermOne - tTermTwo;
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Compute partial derivative of CDF term with respect to probabilities.
     * @param [in] aSampleIJ sample IJ
     * @param [in] aSamples trial samples
     * @param [in] aSamples trial probabilities
    **********************************************************************************/
    ScalarType partialCumulativeDistributionFunctionWrtProbabilities(const ScalarType & aSampleIJ,
                                                                     const Plato::Vector<ScalarType, OrdinalType> & aSamples,
                                                                     const Plato::Vector<ScalarType, OrdinalType> & aProbabilities)
    {
        ScalarType tSum = 0;
        const OrdinalType tNumProbabilities = aProbabilities.size();
        for(OrdinalType tProbIndexK = 0; tProbIndexK < tNumProbabilities; tProbIndexK++)
        {
            // Compute different in true/SROM CDFs at x_ij:
            ScalarType tTrueCDF = mDistribution->cdf(aSamples[tProbIndexK]);
            ScalarType tSromCDF =
                    Plato::compute_srom_cdf<ScalarType, OrdinalType>(aSamples[tProbIndexK], mSromSigma, aSamples, aProbabilities);
            ScalarType tMisfitCDF = tSromCDF - tTrueCDF;
            // Compute CDF derivative term:
            ScalarType tNumerator = aSamples[tProbIndexK] - aSampleIJ;
            ScalarType tDenominator = std::sqrt(static_cast<ScalarType>(2)) * mSromSigma;
            ScalarType tSensitivity = static_cast<ScalarType>(1) + std::erf(tNumerator / tDenominator);
            // Add contribution to summation in k:
            tSum = tSum + (tMisfitCDF * tSensitivity);
        }
        tSum = static_cast<ScalarType>(0.5) * tSum;
        return (tSum);
    }

    /******************************************************************************//**
     * @brief Compute misfit in moments, i.e. difference between target and SROM moments.
     * @param [in] aControl sample/probability pairs
     *
     * NOTE:
     * Compute misfit in moments up to order q for ith dimension (i.e. i-th random vector
     * dimension). Currently, the random vector dimension is always set to one. Hence,
     * random vector has size one and samples are not correlated.
    **********************************************************************************/
    ScalarType computeMomentsMisfit(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(mMaxNumMoments == mTrueMoments->size());
        assert(aControl.getNumVectors() >= static_cast<OrdinalType>(2));
        const OrdinalType tNumRandomVecDim = aControl.getNumVectors() - static_cast<OrdinalType>(1);
        assert(tNumRandomVecDim == mNumRandomVecDim);
        const Plato::Vector<ScalarType, OrdinalType> & tProbabilities = aControl[tNumRandomVecDim];

        ScalarType tTotalSum = 0;
        for(OrdinalType tDimIndex = 0; tDimIndex < tNumRandomVecDim; tDimIndex++)
        {
            ScalarType tMomentSum = 0;
            const Plato::Vector<ScalarType, OrdinalType> & tMySamples = aControl[tDimIndex];
            Plato::Vector<ScalarType, OrdinalType> & tMyMomentMisfit = mMomentsMisfit->operator[](tDimIndex);
            for(OrdinalType tMomentIndex = 0; tMomentIndex < mMaxNumMoments; tMomentIndex++)
            {
                OrdinalType tMomentOrder = tMomentIndex + static_cast<OrdinalType>(1);
                mSromMoments->operator[](tMomentIndex) =
                        Plato::compute_srom_moment<ScalarType, OrdinalType>(tMomentOrder, tMySamples, tProbabilities);
                tMyMomentMisfit[tMomentIndex] = mSromMoments->operator[](tMomentIndex) - mTrueMoments->operator[](tMomentIndex);
                ScalarType tValue = tMyMomentMisfit[tMomentIndex] / mTrueMoments->operator[](tMomentIndex);
                tMomentSum = tMomentSum + (tValue * tValue);
            }
            tTotalSum = tTotalSum + tMomentSum;
        }
        return (tTotalSum);
    }

    /******************************************************************************//**
     * @brief Compute misfit in CDF, i.e. difference between target and SROM CDF.
     * @param [in] aControl sample/probability pairs
     *
     * NOTE:
     * Compute misfit in Cumulative Distribution Function (CDF) for i-th dimension (i.e.
     * i-th random vector dimension). Currently, the random vector dimension is always
     * set to one. Hence, random vector has size one and samples are not correlated.
    **********************************************************************************/
    ScalarType computeCumulativeDistributionFunctionMisfit(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(aControl.getNumVectors() >= static_cast<OrdinalType>(2));
        const OrdinalType tNumRandomVecDim = aControl.getNumVectors() - static_cast<OrdinalType>(1);
        assert(tNumRandomVecDim == mNumRandomVecDim);
        const Plato::Vector<ScalarType, OrdinalType> & tProbabilities = aControl[tNumRandomVecDim];

        ScalarType tTotalSum = 0;
        const OrdinalType tNumProbabilities = tProbabilities.size();
        for(OrdinalType tDimIndex = 0; tDimIndex < tNumRandomVecDim; tDimIndex++)
        {
            Plato::Vector<ScalarType, OrdinalType> & tMySromCDF = mSromCDF->operator[](tDimIndex);
            Plato::Vector<ScalarType, OrdinalType> & tMyTrueCDF = mTrueCDF->operator[](tDimIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMySamples = aControl[tDimIndex];

            ScalarType tMySampleSum = 0;
            for(OrdinalType tProbIndex = 0; tProbIndex < tNumProbabilities; tProbIndex++)
            {
                ScalarType tSample_ij = tMySamples[tProbIndex];
                tMyTrueCDF[tProbIndex] = mDistribution->cdf(tSample_ij);
                tMySromCDF[tProbIndex] = Plato::compute_srom_cdf<ScalarType, OrdinalType>(tSample_ij, mSromSigma, tMySamples, tProbabilities);
                ScalarType tMisfit = tMySromCDF[tProbIndex] - tMyTrueCDF[tProbIndex];
                tMySampleSum = tMySampleSum + (tMisfit * tMisfit);
            }
            tTotalSum = tTotalSum + tMySampleSum;
        }
        return (tTotalSum);
    }

private:
    OrdinalType mMaxNumMoments;
    OrdinalType mNumRandomVecDim;

    ScalarType mSromSigma;
    ScalarType mSqrtConstant;
    ScalarType mWeightCdfMisfit;
    ScalarType mWeightMomentMisfit;
    ScalarType mSromSigmaTimesSigma;
    ScalarType mCumulativeDistributionFunctionMisfit;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mSromMoments;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mTrueMoments;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mSromCDF;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTrueCDF;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mMomentsMisfit;
    std::shared_ptr<Plato::Distribution<ScalarType, OrdinalType>> mDistribution;

private:
    SromObjective(const Plato::SromObjective<ScalarType, OrdinalType> & aRhs);
    Plato::SromObjective<ScalarType, OrdinalType> & operator=(const Plato::SromObjective<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_SROMOBJECTIVE_HPP_ */
