/*
 * Plato_SromObjective.hpp
 *
 *  Created on: Jan 31, 2018
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
                           OrdinalType aNumRandomVecDim = 1) :
            mMaxNumMoments(aMaxNumMoments),
            mNumRandomVecDim(aNumRandomVecDim),
            mSromSigma(1e-3),
            mSqrtConstant(0),
            mWeightCdfMisfit(1/1e2),
            mWeightMomentMisfit(1/1e2),
            mSromSigmaTimesSigma(0),
            mTrueMoments(),
            mMomentsMisfit(),
            mDistribution(aDistribution)
    {
        this->setConstants();
        this->setTrueMoments();
    }
    virtual ~SromObjective()
    {
    }

    void setSromSigma(const ScalarType & aInput)
    {
        mSromSigma = aInput;
        this->setConstants(); // Reset constants due to new user-defined input
    }
    void setCdfMisfitTermWeight(const ScalarType & aInput)
    {
        mWeightCdfMisfit = aInput;
    }
    void setMomentMisfitTermWeight(const ScalarType & aInput)
    {
        mWeightMomentMisfit = aInput;
    }

    void cacheData()
    {
        return;
    }

    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        // NOTE: CORRELATION TERM IS NOT IMPLEMENTED YET. THIS TERM WILL BE ADDED IN THE NEAR FUTURE
        const ScalarType tMomentsMisfit = this->computeMomentsMisfit(aControl);
        const ScalarType tCummulativeDistributionFunctionMisfit =
                this->computeCumulativeDistributionFunctionMisfit(aControl);
        const ScalarType tOutput = static_cast<ScalarType>(0.5)
                * (mWeightCdfMisfit * tCummulativeDistributionFunctionMisfit + mWeightMomentMisfit * tMomentsMisfit);
        return (tOutput);
    }

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

    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        Plato::update(static_cast<ScalarType>(1), aVector, static_cast<ScalarType>(0), aOutput);
    }

private:
    void setConstants()
    {
        mSromSigmaTimesSigma = mSromSigma * mSromSigma;
        mSqrtConstant = static_cast<ScalarType>(2) * static_cast<ScalarType>(M_PI) * mSromSigmaTimesSigma;
        mSqrtConstant = std::sqrt(mSqrtConstant);
    }
    void setTrueMoments()
    {
        mTrueMoments = std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(mMaxNumMoments);
        for(OrdinalType tIndex = 0; tIndex < mMaxNumMoments; tIndex++)
        {
            OrdinalType tMyOrder = tIndex + static_cast<OrdinalType>(1);
            mTrueMoments->operator[](tIndex) = mDistribution->moment(tMyOrder);
        }
        mMomentsMisfit = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(mNumRandomVecDim, mMaxNumMoments);
    }
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
    /*!
     * Compute misfit in moments up to order q for ith dimension (i.e. i-th random vector dimension).
     * Currently, the random vector dimension is always set to one. Hence, random vector has size one
     * and samples are not correlated.
     **/
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
                ScalarType tSromMoment =
                        Plato::compute_srom_moment<ScalarType, OrdinalType>(tMomentOrder, tMySamples, tProbabilities);
                tMyMomentMisfit[tMomentIndex] = tSromMoment - mTrueMoments->operator[](tMomentIndex);
                ScalarType tValue = tMyMomentMisfit[tMomentIndex] / mTrueMoments->operator[](tMomentIndex);
                tMomentSum = tMomentSum + (tValue * tValue);
            }
            tTotalSum = tTotalSum + tMomentSum;
        }
        return (tTotalSum);
    }
    /*!
     * Compute misfit in Cumulative Distribution Function (CDF) for i-th dimension (i.e. i-th random vector
     * dimension). Currently, the random vector dimension is always set to one. Hence, random vector has size
     * one and samples are not correlated.
     **/
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
            ScalarType tMySampleSum = 0;
            const Plato::Vector<ScalarType, OrdinalType> & tMySamples = aControl[tDimIndex];
            for(OrdinalType tProbIndex = 0; tProbIndex < tNumProbabilities; tProbIndex++)
            {
                ScalarType tSample_ij = tMySamples[tProbIndex];
                ScalarType tTrueCDF = mDistribution->cdf(tSample_ij);
                ScalarType tSromCDF =
                        Plato::compute_srom_cdf<ScalarType, OrdinalType>(tSample_ij, mSromSigma, tMySamples, tProbabilities);
                ScalarType tMisfit = tSromCDF - tTrueCDF;
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

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mTrueMoments;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mMomentsMisfit;
    std::shared_ptr<Plato::Distribution<ScalarType, OrdinalType>> mDistribution;

private:
    SromObjective(const Plato::SromObjective<ScalarType, OrdinalType> & aRhs);
    Plato::SromObjective<ScalarType, OrdinalType> & operator=(const Plato::SromObjective<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_SROMOBJECTIVE_HPP_ */
