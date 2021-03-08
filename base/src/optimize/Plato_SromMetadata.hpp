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
 * Plato_SromMetadata.hpp
 *
 *  Created on: June 18, 2019
 */

#pragma once

#include <string>
#include <vector>
#include <sstream>

#include "Plato_Macros.hpp"

namespace Plato
{

namespace srom
{

/******************************************************************************//**
 * \struct Statistics
 * \brief Statistics metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct Statistics
{
    std::string mFilename;                  /*!< filename: contains sample-probability pairs */
    std::string mNumSamples;                /*!< number of samples */
    std::string mDistribution;              /*!< probability distribution */
    std::string mMean;                      /*!< probability distribution mean */
    std::string mUpperBound;                /*!< probability distribution upper bound */
    std::string mLowerBound;                /*!< probability distribution lower bound */
    std::string mDimensions = "1";          /*!< random vector dimensions */
    std::string mRandomSeed = "2";          /*!< random seed used to initialize samples (apply only in random initial guess use cases) */
    std::string mInitialGuess = "uniform";  /*!< method used to compute samples' initial guess, options: random, uniform */
    std::string mStandardDeviation;         /*!< probability distribution standard deviation */
    std::string mCorrelationFilename;       /*!< correlation filename: contains input correlation matrix */

    /******************************************************************************//**
     * \fn check
     * \brief Check if primary statistics have been defined.  The standard deviation \n
     * may not be defined for certain distribution families, e.g. uniform.
    **********************************************************************************/
    void check() const
    {
        if(mNumSamples.empty())
        {
            THROWERR("Statistics: Number of samples is not defined.")
        }

        if(mDistribution.empty())
        {
            THROWERR("Statistics: Distribution is not defined.")
        }

        if(mMean.empty())
        {
            THROWERR("Statistics: Mean is not defined.")
        }

        if(mUpperBound.empty())
        {
            THROWERR("Statistics: Upper bound is not defined.")
        }

        if(mLowerBound.empty())
        {
            THROWERR("Statistics: Lower bound is not defined.")
        }
    }
};
// struct Statistics

/******************************************************************************//**
 * \struct RandomVariable
 * \brief Random variable metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct RandomVariable
{
private:
    int mID = 0;                         /*!< random variable identification number */
    std::string mTag;                    /*!< label, e.g. angle variation */
    std::string mAttribute;              /*!< random variable attribute, e.g. rotation axis */
    Plato::srom::Statistics mStatistics; /*!< statistics for this random variable */

public:
    /******************************************************************************//**
     * \fn check
     * \brief Verify if random variable metadata has been defined.
    **********************************************************************************/
    void check() const
    {
        if(mTag.empty())
            { THROWERR("Random Variable: Random variable tag is not defined.") }

        if(mAttribute.empty())
            { THROWERR("Random Variable: Random variable attribute is not defined.") }

        if(mStatistics.mFilename.empty())
        {
            try
            {
                mStatistics.check();
            }
            catch(std::exception& tException)
            {
                std::ostringstream tMsg;
                tMsg << "Random Variable: Metadata of random variable with tag '" << mTag
                    << "' and attribute '" << mAttribute << "' is not defined. \n"
                    << "Exception message caught: " << tException.what();
                THROWERR(tMsg.str().c_str())
            }
        }
    }

    /******************************************************************************//**
     * \fn id
     * \brief Set a random variable identification number.
     * \param [in] aID identification number
    **********************************************************************************/
    void id(const int& aID)
    {
        mID = aID;
    }

    /******************************************************************************//**
     * \fn tag
     * \brief Set a random variable tag.
     * \param [in] aTag random variable tag/label
    **********************************************************************************/
    void tag(const std::string& aTag)
    {
        mTag = aTag;
    }

    /******************************************************************************//**
     * \fn filename
     * \brief Set name of file with sample-probability pair information.
     * \param [in] aFile filename
    **********************************************************************************/
    void filename(const std::string& aFile)
    {
        mStatistics.mFilename = aFile;
    }

    /******************************************************************************//**
     * \fn attribute
     * \brief Set random variable attribute.
     * \param [in] aAttribute random variable attribute
    **********************************************************************************/
    void attribute(const std::string& aAttribute)
    {
        mAttribute = aAttribute;
    }

    /******************************************************************************//**
     * \fn samples
     * \brief Set number of random variable samples.
     * \param [in] aNumSamples number of random variable samples
    **********************************************************************************/
    void samples(const std::string& aNumSamples)
    {
        mStatistics.mNumSamples = aNumSamples;
    }

    /******************************************************************************//**
     * \fn guess
     * \brief Set method used to compute samples vector initial guess.
     * \param [in] aMethod method used to compute samples vector initial guess
    **********************************************************************************/
    void guess(const std::string& aMethod)
    {
        mStatistics.mInitialGuess = aMethod;
    }

    /******************************************************************************//**
     * \fn distribution
     * \brief Set random variable distribution.
     * \param [in] aDistribution random variable distribution
    **********************************************************************************/
    void distribution(const std::string& aDistribution)
    {
        mStatistics.mDistribution = aDistribution;
    }

    /******************************************************************************//**
     * \fn correlationFilename
     * \brief Set correlation matrix filename
     * \param [in] aFilename filename
    **********************************************************************************/
    void correlationFilename(const std::string& aFilename)
    {
        mStatistics.mCorrelationFilename = aFilename;
    }

    /******************************************************************************//**
     * \fn seed
     * \brief Set random seed.
     * \param [in] aRandomSeed random seed
    **********************************************************************************/
    void seed(const std::string& aRandomSeed)
    {
        mStatistics.mRandomSeed = aRandomSeed;
    }

    /******************************************************************************//**
     * \fn dimensions
     * \brief Set random vector dimensions.
     * \param [in] aRandomSeed random vector dimensions
    **********************************************************************************/
    void dimensions(const std::string& aDimensions)
    {
        mStatistics.mDimensions = aDimensions;
    }

    /******************************************************************************//**
     * \fn mean
     * \brief Set random variable mean.
     * \param [in] aDistribution random variable mean
    **********************************************************************************/
    void mean(const std::string& aMean)
    {
        mStatistics.mMean = aMean;
    }

    /******************************************************************************//**
     * \fn lower
     * \brief Set random variable lower bound.
     * \param [in] aDistribution random variable lower bound
    **********************************************************************************/
    void lower(const std::string& aLower)
    {
        mStatistics.mLowerBound = aLower;
    }

    /******************************************************************************//**
     * \fn upper
     * \brief Set random variable upper bound.
     * \param [in] aDistribution random variable upper bound
    **********************************************************************************/
    void upper(const std::string& aUpper)
    {
        mStatistics.mUpperBound = aUpper;
    }

    /******************************************************************************//**
     * \fn deviation
     * \brief Set random variable standard deviation.
     * \param [in] aDistribution random variable standard deviation
    **********************************************************************************/
    void deviation(const std::string& aStandardDeviation)
    {
        mStatistics.mStandardDeviation = aStandardDeviation;
    }

    /******************************************************************************//**
     * \fn file
     * \brief Set random variable statistics.
     * \param [in] aStats random variable statistics
    **********************************************************************************/
    void statistics(const Plato::srom::Statistics& aStats)
    {
        mStatistics = aStats;
    }

    /******************************************************************************//**
     * \fn id
     * \brief Return random variable identification number.
     * \return identification number
    **********************************************************************************/
    int id() const
    {
        return mID;
    }

    /******************************************************************************//**
     * \fn tag
     * \brief Return random variable tag.
     * \return tag
    **********************************************************************************/
    std::string tag() const
    {
        return mTag;
    }

    /******************************************************************************//**
     * \fn filename
     * \brief Return name of file with sample-probability pair information.
     * \return filename
    **********************************************************************************/
    std::string filename() const
    {
        return mStatistics.mFilename;
    }

    /******************************************************************************//**
     * \fn attribute
     * \brief Return random variable attribute.
     * \return filename
    **********************************************************************************/
    std::string attribute() const
    {
        return mAttribute;
    }

    /******************************************************************************//**
     * \fn seed
     * \brief Return random seed.
     * \return random seed
    **********************************************************************************/
    std::string seed() const
    {
        return mStatistics.mRandomSeed;
    }

    /******************************************************************************//**
     * \fn dimensions
     * \brief Return random vector dimensions.
     * \return random vector dimensions
    **********************************************************************************/
    std::string dimensions() const
    {
        return mStatistics.mDimensions;
    }

    /******************************************************************************//**
     * \fn mean
     * \brief Return random variable mean.
     * \return mean
    **********************************************************************************/
    std::string mean() const
    {
        return mStatistics.mMean;
    }

    /******************************************************************************//**
     * \fn samples
     * \brief Return number of random variable samples.
     * \return number of samples
    **********************************************************************************/
    std::string samples() const
    {
        return mStatistics.mNumSamples;
    }

    /******************************************************************************//**
     * \fn guess
     * \brief Return method used to compute the initial guess for the vector of samples.
     * \return method's name
    **********************************************************************************/
    std::string guess() const
    {
        return mStatistics.mInitialGuess;
    }

    /******************************************************************************//**
     * \fn lower
     * \brief Return random variable lower bound.
     * \return lower bound
    **********************************************************************************/
    std::string lower() const
    {
        return mStatistics.mLowerBound;
    }

    /******************************************************************************//**
     * \fn upper
     * \brief Return random variable upper bound.
     * \return upper bound
    **********************************************************************************/
    std::string upper() const
    {
        return mStatistics.mUpperBound;
    }

    /******************************************************************************//**
     * \fn deviation
     * \brief Return random variable standard deviation.
     * \return standard deviation
    **********************************************************************************/
    std::string deviation() const
    {
        return mStatistics.mStandardDeviation;
    }

    /******************************************************************************//**
     * \fn distribution
     * \brief Return random variable distribution.
     * \return distribution
    **********************************************************************************/
    std::string distribution() const
    {
        return mStatistics.mDistribution;
    }

    /******************************************************************************//**
     * \fn correlationFilename
     * \brief Return name of file that contains input correlation matrix.
     * \return filename
    **********************************************************************************/
    std::string correlationFilename() const
    {
        return mStatistics.mCorrelationFilename;
    }

    /******************************************************************************//**
     * \fn define
     * \brief Define a random variable.
     * \param [in] aTag       random variable tag/label
     * \param [in] aAttribute random variable attribute
     * \param [in] aStats     random variable statistics
    **********************************************************************************/
    void define(const std::string& aTag, const std::string& aAttribute, const Plato::srom::Statistics& aStats)
    {
        mTag = aTag;
        mStatistics = aStats;
        mAttribute = aAttribute;
    }
};
// struct RandomVariable

/******************************************************************************//**
 * \struct DeterministicVariable
 * \brief Metadata used to describe a deterministic variable.
**********************************************************************************/
struct DeterministicVariable
{
private:
    std::string mTag;        /*!< main variable attribute , e.g. Poisson's ratio */
    std::string mValue;      /*!< attribute's deterministic value */
    std::string mAttribute;  /*!< attribute's category, e.g. homogeneous or heterogeneous */

public:
    /******************************************************************************//**
     * \fn tag
     * \brief Return deterministic variable tag.
     * \return tag
    **********************************************************************************/
    std::string tag() const
    {
        return mTag;
    }

    /******************************************************************************//**
     * \fn tag
     * \brief Set a deterministic variable tag.
     * \param [in] aTag deterministic variable tag/label
    **********************************************************************************/
    void tag(const std::string& aTag)
    {
        mTag = aTag;
    }

    /******************************************************************************//**
     * \fn value
     * \brief Return deterministic variable value.
     * \return value
    **********************************************************************************/
    std::string value() const
    {
        return mValue;
    }

    /******************************************************************************//**
     * \fn value
     * \brief Set a deterministic variable value.
     * \param [in] aValue deterministic variable value
    **********************************************************************************/
    void value(const std::string& aValue)
    {
        mValue = aValue;
    }

    /******************************************************************************//**
     * \fn attribute
     * \brief Return deterministic variable attribute.
     * \return attribute
    **********************************************************************************/
    std::string attribute() const
    {
        return mAttribute;
    }

    /******************************************************************************//**
     * \fn attribute
     * \brief Set a deterministic variable attribute.
     * \param [in] aAttribute deterministic variable attribute
    **********************************************************************************/
    void attribute(const std::string& aAttribute)
    {
        mAttribute = aAttribute;
    }
};
// struct DeterministicVariable

/******************************************************************************//**
 * \struct SampleProbabilityPairs
 * \brief Sample-Probability pairs defined by the Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct SampleProbabilityPairs
{
    int mNumSamples; /*!< total number of samples */
    std::vector<double> mSamples; /*!< sample set */
    std::vector<double> mProbabilities; /*!< probability set  */
};
// struct SampleProbabilityPairs

/******************************************************************************//**
 * \struct SromVariable
 * \brief Stochastic Reduced Order Model (SROM) variable metadata for SROM problem.
**********************************************************************************/
struct SromVariable
{
    std::string mTag;        /*!< label, e.g. angle variation, elastic modulus */
    std::string mAttribute;  /*!< SROM variable attribute, e.g. rotation axis, homogeneous */
    Plato::srom::SampleProbabilityPairs mSampleProbPairs; /*!< sample-probability pair for this random variable */
};
// struct SromVariable

}
// namespace srom

}
// namespace Plato
