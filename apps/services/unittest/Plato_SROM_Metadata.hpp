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
 * Plato_SROM_Metadata.hpp
 *
 *  Created on: June 18, 2019
 */

#ifndef PLATO_SROM_METADATA_HPP_
#define PLATO_SROM_METADATA_HPP_

#include <string>
#include <vector>
#include "../../base/src/tools/Plato_Vector3DVariations.hpp"

namespace Plato
{

namespace srom
{

/******************************************************************************//**
 * @brief Statistics metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct Statistics
{
    std::string mNumSamples; /*!< number of samples */
    std::string mDistribution; /*!< probability distribution */
    std::string mMean; /*!< probability distribution mean */
    std::string mUpperBound; /*!< probability distribution upper bound */
    std::string mLowerBound; /*!< probability distribution lower bound */
    std::string mStandardDeviation; /*!< probability distribution standard deviation */
};

/******************************************************************************//**
 * @brief Variable metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct Variable
{
    std::string mType; /*!< random variable type, e.g. random rotation */
    std::string mSubType; /*!< random variable subtype, e.g. rotation axis */
    Plato::srom::Statistics mStatistics; /*!< statistics for this random variable */
};

/******************************************************************************//**
 * @brief Load metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct Load
{
    int mAppID; /*!< application set identifier */
    std::string mLoadID; /*!< global load identifier */
    std::string mAppType; /*!< application type, e.g. sideset, nodeset, etc. */
    std::string mLoadType; /*!< load type, e.g. pressure, traction, etc. */
    std::vector<std::string> mValues; /*!< load magnitudes, e.g. \f$F = \{F_x, F_y, F_z\}\f$. */
    std::vector<Plato::srom::Variable> mRandomVars; /*!< set of random variables, e.g. rotations, \f$\theta = \{\theta_x, \theta_y, \theta_z\}\f$. */
};

/******************************************************************************//**
 * @brief Sample-Probability pairs defined by the Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct SampleProbabilityPairs
{
    int mNumSamples; /*!< total number of samples */
    std::vector<double> mSamples; /*!< sample set */
    std::vector<double> mProbabilities; /*!< probability set  */
};

/******************************************************************************//**
 * @brief Random vriable metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct RandomVariable
{
    std::string mType; /*!< random variable type, e.g. random rotation */
    std::string mSubType; /*!< random variable subtype, e.g. rotation axis */
    Plato::srom::SampleProbabilityPairs mSampleProbPairs; /*!< sample-probability pair for this random variable */
};

/******************************************************************************//**
 * @brief Random rotations metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct RandomRotations
{
    double mProbability; /*!< probability associated with this random rotation */
    Plato::Vector3D mRotations; /*!< vector of random rotations, e.g. /f$(\theta_x, \theta_y, \theta_z)/f$ */
};

/******************************************************************************//**
 * @brief Random load metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct RandomLoad
{
    int mAppID; /*!< application set identifier */
    int mLoadID; /*!< load identifier */
    double mProbability; /*!< probability associated with this random load */
    std::string mAppType; /*!< application type, e.g. sideset, nodeset, etc. */
    std::string mLoadType; /*!< load type, e.g. pressure, traction, etc. */
    Plato::Vector3D mLoadValues; /*!< load components, e.g. /f$(f_x, f_y, f_z)/f$ */
};

/******************************************************************************//**
 * @brief Random load case metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct RandomLoadCase
{
    int mLoadCaseID; /*!< random load case global identifier */
    double mProbability; /*!< probability associated with this random load case */
    std::vector<Plato::srom::RandomLoad> mLoads; /*!< set of random loads associated with this random load case */
};

/******************************************************************************//**
 * @brief Output metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct OutputMetaData
{
    std::vector<Plato::srom::RandomLoadCase> mLoadCases; /*!< set of random load cases */
};

/******************************************************************************//**
 * @brief Input metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct InputMetaData
{
    std::vector<Plato::srom::Load> mLoads; /*!< set of loads */
};

}
// namespace srom

}
// namespace Plato

#endif /* PLATO_SROM_METADATA_HPP_ */
