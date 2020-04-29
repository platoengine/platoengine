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
 * Plato_RandomLoadMetadata.hpp
 *
 *  Created on: Apr 28, 2020
 */

#pragma once

#include "Plato_SromMetadata.hpp"

namespace Plato
{

namespace srom
{

/******************************************************************************//**
 * \struct Load
 * \brief Load metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct Load
{
    int mAppID; /*!< application, i.e. side/node, set identification number */
    std::string mAppName; /*!< application, i.e. side/node, set identification number */
    std::string mLoadID; /*!< global load identification number */
    std::string mAppType; /*!< application type, e.g. side/node set, etc. */
    std::string mLoadType; /*!< load type, e.g. pressure, traction, etc. */
    std::vector<std::string> mValues; /*!< load magnitudes, e.g. \f$F = \{F_x, F_y, F_z\}\f$. */
    std::vector<Plato::srom::RandomVariable> mRandomVars; /*!< set of random variables, e.g. rotations, \f$\theta = \{\theta_x, \theta_y, \theta_z\}\f$. */
};
// struct Load

/******************************************************************************//**
 * \struct RandomRotations
 * \brief Random rotations metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct RandomRotations
{
    double mProbability; /*!< probability associated with this angle variation */
    std::vector<double> mRotations; /*!< vector of angle variations, e.g. /f$(\theta_x, \theta_y, \theta_z)/f$ */
};
// struct RandomRotations

/******************************************************************************//**
 * \struct RandomLoad
 * \brief Random load metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct RandomLoad
{
    int mAppID; /*!< application set identification number */
    int mLoadID; /*!< load identification number */
    double mProbability; /*!< probability associated with this random load */
    std::string mAppType; /*!< application type, e.g. sideset, nodeset, etc. */
    std::string mAppName; /*!< application name. */
    std::string mLoadType; /*!< load type, e.g. pressure, traction, etc. */
    std::vector<double> mLoadValues; /*!< load components, e.g. /f$(f_x, f_y, f_z)/f$ */
};
// struct RandomLoad

/******************************************************************************//**
 * \struct RandomLoadCase
 * \brief Random load case metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct RandomLoadCase
{
    int mLoadCaseID; /*!< random load case global identification number */
    double mProbability; /*!< probability associated with this random load case */
    std::vector<Plato::srom::RandomLoad> mLoads; /*!< set of random loads associated with this random load case */
};
// struct RandomLoadCase

}
// namespace srom

}
// namespace Plato
