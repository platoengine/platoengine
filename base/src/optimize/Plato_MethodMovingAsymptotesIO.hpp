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
 * Plato_MethodMovingAsymptotesIO.hpp
 *
 *  Created on: Sep 1, 2019
 */

#pragma once

#include <string>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "Plato_Types.hpp"
#include "Plato_Macros.hpp"
#include "Plato_OptimizersIO_Utilities.hpp"
#include "Plato_MethodMovingAsymptotesIO_Data.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Check inputs for MMA algorithm diagnostics
 * @param [in] aData diagnostic data for MMA algorithm
 * @param [in] aOutputFile output file
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType, typename OutputType>
void check_mma_inputs(const Plato::OutputDataMMA<ScalarType, OrdinalType> &aData,
                      const OutputType &aOutputFile)
{
    try
    {
        Plato::error::is_file_open(aOutputFile);
        Plato::error::is_vector_empty(aData.mConstraints);
    }
    catch(const std::invalid_argument& tError)
    {
        throw tError;
    }
}
// function check_mma_inputs

/******************************************************************************//**
 * @brief Output a brief sentence explaining why the CCSA optimizer stopped.
 * @param [in] aStopCriterion stopping criterion flag
 * @param [in,out] aOutput string with brief description
**********************************************************************************/
inline void print_mma_stop_criterion(const Plato::algorithm::stop_t & aStopCriterion, std::string & aOutput)
{
    aOutput.clear();
    switch(aStopCriterion)
    {
        case Plato::algorithm::stop_t::OBJECTIVE_STAGNATION:
        {
            aOutput = "\n\n****** Optimization stopping due to objective stagnation. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::CONTROL_STAGNATION:
        {
            aOutput = "\n\n****** Optimization stopping due to control (i.e. design variable) stagnation. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::MAX_NUMBER_ITERATIONS:
        {
            aOutput = "\n\n****** Optimization stopping due to exceeding maximum number of iterations. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::OPTIMALITY_AND_FEASIBILITY:
        {
            aOutput = "\n\n****** Optimization stopping due to optimality and feasibility tolerance being met. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NOT_CONVERGED:
        {
            aOutput = "\n\n****** Optimization algorithm did not converge. ******\n\n";
            break;
        }
        default:
        {
            aOutput = "\n\n****** ERROR: Optimization algorithm stopping due to undefined behavior. ******\n\n";
            break;
        }
    }
}
// function print_mma_stop_criterion

/******************************************************************************//**
 * @brief Print header for MMA diagnostics file
 * @param [in] aData diagnostic data for mma algorithm
 * @param [in,out] aOutputFile output file
**********************************************************************************/
template<typename ScalarType, typename OrdinalType, typename OutputType>
void print_mma_diagnostics_header(const Plato::OutputDataMMA<ScalarType, OrdinalType> &aData,
                                  OutputType &aOutputFile)
{
    try
    {
        Plato::check_mma_inputs(aData, aOutputFile);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        THROWERR(tErrorMsg.what())
    }

    if(aData.mConstraints.size() <= static_cast<OrdinalType>(0))
    {
        THROWERR("CONTAINER WITH CONSTRAINT VALUES IS EMPTY.\n")
    }

    aOutputFile << std::scientific << std::setprecision(6) << std::right << "Iter" << std::setw(10) << "F-count"
            << std::setw(14) << "F(X)" << std::setw(16) << "Norm(F')" << std::setw(10);

    const OrdinalType tNumConstraints = aData.mConstraints.size();
    for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
    {
        if(tIndex != static_cast<OrdinalType>(0))
        {
            aOutputFile << "H" << tIndex + static_cast<OrdinalType>(1) << "(X)" << std::setw(13);
        }
        else
        {
            const OrdinalType tWidth = tNumConstraints > static_cast<OrdinalType>(1) ? 11 : 13;
            aOutputFile << "H" << tIndex + static_cast<OrdinalType>(1) << "(X)" << std::setw(tWidth);
        }
    }

    aOutputFile << std::setw(15) << "abs(dX)" << std::setw(15) << "abs(dF)" << "\n" << std::flush;
}
// function print_mma_diagnostics_header

/******************************************************************************//**
 * @brief Print diagnostics for MMA algorithm
 * @param [in] aData diagnostic data for mma algorithm
 * @param [in,out] aOutputFile output file
**********************************************************************************/
template<typename ScalarType, typename OrdinalType, typename OutputType>
void print_mma_diagnostics(const Plato::OutputDataMMA<ScalarType, OrdinalType> &aData,
                           OutputType &aOutputFile)
{
    try
    {
        Plato::check_mma_inputs(aData, aOutputFile);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        THROWERR(tErrorMsg.what())
    }
    assert(aData.mConstraints.size() > static_cast<OrdinalType>(0));

    aOutputFile << std::scientific << std::setprecision(6) << std::right << aData.mNumIter << std::setw(10)
            << aData.mObjFuncCount << std::setw(20) << aData.mObjFuncValue << std::setw(15) << aData.mNormObjFuncGrad
            << std::setw(15);

    const OrdinalType tNumConstraints = aData.mConstraints.size();
    for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
    {
        aOutputFile << aData.mConstraints[tIndex] << std::setw(15);
    }

    aOutputFile << aData.mControlStagnationMeasure << std::setw(15) << aData.mObjectiveStagnationMeasure << "\n" << std::flush;
}
// function print_mma_diagnostics

}
// namespace Plato
