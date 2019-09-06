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

/*
 * Plato_Types.hpp
 *
 *  Created on: Oct 21, 2017
 */

#ifndef Plato_TYPES_HPP_
#define Plato_TYPES_HPP_

#include <string>

namespace Plato
{

struct algorithm
{
    enum stop_t
    {
        NaN_NORM_TRIAL_STEP = 1,
        NaN_NORM_GRADIENT = 2,
        NORM_GRADIENT = 3,
        NORM_STEP = 4,
        OBJECTIVE_STAGNATION = 5,
        MAX_NUMBER_ITERATIONS = 6,
        OPTIMALITY_AND_FEASIBILITY = 7,
        ACTUAL_REDUCTION_TOLERANCE = 8,
        CONTROL_STAGNATION = 9,
        STATIONARITY_MEASURE = 10,
        SMALL_TRUST_REGION_RADIUS = 11,
        SMALL_PENALTY_PARAMETER = 12,
        NaN_OBJECTIVE_GRADIENT = 13,
        NaN_OBJECTIVE_STAGNATION = 14,
        NaN_CONTROL_STAGNATION = 15,
        NaN_STATIONARITY_MEASURE = 16,
        NaN_ACTUAL_REDUCTION = 17,
        NOT_CONVERGED = 18
    };
};

/******************************************************************************//**
 * @brief Output a brief sentence explaining why the optimizer stopped.
 * @param [in] aStopCriterion stopping criterion flag
 * @param [in,out] aOutput string with brief description
**********************************************************************************/
inline void get_stop_criterion(const Plato::algorithm::stop_t & aStopCriterion, std::string & aOutput)
{
    aOutput.clear();
    switch(aStopCriterion)
    {
        case Plato::algorithm::stop_t::STATIONARITY_MEASURE:
        {
            aOutput = "\n\n****** Optimization stopping due to stationary measure being met. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::ACTUAL_REDUCTION_TOLERANCE:
        {
            aOutput = "\n\n****** Optimization stopping due to actual reduction tolerance being met. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::OBJECTIVE_STAGNATION:
        {
            aOutput = "\n\n****** Optimization stopping due to objective stagnation. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::SMALL_TRUST_REGION_RADIUS:
        {
            aOutput = "\n\n****** Optimization stopping due to small trust region radius. ******\n\n";
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
        case Plato::algorithm::stop_t::NaN_NORM_TRIAL_STEP:
        {
            aOutput = "\n\n****** Optimization stopping due to NaN norm of trial step vector. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NaN_NORM_GRADIENT:
        {
            aOutput = "\n\n****** Optimization stopping due to NaN norm of gradient vector. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NORM_GRADIENT:
        {
            aOutput = "\n\n****** Optimization stopping due to norm of gradient tolerance being met. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NORM_STEP:
        {
            aOutput = "\n\n****** Optimization stopping due to norm of trial step tolerance being met. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::OPTIMALITY_AND_FEASIBILITY:
        {
            aOutput = "\n\n****** Optimization stopping due to optimality and feasibility tolerance being met. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::SMALL_PENALTY_PARAMETER:
        {
            aOutput = "\n\n****** Optimization stopping due to small penalty parameter in augmented Lagrangian being met. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NaN_OBJECTIVE_GRADIENT:
        {
            aOutput = "\n\n****** Optimization stopping due to NaN norm of objective gradient vector. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NaN_OBJECTIVE_STAGNATION:
        {
            aOutput = "\n\n****** Optimization stopping due to NaN objective stagnation. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NaN_CONTROL_STAGNATION:
        {
            aOutput = "\n\n****** Optimization stopping due to NaN control stagnation. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NaN_STATIONARITY_MEASURE:
        {
            aOutput = "\n\n****** Optimization stopping due to NaN stationary measure. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NaN_ACTUAL_REDUCTION:
        {
            aOutput = "\n\n****** Optimization stopping due to NaN actual reduction. ******\n\n";
            break;
        }
        case Plato::algorithm::stop_t::NOT_CONVERGED:
        {
            aOutput = "\n\n****** Optimization algorithm did not converge. ******\n\n";
            break;
        }
    }
}

} // namespace Plato

#endif /* Plato_TYPES_HPP_ */
