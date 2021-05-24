/*
 * Plato_OptimizerInterface.hpp
 *
 *  Created on: Oct 30, 2017
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
//
// *************************************************************************
//@HEADER
*/

#ifndef PLATO_OPTIMIZERINTERFACE_HPP_
#define PLATO_OPTIMIZERINTERFACE_HPP_

namespace Plato
{

struct optimizer
{

    enum algorithm_t
    {
        OPTIMALITY_CRITERIA = 1,
        METHOD_OF_MOVING_ASYMPTOTES = 2,
        GLOBALLY_CONVERGENT_METHOD_OF_MOVING_ASYMPTOTES = 3,
        KELLEY_SACHS_UNCONSTRAINED = 4,
        KELLEY_SACHS_BOUND_CONSTRAINED = 5,
        KELLEY_SACHS_AUGMENTED_LAGRANGIAN = 6,
        DERIVATIVE_CHECKER = 7,
        ROL_KSAL = 8,
        ROL_KSBC = 9,
        STOCHASTIC_REDUCED_ORDER_MODEL = 10,
        PARTICLE_SWARM_OPTMIZATION_ALPSO = 11,
        PARTICLE_SWARM_OPTMIZATION_BCPSO = 12,
        SO_PARAMETER_STUDIES = 13,
        ROL_LINEAR_CONSTRAINT = 14,
        ROL_BOUND_CONSTRAINED = 15,
        ROL_AUGMENTED_LAGRANGIAN = 16
    }; // enum algorithm_t

};
// struct optimizer

/******************************************************************************//**
 * @brief Abstract interface to optimization algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class OptimizerInterface
{
public:
    virtual ~OptimizerInterface()
    {
    }

    /******************************************************************************//**
     * @brief Interface to optimization algorithm - solves optimization problem
    **********************************************************************************/
    virtual void optimize() = 0;

    /******************************************************************************//**
     * @brief Deallocate memory
    **********************************************************************************/
    virtual void finalize() = 0;

    /******************************************************************************//**
     * @brief Allocate memory
    **********************************************************************************/
    virtual void initialize() = 0;
};
// class OptimizerInterface

} // namespace Plato

#endif /* PLATO_OPTIMIZERINTERFACE_HPP_ */
