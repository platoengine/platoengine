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
 * Plato_GradFreeCircle.hpp
 *
 *  Created on: Jan 23, 2019
 */

#pragma once

#include <cmath>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_GradFreeCriterion.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Interface to gradient free Circle criterion
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class GradFreeCircle : public Plato::GradFreeCriterion<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
    **********************************************************************************/
    explicit GradFreeCircle()
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~GradFreeCircle()
    {
    }

    /******************************************************************************//**
     * @brief Evaluate gradient free criterion
     * @param [in] aControl 2D container of control variables (i.e. optimization variables)
     * @param [out] aOutput 1D container of criterion values
     **********************************************************************************/
    void value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
               Plato::Vector<ScalarType, OrdinalType> & aOutput)
    {
        const OrdinalType tNumParticles = aControl.getNumVectors();
        assert(tNumParticles > static_cast<OrdinalType>(0));
        assert(aOutput.size() == tNumParticles);
        for(OrdinalType tIndex = 0; tIndex < tNumParticles; tIndex++)
        {
            aOutput[tIndex] = this->evaluate(aControl[tIndex]);
        }
    }

private:
    /******************************************************************************//**
     * @brief Evaluate Circle function:
     * \f$ \left( x - 1 \right)^2 + 2\left( y - 2 \right)^2 \f$
     * @param [in] aControl 1D container of optimization variables
     * @return function evaluation
     **********************************************************************************/
    ScalarType evaluate(const Plato::Vector<ScalarType, OrdinalType> & aControl)
    {
        assert(aControl.size() > static_cast<OrdinalType>(0));

        const ScalarType tAlpha = aControl[0] - static_cast<ScalarType>(0.75);
        ScalarType tBeta = aControl[1] - static_cast<ScalarType>(0.6);
        tBeta = static_cast<ScalarType>(2.) * std::pow(tBeta, static_cast<ScalarType>(2));
        const ScalarType tOutput = std::pow(tAlpha, static_cast<ScalarType>(2)) + tBeta;

        return (tOutput);
    }

private:
    GradFreeCircle(const Plato::GradFreeCircle<ScalarType, OrdinalType> & aRhs);
    Plato::GradFreeCircle<ScalarType, OrdinalType> & operator=(const Plato::GradFreeCircle<ScalarType, OrdinalType> & aRhs);
};
// class GradFreeCircle

} // namespace Plato
