/*
 * Plato_CcsaTestInequality.hpp
 *
 *  Created on: Nov 4, 2017
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

#pragma once

#include <cmath>
#include <vector>
#include <memory>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Inequality constraint from: Svanberg, Krister. "The method of moving
 * asymptotes: A new method for structural optimization." International journal
 * for numerical methods in engineering 24.2 (1987): 359-373.
 *
 * The constraint is given by:
 *
 * /f$ \frac{61}{x_1^3} + \frac{37}{x_2^3} + \frac{19}{x_3^3} + \frac{7}{x_4^3}
 * + \frac{1}{x_5^3} \leq C_2 /f$, where /f$C_2 = 1/f$
 *
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class CcsaTestInequality : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
    **********************************************************************************/
    CcsaTestInequality() :
            mConstraintLimit(1)
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~CcsaTestInequality()
    {
    }

    /******************************************************************************//**
     * @brief Cache application-based data
    **********************************************************************************/
    void cacheData()
    {
        return;
    }

    /******************************************************************************//**
     * @brief Evaluate objective function
     * @param [in] aControl 2D container of optimization variables
    **********************************************************************************/
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(aControl.getNumVectors() > static_cast<OrdinalType>(0));

        const OrdinalType tVectorIndex = 0;
        const Plato::Vector<ScalarType, OrdinalType> & tMyControl = aControl[tVectorIndex];

        ScalarType tTermOne = static_cast<ScalarType>(61) / std::pow(tMyControl[0], static_cast<ScalarType>(3));
        ScalarType tTermTwo = static_cast<ScalarType>(37) / std::pow(tMyControl[1], static_cast<ScalarType>(3));
        ScalarType tTermThree = static_cast<ScalarType>(19) / std::pow(tMyControl[2], static_cast<ScalarType>(3));
        ScalarType tTermFour = static_cast<ScalarType>(7) / std::pow(tMyControl[3], static_cast<ScalarType>(3));
        ScalarType tTermFive = static_cast<ScalarType>(1) / std::pow(tMyControl[4], static_cast<ScalarType>(3));
        ScalarType tResidual = tTermOne + tTermTwo + tTermThree + tTermFour + tTermFive;
        tResidual = tResidual - mConstraintLimit;

        return (tResidual);
    }
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aControl.getNumVectors() > static_cast<OrdinalType>(0));

        const OrdinalType tVectorIndex = 0;
        Plato::Vector<ScalarType, OrdinalType> & tMyGradient = aOutput[tVectorIndex];
        const Plato::Vector<ScalarType, OrdinalType> & tMyControl = aControl[tVectorIndex];

        ScalarType tScaleFactor = -3;
        tMyGradient[0] = tScaleFactor
                * (static_cast<ScalarType>(61) / std::pow(tMyControl[0], static_cast<ScalarType>(4)));
        tMyGradient[1] = tScaleFactor
                * (static_cast<ScalarType>(37) / std::pow(tMyControl[1], static_cast<ScalarType>(4)));
        tMyGradient[2] = tScaleFactor
                * (static_cast<ScalarType>(19) / std::pow(tMyControl[2], static_cast<ScalarType>(4)));
        tMyGradient[3] = tScaleFactor
                * (static_cast<ScalarType>(7) / std::pow(tMyControl[3], static_cast<ScalarType>(4)));
        tMyGradient[4] = tScaleFactor
                * (static_cast<ScalarType>(1) / std::pow(tMyControl[4], static_cast<ScalarType>(4)));
    }

    /******************************************************************************//**
     * @brief Apply vector to Hessian operator
     * @param [in] aControl 2D container of optimization variables
     * @param [in] aVector 2D container of descent directions
     * @param [in/out] aOutput 2D container with the application of the input vector to
     *   the Hessian
    **********************************************************************************/
    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        const ScalarType tScalarValue = 0;
        const OrdinalType tVectorIndex = 0;
        aOutput[tVectorIndex].fill(tScalarValue);
    }

private:
    ScalarType mConstraintLimit; /*!< constraint upper bound, i.e. limit */

private:
    CcsaTestInequality(const Plato::CcsaTestInequality<ScalarType, OrdinalType> & aRhs);
    Plato::CcsaTestInequality<ScalarType, OrdinalType> & operator=(const Plato::CcsaTestInequality<ScalarType, OrdinalType> & aRhs);
};
// namespace CcsaTestInequality

}
// namespace Plato
