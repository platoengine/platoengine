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
#include <memory>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class Himmelblau : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    Himmelblau()
    {
    }
    virtual ~Himmelblau()
    {
    }

    void cacheData()
    {
        return;
    }
    /*
     * Evaluate Himmelblau function:
     *      f(x,y) = (x^2+y-11)^2+(y^2+x-7)^2
     * */
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(aControl.getNumVectors() == static_cast<OrdinalType>(1));

        const OrdinalType tVectorIndex = 0;
        const Plato::Vector<ScalarType, OrdinalType> & tMyControl = aControl[tVectorIndex];

        ScalarType tFirstTerm = (tMyControl[0] * tMyControl[0]) + tMyControl[1] - 11.;
        ScalarType tSecondTerm = tMyControl[0] + (tMyControl[1] * tMyControl[1]) - 7.;
        ScalarType tOutput = tFirstTerm * tFirstTerm + tSecondTerm * tSecondTerm;
        return (tOutput);
    }
    /*
     * Compute Himmelblau gradient:
     *      f_x = 4*x^3+(4*y-42)*x+2*y^2-14
     *      f_y = 4*y^3+(4*x-26)*y+2*x^2-22
     * */
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aOutput.getNumVectors() == static_cast<OrdinalType>(1));
        assert(aControl.getNumVectors() == static_cast<OrdinalType>(1));

        const OrdinalType tVectorIndex = 0;
        Plato::Vector<ScalarType, OrdinalType> & tMyOutput = aOutput[tVectorIndex];
        const Plato::Vector<ScalarType, OrdinalType> & tMyControl = aControl[tVectorIndex];

        tMyOutput[0] = 4 * tMyControl[0] * tMyControl[0] * tMyControl[0] + (4 * tMyControl[1] - 42) * tMyControl[0]
                       + 2 * tMyControl[1] * tMyControl[1] - 14;
        tMyOutput[1] = 4 * tMyControl[1] * tMyControl[1] * tMyControl[1] + (4 * tMyControl[0] - 26) * tMyControl[1]
                       + 2 * tMyControl[0] * tMyControl[0] - 22;
    }
    /*
     * Compute Himmelblau Hessian times vector:
     *          f_xx = 12*x^2+4*y-42
     *          f_xy = 4*x+4*y
     *          f_yy = 12*y^2+4*x-26
     * */
    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aOutput.getNumVectors() == static_cast<OrdinalType>(1));
        assert(aVector.getNumVectors() == static_cast<OrdinalType>(1));
        assert(aControl.getNumVectors() == static_cast<OrdinalType>(1));

        const OrdinalType tVectorIndex = 0;
        Plato::Vector<ScalarType, OrdinalType> & tMyOutput = aOutput[tVectorIndex];
        const Plato::Vector<ScalarType, OrdinalType> & tMyVector = aVector[tVectorIndex];
        const Plato::Vector<ScalarType, OrdinalType> & tMyControl = aControl[tVectorIndex];

        ScalarType fxx = 12 * tMyControl[0] * tMyControl[0] + 4 * tMyControl[1] - 42;
        ScalarType fxy = 4 * tMyControl[0] + 4 * tMyControl[1];
        ScalarType fyy = 12 * tMyControl[1] * tMyControl[1] + 4 * tMyControl[0] - 26;
        tMyOutput[0] = fxx * tMyVector[0] + fxy * tMyVector[1];
        tMyOutput[1] = fxy * tMyVector[0] + fyy * tMyVector[1];
    }

private:
    Himmelblau(const Plato::Himmelblau<ScalarType, OrdinalType> & aRhs);
    Plato::Himmelblau<ScalarType, OrdinalType> & operator=(const Plato::Himmelblau<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato
