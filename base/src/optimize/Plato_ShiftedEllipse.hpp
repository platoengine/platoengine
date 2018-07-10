#pragma once

#include <cmath>
#include <memory>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"

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

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class ShiftedEllipse : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    ShiftedEllipse() :
            mXCenter(0.),
            mXRadius(1.),
            mYCenter(0.),
            mYRadius(1.)
    {
    }
    virtual ~ShiftedEllipse()
    {
    }
    void specify(ScalarType aXCenter, ScalarType aXRadius, ScalarType aYCenter, ScalarType aYRadius)
    {
        mXCenter = aXCenter;
        mXRadius = aXRadius;
        mYCenter = aYCenter;
        mYRadius = aYRadius;
    }

    void cacheData()
    {
        return;
    }

    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(aControl.getNumVectors() > static_cast<OrdinalType>(0));

        const OrdinalType tVectorIndex = 0;
        ScalarType tOutput = ((aControl(tVectorIndex, 0) - mXCenter) * (aControl(tVectorIndex, 0) - mXCenter) / (mXRadius * mXRadius))
                             + ((aControl(tVectorIndex, 1) - mYCenter) * (aControl(tVectorIndex, 1) - mYCenter) / (mYRadius * mYRadius))
                             - 1.;
        return (tOutput);
    }
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aOutput.getNumVectors() > static_cast<OrdinalType>(0));
        assert(aControl.getNumVectors() > static_cast<OrdinalType>(0));
        assert(aControl.getNumVectors() == aOutput.getNumVectors());

        const OrdinalType tVectorIndex = 0;
        aOutput(tVectorIndex, 0) = (2. / (mXRadius * mXRadius)) * (aControl(tVectorIndex, 0) - mXCenter);
        aOutput(tVectorIndex, 1) = (2. / (mYRadius * mYRadius)) * (aControl(tVectorIndex, 1) - mYCenter);

    }
    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aOutput.getNumVectors() > static_cast<OrdinalType>(0));
        assert(aVector.getNumVectors() > static_cast<OrdinalType>(0));
        assert(aVector.getNumVectors() == aOutput.getNumVectors());

        const OrdinalType tVectorIndex = 0;
        aOutput(tVectorIndex, 0) = (2. / (mXRadius * mXRadius)) * aVector(tVectorIndex, 0);
        aOutput(tVectorIndex, 1) = (2. / (mYRadius * mYRadius)) * aVector(tVectorIndex, 1);
    }

private:
    ScalarType mXCenter;
    ScalarType mXRadius;
    ScalarType mYCenter;
    ScalarType mYRadius;

private:
    ShiftedEllipse(const Plato::ShiftedEllipse<ScalarType, OrdinalType> & aRhs);
    Plato::ShiftedEllipse<ScalarType, OrdinalType> & operator=(const Plato::ShiftedEllipse<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato
