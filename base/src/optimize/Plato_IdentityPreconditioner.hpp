/*
 * Plato_IdentityPreconditioner.hpp
 *
 *  Created on: Oct 21, 2017
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

#ifndef PLATO_IDENTITYPRECONDITIONER_HPP_
#define PLATO_IDENTITYPRECONDITIONER_HPP_

#include "Plato_StateData.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_Preconditioner.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class IdentityPreconditioner : public Preconditioner<ScalarType, OrdinalType>
{
public:
    IdentityPreconditioner()
    {
    }
    virtual ~IdentityPreconditioner()
    {
    }
    void update(const Plato::StateData<ScalarType, OrdinalType> & aStateData)
    {
        return;
    }
    void applyPreconditioner(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                             const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                             Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aVector.getNumVectors() == aOutput.getNumVectors());
        Plato::update(1., aVector, 0., aOutput);
    }
    void applyInvPreconditioner(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                                Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aVector.getNumVectors() == aOutput.getNumVectors());
        Plato::update(1., aVector, 0., aOutput);
    }

private:
    IdentityPreconditioner(const Plato::IdentityPreconditioner<ScalarType, OrdinalType> & aRhs);
    Plato::IdentityPreconditioner<ScalarType, OrdinalType> & operator=(const Plato::IdentityPreconditioner<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_IDENTITYPRECONDITIONER_HPP_ */
