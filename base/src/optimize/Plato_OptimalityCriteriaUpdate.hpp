/*
 * Plato_OptimalityCriteriaUpdate.hpp
 *
 *  Created on: May 6, 2018
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

#ifndef PLATO_OPTIMALITYCRITERIAUPDATE_HPP_
#define PLATO_OPTIMALITYCRITERIAUPDATE_HPP_

#include <memory>

#include "Plato_Vector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class OptimalityCriteriaUpdate
{
public:
    virtual ~OptimalityCriteriaUpdate(){}

    virtual void reset(const ScalarType & aMoveLimit,
                       const ScalarType & aScaleFactor,
                       const ScalarType & aDampingPower) = 0;

    virtual void update(const ScalarType & aTrialDual,
                        const Plato::Vector<ScalarType, OrdinalType> & aLowerBounds,
                        const Plato::Vector<ScalarType, OrdinalType> & aUpperBounds,
                        const Plato::Vector<ScalarType, OrdinalType> & aPreviousControls,
                        const Plato::Vector<ScalarType, OrdinalType> & aObjectiveGradient,
                        const Plato::Vector<ScalarType, OrdinalType> & aInequalityGradient,
                        Plato::Vector<ScalarType, OrdinalType> & aTrialControls) = 0;

    virtual std::shared_ptr<Plato::OptimalityCriteriaUpdate<ScalarType, OrdinalType>> create() const = 0;
};
// class OptimalityCriteriaUpdate

} // namespace Plato

#endif /* PLATO_OPTIMALITYCRITERIAUPDATE_HPP_ */
