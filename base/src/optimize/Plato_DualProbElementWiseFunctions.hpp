/*
 * Plato_DualProbElementWiseFunctions.hpp
 *
 *  Created on: May 7, 2018
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

#ifndef PLATO_DUALPROBELEMENTWISEFUNCTIONS_HPP_
#define PLATO_DUALPROBELEMENTWISEFUNCTIONS_HPP_

#include "Plato_Vector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class DualProbElementWiseFunctions
{
public:
    virtual ~DualProbElementWiseFunctions()
    {
    }

    virtual void updateTrialControl(const Plato::Vector<ScalarType, OrdinalType> & aTermA,
                                    const Plato::Vector<ScalarType, OrdinalType> & aTermB,
                                    const Plato::Vector<ScalarType, OrdinalType> & aLowerBounds,
                                    const Plato::Vector<ScalarType, OrdinalType> & aUpperBounds,
                                    const Plato::Vector<ScalarType, OrdinalType> & aLowerAsymptotes,
                                    const Plato::Vector<ScalarType, OrdinalType> & aUpperAsymptotes,
                                    Plato::Vector<ScalarType, OrdinalType> & aOutput) const  = 0;

    virtual void updateGradientCoeff(const Plato::Vector<ScalarType, OrdinalType> & aTrialControls,
                                     const Plato::Vector<ScalarType, OrdinalType> & aLowerAsymptotes,
                                     const Plato::Vector<ScalarType, OrdinalType> & aUpperAsymptotes,
                                     const Plato::Vector<ScalarType, OrdinalType> & aPcoeff,
                                     const Plato::Vector<ScalarType, OrdinalType> & aQcoeff,
                                     Plato::Vector<ScalarType, OrdinalType> & aOutputOne,
                                     Plato::Vector<ScalarType, OrdinalType> & aOutputTwo) const  = 0;

    virtual void updateObjectiveCoeff(const ScalarType & aGlobalizationFactor,
                                      const Plato::Vector<ScalarType, OrdinalType> & aCurrentSigma,
                                      const Plato::Vector<ScalarType, OrdinalType> & aCurrentGradient,
                                      Plato::Vector<ScalarType, OrdinalType> & aPcoeff,
                                      Plato::Vector<ScalarType, OrdinalType> & tQcoeff,
                                      Plato::Vector<ScalarType, OrdinalType> & aRcoeff) const = 0;

    virtual void updateConstraintCoeff(const ScalarType & aGlobalizationFactor,
                                       const Plato::Vector<ScalarType, OrdinalType> & aCurrentSigma,
                                       const Plato::Vector<ScalarType, OrdinalType> & aCurrentGradient,
                                       Plato::Vector<ScalarType, OrdinalType> & aPcoeff,
                                       Plato::Vector<ScalarType, OrdinalType> & tQcoeff,
                                       Plato::Vector<ScalarType, OrdinalType> & aRcoeff) const  = 0;

    virtual void updateMovingAsymptotesCoeff(const Plato::Vector<ScalarType, OrdinalType> & aTrialControl,
                                             const Plato::Vector<ScalarType, OrdinalType> & aLowerAsymptotes,
                                             const Plato::Vector<ScalarType, OrdinalType> & aUpperAsymptotes,
                                             const Plato::Vector<ScalarType, OrdinalType> & aObjectivePcoeff,
                                             const Plato::Vector<ScalarType, OrdinalType> & aObjectiveQcoeff,
                                             const Plato::Vector<ScalarType, OrdinalType> & aDualTimesConstraintPcoeff,
                                             const Plato::Vector<ScalarType, OrdinalType> & aDualTimesConstraintQcoeff,
                                             Plato::Vector<ScalarType, OrdinalType> & aOutputOne,
                                             Plato::Vector<ScalarType, OrdinalType> & aOutputTwo) const  = 0;
};
// class DualProbElementWiseCCSA

}; // namespace Plato

#endif /* PLATO_DUALPROBELEMENTWISEFUNCTIONS_HPP_ */
