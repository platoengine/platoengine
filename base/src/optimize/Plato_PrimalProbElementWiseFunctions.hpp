/*
 * Plato_PrimalProbElementWiseFunctions.hpp
 *
 *  Created on: May 8, 2018
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

#ifndef PLATO_PRIMALPROBELEMENTWISEFUNCTIONS_HPP_
#define PLATO_PRIMALPROBELEMENTWISEFUNCTIONS_HPP_

#include <map>

#include "Plato_Vector.hpp"

namespace Plato
{

struct element_wise
{
    enum constant_t
    {
        ExpansionFactor = 0,
        ContractionFactor = 1,
        LowerBoundScaleFactor = 2,
        UpperBoundScaleFactor = 3,
    };
};

template<typename ScalarType, typename OrdinalType = size_t>
class PrimalProbElementWiseFunctions
{
public:
    virtual ~PrimalProbElementWiseFunctions()
    {
    }

    virtual void updateInitialSigmaCoeff(const ScalarType & aInitialMovingAsymptoteMultiplier,
                                         const Plato::Vector<ScalarType, OrdinalType> & aUpperBounds,
                                         const Plato::Vector<ScalarType, OrdinalType> & aLowerBounds,
                                         Plato::Vector<ScalarType, OrdinalType> & aCurrentSigma) const = 0;

    virtual void updateSigmaCoeff(const std::map<Plato::element_wise::constant_t, ScalarType> & aConstants,
                                  const Plato::Vector<ScalarType, OrdinalType> & aCurrentControls,
                                  const Plato::Vector<ScalarType, OrdinalType> & aPreviousControls,
                                  const Plato::Vector<ScalarType, OrdinalType> & aAntepenultimateControls,
                                  const Plato::Vector<ScalarType, OrdinalType> & aUpperBounds,
                                  const Plato::Vector<ScalarType, OrdinalType> & aLowerBounds,
                                  const Plato::Vector<ScalarType, OrdinalType> & aPreviousSigma,
                                  Plato::Vector<ScalarType, OrdinalType> & aCurrentSigma) const = 0;

    virtual void updateObjectiveGlobalizationCoeff(const Plato::Vector<ScalarType, OrdinalType> & aDeltaControl,
                                                   const Plato::Vector<ScalarType, OrdinalType> & aCurrentSigma,
                                                   const Plato::Vector<ScalarType, OrdinalType> & aCurrentGradient,
                                                   Plato::Vector<ScalarType, OrdinalType> & aOutputOne,
                                                   Plato::Vector<ScalarType, OrdinalType> & aOutputTwo) const = 0;

    virtual void updateConstraintGlobalizationCoeff(const Plato::Vector<ScalarType, OrdinalType> & aDeltaControl,
                                                    const Plato::Vector<ScalarType, OrdinalType> & aCurrentSigma,
                                                    const Plato::Vector<ScalarType, OrdinalType> & aCurrentGradient,
                                                    Plato::Vector<ScalarType, OrdinalType> & aOutputOne,
                                                    Plato::Vector<ScalarType, OrdinalType> & aOutputTwo) const = 0;

    virtual void computeKarushKuhnTuckerObjectiveConditions(const Plato::Vector<ScalarType, OrdinalType> & aControl,
                                                            const Plato::Vector<ScalarType, OrdinalType> & aObjectiveGradient,
                                                            const Plato::Vector<ScalarType, OrdinalType> & aConstraintGradientTimesDual,
                                                            Plato::Vector<ScalarType, OrdinalType> & aOutputOne,
                                                            Plato::Vector<ScalarType, OrdinalType> & aOutputTwo) const = 0;
};
// class PrimalProbElementWiseCCSA

} // namespace Plato

#endif /* PLATO_PRIMALPROBELEMENTWISEFUNCTIONS_HPP_ */
