/*
 * Plato_DualProbElementWiseFunctions.hpp
 *
 *  Created on: May 7, 2018
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
