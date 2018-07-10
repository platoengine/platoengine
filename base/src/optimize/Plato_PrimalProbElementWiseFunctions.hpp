/*
 * Plato_PrimalProbElementWiseFunctions.hpp
 *
 *  Created on: May 8, 2018
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
