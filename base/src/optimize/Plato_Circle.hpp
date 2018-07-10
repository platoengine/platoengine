/*
 * Plato_Circle.hpp
 *
 *  Created on: Oct 21, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_CIRCLE_HPP_
#define PLATO_CIRCLE_HPP_

#include <cmath>
#include <memory>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class Circle : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    Circle()
    {
    }
    virtual ~Circle()
    {
    }

    void cacheData()
    {
        return;
    }
    /// \left(\mathbf{z}(0) - 1.\right)^2 + 2\left(\mathbf{z}(1) - 2\right)^2
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(aControl.getNumVectors() > static_cast<OrdinalType>(0));

        const OrdinalType tVectorIndex = 0;
        ScalarType tAlpha = aControl(tVectorIndex, 0) - static_cast<ScalarType>(1.);
        ScalarType tBeta = aControl(tVectorIndex, 1) - static_cast<ScalarType>(2);
        tBeta = static_cast<ScalarType>(2.) * std::pow(tBeta, static_cast<ScalarType>(2));
        ScalarType tOutput = std::pow(tAlpha, static_cast<ScalarType>(2)) + tBeta;
        return (tOutput);
    }
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aOutput.getNumVectors() > static_cast<OrdinalType>(0));
        assert(aControl.getNumVectors() > static_cast<OrdinalType>(0));
        assert(aControl.getNumVectors() == aOutput.getNumVectors());

        const OrdinalType tVectorIndex = 0;
        aOutput(tVectorIndex, 0) =
                static_cast<ScalarType>(2.) * (aControl(tVectorIndex, 0) - static_cast<ScalarType>(1.));
        aOutput(tVectorIndex, 1) =
                static_cast<ScalarType>(4.) * (aControl(tVectorIndex, 1) - static_cast<ScalarType>(2.));

    }
    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aOutput.getNumVectors() > static_cast<OrdinalType>(0));
        assert(aVector.getNumVectors() > static_cast<OrdinalType>(0));
        assert(aVector.getNumVectors() == aOutput.getNumVectors());

        const OrdinalType tVectorIndex = 0;
        aOutput(tVectorIndex, 0) = static_cast<ScalarType>(2.) * aVector(tVectorIndex, 0);
        aOutput(tVectorIndex, 1) = static_cast<ScalarType>(4.) * aVector(tVectorIndex, 1);
    }

private:
    Circle(const Plato::Circle<ScalarType, OrdinalType> & aRhs);
    Plato::Circle<ScalarType, OrdinalType> & operator=(const Plato::Circle<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_CIRCLE_HPP_ */
