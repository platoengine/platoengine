/*
 * Plato_Radius.hpp
 *
 *  Created on: Oct 21, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_RADIUS_HPP_
#define PLATO_RADIUS_HPP_

#include <cmath>
#include <memory>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class Radius : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    Radius() :
            mLimit(1)
    {
    }
    virtual ~Radius()
    {
    }
    void setLimit(ScalarType aLimit) {
        mLimit = aLimit;
    }

    void cacheData()
    {
        return;
    }
    /// \left(\mathbf{z}(0)\right)^2 + \left(\mathbf{z}(1)\right)^2 - limit
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(aControl.getNumVectors() > static_cast<OrdinalType>(0));

        const OrdinalType tVectorIndex = 0;
        ScalarType tOutput = std::pow(aControl(tVectorIndex, 0), static_cast<ScalarType>(2.)) +
                std::pow(aControl(tVectorIndex, 1), static_cast<ScalarType>(2.));
        tOutput = tOutput - mLimit;
        return (tOutput);
    }
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aOutput.getNumVectors() > static_cast<OrdinalType>(0));
        assert(aControl.getNumVectors() > static_cast<OrdinalType>(0));
        assert(aControl.getNumVectors() == aOutput.getNumVectors());

        const OrdinalType tVectorIndex = 0;
        aOutput(tVectorIndex, 0) = static_cast<ScalarType>(2.) * aControl(tVectorIndex, 0);
        aOutput(tVectorIndex, 1) = static_cast<ScalarType>(2.) * aControl(tVectorIndex, 1);

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
        aOutput(tVectorIndex, 1) = static_cast<ScalarType>(2.) * aVector(tVectorIndex, 1);
    }

private:
    ScalarType mLimit;

private:
    Radius(const Plato::Radius<ScalarType, OrdinalType> & aRhs);
    Plato::Radius<ScalarType, OrdinalType> & operator=(const Plato::Radius<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_RADIUS_HPP_ */
