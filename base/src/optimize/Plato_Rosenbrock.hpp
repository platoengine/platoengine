/*
 * Plato_Rosenbrock.hpp
 *
 *  Created on: Oct 21, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_ROSENBROCK_HPP_
#define PLATO_ROSENBROCK_HPP_

#include <cmath>
#include <memory>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class Rosenbrock : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    Rosenbrock() :
        mDivisor(1.)
    {
    }
    virtual ~Rosenbrock()
    {
    }

    void cacheData()
    {
        return;
    }
    void setDivisor(ScalarType aDivisor) {
        mDivisor = aDivisor;
    }
    /*!
     * Evaluate Rosenbrock function:
     *      f(\mathbf{x}) = 100 * \left(x_2 - x_1^2\right)^2 + \left(1 - x_1\right)^2
     * */
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(aControl.getNumVectors() == static_cast<OrdinalType>(1));

        const OrdinalType tVectorIndex = 0;
        const Plato::Vector<ScalarType, OrdinalType> & tMyControl = aControl[tVectorIndex];

        ScalarType tOutput = static_cast<ScalarType>(100.)
                * std::pow((tMyControl[1] - tMyControl[0] * tMyControl[0]), static_cast<ScalarType>(2))
                + std::pow(static_cast<ScalarType>(1) - tMyControl[0], static_cast<ScalarType>(2));

        tOutput /= mDivisor;
        return (tOutput);
    }
    /*!
     * Compute Rosenbrock gradient:
     *      \frac{\partial{f}}{\partial x_1} = -400 * \left(x_2 - x_1^2\right) * x_1 +
     *                                          \left(2 * \left(1 - x_1\right) \right)
     *      \frac{\partial{f}}{\partial x_2} = 200 * \left(x_2 - x_1^2\right)
     * */
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aOutput.getNumVectors() == static_cast<OrdinalType>(1));
        assert(aControl.getNumVectors() == static_cast<OrdinalType>(1));

        const OrdinalType tVectorIndex = 0;
        Plato::Vector<ScalarType, OrdinalType> & tMyOutput = aOutput[tVectorIndex];
        const Plato::Vector<ScalarType, OrdinalType> & tMyControl = aControl[tVectorIndex];

        tMyOutput[0] = static_cast<ScalarType>(-400) * (tMyControl[1] - (tMyControl[0] * tMyControl[0])) * tMyControl[0]
                + static_cast<ScalarType>(2) * tMyControl[0] - static_cast<ScalarType>(2);
        tMyOutput[1] = static_cast<ScalarType>(200) * (tMyControl[1] - (tMyControl[0] * tMyControl[0]));

        tMyOutput[0] /= mDivisor;
        tMyOutput[1] /= mDivisor;
    }
    /*!
     * Compute Rosenbrock Hessian times vector:
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

        tMyOutput[0] = ((static_cast<ScalarType>(2)
                - static_cast<ScalarType>(400) * (tMyControl[1] - (tMyControl[0] * tMyControl[0]))
                + static_cast<ScalarType>(800) * (tMyControl[0] * tMyControl[0])) * tMyVector[0])
                - (static_cast<ScalarType>(400) * tMyControl[0] * tMyVector[1]);
        tMyOutput[1] = (static_cast<ScalarType>(-400) * tMyControl[0] * tMyVector[0])
                + (static_cast<ScalarType>(200) * tMyVector[1]);

        tMyOutput[0] /= mDivisor;
        tMyOutput[1] /= mDivisor;
    }

private:
    ScalarType mDivisor;

private:
    Rosenbrock(const Plato::Rosenbrock<ScalarType, OrdinalType> & aRhs);
    Plato::Rosenbrock<ScalarType, OrdinalType> & operator=(const Plato::Rosenbrock<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_ROSENBROCK_HPP_ */
