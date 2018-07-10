#pragma once

#include <cmath>
#include <memory>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class Himmelblau : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    Himmelblau()
    {
    }
    virtual ~Himmelblau()
    {
    }

    void cacheData()
    {
        return;
    }
    /*
     * Evaluate Himmelblau function:
     *      f(x,y) = (x^2+y-11)^2+(y^2+x-7)^2
     * */
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(aControl.getNumVectors() == static_cast<OrdinalType>(1));

        const OrdinalType tVectorIndex = 0;
        const Plato::Vector<ScalarType, OrdinalType> & tMyControl = aControl[tVectorIndex];

        ScalarType tFirstTerm = (tMyControl[0] * tMyControl[0]) + tMyControl[1] - 11.;
        ScalarType tSecondTerm = tMyControl[0] + (tMyControl[1] * tMyControl[1]) - 7.;
        ScalarType tOutput = tFirstTerm * tFirstTerm + tSecondTerm * tSecondTerm;
        return (tOutput);
    }
    /*
     * Compute Himmelblau gradient:
     *      f_x = 4*x^3+(4*y-42)*x+2*y^2-14
     *      f_y = 4*y^3+(4*x-26)*y+2*x^2-22
     * */
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aOutput.getNumVectors() == static_cast<OrdinalType>(1));
        assert(aControl.getNumVectors() == static_cast<OrdinalType>(1));

        const OrdinalType tVectorIndex = 0;
        Plato::Vector<ScalarType, OrdinalType> & tMyOutput = aOutput[tVectorIndex];
        const Plato::Vector<ScalarType, OrdinalType> & tMyControl = aControl[tVectorIndex];

        tMyOutput[0] = 4 * tMyControl[0] * tMyControl[0] * tMyControl[0] + (4 * tMyControl[1] - 42) * tMyControl[0]
                       + 2 * tMyControl[1] * tMyControl[1] - 14;
        tMyOutput[1] = 4 * tMyControl[1] * tMyControl[1] * tMyControl[1] + (4 * tMyControl[0] - 26) * tMyControl[1]
                       + 2 * tMyControl[0] * tMyControl[0] - 22;
    }
    /*
     * Compute Himmelblau Hessian times vector:
     *          f_xx = 12*x^2+4*y-42
     *          f_xy = 4*x+4*y
     *          f_yy = 12*y^2+4*x-26
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

        ScalarType fxx = 12 * tMyControl[0] * tMyControl[0] + 4 * tMyControl[1] - 42;
        ScalarType fxy = 4 * tMyControl[0] + 4 * tMyControl[1];
        ScalarType fyy = 12 * tMyControl[1] * tMyControl[1] + 4 * tMyControl[0] - 26;
        tMyOutput[0] = fxx * tMyVector[0] + fxy * tMyVector[1];
        tMyOutput[1] = fxy * tMyVector[0] + fyy * tMyVector[1];
    }

private:
    Himmelblau(const Plato::Himmelblau<ScalarType, OrdinalType> & aRhs);
    Plato::Himmelblau<ScalarType, OrdinalType> & operator=(const Plato::Himmelblau<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato
