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
class GoldsteinPrice : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    GoldsteinPrice()
    {
    }
    virtual ~GoldsteinPrice()
    {
    }

    void cacheData()
    {
        return;
    }
    /*
     * Evaluate GoldsteinPrice function:
     *      f(x,y) = (1+(x+y+1)^2*(3*x^2+6*x*y+3*y^2-14*x-14*y+19)) * (30+(2*x-3*y)^2*(12*x^2-36*x*y+27*y^2-32*x+48*y+18))
     * */
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(aControl.getNumVectors() == static_cast<OrdinalType>(1));

        const OrdinalType tVectorIndex = 0;
        const Plato::Vector<ScalarType, OrdinalType> & tMyControl = aControl[tVectorIndex];

        ScalarType x = tMyControl[0];
        ScalarType y = tMyControl[1];
        ScalarType tFirstTerm = (1 + (x + y + 1) * (x + y + 1) * (3 * x * x + 6 * x * y + 3 * y * y - 14 * x - 14 * y + 19));
        ScalarType tSecondTerm = (30 + (2 * x - 3 * y) * (2 * x - 3 * y)
                                    * (12 * x * x - 36 * x * y + 27 * y * y - 32 * x + 48 * y + 18));
        ScalarType tOutput = tFirstTerm * tSecondTerm;
        return (tOutput);
    }
    /*
     * Compute GoldsteinPrice gradient:
     *      f_x = ((2*(x+y+1))*(3*x^2+6*x*y+3*y^2-14*x-14*y+19)+(x+y+1)^2*(6*x+6*y-14))
     *                  *(30+(2*x-3*y)^2*(12*x^2-36*x*y+27*y^2-32*x+48*y+18))
     *                  +(1+(x+y+1)^2*(3*x^2+6*x*y+3*y^2-14*x-14*y+19))
     *                  *(4*(2*x-3*y)*(12*x^2-36*x*y+27*y^2-32*x+48*y+18)+(2*x-3*y)^2*(24*x-36*y-32))
     *      f_y = ((2*(x+y+1))*(3*x^2+6*x*y+3*y^2-14*x-14*y+19)+(x+y+1)^2*(6*x+6*y-14))
     *                  *(30+(2*x-3*y)^2*(12*x^2-36*x*y+27*y^2-32*x+48*y+18))
     *                  +(1+(x+y+1)^2*(3*x^2+6*x*y+3*y^2-14*x-14*y+19))
     *                  *(-6*(2*x-3*y)*(12*x^2-36*x*y+27*y^2-32*x+48*y+18)+(2*x-3*y)^2*(-36*x+54*y+48))
     * */
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aOutput.getNumVectors() == static_cast<OrdinalType>(1));
        assert(aControl.getNumVectors() == static_cast<OrdinalType>(1));

        const OrdinalType tVectorIndex = 0;
        Plato::Vector<ScalarType, OrdinalType> & tMyOutput = aOutput[tVectorIndex];
        const Plato::Vector<ScalarType, OrdinalType> & tMyControl = aControl[tVectorIndex];

        ScalarType x = tMyControl[0];
        ScalarType y = tMyControl[1];
        ScalarType tFirstTerm = 2 * (x + y + 1) * (3 * x * x + 6 * x * y + 3 * y * y - 14 * x - 14 * y + 19)
                                + (x + y + 1) * (x + y + 1) * (6 * x + 6 * y - 14);
        ScalarType tSecondTerm = 30
                                 + (2 * x - 3 * y) * (2 * x - 3 * y)
                                   * (12 * x * x - 36 * x * y + 27 * y * y - 32 * x + 48 * y + 18);
        ScalarType tThirdTerm = 1 + (x + y + 1) * (x + y + 1) * (3 * x * x + 6 * x * y + 3 * y * y - 14 * x - 14 * y + 19);
        ScalarType tInnerFourthTerm = (2 * x - 3 * y) * (12 * x * x - 36 * x * y + 27 * y * y - 32 * x + 48 * y + 18);

        tMyOutput[0] = tFirstTerm * tSecondTerm
                       + tThirdTerm * (4 * tInnerFourthTerm + (2 * x - 3 * y) * (2 * x - 3 * y) * (24 * x - 36 * y - 32));
        tMyOutput[1] = tFirstTerm * tSecondTerm
                       + tThirdTerm * (-6 * tInnerFourthTerm + (2 * x - 3 * y) * (2 * x - 3 * y) * (-36 * x + 54 * y + 48));
    }
    /*
     * Compute GoldsteinPrice Hessian times vector:
     *          f_xx = (6*x*x+12*x*y+6*y*y-28*x-28*y+38+(4*(x+y+1))*(6*x+6*y-14)+6*(x+y+1)*(x+y+1))
     *                  *(30+(2*x-3*y)*(2*x-3*y)*(12*x*x-36*x*y+27*y*y-32*x+48*y+18))
     *                  +(2*((2*(x+y+1))*(3*x*x+6*x*y+3*y*y-14*x-14*y+19)+(x+y+1)*(x+y+1)*(6*x+6*y-14)))
     *                  *((4*(2*x-3*y))*(12*x*x-36*x*y+27*y*y-32*x+48*y+18)+(2*x-3*y)*(2*x-3*y)*(24*x-36*y-32))
     *                  +(1+(x+y+1)*(x+y+1)*(3*x*x+6*x*y+3*y*y-14*x-14*y+19))
     *                  *(96*x*x-288*x*y+216*y*y-256*x+384*y+144+(8*(2*x-3*y))*(24*x-36*y-32)+24*(2*x-3*y)*(2*x-3*y))
     *          f_xy = (6*x*x+12*x*y+6*y*y-28*x-28*y+38+(4*(x+y+1))*(6*x+6*y-14)+6*(x+y+1)*(x+y+1))
     *                  *(30+(2*x-3*y)*(2*x-3*y)*(12*x*x-36*x*y+27*y*y-32*x+48*y+18))
     *                  +((2*(x+y+1))*(3*x*x+6*x*y+3*y*y-14*x-14*y+19)+(x+y+1)*(x+y+1)*(6*x+6*y-14))
     *                  *(-(6*(2*x-3*y))*(12*x*x-36*x*y+27*y*y-32*x+48*y+18)+(2*x-3*y)*(2*x-3*y)*(-36*x+54*y+48))
     *                  +((2*(x+y+1))*(3*x*x+6*x*y+3*y*y-14*x-14*y+19)+(x+y+1)*(x+y+1)*(6*x+6*y-14))
     *                  *((4*(2*x-3*y))*(12*x*x-36*x*y+27*y*y-32*x+48*y+18)+(2*x-3*y)*(2*x-3*y)*(24*x-36*y-32))
     *                  +(1+(x+y+1)*(x+y+1)*(3*x*x+6*x*y+3*y*y-14*x-14*y+19))
     *                  *(-144*x*x+432*x*y-324*y*y+384*x-576*y-216+(4*(2*x-3*y))*(-36*x+54*y+48)-(6*(2*x-3*y))*(24*x-36*y-32)-36*(2*x-3*y)*(2*x-3*y))
     *          f_yy = (6*x*x+12*x*y+6*y*y-28*x-28*y+38+(4*(x+y+1))*(6*x+6*y-14)+6*(x+y+1)*(x+y+1))
     *                  *(30+(2*x-3*y)*(2*x-3*y)*(12*x*x-36*x*y+27*y*y-32*x+48*y+18))
     *                  +(2*((2*(x+y+1))*(3*x*x+6*x*y+3*y*y-14*x-14*y+19)+(x+y+1)*(x+y+1)*(6*x+6*y-14)))
     *                  *(-(6*(2*x-3*y))*(12*x*x-36*x*y+27*y*y-32*x+48*y+18)+(2*x-3*y)*(2*x-3*y)*(-36*x+54*y+48))
     *                  +(1+(x+y+1)*(x+y+1)*(3*x*x+6*x*y+3*y*y-14*x-14*y+19))
     *                  *(216*x*x-648*x*y+486*y*y-576*x+864*y+324-(12*(2*x-3*y))*(-36*x+54*y+48)+54*(2*x-3*y)*(2*x-3*y))
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

        ScalarType x = tMyControl[0];
        ScalarType y = tMyControl[1];
        ScalarType fxx =
                (6 * x * x + 12 * x * y + 6 * y * y - 28 * x - 28 * y + 38 + (4 * (x + y + 1)) * (6 * x + 6 * y - 14)
                 + 6 * (x + y + 1) * (x + y + 1))
                * (30 + (2 * x - 3 * y) * (2 * x - 3 * y) * (12 * x * x - 36 * x * y + 27 * y * y - 32 * x + 48 * y + 18))
                + (2
                   * ((2 * (x + y + 1)) * (3 * x * x + 6 * x * y + 3 * y * y - 14 * x - 14 * y + 19)
                      + (x + y + 1) * (x + y + 1) * (6 * x + 6 * y - 14)))
                  * ((4 * (2 * x - 3 * y)) * (12 * x * x - 36 * x * y + 27 * y * y - 32 * x + 48 * y + 18)
                     + (2 * x - 3 * y) * (2 * x - 3 * y) * (24 * x - 36 * y - 32))
                + (1 + (x + y + 1) * (x + y + 1) * (3 * x * x + 6 * x * y + 3 * y * y - 14 * x - 14 * y + 19))
                  * (96 * x * x - 288 * x * y + 216 * y * y - 256 * x + 384 * y + 144
                     + (8 * (2 * x - 3 * y)) * (24 * x - 36 * y - 32)
                     + 24 * (2 * x - 3 * y) * (2 * x - 3 * y));
        ScalarType fxy =
                (6 * x * x + 12 * x * y + 6 * y * y - 28 * x - 28 * y + 38 + (4 * (x + y + 1)) * (6 * x + 6 * y - 14)
                 + 6 * (x + y + 1) * (x + y + 1))
                * (30 + (2 * x - 3 * y) * (2 * x - 3 * y) * (12 * x * x - 36 * x * y + 27 * y * y - 32 * x + 48 * y + 18))
                + ((2 * (x + y + 1)) * (3 * x * x + 6 * x * y + 3 * y * y - 14 * x - 14 * y + 19)
                   + (x + y + 1) * (x + y + 1) * (6 * x + 6 * y - 14))
                  * (-(6 * (2 * x - 3 * y)) * (12 * x * x - 36 * x * y + 27 * y * y - 32 * x + 48 * y + 18)
                     + (2 * x - 3 * y) * (2 * x - 3 * y) * (-36 * x + 54 * y + 48))
                + ((2 * (x + y + 1)) * (3 * x * x + 6 * x * y + 3 * y * y - 14 * x - 14 * y + 19)
                   + (x + y + 1) * (x + y + 1) * (6 * x + 6 * y - 14))
                  * ((4 * (2 * x - 3 * y)) * (12 * x * x - 36 * x * y + 27 * y * y - 32 * x + 48 * y + 18)
                     + (2 * x - 3 * y) * (2 * x - 3 * y) * (24 * x - 36 * y - 32))
                + (1 + (x + y + 1) * (x + y + 1) * (3 * x * x + 6 * x * y + 3 * y * y - 14 * x - 14 * y + 19))
                  * (-144 * x * x + 432 * x * y - 324 * y * y + 384 * x - 576 * y - 216
                     + (4 * (2 * x - 3 * y)) * (-36 * x + 54 * y + 48)
                     - (6 * (2 * x - 3 * y)) * (24 * x - 36 * y - 32)
                     - 36 * (2 * x - 3 * y) * (2 * x - 3 * y));
        ScalarType fyy =
                (6 * x * x + 12 * x * y + 6 * y * y - 28 * x - 28 * y + 38 + (4 * (x + y + 1)) * (6 * x + 6 * y - 14)
                 + 6 * (x + y + 1) * (x + y + 1))
                * (30 + (2 * x - 3 * y) * (2 * x - 3 * y) * (12 * x * x - 36 * x * y + 27 * y * y - 32 * x + 48 * y + 18))
                + (2
                   * ((2 * (x + y + 1)) * (3 * x * x + 6 * x * y + 3 * y * y - 14 * x - 14 * y + 19)
                      + (x + y + 1) * (x + y + 1) * (6 * x + 6 * y - 14)))
                  * (-(6 * (2 * x - 3 * y)) * (12 * x * x - 36 * x * y + 27 * y * y - 32 * x + 48 * y + 18)
                     + (2 * x - 3 * y) * (2 * x - 3 * y) * (-36 * x + 54 * y + 48))
                + (1 + (x + y + 1) * (x + y + 1) * (3 * x * x + 6 * x * y + 3 * y * y - 14 * x - 14 * y + 19))
                  * (216 * x * x - 648 * x * y + 486 * y * y - 576 * x + 864 * y + 324
                     - (12 * (2 * x - 3 * y)) * (-36 * x + 54 * y + 48)
                     + 54 * (2 * x - 3 * y) * (2 * x - 3 * y));
        tMyOutput[0] = fxx * tMyVector[0] + fxy * tMyVector[1];
        tMyOutput[1] = fxy * tMyVector[0] + fyy * tMyVector[1];
    }

private:
    GoldsteinPrice(const Plato::GoldsteinPrice<ScalarType, OrdinalType> & aRhs);
    Plato::GoldsteinPrice<ScalarType, OrdinalType> & operator=(const Plato::GoldsteinPrice<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato
