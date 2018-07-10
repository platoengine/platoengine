/*
 * Plato_DualProblemSolver.hpp
 *
 *  Created on: Nov 4, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_DUALPROBLEMSOLVER_HPP_
#define PLATO_DUALPROBLEMSOLVER_HPP_

namespace Plato
{

template<typename ScalarType, typename OrdinalType>
class MultiVector;
template<typename ScalarType, typename OrdinalType>
class ConservativeConvexSeparableAppxDataMng;

template<typename ScalarType, typename OrdinalType = size_t>
class DualProblemSolver
{
public:
    virtual ~DualProblemSolver()
    {
    }

    virtual void solve(Plato::MultiVector<ScalarType, OrdinalType> & aDual,
                       Plato::MultiVector<ScalarType, OrdinalType> & aTrialControl) = 0;
    virtual void update(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng) = 0;
    virtual void updateObjectiveCoefficients(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng) = 0;
    virtual void updateConstraintCoefficients(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng) = 0;
    virtual void initializeAuxiliaryVariables(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng) = 0;
};

} // namespace Plato

#endif /* PLATO_DUALPROBLEMSOLVER_HPP_ */
