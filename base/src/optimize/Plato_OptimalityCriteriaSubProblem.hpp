/*
 * Plato_OptimalityCriteriaSubProblem.hpp
 *
 *  Created on: Oct 17, 2017
 */

#ifndef PLATO_OPTIMALITYCRITERIASUBPROBLEM_HPP_
#define PLATO_OPTIMALITYCRITERIASUBPROBLEM_HPP_

namespace Plato
{

template<typename ScalarType, typename OrdinalType>
class OptimalityCriteriaDataMng;
template<typename ScalarType, typename OrdinalType>
class OptimalityCriteriaStageMngBase;

template<typename ScalarType, typename OrdinalType = size_t>
class OptimalityCriteriaSubProblem
{
public:
    virtual ~OptimalityCriteriaSubProblem(){}

    virtual void solve(Plato::OptimalityCriteriaDataMng<ScalarType, OrdinalType> & aDataMng,
                       Plato::OptimalityCriteriaStageMngBase<ScalarType, OrdinalType> & aStageMng) = 0;
};

}

#endif /* PLATO_OPTIMALITYCRITERIASUBPROBLEM_HPP_ */
