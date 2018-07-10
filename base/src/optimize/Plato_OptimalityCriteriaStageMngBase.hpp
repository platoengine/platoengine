/*
 * Plato_OptimalityCriteriaStageMngBase.hpp
 *
 *  Created on: Oct 17, 2017
 */

#ifndef PLATO_OPTIMALITYCRITERIASTAGEMNGBASE_HPP_
#define PLATO_OPTIMALITYCRITERIASTAGEMNGBASE_HPP_

namespace Plato
{

template<typename ScalarType, typename OrdinalType>
class MultiVector;
template<typename ScalarType, typename OrdinalType>
class OptimalityCriteriaDataMng;

template<typename ScalarType, typename OrdinalType = size_t>
class OptimalityCriteriaStageMngBase
{
public:
    virtual ~OptimalityCriteriaStageMngBase()
    {
    }

    //! Directive to cache any criterion specific data once the trial control is accepted.
    virtual void cacheData() = 0;
    //! Directive to update optimization specific data once the trial control is accepted.
    virtual void update(Plato::OptimalityCriteriaDataMng<ScalarType, OrdinalType> & aDataMng) = 0;
};

}

#endif /* PLATO_OPTIMALITYCRITERIASTAGEMNGBASE_HPP_ */
