/*
 * Plato_ConservativeConvexSeparableAppxStageMng.hpp
 *
 *  Created on: Nov 4, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_CONSERVATIVECONVEXSEPARABLEAPPXSTAGEMNG_HPP_
#define PLATO_CONSERVATIVECONVEXSEPARABLEAPPXSTAGEMNG_HPP_

namespace Plato
{

template<typename ScalarType, typename OrdinalType>
class MultiVector;
template<typename ScalarType, typename OrdinalType>
class MultiVectorList;
template<typename ScalarType, typename OrdinalType>
class ConservativeConvexSeparableAppxDataMng;

template<typename ScalarType, typename OrdinalType = size_t>
class ConservativeConvexSeparableAppxStageMng
{
public:
    virtual ~ConservativeConvexSeparableAppxStageMng()
    {
    }

    virtual void cacheData() = 0;
    virtual void update(const Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng) = 0;
    virtual ScalarType evaluateObjective(const Plato::MultiVector<ScalarType, OrdinalType> & aControl) = 0;
    virtual void computeGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;

    virtual void evaluateConstraints(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                     Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;
    virtual void computeConstraintGradients(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                            Plato::MultiVectorList<ScalarType, OrdinalType> & aOutput) = 0;
};

} // namespace Plato

#endif /* PLATO_CONSERVATIVECONVEXSEPARABLEAPPXSTAGEMNG_HPP_ */
