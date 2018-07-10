/*
 * Plato_TrustRegionStageMng.hpp
 *
 *  Created on: Oct 21, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_TRUSTREGIONSTAGEMNG_HPP_
#define PLATO_TRUSTREGIONSTAGEMNG_HPP_

#include <limits>

namespace Plato
{

template<typename ScalarType, typename OrdinalType>
class MultiVector;
template<typename ScalarType, typename OrdinalType>
class TrustRegionAlgorithmDataMng;

template<typename ScalarType, typename OrdinalType = size_t>
class TrustRegionStageMng
{
public:
    virtual ~TrustRegionStageMng()
    {
    }

    virtual void cacheData() = 0;
    virtual void updateOptimizationData(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng) = 0;
    virtual ScalarType evaluateObjective(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                          ScalarType aTolerance = std::numeric_limits<ScalarType>::max()) = 0;
    virtual void computeGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;
    virtual void applyVectorToHessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                      const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                                      Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;
    virtual void applyVectorToPreconditioner(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                             const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                                             Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;
    virtual void applyVectorToInvPreconditioner(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                                const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                                                Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;
};

} // namespace Plato

#endif /* PLATO_TRUSTREGIONSTAGEMNG_HPP_ */
