/*
 * Plato_ProxyVolume.hpp
 *
 *  Created on: Nov 27, 2017
 */

#ifndef PLATO_PROXYVOLUME_HPP_
#define PLATO_PROXYVOLUME_HPP_

#include <memory>

#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_EpetraSerialDenseVector.hpp"
#include "Plato_StructuralTopologyOptimization.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class ProxyVolume : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    explicit ProxyVolume(const std::shared_ptr<Plato::StructuralTopologyOptimization> & aSolver) :
            mSolver(aSolver)
    {
    }
    virtual ~ProxyVolume()
    {
    }

    void cacheData()
    {
        return;
    }
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        const OrdinalType tVectorIndex = 0;
        const Plato::Vector<ScalarType, OrdinalType> & tMyControl = aControl[tVectorIndex];
        const Plato::EpetraSerialDenseVector<ScalarType, OrdinalType> & tControl =
                dynamic_cast<const Plato::EpetraSerialDenseVector<ScalarType, OrdinalType>&>(tMyControl);
        assert(tControl.size() == static_cast<OrdinalType>(mSolver->getNumDesignVariables()));
        // Compute volume misfit
        ScalarType tOutput = mSolver->computeVolumeMisfit(tControl.vector());
        return (tOutput);
    }
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        const OrdinalType tVectorIndex = 0;
        Plato::Vector<ScalarType, OrdinalType> & tOutput = aOutput[tVectorIndex];
        const Plato::Vector<ScalarType, OrdinalType> & tControl = aControl[tVectorIndex];
        const Plato::EpetraSerialDenseVector<ScalarType, OrdinalType> & tEpetraControl =
                dynamic_cast<const Plato::EpetraSerialDenseVector<ScalarType, OrdinalType>&>(tControl);
        assert(tEpetraControl.size() == tOutput.size());
        // Compute gradient
        Plato::EpetraSerialDenseVector<ScalarType, OrdinalType> & tEpetraOutput =
                dynamic_cast<Plato::EpetraSerialDenseVector<ScalarType, OrdinalType>&>(tOutput);
        ScalarType tValue = 0;
        OrdinalType tLength = tEpetraOutput.size();
        tEpetraOutput.vector().SCAL(tLength, tValue, tEpetraOutput.vector().A());
        mSolver->computeVolumeGradient(tEpetraControl.vector(), tEpetraOutput.vector());
    }
    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        Plato::fill(static_cast<ScalarType>(0), aOutput);
    }

private:
    std::shared_ptr<Plato::StructuralTopologyOptimization> mSolver;

private:
    ProxyVolume(const Plato::ProxyVolume<ScalarType, OrdinalType>&);
    Plato::ProxyVolume<ScalarType, OrdinalType> & operator=(const Plato::ProxyVolume<ScalarType, OrdinalType>&);
};

} // namespace Plato

#endif /* PLATO_PROXYVOLUME_HPP_ */
