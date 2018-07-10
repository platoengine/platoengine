/*
 * Plato_CcsaTestObjective.hpp
 *
 *  Created on: Nov 4, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_CCSATESTOBJECTIVE_HPP_
#define PLATO_CCSATESTOBJECTIVE_HPP_

#include <vector>
#include <memory>
#include <cassert>
#include <algorithm>

#include "Plato_Vector.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_StandardVectorReductionOperations.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class CcsaTestObjective : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    CcsaTestObjective() :
            mConstant(0.0624),
            mReduction(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType,OrdinalType>>())
    {
    }
    virtual ~CcsaTestObjective()
    {
    }

    void cacheData()
    {
        return;
    }
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(aControl.getNumVectors() > static_cast<OrdinalType>(0));

        OrdinalType tNumVectors = aControl.getNumVectors();
        std::vector<ScalarType> tStorage(tNumVectors);
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyControl = aControl[tVectorIndex];
            tStorage[tVectorIndex] = mReduction->sum(tMyControl);
        }
        const ScalarType tInitialValue = 0;
        ScalarType tSum = std::accumulate(tStorage.begin(), tStorage.end(), tInitialValue);
        ScalarType tOutput = mConstant * tSum;

        return (tOutput);
    }
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        Plato::fill(mConstant, aOutput);
    }
    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        const ScalarType tScalarValue = 0;
        const OrdinalType tVectorIndex = 0;
        aOutput[tVectorIndex].fill(tScalarValue);
    }

private:
    ScalarType mConstant;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mReduction;

private:
    CcsaTestObjective(const Plato::CcsaTestObjective<ScalarType, OrdinalType> & aRhs);
    Plato::CcsaTestObjective<ScalarType, OrdinalType> & operator=(const Plato::CcsaTestObjective<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_CCSATESTOBJECTIVE_HPP_ */
