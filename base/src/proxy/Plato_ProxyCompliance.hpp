/*
 * Plato_ProxyCompliance.hpp
 *
 *  Created on: Nov 27, 2017
 */

#ifndef PLATO_PROXYCOMPLIANCE_HPP_
#define PLATO_PROXYCOMPLIANCE_HPP_

#include <memory>

#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_EpetraSerialDenseVector.hpp"
#include "Plato_StructuralTopologyOptimization.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class ProxyCompliance : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    explicit ProxyCompliance(const std::shared_ptr<Plato::StructuralTopologyOptimization> & aSolver) :
            mIsFilterDisabled(false),
            mFirstObjective(-1),
            mFilteredGradient(Epetra_SerialDenseVector(aSolver->getNumDesignVariables())),
            mUnfilteredGradient(Epetra_SerialDenseVector(aSolver->getNumDesignVariables())),
            mSolver(aSolver)
    {
    }
    virtual ~ProxyCompliance()
    {
    }

    void disableFilter()
    {
        mIsFilterDisabled = true;
    }
    void cacheData()
    {
        mSolver->cacheState();
    }
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        const OrdinalType tVectorIndex = 0;
        const Plato::Vector<ScalarType, OrdinalType> & tMyControl = aControl[tVectorIndex];
        const Plato::EpetraSerialDenseVector<ScalarType, OrdinalType> & tControl =
                dynamic_cast<const Plato::EpetraSerialDenseVector<ScalarType, OrdinalType>&>(tMyControl);

        assert(tControl.size() == static_cast<OrdinalType>(mSolver->getNumDesignVariables()));
        // Solve partial differential equation
        mSolver->solve(tControl.vector());
        // Compute potential energy (i.e. compliance objective)
        ScalarType tOutput = mSolver->computeCompliance(tControl.vector());
        if(mFirstObjective < 0)
        {
            mFirstObjective = tOutput;
        }
        tOutput = tOutput / mFirstObjective;
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
        assert(tOutput.size() == static_cast<OrdinalType>(mFilteredGradient.Length()));
        assert(tOutput.size() == static_cast<OrdinalType>(mUnfilteredGradient.Length()));
        // Compute unfiltered gradient
        ScalarType tValue = 0;
        mUnfilteredGradient.SCAL(mUnfilteredGradient.Length(), tValue, mUnfilteredGradient.A());
        mSolver->computeComplianceGradient(tEpetraControl.vector(), mUnfilteredGradient);
        tValue = static_cast<ScalarType>(1) / mFirstObjective;
        mUnfilteredGradient.SCAL(mUnfilteredGradient.Length(), tValue, mUnfilteredGradient.A());
        // Apply filter to gradient
        const OrdinalType tLength = mFilteredGradient.Length();
        Plato::EpetraSerialDenseVector<ScalarType, OrdinalType> & tEpetraOutput =
                dynamic_cast<Plato::EpetraSerialDenseVector<ScalarType, OrdinalType>&>(tOutput);
        if(mIsFilterDisabled == false)
        {
            mSolver->applySensitivityFilter(tEpetraControl.vector(), mUnfilteredGradient, mFilteredGradient);
            mFilteredGradient.COPY(tLength, mFilteredGradient.A(), tEpetraOutput.vector().A());
        }
        else
        {
            mUnfilteredGradient.COPY(tLength, mUnfilteredGradient.A(), tEpetraOutput.vector().A());
        }
    }
    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        const OrdinalType tVectorIndex = 0;
        assert(aControl.getNumVectors() == aVector.getNumVectors());
        assert(aControl.getNumVectors() == aOutput.getNumVectors());
        assert(aControl[tVectorIndex].size() == aVector[tVectorIndex].size());
        assert(aControl[tVectorIndex].size() == aOutput[tVectorIndex].size());

        Plato::Vector<ScalarType, OrdinalType> & tOutput = aOutput[tVectorIndex];
        Plato::EpetraSerialDenseVector<ScalarType, OrdinalType> & tEpetraOutput =
                dynamic_cast<Plato::EpetraSerialDenseVector<ScalarType, OrdinalType>&>(tOutput);
        const Plato::Vector<ScalarType, OrdinalType> & tVector = aVector[tVectorIndex];
        const Plato::EpetraSerialDenseVector<ScalarType, OrdinalType> & tEpetraVector =
                dynamic_cast<const Plato::EpetraSerialDenseVector<ScalarType, OrdinalType>&>(tVector);
        const Plato::Vector<ScalarType, OrdinalType> & tControl = aControl[tVectorIndex];
        const Plato::EpetraSerialDenseVector<ScalarType, OrdinalType> & tEpetraControl =
                dynamic_cast<const Plato::EpetraSerialDenseVector<ScalarType, OrdinalType>&>(tControl);

        mSolver->computeComplianceHessianTimesVector(tEpetraControl.vector(), tEpetraVector.vector(), tEpetraOutput.vector());
        const double tValue = static_cast<ScalarType>(1) / mFirstObjective;
        tEpetraOutput.scale(tValue);
    }

private:
    bool mIsFilterDisabled;
    ScalarType mFirstObjective;
    Epetra_SerialDenseVector mFilteredGradient;
    Epetra_SerialDenseVector mUnfilteredGradient;
    std::shared_ptr<Plato::StructuralTopologyOptimization> mSolver;

private:
    ProxyCompliance(const Plato::ProxyCompliance<ScalarType, OrdinalType>&);
    Plato::ProxyCompliance<ScalarType, OrdinalType> & operator=(const Plato::ProxyCompliance<ScalarType, OrdinalType>&);
};

} // namespace Plato

#endif /* PLATO_PROXYCOMPLIANCE_HPP_ */
