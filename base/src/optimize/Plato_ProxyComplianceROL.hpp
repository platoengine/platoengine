/*
 * Plato_ProxyComplianceROL.hpp
 *
 *  Created on: Feb 8, 2018
 */

#ifndef PLATO_PROXYCOMPLIANCEROL_HPP_

#define PLATO_PROXYCOMPLIANCEROL_HPP_

#include <memory>
#include <cassert>

#include "ROL_Objective.hpp"

#include "Plato_SerialEpetraVectorROL.hpp"
#include "Plato_StructuralTopologyOptimization.hpp"

namespace Plato
{

template<typename ScalarType>
class ProxyComplianceROL : public ROL::Objective<ScalarType>
{
public:
    explicit ProxyComplianceROL(const std::shared_ptr<Plato::StructuralTopologyOptimization> & aSolver, bool aDisableFilter =
                                        false) :
            mIsFilterDisabled(aDisableFilter),
            mFirstObjective(-1),
            mFilteredGradient(Epetra_SerialDenseVector(aSolver->getNumDesignVariables())),
            mUnfilteredGradient(Epetra_SerialDenseVector(aSolver->getNumDesignVariables())),
            mSolver(aSolver)
    {
    }
    virtual ~ProxyComplianceROL()
    {
    }

    void disableFilter()
    {
        mIsFilterDisabled = true;
    }
    void update(const ROL::Vector<ScalarType> & aControl, bool aFlag, int aIteration = -1) override
    {
        // TODO: UNDERSTAND HOW TO CACHE STATE AND ADJOINT DATA WITH ROL. I THINK THE OBJECTIVE FUNCTION EVALUATION IS OUT OF SYNC INTERNALLY.
        /*        assert(aControl.dimension() == static_cast<int>(mSolver->getNumDesignVariables()));
         // Get Epetra Serial Dense Vector
         const Plato::SerialEpetraVectorROL<ScalarType> & tControl =
         dynamic_cast<const Plato::SerialEpetraVectorROL<ScalarType>&>(aControl);
         // Solve partial differential equation
         mSolver->solve(tControl.data());
         mSolver->cacheState();*/
        return;
    }

    ScalarType value(const ROL::Vector<ScalarType> & aControl, ScalarType & aTolerance) override
    {
        assert(aControl.dimension() == static_cast<int>(mSolver->getNumDesignVariables()));

        // Get Epetra Serial Dense Vector
        const Plato::SerialEpetraVectorROL<ScalarType> & tControl =
                dynamic_cast<const Plato::SerialEpetraVectorROL<ScalarType>&>(aControl);
        const Epetra_SerialDenseVector & tEpetraControl = tControl.vector();
        // Solve partial differential equation
        mSolver->solve(tControl.vector());
        // Compute potential energy (i.e. compliance objective)
        ScalarType tOutput = mSolver->computeCompliance(tControl.vector());
        return (tOutput);
    }

    void gradient(ROL::Vector<ScalarType> & aOutput, const ROL::Vector<ScalarType> & aControl, ScalarType & aTolerance) override
    {
        assert(aControl.dimension() == aOutput.dimension());
        assert(aOutput.dimension() == mFilteredGradient.Length());
        assert(aOutput.dimension() == mUnfilteredGradient.Length());
        assert(aOutput.dimension() == mSolver->getNumDesignVariables());

        // Get Epetra Serial Dense Vector
        Plato::SerialEpetraVectorROL<ScalarType> & tOutput = dynamic_cast<Plato::SerialEpetraVectorROL<ScalarType>&>(aOutput);
        const Plato::SerialEpetraVectorROL<ScalarType> & tControl =
                dynamic_cast<const Plato::SerialEpetraVectorROL<ScalarType>&>(aControl);
        // Compute unfiltered gradient
        ScalarType tValue = 0;
        mUnfilteredGradient.SCAL(mUnfilteredGradient.Length(), tValue, mUnfilteredGradient.A());
        mSolver->computeComplianceGradient(tControl.vector(), mUnfilteredGradient);
        tValue = 1;
        mUnfilteredGradient.SCAL(mUnfilteredGradient.Length(), tValue, mUnfilteredGradient.A());
        // Apply filter to gradient
        const int tLength = mFilteredGradient.Length();
        if(mIsFilterDisabled == false)
        {
            mSolver->applySensitivityFilter(tControl.vector(), mUnfilteredGradient, mFilteredGradient);
            // Copy gradient into Plato::Vector Output Container
            mFilteredGradient.COPY(tLength, mFilteredGradient.A(), tOutput.vector().A());
        }
        else
        {
            mUnfilteredGradient.COPY(tLength, mUnfilteredGradient.A(), tOutput.vector().A());
        }
    }

    void hessVec(ROL::Vector<ScalarType> & aOutput,
                 const ROL::Vector<ScalarType> & aVector,
                 const ROL::Vector<ScalarType> & aControl,
                 ScalarType & aTolerance) override
    {
        assert(aOutput.dimension() == aVector.dimension());
        assert(aOutput.dimension() == aControl.dimension());

        // Get Epetra Serial Dense Vector
        Plato::SerialEpetraVectorROL<ScalarType> & tOutput = dynamic_cast<Plato::SerialEpetraVectorROL<ScalarType>&>(aOutput);
        const Plato::SerialEpetraVectorROL<ScalarType> & tVector =
                dynamic_cast<const Plato::SerialEpetraVectorROL<ScalarType>&>(aVector);
        const Plato::SerialEpetraVectorROL<ScalarType> & tControl =
                dynamic_cast<const Plato::SerialEpetraVectorROL<ScalarType>&>(aControl);
        // Compute application of vector to the Hessian operator
        mSolver->computeComplianceHessianTimesVector(tControl.vector(), tVector.vector(), tOutput.vector());
    }

private:
    bool mIsFilterDisabled;
    ScalarType mFirstObjective;
    Epetra_SerialDenseVector mFilteredGradient;
    Epetra_SerialDenseVector mUnfilteredGradient;
    std::shared_ptr<Plato::StructuralTopologyOptimization> mSolver;

private:
    ProxyComplianceROL(const Plato::ProxyComplianceROL<ScalarType> & aRhs);
    Plato::ProxyComplianceROL<ScalarType> & operator=(const Plato::ProxyComplianceROL<ScalarType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_PROXYCOMPLIANCEROL_HPP_ */
