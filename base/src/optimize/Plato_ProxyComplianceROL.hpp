/*
 * Plato_ProxyComplianceROL.hpp
 *
 *  Created on: Feb 8, 2018
 */

/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
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
            mSolver(aSolver),
            mSolveHasBeenCalled(false)
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
        // Solve partial differential equation
        mSolver->solve(tControl.vector());
        // Compute potential energy (i.e. compliance objective)
        ScalarType tOutput = mSolver->computeCompliance(tControl.vector());
        mSolveHasBeenCalled = true;
        return (tOutput);
    }

    void gradient(ROL::Vector<ScalarType> & aOutput, const ROL::Vector<ScalarType> & aControl, ScalarType & aTolerance) override
    {
        assert(aControl.dimension() == aOutput.dimension());
        assert(aOutput.dimension() == mFilteredGradient.Length());
        assert(aOutput.dimension() == mUnfilteredGradient.Length());
        assert(aOutput.dimension() == mSolver->getNumDesignVariables());

        if(!mSolveHasBeenCalled)
        {
            value(aControl, aTolerance);
        }

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
    bool mSolveHasBeenCalled;

private:
    ProxyComplianceROL(const Plato::ProxyComplianceROL<ScalarType> & aRhs);
    Plato::ProxyComplianceROL<ScalarType> & operator=(const Plato::ProxyComplianceROL<ScalarType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_PROXYCOMPLIANCEROL_HPP_ */
