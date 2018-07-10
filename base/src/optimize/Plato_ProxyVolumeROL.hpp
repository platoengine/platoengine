/*
 * Plato_ProxyVolumeROL.hpp
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

#ifndef PLATO_PROXYVOLUMEROL_HPP_

#define PLATO_PROXYVOLUMEROL_HPP_

#include <memory>
#include <cassert>

#include "ROL_Constraint.hpp"

#include "Plato_SerialEpetraVectorROL.hpp"
#include "Plato_StructuralTopologyOptimization.hpp"

namespace Plato
{

template<class ScalarType>
class ProxyVolumeROL : public ROL::Constraint<ScalarType>
{
public:
    explicit ProxyVolumeROL(const std::shared_ptr<Plato::StructuralTopologyOptimization> & aSolver) :
            mWork(),
            mSolver(aSolver)
    {
        int tLength = mSolver->getNumDesignVariables();
        mWork.Resize(tLength);
    }
    virtual ~ProxyVolumeROL()
    {
    }

    void value(ROL::Vector<ScalarType> & aOutput, const ROL::Vector<ScalarType> & aControl, ScalarType & aTolerance) override
    {
        assert(aOutput.dimension() == static_cast<int>(1));
        assert(aControl.dimension() == mSolver->getNumDesignVariables());

        // Get Epetra Serial Dense Vector
        Plato::SerialEpetraVectorROL < ScalarType > &tOutput = dynamic_cast<Plato::SerialEpetraVectorROL<ScalarType>&>(aOutput);
        const Plato::SerialEpetraVectorROL<ScalarType> & tControl =
                dynamic_cast<const Plato::SerialEpetraVectorROL<ScalarType>&>(aControl);
        // Compute volume misfit
        const int tConstraintIndex = 0;
        Epetra_SerialDenseVector & tConstraint = tOutput.vector();
        tConstraint[tConstraintIndex] = mSolver->computeVolumeMisfit(tControl.vector());
    }
    void applyJacobian(ROL::Vector<ScalarType> & aOutput,
                       const ROL::Vector<ScalarType> & aDirection,
                       const ROL::Vector<ScalarType> & aControl,
                       ScalarType & aTolerance) override
    {
        assert(mWork.Length() == aControl.dimension());
        assert(aOutput.dimension() == static_cast<int>(1));
        assert(aControl.dimension() == aDirection.dimension());

        // Get Epetra Serial Dense Vector
        Plato::SerialEpetraVectorROL < ScalarType > &tOutput = dynamic_cast<Plato::SerialEpetraVectorROL<ScalarType>&>(aOutput);
        const Plato::SerialEpetraVectorROL<ScalarType> & tControl =
                dynamic_cast<const Plato::SerialEpetraVectorROL<ScalarType>&>(aControl);
        const Plato::SerialEpetraVectorROL<ScalarType> & tDirection =
                dynamic_cast<const Plato::SerialEpetraVectorROL<ScalarType>&>(aDirection);
        // Compute gradient
        ScalarType tValue = 0;
        int tLength = mWork.Length();
        mWork.SCAL(tLength, tValue, mWork.A());
        mSolver->computeVolumeGradient(tControl.vector(), mWork);
        // Apply direction to Jacobian
        tOutput.vector()[0] = mWork.DOT(tLength, mWork.A(), tDirection.vector().A());
    }

    void applyAdjointJacobian(ROL::Vector<ScalarType> & aOutput,
                              const ROL::Vector<ScalarType> & aDirection,
                              const ROL::Vector<ScalarType> & aControl,
                              ScalarType & aTolerance) override
    {
        assert(mWork.Length() == aControl.dimension());
        assert(aOutput.dimension() == aControl.dimension());
        assert(aDirection.dimension() == static_cast<int>(1));

        // Get Epetra Serial Dense Vector
        Plato::SerialEpetraVectorROL < ScalarType > &tOutput = dynamic_cast<Plato::SerialEpetraVectorROL<ScalarType>&>(aOutput);
        const Plato::SerialEpetraVectorROL<ScalarType> & tControl =
                dynamic_cast<const Plato::SerialEpetraVectorROL<ScalarType>&>(aControl);
        const Plato::SerialEpetraVectorROL<ScalarType> & tDirection =
                dynamic_cast<const Plato::SerialEpetraVectorROL<ScalarType>&>(aDirection);
        // Compute gradient
        ScalarType tValue = 0;
        int tLength = tOutput.vector().Length();
        mWork.SCAL(tLength, tValue, tOutput.vector().A());
        mSolver->computeVolumeGradient(tControl.vector(), tOutput.vector());
        // Apply direction to Jacobian
        tValue = tDirection.vector()[0];
        mWork.SCAL(tLength, tValue, tOutput.vector().A());
    }

private:
    Epetra_SerialDenseVector mWork;
    std::shared_ptr<Plato::StructuralTopologyOptimization> mSolver;

private:
    ProxyVolumeROL(const Plato::ProxyVolumeROL<ScalarType> & aRhs);
    Plato::ProxyVolumeROL<ScalarType> & operator=(const Plato::ProxyVolumeROL<ScalarType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_PROXYVOLUMEROL_HPP_ */
