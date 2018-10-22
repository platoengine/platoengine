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

    /******************************************************************************//**
     * @brief Disable sensitivity filter
    **********************************************************************************/
    void disableFilter()
    {
        mIsFilterDisabled = true;
    }

    /******************************************************************************//**
     * @brief Set filter radius
     * @param [in] aInput filter radius
    **********************************************************************************/
    void setFilterRadius(const ScalarType& aInput)
    {
        mSolver->setFilterRadius(aInput);
    }

    /******************************************************************************//**
     * @brief Cache displacement solution
    **********************************************************************************/
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
