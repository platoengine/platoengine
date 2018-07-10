/*
 * Plato_ConstraintSimOptROL.hpp
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

#ifndef PLATO_CONSTRAINTSIMOPTROL_HPP_
#define PLATO_CONSTRAINTSIMOPTROL_HPP_

#include <string>
#include <vector>
#include <memory>
#include <cassert>
#include <algorithm>

#include "ROL_Constraint_SimOpt.hpp"

#include "Plato_Interface.hpp"
#include "Plato_DistributedVectorROL.hpp"
#include "Plato_OptimizerEngineStageData.hpp"

namespace Plato
{

template<typename ScalarType>
class ConstraintSimOptROL : public ROL::Constraint_SimOpt<ScalarType>
{
public:
    explicit ConstraintSimOptROL(const Plato::OptimizerEngineStageData & aInputData) :
            mInterface(nullptr),
            mParameterList(std::make_shared<Teuchos::ParameterList>()),
            mEngineStageData(aInputData)
    {
    }
    virtual ~ConstraintSimOptROL()
    {
    }

    void value(ROL::Vector<ScalarType> & aOutput,
               const ROL::Vector<ScalarType> & aState,
               const ROL::Vector<ScalarType> & aControl,
               ScalarType & aTolerance)
    {
        assert(aOutput.dimension() == aState.dimension());

        const size_t tCONTROL_VECTOR_INDEX = 0;
        std::string tControlName = mEngineStageData.getControlName(tCONTROL_VECTOR_INDEX);
        assert(aControl.dimension() == mInterface->size(tControlName));

        const Plato::DistributedVectorROL<ScalarType> & tControl =
                dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aControl);
        const std::vector<ScalarType> & tControlData = tControl.data();
        mParameterList->set(tControlName, tControlData.data());

        const Plato::DistributedVectorROL<ScalarType> & tState =
                dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aState);
        assert(tState.dimension() > static_cast<int>(0));
        const std::vector<ScalarType> & tStateData = tState.data();
        std::string tStateName = mEngineStageData.getStateName();
        mParameterList->set(tStateName, tStateData.data());

        const std::string tToleranceName("InexactnessTolerance");
        mParameterList->set(tToleranceName, &aTolerance);

        const Plato::DistributedVectorROL<ScalarType> & tOutput =
                dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aOutput);
        std::vector<ScalarType> & tOutputData = tOutput.data();
        std::fill(tOutputData.begin(), tOutputData.end(), static_cast<ScalarType>(0));
        const std::string tOutputName("ContraintResidual");
        mParameterList->set(tOutputName, tOutputData.data());

        std::vector<std::string> tStageNames;
        tStageNames.push_back(tOutputName);
        mInterface->compute(tStageNames, *mParameterList);
    }

    void applyJacobian_1(ROL::Vector<ScalarType> & aOutput,
                         const ROL::Vector<ScalarType> & aVector,
                         const ROL::Vector<ScalarType> & aState,
                         const ROL::Vector<ScalarType> & aControl,
                         ScalarType & aTolerance)
    {
        assert(aOutput.dimension() == aState.dimension());
        assert(aVector.dimension() == aState.dimension());

        const std::string tOutputName("ApplyVectorToJacobianState");
        this->compute(aOutput, tOutputName, aVector, aState, aControl, aTolerance);
    }

    void applyJacobian_2(ROL::Vector<ScalarType> & aOutput,
                         const ROL::Vector<ScalarType> & aVector,
                         const ROL::Vector<ScalarType> & aState,
                         const ROL::Vector<ScalarType> & aControl,
                         ScalarType & aTolerance)
    {
        assert(aOutput.dimension() == aState.dimension());
        assert(aVector.dimension() == aControl.dimension());

        const std::string tOutputName("ApplyVectorToJacobianControl");
        this->compute(aOutput, tOutputName, aVector, aState, aControl, aTolerance);
    }

    void applyInverseJacobian_1(ROL::Vector<ScalarType> & aOutput,
                                const ROL::Vector<ScalarType> & aVector,
                                const ROL::Vector<ScalarType> & aState,
                                const ROL::Vector<ScalarType> & aControl,
                                ScalarType & aTolerance)
    {
        assert(aOutput.dimension() == aState.dimension());
        assert(aVector.dimension() == aState.dimension());

        const std::string tOutputName("ApplyVectorToInverseJacobianControl");
        this->compute(aOutput, tOutputName, aVector, aState, aControl, aTolerance);
    }

    void applyAdjointJacobian_1(ROL::Vector<ScalarType> & aOutput,
                                const ROL::Vector<ScalarType> & aVector,
                                const ROL::Vector<ScalarType> & aState,
                                const ROL::Vector<ScalarType> & aControl,
                                ScalarType & aTolerance)
    {
        assert(aOutput.dimension() == aState.dimension());
        assert(aVector.dimension() == aState.dimension());

        const std::string tOutputName("ApplyVectorToAdjointJacobianState");
        this->compute(aOutput, tOutputName, aVector, aState, aControl, aTolerance);
    }

    void applyAdjointJacobian_2(ROL::Vector<ScalarType> & aOutput,
                                const ROL::Vector<ScalarType> & aVector,
                                const ROL::Vector<ScalarType> & aState,
                                const ROL::Vector<ScalarType> & aControl,
                                ScalarType & aTolerance)
    {
        assert(aVector.dimension() == aState.dimension());
        assert(aOutput.dimension() == aControl.dimension());

        const std::string tOutputName("ApplyVectorToAdjointJacobianControl");
        this->compute(aOutput, tOutputName, aVector, aState, aControl, aTolerance);
    }

    void applyInverseAdjointJacobian_1(ROL::Vector<ScalarType> & aOutput,
                                       const ROL::Vector<ScalarType> & aVector,
                                       const ROL::Vector<ScalarType> & aState,
                                       const ROL::Vector<ScalarType> & aControl,
                                       ScalarType & aTolerance)
    {
        assert(aVector.dimension() == aState.dimension());
        assert(aOutput.dimension() == aState.dimension());

        const std::string tOutputName("ApplyVectorToInverseAdjointJacobianControl");
        this->compute(aOutput, tOutputName, aVector, aState, aControl, aTolerance);
    }

    void applyAdjointHessian_11(ROL::Vector<ScalarType> & aOutput,
                                const ROL::Vector<ScalarType> & aDual,
                                const ROL::Vector<ScalarType> & aVector,
                                const ROL::Vector<ScalarType> & aState,
                                const ROL::Vector<ScalarType> & aControl,
                                ScalarType & aTolerance)
    {
        assert(aVector.dimension() == aState.dimension());
        assert(aOutput.dimension() == aState.dimension());

        const std::string tOutputName("ApplyVectorToAdjointHessianStateState");
        this->compute(aOutput, tOutputName, aDual, aVector, aState, aControl, aTolerance);
    }

    void applyAdjointHessian_12(ROL::Vector<ScalarType> & aOutput,
                                const ROL::Vector<ScalarType> & aDual,
                                const ROL::Vector<ScalarType> & aVector,
                                const ROL::Vector<ScalarType> & aState,
                                const ROL::Vector<ScalarType> & aControl,
                                ScalarType & aTolerance)
    {
        assert(aOutput.dimension() == aState.dimension());
        assert(aVector.dimension() == aControl.dimension());

        const std::string tOutputName("ApplyVectorToAdjointHessianStateControl");
        this->compute(aOutput, tOutputName, aDual, aVector, aState, aControl, aTolerance);
    }

    void applyAdjointHessian_21(ROL::Vector<ScalarType> & aOutput,
                                const ROL::Vector<ScalarType> & aDual,
                                const ROL::Vector<ScalarType> & aVector,
                                const ROL::Vector<ScalarType> & aState,
                                const ROL::Vector<ScalarType> & aControl,
                                ScalarType & aTolerance)
    {
        assert(aVector.dimension() == aState.dimension());
        assert(aOutput.dimension() == aControl.dimension());

        const std::string tOutputName("ApplyVectorToAdjointHessianControlState");
        this->compute(aOutput, tOutputName, aDual, aVector, aState, aControl, aTolerance);
    }

    void applyAdjointHessian_22(ROL::Vector<ScalarType> & aOutput,
                                const ROL::Vector<ScalarType> & aDual,
                                const ROL::Vector<ScalarType> & aVector,
                                const ROL::Vector<ScalarType> & aState,
                                const ROL::Vector<ScalarType> & aControl,
                                ScalarType & aTolerance)
    {
        assert(aVector.dimension() == aControl.dimension());
        assert(aOutput.dimension() == aControl.dimension());

        const std::string tOutputName("ApplyVectorToAdjointHessianControlControl");
        this->compute(aOutput, tOutputName, aDual, aVector, aState, aControl, aTolerance);
    }

private:
    void compute(ROL::Vector<ScalarType> & aOutput,
                 const std::string & aOutputName,
                 const ROL::Vector<ScalarType> & aVector,
                 const ROL::Vector<ScalarType> & aState,
                 const ROL::Vector<ScalarType> & aControl,
                 ScalarType & aTolerance)
    {
        const size_t tCONTROL_VECTOR_INDEX = 0;
        std::string tControlName = mEngineStageData.getControlName(tCONTROL_VECTOR_INDEX);
        assert(aControl.dimension() == mInterface->size(tControlName));

        const Plato::DistributedVectorROL<ScalarType> & tControl =
                dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aControl);
        const std::vector<ScalarType> & tControlData = tControl.data();
        mParameterList->set(tControlName, tControlData.data());

        const Plato::DistributedVectorROL<ScalarType> & tState =
                dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aState);
        assert(tState.dimension() > static_cast<int>(0));
        const std::vector<ScalarType> & tStateData = tState.data();
        std::string tStateName = mEngineStageData.getStateName();
        mParameterList->set(tStateName, tStateData.data());

        const Plato::DistributedVectorROL<ScalarType> & tVector =
                dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aVector);
        assert(tVector.dimension() > static_cast<int>(0));
        const std::vector<ScalarType> & tVectorData = tVector.data();
        std::string tVectorName("VectorToApply");
        mParameterList->set(tVectorName, tVectorData.data());

        const std::string tToleranceName("InexactnessTolerance");
        mParameterList->set(tToleranceName, &aTolerance);

        const Plato::DistributedVectorROL<ScalarType> & tOutput =
                dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aOutput);
        std::vector<ScalarType> & tOutputData = tOutput.data();
        std::fill(tOutputData.begin(), tOutputData.end(), static_cast<ScalarType>(0));
        mParameterList->set(aOutputName, tOutputData.data());

        std::vector<std::string> tStageNames;
        std::string tOutputName = aOutputName;
        tStageNames.push_back(tOutputName);
        mInterface->compute(tStageNames, *mParameterList);
    }
    void compute(ROL::Vector<ScalarType> & aOutput,
                 const std::string & aOutputName,
                 const ROL::Vector<ScalarType> & aDual,
                 const ROL::Vector<ScalarType> & aVector,
                 const ROL::Vector<ScalarType> & aState,
                 const ROL::Vector<ScalarType> & aControl,
                 ScalarType & aTolerance)
    {
        const size_t tCONTROL_VECTOR_INDEX = 0;
        std::string tControlName = mEngineStageData.getControlName(tCONTROL_VECTOR_INDEX);
        assert(aControl.dimension() == mInterface->size(tControlName));

        const Plato::DistributedVectorROL<ScalarType> & tControl =
                dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aControl);
        const std::vector<ScalarType> & tControlData = tControl.data();
        mParameterList->set(tControlName, tControlData.data());

        const Plato::DistributedVectorROL<ScalarType> & tState =
                dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aState);
        assert(tState.dimension() > static_cast<int>(0));
        const std::vector<ScalarType> & tStateData = tState.data();
        std::string tStateName = mEngineStageData.getStateName();
        mParameterList->set(tStateName, tStateData.data());

        const Plato::DistributedVectorROL<ScalarType> & tVector =
                dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aVector);
        assert(tVector.dimension() > static_cast<int>(0));
        const std::vector<ScalarType> & tVectorData = tVector.data();
        std::string tVectorName("VectorToApply");
        mParameterList->set(tVectorName, tVectorData.data());

        const Plato::DistributedVectorROL<ScalarType> & tDual =
                dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aDual);
        assert(tDual.dimension() > static_cast<int>(0));
        const std::vector<ScalarType> & tDualData = tDual.data();
        std::string tDualName("Dual");
        mParameterList->set(tDualName, tDualData.data());

        const std::string tToleranceName("InexactnessTolerance");
        mParameterList->set(tToleranceName, &aTolerance);

        const Plato::DistributedVectorROL<ScalarType> & tOutput =
                dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aOutput);
        std::vector<ScalarType> & tOutputData = tOutput.data();
        std::fill(tOutputData.begin(), tOutputData.end(), static_cast<ScalarType>(0));
        mParameterList->set(aOutputName, tOutputData.data());

        std::vector<std::string> tStageNames;
        std::string tOutputName = aOutputName;
        tStageNames.push_back(tOutputName);
        mInterface->compute(tStageNames, *mParameterList);
    }

private:
    Plato::Interface* mInterface;
    Plato::OptimizerEngineStageData mEngineStageData;
    std::shared_ptr<Teuchos::ParameterList> mParameterList;

private:
    ConstraintSimOptROL(const Plato::ConstraintSimOptROL<ScalarType> & aRhs);
    Plato::ConstraintSimOptROL<ScalarType> & operator=(const Plato::ConstraintSimOptROL<ScalarType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_CONSTRAINTSIMOPTROL_HPP_ */
