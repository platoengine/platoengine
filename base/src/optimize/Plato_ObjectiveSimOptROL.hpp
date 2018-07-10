/*
 * Plato_ObjectiveSimOptROL.hpp
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

#ifndef PLATO_OBJECTIVESIMOPTROL_HPP_
#define PLATO_OBJECTIVESIMOPTROL_HPP_

#include <string>
#include <vector>
#include <memory>
#include <cassert>
#include <algorithm>

#include "ROL_Objective_SimOpt.hpp"

#include "Plato_Interface.hpp"
#include "Plato_DistributedVectorROL.hpp"
#include "Plato_OptimizerEngineStageData.hpp"

namespace Plato
{

template<typename ScalarType>
class ObjectiveSimOptROL : public ROL::Objective_SimOpt<ScalarType>
{
public:
    explicit ObjectiveSimOptROL(const Plato::OptimizerEngineStageData & aInputData) :
            mInterface(nullptr),
            mParameterList(std::make_shared<Teuchos::ParameterList>()),
            mEngineStageData(aInputData)
    {
    }
    virtual ~ObjectiveSimOptROL()
    {
    }

    ScalarType value(const ROL::Vector<ScalarType> & aState, const ROL::Vector<ScalarType> & aControl, ScalarType & aTolerance)
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

        const std::string tToleranceName("InexactnessTolerance");
        mParameterList->set(tToleranceName, &aTolerance);

        ScalarType tObjectiveValue = 0;
        std::string tObjectiveValueName = mEngineStageData.getObjectiveValueName();
        mParameterList->set(tObjectiveValueName, &tObjectiveValue);

        std::vector<std::string> tStageNames;
        tStageNames.push_back(tObjectiveValueName);
        mInterface->compute(tStageNames, *mParameterList);

        return (tObjectiveValue);
    }

    void gradient_1(ROL::Vector<ScalarType> & aOutput,
                    const ROL::Vector<ScalarType> & aState,
                    const ROL::Vector<ScalarType> & aControl,
                    ScalarType & aTolerance)
    {
        assert(aState.dimension() == aOutput.dimension());

        const std::string tOutputName("ObjectivePartialDerivativeState");
        this->compute(aOutput, tOutputName, aState, aControl, aTolerance);
    }

    void gradient_2(ROL::Vector<ScalarType> & aOutput,
                    const ROL::Vector<ScalarType> & aState,
                    const ROL::Vector<ScalarType> & aControl,
                    ScalarType & aTolerance)
    {
        assert(aControl.dimension() == aOutput.dimension());

        const std::string tOutputName("ObjectivePartialDerivativeControl");
        this->compute(aOutput, tOutputName, aState, aControl, aTolerance);
    }

    void hessVec_11(ROL::Vector<ScalarType> & aOutput,
                    const ROL::Vector<ScalarType> & aVector,
                    const ROL::Vector<ScalarType> & aState,
                    const ROL::Vector<ScalarType> & aControl,
                    ScalarType & aTolerance)
    {
        assert(aState.dimension() == aOutput.dimension());
        assert(aState.dimension() == aVector.dimension());

        const std::string tOutputName("ObjectivePartialDerivativeStateStateTimesVector");
        this->compute(aOutput, tOutputName, aVector, aState, aControl, aTolerance);
    }

    void hessVec_12(ROL::Vector<ScalarType> & aOutput,
                    const ROL::Vector<ScalarType> & aVector,
                    const ROL::Vector<ScalarType> & aState,
                    const ROL::Vector<ScalarType> & aControl,
                    ScalarType & aTolerance)
    {
        assert(aState.dimension() == aOutput.dimension());
        assert(aControl.dimension() == aVector.dimension());

        const std::string tOutputName("ObjectivePartialDerivativeStateControlTimesVector");
        this->compute(aOutput, tOutputName, aVector, aState, aControl, aTolerance);
    }

    void hessVec_21(ROL::Vector<ScalarType> & aOutput,
                    const ROL::Vector<ScalarType> & aVector,
                    const ROL::Vector<ScalarType> & aState,
                    const ROL::Vector<ScalarType> & aControl,
                    ScalarType & aTolerance)
    {
        assert(aState.dimension() == aVector.dimension());
        assert(aControl.dimension() == aOutput.dimension());

        const std::string tOutputName("ObjectivePartialDerivativeControlStateTimesVector");
        this->compute(aOutput, tOutputName, aVector, aState, aControl, aTolerance);
    }

    void hessVec_22(ROL::Vector<ScalarType> & aOutput,
                    const ROL::Vector<ScalarType> & aVector,
                    const ROL::Vector<ScalarType> & aState,
                    const ROL::Vector<ScalarType> & aControl,
                    ScalarType & aTolerance)
    {
        assert(aControl.dimension() == aOutput.dimension());
        assert(aControl.dimension() == aVector.dimension());

        const std::string tOutputName("ObjectivePartialDerivativeControlControlTimesVector");
        this->compute(aOutput, tOutputName, aVector, aState, aControl, aTolerance);
    }

private:
    void compute(ROL::Vector<ScalarType> & aOutput,
                 const std::string aOutputName,
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
                 const std::string aOutputName,
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
        tStageNames.push_back(aOutputName);
        mInterface->compute(tStageNames, *mParameterList);
    }

private:
    Plato::Interface* mInterface;
    Plato::OptimizerEngineStageData mEngineStageData;
    std::shared_ptr<Teuchos::ParameterList> mParameterList;

private:
    ObjectiveSimOptROL(const Plato::ObjectiveSimOptROL<ScalarType> & aRhs);
    Plato::ObjectiveSimOptROL<ScalarType> & operator=(const Plato::ObjectiveSimOptROL<ScalarType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_OBJECTIVESIMOPTROL_HPP_ */
