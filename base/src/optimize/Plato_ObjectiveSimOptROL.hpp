/*
 * Plato_ObjectiveSimOptROL.hpp
 *
 *  Created on: Feb 8, 2018
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
