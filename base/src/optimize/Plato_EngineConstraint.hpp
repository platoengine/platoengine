/*
 * Plato_EngineConstraint.hpp
 *
 *  Created on: Dec 21, 2017
 */

#ifndef PLATO_ENGINECONSTRAINT_HPP_
#define PLATO_ENGINECONSTRAINT_HPP_

#include <vector>
#include <string>
#include <memory>
#include <cassert>

#include "Plato_Interface.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_DistributedVector.hpp"
#include "Plato_OptimizerEngineStageData.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class EngineConstraint : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    explicit EngineConstraint(const OrdinalType & aConstraintID,
                              const  Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory,
                              const Plato::OptimizerEngineStageData & aInputData,
                              Plato::Interface* aInterface = nullptr) :
            mMyConstraintID(aConstraintID),
            mVector(std::vector<ScalarType>(aDataFactory.getNumControls())),
            mControl(std::vector<ScalarType>(aDataFactory.getNumControls())),
            mGradient(std::vector<ScalarType>(aDataFactory.getNumControls())),
            mHessianTimesVector(std::vector<ScalarType>(aDataFactory.getNumControls())),
            mInterface(aInterface),
            mEngineInputData(aInputData),
            mParameterList(std::make_shared<Teuchos::ParameterList>())
    {
    }
    virtual ~EngineConstraint()
    {
    }
    /*!
     * Set pointer to Plato engine interface, which provides a link between the user's application and the
     * Plato engine optimizer solving the optimization problem.
     **/
    void setPlatoInterface(Plato::Interface* aInterface)
    {
        assert(aInterface != nullptr);
        mInterface = aInterface;
    }
    //! Directive to cache any criterion specific data once trial control is accepted.
    void cacheData()
    {
        return;
    }
    /*!
     * Evaluates criterion of type f(\mathbf{u}(\mathbf{z}),\mathbf{z})\colon\mathbb{R}^{n_u}\times\mathbb{R}^{n_z}
     * \rightarrow\mathbb{R}, where u denotes the state and z denotes the control variables. This criterion
     * is typically associated with nonlinear programming optimization problems. For instance, PDE constrasize_t
     * optimization problems.
     *  Parameters:
     *    \param In
     *          aControl: control variables
     *
     *  \return Objective function value
     **/
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        // ********* Set view to each control vector entry ********* //
        const OrdinalType tControlVectorIndex = 0;
        assert(aControl[tControlVectorIndex].size() == mControl.size());
        for(OrdinalType tControlIndex = 0; tControlIndex < mControl.size(); tControlIndex++)
        {
            mControl[tControlIndex] = aControl(tControlVectorIndex, tControlIndex);
        }
        std::string tControlName = mEngineInputData.getControlName(tControlVectorIndex);
        mParameterList->set(tControlName, mControl.data());

        // ********* Set view to objective function value ********* //
        ScalarType tConstraintValue = 0;
        std::string tMyConstraintName = mEngineInputData.getConstraintValueName(mMyConstraintID);
        mParameterList->set(tMyConstraintName, &tConstraintValue);

        // ********* Compute constraint value ********* //
        std::vector<std::string> tStageNames;
        tStageNames.push_back(tMyConstraintName);
        mInterface->compute(tStageNames, *mParameterList);

        const ScalarType tConstraintTarget = mEngineInputData.getConstraintNormalizedTargetValue(mMyConstraintID);
        const ScalarType tConstraintReferenceValue = mEngineInputData.getConstraintReferenceValue(mMyConstraintID);
        // NOTE: THE CURRENT ASSUMPTION IS THAT THE USER ONLY PROVIDES THE CONSTRAINT EVALUATION.
        // THUS, THE USER IS NOT PROVIDING THE ACUTAL CONSTRAINT RESIDUAL, WHICH IS DEFINED AS
        // RESIDUAL = CONSTRAINT_VALUE - CONSTRAINT_TARGET. THE USER IS JUST PROVIDING THE
        // CONSTRAINT_VALUE.  THE OPTIMALITY CRITERIA OPTIMIZER IS EXPECTING THE CONSTRAINT
        // RESIDUAL AND THUS WE NEED TO DO THE FOLLOWING CALCULATION:
        ScalarType tOutput = (tConstraintValue / tConstraintReferenceValue) - tConstraintTarget;

        return (tOutput);
    }
    /*!
     * Computes the gradient of a criterion of type f(\mathbf{u}(\mathbf{z}),\mathbf{z})\colon\mathbb{R}^{n_u}
     * \times\mathbb{R}^{n_z}\rightarrow\mathbb{R}, where u denotes the state and z denotes the control variables.
     * This criterion is typically associated with nonlinear programming optimization problems. For instance, PDE
     * constraint optimization problems.
     *  Parameters:
     *    \param In
     *          aControl: control variables
     *    \param Out
     *          aOutput: gradient
     **/
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        // ********* Set view to each control vector entry ********* //
        const OrdinalType tControlVectorIndex = 0;
        assert(aControl[tControlVectorIndex].size() == mControl.size());
        for(OrdinalType tControlIndex = 0; tControlIndex < mControl.size(); tControlIndex++)
        {
            mControl[tControlIndex] = aControl(tControlVectorIndex, tControlIndex);
        }
        std::string tControlName = mEngineInputData.getControlName(tControlVectorIndex);
        mParameterList->set(tControlName, mControl.data());

        // ********* Set view to each output vector ********* //
        std::string tConstraintGradientName = mEngineInputData.getConstraintGradientName(mMyConstraintID);
        std::fill(mGradient.begin(), mGradient.end(), static_cast<ScalarType>(0));
        mParameterList->set(tConstraintGradientName, mGradient.data());

        // ********* Compute constraint gradient ********* //
        std::vector<std::string> tStageNames;
        tStageNames.push_back(tConstraintGradientName);
        mInterface->compute(tStageNames, *mParameterList);
        this->copy(mGradient, aOutput);

        const ScalarType tConstraintReferenceValue = mEngineInputData.getConstraintReferenceValue(mMyConstraintID);
        Plato::scale(static_cast<ScalarType>(1. / tConstraintReferenceValue), aOutput);
    }
    /*!
     * Computes the application of a vector to the Hessian of a criterion of type f(\mathbf{u}(\mathbf{z}),\mathbf{z})
     * \colon\mathbb{R}^{n_u}\times\mathbb{R}^{n_z}\rightarrow\mathbb{R}, where u denotes the state and z denotes the
     * control variables. This criterion is typically associated with nonlinear programming optimization problems.
     * For instance, PDE constraint optimization problems.
     *  Parameters:
     *    \param In
     *          aControl: control variables
     *    \param In
     *          aVector:  direction vector
     *    \param Out
     *          aOutput:  Hessian times direction vector
     **/
    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        // ********* Set view to each control and descent direction vector entry ********* //
        const OrdinalType tControlVectorIndex = 0;
        assert(mVector.size() == mControl.size());
        assert(aVector[tControlVectorIndex].size() == mVector.size());
        assert(aControl[tControlVectorIndex].size() == mControl.size());
        for(OrdinalType tControlIndex = 0; tControlIndex < mControl.size(); tControlIndex++)
        {
            mVector[tControlIndex] = aVector(tControlVectorIndex, tControlIndex);
            mControl[tControlIndex] = aControl(tControlVectorIndex, tControlIndex);
        }
        std::string tControlName = mEngineInputData.getControlName(tControlVectorIndex);
        mParameterList->set(tControlName, mControl.data());
        std::string tDescentDirectionlName = mEngineInputData.getDescentDirectionName(tControlVectorIndex);
        mParameterList->set(tDescentDirectionlName, mVector.data());

        // ********* Set view to each output vector ********* //
        std::string tHessianName = mEngineInputData.getConstraintHessianName(mMyConstraintID);
        std::fill(mHessianTimesVector.begin(), mHessianTimesVector.end(), static_cast<ScalarType>(0));
        mParameterList->set(tHessianName, mHessianTimesVector.data());

        // ********* Apply vector to Hessian operator ********* //
        std::vector<std::string> tStageNames;
        tStageNames.push_back(tHessianName);
        mInterface->compute(tStageNames, *mParameterList);
        this->copy(mHessianTimesVector, aOutput);

        const ScalarType tConstraintReferenceValue = mEngineInputData.getConstraintReferenceValue(mMyConstraintID);
        Plato::scale(static_cast<ScalarType>(1. / tConstraintReferenceValue), aOutput);
    }

private:
    void copy(const std::vector<ScalarType> & aFrom, Plato::MultiVector<ScalarType, OrdinalType> & aTo)
    {
        const OrdinalType tVectorIndex = 0;
        Plato::Vector<ScalarType, OrdinalType> & tToVector = aTo[tVectorIndex];
        assert(tToVector.size() == aFrom.size());
        for(OrdinalType tControlIndex = 0; tControlIndex < aFrom.size(); tControlIndex++)
        {
            tToVector[tControlIndex] = aFrom[tControlIndex];
        }
    }

private:
    OrdinalType mMyConstraintID;

    std::vector<ScalarType> mVector;
    std::vector<ScalarType> mControl;
    std::vector<ScalarType> mGradient;
    std::vector<ScalarType> mHessianTimesVector;

    Plato::Interface* mInterface;
    Plato::OptimizerEngineStageData mEngineInputData;
    std::shared_ptr<Teuchos::ParameterList> mParameterList;

private:
    EngineConstraint(const Plato::EngineConstraint<ScalarType, OrdinalType>&);
    Plato::EngineConstraint<ScalarType, OrdinalType> & operator=(const Plato::EngineConstraint<ScalarType, OrdinalType>&);
}; // class EngineConstraint

} // namespace Plato

#endif /* PLATO_ENGINECONSTRAINT_HPP_ */
