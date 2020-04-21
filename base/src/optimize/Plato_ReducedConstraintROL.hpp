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
 * Plato_ReducedConstraintROL.hpp
 *
 *  Created on: Feb 8, 2018
 */

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <cassert>
#include <algorithm>

#include "ROL_Constraint.hpp"

#include "Plato_Interface.hpp"
#include "Plato_SerialVectorROL.hpp"
#include "Plato_DistributedVectorROL.hpp"
#include "Plato_OptimizerEngineStageData.hpp"

namespace Plato
{

/******************************************************************************//**
 * \brief PLATO Engine interface to a ROL reduced constraint function
 * \tparam scalar type, e.g. double, float, etc.
**********************************************************************************/
template<class ScalarType>
class ReducedConstraintROL : public ROL::Constraint<ScalarType>
{
public:
    using ROL::Constraint::applyAdjointJacobian<class ScalarType>;

    /******************************************************************************//**
     * \brief Constructor
     * \param [in] aInputData XML input data
     * \param [in] aInterface PLATO Engine interface
    **********************************************************************************/
    explicit ReducedConstraintROL(const Plato::OptimizerEngineStageData & aInputData, Plato::Interface* aInterface = nullptr) :
            mControl(),
            mAdjointJacobian(),
            mInterface(aInterface),
            mEngineInputData(aInputData),
            mParameterList(std::make_shared<Teuchos::ParameterList>())
    {
        this->initialize();
    }

    /******************************************************************************//**
     * \brief Destructor.
    **********************************************************************************/
    virtual ~ReducedConstraintROL()
    {
    }

    /******************************************************************************//**
     * \brief Evaluate all constraint functions
     * \param [in] aControl design variables
     * \param [in] aTolerance objective function inexactness tolerance
     * \return objective function value
    **********************************************************************************/
    void value(ROL::Vector<ScalarType> & aConstraints, const ROL::Vector<ScalarType> & aControl, ScalarType & aTolerance)
    {
        // ********* Set view to control vector ********* //
        const Plato::DistributedVectorROL<ScalarType> & tControl =
                dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aControl);
        const std::vector<ScalarType> & tControlData = tControl.vector();
        assert(tControlData.size() == mControl.size());
        this->copy(tControlData, mControl);
        const size_t tCONTROL_VECTOR_INDEX = 0;
        std::string tControlName = mEngineInputData.getControlName(tCONTROL_VECTOR_INDEX);
        mParameterList->set(tControlName, mControl.data());

        // ********* Set view to constraint value ********* //
        ScalarType tConstraintValue = 0;
        const size_t tMY_CONSTRAINT_INDEX = 0;
        std::string tConstraintValueName = mEngineInputData.getConstraintValueName(tMY_CONSTRAINT_INDEX);
        mParameterList->set(tConstraintValueName, &tConstraintValue);

        // ********* Compute constraint value. Meaning, just the evaluation not the actual residual, i.e. h(z)<=0 ********* //
        std::vector<std::string> tStageName;
        std::string tConstraintValueStageName = mEngineInputData.getConstraintValueStageName(tMY_CONSTRAINT_INDEX);
        tStageName.push_back(tConstraintValueStageName);
        mInterface->compute(tStageName, *mParameterList);

        // NOTE: THE CURRENT ASSUMPTION IS THAT THE USER ONLY PROVIDES THE CONSTRAINT EVALUATION.
        // THUS, THE USER IS NOT PROVIDING THE ACUTAL CONSTRAINT RESIDUAL, WHICH IS DEFINED AS
        // RESIDUAL = CONSTRAINT_VALUE - CONSTRAINT_TARGET. THE USER IS JUST PROVIDING THE
        // CONSTRAINT_VALUE.  THE OPTIMALITY CRITERIA OPTIMIZER IS EXPECTING THE CONSTRAINT
        // RESIDUAL AND THUS WE NEED TO DO THE FOLLOWING CALCULATION:
        const ScalarType tConstraintTarget = mEngineInputData.getConstraintNormalizedTargetValue(tMY_CONSTRAINT_INDEX);
        const ScalarType tConstraintReferenceValue = mEngineInputData.getConstraintReferenceValue(tMY_CONSTRAINT_INDEX);
        const ScalarType tOutput = (tConstraintValue / tConstraintReferenceValue) - tConstraintTarget;
        Plato::SerialVectorROL<ScalarType> & tConstraints = dynamic_cast<Plato::SerialVectorROL<ScalarType>&>(aConstraints);
        tConstraints.vector()[tMY_CONSTRAINT_INDEX] = tOutput;
    }

    /******************************************************************************//**
     * \brief Apply vector to constraint Jacobian
     * \param [in/out] aJacobianTimesDirection application of vector to constraint Jacobian
     * \param [in] aDirection descent direction
     * \param [in] aControl design variables
     * \param [in] aTolerance inexactness tolerance
    **********************************************************************************/
    void applyJacobian(ROL::Vector<ScalarType> & aJacobianTimesDirection,
                       const ROL::Vector<ScalarType> & aDirection,
                       const ROL::Vector<ScalarType> & aControl,
                       ScalarType & aTolerance)
    {
        assert(aControl.dimension() == aDirection.dimension());
        assert(aJacobianTimesDirection.dimension() == static_cast<int>(1));

        // ********* Set view to control vector ********* //
        const Plato::DistributedVectorROL<ScalarType> & tControl =
                dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aControl);
        const std::vector<ScalarType> & tControlData = tControl.vector();
        assert(tControlData.size() == mControl.size());
        const size_t tCONTROL_VECTOR_INDEX = 0;
        std::string tControlName = mEngineInputData.getControlName(tCONTROL_VECTOR_INDEX);
        this->copy(tControlData, mControl);
        mParameterList->set(tControlName, mControl.data());

        // ********* Set view to constraint value ********* //
        const size_t tMY_CONSTRAINT_INDEX = 0;
        Teuchos::RCP<ROL::Vector<ScalarType>> tJacobian = aControl.clone();
        Plato::DistributedVectorROL<ScalarType>* tJacobianVector =
                dynamic_cast<Plato::DistributedVectorROL<ScalarType>*>(tJacobian.get());
        std::string tConstraintGradientName = mEngineInputData.getConstraintGradientName(tMY_CONSTRAINT_INDEX);
        std::vector<ScalarType> & tJacobianVectorData = tJacobianVector->vector();
        std::fill(tJacobianVectorData.begin(), tJacobianVectorData.end(), static_cast<ScalarType>(0));
        mParameterList->set(tConstraintGradientName, tJacobianVectorData.data());

        // ********* Compute constraint Jacobian ********* //
        std::vector<std::string> tStageNames;
        std::string tConstraintGradientStageName = mEngineInputData.getConstraintGradientStageName(tMY_CONSTRAINT_INDEX);
        tStageNames.push_back(tConstraintGradientStageName);
        mInterface->compute(tStageNames, *mParameterList);

        // ********* Apply direction to Jacobian ********* //
        const ScalarType tJacobianDotDirection = tJacobianVector->dot(aDirection);
        Plato::SerialVectorROL<ScalarType> & tJacobianTimesDirectionVector =
                dynamic_cast<Plato::SerialVectorROL<ScalarType>&>(aJacobianTimesDirection);
        const ScalarType tConstraintReferenceValue = mEngineInputData.getConstraintReferenceValue(tMY_CONSTRAINT_INDEX);
        tJacobianTimesDirectionVector.vector()[tMY_CONSTRAINT_INDEX] = tJacobianDotDirection/tConstraintReferenceValue;
    }

    /******************************************************************************//**
     * \brief Apply dual vector to constraint Jacobian
     * \param [in/out] aAdjointJacobianTimesDirection application of dual vector to adjoint of constraint Jacobian
     * \param [in] aDual dual vector, i.e. Lagrange multipliers
     * \param [in] aControl design variables
     * \param [in] aTolerance inexactness tolerance
    **********************************************************************************/
    void applyAdjointJacobian(ROL::Vector<ScalarType> & aAdjointJacobianTimesDirection,
                              const ROL::Vector<ScalarType> & aDual,
                              const ROL::Vector<ScalarType> & aControl,
                              ScalarType & aTolerance)
    {
        assert(aDual.dimension() == static_cast<int>(1));
        assert(aAdjointJacobianTimesDirection.dimension() == aControl.dimension());

        // ********* Set view to control vector ********* //
        const Plato::DistributedVectorROL<ScalarType> & tControl =
                dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aControl);
        const std::vector<ScalarType> & tControlData = tControl.vector();
        assert(tControlData.size() == mControl.size());
        const size_t tCONTROL_VECTOR_INDEX = 0;
        std::string tControlName = mEngineInputData.getControlName(tCONTROL_VECTOR_INDEX);
        this->copy(tControlData, mControl);
        mParameterList->set(tControlName, mControl.data());

        // ********* Set view to adjoint Jacobian ********* //
        const size_t tMY_CONSTRAINT_INDEX = 0;
        std::fill(mAdjointJacobian.begin(), mAdjointJacobian.end(), static_cast<ScalarType>(0));
        std::string tConstraintGradientName = mEngineInputData.getConstraintGradientName(tMY_CONSTRAINT_INDEX);
        mParameterList->set(tConstraintGradientName, mAdjointJacobian.data());

        // ********* Compute adjoint Jacobian ********* //
        std::vector<std::string> tStageNames;
        std::string tConstraintGradientStageName = mEngineInputData.getConstraintGradientStageName(tMY_CONSTRAINT_INDEX);
        tStageNames.push_back(tConstraintGradientStageName);
        mInterface->compute(tStageNames, *mParameterList);
        Plato::DistributedVectorROL<ScalarType> & tOutput =
                dynamic_cast<Plato::DistributedVectorROL<ScalarType>&>(aAdjointJacobianTimesDirection);
        this->copy(mAdjointJacobian, tOutput.vector());

        // ********* Apply direction to adjoint Jacobian ********* //
        const Plato::SerialVectorROL<ScalarType> & tDirection =
                dynamic_cast<const Plato::SerialVectorROL<ScalarType>&>(aDual);
        ScalarType tValue = tDirection.vector()[tMY_CONSTRAINT_INDEX];
        const ScalarType tConstraintReferenceValue = mEngineInputData.getConstraintReferenceValue(tMY_CONSTRAINT_INDEX);
        tOutput.scale(tValue/tConstraintReferenceValue);
    }

private:
    /******************************************************************************//**
     * \brief Initialize member data
    **********************************************************************************/
    void initialize()
    {
        const size_t tCONTROL_INDEX = 0;
        std::vector<std::string> tControlNames = mEngineInputData.getControlNames();
        std::string tMyControlName = tControlNames[tCONTROL_INDEX];
        const size_t tNumDesignVariables = mInterface->size(tMyControlName);
        assert(tNumDesignVariables >= static_cast<ScalarType>(0));
        mControl.resize(tNumDesignVariables);
        mAdjointJacobian.resize(tNumDesignVariables);
    }

    /******************************************************************************//**
     * \brief Copy data
     * \param [in] aFrom data to copy
     * \param [in] aTo copied data
    **********************************************************************************/
    void copy(const std::vector<ScalarType> & aFrom, std::vector<ScalarType> & aTo)
    {
        assert(aTo.size() == aFrom.size());
        for(size_t tIndex = 0; tIndex < aFrom.size(); tIndex++)
        {
            aTo[tIndex] = aFrom[tIndex];
        }
    }

private:
    std::vector<ScalarType> mControl; /*!< design variables */
    std::vector<ScalarType> mAdjointJacobian; /*!< adjoint constraint Jacobian */

    Plato::Interface* mInterface; /*!< PLATO Engine interface */
    Plato::OptimizerEngineStageData mEngineInputData; /*!< XML input data */
    std::shared_ptr<Teuchos::ParameterList> mParameterList; /*!< parameter list used in-memory to transfer data through PLATO Engine */

private:
    ReducedConstraintROL(const Plato::ReducedConstraintROL<ScalarType> & aRhs);
    Plato::ReducedConstraintROL<ScalarType> & operator=(const Plato::ReducedConstraintROL<ScalarType> & aRhs);
};
// class ReducedConstraintROL

}
//namespace Plato
