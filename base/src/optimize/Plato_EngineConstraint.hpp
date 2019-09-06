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
 * Plato_EngineConstraint.hpp
 *
 *  Created on: Dec 21, 2017
 */

#pragma once

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
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aConstraintID constraint identifier
     * @param [in] aDataFactory PLATO data factory
     * @param [in] aInputData PLATO Engine input data
     * @param [in] aInterface PLATO Engine interface
    **********************************************************************************/
    explicit EngineConstraint(const OrdinalType & aConstraintID,
                              const  Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory,
                              const Plato::OptimizerEngineStageData & aInputData,
                              Plato::Interface* aInterface) :
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

    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aConstraintID constraint identifier
     * @param [in] aInputData PLATO Engine input data
     * @param [in] aInterface PLATO Engine interface
    **********************************************************************************/
    explicit EngineConstraint(const OrdinalType & aConstraintID,
                              const Plato::OptimizerEngineStageData & aInputData,
                              Plato::Interface* aInterface) :
            mMyConstraintID(aConstraintID),
            mVector(),
            mControl(),
            mGradient(),
            mHessianTimesVector(),
            mInterface(aInterface),
            mEngineInputData(aInputData),
            mParameterList(std::make_shared<Teuchos::ParameterList>())
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~EngineConstraint()
    {
    }

    /******************************************************************************//**
     * @brief Allocate control containers needed during optimization
     * @param [in] aNumControls number of controls
    **********************************************************************************/
    void allocateControlContainers(const OrdinalType& aNumControls)
    {
        mVector = std::vector<ScalarType>(aNumControls);
        mControl = std::vector<ScalarType>(aNumControls);
        mGradient = std::vector<ScalarType>(aNumControls);
        mHessianTimesVector = std::vector<ScalarType>(aNumControls);
    }

    /******************************************************************************//**
     * @brief Set PLATO Engine interface
     * @param [in] aInterface PLATO Engine interface
    **********************************************************************************/
    void setPlatoInterface(Plato::Interface* aInterface)
    {
        assert(aInterface != nullptr);
        mInterface = aInterface;
    }

    /******************************************************************************//**
     * @brief Directive to third-party applications to cache app-based data
    **********************************************************************************/
    void cacheData()
    {
        return;
    }

    /******************************************************************************//**
     * @brief Evaluate one or multiple third-party application constraints
     * @param [in] aControl const reference to 2D container of optimization variables
    **********************************************************************************/
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
        std::string tMyStageName = mEngineInputData.getConstraintValueStageName(mMyConstraintID);
        assert(tMyStageName.empty() == false);
        std::vector<std::string> tStageNames;
        tStageNames.push_back(tMyStageName);
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

    /******************************************************************************//**
     * @brief Compute the gradient of one or multiple third-party application constraints
     * @param [in] aControl const reference to 2D container of optimization variables
     * @param [in] aOutput reference to 2D container of the gradient with respect to the
     *                     optimization variables
    **********************************************************************************/
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
        std::string tMyStageName = mEngineInputData.getConstraintGradientStageName(mMyConstraintID);
        assert(tMyStageName.empty() == false);
        std::vector<std::string> tStageNames;
        tStageNames.push_back(tMyStageName);
        mInterface->compute(tStageNames, *mParameterList);
        this->copy(mGradient, aOutput);

        const ScalarType tConstraintReferenceValue = mEngineInputData.getConstraintReferenceValue(mMyConstraintID);
        Plato::scale(static_cast<ScalarType>(1. / tConstraintReferenceValue), aOutput);
    }

    /******************************************************************************//**
     * @brief Compute the application of a vector to the Hessian of one or multiple
     *        third-party application constraints
     * @param [in] aControl const reference to 2D container of optimization variables
     * @param [in] aVector const reference to 2D container of descent directions
     * @param [in] aOutput reference to 2D container of the application of a vector
     *                     to the Hessian
    **********************************************************************************/
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
        std::string tMyStageName = mEngineInputData.getConstraintHessianStageName(mMyConstraintID);
        assert(tMyStageName.empty() == false);
        std::vector<std::string> tStageNames;
        tStageNames.push_back(tMyStageName);
        mInterface->compute(tStageNames, *mParameterList);
        this->copy(mHessianTimesVector, aOutput);

        const ScalarType tConstraintReferenceValue = mEngineInputData.getConstraintReferenceValue(mMyConstraintID);
        Plato::scale(static_cast<ScalarType>(1. / tConstraintReferenceValue), aOutput);
    }

private:
    /******************************************************************************//**
     * @brief Copy elements in standard vector into a Plato multi-vector
     * @param [in] aFrom input data
     * @param [out] aTo output data
    **********************************************************************************/
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
    OrdinalType mMyConstraintID; /*!< constraint identifier */

    std::vector<ScalarType> mVector; /*!< local copy of a descent direction vector */
    std::vector<ScalarType> mControl; /*!< local copy of the control variables */
    std::vector<ScalarType> mGradient; /*!< local copy of the gradient vector */
    std::vector<ScalarType> mHessianTimesVector;/*!< local copy of the application of a vector to the Hessian */

    Plato::Interface* mInterface; /*!< PLATO Engine interface */
    Plato::OptimizerEngineStageData mEngineInputData; /*!< Parsed input data */
    std::shared_ptr<Teuchos::ParameterList> mParameterList; /*!< parameter list with data to be communicated through the PLATO Engine interface */

private:
    EngineConstraint(const Plato::EngineConstraint<ScalarType, OrdinalType>&);
    Plato::EngineConstraint<ScalarType, OrdinalType> & operator=(const Plato::EngineConstraint<ScalarType, OrdinalType>&);
};
// class EngineConstraint

}
// namespace Plato
