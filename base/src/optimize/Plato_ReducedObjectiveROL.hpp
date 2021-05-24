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
 * Plato_ReducedObjectiveROL.hpp
 *
 *  Created on: Feb 8, 2018
 */

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <cassert>
#include <algorithm>

#include "ROL_Objective.hpp"
#include "ROL_UpdateType.hpp"

#include "Plato_Interface.hpp"
#include "Plato_DistributedVectorROL.hpp"
#include "Plato_OptimizerEngineStageData.hpp"

namespace Plato
{

/******************************************************************************//**
 * \brief PLATO Engine interface to a ROL reduced objective function
 * \tparam scalar type, e.g. double, float, etc.
**********************************************************************************/
template<typename ScalarType>
class ReducedObjectiveROL : public ROL::Objective<ScalarType>
{
public:
    /******************************************************************************//**
     * \brief Constructor
     * \param [in] aInputData XML input data
     * \param [in] aInterface PLATO Engine interface
    **********************************************************************************/
    explicit ReducedObjectiveROL(const Plato::OptimizerEngineStageData & aInputData, Plato::Interface* aInterface = nullptr) :
            mControl(),
            mGradient(),
            mObjectiveValue(0),
            mInterface(aInterface),
            mEngineInputData(aInputData),
            mParameterList(std::make_shared<Teuchos::ParameterList>()),
            mLastIteration(-1),
            mUpdateFrequency(0),
            mStateComputed(false),
            mGradientComputed(false),
            mDebugOutput(false)
    {
        this->initialize();
        if(mDebugOutput)
            mFile.open("obj_output.txt", ios::out);
    }

    /******************************************************************************//**
     * \brief Destructor.
    **********************************************************************************/
    virtual ~ReducedObjectiveROL()
    {
        if(mDebugOutput)
            mFile.close();
    }

    /******************************************************************************//**
     * \brief Updates physics and enables continuation of app-based parameters.
     * \param [in] aControl design variables
     * \param [in] aFlag indicates if vector of design variables was updated
     * \param [in] aIteration outer loop optimization iteration
    **********************************************************************************/
    void update(const ROL::Vector<ScalarType> & aControl, ROL::UpdateType aUpdateType, int aIteration = -1)
    //void update(const ROL::Vector<ScalarType> & aControl, bool aFlag, int aIteration = -1)
    {
//std::cout << "Update called with aUpdateType: " << tUpdateTypeString << " and aIteration: " << aIteration << std::endl;
        if(aUpdateType != ROL::UpdateType::Accept)
            mStateComputed = false;
        mGradientComputed = false;

        bool tNewIteration = aIteration != mLastIteration && aIteration != -1;
        bool tUpdateIteration = mUpdateFrequency > 0 && tNewIteration && aIteration > 0 && aIteration % mUpdateFrequency == 0;

std::string tUpdateTypeString;
if(aUpdateType == ROL::UpdateType::Initial)
    tUpdateTypeString = "Initial";
else if(aUpdateType == ROL::UpdateType::Accept)
    tUpdateTypeString = "Accept";
else if(aUpdateType == ROL::UpdateType::Revert)
    tUpdateTypeString = "Revert";
else if(aUpdateType == ROL::UpdateType::Trial)
    tUpdateTypeString = "Trial";
else if(aUpdateType == ROL::UpdateType::Temp)
    tUpdateTypeString = "Temp";
if(mDebugOutput)
    mFile << "update() called with aUpdateType: " << tUpdateTypeString << " and aIteration: " << aIteration << std::endl;
//std::cout << "Called update: freq=" << mUpdateFrequency << ", iter=" << aIteration << ", last_iter=" << mLastIteration << std::endl << std::flush;
        if(tNewIteration)
        {
//          std::cout << "Calling output stage." << std::endl << std::flush;
          callOutputStage();
        }

        if(tUpdateIteration)
        {
//          std::cout << "Calling update stage." << std::endl << std::flush;
          callUpdateStage();
        }

        mLastIteration = aIteration;
    }

    /******************************************************************************//**
     * \brief Returns current objective value
     * \param [in] aControl design variables
     * \param [in] aTolerance inexactness tolerance
     * \return objective function value
    **********************************************************************************/
    ScalarType value(const ROL::Vector<ScalarType> & aControl, ScalarType & aTolerance)
    {
if(mDebugOutput)
    mFile << "value() called" << std::endl;
        if(!mStateComputed)
            computeValue(aControl);

        return mObjectiveValue;
    }

    /******************************************************************************//**
     * \brief Returns current gradient value
     * \param [out] aGradient objective function gradient
     * \param [in] aControl design variables
     * \param [in] aTolerance inexactness tolerance
    **********************************************************************************/
    void gradient(ROL::Vector<ScalarType> & aGradient, const ROL::Vector<ScalarType> & aControl, ScalarType & aTolerance)
    {
if(mDebugOutput)
    mFile << "gradient() called" << std::endl;
        if(!mStateComputed)
            computeValue(aControl);

        if(!mGradientComputed)
            computeGradient(aControl);

        // ********* Set output gradient vector ********* //
        Plato::DistributedVectorROL<ScalarType> & tOutputGradient =
                dynamic_cast<Plato::DistributedVectorROL<ScalarType>&>(aGradient);
        std::vector<ScalarType> & tOutputGradientData = tOutputGradient.vector();
        this->copy(mGradient, tOutputGradientData);
    }
    /******************************************************************************//**
     * \brief Returns current hessian applied to a vector
     * \param [out] aHessVec objective function Hessian applied to a vector
     * \param [in] aVector design variable direction vector
     * \param [in] aControl design variables
     * \param [in] aTolerance inexactness tolerance
     **********************************************************************************/
    void hessVec(ROL::Vector<ScalarType> & aHessVec, const ROL::Vector<ScalarType> & aVector, const ROL::Vector<ScalarType> & aControl, ScalarType & aTolerance)
    {
if(mDebugOutput)
    mFile << "hessVec() called" << std::endl;
        if(mHessianType == "zero")
        {
            aHessVec.zero(); // Zero
        }
        else if(mHessianType == "finite_difference")
        {
            ROL::Objective<ScalarType>::hessVec(aHessVec,aVector,aControl,aTolerance); // Finite differences
        }
        else
        {
            std::cout << "Error: Unexpected hessian type!" << std::endl;
        }
    }

private:
    /******************************************************************************//**
     * \brief Initialize member data
    **********************************************************************************/
    void initialize()
    {
        assert(mInterface != nullptr);
        const size_t tCONTROL_INDEX = 0;
        std::vector<std::string> tControlNames = mEngineInputData.getControlNames();
        std::string tMyControlName = tControlNames[tCONTROL_INDEX];
        const size_t tNumDesignVariables = mInterface->size(tMyControlName);
        assert(tNumDesignVariables >= static_cast<ScalarType>(0));
        mControl.resize(tNumDesignVariables);
        mGradient.resize(tNumDesignVariables);
        mUpdateFrequency = mEngineInputData.getProblemUpdateFrequency();
        mHessianType = mEngineInputData.getHessianType();
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

    void computeValue(const ROL::Vector<ScalarType> & aControl)
    {
if(mDebugOutput)
    mFile << "  computeValue() called" << std::endl;
      // ********* Set view to control vector ********* //
      const Plato::DistributedVectorROL<ScalarType> & tControl =
              dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aControl);
      const std::vector<ScalarType> & tControlData = tControl.vector();
      assert(tControlData.size() == mControl.size());
      const size_t tCONTROL_VECTOR_INDEX = 0;
      std::string tControlName = mEngineInputData.getControlName(tCONTROL_VECTOR_INDEX);
      this->copy(tControlData, mControl);
      mParameterList->set(tControlName, mControl.data());

      // ********* Set view to objective function value ********* //
      std::string tObjectiveValueOutputName = mEngineInputData.getObjectiveValueOutputName();
      mParameterList->set(tObjectiveValueOutputName, &mObjectiveValue);

      // ********* Compute objective function value ********* //
      std::vector<std::string> tStageNames;
      std::string tObjectiveValueStageName = mEngineInputData.getObjectiveValueStageName();
      tStageNames.push_back(tObjectiveValueStageName);
      mInterface->compute(tStageNames, *mParameterList);

      cacheState();
    }

    void computeGradient(const ROL::Vector<ScalarType> & aControl)
    {
if(mDebugOutput)
    mFile << "  computeGradient() called" << std::endl;
      // ********* Set view to control vector ********* //
      const Plato::DistributedVectorROL<ScalarType> & tControl =
              dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aControl);
      const std::vector<ScalarType> & tControlData = tControl.vector();
      assert(tControlData.size() == mControl.size());
      const size_t tCONTROL_VECTOR_INDEX = 0;
      std::string tControlName = mEngineInputData.getControlName(tCONTROL_VECTOR_INDEX);
      this->copy(tControlData, mControl);
      mParameterList->set(tControlName, mControl.data());

      // ********* Set view to gradient vector ********* //
      std::string tObjectiveGradientOutputName = mEngineInputData.getObjectiveGradientOutputName();
      std::fill(mGradient.begin(), mGradient.end(), static_cast<ScalarType>(0));
      mParameterList->set(tObjectiveGradientOutputName, mGradient.data());

      // ********* Compute gradient vector ********* //
      std::vector<std::string> tStageNames;
      std::string tObjectiveGradientStageName = mEngineInputData.getObjectiveGradientStageName();
      tStageNames.push_back(tObjectiveGradientStageName);
      mInterface->compute(tStageNames, *mParameterList);

      mGradientComputed = true;
    }

    void callOutputStage()
    {
      std::vector<std::string> tStageNames;
      std::string tOutputStageName = mEngineInputData.getOutputStageName();
      if(tOutputStageName.empty() == false)
      {
          tStageNames.push_back(tOutputStageName);
          mInterface->compute(tStageNames, *mParameterList);
      }
    }

    void callUpdateStage()
    {
if(mDebugOutput)
    mFile << "  callUpdateStage() called" << std::endl;
      std::vector<std::string> tStageNames;
      std::string tUpdateStageName = mEngineInputData.getUpdateProblemStageName();
      if(tUpdateStageName.empty() == false)
      {
        tStageNames.push_back(tUpdateStageName);
        mInterface->compute(tStageNames, *mParameterList);
      }
    }

    void cacheState()
    {
if(mDebugOutput)
    mFile << "  cacheState() called" << std::endl;
        std::vector<std::string> tStageNames;
        std::string tCacheStageName = mEngineInputData.getCacheStageName();
        if(tCacheStageName.empty() == false)
        {
            tStageNames.push_back(tCacheStageName);
            mInterface->compute(tStageNames, *mParameterList);
        }

        mStateComputed = true;
    }

private:
    std::vector<ScalarType> mControl; /*!< design variables */
    std::vector<ScalarType> mGradient; /*!< objective function gradient */
    ScalarType mObjectiveValue;

    Plato::Interface* mInterface; /*!< PLATO Engine interface */
    Plato::OptimizerEngineStageData mEngineInputData; /*!< XML input data */
    std::shared_ptr<Teuchos::ParameterList> mParameterList; /*!< parameter list used in-memory to transfer data through PLATO Engine */

    int mLastIteration;
    int mUpdateFrequency;

    bool mStateComputed;
    bool mGradientComputed;
    bool mDebugOutput;
    std::string mHessianType;

private:
    ReducedObjectiveROL(const Plato::ReducedObjectiveROL<ScalarType> & aRhs);
    Plato::ReducedObjectiveROL<ScalarType> & operator=(const Plato::ReducedObjectiveROL<ScalarType> & aRhs);
    ofstream mFile;
};
// class ReducedObjectiveROL

}
// namespace Plato
