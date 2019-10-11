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
            mInterface(aInterface),
            mEngineInputData(aInputData),
            mParameterList(std::make_shared<Teuchos::ParameterList>()),
            mStateHasBeenCalculated(false),
            mLastIteration(-1)
    {
        this->initialize();
    }

    /******************************************************************************//**
     * \brief Destructor.
    **********************************************************************************/
    virtual ~ReducedObjectiveROL()
    {
    }

    /******************************************************************************//**
     * \brief Enables continuation of app-based parameters.
     * \param [in] aControl design variables
     * \param [in] aFlag indicates if vector of design variables was updated
     * \param [in] aIteration outer loop optimization iteration
    **********************************************************************************/
    void update(const ROL::Vector<ScalarType> & aControl, bool aFlag, int aIteration = -1)
    {
      printf("flag: %d, iteration: %d \n", aFlag, aIteration);
        if(aIteration != mLastIteration && aIteration != -1)
        {
          assert(mInterface != nullptr);
          std::vector<std::string> tStageNames;
          std::string tOutputStageName = mEngineInputData.getOutputStageName();
          tStageNames.push_back(tOutputStageName);
          mInterface->compute(tStageNames, *mParameterList);
        }

        if(aIteration != mLastIteration && aIteration > 0 && aIteration % mEngineInputData.getProblemUpdateFrequency() == 0)
        {
          std::vector<std::string> tStageNames;
          std::string tUpdateStageName = mEngineInputData.getUpdateProblemStageName();
          tStageNames.push_back(tUpdateStageName);
          mInterface->compute(tStageNames, *mParameterList);
        }

        mLastIteration = aIteration;
    }

    /******************************************************************************//**
     * \brief Evaluate objective function
     * \param [in] aControl design variables
     * \param [in] aTolerance inexactness tolerance
     * \return objective function value
    **********************************************************************************/
    ScalarType value(const ROL::Vector<ScalarType> & aControl, ScalarType & aTolerance)
    {
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
        ScalarType tObjectiveValue = 0;
        std::string tObjectiveValueName = mEngineInputData.getObjectiveValueOutputName();
        mParameterList->set(tObjectiveValueName, &tObjectiveValue);

        // ********* Compute objective function value ********* //
        std::vector<std::string> tStageNames;
        tStageNames.push_back(tObjectiveValueName);
        mInterface->compute(tStageNames, *mParameterList);

        return (tObjectiveValue);
    }

    /******************************************************************************//**
     * \brief Evaluate objective function gradient
     * \param [in\out] aGradient objective function gradient
     * \param [in] aControl design variables
     * \param [in] aTolerance inexactness tolerance
    **********************************************************************************/
    void gradient(ROL::Vector<ScalarType> & aGradient, const ROL::Vector<ScalarType> & aControl, ScalarType & aTolerance)
    {
        assert(aControl.dimension() == aGradient.dimension());

        if(!mStateHasBeenCalculated)
        {
            mStateHasBeenCalculated = true;
            value(aControl, aTolerance);
        }

        // Tell performers to cache the state
        std::vector<std::string> tStageNames;
        std::string tCacheStageName = mEngineInputData.getCacheStageName();
        if(tCacheStageName.empty() == false)
        {
            tStageNames.push_back(tCacheStageName);
            mInterface->compute(tStageNames, *mParameterList);
        }

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
        std::string tObjectiveGradientName = mEngineInputData.getObjectiveGradientOutputName();
        std::fill(mGradient.begin(), mGradient.end(), static_cast<ScalarType>(0));
        mParameterList->set(tObjectiveGradientName, mGradient.data());

        // ********* Compute gradient vector ********* //
        tStageNames.clear();
        tStageNames.push_back(tObjectiveGradientName);
        mInterface->compute(tStageNames, *mParameterList);

        // ********* Set output gradient vector ********* //
        Plato::DistributedVectorROL<ScalarType> & tOutputGradient =
                dynamic_cast<Plato::DistributedVectorROL<ScalarType>&>(aGradient);
        std::vector<ScalarType> & tOutputGradientData = tOutputGradient.vector();
        this->copy(mGradient, tOutputGradientData);
    }

    /******************************************************************************//**
     * \brief Apply descent direction vector to objective function Hessian
     * \param [in\out] aOutput application of descent direction vector to objective function Hessian
     * \param [in] aVector descent direction
     * \param [in] aControl design variables
     * \param [in] aTolerance inexactness tolerance
    **********************************************************************************/
//    void hessVec(ROL::Vector<ScalarType> & aOutput,
//                 const ROL::Vector<ScalarType> & aVector,
//                 const ROL::Vector<ScalarType> & aControl,
//                 ScalarType & aTolerance)
/*
    {
        assert(aVector.dimension() == aOutput.dimension());
        assert(aControl.dimension() == aOutput.dimension());
        aOutput.set(aVector);
    }
*/

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
        mGradient.resize(tNumDesignVariables);
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
    std::vector<ScalarType> mGradient; /*!< objective function gradient */

    Plato::Interface* mInterface; /*!< PLATO Engine interface */
    Plato::OptimizerEngineStageData mEngineInputData; /*!< XML input data */
    std::shared_ptr<Teuchos::ParameterList> mParameterList; /*!< parameter list used in-memory to transfer data through PLATO Engine */

    bool mStateHasBeenCalculated; /*!< flag - indicates if state has been computed */

    int mLastIteration;

private:
    ReducedObjectiveROL(const Plato::ReducedObjectiveROL<ScalarType> & aRhs);
    Plato::ReducedObjectiveROL<ScalarType> & operator=(const Plato::ReducedObjectiveROL<ScalarType> & aRhs);
};
// class ReducedObjectiveROL

}
// namespace Plato
