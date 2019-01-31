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
 * Plato_ProxyAppALPSO.hpp
 *
 *  Created on: Jan 30, 2019
 */

#pragma once

#include <cstdio>
#include <sstream>
#include <algorithm>

#include "Plato_Application.hpp"
#include "Plato_StandardVector.hpp"
#include "Plato_GradFreeHimmelblau.hpp"
#include "Plato_StandardMultiVector.hpp"
#include "Plato_GradFreeShiftedEllipse.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Proxy application to test Augmented Lagrangian Particle Swarm Optimization (ALPSO) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class ProxyAppALPSO : public Plato::Application
{
public:
    /******************************************************************************//**
     * @brief Constructor for ALPSO Proxy application
     **********************************************************************************/
    ProxyAppALPSO() :
            mNumControls(2),
            mNumParticles(1),
            mArgumentNames(),
            mOperationNames(),
            mObjFuncVal(),
            mConstraintVal(),
            mParticle(),
            mObjective(std::make_shared<Plato::GradFreeHimmelblau<ScalarType, OrdinalType>>()),
            mConstraint(std::make_shared<Plato::GradFreeShiftedEllipse<ScalarType, OrdinalType>>())
    {
        this->build();
    }

    /******************************************************************************//**
     * @brief Destructor for ALPSO Proxy application
    **********************************************************************************/
    virtual ~ProxyAppALPSO()
    {
    }

    /******************************************************************************//**
     * @brief Deallocate memory
    **********************************************************************************/
    void finalize()
    {
        return;         // NO MEMORY DEALLOCATION NEEDED
    }

    /******************************************************************************//**
     * @brief Allocate memory
    **********************************************************************************/
    void initialize()
    {
        return;        // NO MEMORY ALLOCATION NEEDED
    }

    /******************************************************************************//**
     * @brief Perform operation
     * @param [in] aOperationName name of operation to perform
    **********************************************************************************/
    void compute(const std::string & aOperationName)
    {
        this->checkOperationName(aOperationName);
        if(aOperationName.compare("ObjFuncEval") == static_cast<int>(0))
        {
            mObjective->value(*mParticle, *mObjFuncVal);
        }
        if(aOperationName.compare("ConstraintEval") == static_cast<int>(0))
        {
            mConstraint->value(*mParticle, *mConstraintVal);
        }
    }

    /******************************************************************************//**
     * @brief Export data from application
     * @param [in] aArgumentName name of data to be exported from application
     * @param [out] aExportData PLATO Engine container to put application data
    **********************************************************************************/
    void exportData(const std::string & aArgumentName, Plato::SharedData & aExportData)
    {
        switch(aExportData.myLayout())
        {
            case Plato::data::layout_t::SCALAR:
            {
                this->outputData(aArgumentName, aExportData);
                break;
            }
            default:
            case Plato::data::layout_t::SCALAR_FIELD:
            case Plato::data::layout_t::VECTOR_FIELD:
            case Plato::data::layout_t::TENSOR_FIELD:
            case Plato::data::layout_t::ELEMENT_FIELD:
            {
                std::ostringstream tErrorMsg;
                tErrorMsg << "\n\n*********\n ERROR IN " << __FILE__ << "\n FUNCTION: " << __PRETTY_FUNCTION__ << ", LINE: "
                          << __LINE__ << "\n, MESSAGE: DATA LAYOUT IS NOT SUPPORTED BY APP. ABORT!\n *********\n\n";
                std::perror(tErrorMsg.str().c_str());
                std::abort();
                break;
            }
        }
    }

    /******************************************************************************//**
     * @brief Import data from PLATO Engine to application
     * @param [in] aArgumentName name of data to be imported from PLATO Engine
     * @param [in] aExportData PLATO Engine container with data
    **********************************************************************************/
    void importData(const std::string & aArgumentName, const Plato::SharedData & aImportData)
    {
        switch(aImportData.myLayout())
        {
            case Plato::data::layout_t::SCALAR:
            case Plato::data::layout_t::SCALAR_FIELD:
            {
                this->inputData(aArgumentName, aImportData);
                break;
            }
            default:
            case Plato::data::layout_t::VECTOR_FIELD:
            case Plato::data::layout_t::TENSOR_FIELD:
            case Plato::data::layout_t::ELEMENT_FIELD:
            {
                std::ostringstream tErrorMsg;
                tErrorMsg << "\n\n*********\n ERROR IN " << __FILE__ << "\n FUNCTION: " << __PRETTY_FUNCTION__ << "\n LINE: "
                          << __LINE__ << "\n MESSAGE: DATA LAYOUT IS NOT SUPPORTED BY APP. ABORT! *********\n\n";
                std::perror(tErrorMsg.str().c_str());
                std::abort();
                break;
            }
        }
    }

    /******************************************************************************//**
     * @brief Export parallel map
     * @param [in] aDataLayout data layout (i.e. node-based or element-based data)
     * @param [out] aMyOwnedGlobalIDs global IDs owned by this processor
    **********************************************************************************/
    void exportDataMap(const Plato::data::layout_t & aDataLayout, std::vector<int> & aMyOwnedGlobalIDs)
    {
        /************************************************************************************
          CONTROLS ARE NOT DISTRIBUTED ACROSS PROCESSORS. HENCE, PARALLEL MAP IS NULL.
        ************************************************************************************/
        return;
    }

private:
    /******************************************************************************//**
     * @brief Build class member containers
    **********************************************************************************/
    void build()
    {
        mArgumentNames.push_back("Particle");
        mArgumentNames.push_back("ObjFuncVal");
        mArgumentNames.push_back("ConstraintVal");

        mOperationNames.push_back("ObjFuncEval");
        mOperationNames.push_back("ConstraintEval");

        mConstraint->define(-2. /* x-center */, 2. /* x-radius */, -3. /* y-center */, 3. /* y-radius */);
        mObjFuncVal =  std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(mNumParticles);
        mConstraintVal =  std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(mNumParticles);
        mParticle =  std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(mNumParticles, mNumControls);
    }

    /******************************************************************************//**
     * @brief Check if argument is defined.
     * @param [in] aArgumentName argument name
    **********************************************************************************/
    void checkArgumentName(const std::string & aArgumentName)
    {
        if(std::find(mArgumentNames.begin(), mArgumentNames.end(), aArgumentName) == mArgumentNames.end())
        {
            std::ostringstream tErrorMsg;
            tErrorMsg << "\n\n*********\n ERROR IN " << __FILE__ << "\n FUNCTION: " << __PRETTY_FUNCTION__ << "\n LINE: "
                      << __LINE__ << "\n MESSAGE: IMPORT ARGUMENT NAME = '" << aArgumentName.c_str()
                      << "' IS NOT DEFINE IN APP. ABORT!\n *********\n\n";
            std::perror(tErrorMsg.str().c_str());
            std::abort();
        }
    }

    /******************************************************************************//**
     * @brief Check if operation name is defined.
     * @param [in] aOperationName operation name
    **********************************************************************************/
    void checkOperationName(const std::string & aOperationName)
    {
        if(std::find(mOperationNames.begin(), mOperationNames.end(), aOperationName) == mOperationNames.end())
        {
            std::ostringstream tErrorMsg;
            tErrorMsg << "\n\n*********\n ERROR IN " << __FILE__ << "\n FUNCTION: " << __PRETTY_FUNCTION__ << "\n LINE: "
                      << __LINE__ << "\n MESSAGE: IMPORT ARGUMENT NAME = '" << aOperationName.c_str()
                      << "' IS NOT DEFINE IN APP. ABORT!\n *********\n\n";
            std::perror(tErrorMsg.str().c_str());
            std::abort();
        }
    }

    /******************************************************************************//**
     * @brief Copy data into PLATO Engine shared data container
     * @param [in] aArgumentName name of data
     * @param [in] aImportData PLATO Engine shared data container
    **********************************************************************************/
    void inputData(const std::string & aArgumentName, const Plato::SharedData & aImportData)
    {
        this->checkArgumentName(aArgumentName);
        Plato::StandardVector<ScalarType, OrdinalType> tData(mNumControls);
        aImportData.getData(tData.vector());
        const OrdinalType tPARTICLE_INDEX = 0;
        (*mParticle)[tPARTICLE_INDEX].update(static_cast<ScalarType>(1), tData, static_cast<ScalarType>(0));
    }

    /******************************************************************************//**
     * @brief Extract data from PLATO Engine shared data container into app-owned container
     * @param [in] aArgumentName name of data
     * @param [in] aExportData PLATO Engine shared data container
    **********************************************************************************/
    void outputData(const std::string & aArgumentName, Plato::SharedData & aExportData)
    {
        this->checkArgumentName(aArgumentName);
        if(aArgumentName.compare("ObjFuncVal") == static_cast<int>(0))
        {
            aExportData.setData(mObjFuncVal->vector());
        }
        if(aArgumentName.compare("ConstraintVal") == static_cast<int>(0))
        {
            aExportData.setData(mConstraintVal->vector());
        }
    }

private:
    OrdinalType mNumControls; /*!< number of controls */
    OrdinalType mNumParticles; /*!< number of particles */
    std::vector<std::string> mArgumentNames; /*!< argument names defined by application */
    std::vector<std::string> mOperationNames; /*!< operation names defined by application */

    std::shared_ptr<Plato::StandardVector<ScalarType, OrdinalType>> mObjFuncVal; /*!< objective function value */
    std::shared_ptr<Plato::StandardVector<ScalarType, OrdinalType>> mConstraintVal; /*!< constraint value */
    std::shared_ptr<Plato::StandardMultiVector<ScalarType, OrdinalType>> mParticle; /*!< control container */

    std::shared_ptr<Plato::GradFreeHimmelblau<ScalarType, OrdinalType>> mObjective; /*!< gradient free objective function interface */
    std::shared_ptr<Plato::GradFreeShiftedEllipse<ScalarType, OrdinalType>> mConstraint; /*!< gradient free constraint interface */

private:
    ProxyAppALPSO(const Plato::ProxyAppALPSO<ScalarType, OrdinalType> & aRhs);
    Plato::ProxyAppALPSO<ScalarType, OrdinalType> & operator=(const Plato::ProxyAppALPSO<ScalarType, OrdinalType> & aRhs);
};
// class ProxyAppALPSO

}
// namespace Plato
