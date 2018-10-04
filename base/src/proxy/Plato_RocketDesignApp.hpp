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
 * Plato_RocketDesignApp.hpp
 *
 *  Created on: Sep 28, 2018
 */

#pragma once

#include <map>
#include <vector>
#include <memory>
#include <string>

#include "Plato_Application.hpp"
#include "Plato_GradBasedRocketObjective.hpp"

namespace Plato
{

class SharedData;

class RocketDesignApp : public Plato::Application
{
public:
    /******************************************************************************//**
     * @brief Default constructor
    **********************************************************************************/
    RocketDesignApp();

    /******************************************************************************//**
     * @brief Constructor
    **********************************************************************************/
    RocketDesignApp(int aArgc, char **aArgv);

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~RocketDesignApp();

    /******************************************************************************//**
     * @brief Deallocate memory
    **********************************************************************************/
    void finalize();

    /******************************************************************************//**
     * @brief Allocate memory
    **********************************************************************************/
    void initialize();

    /******************************************************************************//**
     * @brief Perform an operation, e.g. evaluate objective function
     * @param [in] aOperationName name of operation
    **********************************************************************************/
    void compute(const std::string & aOperationName);

    /******************************************************************************//**
     * @brief Export data from user's application
     * @param [in] aArgumentName name of export data (e.g. objective gradient)
     * @param [out] aExportData container used to store output data
    **********************************************************************************/
    void exportData(const std::string & aArgumentName, Plato::SharedData & aExportData);

    /******************************************************************************//**
     * @brief Import data from Plato to user's application
     * @param [in] aArgumentName name of import data (e.g. design variables)
     * @param [in] aImportData container with import data
    **********************************************************************************/
    void importData(const std::string & aArgumentName, const Plato::SharedData & aImportData);

    /******************************************************************************//**
     * @brief Export distributed memory graph
     * @param [in] aDataLayout data layout (options: SCALAR, SCALAR_FIELD, VECTOR_FIELD,
     *                         TENSOR_FIELD, ELEMENT_FIELD, SCALAR_PARAMETER)
     * @param [out] aMyOwnedGlobalIDs my processor's global IDs
    **********************************************************************************/
    void exportDataMap(const Plato::data::layout_t & aDataLayout, std::vector<int> & aMyOwnedGlobalIDs);

private:
    /******************************************************************************//**
     * @brief Set output shared data container
     * @param [in] aArgumentName export data name (e.g. objective gradient)
     * @param [out] aExportData export shared data container
    **********************************************************************************/
    void outputData(const std::string & aArgumentName, Plato::SharedData & aExportData);

    /******************************************************************************//**
     * @brief Set input shared data container
     * @param [in] aArgumentName name of import data (e.g. design variables)
     * @param [in] aImportData import shared data container
    **********************************************************************************/
    void inputData(const std::string & aArgumentName, const Plato::SharedData & aImportData);

    /******************************************************************************//**
     * @brief Perform valid application-based operation.
     * @param [in] aOperationName name of operation
    **********************************************************************************/
    void performOperation(const std::string & aOperationName);

    /******************************************************************************//**
     * @brief Define valid application-based operations
    **********************************************************************************/
    void defineOperations();

    /******************************************************************************//**
     * @brief Define valid application-based shared data containers and layouts
    **********************************************************************************/
    void defineSharedDataMaps();

    /******************************************************************************//**
     * @brief Evaluate objective function
    **********************************************************************************/
    void evaluateObjective();

    /******************************************************************************//**
     * @brief Compute objective gradient
    **********************************************************************************/
    void computeObjectiveGradient();

    /******************************************************************************//**
     * @brief Set normalization constants for objective function
    **********************************************************************************/
    void setNormalizationConstants();

private:
    size_t mNumDesigVariables; /*!< import/export parameter map */

    std::shared_ptr<Plato::GradBasedRocketObjective<double>> mObjective; /*!< rocket design problem objective */
    std::vector<std::string> mDefinedOperations; /*!< valid operations recognized by app */
    std::map<std::string, std::vector<double>> mSharedDataMap; /*!< import/export shared data map */
    std::map<std::string, Plato::data::layout_t> mDefinedDataLayout; /*!< valid data layouts */

private:
    RocketDesignApp(const Plato::RocketDesignApp & aRhs);
    Plato::RocketDesignApp & operator=(const Plato::RocketDesignApp & aRhs);
};
// class RocketDesignApp

} // namespace Plato
