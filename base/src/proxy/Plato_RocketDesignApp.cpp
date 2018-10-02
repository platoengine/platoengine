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
 * Plato_RocketDesignApp.cpp
 *
 *  Created on: Sep 28, 2018
 */

#include <cassert>
#include <sstream>
#include <stdexcept>

#include "Plato_SharedData.hpp"
#include "Plato_ErrorChecks.hpp"
#include "Plato_AppErrorChecks.hpp"
#include "Plato_RocketDesignApp.hpp"
#include "Plato_StandardMultiVector.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Default constructor
**********************************************************************************/
RocketDesignApp::RocketDesignApp() :
        mNumDesigVariables(2),
        mObjective(),
        mDefinedOperations(),
        mSharedDataMap(),
        mDefinedDataLayout()
{
}

/******************************************************************************//**
 * @brief Constructor
**********************************************************************************/
RocketDesignApp::RocketDesignApp(int aArgc, char **aArgv) :
        mNumDesigVariables(2),
        mObjective(),
        mDefinedOperations(),
        mSharedDataMap(),
        mDefinedDataLayout()
{
}

/******************************************************************************//**
 * @brief Destructor
**********************************************************************************/
RocketDesignApp::~RocketDesignApp()
{
}

/******************************************************************************//**
 * @brief Deallocate memory
**********************************************************************************/
void RocketDesignApp::finalize()
{
    // MEMORY MANAGEMENT AUTOMATED, NO NEED TO EXPLICITLY DEALLOCATE MEMORY
    return;
}

/******************************************************************************//**
 * @brief Allocate memory
**********************************************************************************/
void RocketDesignApp::initialize()
{
    this->defineOperations();
    this->defineSharedDataMaps();

    Plato::AlgebraicRocketInputs<double> tRocketSimInputs;
    std::shared_ptr<Plato::GeometryModel<double>> tGeomModel =
            std::make_shared<Plato::Cylinder<double>>(tRocketSimInputs.mChamberRadius, tRocketSimInputs.mChamberLength);
    mObjective = std::make_shared<Plato::SimpleRocketObjective<double>>(tRocketSimInputs, tGeomModel);
}

/******************************************************************************//**
 * @brief Perform an operation, e.g. evaluate objective function
 * @param [in] aOperationName name of operation
**********************************************************************************/
void RocketDesignApp::compute(const std::string & aOperationName)
{
    try
    {
        Plato::check_operation_definition(aOperationName, mDefinedOperations);
        this->performOperation(aOperationName);
    }
    catch(const std::invalid_argument & tErrorMsg)
    {
        std::ostringstream tFullErrorMsg;
        tFullErrorMsg << "\n\n ********\n ERROR IN FILE: " << __FILE__ << "\n FUNCTION: " << __PRETTY_FUNCTION__
                << "\n LINE: " << __LINE__ << "\n ******** \n\n";
        tFullErrorMsg << tErrorMsg.what();
        std::cout << tFullErrorMsg.str().c_str();
        throw std::invalid_argument(tFullErrorMsg.str().c_str());
    }
}

/******************************************************************************//**
 * @brief Export data from user's application
 * @param [in] aArgumentName name of export data (e.g. objective gradient)
 * @param [out] aExportData container used to store output data
**********************************************************************************/
void RocketDesignApp::exportData(const std::string & aArgumentName, Plato::SharedData & aExportData)
{
    try
    {
        Plato::check_shared_data_argument_definition(aArgumentName, mSharedDataMap);
        Plato::check_shared_data_argument_definition(aArgumentName, mDefinedDataLayout);
        Plato::check_data_layout(mDefinedDataLayout.find(aArgumentName)->second, aExportData.myLayout());
        this->outputData(aArgumentName, aExportData);
    }
    catch(const std::invalid_argument & tErrorMsg)
    {
        std::ostringstream tFullErrorMsg;
        tFullErrorMsg << "\n\n ********\n ERROR IN FILE: " << __FILE__ << "\n FUNCTION: " << __PRETTY_FUNCTION__
                << "\n LINE: " << __LINE__ << "\n ******** \n\n";
        tFullErrorMsg << tErrorMsg.what();
        std::cout << tFullErrorMsg.str().c_str();
        throw std::invalid_argument(tFullErrorMsg.str().c_str());
    }
}

/******************************************************************************//**
 * @brief Import data from Plato to user's application
 * @param [in] aArgumentName name of import data (e.g. design variables)
 * @param [in] aImportData container with import data
**********************************************************************************/
void RocketDesignApp::importData(const std::string & aArgumentName, const Plato::SharedData & aImportData)
{
    try
    {
        Plato::check_shared_data_argument_definition(aArgumentName, mSharedDataMap);
        Plato::check_shared_data_argument_definition(aArgumentName, mDefinedDataLayout);
        Plato::check_data_layout(mDefinedDataLayout.find(aArgumentName)->second, aImportData.myLayout());
        this->inputData(aArgumentName, aImportData);
    }
    catch(const std::invalid_argument & tErrorMsg)
    {
        std::ostringstream tFullErrorMsg;
        tFullErrorMsg << "\n\n ********\n ERROR IN FILE: " << __FILE__ << "\n FUNCTION: " << __PRETTY_FUNCTION__
                << "\n LINE: " << __LINE__ << "\n ******** \n\n";
        tFullErrorMsg << tErrorMsg.what();
        std::cout << tFullErrorMsg.str().c_str();
        throw std::invalid_argument(tFullErrorMsg.str().c_str());
    }
}

/******************************************************************************//**
 * @brief Export distributed memory graph
 * @param [in] aDataLayout data layout (options: SCALAR, SCALAR_FIELD, VECTOR_FIELD,
 *                         TENSOR_FIELD, ELEMENT_FIELD, SCALAR_PARAMETER)
 * @param [out] aMyOwnedGlobalIDs my processor's global IDs
**********************************************************************************/
void RocketDesignApp::exportDataMap(const Plato::data::layout_t & aDataLayout, std::vector<int> & aMyOwnedGlobalIDs)
{
    // THIS IS NOT A DISTRIBUTED MEMORY EXAMPLE; HENCE, THE DISTRIBUTED MEMORY GRAPH IS NOT NEEDEDS
    return;
}

/******************************************************************************//**
 * @brief Set output shared data container
 * @param [in] aArgumentName export data name (e.g. objective gradient)
 * @param [out] aExportData export shared data container
**********************************************************************************/
void RocketDesignApp::outputData(const std::string & aArgumentName, Plato::SharedData & aExportData)
{
    try
    {
        auto tIterator = mSharedDataMap.find(aArgumentName);
        std::vector<double> & tOutputData = tIterator->second;
        Plato::error::check_dimension<size_t>(tOutputData.size(), aExportData.size());
        aExportData.setData(tOutputData);
    }
    catch(const std::invalid_argument & tErrorMsg)
    {
        throw tErrorMsg;
    }
}

/******************************************************************************//**
 * @brief Set input shared data container
 * @param [in] aArgumentName name of import data (e.g. design variables)
 * @param [in] aImportData import shared data container
**********************************************************************************/
void RocketDesignApp::inputData(const std::string & aArgumentName, const Plato::SharedData & aImportData)
{
    try
    {
        auto tIterator = mSharedDataMap.find(aArgumentName);
        std::vector<double> & tImportData = tIterator->second;
        Plato::error::check_dimension<size_t>(tImportData.size(), aImportData.size());
        aImportData.getData(tImportData);
    }
    catch(const std::invalid_argument & tErrorMsg)
    {
        throw tErrorMsg;
    }
}

/******************************************************************************//**
 * @brief Define valid application-based operations
**********************************************************************************/
void RocketDesignApp::defineOperations()
{
    mDefinedOperations.push_back("ObjectiveValue");
    mDefinedOperations.push_back("ObjectiveGradient");
    mDefinedOperations.push_back("SetNormalizationConstants");
}

/******************************************************************************//**
 * @brief Define valid application-based shared data containers
**********************************************************************************/
void RocketDesignApp::defineSharedDataMaps()
{
    const int tLength = 1;
    std::string tName = "ThrustMisfitObjective";
    mSharedDataMap[tName] = std::vector<double>(tLength);
    mDefinedDataLayout[tName] = Plato::data::SCALAR;

    tName = "DesignVariables";
    mSharedDataMap[tName] = std::vector<double>(mNumDesigVariables);
    mDefinedDataLayout[tName] = Plato::data::SCALAR;

    tName = "ThrustMisfitObjectiveGradient";
    mSharedDataMap[tName] = std::vector<double>(mNumDesigVariables);
    mDefinedDataLayout[tName] = Plato::data::SCALAR;

    tName = "UpperBounds";
    mSharedDataMap[tName] = std::vector<double>(mNumDesigVariables);
    mDefinedDataLayout[tName] = Plato::data::SCALAR;
}

/******************************************************************************//**
 * @brief Evaluate objective function
**********************************************************************************/
void RocketDesignApp::evaluateObjective()
{
    std::string tArgumentName("DesignVariables");
    auto tIterator = mSharedDataMap.find(tArgumentName);
    assert(tIterator != mSharedDataMap.end());
    Plato::StandardMultiVector<double> tControl(1 /* NUM CONTROL VECTORS */, mNumDesigVariables);
    std::vector<double> & tControlVector = tIterator->second;
    tControl.setData(0 /* VECTOR INDEX */, tControlVector);
    double tObjectiveValue = mObjective->value(tControl);

    tArgumentName = "ThrustMisfitObjective";
    tIterator = mSharedDataMap.find(tArgumentName);
    assert(tIterator != mSharedDataMap.end());
    const int tIndex = 0;
    tIterator->second.operator[](tIndex) = tObjectiveValue;
}

/******************************************************************************//**
 * @brief Compute objective gradient
**********************************************************************************/
void RocketDesignApp::computeObjectiveGradient()
{
    std::string tArgumentName("DesignVariables");
    auto tIterator = mSharedDataMap.find(tArgumentName);
    assert(tIterator != mSharedDataMap.end());
    std::vector<double> & tControlVector = tIterator->second;
    Plato::StandardMultiVector<double> tControl(1 /* NUM CONTROL VECTORS */, tControlVector);
    tControl.setData(0 /* VECTOR INDEX */, tControlVector);

    tArgumentName = "ThrustMisfitObjectiveGradient";
    tIterator = mSharedDataMap.find(tArgumentName);
    assert(tIterator != mSharedDataMap.end());
    Plato::StandardMultiVector<double> tGradient(1 /* NUM CONTROL VECTORS */, mNumDesigVariables);
    mObjective->gradient(tControl, tGradient);
    std::vector<double> & tGradientVector = tIterator->second;
    assert(tGradientVector.size() == mNumDesigVariables);
    tGradient.getData(0 /* VECTOR INDEX */, tGradientVector);
}

/******************************************************************************//**
 * @brief Set normalization constants for objective function
**********************************************************************************/
void RocketDesignApp::setNormalizationConstants()
{
    std::string tArgumentName("UpperBounds");
    auto tIterator = mSharedDataMap.find(tArgumentName);
    assert(tIterator != mSharedDataMap.end());
    std::vector<double> & tValues = tIterator->second;
    mObjective->setNormalizationConstants(tValues);
}

/******************************************************************************//**
 * @brief Perform valid application-based operation.
 * @param [in] aOperationName name of operation
**********************************************************************************/
void RocketDesignApp::performOperation(const std::string & aOperationName)
{
    if(aOperationName.compare("ObjectiveValue") == static_cast<int>(0))
    {
        this->evaluateObjective();
    }
    else if (aOperationName.compare("ObjectiveGradient") == static_cast<int>(0))
    {
        this->computeObjectiveGradient();
    }
    else
    {
        this->setNormalizationConstants();
    }
}

} // namespace Plato
