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

    std::vector<double> tTargetThrustProfile =
        { 0, 1656714.377766964, 1684717.520617273, 1713123.001583093, 1741935.586049868, 1771160.083875437,
                1800801.349693849, 1830864.28322051, 1861353.829558637, 1892274.979507048, 1923632.769869272,
                1955432.283763989, 1987678.650936801, 2020377.048073344, 2053532.699113719, 2087150.875568287,
                2121236.896834771, 2155796.130516737, 2190833.992743404, 2226355.948490792, 2262367.511904243,
                2298874.246622283, 2335881.766101836, 2373395.733944806, 2411421.864226017, 2449965.921822503,
                2489033.722744186, 2528631.134465915, 2568764.076260844, 2609438.519535244, 2650660.488164633,
                2692436.058831303, 2734771.361363255, 2777672.579074459, 2821145.949106557, 2865197.762771913,
                2909834.365898075, 2955062.159173611, 3000887.598495364, 3047317.195317072, 3094357.516999425,
                3142015.18716148, 3190296.886033527, 3239209.350811319, 3288759.376011737, 3338953.813829865,
                3389799.574497465, 3441303.626642879, 3493472.997652346, 3546314.774032734, 3599836.101775718,
                3654044.186723352, 3708946.294935087, 3764549.753056224, 3820861.948687783, 3877890.330757833,
                3935642.409894215, 3994125.758798767, 4053348.012622938, 4113316.869344868, 4174040.090147917,
                4235525.499800648, 4297780.987038235, 4360814.504945371, 4424634.071340578, 4489247.76916203,
                4554663.746854796, 4620890.218759571, 4687935.465502855, 4755807.834388626, 4824515.739791448,
                4894067.663551098, 4964472.155368621, 5035737.83320389, 5107873.383674653, 5180887.562457044,
                5254789.194687578, 5329587.175366664, 5405290.469763565, 5481908.11382287, 5559449.214572486,
                5637922.950533082, 5717338.572129052, 5797705.402100981, 5879032.835919643, 5961330.342201422,
                6044607.46312535, 6128873.814851565, 6214139.087941348, 6300413.047778608, 6387705.534992979,
                6476026.465884338, 6565385.832848894, 6655793.704806847, 6747260.227631442, 6839795.624579719,
                6933410.196724654, 7028114.32338894, 7123918.462580209, 7220833.151427887 };

    Plato::AlgebraicRocketInputs<double> tRocketSimInputs;
    std::shared_ptr<Plato::GeometryModel<double>> tGeomModel =
            std::make_shared<Plato::Cylinder<double>>(tRocketSimInputs.mChamberRadius, tRocketSimInputs.mChamberLength);
    mObjective = std::make_shared<Plato::GradBasedRocketObjective<double>>(tTargetThrustProfile, tRocketSimInputs, tGeomModel);
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
