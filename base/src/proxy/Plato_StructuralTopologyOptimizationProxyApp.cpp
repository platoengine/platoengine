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
 * Plato_StructuralTopologyOptimizationProxyApp.cpp
 *
 *  Created on: Nov 16, 2017
 */

#include <cstdio>
#include <iostream>

#include "Epetra_SerialDenseVector.h"

#include "Plato_SharedData.hpp"
#include "Plato_StructuralTopologyOptimization.hpp"
#include "Plato_StructuralTopologyOptimizationProxyApp.hpp"

namespace Plato
{

StructuralTopologyOptimizationProxyApp::StructuralTopologyOptimizationProxyApp() :
        mNormalizeObjFunc(true),
        mIsFirstObjFuncValueSet(false),
        mNumElemXDirection(30),
        mNumElemYDirection(10),
        mPoissonRatio(0.3),
        mElasticModulus(1),
        mFirstObjFuncValue(1),
        mMyStateOwnedGlobalIDs(),
        mMyControlOwnedGlobalIDs(),
        mPDE(),
        mDataMap()
{
}

StructuralTopologyOptimizationProxyApp::StructuralTopologyOptimizationProxyApp(int aArgc, char **aArgv) :
        mNormalizeObjFunc(true),
        mIsFirstObjFuncValueSet(false),
        mNumElemXDirection(30),
        mNumElemYDirection(10),
        mPoissonRatio(0.3),
        mElasticModulus(1),
        mFirstObjFuncValue(1),
        mMyControlOwnedGlobalIDs(),
        mPDE(),
        mDataMap()
{
}

StructuralTopologyOptimizationProxyApp::~StructuralTopologyOptimizationProxyApp()
{
}

void StructuralTopologyOptimizationProxyApp::disableObjectiveNormalization()
{
    mNormalizeObjFunc = false;
}

int StructuralTopologyOptimizationProxyApp::getNumDesignVariables() const
{
    return (mPDE->getNumDesignVariables());
}

int StructuralTopologyOptimizationProxyApp::getGlobalNumDofs() const
{
    return (mPDE->getGlobalNumDofs());
}

void StructuralTopologyOptimizationProxyApp::finalize()
{
}

void StructuralTopologyOptimizationProxyApp::initialize()
{
    mPDE = std::make_shared<Plato::StructuralTopologyOptimization>(mPoissonRatio, mElasticModulus, mNumElemXDirection, mNumElemYDirection);
    mPDE->disableCacheState();
    mPDE->setFilterRadius(1.2);

    this->makeGraph();

    const int tTotalNumDOFs = mPDE->getGlobalNumDofs();
    Epetra_SerialDenseVector tForce(tTotalNumDOFs);
    const int tDOFsIndex = 1;
    tForce[tDOFsIndex] = -1;
    mPDE->setForceVector(tForce);

    std::vector<double> tData = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 681};
    Epetra_SerialDenseVector tFixedDOFs(Epetra_DataAccess::Copy, tData.data(), tData.size());
    mPDE->setFixedDOFs(tFixedDOFs);

    this->makeDataMap();
}

void StructuralTopologyOptimizationProxyApp::compute(const std::string & aOperationName)
{
    if(aOperationName.compare("Solve") == static_cast<int>(0))
    {
        this->solvePartialDifferentialEquation();
    }
    else if(aOperationName.compare("Objective") == static_cast<int>(0))
    {
        this->evaluateObjective();
    }
    else if(aOperationName.compare("Constraint") == static_cast<int>(0))
    {
        this->evaluateConstraint();
    }
    else if(aOperationName.compare("ObjectiveGradient") == static_cast<int>(0))
    {
        this->computeObjectiveGradient();
    }
    else if(aOperationName.compare("ConstraintGradient") == static_cast<int>(0))
    {
        this->computeConstraintGradient();
    }
    else if(aOperationName.compare("FilteredObjectiveGradient") == static_cast<int>(0))
    {
        this->computeFilteredObjectiveGradient();
    }
    else
    {
        std::ostringstream tErrorMsg;
        tErrorMsg << "\n\n********* ERROR IN " << __FILE__ << ", FUNCTION: " << __func__ << ", LINE: " << __LINE__
                  << ", MESSAGE: OPERATION NAME = '" << aOperationName.c_str()
                  << "' IS NOT DEFINE IN APPLICATION DATA MAP. ABORT! *********\n\n";
        std::perror(tErrorMsg.str().c_str());
        std::abort();
    }
}

void StructuralTopologyOptimizationProxyApp::exportData(const std::string & aArgumentName, Plato::SharedData & aExportData)
{
    switch(aExportData.myLayout())
    {
        case Plato::data::layout_t::SCALAR:
        case Plato::data::layout_t::SCALAR_FIELD:
        case Plato::data::layout_t::VECTOR_FIELD:
        {
            this->outputData(aArgumentName, aExportData);
            break;
        }
        default:
        case Plato::data::layout_t::TENSOR_FIELD:
        case Plato::data::layout_t::ELEMENT_FIELD:
        {
            std::ostringstream tErrorMsg;
            tErrorMsg << "\n\n********* ERROR IN " << __FILE__ << ", FUNCTION: " << __func__ << ", LINE: " << __LINE__
                      << ", MESSAGE: DATA LAYOUT IS NOT SUPPORTED BY APPLICATION. ABORT! *********\n\n";
            std::perror(tErrorMsg.str().c_str());
            std::abort();
            break;
        }
    }
}

void StructuralTopologyOptimizationProxyApp::importData(const std::string & aArgumentName, const Plato::SharedData & aImportData)
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
            tErrorMsg << "\n\n********* ERROR IN " << __FILE__ << ", FUNCTION: " << __func__ << ", LINE: " << __LINE__
                      << ", MESSAGE: DATA LAYOUT IS NOT SUPPORTED BY APPLICATION. ABORT! *********\n\n";
            std::perror(tErrorMsg.str().c_str());
            std::abort();
            break;
        }
    }
}

void StructuralTopologyOptimizationProxyApp::exportDataMap(const Plato::data::layout_t & aDataLayout, std::vector<int> & aMyOwnedGlobalIDs)
{
    switch(aDataLayout)
    {
        case Plato::data::layout_t::SCALAR_FIELD:
        {
            aMyOwnedGlobalIDs = mMyControlOwnedGlobalIDs;
            break;
        }
        case Plato::data::layout_t::VECTOR_FIELD:
        {
            aMyOwnedGlobalIDs = mMyStateOwnedGlobalIDs;
            break;
        }
        default:
        case Plato::data::layout_t::SCALAR:
        case Plato::data::layout_t::TENSOR_FIELD:
        case Plato::data::layout_t::ELEMENT_FIELD:
        {
            std::ostringstream tErrorMsg;
            tErrorMsg << "\n\n********* ERROR IN " << __FILE__ << ", FUNCTION: " << __func__ << ", LINE: " << __LINE__
                      << ", MESSAGE: GRAPH IS NOT SUPPORTED FOR PROVIDED DATA LAYOUT = '" << aDataLayout
                      << "'. ABORT! *********\n\n";
            std::perror(tErrorMsg.str().c_str());
            std::abort();
            break;
        }
    }
}

void StructuralTopologyOptimizationProxyApp::makeGraph()
{
    const int tNumDofs = this->getGlobalNumDofs();
    mMyStateOwnedGlobalIDs.resize(tNumDofs);
    for(int tIndex = 0; tIndex < tNumDofs; tIndex++)
    {
        mMyStateOwnedGlobalIDs[tIndex] = tIndex;
    }

    const int tNumDesignVariables = this->getNumDesignVariables();
    mMyControlOwnedGlobalIDs.resize(tNumDesignVariables);
    for(int tIndex = 0; tIndex < tNumDesignVariables; tIndex++)
    {
        mMyControlOwnedGlobalIDs[tIndex] = tIndex;
    }
}

void StructuralTopologyOptimizationProxyApp::makeDataMap()
{
    const int tLength = 1;
    std::string tName = "InternalEnergy";
    mDataMap[tName] = std::make_shared<Epetra_SerialDenseVector>(tLength);

    tName = "Volume";
    mDataMap[tName] = std::make_shared<Epetra_SerialDenseVector>(tLength);

    tName = "Topology";
    int tNumDesignVariables = this->getNumDesignVariables();
    mDataMap[tName] = std::make_shared<Epetra_SerialDenseVector>(tNumDesignVariables);

    tName = "VolumeGradient";
    mDataMap[tName] = std::make_shared<Epetra_SerialDenseVector>(tNumDesignVariables);

    tName = "InternalEnergyGradient";
    mDataMap[tName] = std::make_shared<Epetra_SerialDenseVector>(tNumDesignVariables);

    tName = "FilteredInternalEnergyGradient";
    mDataMap[tName] = std::make_shared<Epetra_SerialDenseVector>(tNumDesignVariables);

    tName = "StateSolution";
    const int tNumDofs = this->getGlobalNumDofs();
    mDataMap[tName] = std::make_shared<Epetra_SerialDenseVector>(tNumDofs);
}

void StructuralTopologyOptimizationProxyApp::inputData(const std::string & aArgumentName, const Plato::SharedData & aImportData)
{
    auto tIterator = mDataMap.find(aArgumentName);
    if(tIterator == mDataMap.end())
    {
        std::ostringstream tErrorMsg;
        tErrorMsg << "\n\n********* ERROR IN " << __FILE__ << ", FUNCTION: " << __func__ << ", LINE: " << __LINE__
                  << ", MESSAGE: IMPORT ARGUMENT NAME = '" << aArgumentName.c_str()
                  << "' IS NOT DEFINE IN APPLICATION DATA MAP. ABORT! *********\n\n";
        std::perror(tErrorMsg.str().c_str());
        std::abort();
    }
    Epetra_SerialDenseVector & tEpetraVector = tIterator->second.operator*();
    int tMyLength = tEpetraVector.Length();
    assert(tMyLength == aImportData.size());
    std::vector<double> tImportData(tMyLength);
    aImportData.getData(tImportData);
    tEpetraVector.COPY(tEpetraVector.Length(), tImportData.data(), tEpetraVector.A());
}

void StructuralTopologyOptimizationProxyApp::outputData(const std::string & aArgumentName, Plato::SharedData & aExportData)
{
    auto tIterator = mDataMap.find(aArgumentName);
    if(tIterator == mDataMap.end())
    {
        std::ostringstream tErrorMsg;
        tErrorMsg << "\n\n********* ERROR IN " << __FILE__ << ", FUNCTION: " << __func__ << ", LINE: " << __LINE__
                  << ", MESSAGE: EXPORT ARGUMENT NAME = '" << aArgumentName.c_str()
                  << "' IS NOT DEFINE IN APPLICATION DATA MAP. ABORT! *********\n\n";
        std::perror(tErrorMsg.str().c_str());
        std::abort();
    }
    Epetra_SerialDenseVector & tEpetraVector = tIterator->second.operator*();
    int tMyLength = tEpetraVector.Length();
    assert(tMyLength == aExportData.size());
    std::vector<double> tExportData(tMyLength);
    tEpetraVector.COPY(tEpetraVector.Length(), tEpetraVector.A(), tExportData.data());
    aExportData.setData(tExportData);
}

void StructuralTopologyOptimizationProxyApp::solvePartialDifferentialEquation()
{
    std::string tArgumentName("Topology");
    auto tIterator = mDataMap.find(tArgumentName);
    assert(tIterator != mDataMap.end());
    Epetra_SerialDenseVector & tControl = tIterator->second.operator*();
    mPDE->solve(tControl);
    if(mPDE->isStateCached() == true)
    {
        mPDE->cacheState();
    }

    tArgumentName = "StateSolution";
    tIterator = mDataMap.find(tArgumentName);
    assert(tIterator != mDataMap.end());
    Epetra_SerialDenseVector & tState = tIterator->second.operator*();
    const Epetra_SerialDenseVector & tDisplacements = mPDE->getDisplacements();
    tDisplacements.COPY(tDisplacements.Length(), tDisplacements.A(), tState.A());
}

void StructuralTopologyOptimizationProxyApp::evaluateConstraint()
{
    std::string tArgumentName("Topology");
    auto tIterator = mDataMap.find(tArgumentName);
    assert(tIterator != mDataMap.end());
    Epetra_SerialDenseVector & tControl = tIterator->second.operator*();
    double tConstraintValue = mPDE->computeVolumeMisfit(tControl);

    tArgumentName = "Volume";
    tIterator = mDataMap.find(tArgumentName);
    assert(tIterator != mDataMap.end());
    const int tIndex = 0;
    tIterator->second->A()[tIndex] = tConstraintValue;
}

void StructuralTopologyOptimizationProxyApp::evaluateObjective()
{
    std::string tArgumentName("Topology");
    auto tIterator = mDataMap.find(tArgumentName);
    assert(tIterator != mDataMap.end());
    Epetra_SerialDenseVector & tControl = tIterator->second.operator*();
    double tObjectiveValue = mPDE->computeCompliance(tControl);

    tArgumentName = "InternalEnergy";
    tIterator = mDataMap.find(tArgumentName);
    assert(tIterator != mDataMap.end());

    if(mIsFirstObjFuncValueSet == false && mNormalizeObjFunc == true)
    {
        mFirstObjFuncValue = tObjectiveValue;
        mIsFirstObjFuncValueSet = true;
    }
    tObjectiveValue = tObjectiveValue / mFirstObjFuncValue;
    const int tIndex = 0;
    tIterator->second->A()[tIndex] = tObjectiveValue;
}

void StructuralTopologyOptimizationProxyApp::computeObjectiveGradient()
{
    std::string tArgumentName("Topology");
    auto tIterator = mDataMap.find(tArgumentName);
    assert(tIterator != mDataMap.end());
    Epetra_SerialDenseVector & tControl = tIterator->second.operator*();

    tArgumentName = "InternalEnergyGradient";
    tIterator = mDataMap.find(tArgumentName);
    assert(tIterator != mDataMap.end());
    Epetra_SerialDenseVector & tGradient = tIterator->second.operator*();
    mPDE->computeComplianceGradient(tControl, tGradient);

    if(mIsFirstObjFuncValueSet == true && mNormalizeObjFunc == true)
    {
        double tScale = 1.0 / mFirstObjFuncValue;
        tGradient.SCAL(tGradient.Length(), tScale, tGradient.A());
    }
}

void StructuralTopologyOptimizationProxyApp::computeFilteredObjectiveGradient()
{
    std::string tArgumentName("Topology");
    auto tIterator = mDataMap.find(tArgumentName);
    assert(tIterator != mDataMap.end());
    Epetra_SerialDenseVector & tControl = tIterator->second.operator*();

    tArgumentName = "InternalEnergyGradient";
    tIterator = mDataMap.find(tArgumentName);
    assert(tIterator != mDataMap.end());
    Epetra_SerialDenseVector & tGradient = tIterator->second.operator*();

    tArgumentName = "FilteredInternalEnergyGradient";
    tIterator = mDataMap.find(tArgumentName);
    assert(tIterator != mDataMap.end());
    Epetra_SerialDenseVector & tFilteredGradient = tIterator->second.operator*();
    mPDE->applySensitivityFilter(tControl, tGradient, tFilteredGradient);
}

void StructuralTopologyOptimizationProxyApp::computeConstraintGradient()
{
    std::string tArgumentName("Topology");
    auto tIterator = mDataMap.find(tArgumentName);
    assert(tIterator != mDataMap.end());
    Epetra_SerialDenseVector & tControl = tIterator->second.operator*();

    tArgumentName = "VolumeGradient";
    tIterator = mDataMap.find(tArgumentName);
    assert(tIterator != mDataMap.end());
    Epetra_SerialDenseVector & tGradient = tIterator->second.operator*();
    mPDE->computeVolumeGradient(tControl, tGradient);
}

} // namespace Plato
