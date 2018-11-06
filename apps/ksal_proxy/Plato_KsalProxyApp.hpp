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
 * Plato_KsalProxyApp.hpp
 *
 *  Created on: Nov 6, 2018
 */

#pragma once

#include <map>
#include <vector>
#include <string>
#include <memory>
#include <cstdio>
#include <cstdlib>
#include <sstream>

#include "Plato_SharedData.hpp"
#include "Plato_Application.hpp"

#include "Plato_Radius.hpp"
#include "Plato_Rosenbrock.hpp"
#include "Plato_StandardVector.hpp"
#include "Plato_StandardMultiVector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class KsalProxyApp : public Plato::Application
{
public:
    KsalProxyApp() :
            mNumControls(2),
            mConstraint(std::make_shared<Plato::Radius<ScalarType, OrdinalType>>()),
            mObjective(std::make_shared<Plato::Rosenbrock<ScalarType, OrdinalType>>()),
            mDataMap()
    {
        this->makeDataMap();
    }

    virtual ~KsalProxyApp()
    {
    }

    /****************************************************************************************/
    void finalize()
    /****************************************************************************************/
    {
        // NO MEMORY DEALLOCATION NEEDED
        return;
    }

    /****************************************************************************************/
    void initialize()
    /****************************************************************************************/
    {
        // NO MEMORY ALLOCATION NEEDED
        return;
    }

    /****************************************************************************************/
    void compute(const std::string & aOperationName)
    /****************************************************************************************/
    {
        if(aOperationName.compare("ObjectiveValue") == static_cast<int>(0))
        {
            this->computeObjectiveValue();
        }
        else if(aOperationName.compare("ObjectiveGradient") == static_cast<int>(0))
        {
            this->computeObjectiveGradient();
        }
        else if(aOperationName.compare("ObjectiveHessian") == static_cast<int>(0))
        {
            this->computeObjectiveHessian();
        }
        else if(aOperationName.compare("ConstraintValue") == static_cast<int>(0))
        {
            this->computeConstraintValue();
        }
        else if(aOperationName.compare("ConstraintGradient") == static_cast<int>(0))
        {
            this->computeConstraintGradient();
        }
        else if(aOperationName.compare("ConstraintHessian") == static_cast<int>(0))
        {
            this->computeConstraintHessian();
        }
        else
        {
            std::ostringstream tErrorMsg;
            tErrorMsg << "\n\n*********\n ERROR IN " << __FILE__ << ", FUNCTION: " << __PRETTY_FUNCTION__ << "\n LINE: "
                      << __LINE__ << "\n MESSAGE: OPERATION NAME = '" << aOperationName.c_str()
                      << "' IS NOT DEFINE. ABORT!\n*********\n\n";
            std::perror(tErrorMsg.str().c_str());
            std::abort();
        }
    }

    /****************************************************************************************/
    void exportData(const std::string & aArgumentName, Plato::SharedData & aExportData)
    /****************************************************************************************/
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

    /****************************************************************************************/
    void importData(const std::string & aArgumentName, const Plato::SharedData & aImportData)
    /****************************************************************************************/
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

    /****************************************************************************************/
    void exportDataMap(const Plato::data::layout_t & aDataLayout, std::vector<int> & aMyOwnedGlobalIDs)
    /****************************************************************************************/
    {
        /************************************************************************************
          THIS APPLICATION DOES NOT HAVE A GRAPH. HENCE, EXPORT DATA MAP IS A NULL OPERATION.
          THIS APPLICATION IS MEANT TO BE RUN IN SERIAL (NO DATA IS DISTRIBUTED).
         ************************************************************************************/
    }

private:
    /****************************************************************************************/
    void makeDataMap()
    /****************************************************************************************/
    {
        const OrdinalType tNumVectors = 1;
        const OrdinalType tNumCriteria = 1;
        mConstraint->setLimit(2.0);

        std::string tName = "Controls";
        mDataMap[tName] =  std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(tNumVectors, mNumControls);

        tName = "Direction";
        mDataMap[tName] = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(tNumVectors, mNumControls);

        tName = "RosenbrockObjective";
        mDataMap[tName] = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(tNumVectors, tNumCriteria);

        tName = "RosenbrockGradient";
        mDataMap[tName] = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(tNumVectors, mNumControls);

        tName = "RosenbrockHessian";
        mDataMap[tName] = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(tNumVectors, mNumControls);

        tName = "RadiusConstraint";
        mDataMap[tName] = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(tNumVectors, tNumCriteria);

        tName = "RadiusGradient";
        mDataMap[tName] = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(tNumVectors, mNumControls);

        tName = "RadiusHessian";
        mDataMap[tName] = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(tNumVectors, mNumControls);
    }

    /****************************************************************************************/
    void inputData(const std::string & aArgumentName, const Plato::SharedData & aImportData)
    /****************************************************************************************/
    {
        auto tIterator = mDataMap.find(aArgumentName);
        if(tIterator == mDataMap.end())
        {
            std::ostringstream tErrorMsg;
            tErrorMsg << "\n\n*********\n ERROR IN " << __FILE__ << "\n FUNCTION: " << __PRETTY_FUNCTION__ << "\n LINE: "
                      << __LINE__ << "\n MESSAGE: IMPORT ARGUMENT NAME = '" << aArgumentName.c_str()
                      << "' IS NOT DEFINE BY APP. ABORT!\n *********\n\n";
            std::perror(tErrorMsg.str().c_str());
            std::abort();
        }

        const OrdinalType tVECTOR_INDEX = 0;
        Plato::MultiVector<ScalarType, OrdinalType> & tMyAppMultiVector = tIterator->second.operator*();
        Plato::Vector<ScalarType, OrdinalType> & tMyAppVector = tMyAppMultiVector[tVECTOR_INDEX];
        assert(tMyAppVector.size() == static_cast<OrdinalType>(aImportData.size()));

        const OrdinalType tMyLength = tMyAppVector.size();
        Plato::StandardVector<ScalarType, OrdinalType> tInputData(tMyLength);
        aImportData.getData(tInputData.vector());
        tMyAppVector.update(static_cast<ScalarType>(1), tInputData, static_cast<ScalarType>(0));
    }

    /****************************************************************************************/
    void outputData(const std::string & aArgumentName, Plato::SharedData & aExportData)
    /****************************************************************************************/
    {
        auto tIterator = mDataMap.find(aArgumentName);
        if(tIterator == mDataMap.end())
        {
            std::ostringstream tErrorMsg;
            tErrorMsg << "\n\n*********\n ERROR IN " << __FILE__ << "\n FUNCTION: " << __PRETTY_FUNCTION__ << "\n LINE: "
                      << __LINE__ << "\n MESSAGE: EXPORT ARGUMENT NAME = '" << aArgumentName.c_str()
                      << "' IS NOT DEFINE IN APPLICATION DATA MAP. ABORT!\n *********\n\n";
            std::perror(tErrorMsg.str().c_str());
            std::abort();
        }

        const OrdinalType tVECTOR_INDEX = 0;
        Plato::MultiVector<ScalarType, OrdinalType> & tMyAppMultiVector = tIterator->second.operator*();
        Plato::Vector<ScalarType, OrdinalType> & tMyAppVector = tMyAppMultiVector[tVECTOR_INDEX];
        assert(tMyAppVector.size() == static_cast<OrdinalType>(aExportData.size()));

        const OrdinalType tMyLength = tMyAppVector.size();
        Plato::StandardVector<ScalarType, OrdinalType> tOutputData(tMyLength);
        tOutputData.update(static_cast<ScalarType>(1), tMyAppVector, static_cast<ScalarType>(1));
        aExportData.setData(tOutputData.vector());
    }

    /****************************************************************************************/
    void computeObjectiveValue()
    /****************************************************************************************/
    {
        std::string tArgumentName("Controls");
        auto tIterator = mDataMap.find(tArgumentName);
        assert(tIterator != mDataMap.end());
        const Plato::MultiVector<ScalarType, OrdinalType> & tControls = tIterator->second.operator*();
        ScalarType tObjectiveValue = mObjective->value(tControls);

        tArgumentName = "RosenbrockObjective";
        tIterator = mDataMap.find(tArgumentName);
        assert(tIterator != mDataMap.end());
        const OrdinalType tVECTOR_INDEX = 0;
        const OrdinalType tOBJECTIVE_INDEX = 0;
        tIterator->second->operator()(tVECTOR_INDEX, tOBJECTIVE_INDEX) = tObjectiveValue;
    }

    /****************************************************************************************/
    void computeObjectiveGradient()
    /****************************************************************************************/
    {
        std::string tArgumentName("Controls");
        auto tIterator = mDataMap.find(tArgumentName);
        assert(tIterator != mDataMap.end());
        const Plato::MultiVector<ScalarType, OrdinalType> & tControls = tIterator->second.operator*();

        tArgumentName = "RosenbrockGradient";
        tIterator = mDataMap.find(tArgumentName);
        assert(tIterator != mDataMap.end());
        Plato::MultiVector<ScalarType, OrdinalType> & tGradient = tIterator->second.operator*();
        mObjective->gradient(tControls, tGradient);
    }

    /****************************************************************************************/
    void computeObjectiveHessian()
    /****************************************************************************************/
    {
        std::string tArgumentName("Controls");
        auto Iterator = mDataMap.find(tArgumentName);
        assert(Iterator != mDataMap.end());
        const Plato::MultiVector<ScalarType, OrdinalType> & tControls = Iterator->second.operator*();

        tArgumentName = "Direction";
        Iterator = mDataMap.find(tArgumentName);
        assert(Iterator != mDataMap.end());
        const Plato::MultiVector<ScalarType, OrdinalType> & tDirection = Iterator->second.operator*();

        tArgumentName = "RosenbrockHessian";
        Iterator = mDataMap.find(tArgumentName);
        assert(Iterator != mDataMap.end());
        Plato::MultiVector<ScalarType, OrdinalType> & tHessianTimesVector = Iterator->second.operator*();
        mObjective->hessian(tControls, tDirection, tHessianTimesVector);
    }

    /****************************************************************************************/
    void computeConstraintValue()
    /****************************************************************************************/
    {
        std::string tArgumentName("Controls");
        auto tIterator = mDataMap.find(tArgumentName);
        assert(tIterator != mDataMap.end());
        const Plato::MultiVector<ScalarType, OrdinalType> & tControls = tIterator->second.operator*();
        ScalarType tConstraintValue = mConstraint->value(tControls);

        tArgumentName = "RadiusConstraint";
        tIterator = mDataMap.find(tArgumentName);
        assert(tIterator != mDataMap.end());
        const OrdinalType tVECTOR_INDEX = 0;
        const OrdinalType tCONSTRAINT_INDEX = 0;
        tIterator->second->operator()(tVECTOR_INDEX, tCONSTRAINT_INDEX) = tConstraintValue;
    }

    /****************************************************************************************/
    void computeConstraintGradient()
    /****************************************************************************************/
    {
        std::string tArgumentName("Controls");
        auto tIterator = mDataMap.find(tArgumentName);
        assert(tIterator != mDataMap.end());
        const Plato::MultiVector<ScalarType, OrdinalType> & tControls = tIterator->second.operator*();

        tArgumentName = "RadiusGradient";
        tIterator = mDataMap.find(tArgumentName);
        assert(tIterator != mDataMap.end());
        Plato::MultiVector<ScalarType, OrdinalType> & tGradient = tIterator->second.operator*();
        mConstraint->gradient(tControls, tGradient);
    }

    /****************************************************************************************/
    void computeConstraintHessian()
    /****************************************************************************************/
    {
        std::string tArgumentName("Controls");
        auto Iterator = mDataMap.find(tArgumentName);
        assert(Iterator != mDataMap.end());
        const Plato::MultiVector<ScalarType, OrdinalType> & tControls = Iterator->second.operator*();

        tArgumentName = "Direction";
        Iterator = mDataMap.find(tArgumentName);
        assert(Iterator != mDataMap.end());
        const Plato::MultiVector<ScalarType, OrdinalType> & tDirection = Iterator->second.operator*();

        tArgumentName = "RadiusHessian";
        Iterator = mDataMap.find(tArgumentName);
        assert(Iterator != mDataMap.end());
        Plato::MultiVector<ScalarType, OrdinalType> & tHessianTimesVector = Iterator->second.operator*();
        mConstraint->hessian(tControls, tDirection, tHessianTimesVector);
    }

private:
    OrdinalType mNumControls;
    std::shared_ptr<Plato::Radius<ScalarType, OrdinalType>> mConstraint;
    std::shared_ptr<Plato::Rosenbrock<ScalarType, OrdinalType>> mObjective;
    std::map<std::string, std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>>> mDataMap;

private:
    KsalProxyApp(const Plato::KsalProxyApp<ScalarType, OrdinalType> & aRhs);
    Plato::KsalProxyApp<ScalarType, OrdinalType> & operator=(const Plato::KsalProxyApp<ScalarType, OrdinalType> & aRhs);
};
// class KsalProxyApp

}
// namespace Plato
