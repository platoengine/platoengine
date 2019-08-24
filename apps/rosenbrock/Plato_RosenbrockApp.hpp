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
 * Plato_RosenbrockApp.hpp
 *
 *  Created on: Mar 23, 2018
 */

#ifndef PLATO_ROSENBROCKAPP_HPP_
#define PLATO_ROSENBROCKAPP_HPP_

#include <map>
#include <vector>
#include <string>
#include <memory>
#include <cstdio>
#include <cstdlib>
#include <sstream>

#include "Plato_SharedData.hpp"
#include "Plato_Application.hpp"

#include "Plato_Rosenbrock.hpp"
#include "Plato_StandardVector.hpp"
#include "Plato_StandardMultiVector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class RosenbrockApp : public Plato::Application
{
public:
    RosenbrockApp() :
            mNumControls(2),
            mCriterion(std::make_shared<Plato::Rosenbrock<ScalarType, OrdinalType>>()),
            mDataMap()
    {
        this->makeDataMap();
    }
    virtual ~RosenbrockApp()
    {
    }

    /****************************************************************************************/
    void finalize()
    /****************************************************************************************/
    {
        // NO MEMORY DEALLOCATION NEEDED
    }
    /****************************************************************************************/
    void initialize()
    /****************************************************************************************/
    {
        // NO MEMORY ALLOCATION NEEDED
    }
    /****************************************************************************************/
    void compute(const std::string & aOperationName)
    /****************************************************************************************/
    {
        if(aOperationName.compare("Objective") == static_cast<int>(0))
        {
            this->objective();
        }
        else if(aOperationName.compare("Gradient") == static_cast<int>(0))
        {
            this->gradient();
        }
        else if(aOperationName.compare("HessianTimesVector") == static_cast<int>(0))
        {
            this->hessian();
        }
        else
        {
            std::ostringstream tErrorMsg;
            tErrorMsg << "\n\n********* ERROR IN " << __FILE__ << ", FUNCTION: " << __func__ << ", LINE: " << __LINE__
                      << ", MESSAGE: OPERATION NAME = '" << aOperationName.c_str()
                      << "' IS NOT DEFINED IN THE OPERATION MAP. ABORT! *********\n\n";
            std::perror(tErrorMsg.str().c_str());
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
                tErrorMsg << "\n\n********* ERROR IN " << __FILE__ << ", FUNCTION: " << __func__ << ", LINE: " << __LINE__
                          << ", MESSAGE: DATA LAYOUT IS NOT SUPPORTED BY THE APPLICATION. ABORT! *********\n\n";
                std::perror(tErrorMsg.str().c_str());
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
                tErrorMsg << "\n\n********* ERROR IN " << __FILE__ << ", FUNCTION: " << __func__ << ", LINE: " << __LINE__
                          << ", MESSAGE: DATA LAYOUT IS NOT SUPPORTED BY THE APPLICATION. ABORT! *********\n\n";
                std::perror(tErrorMsg.str().c_str());
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
        std::string tName = "RosenbrockObjective";
        mDataMap[tName] =
                std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(tNumVectors, tNumCriteria);

        tName = "Controls";
        mDataMap[tName] =
                std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(tNumVectors, mNumControls);

        tName = "Direction";
        mDataMap[tName] =
                std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(tNumVectors, mNumControls);

        tName = "RosenbrockGradient";
        mDataMap[tName] =
                std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(tNumVectors, mNumControls);

        tName = "RosenbrockHessianTimesVector";
        mDataMap[tName] =
                std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(tNumVectors, mNumControls);
    }
    /****************************************************************************************/
    void inputData(const std::string & aArgumentName, const Plato::SharedData & aImportData)
    /****************************************************************************************/
    {
        auto tIterator = mDataMap.find(aArgumentName);
        if(tIterator == mDataMap.end())
        {
            std::ostringstream tErrorMsg;
            tErrorMsg << "\n\n********* ERROR IN " << __FILE__ << ", FUNCTION: " << __func__ << ", LINE: " << __LINE__
                      << ", MESSAGE: IMPORT ARGUMENT NAME = '" << aArgumentName.c_str()
                      << "' IS NOT DEFINED IN THE DATA MAP. ABORT! *********\n\n";
            std::perror(tErrorMsg.str().c_str());
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
            tErrorMsg << "\n\n********* ERROR IN " << __FILE__ << ", FUNCTION: " << __func__ << ", LINE: " << __LINE__
                    << ", MESSAGE: EXPORT ARGUMENT NAME = '" << aArgumentName.c_str()
                    << "' IS NOT DEFINED IN THE DATA MAP. ABORT! *********\n\n";
            std::perror(tErrorMsg.str().c_str());
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
    void objective()
    /****************************************************************************************/
    {
        std::string tArgumentName("Controls");
        auto tIterator = mDataMap.find(tArgumentName);
        assert(tIterator != mDataMap.end());
        const Plato::MultiVector<ScalarType, OrdinalType> & tControls = tIterator->second.operator*();
        ScalarType tObjectiveValue = mCriterion->value(tControls);

        tArgumentName = "RosenbrockObjective";
        tIterator = mDataMap.find(tArgumentName);
        assert(tIterator != mDataMap.end());
        const OrdinalType tVECTOR_INDEX = 0;
        const OrdinalType tOBJECTIVE_INDEX = 0;
        tIterator->second->operator()(tVECTOR_INDEX, tOBJECTIVE_INDEX) = tObjectiveValue;
    }
    /****************************************************************************************/
    void gradient()
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
        mCriterion->gradient(tControls, tGradient);
    }
    /****************************************************************************************/
    void hessian()
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

        tArgumentName = "RosenbrockHessianTimesVector";
        Iterator = mDataMap.find(tArgumentName);
        assert(Iterator != mDataMap.end());
        Plato::MultiVector<ScalarType, OrdinalType> & tHessianTimesVector = Iterator->second.operator*();
        mCriterion->hessian(tControls, tDirection, tHessianTimesVector);
    }

private:
    OrdinalType mNumControls;
    std::shared_ptr<Plato::Rosenbrock<ScalarType, OrdinalType>> mCriterion;
    std::map<std::string, std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>>> mDataMap;

private:
    RosenbrockApp(const Plato::RosenbrockApp<ScalarType, OrdinalType> & aRhs);
    Plato::RosenbrockApp<ScalarType, OrdinalType> & operator=(const Plato::RosenbrockApp<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_ROSENBROCKAPP_HPP_ */
