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
 * Plato_Operation.hpp
 *
 *  Created on: March 30, 2018
 *
 */

#ifndef SRC_OPERATION_HPP_
#define SRC_OPERATION_HPP_

#include <map>
#include <string>
#include <vector>

#include "Plato_SharedData.hpp"

namespace Plato
{

class Performer;
class OperationInputDataMng;

//! Performer with input and output shared fields.
/*!
 */
class Operation
{
public:
    virtual ~Operation();
    Operation(const ::Plato::OperationInputDataMng & aOperationDataMng,
              const std::vector<::Plato::Performer*>& aPerformers,
              const std::vector<::Plato::SharedData*>& aSharedData);

    virtual void sendInput();
    virtual void sendOutput();
    virtual void compute();

    virtual void importData(std::string sharedDataName, Plato::SharedData* sf);
    virtual void exportData(std::string sharedDataName, Plato::SharedData* sf);

    std::string getPerformerName() const;
    std::string getOperationName() const;
    std::vector<std::string> getInputDataNames() const;
    std::vector<std::string> getOutputDataNames() const;

    void 
    setParameterValue(std::string paramName, double paramValue)
      { m_parameters[paramName]->setData({1,paramValue}); }

protected:

    void addArgument(const std::string & tArgumentName,
                     const std::string & tSharedDataName,
                     const std::vector<Plato::SharedData*>& aSharedData,
                     std::vector<Plato::SharedData*>& aLocalData);

    class Parameter : public Plato::SharedData {
      const std::string m_name;
      const std::string m_operation;
      double m_value;
      public:
        Parameter(std::string& name, std::string& op, double value=0.0) : 
          m_name(name), m_operation(op), m_value(value){}

        // required (pure virtual in SharedData base class)
        int size() const {return 1;}
        std::string myName() const {return m_name;}
        std::string myContext() const {return m_operation;}
        Plato::data::layout_t myLayout() const 
         {return Plato::data::layout_t::SCALAR_PARAMETER;}
        void transmitData(){assert(0);}
        void setData(const std::vector<double> & aData) {m_value = aData[0];}
        void getData(std::vector<double> & aData) const {aData[0] = m_value;}
    };
    std::map<std::string,Plato::SharedData*> m_parameters;

    Performer* m_performer;
    std::string m_operationName;

    std::vector<Plato::SharedData*> m_inputData;
    std::vector<Plato::SharedData*> m_outputData;

    std::multimap<std::string, std::string> m_argumentNames;
};

} // End namespace Plato

#endif
