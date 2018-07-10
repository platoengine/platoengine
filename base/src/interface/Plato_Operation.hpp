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
