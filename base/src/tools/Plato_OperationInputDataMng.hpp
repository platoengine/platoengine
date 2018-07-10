/*
 * Plato_OperationInputDataMng.hpp
 *
 *  Created on: Oct 10, 2017
 */

#ifndef PLATO_OPERATIONINPUTDATAMNG_HPP_
#define PLATO_OPERATIONINPUTDATAMNG_HPP_

#include <map>
#include <vector>
#include <string>
#include <utility>

#include "Plato_Parser.hpp"

namespace Plato
{

class OperationInputDataMng : public Plato::InputData
{
public:
    OperationInputDataMng();
    ~OperationInputDataMng();

    int getNumPerformers() const;
    int getNumOperations() const;
    bool hasSubOperations() const;

    const std::string & getPerformerName() const;
    const std::string & getPerformerName(const int & aOperationIndex) const;
    const std::string & getOperationName(const int & aOperationIndex) const;
    const std::string & getOperationName(const std::string & aPerformerName) const;

    int getNumInputs(const int & aOperationIndex) const;
    int getNumInputs(const std::string & aPerformerName) const;
    const std::vector<std::pair<std::string, std::string>> & getInputs(const int & aOperationIndex) const;
    const std::vector<std::pair<std::string, std::string>> & getInputs(const std::string & aPerformerName) const;
    const std::string & getInputSharedData(const int & aOperationIndex, const int & aInputDataIndex) const;
    const std::string & getInputSharedData(const std::string & aPerformerName, const int & aInputDataIndex) const;
    const std::string & getInputArgument(const int & aOperationIndex, const int & aInputDataIndex) const;
    const std::string & getInputArgument(const std::string & aPerformerName, const int & aInputDataIndex) const;

    int getNumOutputs(const int & aOperationIndex) const;
    int getNumOutputs(const std::string & aPerformerName) const;
    const std::vector<std::pair<std::string, std::string>> & getOutputs(const int & aOperationIndex) const;
    const std::vector<std::pair<std::string, std::string>> & getOutputs(const std::string & aPerformerName) const;
    const std::string & getOutputSharedData(const int & aOperationIndex, const int & aInputDataIndex) const;
    const std::string & getOutputSharedData(const std::string & aPerformerName, const int & aInputDataIndex) const;
    const std::string & getOutputArgument(const int & aOperationIndex, const int & aInputDataIndex) const;
    const std::string & getOutputArgument(const std::string & aPerformerName, const int & aInputDataIndex) const;

    void addInput(const std::string & aPerformerName,
                  const std::string & aOperationName,
                  const std::string & aSharedDataName,
                  const std::string & aArgumentName);
    void addInputs(const std::string & aPerformerName,
                   const std::string & aOperationName,
                   const std::vector<std::string> & aSharedDataNames,
                   const std::vector<std::string> & aArgumentNames);
    void addOutput(const std::string & aPerformerName,
                   const std::string & aOperationName,
                   const std::string & aSharedDataName,
                   const std::string & aArgumentName);
    void addOutputs(const std::string & aPerformerName,
                    const std::string & aOperationName,
                    const std::vector<std::string> & aSharedDataNames,
                    const std::vector<std::string> & aArgumentNames);


private:
    std::vector<std::string> mPerformerName;
    std::map<std::string, std::string> mOperationMap;
    std::map<std::string, std::vector< std::pair<std::string, std::string> > > mInputDataMap;
    std::map<std::string, std::vector< std::pair<std::string, std::string> > > mOutputDataMap;

    // USING DEFAULT COPY AND ASSIGNMENT CONSTRUCTORS
};

}

#endif /* PLATO_OPERATIONINPUTDATAMNG_HPP_ */
