/*
 * Plato_StageInputDataMng.hpp
 *
 *  Created on: Oct 10, 2017
 */

#ifndef PLATO_STAGEINPUTDATAMNG_HPP_
#define PLATO_STAGEINPUTDATAMNG_HPP_

#include <map>
#include <vector>
#include <string>
#include <utility>

#include "Plato_InputData.hpp"

namespace Plato
{

class OperationInputDataMng;

class StageInputDataMng : public Plato::InputData
{
public:
    StageInputDataMng();
    ~StageInputDataMng();

    int getNumStages() const;
    const std::string & getStageName() const;
    const std::string & getStageName(const int & aStageIndex) const;

    int getNumInputs() const;
    int getNumInputs(const int & aStageIndex) const;
    int getNumInputs(const std::string & aStageName) const;
    const std::vector<std::string> & getInputs(const int & aStageIndex) const;
    const std::string & getInput(const int & aStageIndex, const int & aInputIndex) const;
    const std::vector<std::string> & getInputs(const std::string & aStageName) const;
    const std::string & getInput(const std::string & aStageName, const int & aInputIndex) const;

    int getNumOutputs() const;
    int getNumOutputs(const int & aStageIndex) const;
    int getNumOutputs(const std::string & aStageName) const;
    const std::vector<std::string> & getOutputs(const int & aStageIndex) const;
    const std::string & getOutput(const int & aStageIndex, const int & aOutputIndex) const;
    const std::vector<std::string> & getOutputs(const std::string & aStageName) const;
    const std::string & getOutput(const std::string & aStageName, const int & aOutputIndex) const;

    void add(const std::string & aStageName);
    void add(const std::string & aStageName, const std::vector<std::string> & aInputs, const std::vector<std::string> & aOutputs);

    void addInputs(const std::string & aStageName, const std::vector<std::string> & aInputs);
    void addOutputs(const std::string & aStageName, const std::vector<std::string> & aOutputs);

    int getNumOperations() const;
    int getNumOperations(const int & aStageIndex) const;
    int getNumOperations(const std::string & aStageName) const;
    const Plato::OperationInputDataMng & getOperationInputData(const int & aStageIndex, const int & aOperationIndex) const;
    const Plato::OperationInputDataMng & getOperationInputData(const std::string & aStageName, const int & aOperationIndex) const;
    void addOperationInputData(const std::string & aStageName, const Plato::OperationInputDataMng & aOperationInputData);
    void addOperationInputData(const std::string & aStageName, const std::vector<Plato::OperationInputDataMng> & aOperationInputData);

private:
    std::vector<std::string> mStageNames;
    std::map<std::string, std::vector<Plato::OperationInputDataMng>> mOperationInputs;
    std::map<std::string, std::pair<std::vector<std::string>, std::vector<std::string>>> mSharedDataMap;

    // USING DEFAULT COPY AND ASSIGNMENT CONSTRUCTORS
};

}

#endif /* PLATO_STAGEINPUTDATAMNG_HPP_ */
