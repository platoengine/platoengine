/*
 * Plato_Parser.hpp
 * 
 * Created on: Dec 13, 2016
 * 
 */

#ifndef SRC_PARSER_H_
#define SRC_PARSER_H_

#include <string>
#include <vector>
#include "pugixml.hpp"
#include "Plato_InputData.hpp"

namespace Plato
{

class StageInputDataMng;
class OperationInputDataMng;
class OptimizerEngineStageData;



class Parser {
  public:
    virtual ~Parser(){}
    virtual InputData parseFile(const std::string& fileName) = 0;
    virtual InputData parseString(const std::string& inputString) = 0;
};

class PugiParser : public Parser {
  public:
    ~PugiParser(){}
    Plato::InputData parseFile(const std::string& fileName);
    Plato::InputData parseString(const std::string& inputString);
  private:
    Plato::InputData read(std::shared_ptr<pugi::xml_document> doc);
    void addChildren(const pugi::xml_node& node, InputData& inputData);
};

namespace Get
{
bool                Bool      (const Plato::InputData & aInputData, const std::string & aFieldname);
double              Double    (const Plato::InputData & aInputData, const std::string & aFieldname);
std::vector<double> Doubles   (const Plato::InputData & aInputData, const std::string & aKeyword);
int                 Int       (const Plato::InputData & aInputData, const std::string & aFieldname);
std::vector<int>    Ints      (const Plato::InputData & aInputData, const std::string & aKeyword);
std::string         String    (const Plato::InputData & aInputData, const std::string & aFieldname, bool aToUpper = false);
std::string         String    (      Plato::InputData & aInputData, const std::string & aFieldname, const std::string& aDefault);
Plato::InputData    InputData (Plato::InputData & aNode, const std::string & aNodename, unsigned int aIndex=0);
}

namespace Parse
{

/*** Pugi specific.  To be deprecated ***
 *
 *  These are still used in base/src/analyze
 */
typedef pugi::xml_node InputNode;

void toUppercase(std::string& aString);
void loadFile(pugi::xml_document & aInput);
bool getBool(const pugi::xml_node & aNode, const std::string & aFieldname);
double getDouble(const pugi::xml_node & aNode);
double getDouble(const pugi::xml_node & aNode, const std::string & aFieldname);
std::vector<double> getDoubles(const pugi::xml_node & aXML_Node, const std::string & aKeyword);
int getInt(const pugi::xml_node & aNode, const std::string & aFieldname);
int numChildren(const pugi::xml_node & aNode, const std::string & aFieldname);

std::string getString(pugi::xml_node & aNode, const std::string & aFieldname, const std::string& aDefault);
std::string getStringValue(const pugi::xml_node & aNode, bool aToUpper = false);
std::string getString(const pugi::xml_node & aNode, const std::string & aFieldname, bool aToUpper = false);
std::vector<std::string> getStrings(const pugi::xml_node & aXML_Node, const std::string & aKeyword);
/**** end Pugi specific functions *******/




void toUppercase(std::string& aString);
bool boolFromString(const std::string & strval);

void parseArgumentNameInputs    (const Plato::InputData & aInputData,  std::vector<std::string> & aInputs);
void parseArgumentNameOutputs   (const Plato::InputData & aOutputData, std::vector<std::string> & aOutputs);
void parseSharedDataNameInputs  (const Plato::InputData & tInputData,  std::vector<std::string> & aInputs);
void parseSharedDataNameOutputs (const Plato::InputData & tInputData,  std::vector<std::string> & aOutputs);
void parseStageData             (const Plato::InputData & aStageData,  Plato::StageInputDataMng & aStageInputDataMng);
void parseStageOperations       (const Plato::InputData & aStageData,  Plato::StageInputDataMng & aStageInputDataMng);

void parseConstraintValueNames(const Plato::InputData & aOptimizationNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData);

void parseConstraintHessianNames(const Plato::InputData & aOptimizationNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData);

void parseConstraintTargetValues(const Plato::InputData & aOptimizationNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData);

void parseConstraintGradientNames(const Plato::InputData & aOptimizationNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData);

void parseConstraintReferenceValues(const Plato::InputData & aOptimizationNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData);

void parseConstraintReferenceValueNames(const Plato::InputData & aOptimizationNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData);

void parseOperationData(const Plato::InputData & aOperationNode, Plato::OperationInputDataMng & aOperationData);

void parseObjectiveStagesData(const Plato::InputData & aObjectiveNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData);

void parseLowerBounds(const Plato::InputData & aOptimizerNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData);

void parseUpperBounds(const Plato::InputData & aOptimizerNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData);

void parseInitialGuess(const Plato::InputData & aOptimizerNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData);

void parseConstraints(const Plato::InputData & aConstraintNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData);

void parseOptimizerStages(const Plato::InputData & aOptimizerNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData);

void parseOptimizerOptions(const Plato::InputData & aOptimizerNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData);

void parseOptimizationVariablesNames(const Plato::InputData & aOptimizerNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData);

} // namespace Parse

} /* namespace Plato */

#endif /* SRC_PARSER_H_ */

