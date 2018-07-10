/*
 * Plato_Parser.cpp
 *
 *  Created on: Dec 13, 2016
 *
 */

#include "Plato_Parser.hpp"

#include <cctype>
#include <memory>
#include <cassert>
#include <cstdlib>
#include <sstream>
#include <iterator>
#include <iostream>
#include <algorithm>
#include <stdexcept>

#include "Plato_Exceptions.hpp"
#include "Plato_OperationInputDataMng.hpp"
#include "Plato_OptimizerEngineStageData.hpp"
#include "Plato_StageInputDataMng.hpp"

namespace Plato
{

InputData 
PugiParser::parseFile(const std::string& filename)
{
    auto tInput = std::make_shared<pugi::xml_document>();
    pugi::xml_parse_result tResult = tInput->load_file(filename.c_str());
    if(!tResult)
    {
        std::stringstream ss;
        ss << "Error loading file '" << filename << "'.";
        throw Plato::ParsingException(ss.str());
    }
    return read(tInput);
}

InputData 
PugiParser::parseString(const std::string& inputString)
{
    auto tInput = std::make_shared<pugi::xml_document>();
    pugi::xml_parse_result tResult = tInput->load_string(inputString.c_str());
    if(!tResult)
    {
        //throw
    }
    return read(tInput);
}

InputData 
PugiParser::read(std::shared_ptr<pugi::xml_document> doc){

    InputData retInputData("Input Tree");

    addChildren(*(doc), retInputData);

    retInputData.set<decltype(doc)>("InputTree", doc);

    return retInputData;
}

void
PugiParser::addChildren(const pugi::xml_node& node, InputData& inputData)
{
    for (pugi::xml_node& child: node.children())
    {
      if( std::string(child.child_value()) != "" )
      {
        inputData.add<std::string>(child.name(),child.child_value());
      } 
      else
      {
        Plato::InputData newData(child.name());
        addChildren(child, newData);
        inputData.add<Plato::InputData>(child.name(), newData);
      }
    }
}

namespace Get
{
/******************************************************************************/
std::string String(Plato::InputData & aNode, const std::string & aFieldname, const std::string& aDefault)
/******************************************************************************/
{
    if(aNode.size<std::string>(aFieldname) == 0)
    {
      aNode.set<std::string>(aFieldname, aDefault);
    }
    return String(aNode, aFieldname);
}
/******************************************************************************/
std::string String(const Plato::InputData & aNode, const std::string & aFieldName, bool aToUpper)
/******************************************************************************/
{
    std::string tStrVal;
    if( aNode.size<std::string>(aFieldName) )
    {
        tStrVal = aNode.get<std::string>(aFieldName);
    } else return tStrVal;

    if(aToUpper == true)
    {
        Plato::Parse::toUppercase(tStrVal);
    }
    return tStrVal;
}
/******************************************************************************/
Plato::InputData InputData(Plato::InputData & aNode, const std::string & aFieldName, unsigned int aIndex)
/******************************************************************************/
{
    if( aNode.size<Plato::InputData>(aFieldName) > aIndex )
    {
        // requested entry exists.  return it.
        return aNode.get<Plato::InputData>(aFieldName, aIndex);
    } else
    if( aNode.size<Plato::InputData>(aFieldName) == aIndex )
    {
        // requested entry is one past then end of the list.  add new entry then return it.
        aNode.add<Plato::InputData>(aFieldName, Plato::InputData(aFieldName));
        return aNode.get<Plato::InputData>(aFieldName, aIndex);
    } else 
    {
        // requested entry is more than one past then end of the list.  return empty InputData but don't add it.
        return Plato::InputData(aFieldName);
    }
}
/******************************************************************************/
int Int(const Plato::InputData & aNode, const std::string & aFieldname)
/******************************************************************************/
{
    if( aNode.size<std::string>(aFieldname) )
    {
      const int tBase = 10;
      return std::strtol(aNode.get<std::string>(aFieldname).c_str(), nullptr, tBase);
    }
    else
    {
      return 0;
    }
}
/******************************************************************************/
std::vector<double> Doubles(const Plato::InputData & aInputData, const std::string & aKeyword)
/******************************************************************************/
{
    std::vector<double> tValues;
    auto tStrValues = aInputData.getByName<std::string>(aKeyword);
    for( auto tStrValue : tStrValues )
    {
        double tValue = std::strtold(tStrValue.c_str(), nullptr);
        tValues.push_back(tValue);
    }
    return (tValues);
}
/******************************************************************************/
std::vector<int> Ints(const Plato::InputData & aInputData, const std::string & aKeyword)
/******************************************************************************/
{
    std::vector<int> tValues;
    auto tStrValues = aInputData.getByName<std::string>(aKeyword);
    for( auto tStrValue : tStrValues )
    {
        double tValue = std::stoi(tStrValue.c_str());
        tValues.push_back(tValue);
    }
    return (tValues);
}
/******************************************************************************/
double Double(const Plato::InputData & aNode, const std::string & aFieldname)
/******************************************************************************/
{
    if( aNode.size<std::string>(aFieldname) ){
      return atof(aNode.get<std::string>(aFieldname).c_str());
    } else {
      return 0.0;
    }
}
/******************************************************************************/
bool Bool(const Plato::InputData & aNode, const std::string & aFieldname)
/******************************************************************************/
{
    if( aNode.size<std::string>(aFieldname) ){
      std::string strval = aNode.get<std::string>(aFieldname);
      return Parse::boolFromString(strval);
    } else {
      return false;
    }
}



}

namespace Parse
{

/******************************************************************************/
std::string getString(pugi::xml_node & aNode, const std::string & aFieldname, const std::string& aDefault)
/******************************************************************************/
{
    if(numChildren(aNode, aFieldname) == 0)
    {
      pugi::xml_node newNode = aNode.append_child(aFieldname.c_str());
      newNode.append_child(pugi::node_pcdata).set_value(aDefault.c_str());
    }
    return getString(aNode, aFieldname);
}

/******************************************************************************/
std::string getStringValue(const pugi::xml_node & aNode, bool aToUpper)
/******************************************************************************/
{
    std::string tStrValue = aNode.child_value();
    if(aToUpper == true)
    {
        Plato::Parse::toUppercase(tStrValue);
    }
    return tStrValue;
}

/******************************************************************************/
std::string getString(const pugi::xml_node & aNode, const std::string & aFieldName, bool aToUpper)
/******************************************************************************/
{
    std::string tStrVal = aNode.child_value(aFieldName.c_str());
    if(aToUpper == true)
    {
        Plato::Parse::toUppercase(tStrVal);
    }
    return tStrVal;
}


/******************************************************************************/
std::vector<std::string> getStrings(const pugi::xml_node & aXML_Node, const std::string & aKeyword)
/******************************************************************************/
{
    std::vector<std::string> tValueNames;
    for(pugi::xml_node tNode = aXML_Node.child(aKeyword.c_str()); tNode; tNode = tNode.next_sibling(aKeyword.c_str()))
    {
        const std::string tValue = Plato::Parse::getStringValue(tNode);
        tValueNames.push_back(tValue);
    }
    return (tValueNames);
}


/******************************************************************************/
int getInt(const pugi::xml_node & aNode, const std::string & aFieldname)
/******************************************************************************/
{
    const int tBase = 10;
    return std::strtol(aNode.child_value(aFieldname.c_str()), nullptr, tBase);
}

/******************************************************************************/
double getDouble(const pugi::xml_node & aNode)
/******************************************************************************/
{
    std::string tValue = aNode.child_value();
    return (std::strtold(tValue.c_str(), nullptr));
}

/******************************************************************************/
std::vector<double> getDoubles(const pugi::xml_node & aXML_Node, const std::string & aKeyword)
/******************************************************************************/
{
    std::vector<double> tValues;
    for(pugi::xml_node tNode = aXML_Node.child(aKeyword.c_str()); tNode; tNode = tNode.next_sibling(aKeyword.c_str()))
    {
        const double tValue = Plato::Parse::getDouble(tNode);
        tValues.push_back(tValue);
    }
    return (tValues);
}



/******************************************************************************/
double getDouble(const pugi::xml_node & aNode, const std::string & aFieldname)
/******************************************************************************/
{
    return atof(aNode.child_value(aFieldname.c_str()));
}


/******************************************************************************/
bool getBool(const pugi::xml_node & aNode, const std::string & aFieldname)
/******************************************************************************/
{
    std::string strval = aNode.child_value(aFieldname.c_str());
    return boolFromString(strval);
}

/******************************************************************************/
bool boolFromString(const std::string & strval)
/******************************************************************************/
{
    if(strval.size() == 0)
    {
        return false;
    }
    std::string strCopy(strval);
    Plato::Parse::toUppercase(strCopy);
    if(strCopy == "FALSE")
    {
        return false;
    }
    else if(strCopy == "TRUE")
    {
        return true;
    }
    else
    {
        std::stringstream ss;
        ss << "Plato::Parser: expected a boolean ('True','False'), not '" << strval << "'.";
        throw Plato::ParsingException(ss.str());
    }
    return false;  // prevents compiler warnings
}

/******************************************************************************/
int numChildren(const pugi::xml_node & aNodeIn, const std::string & aFieldname)
/******************************************************************************/
{
    int tCount = 0;
    for(pugi::xml_node aNode = aNodeIn.child(aFieldname.c_str()); aNode; aNode = aNode.next_sibling(aFieldname.c_str()))
    {
        tCount++;
    }
    return tCount;
}

/******************************************************************************/
void toUppercase(std::string & aString)
/******************************************************************************/
{
    std::transform(aString.begin(), aString.end(), aString.begin(), ::toupper);
}

/******************************************************************************/
void loadFile(pugi::xml_document & aInput)
/******************************************************************************/
{
    const char* tInputChar = std::getenv("PLATO_APP_FILE");
    pugi::xml_parse_result tResult = aInput.load_file(tInputChar);
    if(!tResult)
    {
        //throw
    }
}

/******************************************************************************/
void parseArgumentNameInputs(const Plato::InputData & aInputNode, std::vector<std::string> & aInputs)
/******************************************************************************/
{
    assert(aInputs.empty() == true);
    auto tInputs = aInputNode.getByName<Plato::InputData>("Input");
    for(auto tInput=tInputs.begin(); tInput!=tInputs.end(); ++tInput)
    {
        std::string tSharedDataName = tInput->get<std::string>("ArgumentName");
        aInputs.push_back(tSharedDataName);
    }
}

/******************************************************************************/
void parseArgumentNameOutputs(const Plato::InputData & aOutputNode, std::vector<std::string> & aOutputs)
/******************************************************************************/
{
    assert(aOutputs.empty() == true);
    auto tOutputs = aOutputNode.getByName<Plato::InputData>("Output");
    for(auto tOutput=tOutputs.begin(); tOutput!=tOutputs.end(); ++tOutput)
    {
        std::string tSharedDataName = tOutput->get<std::string>("ArgumentName");
        aOutputs.push_back(tSharedDataName);
    }
}

/******************************************************************************/
void parseSharedDataNameInputs(const Plato::InputData & aInputNode, std::vector<std::string> & aInputs)
/******************************************************************************/
{
    assert(aInputs.empty() == true);
    auto tInputs = aInputNode.getByName<Plato::InputData>("Input");
    for(auto tInput=tInputs.begin(); tInput!=tInputs.end(); ++tInput)
    {
        std::string tSharedDataName = tInput->get<std::string>("SharedDataName");
        aInputs.push_back(tSharedDataName);
    }
}

/******************************************************************************/
void parseSharedDataNameOutputs(const Plato::InputData & aOutputNode, std::vector<std::string> & aOutputs)
/******************************************************************************/
{
    assert(aOutputs.empty() == true);
    auto tOutputs = aOutputNode.getByName<Plato::InputData>("Output");
    for(auto tOutput=tOutputs.begin(); tOutput!=tOutputs.end(); ++tOutput)
    {
        std::string tSharedDataName = tOutput->get<std::string>("SharedDataName");
        aOutputs.push_back(tSharedDataName);
    }
}

/******************************************************************************/
void parseOperationData(const Plato::InputData & aOperationNode, Plato::OperationInputDataMng & aOperationData)
/******************************************************************************/
{
    aOperationData.set<Plato::InputData>("Input Data", aOperationNode);
 
    std::string tOperationName = aOperationNode.get<std::string>("Name");
  
    if( aOperationNode.size<std::string>("PerformerName") != static_cast<int>(1) )
    {
        throw Plato::ParsingException("one and only one performer must be specified");
    }
    std::string tPerformerName = aOperationNode.get<std::string>("PerformerName");

    std::vector<std::string> tArgumentNameInputs;
    Plato::Parse::parseArgumentNameInputs(aOperationNode, tArgumentNameInputs);
    std::vector<std::string> tSharedDataNameInputs;
    Plato::Parse::parseSharedDataNameInputs(aOperationNode, tSharedDataNameInputs);
    aOperationData.addInputs(tPerformerName, tOperationName, tSharedDataNameInputs, tArgumentNameInputs);

    std::vector<std::string> tArgumentNameOutputs;
    Plato::Parse::parseArgumentNameOutputs(aOperationNode, tArgumentNameOutputs);
    std::vector<std::string> tSharedDataNameOutputs;
    Plato::Parse::parseSharedDataNameOutputs(aOperationNode, tSharedDataNameOutputs);
    aOperationData.addOutputs(tPerformerName, tOperationName, tSharedDataNameOutputs, tArgumentNameOutputs);

}

/******************************************************************************/
void parseStageOperations(const Plato::InputData & aStageNode, Plato::StageInputDataMng & aStageInputDataMng)
/******************************************************************************/
{
    std::string tStageName = aStageNode.get<std::string>("Name");
    auto tOperationNodes = aStageNode.getByName<Plato::InputData>("Operation");
    for(auto tOperationNode  = tOperationNodes.begin(); 
             tOperationNode != tOperationNodes.end(); ++tOperationNode)
    {
        int numSubOperations = tOperationNode->size<Plato::InputData>("Operation");
        if( numSubOperations > 0 )
        {
            Plato::OperationInputDataMng tOperationInputData;
            auto tSubOperationNodes = tOperationNode->getByName<Plato::InputData>("Operation");
            for(auto tSubOperationNode  = tSubOperationNodes.begin(); 
                     tSubOperationNode != tSubOperationNodes.end(); ++tSubOperationNode)
            {
                int numSubSubOperations = tSubOperationNode->size<Plato::InputData>("Operation");
                if( numSubSubOperations != 0 )
                {
                    throw Plato::ParsingException("Plato::Operation: Cannot have doubly nested Operations");
                }
                Plato::Parse::parseOperationData(*tSubOperationNode, tOperationInputData);
            }
            aStageInputDataMng.addOperationInputData(tStageName, tOperationInputData);
        }
        else
        {
            Plato::OperationInputDataMng tOperationInputData;
            Plato::Parse::parseOperationData(*tOperationNode, tOperationInputData);
            aStageInputDataMng.addOperationInputData(tStageName, tOperationInputData);
        }
    }
}

/******************************************************************************/
void parseInitialGuess(const Plato::InputData & aOptimizerNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData)
/******************************************************************************/
{
    if(aOptimizerNode.size<Plato::InputData>("InitialGuess"))
    {
        Plato::InputData tInitialGuessNode = aOptimizerNode.get<Plato::InputData>("InitialGuess");
        std::vector<std::string> tInitialGuess = tInitialGuessNode.getByName<std::string>("Value");
        aOptimizerEngineStageData.setInitialGuess(tInitialGuess);
    }
}

/******************************************************************************/
void parseStageData(const Plato::InputData & aStageNode, Plato::StageInputDataMng & aStageInputDataMng)
/******************************************************************************/
{
    std::string tStageName = aStageNode.get<std::string>("Name");

    // Find Input SharedData Names
    std::vector<std::string> tSharedDataNameInputs;
    Plato::Parse::parseSharedDataNameInputs(aStageNode, tSharedDataNameInputs);

    // Find Output SharedData Names
    std::vector<std::string> tSharedDataNameOutputs;
    Plato::Parse::parseSharedDataNameOutputs(aStageNode, tSharedDataNameOutputs);

    aStageInputDataMng.add(tStageName, tSharedDataNameInputs, tSharedDataNameOutputs);

    Plato::Parse::parseStageOperations(aStageNode, aStageInputDataMng);
}

/******************************************************************************/
void parseOptimizationVariablesNames(const Plato::InputData & aOptimizerNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData)
/******************************************************************************/
{
    // TODO: parse multiple controls
    Plato::InputData tOptimizationVariablesNode = aOptimizerNode.get<Plato::InputData>("OptimizationVariables");
    std::string tControlName = Plato::Get::String(tOptimizationVariablesNode, "ValueName");
    if(tControlName.empty() == false)
    {
        aOptimizerEngineStageData.addControlName(tControlName);
    }
    std::string tLowerBoundVectorName = Plato::Get::String(tOptimizationVariablesNode, "LowerBoundVectorName");
    if(tLowerBoundVectorName.empty() == false)
    {
        aOptimizerEngineStageData.setLowerBoundVectorName(tLowerBoundVectorName);
    }
    std::string tUpperBoundVectorName = Plato::Get::String(tOptimizationVariablesNode, "UpperBoundVectorName");
    if(tUpperBoundVectorName.empty() == false)
    {
        aOptimizerEngineStageData.setUpperBoundVectorName(tUpperBoundVectorName);
    }
    std::string tLowerBoundValueName = Plato::Get::String(tOptimizationVariablesNode, "LowerBoundValueName");
    if(tLowerBoundValueName.empty() == false)
    {
        aOptimizerEngineStageData.setLowerBoundValueName(tLowerBoundValueName);
    }
    std::string tUpperBoundValueName = Plato::Get::String(tOptimizationVariablesNode, "UpperBoundValueName");
    if(tUpperBoundValueName.empty() == false)
    {
        aOptimizerEngineStageData.setUpperBoundValueName(tUpperBoundValueName);
    }
    std::string tControlInitializationStage = Plato::Get::String(tOptimizationVariablesNode, "InitializationStage");
    if(tControlInitializationStage.empty() == false)
    {
        aOptimizerEngineStageData.setInitializationStageName(tControlInitializationStage);
    }
    std::string tSetLowerBoundsStagName = Plato::Get::String(tOptimizationVariablesNode, "SetLowerBoundsStage");
    if(tSetLowerBoundsStagName.empty() == false)
    {
        aOptimizerEngineStageData.setSetLowerBoundsStageName(tSetLowerBoundsStagName);
    }
    std::string tSetUpperBoundsStageName = Plato::Get::String(tOptimizationVariablesNode, "SetUpperBoundsStage");
    if(tSetUpperBoundsStageName.empty() == false)
    {
        aOptimizerEngineStageData.setSetUpperBoundsStageName(tSetUpperBoundsStageName);
    }
    std::string tDescentDirectionName = Plato::Get::String(tOptimizationVariablesNode, "DescentDirectionName");
    if(tDescentDirectionName.empty() == false)
    {
        aOptimizerEngineStageData.addDescentDirectionName(tDescentDirectionName);
    }
}

/******************************************************************************/
void parseConstraintValueNames(const Plato::InputData & aOptimizationNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData)
/******************************************************************************/
{
    auto tNodes = aOptimizationNode.getByName<Plato::InputData>("Constraint");
    for(auto tNode=tNodes.begin(); tNode!=tNodes.end(); ++tNode)
    {
        if( tNode->size<std::string>("ValueName") )
        {
            std::string tValueName = Plato::Get::String(*tNode, "ValueName");
            aOptimizerEngineStageData.addConstraintValueName(tValueName);
        }
        else
        {
            std::ostringstream tMsg;
            tMsg << "\n\n ********** PLATO ERROR: FILE = " << __FILE__ << ", FUNCTION = " << __PRETTY_FUNCTION__
                 << ", LINE = " << __LINE__ << ", MESSAGE: USERT DID NOT DEFINE A VALUE NAME FOR CONSTRAINT. "
                 << "**********\n\n";
            throw Plato::ParsingException(tMsg.str().c_str());
        }
    }
}

/******************************************************************************/
void parseConstraintGradientNames(const Plato::InputData & aOptimizationNode,
                                  Plato::OptimizerEngineStageData & aOptimizerEngineStageData)
/******************************************************************************/
{
    auto tNodes = aOptimizationNode.getByName<Plato::InputData>("Constraint");
    for( auto tNode=tNodes.begin(); tNode!=tNodes.end(); ++tNode)
    {
        if( tNode->size<std::string>("GradientName") )
        {
            std::string tValueName = tNode->get<std::string>("ValueName");
            assert(tValueName.empty() == false);
            std::string tGradientName = tNode->get<std::string>("GradientName");
            aOptimizerEngineStageData.addConstraintGradientName(tValueName, tGradientName);
        }
        else
        {
            std::ostringstream tMsg;
            tMsg << "\n\n ********** PLATO ERROR: FILE = " << __FILE__ << ", FUNCTION = " << __PRETTY_FUNCTION__
                 << ", LINE = " << __LINE__ << ", MESSAGE: USERT DID NOT DEFINE A GRADIENT NAME FOR CONSTRAINT. "
                 << "**********\n\n";
            throw Plato::ParsingException(tMsg.str().c_str());
        }
    }
}

/******************************************************************************/
void parseConstraintHessianNames(const Plato::InputData & aOptimizationNode,
                                 Plato::OptimizerEngineStageData & aOptimizerEngineStageData)
/******************************************************************************/
{
    auto tNodes = aOptimizationNode.getByName<Plato::InputData>("Constraint");
    for(auto tNode=tNodes.begin(); tNode!=tNodes.end(); ++tNode)
    {
        const std::string tValueName = tNode->get<std::string>("ValueName");
        assert(tValueName.empty() == false);
        std::string tHessianName = Plato::Get::String(*tNode,"HessianName");
        tHessianName = tHessianName.empty() == true ? "LinearCriterionHessian" : tHessianName;
        aOptimizerEngineStageData.addConstraintHessianName(tValueName, tHessianName);
    }
}

/******************************************************************************/
void parseConstraintReferenceValueNames(const Plato::InputData & aOptimizationNode,
                                        Plato::OptimizerEngineStageData & aOptimizerEngineStageData)
/******************************************************************************/
{
    auto tNodes = aOptimizationNode.getByName<Plato::InputData>("Constraint");
    for(auto tNode=tNodes.begin(); tNode!=tNodes.end(); ++tNode)
    {
        if( tNode->size<std::string>("ReferenceValueName") )
        {
            const std::string tValueName = tNode->get<std::string>("ValueName");
            assert(tValueName.empty() == false);
            std::string tRefValueName = tNode->get<std::string>("ReferenceValueName");
            aOptimizerEngineStageData.addConstraintReferenceValueName(tValueName, tRefValueName);
        }
    }
}

/******************************************************************************/
void parseConstraintTargetValues(const Plato::InputData & aOptimizationNode,
                                 Plato::OptimizerEngineStageData & aOptimizerEngineStageData)
/******************************************************************************/
{
    auto tNodes = aOptimizationNode.getByName<Plato::InputData>("Constraint");
    for(auto tNode=tNodes.begin(); tNode!=tNodes.end(); ++tNode)
    {
        const std::string tValueName = tNode->get<std::string>("ValueName");
        assert(tValueName.empty() == false);
        if(tNode->size<std::string>("NormalizedTargetValue"))
        {
            double tValue = Plato::Get::Double(*tNode, "NormalizedTargetValue");
            aOptimizerEngineStageData.addConstraintNormalizedTargetValue(tValueName, tValue);
        }
        if(tNode->size<std::string>("AbsoluteTargetValue"))
        {
            double tValue = Plato::Get::Double(*tNode, "AbsoluteTargetValue");
            aOptimizerEngineStageData.addConstraintAbsoluteTargetValue(tValueName, tValue);
        }
    }
}

/******************************************************************************/
void parseConstraintReferenceValues(const Plato::InputData & aOptimizationNode,
                                    Plato::OptimizerEngineStageData & aOptimizerEngineStageData)
/******************************************************************************/
{
    auto tNodes = aOptimizationNode.getByName<Plato::InputData>("Constraint");
    for(auto tNode=tNodes.begin(); tNode!=tNodes.end(); ++tNode)
    {
        const std::string tValueName = tNode->get<std::string>("ValueName");
        assert(tValueName.empty() == false);
        double tValue = Plato::Get::Double(*tNode, "ReferenceValue");
        tValue = (tValue == static_cast<double>(0.0)) ? static_cast<double>(1) : tValue;
        aOptimizerEngineStageData.addConstraintReferenceValue(tValueName, tValue);
    }
}

/******************************************************************************/
void parseConstraintStagesData(const Plato::InputData & aOptimizationNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData)
/******************************************************************************/
{
    Plato::Parse::parseConstraintValueNames(aOptimizationNode, aOptimizerEngineStageData);
    Plato::Parse::parseConstraintHessianNames(aOptimizationNode, aOptimizerEngineStageData);
    Plato::Parse::parseConstraintTargetValues(aOptimizationNode, aOptimizerEngineStageData);
    Plato::Parse::parseConstraintGradientNames(aOptimizationNode, aOptimizerEngineStageData);
    Plato::Parse::parseConstraintReferenceValues(aOptimizationNode, aOptimizerEngineStageData);
    Plato::Parse::parseConstraintReferenceValueNames(aOptimizationNode, aOptimizerEngineStageData);
}

/******************************************************************************/
void parseObjectiveStagesData(const Plato::InputData & aObjectiveNode, Plato::OptimizerEngineStageData & aOptimizerStageData)
/******************************************************************************/
{
    std::string tOutputSharedDataValueName = Plato::Get::String(aObjectiveNode, "ValueName");
    std::string tObjectiveValueStageName = Plato::Get::String(aObjectiveNode, "ValueStageName");
    if(tOutputSharedDataValueName.empty() == false && tObjectiveValueStageName.empty() == false)
    {
        aOptimizerStageData.setObjectiveValueStageName(tObjectiveValueStageName);
        aOptimizerStageData.setObjectiveValueOutputName(tOutputSharedDataValueName);
    }
    else
    {
        std::ostringstream tMsg;
        tMsg << "\n\n ********** PLATO ERROR: FILE = " << __FILE__ << ", FUNCTION = " << __PRETTY_FUNCTION__ << ", LINE = "
             << __LINE__ << ", MESSAGE: USER DID NOT DEFINE OBJECTIVE FUNCTION VALUE OUTPUT SHARED DATA NAME OR ITS STAGE NAME. "
             << "**********\n\n";
        throw Plato::ParsingException(tMsg.str().c_str());
    }

    std::string tOutputSharedDataGradientName = Plato::Get::String(aObjectiveNode, "GradientName");
    std::string tObjectiveGradientStageName = Plato::Get::String(aObjectiveNode, "GradientStageName");
    if(tOutputSharedDataGradientName.empty() == false && tObjectiveGradientStageName.empty() == false)
    {
        aOptimizerStageData.setObjectiveGradientStageName(tObjectiveGradientStageName);
        aOptimizerStageData.setObjectiveGradientOutputName(tOutputSharedDataGradientName);
    }
    else
    {
        std::ostringstream tMsg;
        tMsg << "\n\n ********** PLATO ERROR: FILE = " << __FILE__ << ", FUNCTION = " << __PRETTY_FUNCTION__ << ", LINE = "
             << __LINE__ << ", MESSAGE: USER DID NOT DEFINE OBJECTIVE FUNCTION OUTPUT SHARED DATA OR ITS STAGE NAME. " << "**********\n\n";
        throw Plato::ParsingException(tMsg.str().c_str());
    }

    std::string tOutputSharedDataHessianName = Plato::Get::String(aObjectiveNode, "HessianName");
    std::string tObjectiveHessianStageName = Plato::Get::String(aObjectiveNode, "HessianStageName");
    if(tOutputSharedDataHessianName.empty() == false && tObjectiveHessianStageName.empty() == false)
    {
        aOptimizerStageData.setObjectiveHessianStageName(tObjectiveHessianStageName);
        aOptimizerStageData.setObjectiveHessianOutputName(tOutputSharedDataHessianName);
    }
}

/******************************************************************************/
void parseOptimizerOptions(const Plato::InputData & aOptimizerNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData)
/******************************************************************************/
{
    if( aOptimizerNode.size<Plato::InputData>("Options") )
    {
        Plato::InputData tOptionsNode = aOptimizerNode.get<Plato::InputData>("Options");

        if( tOptionsNode.size<std::string>("OutputControlToFile") )
        {
            bool tOutputControlToFile = Plato::Get::Bool(tOptionsNode, "OutputControlToFile");
            aOptimizerEngineStageData.setOutputControlToFile(tOutputControlToFile);
        }

        if( tOptionsNode.size<std::string>("OutputDiagnosticsToFile") )
        {
            bool tOutputDiagnosticsToFile = Plato::Get::Bool(tOptionsNode, "OutputDiagnosticsToFile");
            aOptimizerEngineStageData.setOutputDiagnosticsToFile(tOutputDiagnosticsToFile);
        }

        if( tOptionsNode.size<std::string>("InputFileName") )
        {
            std::string tInputFileName = tOptionsNode.get<std::string>("InputFileName");
            aOptimizerEngineStageData.setInputFileName(tInputFileName);
        }

        if( tOptionsNode.size<std::string>("Algebra") )
        {
            std::string tAlgebraType = tOptionsNode.get<std::string>("Algebra");
            aOptimizerEngineStageData.setAlgebra(tAlgebraType);
        }

        if( tOptionsNode.size<std::string>("DerivativeCheckerFinalSuperscript") )
        {
            int tFinalSuperscript = Plato::Get::Int(tOptionsNode, "DerivativeCheckerFinalSuperscript");
            aOptimizerEngineStageData.setDerivativeCheckerFinalSuperscript(tFinalSuperscript);
        }

        if( tOptionsNode.size<std::string>("DerivativeCheckerInitialSuperscript") )
        {
            int tInitialSuperscript = Plato::Get::Int(tOptionsNode, "DerivativeCheckerInitialSuperscript");
            aOptimizerEngineStageData.setDerivativeCheckerInitialSuperscript(tInitialSuperscript);
        }

        if( tOptionsNode.size<std::string>("GCMMAInitialMovingAsymptoteScaleFactor") )
        {
            double tGCMMAInitialMovingAsymptoteScaleFactor = Plato::Get::Double(tOptionsNode, "GCMMAInitialMovingAsymptoteScaleFactor");
            aOptimizerEngineStageData.setGCMMAInitialMovingAsymptoteScaleFactor(tGCMMAInitialMovingAsymptoteScaleFactor);
        }
        if( tOptionsNode.size<std::string>("GCMMAInnerKKTTolerance") )
        {
            double tGCMMAInnerKKTTolerance = Plato::Get::Double(tOptionsNode, "GCMMAInnerKKTTolerance");
            aOptimizerEngineStageData.setGCMMAInnerKKTTolerance(tGCMMAInnerKKTTolerance);
        }
        if( tOptionsNode.size<std::string>("GCMMAOuterKKTTolerance") )
        {
            double tGCMMAOuterKKTTolerance = Plato::Get::Double(tOptionsNode, "GCMMAOuterKKTTolerance");
            aOptimizerEngineStageData.setGCMMAOuterKKTTolerance(tGCMMAOuterKKTTolerance);
        }
        if( tOptionsNode.size<std::string>("GCMMAInnerControlStagnationTolerance") )
        {
            double tGCMMAInnerControlStagnationTolerance = Plato::Get::Double(tOptionsNode, "GCMMAInnerControlStagnationTolerance");
            aOptimizerEngineStageData.setGCMMAInnerControlStagnationTolerance(tGCMMAInnerControlStagnationTolerance);
        }
        if( tOptionsNode.size<std::string>("GCMMAOuterControlStagnationTolerance") )
        {
            double tGCMMAOuterControlStagnationTolerance = Plato::Get::Double(tOptionsNode, "GCMMAOuterControlStagnationTolerance");
            aOptimizerEngineStageData.setGCMMAOuterControlStagnationTolerance(tGCMMAOuterControlStagnationTolerance);
        }
        if( tOptionsNode.size<std::string>("GCMMAOuterObjectiveStagnationTolerance") )
        {
            double tGCMMAOuterObjectiveStagnationTolerance = Plato::Get::Double(tOptionsNode, "GCMMAOuterObjectiveStagnationTolerance");
            aOptimizerEngineStageData.setGCMMAOuterObjectiveStagnationTolerance(tGCMMAOuterObjectiveStagnationTolerance);
        }
        if( tOptionsNode.size<std::string>("GCMMAMaxInnerIterations") )
        {
            int tGCMMAMaxInnerIterations = Plato::Get::Double(tOptionsNode, "GCMMAMaxInnerIterations");
            aOptimizerEngineStageData.setGCMMAMaxInnerIterations(tGCMMAMaxInnerIterations);
        }
        if( tOptionsNode.size<std::string>("GCMMAOuterStationarityTolerance") )
        {
            double tGCMMAOuterStationarityTolerance = Plato::Get::Double(tOptionsNode, "GCMMAOuterStationarityTolerance");
            aOptimizerEngineStageData.setGCMMAOuterStationarityTolerance(tGCMMAOuterStationarityTolerance);
        }
        if( tOptionsNode.size<std::string>("KSTrustRegionExpansionFactor") )
        {
            double tKSTrustRegionExpansionFactor = Plato::Get::Double(tOptionsNode, "KSTrustRegionExpansionFactor");
            aOptimizerEngineStageData.setKSTrustRegionExpansionFactor(tKSTrustRegionExpansionFactor);
        }
        if( tOptionsNode.size<std::string>("KSTrustRegionContractionFactor") )
        {
            double tKSTrustRegionContractionFactor = Plato::Get::Double(tOptionsNode, "KSTrustRegionContractionFactor");
            aOptimizerEngineStageData.setKSTrustRegionContractionFactor(tKSTrustRegionContractionFactor);
        }
        if( tOptionsNode.size<std::string>("KSMaxTrustRegionIterations") )
        {
            int tKSMaxTrustRegionIterations = Plato::Get::Double(tOptionsNode, "KSMaxTrustRegionIterations");
            aOptimizerEngineStageData.setKSMaxTrustRegionIterations(tKSMaxTrustRegionIterations);
        }
        if( tOptionsNode.size<std::string>("KSOuterGradientTolerance") )
        {
            double tKSOuterGradientTolerance = Plato::Get::Double(tOptionsNode, "KSOuterGradientTolerance");
            aOptimizerEngineStageData.setKSOuterGradientTolerance(tKSOuterGradientTolerance);
        }
        if( tOptionsNode.size<std::string>("KSOuterStationarityTolerance") )
        {
            double tKSOuterStationarityTolerance = Plato::Get::Double(tOptionsNode, "KSOuterStationarityTolerance");
            aOptimizerEngineStageData.setKSOuterStationarityTolerance(tKSOuterStationarityTolerance);
        }
        if( tOptionsNode.size<std::string>("KSOuterStagnationTolerance") )
        {
            double tKSOuterStagnationTolerance = Plato::Get::Double(tOptionsNode, "KSOuterStagnationTolerance");
            aOptimizerEngineStageData.setKSOuterStagnationTolerance(tKSOuterStagnationTolerance);
        }
        if( tOptionsNode.size<std::string>("KSOuterControlStagnationTolerance") )
        {
            double tKSOuterControlStagnationTolerance = Plato::Get::Double(tOptionsNode, "KSOuterControlStagnationTolerance");
            aOptimizerEngineStageData.setKSOuterControlStagnationTolerance(tKSOuterControlStagnationTolerance);
        }
        if( tOptionsNode.size<std::string>("KSOuterActualReductionTolerance") )
        {
            double tKSOuterActualReductionTolerance = Plato::Get::Double(tOptionsNode, "KSOuterActualReductionTolerance");
            aOptimizerEngineStageData.setKSOuterActualReductionTolerance(tKSOuterActualReductionTolerance);
        }
    }
}

/******************************************************************************/
void parseLowerBounds(const Plato::InputData & aOptimizerNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData)
/******************************************************************************/
{
    if( aOptimizerNode.size<Plato::InputData>("BoundConstraint") )
    {
        auto tBoundConstraintNode = aOptimizerNode.get<Plato::InputData>("BoundConstraint");
        std::vector<double> tLowerBounds = Plato::Get::Doubles(tBoundConstraintNode, "Lower");
        aOptimizerEngineStageData.setLowerBoundValues(tLowerBounds);
    }
}

/******************************************************************************/
void parseUpperBounds(const Plato::InputData & aOptimizerNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData)
/******************************************************************************/
{
    if( aOptimizerNode.size<Plato::InputData>("BoundConstraint") )
    {
        auto tBoundConstraintNode = aOptimizerNode.get<Plato::InputData>("BoundConstraint");
        std::vector<double> tUpperBounds = Plato::Get::Doubles(tBoundConstraintNode, "Upper");
        aOptimizerEngineStageData.setUpperBoundValues(tUpperBounds);
    }
}

/******************************************************************************/
void parseOptimizerStages(const Plato::InputData & aOptimizerNode, Plato::OptimizerEngineStageData & aOptimizerEngineStageData)
/******************************************************************************/
{
    if( aOptimizerNode.size<Plato::InputData>("Convergence") )
    {
        Plato::InputData tConvergenceNode = aOptimizerNode.get<Plato::InputData>("Convergence");
        int tMaxNumIterations = Plato::Get::Int(tConvergenceNode, "MaxIterations");
        aOptimizerEngineStageData.setMaxNumIterations(tMaxNumIterations);
    }

    if( aOptimizerNode.size<Plato::InputData>("Output") )
    {
        Plato::InputData tOutputNode = aOptimizerNode.get<Plato::InputData>("Output");
        std::string tOutputStageName = tOutputNode.get<std::string>("OutputStage");
        aOptimizerEngineStageData.setOutputStageName(tOutputStageName);
    }

    if( aOptimizerNode.size<Plato::InputData>("CacheStage") )
    {
        Plato::InputData tCacheNode = aOptimizerNode.get<Plato::InputData>("CacheStage");
        std::string tCacheStageName = tCacheNode.get<std::string>("Name");
        aOptimizerEngineStageData.setCacheStageName(tCacheStageName);
    }

    Plato::Parse::parseInitialGuess(aOptimizerNode, aOptimizerEngineStageData);
    Plato::Parse::parseOptimizerOptions(aOptimizerNode, aOptimizerEngineStageData);
    Plato::Parse::parseOptimizationVariablesNames(aOptimizerNode, aOptimizerEngineStageData);
    Plato::Parse::parseConstraintStagesData(aOptimizerNode, aOptimizerEngineStageData);

    if( aOptimizerNode.size<Plato::InputData>("Objective") )
    {
        auto tObjectiveNode = aOptimizerNode.get<Plato::InputData>("Objective");
        Plato::Parse::parseObjectiveStagesData(tObjectiveNode, aOptimizerEngineStageData);
    }
    else
    {
        std::ostringstream tMsg;
        tMsg << "\n\n ********** PLATO ERROR: FILE = " << __FILE__ << ", FUNCTION = " << __PRETTY_FUNCTION__ << ", LINE = "
             << __LINE__ << ", MESSAGE: NO OBJECTIVE IS DEFINED. **********\n\n";
        throw Plato::ParsingException(tMsg.str().c_str());
    }

    Plato::Parse::parseLowerBounds(aOptimizerNode, aOptimizerEngineStageData);
    Plato::Parse::parseUpperBounds(aOptimizerNode, aOptimizerEngineStageData);

    bool tCheckGradient = Plato::Get::Bool(aOptimizerNode, "CheckGradient");
    aOptimizerEngineStageData.setCheckGradient(tCheckGradient);
    bool tCheckHessian = Plato::Get::Bool(aOptimizerNode, "CheckHessian");
    aOptimizerEngineStageData.setCheckHessian(tCheckHessian);
    bool tUseUserInitialGuess = Plato::Get::Bool(aOptimizerNode, "UseUserInitialGuess");
    aOptimizerEngineStageData.setUserInitialGuess(tUseUserInitialGuess);
}

} /* namespace Parse */

} /* namespace Plato */
