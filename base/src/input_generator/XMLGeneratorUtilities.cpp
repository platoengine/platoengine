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
 * XMLGeneratorUtilities.cpp
 *
 *  Created on: Nov 19, 2019
 *
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "Plato_FreeFunctions.hpp"
#include "XMLGeneratorPlatoAnalyzeProblem.hpp"
#include "XMLG_Macros.hpp"
#include <fstream>

namespace XMLGen
{

/******************************************************************************/
bool is_shape_optimization_problem(const XMLGen::InputData& aMetaData)
/******************************************************************************/
{
    if(aMetaData.optimization_parameters().optimizationType() == OT_SHAPE)
        return true;
    else
        return false;
}

/******************************************************************************/
void append_version_entry(pugi::xml_document& aDocument)
/******************************************************************************/
{
    auto tNode = aDocument.append_child(pugi::node_declaration);
    tNode.set_name("xml");
    pugi::xml_attribute tAttribute = tNode.append_attribute("version");
    tAttribute.set_value("1.0");
}

/******************************************************************************/
bool addChild(pugi::xml_node parent_node,
              const std::string &name,
              const std::string &value)
/******************************************************************************/
{
    pugi::xml_node tmp_node = parent_node.append_child(name.c_str());
    tmp_node = tmp_node.append_child(pugi::node_pcdata);
    tmp_node.set_value(value.c_str());
    return true;
}

/******************************************************************************/
bool addNTVParameter(pugi::xml_node parent_node,
                     const std::string &name,
                     const std::string &type,
                     const std::string &value)
/******************************************************************************/
{
    pugi::xml_node node = parent_node.append_child("Parameter");
    node.append_attribute("name") = name.c_str();
    node.append_attribute("type") = type.c_str();
    node.append_attribute("value") = value.c_str();
    return true;
}

/******************************************************************************/
pugi::xml_node createSingleUserNodalSharedData(pugi::xml_document &aDoc,
                                                             const std::string &aName,
                                                             const std::string &aType,
                                                             const std::string &aOwner,
                                                             const std::string &aUser)
/******************************************************************************/
{   
    pugi::xml_node sd_node = aDoc.append_child("SharedData");
    addChild(sd_node, "Name", aName);
    addChild(sd_node, "Type", aType);
    addChild(sd_node, "Layout", "Nodal Field");
    addChild(sd_node, "OwnerName", aOwner);
    addChild(sd_node, "UserName", aUser);
    return sd_node;
}

/******************************************************************************/
pugi::xml_node createSingleUserNodalSharedData(pugi::xml_node &aNode,
                                                             const std::string &aName,
                                                             const std::string &aType,
                                                             const std::string &aOwner,
                                                             const std::string &aUser)
/******************************************************************************/
{   
    pugi::xml_node sd_node = aNode.append_child("SharedData");
    addChild(sd_node, "Name", aName);
    addChild(sd_node, "Type", aType);
    addChild(sd_node, "Layout", "Nodal Field");
    addChild(sd_node, "OwnerName", aOwner);
    addChild(sd_node, "UserName", aUser);
    return sd_node;
}

/******************************************************************************/
pugi::xml_node createSingleUserElementSharedData(pugi::xml_document &aDoc,
                                                               const std::string &aName,
                                                               const std::string &aType,
                                                               const std::string &aOwner,
                                                               const std::string &aUser)
/******************************************************************************/
{
    pugi::xml_node sd_node = aDoc.append_child("SharedData");
    addChild(sd_node, "Name", aName);
    addChild(sd_node, "Type", aType);
    addChild(sd_node, "Layout", "Element Field");
    addChild(sd_node, "OwnerName", aOwner);
    addChild(sd_node, "UserName", aUser);
    return sd_node;
}

/******************************************************************************/
pugi::xml_node createSingleUserElementSharedData(pugi::xml_node &aNode,
                                                               const std::string &aName,
                                                               const std::string &aType,
                                                               const std::string &aOwner,
                                                               const std::string &aUser)
/******************************************************************************/
{
    pugi::xml_node sd_node = aNode.append_child("SharedData");
    addChild(sd_node, "Name", aName);
    addChild(sd_node, "Type", aType);
    addChild(sd_node, "Layout", "Element Field");
    addChild(sd_node, "OwnerName", aOwner);
    addChild(sd_node, "UserName", aUser);
    return sd_node;
}

/******************************************************************************/
pugi::xml_node createSingleUserGlobalSharedData(pugi::xml_document &aDoc,
                                                              const std::string &aName,
                                                              const std::string &aType,
                                                              const std::string &aSize,
                                                              const std::string &aOwner,
                                                              const std::string &aUser)
/******************************************************************************/
{
    pugi::xml_node sd_node = aDoc.append_child("SharedData");
    addChild(sd_node, "Name", aName);
    addChild(sd_node, "Type", aType);
    addChild(sd_node, "Layout", "Global");
    addChild(sd_node, "Size", aSize);
    addChild(sd_node, "OwnerName", aOwner);
    addChild(sd_node, "UserName", aUser);
    return sd_node;
}

/******************************************************************************/
pugi::xml_node createSingleUserGlobalSharedData(pugi::xml_node &aNode,
                                                              const std::string &aName,
                                                              const std::string &aType,
                                                              const std::string &aSize,
                                                              const std::string &aOwner,
                                                              const std::string &aUser)
/******************************************************************************/
{
    pugi::xml_node sd_node = aNode.append_child("SharedData");
    addChild(sd_node, "Name", aName);
    addChild(sd_node, "Type", aType);
    addChild(sd_node, "Layout", "Global");
    addChild(sd_node, "Size", aSize);
    addChild(sd_node, "OwnerName", aOwner);
    addChild(sd_node, "UserName", aUser);
    return sd_node;
}

/******************************************************************************/
pugi::xml_node createMultiUserGlobalSharedData(pugi::xml_document &aDoc,
                                                              const std::string &aName,
                                                              const std::string &aType,
                                                              const std::string &aSize,
                                                              const std::string &aOwner,
                                                              const std::vector<std::string> &aUsers)
/******************************************************************************/
{
    pugi::xml_node sd_node = aDoc.append_child("SharedData");
    addChild(sd_node, "Name", aName);
    addChild(sd_node, "Type", aType);
    addChild(sd_node, "Layout", "Global");
    addChild(sd_node, "Size", aSize);
    addChild(sd_node, "OwnerName", aOwner);
    for(size_t i=0; i<aUsers.size(); ++i)
    {
        addChild(sd_node, "UserName", aUsers[i]);
    }
    return sd_node;
}

/******************************************************************************/
void append_children
(const std::vector<std::string>& aKeys,
 const std::vector<std::string>& aValues,
 pugi::xml_node& aParentNode)
{
    for (auto& tKey : aKeys)
    {
        auto tIndex = &tKey - &aKeys[0];
        auto tLower = Plato::tolower(aValues[tIndex]);
        if (tLower.compare("ignore") != 0)
        {
            auto tChildNode = aParentNode.append_child(tKey.c_str());
            tChildNode = tChildNode.append_child(pugi::node_pcdata);
            tChildNode.set_value(aValues[tIndex].c_str());
        }
    }
}
// function append_children
/******************************************************************************/

/******************************************************************************/
void append_attributes
(const std::vector<std::string>& aKeys,
 const std::vector<std::string>& aValues,
 pugi::xml_node& aParentNode)
{
    for(auto& tKey : aKeys)
    {
        auto tIndex = &tKey - &aKeys[0];
        auto tLower = Plato::tolower(aValues[tIndex]);
        aParentNode.append_attribute(tKey.c_str()) = aValues[tIndex].c_str();
    }
}
// function append_attributes
/******************************************************************************/

/******************************************************************************/
void append_attributes
(const std::string& aNodeName,
 const std::vector<std::string>& aKeywords,
 const std::vector<std::string>& aValues,
 pugi::xml_document& aDocument)
{
    auto tNode = aDocument.append_child(aNodeName.c_str());
    for(auto& tKeyword : aKeywords)
    {
        auto tIndex = &tKeyword - &aKeywords[0];
        auto tLower = Plato::tolower(aValues[tIndex]);
        tNode.append_attribute(tKeyword.c_str()) = aValues[tIndex].c_str();
    }
}
// function append_attributes
/******************************************************************************/

/******************************************************************************/
void append_parameter_plus_attributes
(const std::vector<std::string>& aKeys,
 const std::vector<std::string>& aValues,
 pugi::xml_node& aParentNode)
{
    std::vector<std::string> tCopy = aValues;
    XMLGen::to_lower(tCopy);
    auto tIgnoreAttribute = std::find(tCopy.begin(), tCopy.end(), "ignore");
    if(tIgnoreAttribute == tCopy.end())
    {
        auto tChild = aParentNode.append_child("Parameter");
        XMLGen::append_attributes(aKeys, aValues, tChild);
    }
}
// function append_parameter_plus_attributes
/******************************************************************************/

/******************************************************************************/
size_t compute_greatest_divisor(const size_t& aDividend, size_t aDivisor)
{
    if (aDivisor == 0u)
    {
        THROWERR("Compute Greatest Divisor: Divide by zero.")
    }
    while (aDividend % aDivisor != 0u)
    {
        --aDivisor;
    }
    return aDivisor;
}
// compute_greatest_divisor
/******************************************************************************/

/******************************************************************************/
std::string transform_tokens(const std::vector<std::string> &aTokens)
{
    if(aTokens.empty())
    {
        return std::string("");
    }

    std::string tOutput;
    auto tEndIndex = aTokens.size() - 1u;
    auto tEndIterator = std::next(aTokens.begin(), tEndIndex);
    for(auto tItr = aTokens.begin(); tItr != tEndIterator; ++tItr)
    {
        auto tIndex = std::distance(aTokens.begin(), tItr);
        tOutput += aTokens[tIndex] + ", ";
    }
    tOutput += aTokens[tEndIndex];

    return tOutput;
}
// function transform_tokens
/******************************************************************************/

/******************************************************************************/
std::stringstream read_data_from_file(const std::string& aFilename)
{
    std::ifstream tReadFile;
    tReadFile.open(aFilename);
    std::string tInputString;
    std::stringstream tReadData;
    while (tReadFile >> tInputString)
    {
        tReadData << tInputString.c_str();
    }
    tReadFile.close();
    return (tReadData);
}
// function read_data_from_file
/******************************************************************************/

/******************************************************************************/
void set_key_value
(const std::string& aKey,
 const std::string& aValue,
 std::unordered_map<std::string, std::string>& aKeyToValueMap)
{
    auto tItr = aKeyToValueMap.find(aKey);
    if(tItr == aKeyToValueMap.end())
    {
        THROWERR(std::string("Set Key Value: Key '" + aKey + "' is not supported."))
    }
    tItr->second = aValue.empty() ? std::string("IGNORE") : aValue;
}
// function set_key_value
/******************************************************************************/

/******************************************************************************/
std::string set_value_keyword_to_ignore_if_empty
(const std::string& aValue)
{
/*
    if(aValue.empty())
    {
        std::string tContextString = "\n";
        PRINTIGNOREINFO(aValue, tContextString);
    }
*/

    auto tMyValue = aValue.empty() ? std::string("IGNORE") : aValue;
    return tMyValue;
}
// function set_value_keyword_to_ignore_if_empty
/******************************************************************************/

/******************************************************************************/
void set_value_keyword_to_ignore_if_empty
(std::vector<std::string>& aValues)
{
    for(auto& tValue : aValues)
    {
/*
        if(tValue.empty())
        {
            std::string tContextString = "\n";
            PRINTIGNOREINFO(tValue, tContextString);
        }
*/
        auto tMyValue = tValue.empty() ? std::string("IGNORE") : tValue;
        tValue = tMyValue;
    }
}
// function set_value_keyword_to_ignore_if_empty
/******************************************************************************/

/******************************************************************************/
std::vector<std::string> transform_key_tokens
(const std::unordered_map<std::string, std::string> &aKeyToValueMap)
{
    std::vector<std::string> tKeys;
    for(auto& tPair : aKeyToValueMap)
    {
        tKeys.push_back(tPair.first);
    }
    return tKeys;
}
// function transform_key_tokens
/******************************************************************************/

/******************************************************************************/
std::vector<std::string> transform_value_tokens
(const std::unordered_map<std::string, std::string> &aKeyToValueMap)
{
    std::vector<std::string> tValues;
    for(auto& tPair : aKeyToValueMap)
    {
        tValues.push_back(tPair.second);
    }
    return tValues;
}
// function transform_value_tokens
/******************************************************************************/

void assert_is_positive_integer(const std::string& aString)
{
  if(aString.empty() || !std::isdigit(aString[0]) || aString == "0")
    THROWERR("expected a positive integer\n")

  char * p;
  strtol(aString.c_str(), &p, 10);
  bool is_positive_integer = *p == 0;

  if(!is_positive_integer)
    THROWERR("expected a positive integer\n")
}

/******************************************************************************/
void append_include_defines_xml_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    if(XMLGen::is_robust_optimization_problem(aMetaData) ||
       aMetaData.optimization_parameters().optimizationType() == OT_SHAPE)
    {
        auto tInclude = aDocument.append_child("include");
        XMLGen::append_attributes({"filename"}, {"defines.xml"}, tInclude);
    }
}
/******************************************************************************/

std::string get_salinas_service_id(const XMLGen::InputData& aXMLMetaData)
{
    std::string tServiceID = "";
    for(auto &tID : aXMLMetaData.objective.serviceIDs)
    {
        auto &tService = aXMLMetaData.service(tID);
        if(tService.code() == "sierra_sd")
        {
            tServiceID = tID;
            break;
        }
    }
    if(tServiceID == "")
    {
        for(auto &tConstraint : aXMLMetaData.constraints)
        {
            auto tID = tConstraint.service();
            auto &tService = aXMLMetaData.service(tID);
            if(tService.code() == "sierra_sd")
            {
                tServiceID = tID;
                break;
            }
        } 
    }
    return tServiceID;
}

}
// namespace XMLGen
