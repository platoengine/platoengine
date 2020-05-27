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
 * Plato_Test_Geometry_MLS.cpp
 *
 *  Created on: Feb 27, 2018
 */

#include <gtest/gtest.h>

#include <map>
#include <cmath>
#include <numeric>

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorDefinesFileUtilities.hpp"
#include "XMLGeneratorInterfaceFileUtilities.hpp"
#include "XMLGeneratorRandomInterfaceFileUtilities.hpp"

#include "Plato_SromXMLGenTools.hpp"
#include "XMLGenerator_UnitTester.hpp"
#include "DefaultInputGenerator_UnitTester.hpp"
#include "ComplianceMinTOPlatoAnalyzeInputGenerator_UnitTester.hpp"
#include "ComplianceMinTOPlatoAnalyzeUncertInputGenerator_UnitTester.hpp"
#include "Plato_Vector3DVariations.hpp"
#include "XML_GoldValues.hpp"

#include "XMLG_Macros.hpp"
#include "XMLGeneratorParser.hpp"

const int MAX_CHARS_PER_LINE = 512;

namespace XMLGen
{

struct ParseObjective
{
private:
    XMLGen::UseCaseTags mTags; /*!< map from valid tags to valid tokens-value pairs, i.e. map<tag, pair<tokens,value> > */
    std::vector<XMLGen::Objective> mData; /*!< list of uncertainty metadata */

private:
    /******************************************************************************//**
     * \fn allocate
     * \brief Allocate map from valid tags to valid tokens-value pair
    **********************************************************************************/
    void allocate()
    {
        mTags.clear();
        mTags.insert({ "code", { {"code"}, "" } });
        mTags.insert({ "type", { {"type"}, "" } });
        mTags.insert({ "name", { {"name"}, "" } });
        mTags.insert({ "weight", { {"weight"}, "" } });
        mTags.insert({ "load ids", { {"load", "ids"}, "" } });
        mTags.insert({ "stress limit", { {"stress", "limit"}, "" } });
        mTags.insert({ "number ranks", { {"number", "ranks"}, "" } });
        mTags.insert({ "min frequency", { {"min", "frequency"}, "" } });
        mTags.insert({ "max frequency", { {"max", "frequency"}, "" } });
        mTags.insert({ "frequency step", { {"frequency", "step"}, "" } });
        mTags.insert({ "normalize objective", { {"normalize", "objective"}, "" } });
        mTags.insert({ "distribute objective", { {"distribute", "objective"}, "" } });
        mTags.insert({ "ls tet type", { {"ls", "tet", "type"}, "" } });
        mTags.insert({ "multi load case", { {"multi", "load", "case"}, "" } });
        mTags.insert({ "limit power max", { {"limit", "power", "max"}, "" } });
        mTags.insert({ "limit power min", { {"limit", "power", "min"}, "" } });
        mTags.insert({ "load case weights", { {"load", "case", "weights"}, "" } });
        mTags.insert({ "limit reset count", { {"limit", "reset", "count"}, "" } });
        mTags.insert({ "reference frf file", { {"reference", "frf", "file"}, "" } });
        mTags.insert({ "frf match nodesets", { {"frf", "match", "nodesets"}, "" } });
        mTags.insert({ "stress ramp factor", { {"stress", "ramp", "factor"}, "" } });
        mTags.insert({ "stress favor final", { {"stress", "favor", "final"}, "" } });
        mTags.insert({ "output for plotting", { {"output", "for", "plotting"}, "" } });
        mTags.insert({ "volume penalty bias", { {"volume", "penalty", "bias"}, "" } });
        mTags.insert({ "volume penalty power", { {"volume", "penalty", "power"}, "" } });
        mTags.insert({ "volume misfit target", { {"volume", "misfit", "target"}, "" } });
        mTags.insert({ "analyze new workflow", { {"analyze", "new", "workflow"}, "" } });
        mTags.insert({ "stress favor updates", { {"stress", "favor", "updates"}, "" } });
        mTags.insert({ "scmm initial penalty", { {"scmm", "initial", "penalty"}, "" } });
        mTags.insert({ "stress p norm power", { {"stress", "p", "norm", "power"}, "" } });
        mTags.insert({ "raleigh damping beta", { {"raleigh", "damping", "beta"}, "" } });
        mTags.insert({ "raleigh damping alpha", { {"raleigh", "damping", "alpha"}, "" } });
        mTags.insert({ "complex error measure", { {"complex", "error", "measure"}, "" } });
        mTags.insert({ "relative stress limit", { {"relative", "stress", "limit"}, "" } });
        mTags.insert({ "volume penalty divisor", { {"volume", "penalty", "divisor"}, "" } });
        mTags.insert({ "boundary condition ids", { {"boundary", "condition", "ids"}, "" } });
        mTags.insert({ "weightmass scale factor", { {"weightmass", "scale", "factor"}, "" } });
        mTags.insert({ "stress inequality power", { {"stress", "inequality", "power"}, "" } });
        mTags.insert({ "scmm constraint exponent", { {"scmm", "constraint", "exponent"}, "" } });
        mTags.insert({ "limit reset subfrequency", { {"limit", "reset", "subfrequency"}, "" } });
        mTags.insert({ "analysis solver tolerance", { {"analysis", "solver", "tolerance"}, "" } });
        mTags.insert({ "limit power feasible bias", { {"limit", "power", "feasible", "bias"}, "" } });
        mTags.insert({ "inequality feasibility scale", { {"inequality","feasibility","scale"}, "" } });
        mTags.insert({ "limit power feasible slope", { {"limit", "power", "feasible", "slope"}, "" } });
        mTags.insert({ "limit power infeasible bias", { {"limit", "power", "infeasible", "bias"}, "" } });
        mTags.insert({ "limit power infeasible slope", { {"limit", "power", "infeasible", "slope"}, "" } });
        mTags.insert({ "distribute objective at most", { {"distribute", "objective", "at", "most"}, "" } });
        mTags.insert({ "inequality infeasibility scale", { {"inequality", "infeasibility", "scale"}, "" } });
        mTags.insert({ "scmm penalty expansion factor", { {"scmm", "penalty", "expansion", "factor"}, "" } });
        mTags.insert({ "inequality allowable feasibility upper", { {"inequality", "allowable", "feasibility", "upper"}, "" } });
    }

    /******************************************************************************//**
     * \fn erase
     * \brief Erases value key content in map from valid tags to valid tokens-value pair.
    **********************************************************************************/
    void erase()
    {
        for(auto& tTag : mTags)
        {
            tTag.second.second.clear();
        }
    }

    /******************************************************************************//**
     * \fn parseMetadata
     * \brief Parse uncertainty blocks.
     * \param [in] aInputFile input file metadata
    **********************************************************************************/
    void parseMetadata(std::istream& aInputFile)
    {
        constexpr int tMAX_CHARS_PER_LINE = 512;
        std::vector<char> tBuffer(tMAX_CHARS_PER_LINE);
        // found an uncertainty. parse it.
        while (!aInputFile.eof())
        {
            std::vector<std::string> tTokens;
            aInputFile.getline(tBuffer.data(), tMAX_CHARS_PER_LINE);
            XMLGen::parse_tokens(tBuffer.data(), tTokens);
            XMLGen::to_lower(tTokens);

            std::string tTag;
            if (XMLGen::parse_single_value(tTokens, std::vector<std::string> { "end", "objective" }, tTag))
            {
                break;
            }
            XMLGen::parse_tag_values(tTokens, mTags);
        }
    }

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return objectives metadata.
     * \return objectives metadata
    **********************************************************************************/
    std::vector<XMLGen::Objective> data() const
    {
        return mData;
    }

    /******************************************************************************//**
     * \fn parse
     * \brief Parse objectives metadata.
     * \param [in] aInputFile input file metadata
    **********************************************************************************/
    void parse(std::istream& aInputFile)
    {
        mData.clear();
        this->allocate();
        constexpr int MAX_CHARS_PER_LINE = 512;
        std::vector<char> tBuffer(MAX_CHARS_PER_LINE);
        while (!aInputFile.eof())
        {
            // read an entire line into memory
            std::vector<std::string> tTokens;
            aInputFile.getline(tBuffer.data(), MAX_CHARS_PER_LINE);
            XMLGen::parse_tokens(tBuffer.data(), tTokens);
            XMLGen::to_lower(tTokens);

            std::string tTag;
            std::vector<std::string> tMatchTokens;
            if(XMLGen::parse_single_value(tTokens, tMatchTokens = {"begin","objective"}, tTag))
            {
                XMLGen::Objective tMetadata;
                this->erase();
                this->parseMetadata(aInputFile);
                //this->setMetadata(tMetadata);
                //this->checkMetadata(tMetadata);
                mData.push_back(tMetadata);
            }
        }
    }
};
// struct ParseObjective

}
























namespace PlatoTestXMLGenerator
{

inline void test_children
(const std::vector<std::string>& aKeys,
 const std::vector<std::string>& aValues,
 pugi::xml_node& aParentNode)
{
    ASSERT_TRUE(aKeys.size() == aValues.size());
    for(auto& tChild : aParentNode.children())
    {
        auto tItr = std::find(aKeys.begin(), aKeys.end(), tChild.name());
        ASSERT_TRUE(tItr != aKeys.end());
        ASSERT_STREQ(tItr->c_str(), tChild.name());

        tItr = std::find(aValues.begin(), aValues.end(), tChild.child_value());
        ASSERT_TRUE(tItr != aValues.end());
        ASSERT_STREQ(tItr->c_str(), tChild.child_value());
    }
}

inline void test_attributes
(const std::vector<std::string>& aKeys,
 const std::vector<std::string>& aValues,
 pugi::xml_node& aParentNode)
{
    ASSERT_TRUE(aKeys.size() == aValues.size());
    for(auto& tAttribute : aParentNode.attributes())
    {
        auto tItr = std::find(aKeys.begin(), aKeys.end(), tAttribute.name());
        ASSERT_TRUE(tItr != aKeys.end());
        ASSERT_STREQ(tItr->c_str(), tAttribute.name());

        tItr = std::find(aValues.begin(), aValues.end(), tAttribute.value());
        ASSERT_TRUE(tItr != aValues.end());
        ASSERT_STREQ(tItr->c_str(), tAttribute.value());
    }
}

TEST(PlatoTestXMLGenerator, SetKeyValue)
{
    std::unordered_map<std::string, std::string> tKeyToValueMap =
        { {"ValueName", "Constraint Value"}, {"ValueStageName", "Calculate Constraint Value"},
          {"GradientName", "Constraint Gradient"}, {"GradientStageName", "Calculate Constraint Gradient"},
          {"ReferenceValueName", "Reference Value"}, {"NormalizedTargetValue", ""}, {"AbsoluteTargetValue", ""} };

    // TEST 1: EMPTY VALUE -> RESULT = IGNORE
    XMLGen::set_key_value("AbsoluteTargetValue", "", tKeyToValueMap);
    ASSERT_STREQ("IGNORE", tKeyToValueMap.find("AbsoluteTargetValue")->second.c_str());

    // TEST 2: SET VALUE
    XMLGen::set_key_value("AbsoluteTargetValue", "10", tKeyToValueMap);
    ASSERT_STREQ("10", tKeyToValueMap.find("AbsoluteTargetValue")->second.c_str());
}

TEST(PlatoTestXMLGenerator, SetValueKeywordToIgnoreIfEmpty)
{
    // TEST 1: SET EMPTY VALUES TO IGNORE
    std::vector<std::string> tValues = {"hello", "10", "", ""};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    std::vector<std::string> tGold = {"hello", "10", "IGNORE", "IGNORE"};
    for(auto& tValue : tValues)
    {
        auto tIndex = &tValue - &tValues[0];
        ASSERT_STREQ(tGold[tIndex].c_str(), tValue.c_str());
    }

    // TEST 2: NO CHANGE
    tValues = {"hello", "10"};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    tGold = {"hello", "10"};
    for(auto& tValue : tValues)
    {
        auto tIndex = &tValue - &tValues[0];
        ASSERT_STREQ(tGold[tIndex].c_str(), tValue.c_str());
    }
}

TEST(PlatoTestXMLGenerator, TransformKeyTokens)
{
    std::unordered_map<std::string, std::string> tKeyToValueMap =
        { {"ValueName", "Constraint Value"}, {"ValueStageName", "Calculate Constraint Value"},
          {"GradientName", "Constraint Gradient"}, {"GradientStageName", "Calculate Constraint Gradient"},
          {"ReferenceValueName", "Reference Value"}, {"NormalizedTargetValue", "1"}, {"AbsoluteTargetValue", "10"} };
    auto tKeys = XMLGen::transform_key_tokens(tKeyToValueMap);

    std::vector<std::string> tGold = {"ValueName", "ValueStageName", "GradientName", "GradientStageName",
        "ReferenceValueName", "NormalizedTargetValue", "AbsoluteTargetValue"};
    ASSERT_EQ(7u, tKeys.size());
    for(auto& tKey : tKeys)
    {
        auto tItr = std::find(tGold.begin(), tGold.end(), tKey);
        ASSERT_TRUE(tItr != tGold.end());
        ASSERT_STREQ(tItr->c_str(), tKey.c_str());
    }
}

TEST(PlatoTestXMLGenerator, TransformValueTokens)
{
    std::unordered_map<std::string, std::string> tKeyToValueMap =
        { {"ValueName", "Constraint Value"}, {"ValueStageName", "Calculate Constraint Value"},
          {"GradientName", "Constraint Gradient"}, {"GradientStageName", "Calculate Constraint Gradient"},
          {"ReferenceValueName", "Reference Value"}, {"NormalizedTargetValue", "1"}, {"AbsoluteTargetValue", "10"} };
    auto tValues = XMLGen::transform_value_tokens(tKeyToValueMap);

    std::vector<std::string> tGold = {"Constraint Value", "Calculate Constraint Value",
        "Constraint Gradient", "Calculate Constraint Gradient", "Reference Value", "1", "10"};
    ASSERT_EQ(7u, tValues.size());
    for(auto& tValue : tValues)
    {
        auto tItr = std::find(tGold.begin(), tGold.end(), tValue);
        ASSERT_TRUE(tItr != tGold.end());
        ASSERT_STREQ(tItr->c_str(), tValue.c_str());
    }
}

TEST(PlatoTestXMLGenerator, AppendChilds)
{
    pugi::xml_document tDocument;
    auto tSharedData = tDocument.append_child("SharedData");
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Lower Bound Value", "Scalar", "Global", "1", "PlatoMain", "PlatoMain"};
    XMLGen::append_children(tKeys, tValues, tSharedData);
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);
}

TEST(PlatoTestXMLGenerator, AppendAttributes)
{
    pugi::xml_document tDocument;
    auto tFor = tDocument.append_child("For");
    ASSERT_STREQ("For", tFor.name());
    std::vector<std::string> tKeys = {"var", "in"};
    std::vector<std::string> tValues = {"PerformerIndex", "Performers"};
    XMLGen::append_attributes(tKeys, tValues, tFor);
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tFor);
}

TEST(PlatoTestXMLGenerator, AppendNondeterministicSharedData)
{
    pugi::xml_document tDocument;
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Scalar", "Global", "1", "plato analyze {PerformerIndex}", "PlatoMain"};
    XMLGen::append_nondeterministic_shared_data(tKeys, tValues, tDocument);

    // TEST RESULTS AGAINS GOLD VALUES
    std::vector<std::string> tGoldOuterAttributeKeys = {"var", "in"};
    std::vector<std::string> tGoldOuterAttributeValues = {"PerformerIndex", "Performers"};
    auto tOuterForNode = tDocument.child("For");
    ASSERT_FALSE(tOuterForNode.empty());
    PlatoTestXMLGenerator::test_attributes(tGoldOuterAttributeKeys, tGoldOuterAttributeValues, tOuterForNode);

    auto tInnerForNode = tOuterForNode.child("For");
    ASSERT_FALSE(tInnerForNode.empty());
    std::vector<std::string> tGoldInnerAttributeKeys = {"var", "in"};
    std::vector<std::string> tGoldInnerAttributeValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldInnerAttributeKeys, tGoldInnerAttributeValues, tInnerForNode);

    auto tSharedData = tInnerForNode.child("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);
}

TEST(PlatoTestXMLGenerator, AppendNondeterministicCriterionSharedData_ErrorEmptyObjectiveList)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    ASSERT_THROW(XMLGen::append_criterion_shared_data_for_nondeterministic_usecase("Objective", tInputData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendNondeterministicCriterionSharedData)
{
    pugi::xml_document tDocument;
    XMLGen::Objective tObjective;
    tObjective.performer_name = "plato analyze";
    XMLGen::InputData tInputData;
    tInputData.objectives.push_back(tObjective);

    XMLGen::append_criterion_shared_data_for_nondeterministic_usecase("Objective", tInputData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tGoldOuterAttributeKeys = {"var", "in"};
    std::vector<std::string> tGoldOuterAttributeValues = {"PerformerIndex", "Performers"};
    std::vector<std::string> tGoldInnerAttributeKeys = {"var", "in"};
    std::vector<std::string> tGoldInnerAttributeValues = {"PerformerSampleIndex", "PerformerSamples"};

    std::vector<std::string> tTemp = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataKeys;
    tGoldSharedDataKeys.push_back(std::make_pair("Objective Value", tTemp));
    tTemp = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tGoldSharedDataKeys.push_back(std::make_pair("Objective Gradient", tTemp));

    tTemp = {"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Scalar", "Global", "1", "plato analyze {PerformerIndex}", "PlatoMain"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataValues;
    tGoldSharedDataValues.push_back(std::make_pair("Objective Value", tTemp));
    tTemp = {"Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Scalar", "Nodal Field", "plato analyze {PerformerIndex}", "PlatoMain"};
    tGoldSharedDataValues.push_back(std::make_pair("Objective Gradient", tTemp));

    auto tKeys = tGoldSharedDataKeys.begin();
    auto tValues = tGoldSharedDataValues.begin();
    for (auto &tChild : tDocument.children())
    {
        // TEST OUTER LOOP ATTRIBUTES
        ASSERT_STREQ("For", tChild.name());
        PlatoTestXMLGenerator::test_attributes(tGoldOuterAttributeKeys, tGoldOuterAttributeValues, tChild);

        // TEST INNER LOOP ATTRIBUTES
        auto tGoldInnerAttributeKeysItr = tGoldInnerAttributeKeys.begin();
        auto tGoldInnerAttributeValuesItr = tGoldInnerAttributeValues.begin();
        auto tInnerForNode = tChild.child("For");
        for (auto &tAttribute : tInnerForNode.attributes())
        {
            ASSERT_STREQ(tGoldInnerAttributeKeysItr->c_str(), tAttribute.name());
            std::advance(tGoldInnerAttributeKeysItr, 1);
            ASSERT_STREQ(tGoldInnerAttributeValuesItr->c_str(), tAttribute.value());
            std::advance(tGoldInnerAttributeValuesItr, 1);

            // TEST INNER LOOP SHARED DATA
            auto tSharedData = tInnerForNode.child("SharedData");
            ASSERT_FALSE(tSharedData.empty());
            ASSERT_STREQ("SharedData", tSharedData.name());
            PlatoTestXMLGenerator::test_children(tKeys->second, tValues->second, tSharedData);
        }
        std::advance(tKeys, 1);
        std::advance(tValues, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendQoiSharedDataForNondeterministicUsecase_ErrorEmptyObjectiveList)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    ASSERT_THROW(XMLGen::append_qoi_shared_data_for_nondeterministic_usecase(tInputData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendQoiSharedDataForNondeterministicUsecase)
{
    pugi::xml_document tDocument;
    XMLGen::Objective tObjective;
    tObjective.performer_name = "plato analyze";
    XMLGen::InputData tInputData;
    tInputData.objectives.push_back(tObjective);

    ASSERT_NO_THROW(XMLGen::append_qoi_shared_data_for_nondeterministic_usecase(tInputData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tGoldOuterAttributeKeys = {"var", "in"};
    std::vector<std::string> tGoldOuterAttributeValues = {"PerformerIndex", "Performers"};
    std::vector<std::string> tGoldInnerAttributeKeys = {"var", "in"};
    std::vector<std::string> tGoldInnerAttributeValues = {"PerformerSampleIndex", "PerformerSamples"};

    std::vector<std::string> tTemp = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataKeys;
    tGoldSharedDataKeys.push_back(std::make_pair("Von Mises", tTemp));
    tTemp = {"Von Mises {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Scalar", "Element Field", "plato analyze {PerformerIndex}", "PlatoMain"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataValues;
    tGoldSharedDataValues.push_back(std::make_pair("Von Mises", tTemp));

    auto tKeys = tGoldSharedDataKeys.begin();
    auto tValues = tGoldSharedDataValues.begin();
    for (auto &tChild : tDocument.children())
    {
        // TEST OUTER LOOP ATTRIBUTES
        ASSERT_STREQ("For", tChild.name());
        PlatoTestXMLGenerator::test_attributes(tGoldOuterAttributeKeys, tGoldOuterAttributeValues, tChild);

        // TEST INNER LOOP ATTRIBUTES
        auto tGoldInnerAttributeKeysItr = tGoldInnerAttributeKeys.begin();
        auto tGoldInnerAttributeValuesItr = tGoldInnerAttributeValues.begin();
        auto tInnerForNode = tChild.child("For");
        for (auto &tAttribute : tInnerForNode.attributes())
        {
            ASSERT_STREQ(tGoldInnerAttributeKeysItr->c_str(), tAttribute.name());
            std::advance(tGoldInnerAttributeKeysItr, 1);
            ASSERT_STREQ(tGoldInnerAttributeValuesItr->c_str(), tAttribute.value());
            std::advance(tGoldInnerAttributeValuesItr, 1);

            // TEST INNER LOOP SHARED DATA
            auto tSharedData = tInnerForNode.child("SharedData");
            ASSERT_FALSE(tSharedData.empty());
            ASSERT_STREQ("SharedData", tSharedData.name());
            PlatoTestXMLGenerator::test_children(tKeys->second, tValues->second, tSharedData);
        }
        std::advance(tKeys, 1);
        std::advance(tValues, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendLowerBoundsSharedData)
{
    pugi::xml_document tDocument;
    XMLGen::append_lower_bounds_shared_data(tDocument);

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tTemp = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataKeys;
    tGoldSharedDataKeys.push_back(std::make_pair("Lower Bound Value", tTemp));
    tTemp = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tGoldSharedDataKeys.push_back(std::make_pair("Lower Bound Vector", tTemp));

    tTemp = {"Lower Bound Value", "Scalar", "Global", "1", "PlatoMain", "PlatoMain"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataValues;
    tGoldSharedDataValues.push_back(std::make_pair("Lower Bound Value", tTemp));
    tTemp = {"Lower Bound Vector", "Scalar", "Nodal Field", "PlatoMain", "PlatoMain"};
    tGoldSharedDataValues.push_back(std::make_pair("Lower Bound Vector", tTemp));

    auto tKeys = tGoldSharedDataKeys.begin();
    auto tValues = tGoldSharedDataValues.begin();
    for (auto &tOuterChild : tDocument.children())
    {
        PlatoTestXMLGenerator::test_children(tKeys->second, tValues->second, tOuterChild);
        std::advance(tKeys, 1);
        std::advance(tValues, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendUpperBoundsSharedData)
{
    pugi::xml_document tDocument;
    XMLGen::append_upper_bounds_shared_data(tDocument);

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tTemp = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataKeys;
    tGoldSharedDataKeys.push_back(std::make_pair("Upper Bound Value", tTemp));
    tTemp = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tGoldSharedDataKeys.push_back(std::make_pair("Upper Bound Vector", tTemp));

    tTemp = {"Upper Bound Value", "Scalar", "Global", "1", "PlatoMain", "PlatoMain"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataValues;
    tGoldSharedDataValues.push_back(std::make_pair("Upper Bound Value", tTemp));
    tTemp = {"Upper Bound Vector", "Scalar", "Nodal Field", "PlatoMain", "PlatoMain"};
    tGoldSharedDataValues.push_back(std::make_pair("Upper Bound Vector", tTemp));

    auto tKeys = tGoldSharedDataKeys.begin();
    auto tValues = tGoldSharedDataValues.begin();
    for (auto &tOuterChild : tDocument.children())
    {
        PlatoTestXMLGenerator::test_children(tKeys->second, tValues->second, tOuterChild);
        std::advance(tKeys, 1);
        std::advance(tValues, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendDesignVolumeSaredData)
{
    pugi::xml_document tDocument;
    XMLGen::append_design_volume_shared_data(tDocument);

    // TEST RESULTS AGAINST GOLD VALUES
    auto tSharedData = tDocument.child("SharedData");
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tGoldKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tGoldValues = {"Reference Value", "Scalar", "Global", "1", "PlatoMain", "PlatoMain"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSharedData);
}

TEST(PlatoTestXMLGenerator, AppendCriterionSharedData_ErrorEmptyObjectiveList)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    ASSERT_THROW(XMLGen::append_criterion_shared_data("Objective", tInputData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendCriterionSharedData)
{
    pugi::xml_document tDocument;
    XMLGen::Objective tObjective;
    tObjective.performer_name = "plato analyze";
    XMLGen::InputData tInputData;
    tInputData.objectives.push_back(tObjective);

    ASSERT_NO_THROW(XMLGen::append_criterion_shared_data("Objective", tInputData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tTemp = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataKeys;
    tGoldSharedDataKeys.push_back(std::make_pair("Objective Value", tTemp));
    tTemp = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tGoldSharedDataKeys.push_back(std::make_pair("Objective Gradient", tTemp));

    tTemp = {"Objective Value", "Scalar", "Global", "1", "plato analyze", "PlatoMain"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataValues;
    tGoldSharedDataValues.push_back(std::make_pair("Objective Value", tTemp));
    tTemp = {"Objective Gradient", "Scalar", "Nodal Field", "plato analyze", "PlatoMain"};
    tGoldSharedDataValues.push_back(std::make_pair("Objective Gradient", tTemp));

    auto tKeys = tGoldSharedDataKeys.begin();
    auto tValues = tGoldSharedDataValues.begin();
    for (auto &tOuterChild : tDocument.children())
    {
        PlatoTestXMLGenerator::test_children(tKeys->second, tValues->second, tOuterChild);
        std::advance(tKeys, 1);
        std::advance(tValues, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendControlSharedData)
{
    pugi::xml_document tDocument;
    XMLGen::append_control_shared_data(tDocument);

    // TEST RESULTS AGAINST GOLD VALUES
    auto tSharedData = tDocument.child("SharedData");
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tGoldKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    std::vector<std::string> tGoldValues = {"Control", "Scalar", "Nodal Field", "PlatoMain", "PlatoMain"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSharedData);
}

TEST(PlatoTestXMLGenerator, AppendTopologySharedDataForNondeterministicUseCase_ErrorEmptyObjectiveList)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    ASSERT_THROW(XMLGen::append_topology_shared_data_for_nondeterministic_usecase(tInputData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendTopologySharedDataForNondeterministicUseCase)
{
    pugi::xml_document tDocument;
    XMLGen::Objective tObjective;
    tObjective.performer_name = "plato analyze";
    XMLGen::InputData tInputData;
    tInputData.objectives.push_back(tObjective);

    ASSERT_NO_THROW(XMLGen::append_topology_shared_data_for_nondeterministic_usecase(tInputData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tSharedData = tDocument.child("SharedData");
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tGoldKeys = {"Name", "Type", "Layout", "OwnerName", "UserName", "For"};
    std::vector<std::string> tGoldValues = {"Topology", "Scalar", "Nodal Field", "PlatoMain", "PlatoMain", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSharedData);

    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    auto tForNode = tSharedData.child("For");
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tForNode);
    PlatoTestXMLGenerator::test_children({"UserName"}, {"plato analyze {PerformerIndex}"}, tForNode);
}

TEST(PlatoTestXMLGenerator, AppendPlatoMainPerformer)
{
    pugi::xml_document tDocument;
    XMLGen::append_plato_main_performer(tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tPerformer = tDocument.child("Performer");
    std::vector<std::string> tGoldKeys = {"Name", "Code", "PerformerID"};
    std::vector<std::string> tGoldValues = {"PlatoMain", "PlatoMain", "0"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tPerformer);
}

TEST(PlatoTestXMLGenerator, AppendPhysicsPerformersForNondeterministicUsecase_ErrorEmptyObjectiveList)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    ASSERT_THROW(XMLGen::append_physics_performers_for_nondeterministic_usecase(tInputData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendPhysicsPerformersForNondeterministicUsecase)
{
    pugi::xml_document tDocument;
    XMLGen::Objective tObjective;
    tObjective.code_name = "analyze";
    tObjective.performer_name = "plato analyze";
    XMLGen::InputData tInputData;
    tInputData.objectives.push_back(tObjective);

    ASSERT_NO_THROW(XMLGen::append_topology_shared_data_for_nondeterministic_usecase(tInputData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tPerformer = tDocument.child("Performer");
    std::vector<std::string> tGoldKeys = {"PerformerID", "For"};
    std::vector<std::string> tGoldValues = {"1", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tPerformer);

    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    auto tForNode = tPerformer.child("For");
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tForNode);

    tGoldKeys = {"Name", "Code"};
    tGoldValues = {"plato analyze {PerformerIndex}", "analyze"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tForNode);
}

TEST(PlatoTestXMLGenerator, AppendFilterControlOperation)
{
    pugi::xml_document tDocument;
    XMLGen::append_filter_control_operation(tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    std::vector<std::string> tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    std::vector<std::string> tGoldValues = {"Filter Control", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tInput = tDocument.child("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);

    auto tOutput = tDocument.child("Output");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Filtered Field", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendFilterCriterionGradientSamplesOperation)
{
    pugi::xml_document tDocument;
    XMLGen::append_filter_criterion_gradient_samples_operation("Objective", tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOuterFor = tDocument.child("For");
    std::vector<std::string> tGoldKeys = {"var", "in"};
    std::vector<std::string> tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tOuterFor);

    auto tInnerFor = tOuterFor.child("Operation").child("For");
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tInnerFor);

    auto tOperation = tInnerFor.child("Operation");
    tGoldKeys = {"Name", "PerformerName", "Input", "Input", "Output"};
    tGoldValues = {"Filter Gradient", "PlatoMain", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tOutput = tOperation.child("Output");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Filtered Gradient", "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);

    auto tInput = tOperation.next_sibling("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);

    tInput = tOperation.next_sibling("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Gradient", "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);
}

TEST(PlatoTestXMLGenerator, AppendFilterCriterionGradientOperation)
{
    pugi::xml_document tDocument;
    XMLGen::append_filter_criterion_gradient_operation("Objective", tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    std::vector<std::string> tGoldKeys = {"Name", "PerformerName", "Input", "Input", "Output"};
    std::vector<std::string> tGoldValues = {"Filter Gradient", "PlatoMain", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tOutput = tOperation.child("Output");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Filtered Gradient", "Objective Gradient"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);

    auto tInput = tOperation.next_sibling("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);

    tInput = tOperation.next_sibling("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Gradient", "Objective Gradient"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);
}

TEST(PlatoTestXMLGenerator, AppendInitialGuessStage)
{
    pugi::xml_document tDocument;
    XMLGen::append_initial_guess_stage(tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    std::vector<std::string> tGoldKeys = {"Name", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Initial Guess", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOperation = tStage.child("Operation");
    tGoldKeys = {"Name", "PerformerName", "Output"};
    tGoldValues = {"Initialize Field", "PlatoMain", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tOutput = tOperation.child("Output");
    tGoldKeys = {"ArgumentName", "Initialized Field"};
    tGoldValues = {"SharedDataName", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);

    tOutput = tStage.child("Output");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendNondeterministicOperation)
{
    pugi::xml_document tDocument;
    std::vector<std::string> tKeys = {"Name", "PerformerName"};
    auto tPerformerName = std::string("plato analyze") + " {PerformerIndex}";
    std::vector<std::string> tValues = {"Cache State", tPerformerName};
    XMLGen::append_initial_guess_stage(tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOuterFor = tDocument.child("For");
    std::vector<std::string> tGoldKeys = {"var", "in"};
    std::vector<std::string> tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tOuterFor);

    auto tInnerFor = tOuterFor.child("For");
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tInnerFor);

    auto tOperation = tInnerFor.child("Operation");
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
}

TEST(PlatoTestXMLGenerator, AppendCacheStateStageForNondeterministicUsecase_ErrorEmptyObjectiveList)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    ASSERT_THROW(XMLGen::append_cache_state_stage_for_nondeterministic_usecase(tInputData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendCacheStateStageForNondeterministicUsecase)
{
    pugi::xml_document tDocument;
    XMLGen::Objective tObjective;
    tObjective.performer_name = "plato analyze";
    XMLGen::InputData tInputData;
    tInputData.objectives.push_back(tObjective);

    ASSERT_NO_THROW(XMLGen::append_cache_state_stage_for_nondeterministic_usecase(tInputData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    std::vector<std::string> tGoldKeys = {"Name", "For"};
    std::vector<std::string> tGoldValues = {"Cache State : plato analyze 0", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOuterFor = tDocument.child("For");
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tOuterFor);

    auto tInnerFor = tOuterFor.child("For");
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tInnerFor);

    auto tOperation = tInnerFor.child("Operation");
    tGoldKeys = {"Name", "PerformerName"};
    tGoldValues = {"Cache State", "plato analyze {PerformerIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
}

TEST(PlatoTestXMLGenerator, AppendUpdateProblemStageForNondeterministicUsecase_ErrorEmptyObjectiveList)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    ASSERT_THROW(XMLGen::append_update_problem_stage_for_nondeterministic_usecase(tInputData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendUpdateProblemStageForNondeterministicUsecase)
{
    pugi::xml_document tDocument;
    XMLGen::Objective tObjective;
    tObjective.performer_name = "plato analyze";
    XMLGen::InputData tInputData;
    tInputData.objectives.push_back(tObjective);

    ASSERT_NO_THROW(XMLGen::append_update_problem_stage_for_nondeterministic_usecase(tInputData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    std::vector<std::string> tGoldKeys = {"Name", "For"};
    std::vector<std::string> tGoldValues = {"Update Problem : plato analyze 0", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOuterFor = tDocument.child("For");
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tOuterFor);

    auto tInnerFor = tOuterFor.child("For");
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tInnerFor);

    auto tOperation = tInnerFor.child("Operation");
    tGoldKeys = {"Name", "PerformerName"};
    tGoldValues = {"Update Problem", "plato analyze {PerformerIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
}

TEST(PlatoTestXMLGenerator, AppendUpdateProblemStage_ErrorEmptyObjectiveList)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    ASSERT_THROW(XMLGen::append_update_problem_stage(tInputData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendUpdateProblemStage)
{
    pugi::xml_document tDocument;
    XMLGen::Objective tObjective;
    tObjective.performer_name = "plato analyze";
    XMLGen::InputData tInputData;
    tInputData.objectives.push_back(tObjective);

    ASSERT_NO_THROW(XMLGen::append_update_problem_stage(tInputData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    std::vector<std::string> tGoldKeys = {"Name", "Operation"};
    std::vector<std::string> tGoldValues = {"Update Problem : plato analyze 0", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOperation = tStage.child("Operation");
    tGoldKeys = {"Name", "PerformerName"};
    tGoldValues = {"Update Problem", "plato analyze"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
}

TEST(PlatoTestXMLGenerator, AppendLowerBoundStage)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "topology";
    XMLGen::append_lower_bound_stage(tInputData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    std::vector<std::string> tGoldKeys = {"Name", "Input", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Set Lower Bounds", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOuterInput = tStage.child("Input");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Lower Bound Value"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterInput);

    auto tOperation = tStage.child("Operation");
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Calculate Lower Bounds", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    auto tInnerInput = tOperation.child("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Lower Bound Vector", "Lower Bound Vector"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInnerInput);
    auto tInnerOutput = tOperation.child("Output");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Lower Bound Vector", "Lower Bound Vector"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInnerOutput);

    auto tOuterOutput = tStage.child("Output");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Lower Bound Vector"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterOutput);
}

TEST(PlatoTestXMLGenerator, AppendLowerBoundStage_TypeNotEqualToplogy)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "inverse";
    XMLGen::append_lower_bound_stage(tInputData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    std::vector<std::string> tGoldKeys = {"Name", "Output"};
    std::vector<std::string> tGoldValues = {"Set Lower Bounds", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOuterOutput = tStage.child("Output");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Lower Bound Vector"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterOutput);
}

TEST(PlatoTestXMLGenerator, AppendUpperBoundStage)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "topology";
    XMLGen::append_upper_bound_stage(tInputData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    std::vector<std::string> tGoldKeys = {"Name", "Input", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Set Upper Bounds", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOuterInput = tStage.child("Input");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Upper Bound Value"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterInput);

    auto tOperation = tStage.child("Operation");
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Calculate Upper Bounds", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    auto tInnerInput = tOperation.child("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Upper Bound Vector", "Upper Bound Vector"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInnerInput);
    auto tInnerOutput = tOperation.child("Output");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Upper Bound Vector", "Upper Bound Vector"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInnerOutput);

    auto tOuterOutput = tStage.child("Output");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Upper Bound Vector"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterOutput);
}

TEST(PlatoTestXMLGenerator, AppendUpperBoundStage_TypeNotEqualToplogy)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "inverse";
    XMLGen::append_upper_bound_stage(tInputData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    std::vector<std::string> tGoldKeys = {"Name", "Output"};
    std::vector<std::string> tGoldValues = {"Set Upper Bounds", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOuterOutput = tStage.child("Output");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Upper Bound Vector"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterOutput);
}

TEST(PlatoTestXMLGenerator, AppendDesignVolumeStage)
{
    pugi::xml_document tDocument;
    XMLGen::append_design_volume_stage(tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    std::vector<std::string> tGoldKeys = {"Name", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Calculate Design Domain Volume", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOperation = tStage.child("Operation");
    tGoldKeys = {"Name", "PerformerName", "Output"};
    tGoldValues = {"Calculate Design Domain Volume", "PlatoMain", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tInnerOutput = tOperation.child("Output");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Reference Value", "Reference Value"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInnerOutput);

    auto tOuterOutput = tStage.child("Output");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Reference Value"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterOutput);
}

TEST(PlatoTestXMLGenerator, AppendConstraintValueStage)
{
    pugi::xml_document tDocument;
    XMLGen::Constraint tConstraint;
    tConstraint.type = "volume";
    tConstraint.mPerformerName = "PlatoMain";
    XMLGen::InputData tInputData;
    tInputData.constraints.push_back(tConstraint);
    XMLGen::append_constraint_value_stage(tInputData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    std::vector<std::string> tGoldKeys = {"Name", "Type", "Input", "Operation", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Calculate Constraint Value 0", "volume", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOuterInput = tStage.child("Input");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterInput);

    auto tOuterOutput = tStage.child("Output");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Constraint Value"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterOutput);

    auto tFilter = tStage.next_sibling("Operation");
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Filter Control", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilter);
    auto tFilterInput = tFilter.child("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterInput);
    auto tFilterOutput = tFilter.child("Output");
    tGoldValues = {"Filtered Field", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterOutput);

    auto tConstraintValue = tStage.next_sibling("Operation");
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Calculate Constraint Value", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConstraintValue);
    auto tConstraintInput = tConstraintValue.child("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Topology", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConstraintInput);
    auto tConstraintOutput = tConstraintValue.child("Output");
    tGoldValues = {"Constraint Value", "Constraint Value"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConstraintOutput);
}

TEST(PlatoTestXMLGenerator, AppendConstraintGradientStage)
{
    pugi::xml_document tDocument;
    XMLGen::Constraint tConstraint;
    tConstraint.type = "volume";
    tConstraint.mPerformerName = "PlatoMain";
    XMLGen::InputData tInputData;
    tInputData.constraints.push_back(tConstraint);
    XMLGen::append_constraint_gradient_stage(tInputData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    std::vector<std::string> tGoldKeys = {"Name", "Type", "Input", "Operation", "Operation", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Calculate Constraint Gradient 0", "volume", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOuterInput = tStage.child("Input");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterInput);

    auto tOuterOutput = tStage.child("Output");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Constraint Gradient"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterOutput);

    auto tFilter = tStage.next_sibling("Operation");
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Filter Control", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilter);
    auto tFilterInput = tFilter.child("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterInput);
    auto tFilterOutput = tFilter.child("Output");
    tGoldValues = {"Filtered Field", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterOutput);

    auto tConstraintGrad = tStage.next_sibling("Operation");
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Calculate Constraint Gradient", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConstraintGrad);
    auto tConstraintInput = tConstraintGrad.child("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Topology", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConstraintInput);
    auto tConstraintOutput = tConstraintGrad.child("Output");
    tGoldValues = {"Constraint Gradient", "Constraint Gradient"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConstraintOutput);

    tFilter = tStage.next_sibling("Operation");
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Filter Gradient", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilter);
    tFilterInput = tFilter.next_sibling("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterInput);
    tFilterInput = tFilter.next_sibling("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Gradient", "Constraint Gradient"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterInput);
    tFilterOutput = tFilter.child("Output");
    tGoldValues = {"Filtered Gradient", "Constraint Gradient"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterOutput);
}

TEST(PlatoTestXMLGenerator, AppendNondeterministicParameters)
{
    pugi::xml_document tDocument;
    auto tOperation = tDocument.append_child("Operation");
    std::unordered_map<std::string, std::vector<std::string>> tTags =
        { { "0", {"traction load-id-0 x-axis", "traction load-id-0 y-axis", "traction load-id-0 z-axis"} } };
    XMLGen::append_nondeterministic_parameters(tTags, tOperation);

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tGoldKeys = {"Parameter", "Parameter", "Parameter"};
    std::vector<std::string> tGoldValues = {"", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tSibling = tOperation.next_sibling("Parameter");
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction load-id-0 x-axis",
                   "{traction load-id-0 x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldValues = {"traction load-id-0 y-axis",
                   "{traction load-id-0 y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldValues = {"traction load-id-0 z-axis",
                   "{traction load-id-0 z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);
}

TEST(PlatoTestXMLGenerator, AppendSampleObjectiveValueOperation)
{
    // POSE LOAD SET 1
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::Load tLoad1;
    tLoad1.mIsRandom = true;
    tLoad1.type = "traction";
    tLoad1.app_name = "sideset";
    tLoad1.values.push_back("1");
    tLoad1.values.push_back("2");
    tLoad1.values.push_back("3");
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::Load tLoad2;
    tLoad2.mIsRandom = true;
    tLoad2.type = "traction";
    tLoad2.app_name = "sideset";
    tLoad2.values.push_back("4");
    tLoad2.values.push_back("5");
    tLoad2.values.push_back("6");
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::Load tLoad3;
    tLoad3.type = "traction";
    tLoad3.mIsRandom = false;
    tLoad3.app_name = "sideset";
    tLoad3.values.push_back("7");
    tLoad3.values.push_back("8");
    tLoad3.values.push_back("9");
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    // POSE LOAD SET 2
    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::Load tLoad4;
    tLoad4.mIsRandom = true;
    tLoad4.type = "traction";
    tLoad4.app_name = "sideset";
    tLoad4.values.push_back("11");
    tLoad4.values.push_back("12");
    tLoad4.values.push_back("13");
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::Load tLoad5;
    tLoad5.mIsRandom = true;
    tLoad5.type = "traction";
    tLoad5.app_name = "sideset";
    tLoad5.values.push_back("14");
    tLoad5.values.push_back("15");
    tLoad5.values.push_back("16");
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // POSE OBJECTIVE
    XMLGen::Objective tObjective;
    tObjective.performer_name = "plato analyze";

    // DEFINE XML GENERATOR INPUT DATA
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.objectives.push_back(tObjective);
    tXMLMetaData.mRandomMetaData = tRandomMetaData;

    // CALL FUNCTION
    pugi::xml_document tDocument;
    auto tStage = tDocument.append_child("Stage");
    XMLGen::append_sample_objective_value_operation(tXMLMetaData, tStage);
    ASSERT_FALSE(tStage.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tGoldKeys = {"For"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);
    auto tFor = tStage.child("For");
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tOperation = tFor.child("Operation");
    tFor = tOperation.child("For");
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tOperation = tFor.child("Operation");
    tGoldKeys = {"Name", "PerformerName", "Parameter", "Parameter", "Parameter",
        "Parameter", "Parameter", "Parameter", "Input", "Output"};
    tGoldValues = {"Compute Objective Value", "plato analyze {PerformerIndex}",
        "", "", "", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    // TEST PARAMETERS AGAINST GOLD VALUES
    auto tSibling = tOperation.next_sibling("Parameter");
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction load-id-0 x-axis",
                   "{traction load-id-0 x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldValues = {"traction load-id-0 y-axis",
                   "{traction load-id-0 y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldValues = {"traction load-id-0 z-axis",
                   "{traction load-id-0 z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction load-id-1 x-axis",
                   "{traction load-id-1 x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldValues = {"traction load-id-1 y-axis",
                   "{traction load-id-1 y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldValues = {"traction load-id-1 z-axis",
                   "{traction load-id-1 z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    // TEST OPERATION INPUTS AND OUTPUTS AGAINST GOLD VALUES
    tSibling = tOperation.next_sibling("Input");
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Topology", "Topology" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Output");
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Objective Value", "Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);
}

TEST(PlatoTestXMLGenerator, AppendEvaluateNondeterministicCriterionValueOperation)
{
    // CALL FUNCTION
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.objective_number_standard_deviations = "1";
    pugi::xml_document tDocument;
    auto tStage = tDocument.append_child("Stage");
    XMLGen::append_evaluate_nondeterministic_criterion_value_operation("Objective", tXMLMetaData, tStage);
    ASSERT_FALSE(tStage.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tGoldKeys = {"Operation"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);
    auto tOperation = tStage.child("Operation");
    tGoldKeys = {"Name", "PerformerName", "For", "Output"};
    tGoldValues = {"Calculate Non-Deterministic Objective Value", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tFor = tOperation.child("For");
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tFor = tFor.child("For");
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tInput = tFor.child("Input");
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tInput);

    auto tOutput = tOperation.child("Output");
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Value", "Objective Mean Plus 1 StdDev"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendObjectiveValueStageForNondeterministicUsecase)
{
    // POSE LOAD SET 1
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::Load tLoad1;
    tLoad1.mIsRandom = true;
    tLoad1.type = "traction";
    tLoad1.app_name = "sideset";
    tLoad1.values.push_back("1");
    tLoad1.values.push_back("2");
    tLoad1.values.push_back("3");
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::Load tLoad2;
    tLoad2.mIsRandom = true;
    tLoad2.type = "traction";
    tLoad2.app_name = "sideset";
    tLoad2.values.push_back("4");
    tLoad2.values.push_back("5");
    tLoad2.values.push_back("6");
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::Load tLoad3;
    tLoad3.type = "traction";
    tLoad3.mIsRandom = false;
    tLoad3.app_name = "sideset";
    tLoad3.values.push_back("7");
    tLoad3.values.push_back("8");
    tLoad3.values.push_back("9");
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    // POSE LOAD SET 2
    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::Load tLoad4;
    tLoad4.mIsRandom = true;
    tLoad4.type = "traction";
    tLoad4.app_name = "sideset";
    tLoad4.values.push_back("11");
    tLoad4.values.push_back("12");
    tLoad4.values.push_back("13");
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::Load tLoad5;
    tLoad5.mIsRandom = true;
    tLoad5.type = "traction";
    tLoad5.app_name = "sideset";
    tLoad5.values.push_back("14");
    tLoad5.values.push_back("15");
    tLoad5.values.push_back("16");
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // POSE OBJECTIVE
    XMLGen::Objective tObjective;
    tObjective.performer_name = "plato analyze";

    // DEFINE XML GENERATOR INPUT DATA
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.objectives.push_back(tObjective);
    tXMLMetaData.mRandomMetaData = tRandomMetaData;

    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::append_objective_value_stage_for_nondeterministic_usecase(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // ****** 1) TEST RESULTS AGAINST STAGE GOLD VALUES ******
    auto tStage = tDocument.child("Stage");
    std::vector<std::string> tGoldKeys = {"Name", "Input", "Operation", "For", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Calculate Objective Value", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tStageInput= tStage.child("Input");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageInput);

    auto tStageOutput= tStage.child("Output");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Objective Value"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageOutput);

    // ****** 2) TEST RESULTS AGAINST FILTER OPERATION GOLD VALUES ******
    auto tFilterOperation = tStage.next_sibling("Operation");
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Filter Control", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterOperation);

    auto tFilterInput = tFilterOperation.child("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterInput);

    auto tFilterOutput = tFilterOperation.child("Output");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Filtered Field", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterOutput);

    // ****** 3) TEST RESULTS AGAINST EVALUATE RANDOM OBJECTIVE OPERATION GOLD VALUES ******
    auto tRandomObjectiveOperation = tStage.next_sibling("Operation");
    tGoldKeys = {"Name", "PerformerName", "For", "Output"};
    tGoldValues = {"Calculate Non-Deterministic Objective Value", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tRandomObjectiveOperation);

    auto tFor = tRandomObjectiveOperation.child("For");
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tFor = tFor.child("For");
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tRandomObjectiveInput = tRandomObjectiveOperation.child("Input");
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tRandomObjectiveInput);

    auto tRandomObjectiveOutput = tRandomObjectiveOperation.child("Output");
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Value", "Objective Mean Plus 1 StdDev"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tRandomObjectiveOutput);

    // ****** 4) TEST RESULTS AGAINST SAMPLE OBJECTIVE GOLD VALUES ******
    tFor = tStage.next_sibling("For");
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tOperation = tFor.child("Operation");
    tFor = tOperation.child("For");
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tOperation = tFor.child("Operation");
    tGoldKeys = {"Name", "PerformerName", "Parameter", "Parameter", "Parameter",
        "Parameter", "Parameter", "Parameter", "Input", "Output"};
    tGoldValues = {"Compute Objective Value", "plato analyze {PerformerIndex}",
        "", "", "", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    // ****** 4.1) TEST PARAMETERS AGAINST GOLD VALUES ******
    auto tSibling = tOperation.next_sibling("Parameter");
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction load-id-0 x-axis",
                   "{traction load-id-0 x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldValues = {"traction load-id-0 y-axis",
                   "{traction load-id-0 y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldValues = {"traction load-id-0 z-axis",
                   "{traction load-id-0 z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction load-id-1 x-axis",
                   "{traction load-id-1 x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldValues = {"traction load-id-1 y-axis",
                   "{traction load-id-1 y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldValues = {"traction load-id-1 z-axis",
                   "{traction load-id-1 z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    // ****** 4.2) TEST SAMPLE OBJECTIVE OPERATION INPUTS AND OUTPUTS AGAINST GOLD VALUES ******
    tSibling = tOperation.next_sibling("Input");
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Topology", "Topology" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Output");
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Objective Value", "Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);
}

TEST(PlatoTestXMLGenerator, AppendSampleObjectiveGradientOperation)
{
    // POSE LOAD SET 1
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::Load tLoad1;
    tLoad1.mIsRandom = true;
    tLoad1.type = "traction";
    tLoad1.app_name = "sideset";
    tLoad1.values.push_back("1");
    tLoad1.values.push_back("2");
    tLoad1.values.push_back("3");
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::Load tLoad2;
    tLoad2.mIsRandom = true;
    tLoad2.type = "traction";
    tLoad2.app_name = "sideset";
    tLoad2.values.push_back("4");
    tLoad2.values.push_back("5");
    tLoad2.values.push_back("6");
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::Load tLoad3;
    tLoad3.type = "traction";
    tLoad3.mIsRandom = false;
    tLoad3.app_name = "sideset";
    tLoad3.values.push_back("7");
    tLoad3.values.push_back("8");
    tLoad3.values.push_back("9");
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    // POSE LOAD SET 2
    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::Load tLoad4;
    tLoad4.mIsRandom = true;
    tLoad4.type = "traction";
    tLoad4.app_name = "sideset";
    tLoad4.values.push_back("11");
    tLoad4.values.push_back("12");
    tLoad4.values.push_back("13");
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::Load tLoad5;
    tLoad5.mIsRandom = true;
    tLoad5.type = "traction";
    tLoad5.app_name = "sideset";
    tLoad5.values.push_back("14");
    tLoad5.values.push_back("15");
    tLoad5.values.push_back("16");
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // POSE OBJECTIVE
    XMLGen::Objective tObjective;
    tObjective.performer_name = "plato analyze";

    // DEFINE XML GENERATOR INPUT DATA
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.objectives.push_back(tObjective);
    tXMLMetaData.mRandomMetaData = tRandomMetaData;

    // CALL FUNCTION
    pugi::xml_document tDocument;
    auto tStage = tDocument.append_child("Stage");
    XMLGen::append_sample_objective_gradient_operation(tXMLMetaData, tStage);
    ASSERT_FALSE(tStage.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tGoldKeys = {"For"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);
    auto tFor = tStage.child("For");
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tFor = tFor.child("Operation").child("For");
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tOperation = tFor.child("Operation");
    tGoldKeys = {"Name", "PerformerName", "Parameter", "Parameter", "Parameter",
        "Parameter", "Parameter", "Parameter", "Input", "Output"};
    tGoldValues = {"Compute Objective Gradient", "plato analyze {PerformerIndex}",
        "", "", "", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    // TEST PARAMETERS AGAINST GOLD VALUES
    auto tSibling = tOperation.next_sibling("Parameter");
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction load-id-0 x-axis",
                   "{traction load-id-0 x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldValues = {"traction load-id-0 y-axis",
                   "{traction load-id-0 y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldValues = {"traction load-id-0 z-axis",
                   "{traction load-id-0 z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction load-id-1 x-axis",
                   "{traction load-id-1 x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldValues = {"traction load-id-1 y-axis",
                   "{traction load-id-1 y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldValues = {"traction load-id-1 z-axis",
                   "{traction load-id-1 z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    // TEST OPERATION INPUTS AND OUTPUTS AGAINST GOLD VALUES
    tSibling = tOperation.next_sibling("Input");
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Topology", "Topology" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Output");
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Objective Gradient", "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);
}

TEST(PlatoTestXMLGenerator, AppendEvaluateNondeterministicCriterionGradientOperation)
{
    // CALL FUNCTION
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.objective_number_standard_deviations = "1";
    pugi::xml_document tDocument;
    auto tStage = tDocument.append_child("Stage");
    XMLGen::append_evaluate_nondeterministic_criterion_gradient_operation("Objective", tXMLMetaData, tStage);
    ASSERT_FALSE(tStage.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tGoldKeys = {"Operation"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);
    auto tOperation = tStage.child("Operation");
    tGoldKeys = {"Name", "PerformerName", "For", "Output"};
    tGoldValues = {"Calculate Non-Deterministic Objective Gradient", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tFor = tOperation.child("For");
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tFor = tFor.child("For");
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tInput = tFor.next_sibling("Input");
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tInput);
    tInput = tFor.next_sibling("Input");
    tGoldValues = {"Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tInput);

    auto tOutput = tOperation.child("Output");
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Gradient", "Objective Mean Plus 1 StdDev Gradient"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendObjectiveGradientStageForNondeterministicUsecase)
{
    // POSE LOAD SET 1
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::Load tLoad1;
    tLoad1.mIsRandom = true;
    tLoad1.type = "traction";
    tLoad1.app_name = "sideset";
    tLoad1.values.push_back("1");
    tLoad1.values.push_back("2");
    tLoad1.values.push_back("3");
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::Load tLoad2;
    tLoad2.mIsRandom = true;
    tLoad2.type = "traction";
    tLoad2.app_name = "sideset";
    tLoad2.values.push_back("4");
    tLoad2.values.push_back("5");
    tLoad2.values.push_back("6");
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::Load tLoad3;
    tLoad3.type = "traction";
    tLoad3.mIsRandom = false;
    tLoad3.app_name = "sideset";
    tLoad3.values.push_back("7");
    tLoad3.values.push_back("8");
    tLoad3.values.push_back("9");
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    // POSE LOAD SET 2
    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::Load tLoad4;
    tLoad4.mIsRandom = true;
    tLoad4.type = "traction";
    tLoad4.app_name = "sideset";
    tLoad4.values.push_back("11");
    tLoad4.values.push_back("12");
    tLoad4.values.push_back("13");
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::Load tLoad5;
    tLoad5.mIsRandom = true;
    tLoad5.type = "traction";
    tLoad5.app_name = "sideset";
    tLoad5.values.push_back("14");
    tLoad5.values.push_back("15");
    tLoad5.values.push_back("16");
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // POSE OBJECTIVE
    XMLGen::Objective tObjective;
    tObjective.performer_name = "plato analyze";

    // DEFINE XML GENERATOR INPUT DATA
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.objectives.push_back(tObjective);
    tXMLMetaData.mRandomMetaData = tRandomMetaData;

    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::append_objective_gradient_stage_for_nondeterministic_usecase(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // ****** 1) TEST RESULTS AGAINST STAGE GOLD VALUES ******
    auto tStage = tDocument.child("Stage");
    std::vector<std::string> tGoldKeys = {"Name", "Input", "Operation", "For", "For", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Calculate Objective Gradient", "", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tStageInput= tStage.child("Input");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageInput);

    auto tStageOutput= tStage.child("Output");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Objective Gradient"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageOutput);

    // ****** 2) TEST RESULTS AGAINST FILTER OPERATION GOLD VALUES ******
    auto tFilterOperation = tStage.next_sibling("Operation");
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Filter Control", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterOperation);

    auto tFilterInput = tFilterOperation.child("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterInput);

    auto tFilterOutput = tFilterOperation.child("Output");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Filtered Field", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterOutput);

    // ****** 3) TEST RESULTS AGAINST EVALUATE RANDOM OBJECTIVE GRADIENT OPERATION GOLD VALUES ******
    auto tRandomObjGradOperation = tStage.next_sibling("Operation");
    tGoldKeys = {"Name", "PerformerName", "For", "Output"};
    tGoldValues = {"Calculate Non-Deterministic Objective Gradient", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tRandomObjGradOperation);

    auto tFor = tRandomObjGradOperation.child("For");
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tFor = tFor.child("For");
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tRandomObjectiveInput = tRandomObjGradOperation.child("Input");
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tRandomObjectiveInput);

    auto tRandomObjectiveOutput = tRandomObjGradOperation.child("Output");
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Gradient", "Objective Mean Plus 1 StdDev Gradient"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tRandomObjectiveOutput);

    // ****** 4) TEST RESULTS AGAINST SAMPLE OBJECTIVE GRADIENT GOLD VALUES ******
    tFor = tStage.next_sibling("For");
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tFor = tFor.child("Operation").child("For");
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tOperation = tFor.child("Operation");
    tGoldKeys = {"Name", "PerformerName", "Parameter", "Parameter", "Parameter",
        "Parameter", "Parameter", "Parameter", "Input", "Output"};
    tGoldValues = {"Compute Objective Gradient", "plato analyze {PerformerIndex}",
        "", "", "", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    // ****** 4.1) TEST PARAMETERS AGAINST GOLD VALUES ******
    auto tSibling = tOperation.next_sibling("Parameter");
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction load-id-0 x-axis",
                   "{traction load-id-0 x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldValues = {"traction load-id-0 y-axis",
                   "{traction load-id-0 y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldValues = {"traction load-id-0 z-axis",
                   "{traction load-id-0 z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction load-id-1 x-axis",
                   "{traction load-id-1 x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldValues = {"traction load-id-1 y-axis",
                   "{traction load-id-1 y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Parameter");
    tGoldValues = {"traction load-id-1 z-axis",
                   "{traction load-id-1 z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    // ****** 4.2) TEST OPERATION INPUTS AND OUTPUTS AGAINST GOLD VALUES ******
    tSibling = tOperation.next_sibling("Input");
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Topology", "Topology" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    tSibling = tOperation.next_sibling("Output");
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Objective Gradient", "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSibling);

    // ****** 5) TEST RESULTS AGAINST FILTER GRADIENT OPERATION GOLD VALUES ******
    auto tOuterFor = tStage.next_sibling("For");
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tOuterFor);

    auto tInnerFor = tOuterFor.child("Operation").child("For");
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tInnerFor);

    auto tFilterGradOperation = tInnerFor.child("Operation");
    tGoldKeys = {"Name", "PerformerName", "Input", "Input", "Output"};
    tGoldValues = {"Filter Gradient", "PlatoMain", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterGradOperation);

    auto tFilterGradOutput = tFilterGradOperation.child("Output");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Filtered Gradient", "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterGradOutput);

    auto tFilterGradInput = tFilterGradOperation.next_sibling("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterGradInput);

    tFilterGradInput = tFilterGradOperation.next_sibling("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Gradient", "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterGradInput);
}

TEST(PlatoTestXMLGenerator, AppendDerivativeCheckerOptions)
{
    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.check_gradient = "true";
    tXMLMetaData.check_hessian = "true";
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    XMLGen::append_derivative_checker_parameters_options(tXMLMetaData, tOptimizerNode);
    ASSERT_FALSE(tOptimizerNode.empty());

    // ****** TEST RESULTS AGAINST OPTIMIZER NODE GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"CheckGradient", "CheckHessian", "UseUserInitialGuess", "Options"};
    std::vector<std::string> tGoldValues = {"true", "true", "True", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tOptionsNode = tOptimizerNode.child("Options");
    tGoldKeys = {"DerivativeCheckerInitialSuperscript", "DerivativeCheckerFinalSuperscript"};
    tGoldValues = {"1", "8"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptionsNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationAlgorithmOC_Options)
{
    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.max_iterations = "11";
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    XMLGen::append_optimization_algorithm_oc_parameters_options(tXMLMetaData, tOptimizerNode);
    ASSERT_FALSE(tOptimizerNode.empty());

    // ****** TEST RESULTS AGAINST OPTIMIZER NODE GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"Convergence"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tConvergenceNode = tOptimizerNode.child("Convergence");
    tGoldKeys = {"MaxIterations"};
    tGoldValues = {"11"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConvergenceNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationAlgorithmMMA_Options)
{
    // 1) TEST CASE WHERE ONLY A FEW PARAMETERS ARE DEFINED
    pugi::xml_document tDocument1;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.max_iterations = "11";
    tXMLMetaData.mMMAMoveLimit = "0.2";
    auto tOptimizerNode = tDocument1.append_child("Optimizer");
    XMLGen::append_optimization_algorithm_mma_parameters_options(tXMLMetaData, tOptimizerNode);
    ASSERT_FALSE(tOptimizerNode.empty());

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"Options"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tOptionsNode = tOptimizerNode.child("Options");
    tGoldKeys = {"MaxNumOuterIterations", "MoveLimit"};
    tGoldValues = {"11", "0.2"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptionsNode);

    // 2) TEST CASE WHERE ALL THE PARAMETERS ARE DEFINED
    pugi::xml_document tDocument2;
    tXMLMetaData.mMMAAsymptoteExpansion = "2";
    tXMLMetaData.mMMAAsymptoteContraction = "0.75";
    tXMLMetaData.mMMAMaxNumSubProblemIterations = "50";
    tXMLMetaData.mMMAControlStagnationTolerance = "1e-3";
    tXMLMetaData.mMMAObjectiveStagnationTolerance = "1e-8";
    tOptimizerNode = tDocument2.append_child("Optimizer");
    XMLGen::append_optimization_algorithm_mma_parameters_options(tXMLMetaData, tOptimizerNode);
    ASSERT_FALSE(tOptimizerNode.empty());

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    tGoldKeys = {"Options"};
    tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    tOptionsNode = tOptimizerNode.child("Options");
    tGoldKeys = {"MaxNumOuterIterations", "MoveLimit", "AsymptoteExpansion", "AsymptoteContraction",
        "MaxNumSubProblemIter", "ControlStagnationTolerance", "ObjectiveStagnationTolerance"};
    tGoldValues = {"11", "0.2", "2", "0.75", "50", "1e-3", "1e-8"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptionsNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationAlgorithmOptions_ErrorOptimizerNotSupported)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.optimization_algorithm = "stochastic gradient descent";
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    ASSERT_THROW(XMLGen::append_optimization_algorithm_parameters_options(tXMLMetaData, tOptimizerNode), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationAlgorithmOptionsMMA)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.max_iterations = "11";
    tXMLMetaData.mMMAMoveLimit = "0.2";
    tXMLMetaData.optimization_algorithm = "MmA";
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    ASSERT_NO_THROW(XMLGen::append_optimization_algorithm_parameters_options(tXMLMetaData, tOptimizerNode));
    ASSERT_FALSE(tOptimizerNode.empty());

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"Options"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tOptionsNode = tOptimizerNode.child("Options");
    tGoldKeys = {"MaxNumOuterIterations", "MoveLimit"};
    tGoldValues = {"11", "0.2"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptionsNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationAlgorithmOptionsOC)
{
    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.max_iterations = "11";
    tXMLMetaData.optimization_algorithm = "Oc";
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    ASSERT_NO_THROW(XMLGen::append_optimization_algorithm_parameters_options(tXMLMetaData, tOptimizerNode));
    ASSERT_FALSE(tOptimizerNode.empty());

    // ****** TEST RESULTS AGAINST OPTIMIZER NODE GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"Convergence"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tCovergenceNode = tOptimizerNode.child("Convergence");
    tGoldKeys = {"MaxIterations"};
    tGoldValues = {"11"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tCovergenceNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationAlgorithmOption_ErrorOptimizerNotSupported)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.optimization_algorithm = "stochastic gradient descent";
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    ASSERT_THROW(XMLGen::append_optimization_algorithm_options(tXMLMetaData, tOptimizerNode), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationAlgorithmOption_DerivativeChecker)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.check_hessian = "false";
    tXMLMetaData.check_gradient = "true";
    tXMLMetaData.optimization_algorithm = "derivativechecker";
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    ASSERT_NO_THROW(XMLGen::append_optimization_algorithm_options(tXMLMetaData, tOptimizerNode));

    // ****** TEST RESULTS AGAINST OPTIMIZER NODE GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"Package", "CheckGradient", "CheckHessian", "UseUserInitialGuess", "Options"};
    std::vector<std::string> tGoldValues = {"DerivativeChecker", "true", "false", "True", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tOptionsNode = tOptimizerNode.child("Options");
    tGoldKeys = {"DerivativeCheckerInitialSuperscript", "DerivativeCheckerFinalSuperscript"};
    tGoldValues = {"1", "8"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptionsNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationAlgorithmOption)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.max_iterations = "12";
    tXMLMetaData.optimization_algorithm = "mma";
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    ASSERT_NO_THROW(XMLGen::append_optimization_algorithm_options(tXMLMetaData, tOptimizerNode));

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"Package", "Options"};
    std::vector<std::string> tGoldValues = {"MMA", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tOptionsNode = tOptimizerNode.child("Options");
    tGoldKeys = {"MaxNumOuterIterations"};
    tGoldValues = {"12"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptionsNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationOutputOptions)
{
    pugi::xml_document tDocument;
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    XMLGen::append_optimization_output_options(tOptimizerNode);

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"Output"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tOutputNode = tOptimizerNode.child("Output");
    tGoldKeys = {"OutputStage"};
    tGoldValues = {"Output To File"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutputNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationCacheStateOptions)
{
    pugi::xml_document tDocument;
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    XMLGen::append_optimization_cache_stage_options(tOptimizerNode);

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"CacheStage"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tOutputNode = tOptimizerNode.child("CacheStage");
    tGoldKeys = {"Name"};
    tGoldValues = {"Cache State"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutputNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationUpdateProblemOptions)
{
    pugi::xml_document tDocument;
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    XMLGen::append_optimization_update_problem_stage_options(tOptimizerNode);

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"UpdateProblemStage"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tOutputNode = tOptimizerNode.child("UpdateProblemStage");
    tGoldKeys = {"Name"};
    tGoldValues = {"Update Problem"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutputNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationVariablesOptions)
{
    pugi::xml_document tDocument;
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    XMLGen::append_optimization_variables_options(tOptimizerNode);

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"OptimizationVariables"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tNode = tOptimizerNode.child("OptimizationVariables");
    tGoldKeys = {"ValueName", "InitializationStage", "FilteredName", "LowerBoundValueName", "LowerBoundVectorName",
        "UpperBoundValueName", "UpperBoundVectorName", "SetLowerBoundsStage", "SetUpperBoundsStage"};
    tGoldValues = {"Control", "Initial Guess", "Topology", "Lower Bound Value", "Lower Bound Vector",
        "Upper Bound Value", "Upper Bound Vector", "Set Lower Bounds", "Set Upper Bounds"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationObjectiveOptions)
{
    pugi::xml_document tDocument;
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    XMLGen::append_optimization_objective_options(tOptimizerNode);

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"Objective"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tObjectiveNode = tOptimizerNode.child("Objective");
    tGoldKeys = {"ValueName", "ValueStageName", "GradientName", "GradientStageName"};
    tGoldValues = {"Objective Value", "Calculate Objective Value", "Objective Gradient",
        "Calculate Objective Gradient"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tObjectiveNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationConstraintOptions)
{
    pugi::xml_document tDocument;
    XMLGen::Constraint tConstraint;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.constraints.push_back(tConstraint);
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    XMLGen::append_optimization_constraint_options(tXMLMetaData, tOptimizerNode);

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    // CASE 1: TARGET VALUES, ABSOLUTE OR NORMALIZED, ARE NOT DEFINED
    std::vector<std::string> tGoldKeys = {"Constraint"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tConstraintNode = tOptimizerNode.child("Constraint");
    tGoldKeys = {"ValueName", "ValueStageName", "GradientName", "GradientStageName", "ReferenceValueName"};
    tGoldValues = {"Constraint Value", "Calculate Constraint Value", "Constraint Gradient",
        "Calculate Constraint Gradient", "Reference Value"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConstraintNode);

    // CASE 2: NORMALIZED TARGET VALUE IS DEFINED
    tOptimizerNode = tDocument.append_child("Optimizer");
    tXMLMetaData.constraints[0].mNormalizedTargetValue = "1.0";
    XMLGen::append_optimization_constraint_options(tXMLMetaData, tOptimizerNode);

    tGoldKeys = {"Constraint"};
    tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    tConstraintNode = tOptimizerNode.child("Constraint");
    tGoldKeys = {"ValueName", "ValueStageName", "GradientName", "GradientStageName",
        "ReferenceValueName", "NormalizedTargetValue"};
    tGoldValues = {"Constraint Value", "Calculate Constraint Value", "Constraint Gradient",
        "Calculate Constraint Gradient", "Reference Value", "1.0"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConstraintNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationBoundConstraintsOptions)
{
    pugi::xml_document tDocument;
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    XMLGen::append_optimization_bound_constraints_options({"1.0", "0.0"}, tOptimizerNode);

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"BoundConstraint"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tBoundConstraintNode = tOptimizerNode.child("BoundConstraint");
    tGoldKeys = {"Upper", "Lower"};
    tGoldValues = {"1.0", "0.0"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tBoundConstraintNode);
}

TEST(PlatoTestXMLGenerator, WriteInterfaceXmlFile_ErrorEmptyObjective)
{
    XMLGen::InputData tXMLMetaData;
    ASSERT_THROW(XMLGen::write_interface_xml_file_for_nondeterministic_usecase(tXMLMetaData), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, WriteInterfaceXmlFile_ErrorMultipleObjectives)
{
    XMLGen::InputData tXMLMetaData;
    XMLGen::Objective tObjective1;
    XMLGen::Objective tObjective2;
    tXMLMetaData.objectives.push_back(tObjective1);
    tXMLMetaData.objectives.push_back(tObjective2);
    ASSERT_THROW(XMLGen::write_interface_xml_file_for_nondeterministic_usecase(tXMLMetaData), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, WriteInterfaceXmlFile)
{
    // POSE MATERIAL SET 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("2");
    tMaterial1.category("isotropic");
    tMaterial1.property("elastic modulus", "1");
    tMaterial1.property("poissons ratio", "0.3");
    XMLGen::Material tMaterial2;
    tMaterial2.id("2");
    tMaterial2.category("isotropic");
    tMaterial2.property("elastic modulus", "1");
    tMaterial2.property("poissons ratio", "0.3");

    XMLGen::MaterialSet tMaterialSetOne;
    tMaterialSetOne.insert({"1", tMaterial1});
    tMaterialSetOne.insert({"2", tMaterial2});
    auto tRandomMaterialCase1 = std::make_pair(0.5, tMaterialSetOne);

    // POSE MATERIAL SET 2
    XMLGen::Material tMaterial3;
    tMaterial3.id("2");
    tMaterial3.category("isotropic");
    tMaterial3.property("elastic modulus", "1.1");
    tMaterial3.property("poissons ratio", "0.33");
    XMLGen::Material tMaterial4;
    tMaterial4.id("2");
    tMaterial4.category("isotropic");
    tMaterial4.property("elastic modulus", "1");
    tMaterial4.property("poissons ratio", "0.3");

    XMLGen::MaterialSet tMaterialSetTwo;
    tMaterialSetTwo.insert({"1", tMaterial3});
    tMaterialSetTwo.insert({"2", tMaterial4});
    auto tRandomMaterialCase2 = std::make_pair(0.5, tMaterialSetTwo);

    // POSE LOAD SET 1
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::Load tLoad1;
    tLoad1.mIsRandom = true;
    tLoad1.type = "traction";
    tLoad1.app_name = "sideset";
    tLoad1.values.push_back("1");
    tLoad1.values.push_back("2");
    tLoad1.values.push_back("3");
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::Load tLoad2;
    tLoad2.mIsRandom = true;
    tLoad2.type = "traction";
    tLoad2.app_name = "sideset";
    tLoad2.values.push_back("4");
    tLoad2.values.push_back("5");
    tLoad2.values.push_back("6");
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::Load tLoad3;
    tLoad3.type = "traction";
    tLoad3.mIsRandom = false;
    tLoad3.app_name = "sideset";
    tLoad3.values.push_back("7");
    tLoad3.values.push_back("8");
    tLoad3.values.push_back("9");
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    // POSE LOAD SET 2
    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::Load tLoad4;
    tLoad4.mIsRandom = true;
    tLoad4.type = "traction";
    tLoad4.app_name = "sideset";
    tLoad4.values.push_back("11");
    tLoad4.values.push_back("12");
    tLoad4.values.push_back("13");
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::Load tLoad5;
    tLoad5.mIsRandom = true;
    tLoad5.type = "traction";
    tLoad5.app_name = "sideset";
    tLoad5.values.push_back("14");
    tLoad5.values.push_back("15");
    tLoad5.values.push_back("16");
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.append(tRandomMaterialCase1));
    ASSERT_NO_THROW(tRandomMetaData.append(tRandomMaterialCase2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // DEFINE CONSTRAINT
    XMLGen::Constraint tConstraint;
    tConstraint.mNormalizedTargetValue = "1.0";
    tConstraint.type = "volume";

    // DEFINE OBJECTIVE
    XMLGen::Objective tObjective;
    tObjective.type = "total work";
    tObjective.code_name = "analyze";
    tObjective.performer_name = "plato analyze";

    // DEFINE XML GENERATOR INPUT DATA
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.constraints.push_back(tConstraint);
    tXMLMetaData.objectives.push_back(tObjective);
    tXMLMetaData.optimization_type = "topology";
    tXMLMetaData.max_iterations = "100";
    tXMLMetaData.optimization_algorithm = "mma";
    tXMLMetaData.objective_number_standard_deviations = "1";
    tXMLMetaData.mRandomMetaData = tRandomMetaData;

    // CALL FUNCTION
    XMLGen::write_interface_xml_file_for_nondeterministic_usecase(tXMLMetaData);
}

TEST(PlatoTestXMLGenerator, WriteDefineXmlFile_Materials)
{
    // POSE MATERIAL SET 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("2");
    tMaterial1.category("isotropic");
    tMaterial1.property("elastic modulus", "1");
    tMaterial1.property("poissons ratio", "0.3");
    XMLGen::Material tMaterial2;
    tMaterial2.id("2");
    tMaterial2.category("isotropic");
    tMaterial2.property("elastic modulus", "1");
    tMaterial2.property("poissons ratio", "0.3");

    XMLGen::MaterialSet tMaterialSetOne;
    tMaterialSetOne.insert({"1", tMaterial1});
    tMaterialSetOne.insert({"2", tMaterial2});
    auto tRandomMaterialCaseOne = std::make_pair(0.5, tMaterialSetOne);

    // POSE MATERIAL SET 2
    XMLGen::Material tMaterial3;
    tMaterial3.id("2");
    tMaterial3.category("isotropic");
    tMaterial3.property("elastic modulus", "1.1");
    tMaterial3.property("poissons ratio", "0.33");
    XMLGen::Material tMaterial4;
    tMaterial4.id("2");
    tMaterial4.category("isotropic");
    tMaterial4.property("elastic modulus", "1");
    tMaterial4.property("poissons ratio", "0.3");

    XMLGen::MaterialSet tMaterialSetTwo;
    tMaterialSetTwo.insert({"1", tMaterial3});
    tMaterialSetTwo.insert({"2", tMaterial4});
    auto tRandomMaterialCaseTwo = std::make_pair(0.5, tMaterialSetTwo);

    // CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tRandomMaterialCaseOne));
    ASSERT_NO_THROW(tRandomMetaData.append(tRandomMaterialCaseTwo));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // SET NUM PERFORMERS
    XMLGen::UncertaintyMetaData tUncertaintyMetaData;
    tUncertaintyMetaData.numPeformers = 2;

    // CALL FUNCTION
    ASSERT_NO_THROW(XMLGen::write_define_xml_file(tRandomMetaData, tUncertaintyMetaData));

    // TEST OUTPUT FILE
    auto tReadData = XMLGen::read_data_from_file("defines.xml");

    auto tGold = std::string("<?xmlversion=\"1.0\"?><Definename=\"NumSamples\"type=\"int\"value=\"2\"/>")
        + "<Definename=\"NumPerformers\"type=\"int\"value=\"2\"/>"
        + "<Definename=\"NumSamplesPerPerformer\"type=\"int\"value=\"{NumSamples/NumPerformers}\"/>"
        + "<Definename=\"Samples\"type=\"int\"from=\"0\"to=\"{NumSamples-1}\"/>"
        + "<Definename=\"Performers\"type=\"int\"from=\"0\"to=\"{NumPerformers-1}\"/>"
        + "<Definename=\"PerformerSamples\"type=\"int\"from=\"0\"to=\"{NumSamplesPerPerformer-1}\"/>"
        + "<Arrayname=\"poissonsratioblock-id-1\"type=\"real\"value=\"0.3,0.33\"/>"
        + "<Arrayname=\"elasticmodulusblock-id-1\"type=\"real\"value=\"1,1.1\"/>"
        + "<Arrayname=\"poissonsratioblock-id-2\"type=\"real\"value=\"0.3,0.3\"/>"
        + "<Arrayname=\"elasticmodulusblock-id-2\"type=\"real\"value=\"1,1\"/>"
        + "<Arrayname=\"Probabilities\"type=\"real\"value=\"5.000000000000000000000e-01,5.000000000000000000000e-01\"/>";

    ASSERT_STREQ(tGold.c_str(), tReadData.str().c_str());

    std::system("rm -f defines.xml");
}

TEST(PlatoTestXMLGenerator, WriteDefineXmlFile_Loads)
{
    // 1.1 APPEND LOADS
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::Load tLoad1;
    tLoad1.mIsRandom = true;
    tLoad1.type = "traction";
    tLoad1.app_name = "sideset";
    tLoad1.values.push_back("1");
    tLoad1.values.push_back("2");
    tLoad1.values.push_back("3");
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::Load tLoad2;
    tLoad2.mIsRandom = true;
    tLoad2.type = "traction";
    tLoad2.app_name = "sideset";
    tLoad2.values.push_back("4");
    tLoad2.values.push_back("5");
    tLoad2.values.push_back("6");
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::Load tLoad3;
    tLoad3.mIsRandom = false;
    tLoad3.type = "traction";
    tLoad3.app_name = "sideset";
    tLoad3.values.push_back("7");
    tLoad3.values.push_back("8");
    tLoad3.values.push_back("9");
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::Load tLoad4;
    tLoad4.mIsRandom = true;
    tLoad4.type = "traction";
    tLoad4.app_name = "sideset";
    tLoad4.values.push_back("11");
    tLoad4.values.push_back("12");
    tLoad4.values.push_back("13");
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::Load tLoad5;
    tLoad5.mIsRandom = true;
    tLoad5.type = "traction";
    tLoad5.app_name = "sideset";
    tLoad5.values.push_back("14");
    tLoad5.values.push_back("15");
    tLoad5.values.push_back("16");
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // 1.2 CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // 2. SET NUM PERFORMERS
    XMLGen::UncertaintyMetaData tUncertaintyMetaData;
    tUncertaintyMetaData.numPeformers = 2;

    // 3. CALL FUNCTION
    ASSERT_NO_THROW(XMLGen::write_define_xml_file(tRandomMetaData, tUncertaintyMetaData));

    // 4. TEST OUTPUT FILE
    auto tReadData = XMLGen::read_data_from_file("defines.xml");

    auto tGold = std::string("<?xmlversion=\"1.0\"?><Definename=\"NumSamples\"type=\"int\"value=\"2\"/>")
            + "<Definename=\"NumPerformers\"type=\"int\"value=\"2\"/>"
            + "<Definename=\"NumSamplesPerPerformer\"type=\"int\"value=\"{NumSamples/NumPerformers}\"/>"
            + "<Definename=\"Samples\"type=\"int\"from=\"0\"to=\"{NumSamples-1}\"/>"
            + "<Definename=\"Performers\"type=\"int\"from=\"0\"to=\"{NumPerformers-1}\"/>"
            + "<Definename=\"PerformerSamples\"type=\"int\"from=\"0\"to=\"{NumSamplesPerPerformer-1}\"/>"
            + "<Arrayname=\"tractionload-id-0x-axis\"type=\"real\"value=\"1,11\"/>"
            + "<Arrayname=\"tractionload-id-0y-axis\"type=\"real\"value=\"2,12\"/>"
            + "<Arrayname=\"tractionload-id-0z-axis\"type=\"real\"value=\"3,13\"/>"
            + "<Arrayname=\"tractionload-id-1x-axis\"type=\"real\"value=\"4,14\"/>"
            + "<Arrayname=\"tractionload-id-1y-axis\"type=\"real\"value=\"5,15\"/>"
            + "<Arrayname=\"tractionload-id-1z-axis\"type=\"real\"value=\"6,16\"/>"
            + "<Arrayname=\"Probabilities\"type=\"real\"value=\"5.000000000000000000000e-01,5.000000000000000000000e-01\"/>";
    ASSERT_STREQ(tGold.c_str(), tReadData.str().c_str());

    std::system("rm -f defines.xml");
}

TEST(PlatoTestXMLGenerator, WriteDefineXmlFile_MaterialsAndLoads)
{
    // POSE MATERIAL SET 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("2");
    tMaterial1.category("isotropic");
    tMaterial1.property("elastic modulus", "1");
    tMaterial1.property("poissons ratio", "0.3");
    XMLGen::Material tMaterial2;
    tMaterial2.id("2");
    tMaterial2.category("isotropic");
    tMaterial2.property("elastic modulus", "1");
    tMaterial2.property("poissons ratio", "0.3");

    XMLGen::MaterialSet tMaterialSetOne;
    tMaterialSetOne.insert({"1", tMaterial1});
    tMaterialSetOne.insert({"2", tMaterial2});
    auto tRandomMaterialCase1 = std::make_pair(0.5, tMaterialSetOne);

    // POSE MATERIAL SET 2
    XMLGen::Material tMaterial3;
    tMaterial3.id("2");
    tMaterial3.category("isotropic");
    tMaterial3.property("elastic modulus", "1.1");
    tMaterial3.property("poissons ratio", "0.33");
    XMLGen::Material tMaterial4;
    tMaterial4.id("2");
    tMaterial4.category("isotropic");
    tMaterial4.property("elastic modulus", "1");
    tMaterial4.property("poissons ratio", "0.3");

    XMLGen::MaterialSet tMaterialSetTwo;
    tMaterialSetTwo.insert({"1", tMaterial3});
    tMaterialSetTwo.insert({"2", tMaterial4});
    auto tRandomMaterialCase2 = std::make_pair(0.5, tMaterialSetTwo);

    // POSE LOAD SET 1
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::Load tLoad1;
    tLoad1.mIsRandom = true;
    tLoad1.type = "traction";
    tLoad1.app_name = "sideset";
    tLoad1.values.push_back("1");
    tLoad1.values.push_back("2");
    tLoad1.values.push_back("3");
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::Load tLoad2;
    tLoad2.mIsRandom = true;
    tLoad2.type = "traction";
    tLoad2.app_name = "sideset";
    tLoad2.values.push_back("4");
    tLoad2.values.push_back("5");
    tLoad2.values.push_back("6");
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::Load tLoad3;
    tLoad3.type = "traction";
    tLoad3.mIsRandom = false;
    tLoad3.app_name = "sideset";
    tLoad3.values.push_back("7");
    tLoad3.values.push_back("8");
    tLoad3.values.push_back("9");
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    // POSE LOAD SET 2
    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::Load tLoad4;
    tLoad4.mIsRandom = true;
    tLoad4.type = "traction";
    tLoad4.app_name = "sideset";
    tLoad4.values.push_back("11");
    tLoad4.values.push_back("12");
    tLoad4.values.push_back("13");
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::Load tLoad5;
    tLoad5.mIsRandom = true;
    tLoad5.type = "traction";
    tLoad5.app_name = "sideset";
    tLoad5.values.push_back("14");
    tLoad5.values.push_back("15");
    tLoad5.values.push_back("16");
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.append(tRandomMaterialCase1));
    ASSERT_NO_THROW(tRandomMetaData.append(tRandomMaterialCase2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // SET NUM PERFORMERS
    XMLGen::UncertaintyMetaData tUncertaintyMetaData;
    tUncertaintyMetaData.numPeformers = 2;

    // CALL FUNCTION
    ASSERT_NO_THROW(XMLGen::write_define_xml_file(tRandomMetaData, tUncertaintyMetaData));

    // TEST OUTPUT FILE
    auto tReadData = XMLGen::read_data_from_file("defines.xml");

    auto tGold = std::string("<?xmlversion=\"1.0\"?><Definename=\"NumSamples\"type=\"int\"value=\"4\"/>")
        + "<Definename=\"NumPerformers\"type=\"int\"value=\"2\"/>"
        + "<Definename=\"NumSamplesPerPerformer\"type=\"int\"value=\"{NumSamples/NumPerformers}\"/>"
        + "<Definename=\"Samples\"type=\"int\"from=\"0\"to=\"{NumSamples-1}\"/>"
        + "<Definename=\"Performers\"type=\"int\"from=\"0\"to=\"{NumPerformers-1}\"/>"
        + "<Definename=\"PerformerSamples\"type=\"int\"from=\"0\"to=\"{NumSamplesPerPerformer-1}\"/>"
        + "<Arrayname=\"tractionload-id-0x-axis\"type=\"real\"value=\"1,1,11,11\"/>"
        + "<Arrayname=\"tractionload-id-0y-axis\"type=\"real\"value=\"2,2,12,12\"/>"
        + "<Arrayname=\"tractionload-id-0z-axis\"type=\"real\"value=\"3,3,13,13\"/>"
        + "<Arrayname=\"tractionload-id-1x-axis\"type=\"real\"value=\"4,4,14,14\"/>"
        + "<Arrayname=\"tractionload-id-1y-axis\"type=\"real\"value=\"5,5,15,15\"/>"
        + "<Arrayname=\"tractionload-id-1z-axis\"type=\"real\"value=\"6,6,16,16\"/>"
        + "<Arrayname=\"poissonsratioblock-id-1\"type=\"real\"value=\"0.3,0.33,0.3,0.33\"/>"
        + "<Arrayname=\"elasticmodulusblock-id-1\"type=\"real\"value=\"1,1.1,1,1.1\"/>"
        + "<Arrayname=\"poissonsratioblock-id-2\"type=\"real\"value=\"0.3,0.3,0.3,0.3\"/>"
        + "<Arrayname=\"elasticmodulusblock-id-2\"type=\"real\"value=\"1,1,1,1\"/>"
        + "<Arrayname=\"Probabilities\"type=\"real\"value=\"2.500000000000000000000e-01,"
        + "2.500000000000000000000e-01,2.500000000000000000000e-01,2.500000000000000000000e-01\"/>";
    ASSERT_STREQ(tGold.c_str(), tReadData.str().c_str());

    std::system("rm -f defines.xml");
}

TEST(PlatoTestXMLGenerator, PrepareProbabilitiesForDefineXmlFile)
{
    // 1.1 APPEND LOADS
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::Load tLoad1;
    tLoad1.mIsRandom = true;
    tLoad1.values.push_back("1");
    tLoad1.values.push_back("2");
    tLoad1.values.push_back("3");
    tLoadCase1.loads.push_back(tLoad1);
    tLoadCase1.loads[0].app_name = "sideset";
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::Load tLoad2;
    tLoad2.mIsRandom = true;
    tLoad2.values.push_back("11");
    tLoad2.values.push_back("12");
    tLoad2.values.push_back("13");
    tLoadCase2.loads.push_back(tLoad2);
    tLoadCase2.loads[0].app_name = "sideset";
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // 1.2 CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // 2. CALL FUNCTION
    auto tProbabilities = XMLGen::prepare_probabilities_for_define_xml_file(tRandomMetaData);
    ASSERT_FALSE(tProbabilities.empty());

    // 3. POSE GOLD PROBABILITIES AND TEST RESULTS
    std::vector<std::string> tGoldProbs = {"5.000000000000000000000e-01", "5.000000000000000000000e-01"};
    for(auto& tGoldProb : tGoldProbs)
    {
        auto tIndex = &tGoldProb - &tGoldProbs[0];
        ASSERT_STREQ(tGoldProb.c_str(), tProbabilities[tIndex].c_str());
    }
}

TEST(PlatoTestXMLGenerator, PrepareRandomTractionsForDefineXmlFile_AllRandomLoads_1LoadPerLoadCase_3Dim)
{
    // 1.1 APPEND LOADS
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::Load tLoad1;
    tLoad1.mIsRandom = true;
    tLoad1.values.push_back("1");
    tLoad1.values.push_back("2");
    tLoad1.values.push_back("3");
    tLoadCase1.loads.push_back(tLoad1);
    tLoadCase1.loads[0].app_name = "sideset";
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::Load tLoad2;
    tLoad2.mIsRandom = true;
    tLoad2.values.push_back("11");
    tLoad2.values.push_back("12");
    tLoad2.values.push_back("13");
    tLoadCase2.loads.push_back(tLoad2);
    tLoadCase2.loads[0].app_name = "sideset";
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // 1.2 CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // 2. CALL FUNCTION
    auto tTractionValues = XMLGen::prepare_tractions_for_define_xml_file(tRandomMetaData);
    ASSERT_FALSE(tTractionValues.empty());

    // 3. POSE GOLD LOAD VALUES AND TEST
    std::vector<std::vector<std::vector<std::string>>> tGoldValues =
        { { {"1", "11"}, {"2", "12"}, {"3", "13"} } };

    for(auto& tDims : tTractionValues)
    {
        auto tLoadIndex = &tDims - &tTractionValues[0];
        for(auto& tDim : tDims)
        {
            auto tDimIndex = &tDim - &tDims[0];
            for(auto& tSample : tDim)
            {
                auto tSampleIndex = &tSample - &tDim[0];
                ASSERT_STREQ(tGoldValues[tLoadIndex][tDimIndex][tSampleIndex].c_str(), tSample.c_str());
            }
        }
    }
}

TEST(PlatoTestXMLGenerator, PrepareRandomTractionsForDefineXmlFile_AllRandomLoads_2LoadPerLoadCase_3Dim)
{
    // 1.1 APPEND LOADS
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::Load tLoad1;
    tLoad1.mIsRandom = true;
    tLoad1.app_name = "sideset";
    tLoad1.values.push_back("1");
    tLoad1.values.push_back("2");
    tLoad1.values.push_back("3");
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::Load tLoad2;
    tLoad2.mIsRandom = true;
    tLoad2.app_name = "sideset";
    tLoad2.values.push_back("4");
    tLoad2.values.push_back("5");
    tLoad2.values.push_back("6");
    tLoadCase1.loads.push_back(tLoad2);
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::Load tLoad3;
    tLoad3.mIsRandom = true;
    tLoad3.app_name = "sideset";
    tLoad3.values.push_back("11");
    tLoad3.values.push_back("12");
    tLoad3.values.push_back("13");
    tLoadCase2.loads.push_back(tLoad3);
    XMLGen::Load tLoad4;
    tLoad4.mIsRandom = true;
    tLoad4.app_name = "sideset";
    tLoad4.values.push_back("14");
    tLoad4.values.push_back("15");
    tLoad4.values.push_back("16");
    tLoadCase2.loads.push_back(tLoad4);
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // 1.2 CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // 2. CALL FUNCTION
    auto tTractionValues = XMLGen::prepare_tractions_for_define_xml_file(tRandomMetaData);
    ASSERT_FALSE(tTractionValues.empty());

    // 3. POSE GOLD LOAD VALUES AND TEST
    std::vector<std::vector<std::vector<std::string>>> tGoldValues =
        {
          { {"1", "11"}, {"2", "12"}, {"3", "13"} },
          { {"4", "14"}, {"5", "15"}, {"6", "16"} }
        };

    for(auto& tDims : tTractionValues)
    {
        auto tLoadIndex = &tDims - &tTractionValues[0];
        for(auto& tDim : tDims)
        {
            auto tDimIndex = &tDim - &tDims[0];
            for(auto& tSample : tDim)
            {
                auto tSampleIndex = &tSample - &tDim[0];
                ASSERT_STREQ(tGoldValues[tLoadIndex][tDimIndex][tSampleIndex].c_str(), tSample.c_str());
            }
        }
    }
}

TEST(PlatoTestXMLGenerator, PrepareRandomTractionsForDefineXmlFile_2RandomLoadsAnd1DeterministicLoadPerLoadCase_3Dim)
{
    // 1.1 APPEND LOADS
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::Load tLoad1;
    tLoad1.mIsRandom = true;
    tLoad1.app_name = "sideset";
    tLoad1.values.push_back("1");
    tLoad1.values.push_back("2");
    tLoad1.values.push_back("3");
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::Load tLoad2;
    tLoad2.mIsRandom = true;
    tLoad2.app_name = "sideset";
    tLoad2.values.push_back("4");
    tLoad2.values.push_back("5");
    tLoad2.values.push_back("6");
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::Load tLoad3;
    tLoad3.mIsRandom = false;
    tLoad3.app_name = "sideset";
    tLoad3.values.push_back("7");
    tLoad3.values.push_back("8");
    tLoad3.values.push_back("9");
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::Load tLoad4;
    tLoad4.mIsRandom = true;
    tLoad4.app_name = "sideset";
    tLoad4.values.push_back("11");
    tLoad4.values.push_back("12");
    tLoad4.values.push_back("13");
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::Load tLoad5;
    tLoad5.mIsRandom = true;
    tLoad5.app_name = "sideset";
    tLoad5.values.push_back("14");
    tLoad5.values.push_back("15");
    tLoad5.values.push_back("16");
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // 1.2 CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // 2. CALL FUNCTION
    auto tTractionValues = XMLGen::prepare_tractions_for_define_xml_file(tRandomMetaData);
    ASSERT_FALSE(tTractionValues.empty());

    // 3. POSE GOLD LOAD VALUES AND TEST
    std::vector<std::vector<std::vector<std::string>>> tGoldValues =
        {
          { {"1", "11"}, {"2", "12"}, {"3", "13"} },
          { {"4", "14"}, {"5", "15"}, {"6", "16"} }
        };

    for(auto& tDims : tTractionValues)
    {
        auto tLoadIndex = &tDims - &tTractionValues[0];
        for(auto& tDim : tDims)
        {
            auto tDimIndex = &tDim - &tDims[0];
            for(auto& tSample : tDim)
            {
                auto tSampleIndex = &tSample - &tDim[0];
                ASSERT_STREQ(tGoldValues[tLoadIndex][tDimIndex][tSampleIndex].c_str(), tSample.c_str());
            }
        }
    }
}

TEST(PlatoTestXMLGenerator, PrepareRandomTractionsForDefineXmlFile_AllRandomLoads_2LoadPerLoadCase_2Dim)
{
    // 1.1 APPEND LOADS
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::Load tLoad1;
    tLoad1.mIsRandom = true;
    tLoad1.app_name = "sideset";
    tLoad1.values.push_back("1");
    tLoad1.values.push_back("2");
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::Load tLoad2;
    tLoad2.mIsRandom = true;
    tLoad2.app_name = "sideset";
    tLoad2.values.push_back("4");
    tLoad2.values.push_back("5");
    tLoadCase1.loads.push_back(tLoad2);
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::Load tLoad3;
    tLoad3.mIsRandom = true;
    tLoad3.app_name = "sideset";
    tLoad3.values.push_back("11");
    tLoad3.values.push_back("12");
    tLoadCase2.loads.push_back(tLoad3);
    XMLGen::Load tLoad4;
    tLoad4.mIsRandom = true;
    tLoad4.app_name = "sideset";
    tLoad4.values.push_back("14");
    tLoad4.values.push_back("15");
    tLoadCase2.loads.push_back(tLoad4);
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // 1.2 CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // 2. CALL FUNCTION
    auto tTractionValues = XMLGen::prepare_tractions_for_define_xml_file(tRandomMetaData);
    ASSERT_FALSE(tTractionValues.empty());

    // 3. POSE GOLD LOAD VALUES AND TEST
    std::vector<std::vector<std::vector<std::string>>> tGoldValues =
        {
          { {"1", "11"}, {"2", "12"} },
          { {"4", "14"}, {"5", "15"} }
        };

    for(auto& tDims : tTractionValues)
    {
        auto tLoadIndex = &tDims - &tTractionValues[0];
        for(auto& tDim : tDims)
        {
            auto tDimIndex = &tDim - &tDims[0];
            for(auto& tSample : tDim)
            {
                auto tSampleIndex = &tSample - &tDim[0];
                ASSERT_STREQ(tGoldValues[tLoadIndex][tDimIndex][tSampleIndex].c_str(), tSample.c_str());
            }
        }
    }
}

TEST(PlatoTestXMLGenerator, PrepareRandomTractionsForDefineXmlFile_2RandomLoadsAnd1DeterministicLoadPerLoadCase_2Dim)
{
    // 1.1 APPEND LOADS
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::Load tLoad1;
    tLoad1.mIsRandom = true;
    tLoad1.app_name = "sideset";
    tLoad1.values.push_back("1");
    tLoad1.values.push_back("2");
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::Load tLoad2;
    tLoad2.mIsRandom = true;
    tLoad2.app_name = "sideset";
    tLoad2.values.push_back("4");
    tLoad2.values.push_back("5");
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::Load tLoad3;
    tLoad3.mIsRandom = false;
    tLoad3.app_name = "sideset";
    tLoad3.values.push_back("7");
    tLoad3.values.push_back("8");
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::Load tLoad4;
    tLoad4.mIsRandom = true;
    tLoad4.app_name = "sideset";
    tLoad4.values.push_back("11");
    tLoad4.values.push_back("12");
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::Load tLoad5;
    tLoad5.mIsRandom = true;
    tLoad5.app_name = "sideset";
    tLoad5.values.push_back("14");
    tLoad5.values.push_back("15");
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // 1.2 CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // 2. CALL FUNCTION
    auto tTractionValues = XMLGen::prepare_tractions_for_define_xml_file(tRandomMetaData);
    ASSERT_FALSE(tTractionValues.empty());

    // 3. POSE GOLD LOAD VALUES AND TEST
    std::vector<std::vector<std::vector<std::string>>> tGoldValues =
        {
          { {"1", "11"}, {"2", "12"} },
          { {"4", "14"}, {"5", "15"} }
        };

    for(auto& tDims : tTractionValues)
    {
        auto tLoadIndex = &tDims - &tTractionValues[0];
        for(auto& tDim : tDims)
        {
            auto tDimIndex = &tDim - &tDims[0];
            for(auto& tSample : tDim)
            {
                auto tSampleIndex = &tSample - &tDim[0];
                ASSERT_STREQ(tGoldValues[tLoadIndex][tDimIndex][tSampleIndex].c_str(), tSample.c_str());
            }
        }
    }
}

TEST(PlatoTestXMLGenerator, AppendProbabilitiesToDefineXmlFile)
{
    pugi::xml_document tDocument;
    std::vector<std::string> tProbabilities = {"5.000000000000000000000e-01", "5.000000000000000000000e-01"};
    ASSERT_NO_THROW(XMLGen::append_probabilities_to_define_xml_file(tProbabilities, tDocument));

    // 4. POSE GOLD VALUES
    std::vector<std::string> tGoldTypes = {"real"};
    std::vector<std::string> tGoldNames = {"Probabilities"};
    std::vector<std::string> tGoldValues = {"5.000000000000000000000e-01, 5.000000000000000000000e-01"};

    // 4. TEST RESULTS AGAINST GOLD VALUES
    auto tNamesIterator = tGoldNames.begin();
    auto tTypesIterator = tGoldTypes.begin();
    auto tValuesIterator = tGoldValues.begin();
    for(pugi::xml_node tNode : tDocument.children("Array"))
    {
        ASSERT_STREQ(tNamesIterator.operator*().c_str(), tNode.attribute("name").value());
        std::advance(tNamesIterator, 1);
        ASSERT_STREQ(tTypesIterator.operator*().c_str(), tNode.attribute("type").value());
        std::advance(tTypesIterator, 1);
        ASSERT_STREQ(tValuesIterator.operator*().c_str(), tNode.attribute("value").value());
        std::advance(tValuesIterator, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendMaterialPropertiesToDefineXmlFile)
{
    // CALL FUNCTION
    std::unordered_map<std::string, std::vector<std::string>> tData =
            { {"elastic modulus block-id-1", {"1", "1.1"} },
              {"elastic modulus block-id-2", {"1", "1"} },
              {"poissons ratio block-id-1", {"0.3", "0.33"} },
              {"poissons ratio block-id-2", {"0.3", "0.3"} } };

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_material_properties_to_define_xml_file(tData, tDocument));

    // POSE GOLD VALUES
    std::vector<std::string> tGoldNames =
        {"elastic modulus block-id-1", "elastic modulus block-id-2", "poissons ratio block-id-1","poissons ratio block-id-2"};
    std::vector<std::string> tGoldValues = {"1, 1.1", "1, 1", "0.3, 0.33", "0.3, 0.3"};

    // TEST RESULTS AGAINST GOLD VALUES
    for(pugi::xml_node tNode : tDocument.children("Array"))
    {
        auto tGoldNameItr = std::find(tGoldNames.begin(), tGoldNames.end(), tNode.attribute("name").value());
        ASSERT_TRUE(tGoldNameItr != tGoldNames.end());
        ASSERT_STREQ(tGoldNameItr->c_str(), tNode.attribute("name").value());

        auto tGoldValueItr = std::find(tGoldValues.begin(), tGoldValues.end(), tNode.attribute("value").value());
        ASSERT_TRUE(tGoldValueItr != tGoldNames.end());
        ASSERT_STREQ(tGoldValueItr->c_str(), tNode.attribute("value").value());

        ASSERT_STREQ("real", tNode.attribute("type").value());
    }
}

TEST(PlatoTestXMLGenerator, ReturnMaterialPropertiesTagsForDefineXmlFile)
{
    // POSE MATERIAL SET 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("2");
    tMaterial1.category("isotropic");
    tMaterial1.property("elastic modulus", "1");
    tMaterial1.property("poissons ratio", "0.3");
    XMLGen::Material tMaterial2;
    tMaterial2.id("2");
    tMaterial2.category("isotropic");
    tMaterial2.property("elastic modulus", "1");
    tMaterial2.property("poissons ratio", "0.3");

    XMLGen::MaterialSet tMaterialSetOne;
    tMaterialSetOne.insert({"1", tMaterial1});
    tMaterialSetOne.insert({"2", tMaterial2});
    auto tRandomMaterialCaseOne = std::make_pair(0.5, tMaterialSetOne);

    // POSE MATERIAL SET 2
    XMLGen::Material tMaterial3;
    tMaterial3.id("2");
    tMaterial3.category("isotropic");
    tMaterial3.property("elastic modulus", "1.1");
    tMaterial3.property("poissons ratio", "0.33");
    XMLGen::Material tMaterial4;
    tMaterial4.id("2");
    tMaterial4.category("isotropic");
    tMaterial4.property("elastic modulus", "1");
    tMaterial4.property("poissons ratio", "0.3");

    XMLGen::MaterialSet tMaterialSetTwo;
    tMaterialSetTwo.insert({"1", tMaterial3});
    tMaterialSetTwo.insert({"2", tMaterial4});
    auto tRandomMaterialCaseTwo = std::make_pair(0.5, tMaterialSetTwo);

    // CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tRandomMaterialCaseOne));
    ASSERT_NO_THROW(tRandomMetaData.append(tRandomMaterialCaseTwo));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // CALL FUNCTION
    auto tMap = XMLGen::return_material_properties_tags_for_define_xml_file(tRandomMetaData);
    ASSERT_FALSE(tMap.empty());

    // 4. TEST RESULTS
    std::unordered_map<std::string, std::vector<std::string>> tGold =
        {
          {"1", {"elastic modulus block-id-1", "poissons ratio block-id-1"} },
          {"2", {"elastic modulus block-id-2", "poissons ratio block-id-2"} }
        };

    for(auto& tPair : tMap)
    {
        auto tGoldItr = tGold.find(tPair.first);
        ASSERT_TRUE(tGoldItr != tGold.end());
        ASSERT_STREQ(tGoldItr->first.c_str(), tPair.first.c_str());
        for(auto& tTag : tPair.second)
        {
            auto tIndex = &tTag - &tPair.second[0];
            ASSERT_STREQ(tGoldItr->second[tIndex].c_str(), tTag.c_str());
        }
    }
}

TEST(PlatoTestXMLGenerator, ReturnRandomTractionsTags)
{
    // 1. APPEND LOADS
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::Load tLoad1;
    tLoad1.mIsRandom = true;
    tLoad1.type = "traction";
    tLoad1.app_name = "sideset";
    tLoad1.values.push_back("1");
    tLoad1.values.push_back("2");
    tLoad1.values.push_back("3");
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::Load tLoad2;
    tLoad2.mIsRandom = true;
    tLoad2.type = "traction";
    tLoad2.app_name = "sideset";
    tLoad2.values.push_back("4");
    tLoad2.values.push_back("5");
    tLoad2.values.push_back("6");
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::Load tLoad3;
    tLoad3.mIsRandom = false;
    tLoad3.type = "traction";
    tLoad3.app_name = "sideset";
    tLoad3.values.push_back("7");
    tLoad3.values.push_back("8");
    tLoad3.values.push_back("9");
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::Load tLoad4;
    tLoad4.mIsRandom = true;
    tLoad4.type = "traction";
    tLoad4.app_name = "sideset";
    tLoad4.values.push_back("11");
    tLoad4.values.push_back("12");
    tLoad4.values.push_back("13");
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::Load tLoad5;
    tLoad5.mIsRandom = true;
    tLoad5.type = "traction";
    tLoad5.app_name = "sideset";
    tLoad5.values.push_back("14");
    tLoad5.values.push_back("15");
    tLoad5.values.push_back("16");
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // 2. CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // 3. CALL FUNCTION
    auto tMap = XMLGen::return_random_tractions_tags_for_define_xml_file(tRandomMetaData);
    ASSERT_FALSE(tMap.empty());

    // 4. TEST RESULTS
    std::unordered_map<std::string, std::vector<std::string>> tGold =
        {
          {"0", {"traction load-id-0 x-axis", "traction load-id-0 y-axis", "traction load-id-0 z-axis"} },
          {"1", {"traction load-id-1 x-axis", "traction load-id-1 y-axis", "traction load-id-1 z-axis"} }
        };

    for(auto& tPair : tMap)
    {
        auto tGoldItr = tGold.find(tPair.first);
        ASSERT_TRUE(tGoldItr != tGold.end());
        ASSERT_STREQ(tGoldItr->first.c_str(), tPair.first.c_str());
        for(auto& tTag : tPair.second)
        {
            auto tIndex = &tTag - &tPair.second[0];
            ASSERT_STREQ(tGoldItr->second[tIndex].c_str(), tTag.c_str());
        }
    }
}

TEST(PlatoTestXMLGenerator, AppendRandomTractionsToDefineXmlFile)
{
    std::unordered_map<std::string, std::vector<std::string>> tTractionTags =
        {
          {"0", {"traction load-id-0 x-axis","traction load-id-0 y-axis", "traction load-id-0 z-axis"} },
          {"1", {"traction load-id-1 x-axis","traction load-id-1 y-axis", "traction load-id-1 z-axis"} }
        };
    std::vector<std::vector<std::vector<std::string>>> tTractionValues =
        {
          { {"1", "11"}, {"2", "12"}, {"3", "13"} },
          { {"4", "14"}, {"5", "15"}, {"6", "16"} }
        };

    pugi::xml_document tDocument;

    ASSERT_NO_THROW(XMLGen::append_tractions_to_define_xml_file(tTractionTags, tTractionValues, tDocument));

    // 4. POSE GOLD VALUES
    std::vector<std::string> tGoldTypes =
        {"real", "real", "real", "real", "real", "real"};
    std::vector<std::string> tGoldNames =
        {"traction load-id-0 x-axis", "traction load-id-0 y-axis", "traction load-id-0 z-axis",
         "traction load-id-1 x-axis", "traction load-id-1 y-axis", "traction load-id-1 z-axis"};
    std::vector<std::string> tGoldValues = {"1, 11", "2, 12", "3, 13", "4, 14", "5, 15", "6, 16"};

    // 4. TEST RESULTS AGAINST GOLD VALUES
    auto tNamesIterator = tGoldNames.begin();
    auto tTypesIterator = tGoldTypes.begin();
    auto tValuesIterator = tGoldValues.begin();
    for(pugi::xml_node tNode : tDocument.children("Array"))
    {
        ASSERT_STREQ(tNamesIterator.operator*().c_str(), tNode.attribute("name").value());
        std::advance(tNamesIterator, 1);
        ASSERT_STREQ(tTypesIterator.operator*().c_str(), tNode.attribute("type").value());
        std::advance(tTypesIterator, 1);
        ASSERT_STREQ(tValuesIterator.operator*().c_str(), tNode.attribute("value").value());
        std::advance(tValuesIterator, 1);
    }
}

TEST(PlatoTestXMLGenerator, PrepareRandomMaterialPropertiesForDefineXmlFile)
{
    // POSE MATERIAL SET 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("2");
    tMaterial1.category("isotropic");
    tMaterial1.property("elastic modulus", "1");
    tMaterial1.property("poissons ratio", "0.3");
    XMLGen::Material tMaterial2;
    tMaterial2.id("2");
    tMaterial2.category("isotropic");
    tMaterial2.property("elastic modulus", "1");
    tMaterial2.property("poissons ratio", "0.3");

    XMLGen::MaterialSet tMaterialSetOne;
    tMaterialSetOne.insert({"1", tMaterial1});
    tMaterialSetOne.insert({"2", tMaterial2});
    auto tRandomMaterialCaseOne = std::make_pair(0.5, tMaterialSetOne);

    // POSE MATERIAL SET 2
    XMLGen::Material tMaterial3;
    tMaterial3.id("2");
    tMaterial3.category("isotropic");
    tMaterial3.property("elastic modulus", "1.1");
    tMaterial3.property("poissons ratio", "0.33");
    XMLGen::Material tMaterial4;
    tMaterial4.id("2");
    tMaterial4.category("isotropic");
    tMaterial4.property("elastic modulus", "1");
    tMaterial4.property("poissons ratio", "0.3");

    XMLGen::MaterialSet tMaterialSetTwo;
    tMaterialSetTwo.insert({"1", tMaterial3});
    tMaterialSetTwo.insert({"2", tMaterial4});
    auto tRandomMaterialCaseTwo = std::make_pair(0.5, tMaterialSetTwo);

    // CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tRandomMaterialCaseOne));
    ASSERT_NO_THROW(tRandomMetaData.append(tRandomMaterialCaseTwo));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // CALL FUNCTION
    auto tMaterialValues = XMLGen::prepare_material_properties_for_define_xml_file(tRandomMetaData);
    ASSERT_FALSE(tMaterialValues.empty());

    // POSE GOLD LOAD VALUES AND TEST
    std::unordered_map<std::string, std::vector<std::string>> tGold =
            { {"elastic modulus block-id-1", {"1", "1.1"} },
              {"elastic modulus block-id-2", {"1", "1"} },
              {"poissons ratio block-id-1", {"0.3", "0.33"} },
              {"poissons ratio block-id-2", {"0.3", "0.3"} } };
    for(auto& tPair : tMaterialValues)
    {
        auto tGoldItr = tGold.find(tPair.first);
        ASSERT_TRUE(tGoldItr != tGold.end());
        ASSERT_STREQ(tGoldItr->first.c_str(), tPair.first.c_str());

        auto tGoldSamplesItr = tGoldItr->second.begin();
        for(auto& tSample : tPair.second)
        {
            ASSERT_STREQ(tGoldSamplesItr.operator*().c_str(), tSample.c_str());
            std::advance(tGoldSamplesItr, 1);
        }
    }
}

TEST(PlatoTestXMLGenerator, AppendBasicAttributesToDefineXmlFile_ErrorZeroNumSamples)
{
    pugi::xml_document tDocument;
    XMLGen::RandomMetaData tRandomMetaData;
    XMLGen::UncertaintyMetaData tUncertaintyMetaData;
    ASSERT_THROW(XMLGen::append_basic_attributes_to_define_xml_file(tRandomMetaData, tUncertaintyMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendBasicAttributesToDefineXmlFile_ErrorZeroPerformers)
{
    // 1.1 APPEND LOADS
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    tLoadCase1.loads.push_back(XMLGen::Load());
    tLoadCase1.loads[0].app_name = "sideset";
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    tLoadCase2.loads.push_back(XMLGen::Load());
    tLoadCase2.loads[0].app_name = "sideset";
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // 1.2 CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    pugi::xml_document tDocument;
    XMLGen::UncertaintyMetaData tUncertaintyMetaData;
    ASSERT_THROW(XMLGen::append_basic_attributes_to_define_xml_file(tRandomMetaData, tUncertaintyMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendBasicAttributesToDefineXmlFile)
{
    // 1.1 APPEND LOADS
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    tLoadCase1.loads.push_back(XMLGen::Load());
    tLoadCase1.loads[0].app_name = "sideset";
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    tLoadCase2.loads.push_back(XMLGen::Load());
    tLoadCase2.loads[0].app_name = "sideset";
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // 1.2 CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // 2 SET NUM PERFORMERS
    XMLGen::UncertaintyMetaData tUncertaintyMetaData;
    tUncertaintyMetaData.numPeformers = 2;

    // 3. CALL FUNCTION
    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_basic_attributes_to_define_xml_file(tRandomMetaData, tUncertaintyMetaData, tDocument));

    // 4. POSE GOLD VALUES
    std::vector<std::string> tGoldTypes =
        {"int", "int", "int", "int", "int", "int"};
    std::vector<std::string> tGoldNames =
        {"NumSamples", "NumPerformers", "NumSamplesPerPerformer", "Samples", "Performers", "PerformerSamples"};
    std::vector<std::string> tGoldValues = {"2", "2", "{NumSamples/NumPerformers}", "", "", ""};
    std::vector<std::string> tGoldToValues = {"", "", "", "{NumSamples-1}", "{NumPerformers-1}", "{NumSamplesPerPerformer-1}"};
    std::vector<std::string> tGoldFromValues = {"", "", "", "0", "0", "0"};

    // 4. TEST RESULTS AGAINST GOLD VALUES
    auto tNamesIterator = tGoldNames.begin();
    auto tTypesIterator = tGoldTypes.begin();
    auto tValuesIterator = tGoldValues.begin();
    auto tToValuesIterator = tGoldToValues.begin();
    auto tFromValuesIterator = tGoldFromValues.begin();
    for(pugi::xml_node tNode : tDocument.children("Define"))
    {
        ASSERT_STREQ(tNamesIterator.operator*().c_str(), tNode.attribute("name").value());
        std::advance(tNamesIterator, 1);
        ASSERT_STREQ(tTypesIterator.operator*().c_str(), tNode.attribute("type").value());
        std::advance(tTypesIterator, 1);
        ASSERT_STREQ(tValuesIterator.operator*().c_str(), tNode.attribute("value").value());
        std::advance(tValuesIterator, 1);
        ASSERT_STREQ(tToValuesIterator.operator*().c_str(), tNode.attribute("to").value());
        std::advance(tToValuesIterator, 1);
        ASSERT_STREQ(tFromValuesIterator.operator*().c_str(), tNode.attribute("from").value());
        std::advance(tFromValuesIterator, 1);
    }
}

TEST(PlatoTestXMLGenerator, ComputeGreatestDivisor)
{
    size_t tNumSamples = 10;
    size_t tNumPerformers = 2;
    auto tOutput = XMLGen::compute_greatest_divisor(tNumSamples, tNumPerformers);
    ASSERT_EQ(2u, tOutput);

    tNumPerformers = 3;
    tOutput = XMLGen::compute_greatest_divisor(tNumSamples, tNumPerformers);
    ASSERT_EQ(2u, tOutput);

    tNumPerformers = 4;
    tOutput = XMLGen::compute_greatest_divisor(tNumSamples, tNumPerformers);
    ASSERT_EQ(2u, tOutput);

    tNumPerformers = 6;
    tOutput = XMLGen::compute_greatest_divisor(tNumSamples, tNumPerformers);
    ASSERT_EQ(5u, tOutput);

    tNumPerformers = 0;
    ASSERT_THROW(XMLGen::compute_greatest_divisor(tNumSamples, tNumPerformers), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, TransformTokens)
{
    // TEST 1
    std::vector<std::string> tTokens;
    auto tOutput = XMLGen::transform_tokens(tTokens);
    ASSERT_STREQ("", tOutput.c_str());

    // TEST 2
    tTokens = {"1", "2", "3", "4", "5", "6"};
    tOutput = XMLGen::transform_tokens(tTokens);
    ASSERT_STREQ("1, 2, 3, 4, 5, 6", tOutput.c_str());
}

TEST(PlatoTestXMLGenerator, CheckOutputLoadSetType_Error)
{
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    tLoadCase1.loads.push_back(XMLGen::Load());
    tLoadCase1.loads[0].type = "traction";
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    tLoadCase2.loads.push_back(XMLGen::Load());
    tLoadCase2.loads[0].type = "pressure";
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    XMLGen::RandomMetaData tMetaData;
    ASSERT_NO_THROW(tMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tMetaData.finalize());
    ASSERT_EQ(2u, tMetaData.numSamples());

    ASSERT_THROW(Plato::srom::check_output_load_set_types(tMetaData), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, CheckOutputLoadSetSize_Error)
{
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    tLoadCase1.loads.push_back(XMLGen::Load());
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    tLoadCase2.loads.push_back(XMLGen::Load());
    tLoadCase2.loads.push_back(XMLGen::Load());
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    XMLGen::RandomMetaData tMetaData;
    ASSERT_NO_THROW(tMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tMetaData.finalize());
    ASSERT_EQ(2u, tMetaData.numSamples());

    ASSERT_THROW(Plato::srom::check_output_load_set_size(tMetaData), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, CheckOutputLoadSetApplicationName_Error)
{
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    tLoadCase1.loads.push_back(XMLGen::Load());
    tLoadCase1.loads[0].app_name = "sideset";
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    tLoadCase2.loads.push_back(XMLGen::Load());
    tLoadCase2.loads[0].app_name = "nodeset";
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    XMLGen::RandomMetaData tMetaData;
    ASSERT_NO_THROW(tMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tMetaData.finalize());
    ASSERT_EQ(2u, tMetaData.numSamples());

    ASSERT_THROW(Plato::srom::check_output_load_set_application_name(tMetaData), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, PostprocessMaterialOutputs_ErrorEmptyRandomMaterialCases)
{
    Plato::srom::OutputMetaData tOutput;
    XMLGen::InputData tXMLGenMetaData;
    ASSERT_THROW(Plato::srom::postprocess_material_outputs(tOutput, tXMLGenMetaData), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, BuildMaterialSet_ErrorEmptyMaterialList)
{
    Plato::srom::RandomMaterialCase tRandMaterialCase;
    EXPECT_THROW(Plato::srom::build_material_set(tRandMaterialCase), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, BuildMaterialSet)
{
    // POSE DATA
    Plato::srom::RandomMaterial tRandMaterial1;
    tRandMaterial1.blockID("0");
    tRandMaterial1.materialID("10");
    tRandMaterial1.category("isotropic");
    tRandMaterial1.append("youngs modulus", "homogeneous", "1");
    tRandMaterial1.append("poissons ratio", "homogeneous", "0.3");

    Plato::srom::RandomMaterial tRandMaterial2;
    tRandMaterial2.blockID("1");
    tRandMaterial2.materialID("11");
    tRandMaterial2.category("isotropic");
    tRandMaterial2.append("youngs modulus", "homogeneous", "2");
    tRandMaterial2.append("poissons ratio", "homogeneous", "0.33");

    Plato::srom::RandomMaterialCase tRandMaterialCase;
    tRandMaterialCase.caseID("0");
    tRandMaterialCase.probability(0.5);
    tRandMaterialCase.append("10", tRandMaterial1);
    tRandMaterialCase.append("11", tRandMaterial2);

    // CALL FUNCTION
    auto tMaterialSet = Plato::srom::build_material_set(tRandMaterialCase);

    // TEST DATA
    const double tTolerance = 1e-6;
    ASSERT_NEAR(0.5, tMaterialSet.first, tTolerance);

    std::vector<std::string> tGoldPropertiesMatOne = {"0.3" , "1"};
    std::vector<std::string> tGoldPropertiesMatTwo = {"0.33" , "2"};

    for(auto& tPair : tMaterialSet.second)
    {
        // unordered map; thus, if statement
        if(tPair.first == "0")
        {
            ASSERT_STREQ("10", tPair.second.id().c_str());
            ASSERT_STREQ("isotropic", tPair.second.category().c_str());
            auto tTags = tPair.second.tags();
            for(auto& tTag : tTags)
            {
                auto tTagIndex = &tTag - &tTags[0];
                ASSERT_STREQ(tGoldPropertiesMatOne[tTagIndex].c_str(), tPair.second.property(tTag).c_str());
            }
        }
        else
        {
            ASSERT_STREQ("11", tPair.second.id().c_str());
            ASSERT_STREQ("isotropic", tPair.second.category().c_str());
            auto tTags = tPair.second.tags();
            for(auto& tTag : tTags)
            {
                auto tTagIndex = &tTag - &tTags[0];
                ASSERT_STREQ(tGoldPropertiesMatTwo[tTagIndex].c_str(), tPair.second.property(tTag).c_str());
            }
        }
    }
}

TEST(PlatoTestXMLGenerator, PostprocessMaterialOutputs)
{
    Plato::srom::OutputMetaData tOutput;
    tOutput.usecase(Plato::srom::usecase::MATERIAL);

    // CASE 1
    Plato::srom::RandomMaterial tRandMaterial1;
    tRandMaterial1.blockID("0");
    tRandMaterial1.materialID("10");
    tRandMaterial1.category("isotropic");
    tRandMaterial1.append("youngs modulus", "homogeneous", "1");
    tRandMaterial1.append("poissons ratio", "homogeneous", "0.3");

    Plato::srom::RandomMaterial tRandMaterial2;
    tRandMaterial2.blockID("1");
    tRandMaterial2.materialID("11");
    tRandMaterial2.category("isotropic");
    tRandMaterial2.append("youngs modulus", "homogeneous", "2");
    tRandMaterial2.append("poissons ratio", "homogeneous", "0.33");

    Plato::srom::RandomMaterialCase tRandMaterialCase1;
    tRandMaterialCase1.caseID("0");
    tRandMaterialCase1.probability(0.5);
    tRandMaterialCase1.append("10", tRandMaterial1);
    tRandMaterialCase1.append("11", tRandMaterial2);

    // CASE 2
    Plato::srom::RandomMaterial tRandMaterial3;
    tRandMaterial3.blockID("0");
    tRandMaterial3.materialID("10");
    tRandMaterial3.category("isotropic");
    tRandMaterial3.append("youngs modulus", "homogeneous", "1.1");
    tRandMaterial3.append("poissons ratio", "homogeneous", "0.31");

    Plato::srom::RandomMaterial tRandMaterial4;
    tRandMaterial4.blockID("1");
    tRandMaterial4.materialID("11");
    tRandMaterial4.category("isotropic");
    tRandMaterial4.append("youngs modulus", "homogeneous", "2.2");
    tRandMaterial4.append("poissons ratio", "homogeneous", "0.35");

    Plato::srom::RandomMaterialCase tRandMaterialCase2;
    tRandMaterialCase2.caseID("1");
    tRandMaterialCase2.probability(0.5);
    tRandMaterialCase2.append("10", tRandMaterial3);
    tRandMaterialCase2.append("11", tRandMaterial4);

    // APPEND CASES
    tOutput.append(tRandMaterialCase1);
    tOutput.append(tRandMaterialCase2);

    // TEST
    XMLGen::InputData tXMLGenMetaData;
    EXPECT_NO_THROW(Plato::srom::postprocess_material_outputs(tOutput, tXMLGenMetaData));
    EXPECT_NO_THROW(tXMLGenMetaData.mRandomMetaData.finalize());

    std::vector<std::string> tBlockIDs = {"0", "1"};
    std::vector<std::vector<std::string>> tGoldMatIDs = { {"10", "11"}, {"10", "11"} };
    std::vector<std::vector<std::string>> tGoldCategoryIDs = { {"isotropic", "isotropic"}, {"isotropic", "isotropic"} };
    std::vector<std::vector<std::vector<std::string>>> tGoldProperties =
        {
          { {"0.3" , "1"}  , {"0.33", "2"  } },
          { {"0.31", "1.1"}, {"0.35", "2.2"} }
        };
    ASSERT_EQ(2u, tXMLGenMetaData.mRandomMetaData.numSamples());
    auto tRandomSamples = tXMLGenMetaData.mRandomMetaData.samples();
    for(auto& tRandomSample : tRandomSamples)
    {
        auto tSampleIndex = &tRandomSample - &tRandomSamples[0];
        for(auto& tBlockID : tBlockIDs)
        {
            auto tBlockIndex = &tBlockID - &tBlockIDs[0];
            auto tMaterial = tRandomSample.material(tBlockID);
            ASSERT_STREQ(tGoldMatIDs[tSampleIndex][tBlockIndex].c_str(), tMaterial.id().c_str());
            ASSERT_STREQ(tGoldCategoryIDs[tSampleIndex][tBlockIndex].c_str(), tMaterial.category().c_str());
            auto tTags = tMaterial.tags();
            for(auto& tTag : tTags)
            {
                auto tTagIndex = &tTag - &tTags[0];
                ASSERT_STREQ(tGoldProperties[tSampleIndex][tBlockIndex][tTagIndex].c_str(), tMaterial.property(tTag).c_str());
            }
        }
    }
}

TEST(PlatoTestXMLGenerator, RandomMetaData_Set_ErrorUndefinedLoadCaseID)
{
    XMLGen::RandomMetaData tMetaData;
    auto tLoadSet = std::make_pair(0.5, XMLGen::LoadCase());
    EXPECT_THROW(tMetaData.append(tLoadSet), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, RandomMetaData_Set_ErrorUndefinedLoadsContainer)
{
    XMLGen::RandomMetaData tMetaData;
    XMLGen::LoadCase tLoadCase;
    tLoadCase.id = "1";
    auto tLoadSet = std::make_pair(0.5, tLoadCase);
    EXPECT_THROW(tMetaData.append(tLoadSet), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, RandomMetaData_Append_ErrorUndefinedBlockID)
{
    XMLGen::RandomMetaData tMetaData;
    XMLGen::Material tMaterial;
    XMLGen::MaterialSet tMaterialMap;
    tMaterialMap.insert({"", tMaterial});
    auto tRandomMaterialCase = std::make_pair(0.5, tMaterialMap);
    EXPECT_THROW(tMetaData.append(tRandomMaterialCase), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, RandomMetaData_Append_ErrorUndefinedMaterialID)
{
    XMLGen::RandomMetaData tMetaData;
    XMLGen::Material tMaterial;
    XMLGen::MaterialSet tMaterialMap;
    tMaterialMap.insert({"1", tMaterial});
    auto tRandomMaterialCase = std::make_pair(0.5, tMaterialMap);
    EXPECT_THROW(tMetaData.append(tRandomMaterialCase), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, RandomMetaData_Append_ErrorUndefinedCategory)
{
    XMLGen::RandomMetaData tMetaData;
    XMLGen::Material tMaterial;
    tMaterial.id("2");
    tMaterial.category("");
    XMLGen::MaterialSet tMaterialMap;
    tMaterialMap.insert({"1", tMaterial});
    auto tRandomMaterialCase = std::make_pair(0.5, tMaterialMap);
    EXPECT_THROW(tMetaData.append(tRandomMaterialCase), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, RandomMetaData_Append_ErrorUndefinedProperties)
{
    XMLGen::RandomMetaData tMetaData;
    XMLGen::Material tMaterial;
    tMaterial.id("2");
    tMaterial.category("isotropic");
    XMLGen::MaterialSet tMaterialMap;
    tMaterialMap.insert({"1", tMaterial});
    auto tRandomMaterialCase = std::make_pair(0.5, tMaterialMap);
    EXPECT_THROW(tMetaData.append(tRandomMaterialCase), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, SetBlockIdentificationNumber_Error1)
{
    // ERROR - EMPTY MATERIAL ID
    Plato::srom::Material tMaterial;
    tMaterial.category("isotropic");
    tMaterial.append("poissons ratio", "homogeneous", "0.35");
    tMaterial.append("elastic modulus", "homogeneous", "2.5e8");

    std::unordered_map<std::string, std::string> tMap;
    EXPECT_THROW(Plato::srom::set_block_identification_number(tMap, tMaterial), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, SetBlockIdentificationNumber_Error2)
{
    // ERROR - MATERIAL ID IS NOT OWN BY ANY BLOCK ON THE LIST
    Plato::srom::Material tMaterial;
    tMaterial.materialID("1");
    tMaterial.category("isotropic");
    tMaterial.append("poissons ratio", "homogeneous", "0.35");
    tMaterial.append("elastic modulus", "homogeneous", "2.5e8");

    std::unordered_map<std::string, std::string> tMap;
    EXPECT_THROW(Plato::srom::set_block_identification_number(tMap, tMaterial), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, SetBlockIdentificationNumber_Error3)
{
    // ERROR - EMPTY BLOCK ID
    Plato::srom::Material tMaterial;
    tMaterial.materialID("1");
    tMaterial.category("isotropic");
    tMaterial.append("poissons ratio", "homogeneous", "0.35");
    tMaterial.append("elastic modulus", "homogeneous", "2.5e8");

    std::unordered_map<std::string, std::string> tMap;
    tMap.insert({"1", ""});
    EXPECT_THROW(Plato::srom::set_block_identification_number(tMap, tMaterial), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, SetBlockIdentificationNumber1)
{
    Plato::srom::Material tMaterial;
    tMaterial.materialID("1");
    tMaterial.category("isotropic");
    tMaterial.append("poissons ratio", "homogeneous", "0.35");
    tMaterial.append("elastic modulus", "homogeneous", "2.5e8");

    std::unordered_map<std::string, std::string> tMap;
    tMap.insert({"1", "10"});
    EXPECT_TRUE(tMaterial.blockID().empty());
    EXPECT_NO_THROW(Plato::srom::set_block_identification_number(tMap, tMaterial));
    ASSERT_STREQ("10", tMaterial.blockID().c_str());
}

TEST(PlatoTestXMLGenerator, SetBlockIdentificationNumber2)
{
    Plato::srom::Material tMaterial;
    tMaterial.materialID("1");
    tMaterial.category("isotropic");
    tMaterial.append("poissons ratio", "homogeneous", "0.35");
    tMaterial.append("elastic modulus", "homogeneous", "2.5e8");

    std::unordered_map<std::string, std::string> tMap;
    tMap.insert({"1", "10"});
    tMap.insert({"2", "1"});
    tMap.insert({"13", "14"});
    EXPECT_TRUE(tMaterial.blockID().empty());
    EXPECT_NO_THROW(Plato::srom::set_block_identification_number(tMap, tMaterial));
    ASSERT_STREQ("10", tMaterial.blockID().c_str());
}

TEST(PlatoTestXMLGenerator, BuildBlockIDtoMaterialIDmap_Error1)
{
    XMLGen::InputData tInputMetaData;
    EXPECT_THROW(Plato::srom::build_material_id_to_block_id_map(tInputMetaData), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, BuildBlockIDtoMaterialIDmap_Error2)
{
    XMLGen::Material tMaterial;
    tMaterial.id("30");
    tMaterial.category("isotropic");
    tMaterial.property("youngs modulus", "0.5");
    tMaterial.property("poissons ratio", "0.3");

    XMLGen::InputData tInputMetaData;
    tInputMetaData.materials.push_back(tMaterial);

    EXPECT_THROW(Plato::srom::build_material_id_to_block_id_map(tInputMetaData), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, BuildBlockIDtoMaterialIDmap)
{
    // BUILD MATERIALS AND BLOCKS
    XMLGen::Material tMaterial1;
    tMaterial1.id("30");
    tMaterial1.category("isotropic");
    tMaterial1.property("youngs modulus", "0.5");
    tMaterial1.property("poissons ratio", "0.3");

    XMLGen::Material tMaterial2;
    tMaterial2.id("3");
    tMaterial2.category("isotropic");
    tMaterial2.property("youngs modulus", "0.5");
    tMaterial2.property("poissons ratio", "0.3");

    XMLGen::Block tBlock1;
    tBlock1.block_id = "1";
    tBlock1.material_id = "30";
    tBlock1.element_type = "tet4";

    XMLGen::Block tBlock2;
    tBlock2.block_id = "2";
    tBlock2.material_id = "3";
    tBlock2.element_type = "tet4";

    // APPEND MATERIALS AND BLOCKS
    XMLGen::InputData tInputMetaData;
    tInputMetaData.blocks.push_back(tBlock1);
    tInputMetaData.blocks.push_back(tBlock2);
    tInputMetaData.materials.push_back(tMaterial1);
    tInputMetaData.materials.push_back(tMaterial2);

    // BUILD MAP
    auto tMap = Plato::srom::build_material_id_to_block_id_map(tInputMetaData);

    // TEST RESULTS
    ASSERT_STREQ("1", tMap.find("30")->second.c_str());
    ASSERT_STREQ("2", tMap.find("3")->second.c_str());
}

TEST(PlatoTestXMLGenerator, PreprocessNondeterministicMaterialInputs_Error1)
{
    // ERROR - INPUT MATERIAL CONTAINER IS EMPTY
    XMLGen::InputData tInputMetadata;
    Plato::srom::InputMetaData tSromInputs;
    EXPECT_THROW(Plato::srom::preprocess_material_inputs(tInputMetadata, tSromInputs), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, PreprocessNondeterministicMaterialInputs_Error2)
{
    // 1. POSE DATA
    XMLGen::Uncertainty tCase1;
    tCase1.variable_type = "load";
    tCase1.axis = "x";
    tCase1.distribution = "beta";
    tCase1.id = "2";
    tCase1.lower = "-2";
    tCase1.upper = "2";
    tCase1.mean = "0";
    tCase1.standard_deviation = "0.2";
    tCase1.num_samples = "12";
    tCase1.type = "angle variation";

    XMLGen::Material tMaterial;
    tMaterial.id("30");
    tMaterial.category("isotropic");
    tMaterial.property("youngs modulus", "0.5");
    tMaterial.property("poissons ratio", "0.3");

    XMLGen::InputData tMetadata;
    tMetadata.materials.push_back(tMaterial);
    tMetadata.uncertainties.push_back(tCase1);

    // ERROR - NO RANDOM MATERIAL PROPERTIES
    Plato::srom::InputMetaData tSromInputs;
    EXPECT_THROW(Plato::srom::preprocess_material_inputs(tMetadata, tSromInputs), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, PreprocessNondeterministicMaterialInputs)
{
    // 1. POSE UNCERTAINTY DATA
    XMLGen::Uncertainty tCase1;
    tCase1.variable_type = "load";
    tCase1.axis = "x";
    tCase1.distribution = "beta";
    tCase1.id = "2";
    tCase1.lower = "-2";
    tCase1.upper = "2";
    tCase1.mean = "0";
    tCase1.standard_deviation = "0.2";
    tCase1.num_samples = "12";
    tCase1.type = "angle variation";

    XMLGen::Uncertainty tCase2;
    tCase2.variable_type = "load";
    tCase2.axis = "y";
    tCase2.distribution = "beta";
    tCase2.id = "20";
    tCase2.lower = "-20";
    tCase2.upper = "20";
    tCase2.mean = "2";
    tCase2.standard_deviation = "2";
    tCase2.num_samples = "8";
    tCase2.type = "angle variation";

    XMLGen::Uncertainty tCase3;
    tCase3.variable_type = "material";
    tCase3.axis = "homogeneous";
    tCase3.distribution = "beta";
    tCase3.id = "3";
    tCase3.lower = "0.2";
    tCase3.upper = "0.35";
    tCase3.mean = "0.27";
    tCase3.standard_deviation = "0.05";
    tCase3.num_samples = "6";
    tCase3.type = "poissons ratio";

    XMLGen::InputData tMetadata;
    tMetadata.uncertainties.push_back(tCase1);
    tMetadata.uncertainties.push_back(tCase2);
    tMetadata.uncertainties.push_back(tCase3);

    // 2. POSE MATERIAL DATA
    XMLGen::Material tMaterial1;
    tMaterial1.id("30");
    tMaterial1.category("isotropic");
    tMaterial1.property("youngs modulus", "0.5");
    tMaterial1.property("poissons ratio", "0.3");
    tMetadata.materials.push_back(tMaterial1);

    XMLGen::Material tMaterial2;
    tMaterial2.id("3");
    tMaterial2.category("isotropic");
    tMaterial2.property("youngs modulus", "1");
    tMaterial2.property("poissons ratio", "0.3");
    tMetadata.materials.push_back(tMaterial2);

    // 3. POSE MATERIAL BLOCKS
    XMLGen::Block tBlock1;
    tBlock1.block_id = "1";
    tBlock1.material_id = "30";
    tBlock1.element_type = "tet4";
    tMetadata.blocks.push_back(tBlock1);

    XMLGen::Block tBlock2;
    tBlock2.block_id = "2";
    tBlock2.material_id = "3";
    tBlock2.element_type = "tet4";
    tMetadata.blocks.push_back(tBlock2);

    // 3. CALL FUNCTION
    Plato::srom::InputMetaData tSromInputs;
    tSromInputs.usecase(Plato::srom::usecase::MATERIAL);
    EXPECT_NO_THROW(Plato::srom::preprocess_material_inputs(tMetadata, tSromInputs));
    ASSERT_EQ(2u, tSromInputs.materials().size());

    // 4.1 TEST RESULTS
    auto tMaterials = tSromInputs.materials();

    // 4.2 DETERMINISTIC MATERIAL
    ASSERT_EQ(2u, tMaterials[0].tags().size());
    ASSERT_STREQ("1", tMaterials[0].blockID().c_str());
    ASSERT_STREQ("30", tMaterials[0].materialID().c_str());
    ASSERT_STREQ("isotropic", tMaterials[0].category().c_str());
    ASSERT_STREQ("poissons ratio", tMaterials[0].tags()[0].c_str());
    ASSERT_STREQ("youngs modulus", tMaterials[0].tags()[1].c_str());

    EXPECT_FALSE(tMaterials[0].isRandom());
    EXPECT_TRUE(tMaterials[0].isDeterministic());
    EXPECT_TRUE(tMaterials[0].randomVars().empty());
    ASSERT_STREQ("poissons ratio", tMaterials[0].deterministicVars()[0].tag().c_str());
    ASSERT_STREQ("0.3", tMaterials[0].deterministicVars()[0].value().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[0].deterministicVars()[0].attribute().c_str());
    ASSERT_STREQ("youngs modulus", tMaterials[0].deterministicVars()[1].tag().c_str());
    ASSERT_STREQ("0.5", tMaterials[0].deterministicVars()[1].value().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[0].deterministicVars()[1].attribute().c_str());

    // 4.3 NON-DETERMINISTIC MATERIAL
    ASSERT_STREQ("2", tMaterials[1].blockID().c_str());
    ASSERT_STREQ("3", tMaterials[1].materialID().c_str());
    ASSERT_STREQ("isotropic", tMaterials[1].category().c_str());
    ASSERT_STREQ("poissons ratio", tMaterials[1].tags()[0].c_str());
    ASSERT_STREQ("youngs modulus", tMaterials[1].tags()[1].c_str());

    EXPECT_TRUE(tMaterials[1].isRandom());
    EXPECT_FALSE(tMaterials[1].isDeterministic());
    ASSERT_EQ(1u, tMaterials[1].deterministicVars().size());
    ASSERT_STREQ("youngs modulus", tMaterials[1].deterministicVars()[0].tag().c_str());
    ASSERT_STREQ("1", tMaterials[1].deterministicVars()[0].value().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[1].deterministicVars()[0].attribute().c_str());

    ASSERT_EQ(1u, tMaterials[1].randomVars().size());
    ASSERT_EQ(0, tMaterials[1].randomVars()[0].id());
    EXPECT_TRUE(tMaterials[1].randomVars()[0].file().empty());
    ASSERT_STREQ("poissons ratio", tMaterials[1].randomVars()[0].tag().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[1].randomVars()[0].attribute().c_str());
    ASSERT_STREQ("0.05", tMaterials[1].randomVars()[0].deviation().c_str());
    ASSERT_STREQ("beta", tMaterials[1].randomVars()[0].distribution().c_str());
    ASSERT_STREQ("6", tMaterials[1].randomVars()[0].samples().c_str());
    ASSERT_STREQ("0.27", tMaterials[1].randomVars()[0].mean().c_str());
    ASSERT_STREQ("0.2", tMaterials[1].randomVars()[0].lower().c_str());
    ASSERT_STREQ("0.35", tMaterials[1].randomVars()[0].upper().c_str());
}

TEST(PlatoTestXMLGenerator, AppendRandomMaterial_Error)
{
    XMLGen::Material tMaterial;
    Plato::srom::Material tSromMaterial;
    Plato::srom::RandomMatPropMap tEmptyMap;
    EXPECT_THROW(Plato::srom::append_material_properties(tMaterial, tEmptyMap, tSromMaterial), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendRandomMaterial_DeterministicMaterialCase)
{
    // 1. BUILD RANDOM MATERIAL VARIABLE
    XMLGen::Uncertainty tCase1;
    tCase1.id = "3";
    tCase1.axis = "homogeneous";
    tCase1.type = "poissons ratio";
    tCase1.variable_type = "material";
    tCase1.distribution = "beta";
    tCase1.lower = "0.2";
    tCase1.upper = "0.35";
    tCase1.mean = "0.27";
    tCase1.num_samples = "6";
    tCase1.standard_deviation = "0.05";

    // 2. BUILD RANDOM MATERIAL MAP
    std::vector<XMLGen::Uncertainty> tRandomVars;
    tRandomVars.push_back(tCase1);
    auto tRandomMaterialMap = Plato::srom::build_material_id_to_random_material_map(tRandomVars);

    // 3. BUILD DETERMINISTIC MATERIAL METADATA
    XMLGen::Material tMaterial;
    tMaterial.id("30");
    tMaterial.category("isotropic");
    tMaterial.property("youngs modulus", "0.5");
    tMaterial.property("poissons ratio", "0.3");

    // 4. APPEND MATERIAL TO SROM MATERIAL METADATA
    Plato::srom::Material tSromMaterial;
    EXPECT_NO_THROW(Plato::srom::append_material_properties(tMaterial, tRandomMaterialMap, tSromMaterial));

    // 5. TEST RESULTS
    ASSERT_FALSE(tSromMaterial.isRandom());
    ASSERT_TRUE(tSromMaterial.isDeterministic());
    ASSERT_TRUE(tSromMaterial.randomVars().empty());
    ASSERT_EQ(2u, tSromMaterial.deterministicVars().size());
    ASSERT_STREQ("30", tSromMaterial.materialID().c_str());
    ASSERT_STREQ("isotropic", tSromMaterial.category().c_str());
    ASSERT_STREQ("poissons ratio", tSromMaterial.tags()[0].c_str());
    ASSERT_STREQ("youngs modulus", tSromMaterial.tags()[1].c_str());

    ASSERT_STREQ("0.3", tSromMaterial.deterministicVars()[0].value().c_str());
    ASSERT_STREQ("poissons ratio", tSromMaterial.deterministicVars()[0].tag().c_str());
    ASSERT_STREQ("homogeneous", tSromMaterial.deterministicVars()[0].attribute().c_str());
    ASSERT_STREQ("0.5", tSromMaterial.deterministicVars()[1].value().c_str());
    ASSERT_STREQ("youngs modulus", tSromMaterial.deterministicVars()[1].tag().c_str());
    ASSERT_STREQ("homogeneous", tSromMaterial.deterministicVars()[1].attribute().c_str());
}

TEST(PlatoTestXMLGenerator, AppendRandomMaterial_RandomMaterialCase)
{
    // 1. BUILD RANDOM MATERIAL MAP
    XMLGen::Uncertainty tCase1;
    tCase1.id = "3";
    tCase1.axis = "homogeneous";
    tCase1.type = "poissons ratio";
    tCase1.variable_type = "material";
    tCase1.distribution = "beta";
    tCase1.lower = "0.2";
    tCase1.upper = "0.35";
    tCase1.mean = "0.27";
    tCase1.num_samples = "6";
    tCase1.standard_deviation = "0.05";

    XMLGen::Uncertainty tCase2;
    tCase2.id = "30";
    tCase2.axis = "homogeneous";
    tCase2.type = "youngs modulus";
    tCase2.variable_type = "material";
    tCase2.distribution = "beta";
    tCase2.lower = "1";
    tCase2.upper = "2";
    tCase2.mean = "0.2";
    tCase2.num_samples = "8";
    tCase2.standard_deviation = "0.15";

    std::vector<XMLGen::Uncertainty> tRandomVars;
    tRandomVars.push_back(tCase1);
    tRandomVars.push_back(tCase2);
    auto tRandomMaterialMap = Plato::srom::build_material_id_to_random_material_map(tRandomVars);

    // 2. BUILD MATERIAL METADATA
    XMLGen::Material tMaterial;
    tMaterial.id("30");
    tMaterial.category("isotropic");
    tMaterial.property("youngs modulus", "0.5");
    tMaterial.property("poissons ratio", "0.3");

    // 3. APPEND MATERIAL TO SROM MATERIAL METADATA
    Plato::srom::Material tSromMaterial;
    EXPECT_NO_THROW(Plato::srom::append_material_properties(tMaterial, tRandomMaterialMap, tSromMaterial));

    // 4. TEST RESULTS
    ASSERT_TRUE(tSromMaterial.isRandom());
    ASSERT_FALSE(tSromMaterial.isDeterministic());
    ASSERT_EQ(1u, tSromMaterial.randomVars().size());
    ASSERT_EQ(1u, tSromMaterial.deterministicVars().size());
    ASSERT_STREQ("30", tSromMaterial.materialID().c_str());
    ASSERT_STREQ("isotropic", tSromMaterial.category().c_str());
    ASSERT_STREQ("youngs modulus", tSromMaterial.tags()[0].c_str());
    ASSERT_STREQ("poissons ratio", tSromMaterial.tags()[1].c_str());

    ASSERT_STREQ("0.3", tSromMaterial.deterministicVars()[0].value().c_str());
    ASSERT_STREQ("poissons ratio", tSromMaterial.deterministicVars()[0].tag().c_str());
    ASSERT_STREQ("homogeneous", tSromMaterial.deterministicVars()[0].attribute().c_str());

    ASSERT_EQ(0, tSromMaterial.randomVars()[0].id());
    ASSERT_TRUE(tSromMaterial.randomVars()[0].file().empty());
    ASSERT_STREQ("0.2", tSromMaterial.randomVars()[0].mean().c_str());
    ASSERT_STREQ("2", tSromMaterial.randomVars()[0].upper().c_str());
    ASSERT_STREQ("1", tSromMaterial.randomVars()[0].lower().c_str());
    ASSERT_STREQ("8", tSromMaterial.randomVars()[0].samples().c_str());
    ASSERT_STREQ("0.15", tSromMaterial.randomVars()[0].deviation().c_str());
    ASSERT_STREQ("beta", tSromMaterial.randomVars()[0].distribution().c_str());
    ASSERT_STREQ("youngs modulus", tSromMaterial.randomVars()[0].tag().c_str());
    ASSERT_STREQ("homogeneous", tSromMaterial.randomVars()[0].attribute().c_str());
}

TEST(PlatoTestXMLGenerator, BuildMaterialIdToRandomMaterialMap_Error)
{
    XMLGen::Uncertainty tCase1;
    tCase1.id = "3";
    tCase1.axis = "homogeneous";
    tCase1.type = "poissons ratio";
    tCase1.variable_type = "material";
    tCase1.distribution = "beta";
    tCase1.lower = "0.2";
    tCase1.upper = "0.35";
    tCase1.mean = "0.27";
    tCase1.num_samples = "6";
    tCase1.standard_deviation = "0.05";

    XMLGen::Uncertainty tCase2;
    tCase2.variable_type = "load";
    tCase2.axis = "y";
    tCase2.distribution = "beta";
    tCase2.id = "20";
    tCase2.lower = "-20";
    tCase2.upper = "20";
    tCase2.mean = "2";
    tCase2.standard_deviation = "2";
    tCase2.num_samples = "8";
    tCase2.type = "angle variation";

    std::vector<XMLGen::Uncertainty> tRandomVars;
    tRandomVars.push_back(tCase1);
    tRandomVars.push_back(tCase2);

    EXPECT_THROW(Plato::srom::build_material_id_to_random_material_map(tRandomVars), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, BuildMaterialIdToRandomMaterialMap)
{
    XMLGen::Uncertainty tCase1;
    tCase1.id = "3";
    tCase1.axis = "homogeneous";
    tCase1.type = "poissons ratio";
    tCase1.variable_type = "material";
    tCase1.distribution = "beta";
    tCase1.lower = "0.2";
    tCase1.upper = "0.35";
    tCase1.mean = "0.27";
    tCase1.num_samples = "6";
    tCase1.standard_deviation = "0.05";

    XMLGen::Uncertainty tCase2;
    tCase2.id = "30";
    tCase2.axis = "homogeneous";
    tCase2.type = "youngs modulus";
    tCase2.variable_type = "material";
    tCase2.distribution = "beta";
    tCase2.lower = "1";
    tCase2.upper = "2";
    tCase2.mean = "0.2";
    tCase2.num_samples = "8";
    tCase2.standard_deviation = "0.15";

    std::vector<XMLGen::Uncertainty> tRandomVars;
    tRandomVars.push_back(tCase1);
    tRandomVars.push_back(tCase2);

    auto tRandomVarMap = Plato::srom::build_material_id_to_random_material_map(tRandomVars);
    ASSERT_EQ(2u, tRandomVarMap.size());
    std::vector<std::string> tGoldIDs = {"3", "30"};
    std::vector<std::string> tGoldTags = {"poissons ratio", "youngs modulus"};
    auto tGoldIdIterator = tGoldIDs.begin();
    auto tGoldTagIterator = tGoldTags.begin();
    for(auto& tRandomVar : tRandomVarMap)
    {
        ASSERT_STREQ(tGoldIdIterator->c_str(), tRandomVar.first.c_str());
        ASSERT_STREQ(tGoldTagIterator->c_str(), tRandomVar.second.begin()->first.c_str());
        std::advance(tGoldIdIterator, 1);
        std::advance(tGoldTagIterator, 1);
    }

    // TEST MATERIAL 1
    ASSERT_TRUE(tRandomVarMap.find("3")->second.find("poissons ratio")->second.file.empty());
    ASSERT_STREQ("3", tRandomVarMap.find("3")->second.find("poissons ratio")->second.id.c_str());
    ASSERT_STREQ("0.27", tRandomVarMap.find("3")->second.find("poissons ratio")->second.mean.c_str());
    ASSERT_STREQ("0.2", tRandomVarMap.find("3")->second.find("poissons ratio")->second.lower.c_str());
    ASSERT_STREQ("0.35", tRandomVarMap.find("3")->second.find("poissons ratio")->second.upper.c_str());
    ASSERT_STREQ("6", tRandomVarMap.find("3")->second.find("poissons ratio")->second.num_samples.c_str());
    ASSERT_STREQ("homogeneous", tRandomVarMap.find("3")->second.find("poissons ratio")->second.axis.c_str());
    ASSERT_STREQ("beta", tRandomVarMap.find("3")->second.find("poissons ratio")->second.distribution.c_str());
    ASSERT_STREQ("material", tRandomVarMap.find("3")->second.find("poissons ratio")->second.variable_type.c_str());
    ASSERT_STREQ("0.05", tRandomVarMap.find("3")->second.find("poissons ratio")->second.standard_deviation.c_str());

    // TEST MATERIAL 2
    ASSERT_TRUE(tRandomVarMap.find("30")->second.find("youngs modulus")->second.file.empty());
    ASSERT_STREQ("30", tRandomVarMap.find("30")->second.find("youngs modulus")->second.id.c_str());
    ASSERT_STREQ("0.2", tRandomVarMap.find("30")->second.find("youngs modulus")->second.mean.c_str());
    ASSERT_STREQ("1", tRandomVarMap.find("30")->second.find("youngs modulus")->second.lower.c_str());
    ASSERT_STREQ("2", tRandomVarMap.find("30")->second.find("youngs modulus")->second.upper.c_str());
    ASSERT_STREQ("8", tRandomVarMap.find("30")->second.find("youngs modulus")->second.num_samples.c_str());
    ASSERT_STREQ("homogeneous", tRandomVarMap.find("30")->second.find("youngs modulus")->second.axis.c_str());
    ASSERT_STREQ("beta", tRandomVarMap.find("30")->second.find("youngs modulus")->second.distribution.c_str());
    ASSERT_STREQ("material", tRandomVarMap.find("30")->second.find("youngs modulus")->second.variable_type.c_str());
    ASSERT_STREQ("0.15", tRandomVarMap.find("30")->second.find("youngs modulus")->second.standard_deviation.c_str());
}

TEST(PlatoTestXMLGenerator, SplitUncertaintiesIntoCategories_Error1)
{
    // EMPTY UNCERTAINTY LIST
    XMLGen::InputData tMetadata;
    EXPECT_THROW(Plato::srom::split_uncertainties_into_categories(tMetadata), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, SplitUncertaintiesIntoCategories_Error2)
{
    // UNSUPPORTED CATEGORY, I.E. USE CASE
    XMLGen::InputData tMetadata;
    tMetadata.uncertainties.push_back(XMLGen::Uncertainty());
    tMetadata.uncertainties[0].variable_type = "boundary condition";
    EXPECT_THROW(Plato::srom::split_uncertainties_into_categories(tMetadata), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, SplitUncertaintiesIntoCategories)
{
    // 1. POSE DATA
    XMLGen::Uncertainty tCase1;
    tCase1.variable_type = "load";
    tCase1.axis = "x";
    tCase1.distribution = "beta";
    tCase1.id = "2";
    tCase1.lower = "-2";
    tCase1.upper = "2";
    tCase1.mean = "0";
    tCase1.standard_deviation = "0.2";
    tCase1.num_samples = "12";
    tCase1.type = "angle variation";

    XMLGen::Uncertainty tCase2;
    tCase2.variable_type = "load";
    tCase2.axis = "y";
    tCase2.distribution = "beta";
    tCase2.id = "20";
    tCase2.lower = "-20";
    tCase2.upper = "20";
    tCase2.mean = "2";
    tCase2.standard_deviation = "2";
    tCase2.num_samples = "8";
    tCase2.type = "angle variation";

    XMLGen::Uncertainty tCase3;
    tCase3.variable_type = "material";
    tCase3.axis = "homogeneous";
    tCase3.distribution = "beta";
    tCase3.id = "3";
    tCase3.lower = "0.2";
    tCase3.upper = "0.35";
    tCase3.mean = "0.27";
    tCase3.standard_deviation = "0.05";
    tCase3.num_samples = "6";
    tCase3.type = "poissons ratio";

    XMLGen::InputData tMetadata;
    tMetadata.uncertainties.push_back(tCase1);
    tMetadata.uncertainties.push_back(tCase2);
    tMetadata.uncertainties.push_back(tCase3);

    auto tCategoriesMap = Plato::srom::split_uncertainties_into_categories(tMetadata);
    ASSERT_EQ(2u, tCategoriesMap.size());

    // 2.1 TEST LOADS
    auto tLoads = tCategoriesMap.find(Plato::srom::category::LOAD);
    ASSERT_EQ(2u, tLoads->second.size());
    ASSERT_STREQ("load", tLoads->second[0].variable_type.c_str());
    ASSERT_STREQ("angle variation", tLoads->second[0].type.c_str());
    ASSERT_STREQ("x", tLoads->second[0].axis.c_str());
    ASSERT_STREQ("2", tLoads->second[0].id.c_str());
    ASSERT_STREQ("2", tLoads->second[0].upper.c_str());
    ASSERT_STREQ("-2", tLoads->second[0].lower.c_str());
    ASSERT_STREQ("0", tLoads->second[0].mean.c_str());
    ASSERT_STREQ("0.2", tLoads->second[0].standard_deviation.c_str());
    ASSERT_STREQ("12", tLoads->second[0].num_samples.c_str());
    ASSERT_STREQ("beta", tLoads->second[0].distribution.c_str());

    ASSERT_STREQ("load", tLoads->second[1].variable_type.c_str());
    ASSERT_STREQ("angle variation", tLoads->second[1].type.c_str());
    ASSERT_STREQ("y", tLoads->second[1].axis.c_str());
    ASSERT_STREQ("20", tLoads->second[1].id.c_str());
    ASSERT_STREQ("20", tLoads->second[1].upper.c_str());
    ASSERT_STREQ("-20", tLoads->second[1].lower.c_str());
    ASSERT_STREQ("2", tLoads->second[1].mean.c_str());
    ASSERT_STREQ("2", tLoads->second[1].standard_deviation.c_str());
    ASSERT_STREQ("8", tLoads->second[1].num_samples.c_str());
    ASSERT_STREQ("beta", tLoads->second[1].distribution.c_str());

    // 2.2 TEST MATERIALS
    auto tMaterials = tCategoriesMap.find(Plato::srom::category::MATERIAL);
    ASSERT_EQ(1u, tMaterials->second.size());
    ASSERT_STREQ("material", tMaterials->second[0].variable_type.c_str());
    ASSERT_STREQ("poissons ratio", tMaterials->second[0].type.c_str());
    ASSERT_STREQ("homogeneous", tMaterials->second[0].axis.c_str());
    ASSERT_STREQ("3", tMaterials->second[0].id.c_str());
    ASSERT_STREQ("0.35", tMaterials->second[0].upper.c_str());
    ASSERT_STREQ("0.2", tMaterials->second[0].lower.c_str());
    ASSERT_STREQ("0.27", tMaterials->second[0].mean.c_str());
    ASSERT_STREQ("0.05", tMaterials->second[0].standard_deviation.c_str());
    ASSERT_STREQ("6", tMaterials->second[0].num_samples.c_str());
    ASSERT_STREQ("beta", tMaterials->second[0].distribution.c_str());
}

TEST(PlatoTestXMLGenerator, PreprocessNondeterministicLoadInputs_Error)
{
    // Generate interface.xml using the new writer
    XMLGenerator_UnitTester tTester;
    std::istringstream tInputSS;
    std::string tStringInput =
        "begin objective\n"
        "   type maximize stiffness\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1\n"
        "   analyze new workflow true\n"
        "   number ranks 1\n"
        "end objective\n"
        "begin boundary conditions\n"
        "   fixed displacement nodeset name 1 bc id 11\n"
        "end boundary conditions\n"
        "begin loads\n"
        "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
        "end loads\n"
        "begin material 1\n"
            "penalty exponent 3\n"
            "youngs modulus 1e6\n"
            "poissons ratio 0.33\n"
            "thermal conductivity .02\n"
            "density .001\n"
        "end material\n"
        "begin block 1\n"
        "   material 1\n"
        "end block\n"
        "begin optimization parameters\n"
        "end optimization parameters\n";

    tInputSS.str(tStringInput);
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseObjectives(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBCs(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));
    auto tInputData = tTester.getInputData();

    Plato::srom::InputMetaData tSromInputs;
    EXPECT_THROW(Plato::srom::preprocess_load_inputs(tInputData, tSromInputs), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, PreprocessNondeterministicLoadInputs)
{
    // Generate interface.xml using the new writer
    XMLGenerator_UnitTester tTester;
    std::istringstream tInputSS;
    std::string tStringInput =
        "begin objective\n"
        "   type maximize stiffness\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1\n"
        "   analyze new workflow true\n"
        "   number ranks 1\n"
        "end objective\n"
        "begin boundary conditions\n"
        "   fixed displacement nodeset name 1 bc id 11\n"
        "end boundary conditions\n"
        "begin loads\n"
        "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
        "end loads\n"
        "begin material 1\n"
            "penalty exponent 3\n"
            "youngs modulus 1e6\n"
            "poissons ratio 0.33\n"
            "thermal conductivity .02\n"
            "density .001\n"
        "end material\n"
        "begin block 1\n"
        "   material 1\n"
        "end block\n"
        "begin uncertainty\n"
        "    tag angle variation\n"
        "    load id 10\n"
        "    attribute X\n"
        "    distribution beta\n"
        "    mean 0.0\n"
        "    upper bound 45.0\n"
        "    lower bound -45.0\n"
        "    standard deviation 22.5\n"
        "    num samples 2\n"
        "end uncertainty\n"
        "begin optimization parameters\n"
        "end optimization parameters\n";

    tInputSS.str(tStringInput);
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseObjectives(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBCs(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseUncertainties(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));
    auto tInputData = tTester.getInputData();

    Plato::srom::InputMetaData tSromInputs;
    Plato::srom::preprocess_load_inputs(tInputData, tSromInputs);

    // 1 TEST RESULTS
    auto tLoads = tSromInputs.loads();
    auto tMaterials = tSromInputs.materials();
    ASSERT_FALSE(tLoads.empty());
    ASSERT_TRUE(tMaterials.empty());

    // 1.1. TEST INTEGERS
    ASSERT_EQ(1u, tLoads.size());
    ASSERT_EQ(2147483647, tLoads[0].mAppID);
    ASSERT_STREQ("2", tLoads[0].mAppName.c_str());

    // 1.2. TEST STRINGS
    ASSERT_STREQ("2", tLoads[0].mAppName.c_str());
    ASSERT_STREQ("sideset", tLoads[0].mAppType.c_str());
    ASSERT_STREQ("10", tLoads[0].mLoadID.c_str());
    ASSERT_STREQ("traction", tLoads[0].mLoadType.c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[0].c_str());
    ASSERT_STREQ("-5e4", tLoads[0].mValues[1].c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[2].c_str());

    // 1.3. TEST STATISTICS
    ASSERT_FALSE(tLoads[0].mRandomVars.empty());
    ASSERT_EQ(1u, tLoads[0].mRandomVars.size());
    ASSERT_EQ(0, tLoads[0].mRandomVars[0].id());

    ASSERT_STREQ("angle variation", tLoads[0].mRandomVars[0].tag().c_str());
    ASSERT_STREQ("x", tLoads[0].mRandomVars[0].attribute().c_str());
    ASSERT_STREQ("22.5", tLoads[0].mRandomVars[0].deviation().c_str());
    ASSERT_STREQ("beta", tLoads[0].mRandomVars[0].distribution().c_str());
    ASSERT_STREQ("-45.0", tLoads[0].mRandomVars[0].lower().c_str());
    ASSERT_STREQ("45.0", tLoads[0].mRandomVars[0].upper().c_str());
    ASSERT_STREQ("0.0", tLoads[0].mRandomVars[0].mean().c_str());
    ASSERT_STREQ("2", tLoads[0].mRandomVars[0].samples().c_str());
}

TEST(PlatoTestXMLGenerator, PreprocessSromProblemInputs_Error)
{
    // Generate interface.xml using the new writer
    XMLGenerator_UnitTester tTester;
    std::istringstream tInputSS;
    std::string tStringInput =
        "begin objective\n"
        "   type maximize stiffness\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1\n"
        "   analyze new workflow true\n"
        "   number ranks 1\n"
        "end objective\n"
        "begin boundary conditions\n"
        "   fixed displacement nodeset name 1 bc id 11\n"
        "end boundary conditions\n"
        "begin loads\n"
        "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
        "end loads\n"
        "begin material 1\n"
            "penalty exponent 3\n"
            "youngs modulus 1e6\n"
            "poissons ratio 0.33\n"
            "thermal conductivity .02\n"
            "density .001\n"
        "end material\n"
        "begin block 1\n"
        "   material 1\n"
        "end block\n"
        "begin uncertainty\n"
        "    tag angle variation\n"
        "    load id 10\n"
        "    attribute X\n"
        "    distribution beta\n"
        "    mean 0.0\n"
        "    upper bound 45.0\n"
        "    lower bound -45.0\n"
        "    standard deviation 22.5\n"
        "    num samples 2\n"
        "end uncertainty\n"
        "begin optimization parameters\n"
        "end optimization parameters\n";

    tInputSS.str(tStringInput);
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseObjectives(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBCs(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseUncertainties(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));
    auto tInputData = tTester.getInputData();

    // CALL FUNCTION
    Plato::srom::InputMetaData tSromInputs;
    tSromInputs.usecase(Plato::srom::usecase::UNDEFINED);
    EXPECT_THROW(Plato::srom::preprocess_srom_problem_inputs(tInputData, tSromInputs), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, PreprocessSromProblemInputs_Loads)
{
    // Generate interface.xml using the new writer
    XMLGenerator_UnitTester tTester;
    std::istringstream tInputSS;
    std::string tStringInput =
        "begin objective\n"
        "   type maximize stiffness\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1\n"
        "   analyze new workflow true\n"
        "   number ranks 1\n"
        "end objective\n"
        "begin boundary conditions\n"
        "   fixed displacement nodeset name 1 bc id 11\n"
        "end boundary conditions\n"
        "begin loads\n"
        "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
        "end loads\n"
        "begin material 1\n"
            "penalty exponent 3\n"
            "youngs modulus 1e6\n"
            "poissons ratio 0.33\n"
            "thermal conductivity .02\n"
            "density .001\n"
        "end material\n"
        "begin block 1\n"
        "   material 1\n"
        "end block\n"
        "begin uncertainty\n"
        "    tag angle variation\n"
        "    load id 10\n"
        "    attribute X\n"
        "    distribution beta\n"
        "    mean 0.0\n"
        "    upper bound 45.0\n"
        "    lower bound -45.0\n"
        "    standard deviation 22.5\n"
        "    num samples 2\n"
        "end uncertainty\n"
        "begin optimization parameters\n"
        "end optimization parameters\n";

    tInputSS.str(tStringInput);
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseObjectives(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBCs(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseUncertainties(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));
    auto tInputData = tTester.getInputData();

    // CALL FUNCTION
    Plato::srom::InputMetaData tSromInputs;
    tSromInputs.usecase(Plato::srom::usecase::LOAD);
    Plato::srom::preprocess_srom_problem_inputs(tInputData, tSromInputs);

    // 1 TEST RESULTS
    auto tLoads = tSromInputs.loads();
    auto tMaterials = tSromInputs.materials();
    ASSERT_FALSE(tLoads.empty());
    ASSERT_TRUE(tMaterials.empty());

    // 1.1. TEST INTEGERS
    ASSERT_EQ(1u, tLoads.size());
    ASSERT_EQ(2147483647, tLoads[0].mAppID); // not defined, thus it takes the value std::numeric_limit::max
    ASSERT_STREQ("2", tLoads[0].mAppName.c_str());

    // 1.2. TEST STRINGS
    ASSERT_STREQ("2", tLoads[0].mAppName.c_str());
    ASSERT_STREQ("sideset", tLoads[0].mAppType.c_str());
    ASSERT_STREQ("10", tLoads[0].mLoadID.c_str());
    ASSERT_STREQ("traction", tLoads[0].mLoadType.c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[0].c_str());
    ASSERT_STREQ("-5e4", tLoads[0].mValues[1].c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[2].c_str());

    // 1.3. TEST STATISTICS
    ASSERT_FALSE(tLoads[0].mRandomVars.empty());
    ASSERT_EQ(1u, tLoads[0].mRandomVars.size());
    ASSERT_EQ(0, tLoads[0].mRandomVars[0].id());

    ASSERT_STREQ("angle variation", tLoads[0].mRandomVars[0].tag().c_str());
    ASSERT_STREQ("x", tLoads[0].mRandomVars[0].attribute().c_str());
    ASSERT_STREQ("22.5", tLoads[0].mRandomVars[0].deviation().c_str());
    ASSERT_STREQ("beta", tLoads[0].mRandomVars[0].distribution().c_str());
    ASSERT_STREQ("-45.0", tLoads[0].mRandomVars[0].lower().c_str());
    ASSERT_STREQ("45.0", tLoads[0].mRandomVars[0].upper().c_str());
    ASSERT_STREQ("0.0", tLoads[0].mRandomVars[0].mean().c_str());
    ASSERT_STREQ("2", tLoads[0].mRandomVars[0].samples().c_str());
}

TEST(PlatoTestXMLGenerator, PreprocessSromProblemInputs_Materials)
{
    // Generate interface.xml using the new writer
    XMLGenerator_UnitTester tTester;
    std::istringstream tInputSS;
    std::string tStringInput =
        "begin objective\n"
        "   type maximize stiffness\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1\n"
        "   analyze new workflow true\n"
        "   number ranks 1\n"
        "end objective\n"
        "begin boundary conditions\n"
        "   fixed displacement nodeset name 1 bc id 11\n"
        "end boundary conditions\n"
        "begin loads\n"
        "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
        "end loads\n"
        "begin material 1\n"
            "penalty exponent 3\n"
            "youngs modulus 1e6\n"
            "poissons ratio 0.33\n"
            "thermal conductivity .02\n"
            "density .001\n"
        "end material\n"
        "begin block 1\n"
        "   material 1\n"
        "end block\n"
        "begin uncertainty\n"
        "    category material\n"
        "    tag thermal conductivity\n"
        "    material id 1\n"
        "    attribute homogeneous\n"
        "    distribution beta\n"
        "    mean 0.02\n"
        "    upper bound 0.04\n"
        "    lower bound 0.01\n"
        "    standard deviation 0.0075\n"
        "    num samples 8\n"
        "end uncertainty\n"
        "begin optimization parameters\n"
        "end optimization parameters\n";

    tInputSS.str(tStringInput);
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseObjectives(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseMaterials(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBCs(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseUncertainties(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));

    // CALL FUNCTION
    Plato::srom::InputMetaData tSromInputs;
    auto tInputData = tTester.getInputData();
    EXPECT_FALSE(tInputData.uncertainties.empty());
    tSromInputs.usecase(Plato::srom::usecase::MATERIAL);
    Plato::srom::preprocess_srom_problem_inputs(tInputData, tSromInputs);

    // 1 TEST RESULTS
    auto tLoads = tSromInputs.loads();
    auto tMaterials = tSromInputs.materials();
    ASSERT_TRUE(tLoads.empty());
    ASSERT_FALSE(tMaterials.empty());

    // 1.1. TEST INTEGERS
    ASSERT_EQ(1u, tMaterials.size());
    EXPECT_TRUE(tMaterials[0].isRandom());
    EXPECT_FALSE(tMaterials[0].isDeterministic());

    // 1.2. TEST STRINGS
    ASSERT_STREQ("1", tMaterials[0].blockID().c_str());
    ASSERT_STREQ("1", tMaterials[0].materialID().c_str());
    ASSERT_STREQ("isotropic", tMaterials[0].category().c_str());
    ASSERT_EQ(5u, tMaterials[0].tags().size());

    // 1.3. TEST STATISTICS
    ASSERT_FALSE(tMaterials[0].randomVars().empty());
    ASSERT_FALSE(tMaterials[0].deterministicVars().empty());
    ASSERT_EQ(1u, tMaterials[0].randomVars().size());
    ASSERT_EQ(4u, tMaterials[0].deterministicVars().size());
    ASSERT_EQ(0, tMaterials[0].randomVars()[0].id());

    ASSERT_STREQ("8", tMaterials[0].randomVars()[0].samples().c_str());
    ASSERT_STREQ("0.02", tMaterials[0].randomVars()[0].mean().c_str());
    ASSERT_STREQ("0.01", tMaterials[0].randomVars()[0].lower().c_str());
    ASSERT_STREQ("0.04", tMaterials[0].randomVars()[0].upper().c_str());
    ASSERT_STREQ("0.0075", tMaterials[0].randomVars()[0].deviation().c_str());
    ASSERT_STREQ("beta", tMaterials[0].randomVars()[0].distribution().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[0].randomVars()[0].attribute().c_str());
    ASSERT_STREQ("thermal conductivity", tMaterials[0].randomVars()[0].tag().c_str());
}

TEST(PlatoTestXMLGenerator, PreprocessSromProblemInputs_Materials_1RandomAnd1Deterministic)
{
    // Generate interface.xml using the new writer
    XMLGenerator_UnitTester tTester;
    std::istringstream tInputSS;
    std::string tStringInput =
        "begin objective\n"
        "   type maximize stiffness\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1\n"
        "   analyze new workflow true\n"
        "   number ranks 1\n"
        "end objective\n"
        "begin boundary conditions\n"
        "   fixed displacement nodeset name 1 bc id 11\n"
        "end boundary conditions\n"
        "begin loads\n"
        "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
        "end loads\n"
        "begin material 1\n"
            "penalty exponent 3\n"
            "youngs modulus 1e6\n"
            "poissons ratio 0.33\n"
            "thermal conductivity .02\n"
            "density .001\n"
        "end material\n"
        "begin material 2\n"
            "penalty exponent 3\n"
            "youngs modulus 5e6\n"
            "poissons ratio 0.28\n"
        "end material\n"
        "begin block 1\n"
        "   material 1\n"
        "end block\n"
        "begin block 2\n"
        "   material 2\n"
        "end block\n"
        "begin uncertainty\n"
        "    category material\n"
        "    tag thermal conductivity\n"
        "    material id 1\n"
        "    attribute homogeneous\n"
        "    distribution beta\n"
        "    mean 0.02\n"
        "    upper bound 0.04\n"
        "    lower bound 0.01\n"
        "    standard deviation 0.0075\n"
        "    num samples 8\n"
        "end uncertainty\n"
        "begin optimization parameters\n"
        "end optimization parameters\n";

    tInputSS.str(tStringInput);
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseObjectives(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseMaterials(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBCs(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseUncertainties(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));

    // CALL FUNCTION
    Plato::srom::InputMetaData tSromInputs;
    auto tInputData = tTester.getInputData();
    EXPECT_FALSE(tInputData.uncertainties.empty());
    tSromInputs.usecase(Plato::srom::usecase::MATERIAL);
    Plato::srom::preprocess_srom_problem_inputs(tInputData, tSromInputs);

    // 1 TEST RESULTS
    auto tLoads = tSromInputs.loads();
    auto tMaterials = tSromInputs.materials();
    ASSERT_TRUE(tLoads.empty());
    ASSERT_FALSE(tMaterials.empty());

    // 1.1. TEST INTEGERS
    ASSERT_EQ(2u, tMaterials.size());
    EXPECT_TRUE(tMaterials[0].isRandom());
    EXPECT_FALSE(tMaterials[0].isDeterministic());

    // 1.2. TEST STRINGS
    ASSERT_STREQ("1", tMaterials[0].blockID().c_str());
    ASSERT_STREQ("isotropic", tMaterials[0].category().c_str());
    ASSERT_STREQ("1", tMaterials[0].materialID().c_str());
    ASSERT_EQ(5u, tMaterials[0].tags().size());

    // 1.3. TEST STATISTICS
    ASSERT_FALSE(tMaterials[0].randomVars().empty());
    ASSERT_FALSE(tMaterials[0].deterministicVars().empty());
    ASSERT_EQ(1u, tMaterials[0].randomVars().size());
    ASSERT_EQ(4u, tMaterials[0].deterministicVars().size());
    ASSERT_EQ(0, tMaterials[0].randomVars()[0].id());

    ASSERT_STREQ("8", tMaterials[0].randomVars()[0].samples().c_str());
    ASSERT_STREQ("0.02", tMaterials[0].randomVars()[0].mean().c_str());
    ASSERT_STREQ("0.01", tMaterials[0].randomVars()[0].lower().c_str());
    ASSERT_STREQ("0.04", tMaterials[0].randomVars()[0].upper().c_str());
    ASSERT_STREQ("0.0075", tMaterials[0].randomVars()[0].deviation().c_str());
    ASSERT_STREQ("beta", tMaterials[0].randomVars()[0].distribution().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[0].randomVars()[0].attribute().c_str());
    ASSERT_STREQ("thermal conductivity", tMaterials[0].randomVars()[0].tag().c_str());

    // 2.1. TEST DETERMINISTIC MATERIAL
    EXPECT_FALSE(tMaterials[1].isRandom());
    EXPECT_TRUE(tMaterials[1].isDeterministic());

    // 2.2. TEST STRINGS
    ASSERT_STREQ("2", tMaterials[1].blockID().c_str());
    ASSERT_STREQ("isotropic", tMaterials[1].category().c_str());
    ASSERT_STREQ("2", tMaterials[1].materialID().c_str());
    ASSERT_EQ(3u, tMaterials[1].tags().size());

    ASSERT_STREQ("3", tMaterials[1].deterministicVars()[0].value().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[1].deterministicVars()[0].attribute().c_str());
    ASSERT_STREQ("penalty exponent", tMaterials[1].deterministicVars()[0].tag().c_str());

    ASSERT_STREQ("0.28", tMaterials[1].deterministicVars()[1].value().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[1].deterministicVars()[1].attribute().c_str());
    ASSERT_STREQ("poissons ratio", tMaterials[1].deterministicVars()[1].tag().c_str());

    ASSERT_STREQ("5e6", tMaterials[1].deterministicVars()[2].value().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[1].deterministicVars()[2].attribute().c_str());
    ASSERT_STREQ("youngs modulus", tMaterials[1].deterministicVars()[2].tag().c_str());
}

TEST(PlatoTestXMLGenerator, PreprocessSromProblemInputs_MaterialsPlusLoad)
{
    // Generate interface.xml using the new writer
    XMLGenerator_UnitTester tTester;
    std::istringstream tInputSS;
    std::string tStringInput =
        "begin objective\n"
        "   type maximize stiffness\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1\n"
        "   analyze new workflow true\n"
        "   number ranks 1\n"
        "end objective\n"
        "begin boundary conditions\n"
        "   fixed displacement nodeset name 1 bc id 11\n"
        "end boundary conditions\n"
        "begin loads\n"
        "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
        "end loads\n"
        "begin material 1\n"
            "penalty exponent 3\n"
            "youngs modulus 1e6\n"
            "poissons ratio 0.33\n"
            "thermal conductivity .02\n"
            "density .001\n"
        "end material\n"
        "begin block 1\n"
        "   material 1\n"
        "end block\n"
        "begin uncertainty\n"
        "    category material\n"
        "    tag thermal conductivity\n"
        "    material id 1\n"
        "    attribute homogeneous\n"
        "    distribution beta\n"
        "    mean 0.02\n"
        "    upper bound 0.04\n"
        "    lower bound 0.01\n"
        "    standard deviation 0.0075\n"
        "    num samples 8\n"
        "end uncertainty\n"
        "begin uncertainty\n"
        "    category load\n"
        "    tag angle variation\n"
        "    load id 10\n"
        "    attribute X\n"
        "    distribution beta\n"
        "    mean 0.0\n"
        "    upper bound 45.0\n"
        "    lower bound -45.0\n"
        "    standard deviation 22.5\n"
        "    num samples 2\n"
        "end uncertainty\n"
        "begin optimization parameters\n"
        "end optimization parameters\n";

    tInputSS.str(tStringInput);
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseObjectives(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseMaterials(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBCs(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseUncertainties(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));

    // CALL FUNCTION
    Plato::srom::InputMetaData tSromInputs;
    auto tInputData = tTester.getInputData();
    EXPECT_FALSE(tInputData.uncertainties.empty());
    EXPECT_EQ(2u, tInputData.uncertainties.size());
    tSromInputs.usecase(Plato::srom::usecase::MATERIAL_PLUS_LOAD);
    Plato::srom::preprocess_srom_problem_inputs(tInputData, tSromInputs);

    // 1 TEST RESULTS
    auto tLoads = tSromInputs.loads();
    auto tMaterials = tSromInputs.materials();
    ASSERT_FALSE(tLoads.empty());
    ASSERT_FALSE(tMaterials.empty());

    // 1.1. TEST MATERIAL INTEGERS
    ASSERT_EQ(1u, tMaterials.size());
    EXPECT_TRUE(tMaterials[0].isRandom());
    EXPECT_FALSE(tMaterials[0].isDeterministic());

    // 1.2. TEST MATERIAL STRINGS
    ASSERT_STREQ("isotropic", tMaterials[0].category().c_str());
    ASSERT_STREQ("1", tMaterials[0].materialID().c_str());
    ASSERT_EQ(5u, tMaterials[0].tags().size());

    // 1.3. TEST MATERIAL STATISTICS
    ASSERT_FALSE(tMaterials[0].randomVars().empty());
    ASSERT_FALSE(tMaterials[0].deterministicVars().empty());
    ASSERT_EQ(1u, tMaterials[0].randomVars().size());
    ASSERT_EQ(4u, tMaterials[0].deterministicVars().size());
    ASSERT_EQ(0, tMaterials[0].randomVars()[0].id());

    ASSERT_STREQ("8", tMaterials[0].randomVars()[0].samples().c_str());
    ASSERT_STREQ("0.02", tMaterials[0].randomVars()[0].mean().c_str());
    ASSERT_STREQ("0.01", tMaterials[0].randomVars()[0].lower().c_str());
    ASSERT_STREQ("0.04", tMaterials[0].randomVars()[0].upper().c_str());
    ASSERT_STREQ("0.0075", tMaterials[0].randomVars()[0].deviation().c_str());
    ASSERT_STREQ("beta", tMaterials[0].randomVars()[0].distribution().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[0].randomVars()[0].attribute().c_str());
    ASSERT_STREQ("thermal conductivity", tMaterials[0].randomVars()[0].tag().c_str());

    // 2.1. TEST LOAD INTEGERS
    ASSERT_EQ(1u, tLoads.size());
    ASSERT_EQ(2147483647, tLoads[0].mAppID); // not defined, thus it takes the value std::numeric_limit::max
    ASSERT_STREQ("2", tLoads[0].mAppName.c_str());

    // 2.2. TEST LOAD STRINGS
    ASSERT_STREQ("2", tLoads[0].mAppName.c_str());
    ASSERT_STREQ("sideset", tLoads[0].mAppType.c_str());
    ASSERT_STREQ("10", tLoads[0].mLoadID.c_str());
    ASSERT_STREQ("traction", tLoads[0].mLoadType.c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[0].c_str());
    ASSERT_STREQ("-5e4", tLoads[0].mValues[1].c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[2].c_str());

    // 2.3. TEST LOAD STATISTICS
    ASSERT_FALSE(tLoads[0].mRandomVars.empty());
    ASSERT_EQ(1u, tLoads[0].mRandomVars.size());
    ASSERT_EQ(0, tLoads[0].mRandomVars[0].id());

    ASSERT_STREQ("angle variation", tLoads[0].mRandomVars[0].tag().c_str());
    ASSERT_STREQ("x", tLoads[0].mRandomVars[0].attribute().c_str());
    ASSERT_STREQ("22.5", tLoads[0].mRandomVars[0].deviation().c_str());
    ASSERT_STREQ("beta", tLoads[0].mRandomVars[0].distribution().c_str());
    ASSERT_STREQ("-45.0", tLoads[0].mRandomVars[0].lower().c_str());
    ASSERT_STREQ("45.0", tLoads[0].mRandomVars[0].upper().c_str());
    ASSERT_STREQ("0.0", tLoads[0].mRandomVars[0].mean().c_str());
    ASSERT_STREQ("2", tLoads[0].mRandomVars[0].samples().c_str());
}

TEST(PlatoTestXMLGenerator, PreprocessSromProblemInputs_MaterialsPlusLoad_1RandomAnd1Deterministic)
{
    // Generate interface.xml using the new writer
    XMLGenerator_UnitTester tTester;
    std::istringstream tInputSS;
    std::string tStringInput =
        "begin objective\n"
        "   type maximize stiffness\n"
        "   load ids 1 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1\n"
        "   analyze new workflow true\n"
        "   number ranks 1\n"
        "end objective\n"
        "begin boundary conditions\n"
        "   fixed displacement nodeset name 1 bc id 11\n"
        "end boundary conditions\n"
        "begin loads\n"
        "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
        "    traction sideset name 3 value 0 -1e4 0 load id 1\n"
        "end loads\n"
        "begin material 1\n"
            "penalty exponent 3\n"
            "youngs modulus 1e6\n"
            "poissons ratio 0.33\n"
            "thermal conductivity .02\n"
            "density .001\n"
        "end material\n"
        "begin material 2\n"
            "penalty exponent 3\n"
            "youngs modulus 5e6\n"
            "poissons ratio 0.28\n"
        "end material\n"
        "begin block 1\n"
        "   material 1\n"
        "end block\n"
        "begin block 2\n"
        "   material 2\n"
        "end block\n"
        "begin uncertainty\n"
        "    category material\n"
        "    tag thermal conductivity\n"
        "    material id 1\n"
        "    attribute homogeneous\n"
        "    distribution beta\n"
        "    mean 0.02\n"
        "    upper bound 0.04\n"
        "    lower bound 0.01\n"
        "    standard deviation 0.0075\n"
        "    num samples 8\n"
        "end uncertainty\n"
        "begin uncertainty\n"
        "    category load\n"
        "    tag angle variation\n"
        "    load id 10\n"
        "    attribute X\n"
        "    distribution beta\n"
        "    mean 0.0\n"
        "    upper bound 45.0\n"
        "    lower bound -45.0\n"
        "    standard deviation 22.5\n"
        "    num samples 2\n"
        "end uncertainty\n"
        "begin optimization parameters\n"
        "end optimization parameters\n";

    tInputSS.str(tStringInput);
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseObjectives(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseMaterials(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBCs(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseUncertainties(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));

    // CALL FUNCTION
    Plato::srom::InputMetaData tSromInputs;
    auto tInputData = tTester.getInputData();
    EXPECT_FALSE(tInputData.uncertainties.empty());
    EXPECT_EQ(2u, tInputData.uncertainties.size());
    tSromInputs.usecase(Plato::srom::usecase::MATERIAL_PLUS_LOAD);
    Plato::srom::preprocess_srom_problem_inputs(tInputData, tSromInputs);

    // 1 TEST RESULTS
    auto tLoads = tSromInputs.loads();
    auto tMaterials = tSromInputs.materials();
    ASSERT_FALSE(tLoads.empty());
    ASSERT_FALSE(tMaterials.empty());

    // 1.1. TEST MATERIAL INTEGERS
    ASSERT_EQ(2u, tMaterials.size());
    EXPECT_TRUE(tMaterials[0].isRandom());
    EXPECT_FALSE(tMaterials[0].isDeterministic());

    // 1.2. TEST MATERIAL STRINGS
    ASSERT_STREQ("1", tMaterials[0].blockID().c_str());
    ASSERT_STREQ("1", tMaterials[0].materialID().c_str());
    ASSERT_STREQ("isotropic", tMaterials[0].category().c_str());
    ASSERT_EQ(5u, tMaterials[0].tags().size());

    // 1.3. TEST MATERIAL STATISTICS
    ASSERT_FALSE(tMaterials[0].randomVars().empty());
    ASSERT_FALSE(tMaterials[0].deterministicVars().empty());
    ASSERT_EQ(1u, tMaterials[0].randomVars().size());
    ASSERT_EQ(4u, tMaterials[0].deterministicVars().size());
    ASSERT_EQ(0, tMaterials[0].randomVars()[0].id());

    ASSERT_STREQ("8", tMaterials[0].randomVars()[0].samples().c_str());
    ASSERT_STREQ("0.02", tMaterials[0].randomVars()[0].mean().c_str());
    ASSERT_STREQ("0.01", tMaterials[0].randomVars()[0].lower().c_str());
    ASSERT_STREQ("0.04", tMaterials[0].randomVars()[0].upper().c_str());
    ASSERT_STREQ("0.0075", tMaterials[0].randomVars()[0].deviation().c_str());
    ASSERT_STREQ("beta", tMaterials[0].randomVars()[0].distribution().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[0].randomVars()[0].attribute().c_str());
    ASSERT_STREQ("thermal conductivity", tMaterials[0].randomVars()[0].tag().c_str());

    // 1.4 TEST DETERMINISTIC MATERIAL
    EXPECT_FALSE(tMaterials[1].isRandom());
    EXPECT_TRUE(tMaterials[1].isDeterministic());

    ASSERT_STREQ("2", tMaterials[1].blockID().c_str());
    ASSERT_STREQ("2", tMaterials[1].materialID().c_str());
    ASSERT_STREQ("isotropic", tMaterials[1].category().c_str());
    ASSERT_EQ(3u, tMaterials[1].tags().size());

    ASSERT_STREQ("3", tMaterials[1].deterministicVars()[0].value().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[1].deterministicVars()[0].attribute().c_str());
    ASSERT_STREQ("penalty exponent", tMaterials[1].deterministicVars()[0].tag().c_str());

    ASSERT_STREQ("0.28", tMaterials[1].deterministicVars()[1].value().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[1].deterministicVars()[1].attribute().c_str());
    ASSERT_STREQ("poissons ratio", tMaterials[1].deterministicVars()[1].tag().c_str());

    ASSERT_STREQ("5e6", tMaterials[1].deterministicVars()[2].value().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[1].deterministicVars()[2].attribute().c_str());
    ASSERT_STREQ("youngs modulus", tMaterials[1].deterministicVars()[2].tag().c_str());

    // 2.1. TEST LOAD INTEGERS
    ASSERT_EQ(2u, tLoads.size());
    ASSERT_EQ(2147483647, tLoads[0].mAppID); // not defined, thus it takes the value std::numeric_limit::max
    ASSERT_STREQ("2", tLoads[0].mAppName.c_str());

    // 2.2. TEST LOAD STRINGS
    ASSERT_STREQ("2", tLoads[0].mAppName.c_str());
    ASSERT_STREQ("sideset", tLoads[0].mAppType.c_str());
    ASSERT_STREQ("10", tLoads[0].mLoadID.c_str());
    ASSERT_STREQ("traction", tLoads[0].mLoadType.c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[0].c_str());
    ASSERT_STREQ("-5e4", tLoads[0].mValues[1].c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[2].c_str());

    // 2.3. TEST LOAD STATISTICS
    ASSERT_FALSE(tLoads[0].mRandomVars.empty());
    ASSERT_EQ(1u, tLoads[0].mRandomVars.size());
    ASSERT_EQ(0, tLoads[0].mRandomVars[0].id());

    ASSERT_STREQ("angle variation", tLoads[0].mRandomVars[0].tag().c_str());
    ASSERT_STREQ("x", tLoads[0].mRandomVars[0].attribute().c_str());
    ASSERT_STREQ("22.5", tLoads[0].mRandomVars[0].deviation().c_str());
    ASSERT_STREQ("beta", tLoads[0].mRandomVars[0].distribution().c_str());
    ASSERT_STREQ("-45.0", tLoads[0].mRandomVars[0].lower().c_str());
    ASSERT_STREQ("45.0", tLoads[0].mRandomVars[0].upper().c_str());
    ASSERT_STREQ("0.0", tLoads[0].mRandomVars[0].mean().c_str());
    ASSERT_STREQ("2", tLoads[0].mRandomVars[0].samples().c_str());

    // 2.4 TEST DETERMINISTIC LOAD
    ASSERT_STREQ("1", tLoads[1].mLoadID.c_str());
    ASSERT_STREQ("3", tLoads[1].mAppName.c_str());
    ASSERT_STREQ("sideset", tLoads[1].mAppType.c_str());
    ASSERT_STREQ("traction", tLoads[1].mLoadType.c_str());
    ASSERT_STREQ("0", tLoads[1].mValues[0].c_str());
    ASSERT_STREQ("-1e4", tLoads[1].mValues[1].c_str());
    ASSERT_STREQ("0", tLoads[1].mValues[2].c_str());
    ASSERT_EQ(std::numeric_limits<int>::max(), tLoads[1].mAppID);
    ASSERT_TRUE(tLoads[1].mRandomVars.empty());
}

TEST(PlatoTestXMLGenerator, PreprocessSromProblemInputs_Loads_1RandomAnd1Deterministic)
{
    // Generate interface.xml using the new writer
    XMLGenerator_UnitTester tTester;
    std::istringstream tInputSS;
    std::string tStringInput =
        "begin objective\n"
        "   type maximize stiffness\n"
        "   load ids 1 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1\n"
        "   analyze new workflow true\n"
        "   number ranks 1\n"
        "end objective\n"
        "begin boundary conditions\n"
        "   fixed displacement nodeset name 1 bc id 11\n"
        "end boundary conditions\n"
        "begin loads\n"
        "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
        "    traction sideset name 3 value 0 -1e4 0 load id 1\n"
        "end loads\n"
        "begin material 1\n"
            "penalty exponent 3\n"
            "youngs modulus 1e6\n"
            "poissons ratio 0.33\n"
            "thermal conductivity .02\n"
            "density .001\n"
        "end material\n"
        "begin block 1\n"
        "   material 1\n"
        "end block\n"
        "begin uncertainty\n"
        "    tag angle variation\n"
        "    load id 10\n"
        "    attribute X\n"
        "    distribution beta\n"
        "    mean 0.0\n"
        "    upper bound 45.0\n"
        "    lower bound -45.0\n"
        "    standard deviation 22.5\n"
        "    num samples 2\n"
        "end uncertainty\n"
        "begin optimization parameters\n"
        "end optimization parameters\n";

    tInputSS.str(tStringInput);
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseObjectives(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBCs(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseUncertainties(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));

    auto tInputData = tTester.getInputData();
    ASSERT_EQ(2u, tInputData.load_cases.size());
    for(auto& tLoadCase : tInputData.load_cases)
    {
        ASSERT_EQ(1u, tLoadCase.loads.size());
    }

    // CALL FUNCTION
    Plato::srom::InputMetaData tSromInputs;
    tSromInputs.usecase(Plato::srom::usecase::LOAD);
    Plato::srom::preprocess_srom_problem_inputs(tInputData, tSromInputs);

    // 1 TEST RANDOM LOAD
    auto tLoads = tSromInputs.loads();
    auto tMaterials = tSromInputs.materials();
    ASSERT_FALSE(tLoads.empty());
    ASSERT_TRUE(tMaterials.empty());

    // 1.1. TEST INTEGERS
    ASSERT_EQ(2u, tLoads.size());
    ASSERT_EQ(std::numeric_limits<int>::max(), tLoads[0].mAppID);
    ASSERT_EQ(std::numeric_limits<int>::max(), tLoads[1].mAppID);
    ASSERT_STREQ("2", tLoads[0].mAppName.c_str());
    ASSERT_STREQ("3", tLoads[1].mAppName.c_str());

    // 1.2. TEST STRINGS - LOAD 1
    ASSERT_STREQ("2", tLoads[0].mAppName.c_str());
    ASSERT_STREQ("sideset", tLoads[0].mAppType.c_str());
    ASSERT_STREQ("10", tLoads[0].mLoadID.c_str());
    ASSERT_STREQ("traction", tLoads[0].mLoadType.c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[0].c_str());
    ASSERT_STREQ("-5e4", tLoads[0].mValues[1].c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[2].c_str());

    // 1.3. TEST STATISTICS - LOAD 1
    ASSERT_FALSE(tLoads[0].mRandomVars.empty());
    ASSERT_EQ(1u, tLoads[0].mRandomVars.size());
    ASSERT_EQ(0, tLoads[0].mRandomVars[0].id());

    ASSERT_STREQ("angle variation", tLoads[0].mRandomVars[0].tag().c_str());
    ASSERT_STREQ("x", tLoads[0].mRandomVars[0].attribute().c_str());
    ASSERT_STREQ("22.5", tLoads[0].mRandomVars[0].deviation().c_str());
    ASSERT_STREQ("beta", tLoads[0].mRandomVars[0].distribution().c_str());
    ASSERT_STREQ("-45.0", tLoads[0].mRandomVars[0].lower().c_str());
    ASSERT_STREQ("45.0", tLoads[0].mRandomVars[0].upper().c_str());
    ASSERT_STREQ("0.0", tLoads[0].mRandomVars[0].mean().c_str());
    ASSERT_STREQ("2", tLoads[0].mRandomVars[0].samples().c_str());

    // 2 TEST DETERMINISTIC LOAD
    ASSERT_STREQ("1", tLoads[1].mLoadID.c_str());
    ASSERT_STREQ("3", tLoads[1].mAppName.c_str());
    ASSERT_STREQ("sideset", tLoads[1].mAppType.c_str());
    ASSERT_STREQ("traction", tLoads[1].mLoadType.c_str());
    ASSERT_STREQ("0", tLoads[1].mValues[0].c_str());
    ASSERT_STREQ("-1e4", tLoads[1].mValues[1].c_str());
    ASSERT_STREQ("0", tLoads[1].mValues[2].c_str());
    ASSERT_EQ(std::numeric_limits<int>::max(), tLoads[1].mAppID);
    ASSERT_TRUE(tLoads[1].mRandomVars.empty());

}

TEST(PlatoTestXMLGenerator, ParseTagValues)
{
    XMLGen::UseCaseTags tTags;
    tTags.insert({ "tag", { {"tag"}, "" } });
    tTags.insert({ "attribute", { {"attribute"}, "" } });
    tTags.insert({ "load", { {"load"}, "" } });
    tTags.insert({ "mean", { {"mean"}, "" } });
    tTags.insert({ "distribution", { {"distribution"}, "" } });
    tTags.insert({ "num samples", { {"num", "samples"}, "" } });
    tTags.insert({ "lower bound", { {"lower", "bound"}, "" } });
    tTags.insert({ "upper bound", { {"upper", "bound"}, "" } });
    tTags.insert({ "category", { {"category"}, "" } });
    tTags.insert({ "standard deviation", { {"standard", "deviation"}, "" } });

    XMLGen::parse_tag_values(std::vector<std::string>{"tag", "angle", "variation"}, tTags);
    ASSERT_STREQ("angle variation", tTags.find("tag")->second.second.c_str());
    XMLGen::parse_tag_values(std::vector<std::string>{"attribute", "x"}, tTags);
    ASSERT_STREQ("x", tTags.find("attribute")->second.second.c_str());
    XMLGen::parse_tag_values(std::vector<std::string>{"load", "10"}, tTags);
    ASSERT_STREQ("10", tTags.find("load")->second.second.c_str());
    XMLGen::parse_tag_values(std::vector<std::string>{"mean", "1"}, tTags);
    ASSERT_STREQ("1", tTags.find("mean")->second.second.c_str());
    XMLGen::parse_tag_values(std::vector<std::string>{"distribution", "beta"}, tTags);
    ASSERT_STREQ("beta", tTags.find("distribution")->second.second.c_str());
    XMLGen::parse_tag_values(std::vector<std::string>{"num", "samples", "4"}, tTags);
    ASSERT_STREQ("4", tTags.find("num samples")->second.second.c_str());
    XMLGen::parse_tag_values(std::vector<std::string>{"lower", "bound", "0.5"}, tTags);
    ASSERT_STREQ("0.5", tTags.find("lower bound")->second.second.c_str());
    XMLGen::parse_tag_values(std::vector<std::string>{"upper", "bound", "2"}, tTags);
    ASSERT_STREQ("2", tTags.find("upper bound")->second.second.c_str());
    XMLGen::parse_tag_values(std::vector<std::string>{"category", "load"}, tTags);
    ASSERT_STREQ("load", tTags.find("category")->second.second.c_str());
    XMLGen::parse_tag_values(std::vector<std::string>{"standard", "deviation", "0.2"}, tTags);
    ASSERT_STREQ("0.2", tTags.find("standard deviation")->second.second.c_str());
}

TEST(PlatoTestXMLGenerator, ParseUncertainty_OneRandomVar)
{
    std::string tStringInput =
        "begin objective\n"
        "   type maximize stiffness\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1\n"
        "   analyze new workflow true\n"
        "   number ranks 1\n"
        "end objective\n"
        "begin boundary conditions\n"
        "   fixed displacement nodeset name 1 bc id 11\n"
        "end boundary conditions\n"
        "begin loads\n"
        "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
        "end loads\n"
        "begin material 1\n"
            "penalty exponent 3\n"
            "youngs modulus 1e6\n"
            "poissons ratio 0.33\n"
            "thermal conductivity .02\n"
            "density .001\n"
        "end material\n"
        "begin block 1\n"
        "   material 1\n"
        "end block\n"
        "begin uncertainty\n"
        "    category load\n"
        "    tag angle variation\n"
        "    load id 10\n"
        "    attribute X\n"
        "    distribution beta\n"
        "    mean 0.0\n"
        "    upper bound 45.0\n"
        "    lower bound -45.0\n"
        "    standard deviation 22.5\n"
        "    num samples 2\n"
        "end uncertainty\n"
        "begin optimization parameters\n"
        "end optimization parameters\n";

    std::istringstream tInputs;
    tInputs.str(tStringInput);
    XMLGen::ParseUncertainty tUncertainty;
    tUncertainty.parse(tInputs);

    auto tMetadata = tUncertainty.data();
    ASSERT_STREQ("10", tMetadata[0].id.c_str());
    ASSERT_STREQ("x", tMetadata[0].axis.c_str());
    ASSERT_STREQ("0.0", tMetadata[0].mean.c_str());
    ASSERT_STREQ("45.0", tMetadata[0].upper.c_str());
    ASSERT_STREQ("-45.0", tMetadata[0].lower.c_str());
    ASSERT_STREQ("2", tMetadata[0].num_samples.c_str());
    ASSERT_STREQ("beta", tMetadata[0].distribution.c_str());
    ASSERT_STREQ("load", tMetadata[0].variable_type.c_str());
    ASSERT_STREQ("angle variation", tMetadata[0].type.c_str());
    ASSERT_STREQ("22.5", tMetadata[0].standard_deviation.c_str());
}

TEST(PlatoTestXMLGenerator, ParseUncertainty_TwoRandomVar)
{
    std::string tStringInput =
        "begin objective\n"
        "   type maximize stiffness\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1\n"
        "   analyze new workflow true\n"
        "   number ranks 1\n"
        "end objective\n"
        "begin boundary conditions\n"
        "   fixed displacement nodeset name 1 bc id 11\n"
        "end boundary conditions\n"
        "begin loads\n"
        "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
        "end loads\n"
        "begin material 1\n"
            "penalty exponent 3\n"
            "youngs modulus 1e6\n"
            "poissons ratio 0.33\n"
            "thermal conductivity .02\n"
            "density .001\n"
        "end material\n"
        "begin block 1\n"
        "   material 1\n"
        "end block\n"
        "begin uncertainty\n"
        "    category load\n"
        "    load id 10\n"
        "    tag angle variation\n"
        "    attribute X\n"
        "    distribution beta\n"
        "    mean 0.0\n"
        "    upper bound 45.0\n"
        "    lower bound -45.0\n"
        "    standard deviation 22.5\n"
        "    num samples 2\n"
        "end uncertainty\n"
        "begin uncertainty\n"
        "    category material\n"
        "    material id 1\n"
        "    tag poissons ratio\n"
        "    attribute homogeneous\n"
        "    distribution beta\n"
        "    mean 0.28\n"
        "    upper bound 0.4\n"
        "    lower bound 0.2\n"
        "    standard deviation 0.05\n"
        "    num samples 3\n"
        "end uncertainty\n"
        "begin optimization parameters\n"
        "end optimization parameters\n";

    std::istringstream tInputs;
    tInputs.str(tStringInput);
    XMLGen::ParseUncertainty tUncertainty;
    tUncertainty.parse(tInputs);

    std::vector<std::string> tGoldID = {"10", "1"};
    std::vector<std::string> tGoldMean = {"0.0", "0.28"};
    std::vector<std::string> tGoldUpper = {"45.0", "0.4"};
    std::vector<std::string> tGoldNumSamples = {"2", "3"};
    std::vector<std::string> tGoldLower = {"-45.0", "0.2"};
    std::vector<std::string> tGoldStdDev = {"22.5", "0.05"};
    std::vector<std::string> tGoldCategory = {"load", "material"};
    std::vector<std::string> tGoldDistribution = {"beta", "beta"};
    std::vector<std::string> tGoldAttribute = {"x", "homogeneous"};
    std::vector<std::string> tGoldTag = {"angle variation", "poissons ratio"};
    auto tMetadata = tUncertainty.data();
    for (auto& tVar : tMetadata)
    {
        auto tIndex = &tVar - &tMetadata[0];
        ASSERT_STREQ(tGoldID[tIndex].c_str(), tVar.id.c_str());
        ASSERT_STREQ(tGoldMean[tIndex].c_str(), tVar.mean.c_str());
        ASSERT_STREQ(tGoldUpper[tIndex].c_str(), tVar.upper.c_str());
        ASSERT_STREQ(tGoldLower[tIndex].c_str(), tVar.lower.c_str());
        ASSERT_STREQ(tGoldAttribute[tIndex].c_str(), tVar.axis.c_str());
        ASSERT_STREQ(tGoldCategory[tIndex].c_str(), tVar.variable_type.c_str());
        ASSERT_STREQ(tGoldNumSamples[tIndex].c_str(), tVar.num_samples.c_str());
        ASSERT_STREQ(tGoldStdDev[tIndex].c_str(), tVar.standard_deviation.c_str());
        ASSERT_STREQ(tGoldDistribution[tIndex].c_str(), tVar.distribution.c_str());
    }
}

TEST(PlatoTestXMLGenerator, parseSingleValue)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tTokens;
    std::vector<std::string> tInputStringList;
    std::string tStringValue;
    bool tReturnValue;

    tReturnValue = tester.publicParseSingleValue(tTokens, tInputStringList = {"car", "truck", "bus"}, tStringValue);
    EXPECT_EQ(tReturnValue, false);
    tTokens.push_back("car");
    tTokens.push_back("truck");
    tTokens.push_back("bus");
    tReturnValue = tester.publicParseSingleValue(tTokens, tInputStringList = {}, tStringValue);
    EXPECT_EQ(tReturnValue, false);
    tStringValue = "";
    tReturnValue = tester.publicParseSingleValue(tTokens, tInputStringList = {"car"}, tStringValue);
    EXPECT_EQ(tReturnValue, true);
    EXPECT_EQ(tStringValue, "");
    tStringValue = "";
    tReturnValue = tester.publicParseSingleValue(tTokens, tInputStringList = {"car", "truck", "bus"}, tStringValue);
    EXPECT_EQ(tReturnValue, true);
    EXPECT_EQ(tStringValue, "");
    tReturnValue = tester.publicParseSingleValue(tTokens, tInputStringList = {"car", "bus"}, tStringValue);
    EXPECT_EQ(tReturnValue, false);
    tStringValue = "";
    tReturnValue = tester.publicParseSingleValue(tTokens, tInputStringList = {"car","truck"}, tStringValue);
    EXPECT_EQ(tReturnValue, true);
    EXPECT_EQ(tStringValue, "bus");
}
TEST(PlatoTestXMLGenerator, parseSingleUnLoweredValue)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tTokens;
    std::vector<std::string> tUnLoweredTokens;
    std::vector<std::string> tInputStringList;
    std::string tStringValue;
    bool tReturnValue;

    tReturnValue = tester.publicParseSingleUnLoweredValue(tTokens, tUnLoweredTokens, tInputStringList = {"car", "truck", "bus"}, tStringValue);
    EXPECT_EQ(tReturnValue, false);
    tTokens.push_back("car");
    tTokens.push_back("truck");
    tTokens.push_back("bus");
    tUnLoweredTokens.push_back("Car");
    tUnLoweredTokens.push_back("Truck");
    tUnLoweredTokens.push_back("Bus");
    tReturnValue = tester.publicParseSingleUnLoweredValue(tTokens, tUnLoweredTokens, tInputStringList = {}, tStringValue);
    EXPECT_EQ(tReturnValue, false);
    tStringValue = "";
    tReturnValue = tester.publicParseSingleUnLoweredValue(tTokens, tUnLoweredTokens, tInputStringList = {"car"}, tStringValue);
    EXPECT_EQ(tReturnValue, true);
    EXPECT_EQ(tStringValue, "");
    tStringValue = "";
    tReturnValue = tester.publicParseSingleUnLoweredValue(tTokens, tUnLoweredTokens, tInputStringList = {"car", "truck", "bus"}, tStringValue);
    EXPECT_EQ(tReturnValue, true);
    EXPECT_EQ(tStringValue, "");
    tStringValue = "";
    tReturnValue = tester.publicParseSingleUnLoweredValue(tTokens, tUnLoweredTokens, tInputStringList = {"car", "bus"}, tStringValue);
    EXPECT_EQ(tReturnValue, false);
    tStringValue = "";
    tReturnValue = tester.publicParseSingleUnLoweredValue(tTokens, tUnLoweredTokens, tInputStringList = {"car","truck"}, tStringValue);
    EXPECT_EQ(tReturnValue, true);
    EXPECT_EQ(tStringValue, "Bus");
}
TEST(PlatoTestXMLGenerator, parseTokens)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    char buffer[MAX_CHARS_PER_LINE];

    buffer[0] = '\0';
    tester.publicParseTokens(buffer, tokens);
    EXPECT_EQ((int)tokens.size(), 0);
    buffer[0] = '\t';
    buffer[1] = '\0';
    tester.publicParseTokens(buffer, tokens);
    EXPECT_EQ((int)tokens.size(), 0);
    strcpy(buffer, "   car");
    tester.publicParseTokens(buffer, tokens);
    EXPECT_EQ((int)tokens.size(), 1);
    tokens.clear();
    buffer[0] = '\t';
    strcpy(&(buffer[1]), "   car");
    tester.publicParseTokens(buffer, tokens);
    EXPECT_EQ((int)tokens.size(), 1);
    tokens.clear();
    strcpy(buffer, "   car\tbus   trike\t");
    tester.publicParseTokens(buffer, tokens);
    EXPECT_EQ((int)tokens.size(), 3);
    EXPECT_EQ(tokens[0], "car");
    EXPECT_EQ(tokens[1], "bus");
    EXPECT_EQ(tokens[2], "trike");
}

TEST(PlatoTestXMLGenerator, parseOptimizationParameters)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput;

    // material_box
    stringInput = "begin optimization parameters\n"
            "begin material_box\n"
            "min coords 0 0 0\n"
            "end material_box\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "begin material_box\n"
            "max coords 0 0 0\n"
            "end material_box\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "begin material_box\n"
            "min coords 0 0 1 \n"
            "max coords 0 0 \n"
            "end material_box\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "begin material_box\n"
            "min coords 0 0  \n"
            "max coords 0 0 1\n"
            "end material_box\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "begin material_box\n"
            "min coords 1 2 3 \n"
            "max coords 4 5 6\n"
            "end material_box\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getMatBoxMinCoords(), "1 2 3");
    EXPECT_EQ(tester.getMatBoxMaxCoords(), "4 5 6");

    // Invalid keywords
    stringInput = "begin optimization parameters\n"
            "hippo\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);

    // initial density value
    stringInput = "begin optimization parameters\n"
            "initial density value\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "initial density value .1\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getInitDensityValue(), ".1");

    // create levelset spheres
    stringInput = "begin optimization parameters\n"
            "create levelset spheres\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "create levelset spheres true\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getCreateLevelsetSpheres(), "true");

    // levelset initialization method
    stringInput = "begin optimization parameters\n"
            "levelset initialization method\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "levelset initialization method primitives\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getLevelsetInitMethod(), "primitives");

    // max iterations
    stringInput = "begin optimization parameters\n"
            "max iterations\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "max iterations 45\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getMaxIterations(), "45");

    // restart iteration
    stringInput = "begin optimization parameters\n"
            "restart iteration\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "restart iteration 22\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getRestartIteration(), "1");
    stringInput = "begin optimization parameters\n"
            "initial guess filename file.gen\n"
            "initial guess field name topology\n"
            "restart iteration 22\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getRestartIteration(), "22");
    EXPECT_EQ(tester.getRestartFieldName(), "topology");
    EXPECT_EQ(tester.getRestartMeshFilename(), "file.gen");
    stringInput = "begin optimization parameters\n"
            "initial guess filename fIle.gen\n"
            "initial guess field name TopologY\n"
            "restart iteration 22\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getRestartIteration(), "22");
    EXPECT_EQ(tester.getRestartFieldName(), "TopologY");
    EXPECT_EQ(tester.getRestartMeshFilename(), "fIle.gen");

    // ks max trust region iterations
    stringInput = "begin optimization parameters\n"
            "ks max trust region iterations\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks max trust region iterations 10\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSMaxTrustIterations(), "10");

    // ks trust region ratio low/mid/upper
    stringInput = "begin optimization parameters\n"
            "ks trust region ratio low\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks trust region ratio mid\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks trust region ratio high\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks trust region ratio low 1.1\n"
            "ks trust region ratio mid 2.1\n"
            "ks trust region ratio high 3.1\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->mTrustRegionRatioLowKS, "1.1");
    EXPECT_EQ(tester.exposeInputData()->mTrustRegionRatioMidKS, "2.1");
    EXPECT_EQ(tester.exposeInputData()->mTrustRegionRatioUpperKS, "3.1");

    // ks disable post smoothing
    stringInput = "begin optimization parameters\n"
            "ks disable post smoothing\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "end optimization parameters\n";
    stringInput = "begin optimization parameters\n"
            "ks disable post smoothing fALse\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->mDisablePostSmoothingKS, "false");
    stringInput = "begin optimization parameters\n"
            "ks disable post smoothing tRuE\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->mDisablePostSmoothingKS, "true");


    // ks trust region expansion factor
    stringInput = "begin optimization parameters\n"
            "ks trust region expansion factor\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks trust region expansion factor .5\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSExpansionFactor(), ".5");

    // ks trust region contraction factor
    stringInput = "begin optimization parameters\n"
            "ks trust region contraction factor\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks trust region contraction factor .2\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSContractionFactor(), ".2");

    // ks outer gradient tolerance
    stringInput = "begin optimization parameters\n"
            "ks outer gradient tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks outer gradient tolerance .001\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSOuterGradientTolerance(), ".001");

    // ks outer stationarity tolerance
    stringInput = "begin optimization parameters\n"
            "ks outer stationarity tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks outer stationarity tolerance .00001\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSOuterStationarityTolerance(), ".00001");

    // ks outer stagnation tolerance
    stringInput = "begin optimization parameters\n"
            "ks outer stagnation tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks outer stagnation tolerance 2.3\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSOuterStagnationTolerance(), "2.3");

    // ks outer control stagnation tolerance
    stringInput = "begin optimization parameters\n"
            "ks outer control stagnation tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks outer control stagnation tolerance 1e-8\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSOuterControlStagnationTolerance(), "1e-8");

    // ks outer actual reduction tolerance
    stringInput = "begin optimization parameters\n"
            "ks outer actual reduction tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks outer actual reduction tolerance 1e-7\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSOuterActualReductionTolerance(), "1e-7");

    // gcmma max inner iterations
    stringInput = "begin optimization parameters\n"
            "gcmma max inner iterations\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "gcmma max inner iterations 100\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAMaxInnerIterations(), "100");

    // gcmma inner kkt tolerance
    stringInput = "begin optimization parameters\n"
            "gcmma inner kkt tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "gcmma inner kkt tolerance 1e-12\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAInnerKKTTolerance(), "1e-12");

    // gcmma inner control stagnation tolerance
    stringInput = "begin optimization parameters\n"
            "gcmma inner control stagnation tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "gcmma inner control stagnation tolerance 5e-10\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAInnerControlStagnationTolerance(), "5e-10");

    // gcmma outer kkt tolerance
    stringInput = "begin optimization parameters\n"
            "gcmma outer kkt tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "gcmma outer kkt tolerance 1e-8\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAOuterKKTTolerance(), "1e-8");

    // gcmma outer control stagnation tolerance
    stringInput = "begin optimization parameters\n"
            "gcmma outer control stagnation tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "gcmma outer control stagnation tolerance 1e-11\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAOuterControlStagnationTolerance(), "1e-11");

    // gcmma outer objective stagnation tolerance
    stringInput = "begin optimization parameters\n"
            "gcmma outer objective stagnation tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "gcmma outer objective stagnation tolerance 4e-8\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAOuterObjectiveStagnationTolerance(), "4e-8");

    // gcmma outer stationarity tolerance
    stringInput = "begin optimization parameters\n"
            "gcmma outer stationarity tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "gcmma outer stationarity tolerance 2e-10\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAOuterStationarityTolerance(), "2e-10");

    // gcmma initial moving asymptotes scale factor
    stringInput = "begin optimization parameters\n"
            "gcmma initial moving asymptotes scale factor\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "gcmma initial moving asymptotes scale factor 0.4\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAInitialMovingAsymptotesScaleFactor(), "0.4");

    // levelset sphere packing factor
    stringInput = "begin optimization parameters\n"
            "levelset sphere packing factor\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "levelset sphere packing factor .2\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getLevelsetSpherePackingFactor(), ".2");

    // levelset sphere radius
    stringInput = "begin optimization parameters\n"
            "levelset sphere radius\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "levelset sphere radius .33\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getLevelsetSphereRadius(), ".33");

    // levelset nodesets
    stringInput = "begin optimization parameters\n"
            "levelset nodesets\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "levelset nodesets 4 10 22\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getLevelsetNodeset(0), "4");
    EXPECT_EQ(tester.getLevelsetNodeset(1), "10");
    EXPECT_EQ(tester.getLevelsetNodeset(2), "22");

    // output frequency
    stringInput = "begin optimization parameters\n"
            "output frequency\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "output frequency 5\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getOutputFrequency(), "5");

    // output method
    stringInput = "begin optimization parameters\n"
            "output method\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "output method parallel write\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getOutputMethod(), "parallel write");

    // fixed blocks
    stringInput = "begin optimization parameters\n"
            "fixed blocks\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "fixed blocks 1 3 5\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFixedBlock(0), "1");
    EXPECT_EQ(tester.getFixedBlock(1), "3");
    EXPECT_EQ(tester.getFixedBlock(2), "5");

    // fixed sidesets
    stringInput = "begin optimization parameters\n"
            "fixed sidesets\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "fixed sidesets 33 44 55\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFixedSideset(0), "33");
    EXPECT_EQ(tester.getFixedSideset(1), "44");
    EXPECT_EQ(tester.getFixedSideset(2), "55");

    // fixed nodesets
    stringInput = "begin optimization parameters\n"
            "fixed nodesets\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "fixed nodesets 5 7 33\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFixedNodeset(0), "5");
    EXPECT_EQ(tester.getFixedNodeset(1), "7");
    EXPECT_EQ(tester.getFixedNodeset(2), "33");

    // number processors
    stringInput = "begin optimization parameters\n"
            "number processors\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "number processors 16\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getNumberProcessors(), "16");

    // filter scale
    stringInput = "begin optimization parameters\n"
            "filter radius scale\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter radius scale 1.6\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFilterScale(), "1.6");

    // filter type
    stringInput = "begin optimization parameters\n"
            "filter type kernel\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->filter_type,"kernel");
    stringInput = "begin optimization parameters\n"
            "filter type kernel then heaviside\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->filter_type,"kernel then heaviside");
    stringInput = "begin optimization parameters\n"
            "filter type\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter type lions and tigers\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);

    // filter heaviside min
    stringInput = "begin optimization parameters\n"
            "filter heaviside min\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter heaviside min 1.526\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_min,"1.526");

    // filter heaviside update
    stringInput = "begin optimization parameters\n"
            "filter heaviside update\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter heaviside update 1.526\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_update,"1.526");

    // filter heaviside max
    stringInput = "begin optimization parameters\n"
            "filter heaviside max\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter heaviside max 1.526\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_max,"1.526");

    // filter heaviside scale
    stringInput = "begin optimization parameters\n"
            "filter heaviside scale\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter heaviside scale 1.526\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_min,"1.526");
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_max,"1.526");

    // filter heaviside various
    stringInput = "begin optimization parameters\n"
            "filter heaviside update 4.4\n"
            "filter heaviside scale 1.2\n"
            "filter heaviside min 0.5\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter heaviside update 4.4\n"
            "filter heaviside scale 1.2\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_min,"1.2");
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_update,"4.4");
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_max,"1.2");
    stringInput = "begin optimization parameters\n"
            "filter heaviside update 4.4\n"
            "filter heaviside min 0.5\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_min,"0.5");
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_update,"4.4");

    // filter absolute
    stringInput = "begin optimization parameters\n"
            "filter radius absolute\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter radius absolute 1.6\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFilterAbsolute(), "1.6");

    // filter absolute
    stringInput = "begin optimization parameters\n"
            "filter radius absolute\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter radius absolute 1.6\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFilterAbsolute(), "1.6");

    // filter radial power
    stringInput = "begin optimization parameters\n"
            "filter radial power\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter radial power 2.5\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFilterPower(), "2.5");

    // algorithm
    stringInput = "begin optimization parameters\n"
            "algorithm\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "algorithm ksbc\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getAlgorithm(), "ksbc");

    // discretization
    stringInput = "begin optimization parameters\n"
            "discretization\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "discretization levelset\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getDiscretization(), "levelset");

    // check gradient
    stringInput = "begin optimization parameters\n"
            "check gradient\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "check gradient false\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getCheckGradient(), "false");

    // check hessian
    stringInput = "begin optimization parameters\n"
            "check hessian\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "check hessian true\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getCheckHessian(), "true");
}

TEST(PlatoTestXMLGenerator, parseConstraints)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput;

    stringInput = "begin constraint\n"
            "type\n"
            "end constraint\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseConstraints(iss), false);
    stringInput = "begin constraint\n"
            "name\n"
            "end constraint\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseConstraints(iss), false);
    stringInput = "begin constraint\n"
            "volume fraction\n"
            "end constraint\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseConstraints(iss), false);
    stringInput = "begin constraint\n"
            "surface area\n"
            "end constraint\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseConstraints(iss), false);
    stringInput = "begin constraint\n"
            "surface area sideset id\n"
            "end constraint\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseConstraints(iss), false);
    stringInput = "begin constraint\n"
            "bad_keyword\n"
            "end constraint\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseConstraints(iss), false);
    stringInput = "begin constraint\n"
            "type volume\n"
            "name vol1\n"
            "volume fraction .5\n"
            "end constraint\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseConstraints(iss), true);
    EXPECT_EQ(tester.getConstraintName(0), "vol1");
    EXPECT_EQ(tester.getConstraintType(0), "volume");
    EXPECT_EQ(tester.getConstraintVolFrac(0), ".5");
    stringInput = "begin constraint\n"
            "type surface area\n"
            "name surf1\n"
            "surface area 23\n"
            "surface area sideset id 20\n"
            "end constraint\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseConstraints(iss), true);
    EXPECT_EQ(tester.getConstraintName(0), "surf1");
    EXPECT_EQ(tester.getConstraintType(0), "surface area");
    EXPECT_EQ(tester.getConstraintSurfArea(0), "23");
    EXPECT_EQ(tester.getConstraintSurfAreaSidesetID(0), "20");
}
TEST(PlatoTestXMLGenerator, parseMesh)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput;

    stringInput = "begin mesh\n"
            "name\n"
            "end mesh\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMesh(iss), false);
    stringInput = "begin mesh\n"
            "bad_keyword\n"
            "end mesh\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMesh(iss), false);
    stringInput = "begin mesh\n"
            "name file.gen\n"
            "end mesh\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMesh(iss), true);
    EXPECT_EQ(tester.getMeshName(), "file.gen");
}
TEST(PlatoTestXMLGenerator, parseCodePaths)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput;

    stringInput = "begin paths\n"
            "code sierra_sd\n"
            "end paths\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseCodePaths(iss), false);
    stringInput = "begin paths\n"
            "code lightmp\n"
            "end paths\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseCodePaths(iss), false);
    stringInput = "begin paths\n"
            "code albany\n"
            "end paths\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseCodePaths(iss), false);
    stringInput = "begin paths\n"
            "code platomain\n"
            "end paths\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseCodePaths(iss), false);
    stringInput = "begin paths\n"
            "bad_keyword\n"
            "end paths\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseCodePaths(iss), false);
    stringInput = "begin paths\n"
            "code sierra_sd /Users/bwclark/salinas\n"
            "code albany /Users/bwclark/albany\n"
            "code lightmp /Users/bwclark/lightmp\n"
            "code platomain /Users/bwclark/platomain\n"
            "end paths\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseCodePaths(iss), true);
    EXPECT_EQ(tester.getSalinasPath(), "/Users/bwclark/salinas");
    EXPECT_EQ(tester.getAlbanyPath(), "/Users/bwclark/albany");
    EXPECT_EQ(tester.getLightMPPath(), "/Users/bwclark/lightmp");
    EXPECT_EQ(tester.getPlatoMainPath(), "/Users/bwclark/platomain");
}
TEST(PlatoTestXMLGenerator, parseBlocks)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput;

    stringInput = "begin block\n"
            "end block\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBlocks(iss), false);
    stringInput = "begin block 1\n"
            "material\n"
            "end block\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBlocks(iss), false);
    stringInput = "begin block 1\n"
            "bad_keywordl\n"
            "end block\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBlocks(iss), false);
    stringInput = "begin block 1\n"
            "end block\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBlocks(iss), true);
    stringInput = "begin block\n"
            "material 1\n"
            "end block\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBlocks(iss), false);
    stringInput = "begin block 44\n"
            "material 89\n"
            "end block\n"
            "begin block 33\n"
            "material 34\n"
            "end block\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBlocks(iss), true);
    EXPECT_EQ(tester.getBlockID(0), "44");
    EXPECT_EQ(tester.getBlockMaterialID(0), "89");
    EXPECT_EQ(tester.getBlockID(1), "33");
    EXPECT_EQ(tester.getBlockMaterialID(1), "34");
}
TEST(PlatoTestXMLGenerator, parseMaterials)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput;

    stringInput = "begin material 1\n"
            "penalty exponent\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material 1\n"
            "youngs modulus\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material 1\n"
            "poissons ratio\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material 1\n"
            "density\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material 1\n"
            "thermal conductivity\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material 1\n"
            "bad_keyword\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material 34\n"
            "penalty exponent 3\n"
            "youngs modulus 1e6\n"
            "poissons ratio 0.33\n"
            "thermal conductivity .02\n"
            "density .001\n"
            "end material\n"
            "begin material 388\n"
            "penalty exponent 5\n"
            "youngs modulus 1e7\n"
            "poissons ratio 0.34\n"
            "thermal conductivity .03\n"
            "density .009\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), true);
    EXPECT_EQ(tester.getMaterialID(0), "34");
    EXPECT_EQ(tester.getMaterialPenaltyExponent(0), "3");
    EXPECT_EQ(tester.getMaterialYoungsModulus(0), "1e6");
    EXPECT_EQ(tester.getMaterialPoissonsRatio(0), "0.33");
    EXPECT_EQ(tester.getMaterialThermalConductivity(0), ".02");
    EXPECT_EQ(tester.getMaterialDensity(0), ".001");
    EXPECT_EQ(tester.getMaterialID(1), "388");
    EXPECT_EQ(tester.getMaterialPenaltyExponent(1), "5");
    EXPECT_EQ(tester.getMaterialYoungsModulus(1), "1e7");
    EXPECT_EQ(tester.getMaterialPoissonsRatio(1), "0.34");
    EXPECT_EQ(tester.getMaterialThermalConductivity(1), ".03");
    EXPECT_EQ(tester.getMaterialDensity(1), ".009");
}
TEST(PlatoTestXMLGenerator, parseObjectives)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput;

    // Test some examples of valid objectives.
    stringInput = "begin objective\n"
            "type match frf data\n"
            "code sierra_sd\n"
            "end objective\n"
            "begin objective\n"
            "type match frf data\n"
            "name frf_1\n"
            "end objective\n";
    iss.str(stringInput);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    EXPECT_EQ(tester.getObjectiveName(0), "1");
    EXPECT_EQ(tester.getObjectiveName(1), "frf_1");

    // Test the unrecognized keyword
    stringInput = "begin objective\n"
            "hippo \n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    stringInput = "begin objective\n"
            "fire fly\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    stringInput = "begin objective\n"
            "begin\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "type" keyword
    stringInput = "begin objective\n"
            "type \n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "distribute objective" keyword
    stringInput = "begin objective\n"
            "distribute\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    stringInput = "begin objective\n"
            "distribute objective\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    stringInput = "begin objective\n"
            "distribute objective kangaroo\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    stringInput = "begin objective\n"
            "distribute objective none\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    stringInput = "begin objective\n"
            "multi load case true\n"
            "distribute objective at most 256 processors\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);

    // Test the "name" keyword
    stringInput = "begin objective\n"
            "name \n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "code" keyword
    stringInput = "begin objective\n"
            "code \n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "complex error measure" keywords
    stringInput = "begin objective\n"
            "complex error measure \n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "number processors" keywords
    stringInput = "begin objective\n"
            "number processors\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "weight" keyword
    stringInput = "begin objective\n"
            "weight \n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "frf match nodesets" keywords
    stringInput = "begin objective\n"
            "frf match nodesets\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "sierra_sd weightmass scale factor" keywords
    stringInput = "begin objective\n"
            "weightmass scale factor\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "sierra_sd analysis solver tolerance" keywords
    stringInput = "begin objective\n"
            "analysis solver tolerance\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "reference frf file" keywords
    stringInput = "begin objective\n"
            "reference frf file\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "raleigh damping alpha" keywords
    stringInput = "begin objective\n"
            "raleigh damping alpha\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "raleigh damping beta" keywords
    stringInput = "begin objective\n"
            "raleigh damping beta\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    tester.clearInputData();

    // Test the "volume misfit target"
    stringInput = "begin objective\n"
            "volume misfit target 0.5125\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    EXPECT_EQ(tester.exposeInputData()->objectives[0].volume_misfit_target, "0.5125");
    tester.clearInputData();

    stringInput = "begin objective\n"
            "volume misfit target\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    tester.clearInputData();

    // Test the "stress p norm"
    stringInput = "begin objective\n"
            "type stress p norm\n"
            "stress limit 39.5\n"
            "stress ramp factor 0.22\n"
            "stress p norm power 7\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    EXPECT_EQ(tester.exposeInputData()->objectives[0].type, "stress p norm");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].stress_limit, "39.5");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].stress_ramp_factor, "0.22");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].stress_p_norm_power, "7");
    tester.clearInputData();

    stringInput = "begin objective\n"
            "type stress p norm\n"
            "stress p norm power\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    tester.clearInputData();

    stringInput = "begin objective\n"
            "type stress p norm\n"
            "stress ramp factor\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    tester.clearInputData();

    // Test the "stress limit" keywords
    stringInput = "begin objective\n"
            "type limit stress\n"
            "stress limit 42.5\n"
            "stress ramp factor 0.24\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    EXPECT_EQ(tester.getObjectiveType(0), "limit stress");
    EXPECT_EQ(tester.getObjStressLimit(0), "42.5");
    EXPECT_EQ(tester.getObjStressRampFactor(0), "0.24");
    tester.clearInputData();

    stringInput = "begin objective\n"
            "type limit stress\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    EXPECT_EQ(tester.getObjectiveType(0), "limit stress");
    EXPECT_EQ(tester.getObjStressLimit(0), "");
    EXPECT_EQ(tester.getObjStressRampFactor(0), "");
    tester.clearInputData();

    stringInput = "begin objective\n"
            "type limit stress\n"
            "limit power min 2.1\n"
            "limit power max 42.3\n"
            "limit power feasible bias 1.1\n"
            "limit power feasible slope 1.9\n"
            "limit power infeasible bias -0.51\n"
            "limit power infeasible slope 0.92\n"
            "limit reset subfrequency 11\n"
            "limit reset count 3\n"
            "inequality allowable feasibility upper 0.045\n"
            "stress inequality power 1.51\n"
            "volume penalty power 1.254\n"
            "volume penalty divisor 5.124\n"
            "volume penalty bias 0.4242\n"
            "inequality feasibility scale 2.2\n"
            "inequality infeasibility scale 0.48\n"
            "stress favor final 10.12\n"
            "stress favor updates 39\n"
            "relative stress limit 1.57\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    EXPECT_EQ(tester.getObjectiveType(0), "limit stress");
    EXPECT_EQ(tester.getObjStressLimit(0), "");
    EXPECT_EQ(tester.getObjStressRampFactor(0), "");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].limit_power_min, "2.1");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].limit_power_max, "42.3");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].limit_power_feasible_bias, "1.1");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].limit_power_feasible_slope, "1.9");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].limit_power_infeasible_bias, "-0.51");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].limit_power_infeasible_slope, "0.92");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].inequality_allowable_feasiblity_upper, "0.045");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].stress_inequality_power, "1.51");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].volume_penalty_power, "1.254");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].volume_penalty_divisor, "5.124");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].volume_penalty_bias, "0.4242");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].limit_reset_subfrequency, "11");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].limit_reset_count, "3");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].inequality_feasibility_scale, "2.2");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].inequality_infeasibility_scale, "0.48");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].stress_favor_final, "10.12");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].stress_favor_updates, "39");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].relative_stress_limit, "1.57");
    tester.clearInputData();

    stringInput = "begin objective\n"
            "type limit stress\n"
            "stress limit\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    tester.clearInputData();

    stringInput = "begin objective\n"
            "type limit stress\n"
            "stress limit 1.5\n"
            "relative stress limit 2.5\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    tester.clearInputData();

    stringInput = "begin objective\n"
            "type limit stress\n"
            "relative stress limit 1.3\n"
            "stress limit 2.1\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    tester.clearInputData();

    stringInput = "begin objective\n"
            "type limit stress\n"
            "stress ramp factor\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    tester.clearInputData();

    // Test the "bcs" keywords
    stringInput =
            "begin boundary conditions\n"
            "fixed displacement nodeset 1 bc id 33\n"
            "end boundary conditions\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    EXPECT_EQ(tester.getBCApplicationType("33"), "nodeset");
    EXPECT_EQ(tester.getBCApplicationID("33"), "1");
    EXPECT_EQ(tester.getBCApplicationDOF("33"), "");
    stringInput =
            "begin boundary conditions\n"
            "fixed displacement nodeset 2 x bc id 33\n"
            "end boundary conditions\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    EXPECT_EQ(tester.getBCApplicationType("33"), "nodeset");
    EXPECT_EQ(tester.getBCApplicationID("33"), "2");
    EXPECT_EQ(tester.getBCApplicationDOF("33"), "x");
    stringInput =
            "begin boundary conditions\n"
            "fixed displacement\n"
            "end boundary conditions\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBCs(iss), false);
    stringInput =
            "begin boundary conditions\n"
            "displacement nodeset 1 x\n"
            "end boundary conditions\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBCs(iss), false);
    stringInput =
            "begin boundary conditions\n"
            "fixed flux nodeset 1 x\n"
            "end boundary conditions\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBCs(iss), false);
    stringInput =
            "begin boundary conditions\n"
            "fixed displacement sideset 1 x bc id 33\n"
            "end boundary conditions\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    stringInput =
            "begin boundary conditions\n"
            "fixed displacement nodeset 1 truck bc id 33\n"
            "end boundary conditions\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBCs(iss), false);

    // Test loads.
    // Check for known load types.
    stringInput =
            "begin loads\n"
            "pressure blah blah blah\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);

    // Tractions
    // check number of parameters
    stringInput =
            "begin loads\n"
            "traction\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // check for direction keyword
    stringInput =
            "begin loads\n"
            "traction sideset 1 bad_keyword 0 0 1 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // Do a correct example
    stringInput =
            "begin loads\n"
            "traction sideset 3 value 990 0 1000 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    EXPECT_EQ(tester.getLoadType("34",0), "traction");
    EXPECT_EQ(tester.getLoadApplicationType("34",0), "sideset");
    EXPECT_EQ(tester.getLoadApplicationID("34",0), "3");
    EXPECT_EQ(tester.getLoadDirectionX("34",0), "990");
    EXPECT_EQ(tester.getLoadDirectionY("34",0), "0");
    EXPECT_EQ(tester.getLoadDirectionZ("34",0), "1000");

    // Pressure
    // check number of parameters
    stringInput =
            "begin loads\n"
            "pressure\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // check applying to correct mesh type
    stringInput =
            "begin loads\n"
            "pressure nodeset 1 value 1000 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // check for value keyword
    stringInput =
            "begin loads\n"
            "pressure sideset 1 bad_keyword 1000 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // Do a correct example
    stringInput =
            "begin loads\n"
            "pressure sideset 3 value 1000 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    EXPECT_EQ(tester.getLoadType("34",0), "pressure");
    EXPECT_EQ(tester.getLoadApplicationType("34",0), "sideset");
    EXPECT_EQ(tester.getLoadApplicationID("34",0), "3");

    // Heat Flux
    // check number of parameters
    stringInput =
            "begin loads\n"
            "heat flux\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // check applying to correct mesh type
    stringInput =
            "begin loads\n"
            "heat flux nodeset 1 value 1000 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // Do a correct example
    stringInput =
            "begin loads\n"
            "heat flux sideset 4 value 90 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    EXPECT_EQ(tester.getLoadType("34",0), "heat");
    EXPECT_EQ(tester.getLoadApplicationType("34",0), "sideset");
    EXPECT_EQ(tester.getLoadApplicationID("34",0), "4");

    // Force
    // check number of parameters
    stringInput =
            "begin loads\n"
            "force\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // check applying to correct mesh type
    stringInput =
            "begin loads\n"
            "force block 1 value 0 0 1000 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // check for direction keyword
    stringInput =
            "begin loads\n"
            "force nodeset 1 bad_keyword 0 0 1 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // Do a good example
    stringInput =
            "begin loads\n"
            "force nodeset 2 value 222 0 0 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    EXPECT_EQ(tester.getLoadType("34",0), "force");
    EXPECT_EQ(tester.getLoadApplicationType("34",0), "nodeset");
    EXPECT_EQ(tester.getLoadApplicationID("34",0), "2");
    EXPECT_EQ(tester.getLoadDirectionX("34",0), "222");
    EXPECT_EQ(tester.getLoadDirectionY("34",0), "0");
    EXPECT_EQ(tester.getLoadDirectionZ("34",0), "0");

    // Test frequency block
    // check for scale keyword
    stringInput = "begin objective\n"
            "begin frequency\n"
            "bad_keyword\n"
            "end frequency\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    // check for missing info
    stringInput = "begin objective\n"
            "begin frequency\n"
            "min 100\n"
            "step 4\n"
            "end frequency\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    // Do good example
    stringInput = "begin objective\n"
            "begin frequency\n"
            "min 100\n"
            "max 900\n"
            "step 4\n"
            "end frequency\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    EXPECT_EQ(tester.getObjFreqMin(0), "100");
    EXPECT_EQ(tester.getObjFreqMax(0), "900");
    EXPECT_EQ(tester.getObjFreqStep(0), "4");

    // Some checks for objective names
    stringInput = "begin objective\n"
            "code sierra_sd\n"
            "name obj1\n"
            "end objective\n"
            "begin objective\n"
            "code sierra_sd\n"
            "end objective\n"
            "begin objective\n"
            "code albany\n"
            "end objective\n"
            "begin objective\n"
            "code sierra_sd\n"
            "end objective\n"
            "begin objective\n"
            "code lightmp\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    EXPECT_EQ(tester.getObjectiveName(0), "obj1");
    EXPECT_EQ(tester.getObjectiveName(1), "2");
    EXPECT_EQ(tester.getObjectiveName(2), "1");
    EXPECT_EQ(tester.getObjectiveName(3), "3");
    EXPECT_EQ(tester.getObjectiveName(4), "1");
    EXPECT_EQ(tester.getObjCodeName(0), "sierra_sd");
    EXPECT_EQ(tester.getObjCodeName(1), "sierra_sd");
    EXPECT_EQ(tester.getObjCodeName(2), "albany");
    EXPECT_EQ(tester.getObjCodeName(3), "sierra_sd");
    EXPECT_EQ(tester.getObjCodeName(4), "lightmp");
    EXPECT_EQ(tester.getObjPerfName(0), "sierra_sd_obj1");
    EXPECT_EQ(tester.getObjPerfName(1), "sierra_sd_2");
    EXPECT_EQ(tester.getObjPerfName(2), "albany_1");
    EXPECT_EQ(tester.getObjPerfName(3), "sierra_sd_3");
    EXPECT_EQ(tester.getObjPerfName(4), "lightmp_1");
}

TEST(PlatoTestXMLGenerator,parseTractionLoad_valid_input)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "traction sideset 2 value 0 -3e3 0 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParseTractionLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"traction");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    std::vector<std::string> values = {"0","-3e3","0"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parseTractionLoad_valid_input_name_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "traction sideset name ss_2 value 0 -3e3 0 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParseTractionLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"traction");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"");
    EXPECT_EQ(new_load.app_name,"ss_2");
    std::vector<std::string> values = {"0","-3e3","0"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parseTractionLoad_valid_input_id_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "traction sideset id 2 value 0 -3e3 0 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParseTractionLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"traction");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    EXPECT_EQ(new_load.app_name,"");
    std::vector<std::string> values = {"0","-3e3","0"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parseTractionLoad_valid_input_name_and_id_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "traction sideset id 2 name ss_2 value 0 -3e3 0 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParseTractionLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"traction");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    EXPECT_EQ(new_load.app_name,"ss_2");
    std::vector<std::string> values = {"0","-3e3","0"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parsePressureLoad_valid_input)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "pressure sideset 2 value 5 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParsePressureLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"pressure");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    std::vector<std::string> values = {"5"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parsePressureLoad_valid_input_name_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "pressure sideset name ss_2 value 5 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParsePressureLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"pressure");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"");
    EXPECT_EQ(new_load.app_name,"ss_2");
    std::vector<std::string> values = {"5"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parsePressureLoad_valid_input_id_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "pressure sideset id 2 value -3e3 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParsePressureLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"pressure");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    EXPECT_EQ(new_load.app_name,"");
    std::vector<std::string> values = {"-3e3"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parsePressureLoad_valid_input_name_and_id_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "pressure sideset id 2 name ss_2 value -3e3 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParsePressureLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"pressure");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    EXPECT_EQ(new_load.app_name,"ss_2");
    std::vector<std::string> values = {"-3e3"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parseHeatFluxLoad_valid_input_id_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "heat flux sideset id 2 value -3e3 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParseHeatFluxLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"heat");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    EXPECT_EQ(new_load.app_name,"");
    std::vector<std::string> values = {"-3e3"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parseHeatFluxLoad_valid_input_name_and_id_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "heat flux sideset id 2 name ss_2 value -3e3 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParseHeatFluxLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"heat");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    EXPECT_EQ(new_load.app_name,"ss_2");
    std::vector<std::string> values = {"-3e3"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parseForceLoad_valid_input_id_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "force sideset id 2 value 0 -3e3 0 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParseForceLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"force");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    EXPECT_EQ(new_load.app_name,"");
    std::vector<std::string> values = {"0","-3e3","0"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parseForceLoad_valid_input_name_and_id_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "force sideset id 2 name ss_2 value 0 -3e3 0 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParseForceLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"force");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    EXPECT_EQ(new_load.app_name,"ss_2");
    std::vector<std::string> values = {"0","-3e3","0"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,distributeObjective_notEnoughProcessors)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 2 5 7 11\n"
            "    boundary condition ids 256\n"
            "    code sierra_sd\n"
            "    number processors 2\n"
            "    multi load case true\n"
            "    weight 1 \n"
            "    distribute objective at most 7 processors\n"
            "end objective\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);

    // BEFORE modification

    // one objective
    ASSERT_EQ(tester.getNumObjectives(), 1u);

    // do expand
    EXPECT_EQ(tester.publicDistributeObjectivesForGenerate(), true);

    // AFTER modification

    // three objectives
    const size_t expected_num_objectives = 3u;
    ASSERT_EQ(tester.getNumObjectives(), expected_num_objectives);

    // check names
    EXPECT_EQ(tester.getObjectiveName(0u),"1");
    EXPECT_EQ(tester.getObjectiveName(1u),"2");
    EXPECT_EQ(tester.getObjectiveName(2u),"3");
    EXPECT_EQ(tester.getObjPerfName(0u),"sierra_sd_1");
    EXPECT_EQ(tester.getObjPerfName(1u),"sierra_sd_2");
    EXPECT_EQ(tester.getObjPerfName(2u),"sierra_sd_3");

    // get loadIds for distributed objective
    std::vector<std::string> distributed_loadIds;
    for(size_t objI = 0u; objI < expected_num_objectives; objI++)
    {
        const std::vector<std::string> thisObj_loadIds = tester.getObjLoadIds(objI);

        // expected number of loads is either 1 or 2
        const size_t num_load_ids = thisObj_loadIds.size();
        EXPECT_EQ((num_load_ids==1u) || (num_load_ids==2u), true);

        // concatenate
        distributed_loadIds.insert(distributed_loadIds.end(), thisObj_loadIds.begin(), thisObj_loadIds.end());
    }

    // expect maintenance of num of loads in distributed loads
    const size_t original_num_loads = 4u;
    ASSERT_EQ(distributed_loadIds.size(), original_num_loads);
    std::vector<bool> wasMaintained(12u, false);
    for(size_t di = 0u; di < original_num_loads; di++)
    {
        wasMaintained[std::atoi(distributed_loadIds[di].c_str())] = true;
    }

    // expect each load id was maintained
    EXPECT_EQ(wasMaintained[2], true);
    EXPECT_EQ(wasMaintained[5], true);
    EXPECT_EQ(wasMaintained[7], true);
    EXPECT_EQ(wasMaintained[11], true);
}

TEST(PlatoTestXMLGenerator,distributeObjective_exactlyEnoughProcessors)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 2 6 4 8\n"
            "    boundary condition ids 256\n"
            "    code sierra_sd\n"
            "    number processors 2\n"
            "    multi load case true\n"
            "    weight 1 \n"
            "    distribute objective at most 8 processors\n"
            "end objective\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);

    // BEFORE modification

    // one objective
    ASSERT_EQ(tester.getNumObjectives(), 1u);

    // do expand
    EXPECT_EQ(tester.publicDistributeObjectivesForGenerate(), true);

    // AFTER modification

    // four objectives
    const size_t expected_num_objectives = 4u;
    ASSERT_EQ(tester.getNumObjectives(), expected_num_objectives);

    // check names
    EXPECT_EQ(tester.getObjectiveName(0u), "1");
    EXPECT_EQ(tester.getObjectiveName(1u), "2");
    EXPECT_EQ(tester.getObjectiveName(2u), "3");
    EXPECT_EQ(tester.getObjectiveName(3u), "4");
    EXPECT_EQ(tester.getObjPerfName(0u), "sierra_sd_1");
    EXPECT_EQ(tester.getObjPerfName(1u), "sierra_sd_2");
    EXPECT_EQ(tester.getObjPerfName(2u), "sierra_sd_3");
    EXPECT_EQ(tester.getObjPerfName(3u), "sierra_sd_4");

    // get loadIds for distributed objective
    std::vector<std::string> distributed_loadIds;
    for(size_t objI = 0u; objI < expected_num_objectives; objI++)
    {
        const std::vector<std::string> thisObj_loadIds = tester.getObjLoadIds(objI);

        // expected number of loads is 1
        const size_t num_load_ids = thisObj_loadIds.size();
        EXPECT_EQ(num_load_ids,1u);

        // concatenate
        distributed_loadIds.insert(distributed_loadIds.end(), thisObj_loadIds.begin(), thisObj_loadIds.end());
    }

    // expect maintenance of num of loads in distributed loads
    const size_t original_num_loads = 4u;
    ASSERT_EQ(distributed_loadIds.size(), original_num_loads);
    std::vector<bool> wasMaintained(12u, false);
    for(size_t di = 0u; di < original_num_loads; di++)
    {
        wasMaintained[std::stoi(distributed_loadIds[di])] = true;
    }

    // expect each load id was maintained
    EXPECT_EQ(wasMaintained[2], true);
    EXPECT_EQ(wasMaintained[6], true);
    EXPECT_EQ(wasMaintained[4], true);
    EXPECT_EQ(wasMaintained[8], true);
}

TEST(PlatoTestXMLGenerator,distributeObjective_moreThanEnoughProcessors)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 1 7 3 4\n"
            "    boundary condition ids 256\n"
            "    code sierra_sd\n"
            "    number processors 2\n"
            "    multi load case true\n"
            "    weight 1 \n"
            "    distribute objective at most 9 processors\n"
            "end objective\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);

    // BEFORE modification

    // one objective
    ASSERT_EQ(tester.getNumObjectives(), 1u);

    // do expand
    EXPECT_EQ(tester.publicDistributeObjectivesForGenerate(), true);

    // AFTER modification

    // four objectives
    const size_t expected_num_objectives = 4u;
    ASSERT_EQ(tester.getNumObjectives(), expected_num_objectives);

    // check names
    EXPECT_EQ(tester.getObjectiveName(0u), "1");
    EXPECT_EQ(tester.getObjectiveName(1u), "2");
    EXPECT_EQ(tester.getObjectiveName(2u), "3");
    EXPECT_EQ(tester.getObjectiveName(3u), "4");
    EXPECT_EQ(tester.getObjPerfName(0u), "sierra_sd_1");
    EXPECT_EQ(tester.getObjPerfName(1u), "sierra_sd_2");
    EXPECT_EQ(tester.getObjPerfName(2u), "sierra_sd_3");
    EXPECT_EQ(tester.getObjPerfName(3u), "sierra_sd_4");

    // get loadIds for distributed objective
    std::vector<std::string> distributed_loadIds;
    for(size_t objI = 0u; objI < expected_num_objectives; objI++)
    {
        const std::vector<std::string> thisObj_loadIds = tester.getObjLoadIds(objI);

        // expected number of loads is 1
        const size_t num_load_ids = thisObj_loadIds.size();
        EXPECT_EQ(num_load_ids,1u);

        // concatenate
        distributed_loadIds.insert(distributed_loadIds.end(), thisObj_loadIds.begin(), thisObj_loadIds.end());
    }

    // expect maintenance of num of loads in distributed loads
    const size_t original_num_loads = 4u;
    ASSERT_EQ(distributed_loadIds.size(), original_num_loads);
    std::vector<bool> wasMaintained(12u, false);
    for(size_t di = 0u; di < original_num_loads; di++)
    {
        wasMaintained[std::atoi(distributed_loadIds[di].c_str())] = true;
    }

    // expect each load id was maintained
    EXPECT_EQ(wasMaintained[1], true);
    EXPECT_EQ(wasMaintained[7], true);
    EXPECT_EQ(wasMaintained[3], true);
    EXPECT_EQ(wasMaintained[4], true);
}

TEST(PlatoTestXMLGenerator, uncertainty_analyzeNewWorkflow)
{
    // POSE PROBLEM
  XMLGenerator_UnitTester tTester;
  std::istringstream iss;
  std::string stringInput =
  "begin objective\n"
  "   type maximize stiffness\n"
  "   load ids 10\n"
  "   boundary condition ids 11\n"
  "   code plato_analyze\n"
  "   number processors 1\n"
  "   weight 1\n"
  "   analyze new workflow true\n"
  "   number ranks 1\n"
  "end objective\n"
  "begin boundary conditions\n"
  "   fixed displacement nodeset name 1 bc id 11\n"
  "end boundary conditions\n"
  "begin loads\n"
  "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
  "end loads\n"
  "begin uncertainty\n"
  "    tag angle variation\n"
  "    load id 10\n"
  "    attribute X\n"
  "    distribution beta\n"
  "    mean 0.0\n"
  "    upper bound 45.0\n"
  "    lower bound -45.0\n"
  "    standard deviation 22.5\n"
  "    num samples 2\n"
  "end uncertainty\n";
  // do parse
  iss.str(stringInput);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tTester.publicParseObjectives(iss), true);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tTester.publicParseLoads(iss), true);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tTester.publicParseBCs(iss), true);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tTester.publicParseUncertainties(iss), true);
  EXPECT_EQ(tTester.publicRunSROMForUncertainVariables(), true);
  EXPECT_EQ(tTester.publicDistributeObjectivesForGenerate(), true);

  auto tXMLGenMetadata = tTester.getInputData();
  auto tNumSamples = tXMLGenMetadata.mRandomMetaData.numSamples();
  size_t numPeformers = tTester.getNumPerformers();
  EXPECT_EQ(tNumSamples,2u);
  EXPECT_EQ(numPeformers,1u);

  // TEST SAMPLES
  std::vector<std::string> tGoldLoadCaseProbabilities = { "0.36112468067266207", "0.63887286897558715" };
  std::vector<std::vector<std::string>> tGoldValues =
      {
        { "0.000000000000000000000e+00", "-4.751921387767659325618e+04", "1.555391630579348566243e+04" },
        { "0.000000000000000000000e+00", "-4.813588076578034088016e+04", "-1.352541987897522631101e+04"}
      };

  const double tTolerance = 1e-10;
  auto tSamples = tXMLGenMetadata.mRandomMetaData.samples();
  for(auto& tSample : tSamples)
  {
      auto tSampleIndex = &tSample - &tSamples[0];
      ASSERT_NEAR(std::stod(tGoldLoadCaseProbabilities[tSampleIndex]), std::stod(tSample.probability()), tTolerance);

      for(auto& tLoad : tSample.load().loads)
      {
          ASSERT_STREQ("traction", tLoad.type.c_str());
          for(auto& tValue : tLoad.values)
          {
              auto tComponent = &tValue - &tLoad.values[0];
              ASSERT_NEAR(std::stod(tValue), std::stod(tGoldValues[tSampleIndex][tComponent]), tTolerance);
          }
      }
  }

  size_t numObjectives = tTester.getNumObjectives();
  EXPECT_EQ(numObjectives, 1u);
}

TEST(PlatoTestXMLGenerator,uncertainty_analyzeNewWorkflow_randomPlusDeterministic)
{
    // POSE INPUT DATA
    XMLGenerator_UnitTester tTester;
    std::istringstream tInputSS;
    std::string tStringInput =
    "begin objective\n"
    "   type maximize stiffness\n"
    "   load ids 10 1\n"
    "   boundary condition ids 11\n"
    "   code plato_analyze\n"
    "   number processors 1\n"
    "   weight 1\n"
    "   analyze new workflow true\n"
    "   number ranks 5\n"
    "end objective\n"
    "begin boundary conditions\n"
    "   fixed displacement nodeset name 1 bc id 11\n"
    "end boundary conditions\n"
    "begin loads\n"
    "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
    "    traction sideset name 3 value 0 -5e4 0 load id 1\n"
    "end loads\n"
    "begin uncertainty\n"
    "    tag angle variation\n"
    "    load id 10\n"
    "    attribute X\n"
    "    distribution beta\n"
    "    mean 0.0\n"
    "    upper bound 45.0\n"
    "    lower bound -45.0\n"
    "    standard deviation 22.5\n"
    "    num samples 10\n"
    "end uncertainty\n";
    // do parse
    tInputSS.str(tStringInput);
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_EQ(tTester.publicParseObjectives(tInputSS), true);
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_EQ(tTester.publicParseLoads(tInputSS), true);
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_EQ(tTester.publicParseBCs(tInputSS), true);
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_EQ(tTester.publicParseUncertainties(tInputSS), true);
    EXPECT_EQ(tTester.publicRunSROMForUncertainVariables(), true);
    EXPECT_EQ(tTester.publicDistributeObjectivesForGenerate(), true);

    // TEST DATA
    auto tXMLGenMetadata = tTester.getInputData();
    auto tNumSamples = tXMLGenMetadata.mRandomMetaData.numSamples();
    EXPECT_EQ(tNumSamples,10u);
    size_t tNumPeformers = tTester.getNumPerformers();
    EXPECT_EQ(tNumPeformers,5u);

    // POSE GOLD VALUES
    std::vector<std::string> tGoldLoadCaseProbabilities =
        {"0.094172629104440519", "0.096118338919238849", "0.099663940442524482", "0.10447870334065364" , "0.10564855564584232",
         "0.10576322174283935" , "0.10486290959769146" , "0.099755080570144233", "0.095660857410530639", "0.093878478812968374"};

    std::vector<std::vector<std::vector<std::string>>> tGoldLoadValues =
      {
        { { "0.000000000000000000000e+00", "-4.008042184600126347505e+04", "2.989247036707714141812e+04" },
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } },

        { { "0.000000000000000000000e+00", "-4.410827789159363601357e+04", "2.354697053631213930203e+04" },
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } },

        { { "0.000000000000000000000e+00", "-4.704161308330694009783e+04", "1.694363120822820928879e+04" },
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } },

        { { "0.000000000000000000000e+00", "-4.894831379618879145710e+04", "1.020110663162747732713e+04" },
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } },

        { { "0.000000000000000000000e+00", "-4.988485492991151841125e+04", "3.391349085789085165743e+03" },
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } },

        { { "0.000000000000000000000e+00", "-4.988284696095977415098e+04", "-3.420757090099503329839e+03"},
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } },

        { { "0.000000000000000000000e+00", "-4.894769754769306018716e+04", "-1.020406315050838020397e+04"},
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } },

        { { "0.000000000000000000000e+00", "-4.706038546730067173485e+04", "-1.689142148160054057371e+04"},
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } },

        { { "0.000000000000000000000e+00", "-4.418230226249388215365e+04", "-2.340778004821533613722e+04"},
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } },

        { { "0.000000000000000000000e+00", "-4.025755085941775905667e+04", "-2.965349218559918881510e+04"},
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } }
      };

    // TEST SAMPLES
    constexpr double tTolerance = 1e-10;
    auto tSamples = tXMLGenMetadata.mRandomMetaData.samples();
    for (auto &tSample : tSamples)
    {
        auto tSampleIndex = &tSample - &tSamples[0];
        ASSERT_NEAR(std::stod(tGoldLoadCaseProbabilities[tSampleIndex]), std::stod(tSample.probability()), tTolerance);

        auto tLoadCase = tSample.load();
        for (auto &tLoad : tLoadCase.loads)
        {
            auto tLoadIndex = &tLoad - &tLoadCase.loads[0];
            ASSERT_STREQ("traction", tLoad.type.c_str());
            for (auto &tValue : tLoad.values)
            {
                auto tComponent = &tValue - &tLoad.values[0];
                ASSERT_NEAR(std::stod(tValue), std::stod(tGoldLoadValues[tSampleIndex][tLoadIndex][tComponent]), tTolerance);
            }
        }
    }

    const size_t tNumObjectives = tTester.getNumObjectives();
    ASSERT_EQ(tNumObjectives, 1u);
}

TEST(PlatoTestXMLGenerator,uncertainty_analyzeOldWorkflow)
{
    // POSE INPUT DATA
    XMLGenerator_UnitTester tTester;
    std::istringstream iss;
    std::string stringInput =
    "begin objective\n"
    "   type maximize stiffness\n"
    "   load ids 10\n"
    "   boundary condition ids 11\n"
    "   code plato_analyze\n"
    "   number processors 1\n"
    "   weight 1\n"
    "   multi load case true\n"
    "   distribute objective at most 3 processors\n"
    "end objective\n"
    "begin boundary conditions\n"
    "   fixed displacement nodeset name 1 bc id 11\n"
    "end boundary conditions\n"
    "begin loads\n"
    "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
    "end loads\n"
    "begin uncertainty\n"
    "    tag angle variation\n"
    "    load id 10\n"
    "    attribute X\n"
    "    distribution beta\n"
    "    mean 0.0\n"
    "    upper bound 45.0\n"
    "    lower bound -45.0\n"
    "    standard deviation 22.5\n"
    "    num samples 3\n"
    "end uncertainty\n";

    // PARSE INPUT DATA
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_TRUE(tTester.publicParseObjectives(iss));
    iss.clear();
    iss.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(iss));
    iss.clear();
    iss.seekg(0);
    EXPECT_TRUE(tTester.publicParseBCs(iss));
    iss.clear();
    iss.seekg(0);
    EXPECT_TRUE(tTester.publicParseUncertainties(iss));
    EXPECT_TRUE(tTester.publicRunSROMForUncertainVariables());
    EXPECT_TRUE(tTester.publicDistributeObjectivesForGenerate());

    // GET PARSED INPUT DATA
    auto tXMLGenMetadata = tTester.getInputData();
    auto tNumSamples = tXMLGenMetadata.mRandomMetaData.numSamples();
    ASSERT_EQ(3u, tNumSamples);
    size_t tNumPeformers = tTester.getNumPerformers();
    ASSERT_EQ(3u, tNumPeformers);

    // POSE GOLD VALUES
    std::vector<std::string> tGoldLoadCaseProbabilities =
        { "0.36406606886994869", "0.27079380819551258", "0.36514142000522448"};

    std::vector<std::vector<std::string>> tGoldValues =
      { { "0.000000000000000000000e+00", "-4.534825437206416972913e+04", "2.106029024981761904201e+04" },
        { "0.000000000000000000000e+00", "-4.999912137119322869694e+04", "-2.964154326084487820481e+02" },
        { "0.000000000000000000000e+00", "-4.486609665898913226556e+04", "-2.206883256056478785467e+04" } };

    // TEST RESULTS
    constexpr double tTolerance = 1e-8;
    auto tSamples = tXMLGenMetadata.mRandomMetaData.samples();
    for (auto &tSample : tSamples)
    {
        auto tSampleIndex = &tSample - &tSamples[0];
        ASSERT_NEAR(std::stod(tGoldLoadCaseProbabilities[tSampleIndex]), std::stod(tSample.probability()), tTolerance);

        auto tLoadCase = tSample.load();
        for (auto &tLoad : tLoadCase.loads)
        {
            ASSERT_STREQ("traction", tLoad.type.c_str());
            for (auto &tValue : tLoad.values)
            {
                auto tDim = &tValue - &tLoad.values[0];
                ASSERT_NEAR(std::stod(tGoldValues[tSampleIndex][tDim]), std::stod(tValue), tTolerance);
            }
        }
    }

    auto tNumObjectives = tTester.getNumObjectives();
    ASSERT_EQ(3u, tNumObjectives);

    ASSERT_EQ(tNumSamples % tNumObjectives, 0u);
}

TEST(PlatoTestXMLGenerator, uncertainty_sierra)
{
    // POSE INPUT DATA
    XMLGenerator_UnitTester tTester;
    std::istringstream iss;
    std::string stringInput =
    "begin objective\n"
    "   type maximize stiffness\n"
    "   load ids 10\n"
    "   boundary condition ids 11\n"
    "   code sierra_sd\n"
    "   number processors 1\n"
    "   weight 1\n"
    "   multi load case true\n"
    "   distribute objective at most 3 processors\n"
    "end objective\n"
    "begin boundary conditions\n"
    "   fixed displacement nodeset name 1 bc id 11\n"
    "end boundary conditions\n"
    "begin loads\n"
    "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
    "end loads\n"
    "begin uncertainty\n"
    "    tag angle variation\n"
    "    load id 10\n"
    "    attribute X\n"
    "    distribution beta\n"
    "    mean 0.0\n"
    "    upper bound 45.0\n"
    "    lower bound -45.0\n"
    "    standard deviation 22.5\n"
    "    num samples 3\n"
    "end uncertainty\n";

    // PARSE INPUT DATA
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_TRUE(tTester.publicParseObjectives(iss));
    iss.clear();
    iss.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(iss));
    iss.clear();
    iss.seekg(0);
    EXPECT_TRUE(tTester.publicParseBCs(iss));
    iss.clear();
    iss.seekg(0);
    EXPECT_TRUE(tTester.publicParseUncertainties(iss));
    EXPECT_TRUE(tTester.publicRunSROMForUncertainVariables());
    EXPECT_TRUE(tTester.publicDistributeObjectivesForGenerate());

    // GET PARSED INPUT DATA
    auto tXMLGenMetadata = tTester.getInputData();
    auto tNumSamples = tXMLGenMetadata.mRandomMetaData.numSamples();
    EXPECT_EQ(3u, tNumSamples);
    size_t numPeformers = tTester.getNumPerformers();
    EXPECT_EQ(3u, numPeformers);

    // POSE GOLD VALUES
    std::vector<std::string> tGoldLoadCaseProbabilities =
        { "0.36406606886994869", "0.27079380819551258", "0.36514142000522448"};

    std::vector<std::vector<std::string>> tGoldLoadValues =
      { { "0.000000000000000000000e+00", "-4.534825437206416972913e+04", "2.106029024981761904201e+04" },
        { "0.000000000000000000000e+00", "-4.999912137119322869694e+04", "-2.964154326084487820481e+02" },
        { "0.000000000000000000000e+00", "-4.486609665898913226556e+04", "-2.206883256056478785467e+04" } };

    // TEST RESULTS
    constexpr double tTolerance = 1e-8;
    auto tSamples = tXMLGenMetadata.mRandomMetaData.samples();
    for (auto &tSample : tSamples)
    {
        auto tSampleIndex = &tSample - &tSamples[0];
        ASSERT_NEAR(std::stod(tGoldLoadCaseProbabilities[tSampleIndex]), std::stod(tSample.probability()), tTolerance);

        auto tLoadCase = tSample.load();
        for (auto &tLoad : tLoadCase.loads)
        {
            ASSERT_EQ("traction", tLoad.type);
            for (auto &tValue : tLoad.values)
            {
                auto tDim = &tValue - &tLoad.values[0];
                ASSERT_NEAR(std::stod(tGoldLoadValues[tSampleIndex][tDim]), std::stod(tValue), tTolerance);
            }
        }
    }

    auto tNumObjectives = tTester.getNumObjectives();
    ASSERT_EQ(3u, tNumObjectives);
    ASSERT_EQ(tNumSamples % tNumObjectives, 0u);
}

TEST(PlatoTestXMLGenerator,generatePlatoAnalyzeInputDeck_mechanical_valid)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 1\n"
            "    boundary condition ids 1 2 3\n"
            "    code plato_analyze\n"
            "    number processors 2\n"
            "end objective\n"
            "begin loads\n"
            "    traction sideset name 2 value 0 -3e3 0 load id 1\n"
            "end loads\n"
            "begin material 1\n"
            "    poissons ratio 0.3\n"
            "    youngs modulus 1e8\n"
            "end material\n"
            "begin boundary conditions\n"
            "    fixed displacement nodeset name 1 bc id 1\n"
            "    fixed displacement nodeset name 1 x bc id 2\n"
            "    fixed displacement nodeset name 1 y 3.0 bc id 3\n"
            "end boundary conditions\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseMaterials(iss), true);
    const XMLGen::InputData& tInputData = tester.getInputData();
    DefaultInputGenerator_UnitTester tGenerator(tInputData);;
    std::ostringstream tOStringStream;
    EXPECT_EQ(tGenerator.publicGeneratePlatoAnalyzeInputDecks(&tOStringStream), true);
    EXPECT_EQ(tOStringStream.str(), gMechanicalGoldString);
}

TEST(PlatoTestXMLGenerator,generatePlatoAnalyzeInputDeckNewWriter_mechanical_valid)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 1\n"
            "    boundary condition ids 1 2 3\n"
            "    code plato_analyze\n"
            "    number processors 2\n"
            "end objective\n"
            "begin loads\n"
            "    traction sideset name 2 value 0 -3e3 0 load id 1\n"
            "end loads\n"
            "begin material 1\n"
            "    poissons ratio 0.3\n"
            "    youngs modulus 1e8\n"
            "end material\n"
            "begin boundary conditions\n"
            "    fixed displacement nodeset name 1 bc id 1\n"
            "    fixed displacement nodeset name 1 x bc id 2\n"
            "    fixed displacement nodeset name 1 y 3.0 bc id 3\n"
            "end boundary conditions\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseMaterials(iss), true);
    const XMLGen::InputData& tInputData = tester.getInputData();
    ComplianceMinTOPlatoAnalyzeInputGenerator_UnitTester tGenerator(tInputData);;
    std::ostringstream tOStringStream;
    EXPECT_EQ(tGenerator.publicGeneratePlatoAnalyzeInputDecks(&tOStringStream), true);
    EXPECT_EQ(tOStringStream.str(), gMechanicalGoldString);
}

TEST(PlatoTestXMLGenerator,generateInterfaceXMLWithCompMinTOPlatoAnalyzeWriter)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 1\n"
            "    boundary condition ids 1 2 3\n"
            "    code plato_analyze\n"
            "    number processors 2\n"
            "   analyze new workflow true\n"
            "end objective\n"
            "begin loads\n"
            "    traction sideset name 2 value 0 -3e3 0 load id 1\n"
            "end loads\n"
            "begin material 1\n"
            "    poissons ratio 0.3\n"
            "    youngs modulus 1e8\n"
            "end material\n"
            "begin boundary conditions\n"
            "    fixed displacement nodeset name 1 bc id 1\n"
            "    fixed displacement nodeset name 1 x bc id 2\n"
            "    fixed displacement nodeset name 1 y 3.0 bc id 3\n"
            "end boundary conditions\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseMaterials(iss), true);
    tester.publicLookForPlatoAnalyzePerformers();
    const XMLGen::InputData& tInputData = tester.getInputData();
    ComplianceMinTOPlatoAnalyzeInputGenerator_UnitTester tGenerator(tInputData);;
    std::ostringstream tOStringStream;
    EXPECT_EQ(tGenerator.publicGenerateInterfaceXML(&tOStringStream), true);
    EXPECT_EQ(tOStringStream.str(), gInterfaceXMLCompMinTOPAGoldString);
}

TEST(PlatoTestXMLGenerator,generateInterfaceXMLWithCompMinTOPlatoAnalyzeUncertWriter)
{
    // Generate interface.xml using the new writer
    XMLGenerator_UnitTester tester_new;
    std::istringstream iss;
    std::string stringInput =
        "begin objective\n"
        "   type maximize stiffness\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1\n"
        "   analyze new workflow true\n"
        "   number ranks 1\n"
        "end objective\n"
        "begin boundary conditions\n"
        "   fixed displacement nodeset name 1 bc id 11\n"
        "end boundary conditions\n"
        "begin loads\n"
        "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
        "end loads\n"
        "begin material 1\n"
            "penalty exponent 3\n"
            "youngs modulus 1e6\n"
            "poissons ratio 0.33\n"
            "thermal conductivity .02\n"
            "density .001\n"
        "end material\n"
        "begin block 1\n"
        "   material 1\n"
        "end block\n"
        "begin uncertainty\n"
        "    tag angle variation\n"
        "    load id 10\n"
        "    attribute X\n"
        "    distribution beta\n"
        "    mean 0.0\n"
        "    upper bound 45.0\n"
        "    lower bound -45.0\n"
        "    standard deviation 22.5\n"
        "    num samples 2\n"
        "end uncertainty\n"
        "begin optimization parameters\n"
        "end optimization parameters\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester_new.publicParseObjectives(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester_new.publicParseLoads(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester_new.publicParseBCs(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester_new.publicParseBlocks(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester_new.publicParseMaterials(iss), true);
    iss.clear();
    iss.seekg(0);
    tester_new.publicParseOptimizationParameters(iss);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester_new.publicParseUncertainties(iss), true);
    tester_new.publicGetUncertaintyFlags();
    EXPECT_EQ(tester_new.publicRunSROMForUncertainVariables(), true);
    EXPECT_EQ(tester_new.publicDistributeObjectivesForGenerate(), true);
    tester_new.publicLookForPlatoAnalyzePerformers();
    const XMLGen::InputData& tInputData = tester_new.getInputData();
    ComplianceMinTOPlatoAnalyzeUncertInputGenerator_UnitTester tGenerator(tInputData);;
    std::ostringstream tOStringStream;
 //   EXPECT_EQ(tGenerator.publicGenerateInterfaceXML(), true);
    EXPECT_EQ(tGenerator.publicGenerateInterfaceXML(&tOStringStream), true);
    std::ostringstream tInputDeckStreamNew;
    EXPECT_EQ(tGenerator.publicGeneratePlatoAnalyzeInputDecks(&tInputDeckStreamNew), true);
    

    // Generate interface.xml using the old writer
    XMLGenerator_UnitTester tester_old;
    std::istringstream iss_old;
    std::string stringInput_old =
        "begin objective\n"
        "   type maximize stiffness\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1\n"
        "   analyze new workflow true\n"
        "   number ranks 1\n"
        "end objective\n"
        "begin boundary conditions\n"
        "   fixed displacement nodeset name 1 bc id 11\n"
        "end boundary conditions\n"
        "begin loads\n"
        "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
        "end loads\n"
        "begin material 1\n"
            "penalty exponent 3\n"
            "youngs modulus 1e6\n"
            "poissons ratio 0.33\n"
            "thermal conductivity .02\n"
            "density .001\n"
        "end material\n"
        "begin block 1\n"
        "   material 1\n"
        "end block\n"
        "begin uncertainty\n"
        "    tag angle variation\n"
        "    load id 10\n"
        "    attribute X\n"
        "    distribution beta\n"
        "    mean 0.0\n"
        "    upper bound 45.0\n"
        "    lower bound -45.0\n"
        "    standard deviation 22.5\n"
        "    num samples 2\n"
        "end uncertainty\n"
        "begin optimization parameters\n"
        "    input generator version old\n"
        "end optimization parameters\n";

    // do parse
    iss_old.str(stringInput_old);
    iss_old.clear();
    iss_old.seekg(0);
    EXPECT_EQ(tester_old.publicParseObjectives(iss_old), true);
    iss_old.clear();
    iss_old.seekg(0);
    EXPECT_EQ(tester_old.publicParseLoads(iss_old), true);
    iss_old.clear();
    iss_old.seekg(0);
    EXPECT_EQ(tester_old.publicParseBCs(iss_old), true);
    iss_old.clear();
    iss_old.seekg(0);
    EXPECT_EQ(tester_old.publicParseMaterials(iss_old), true);
    iss_old.clear();
    iss_old.seekg(0);
    EXPECT_EQ(tester_old.publicParseBlocks(iss_old), true);
    iss_old.clear();
    iss_old.seekg(0);
    tester_old.publicParseOptimizationParameters(iss_old);
    iss_old.clear();
    iss_old.seekg(0);
    EXPECT_EQ(tester_old.publicParseUncertainties(iss_old), true);
    tester_old.publicGetUncertaintyFlags();
    EXPECT_EQ(tester_old.publicRunSROMForUncertainVariables(), true);
    EXPECT_EQ(tester_old.publicDistributeObjectivesForGenerate(), true);
    tester_old.publicLookForPlatoAnalyzePerformers();
    const XMLGen::InputData& tInputData_old = tester_old.getInputData();
    DefaultInputGenerator_UnitTester tGenerator_old(tInputData_old);
    std::ostringstream tOStringStream_old;
 //   EXPECT_EQ(tGenerator_old.publicGenerateInterfaceXML(), true);
    EXPECT_EQ(tGenerator_old.publicGenerateInterfaceXML(&tOStringStream_old), true);
    std::ostringstream tInputDeckStreamOld;
    EXPECT_EQ(tGenerator_old.publicGeneratePlatoAnalyzeInputDecks(&tInputDeckStreamOld), true);

    EXPECT_EQ(tOStringStream_old.str(), tOStringStream.str());
    EXPECT_EQ(tInputDeckStreamOld.str(), tInputDeckStreamNew.str());
}

TEST(PlatoTestXMLGenerator,generatePlatoAnalyzeInputDeck_mechanical_duplicate_names)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 1\n"
            "    boundary condition ids 1 2 3\n"
            "    code plato_analyze\n"
            "    number processors 2\n"
            "end objective\n"
            "begin loads\n"
            "    traction sideset name 1 value 0 -3e3 0 load id 1\n"
            "end loads\n"
            "begin material 1\n"
            "    poissons ratio 0.3\n"
            "    youngs modulus 1e8\n"
            "end material\n"
            "begin boundary conditions\n"
            "    fixed displacement nodeset name 1 bc id 1\n"
            "    fixed displacement nodeset name 1 x bc id 2\n"
            "    fixed displacement nodeset name 1 y 3.0 bc id 3\n"
            "end boundary conditions\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseMaterials(iss), true);

    const XMLGen::InputData& tInputData = tester.getInputData();
    DefaultInputGenerator_UnitTester tGenerator(tInputData);
    std::ostringstream tOStringStream;
    EXPECT_EQ(tGenerator.publicGeneratePlatoAnalyzeInputDecks(&tOStringStream), false);
    EXPECT_EQ(tOStringStream.str(), "");

}
TEST(PlatoTestXMLGenerator,generatePlatoAnalyzeInputDeckNewWriter_mechanical_duplicate_names)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 1\n"
            "    boundary condition ids 1 2 3\n"
            "    code plato_analyze\n"
            "    number processors 2\n"
            "end objective\n"
            "begin loads\n"
            "    traction sideset name 1 value 0 -3e3 0 load id 1\n"
            "end loads\n"
            "begin material 1\n"
            "    poissons ratio 0.3\n"
            "    youngs modulus 1e8\n"
            "end material\n"
            "begin boundary conditions\n"
            "    fixed displacement nodeset name 1 bc id 1\n"
            "    fixed displacement nodeset name 1 x bc id 2\n"
            "    fixed displacement nodeset name 1 y 3.0 bc id 3\n"
            "end boundary conditions\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseMaterials(iss), true);

    const XMLGen::InputData& tInputData = tester.getInputData();
    ComplianceMinTOPlatoAnalyzeInputGenerator_UnitTester tGenerator(tInputData);
    std::ostringstream tOStringStream;
    EXPECT_EQ(tGenerator.publicGeneratePlatoAnalyzeInputDecks(&tOStringStream), false);
    EXPECT_EQ(tOStringStream.str(), "");

}

TEST(PlatoTestXMLGenerator,generatePlatoAnalyzeInputDeck_thermal)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize heat conduction\n"
            "    load ids 1\n"
            "    boundary condition ids 1 2 3\n"
            "    code plato_analyze\n"
            "    number processors 2\n"
            "end objective\n"
            "begin loads\n"
            "    heat flux sideset name ss_1 value -1e2 load id 1\n"
            "end loads\n"
            "begin material 1\n"
            "    density 2703\n"
            "    specific heat 900\n"
            "    thermal conductivity 210.0\n"
            "end material\n"
            "begin boundary conditions\n"
            "    fixed temperature nodeset name 1 bc id 1\n"
            "    fixed temperature nodeset name 2 bc id 2\n"
            "    fixed temperature nodeset name 3 value 25.0 bc id 3\n"
            "end boundary conditions\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseMaterials(iss), true);

    const XMLGen::InputData& tInputData = tester.getInputData();
    DefaultInputGenerator_UnitTester tGenerator(tInputData);
    std::ostringstream tOStringStream;
    EXPECT_EQ(tGenerator.publicGeneratePlatoAnalyzeInputDecks(&tOStringStream), true);
    EXPECT_EQ(tOStringStream.str(), gThermalGoldString);

}

TEST(PlatoTestXMLGenerator,generatePlatoAnalyzeInputDeck_thermoelastic)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type minimize thermoelastic energy\n"
            "    load ids 1 2\n"
            "    boundary condition ids 1 2 3 4 5 6 7 8\n"
            "    code plato_analyze\n"
            "    number processors 2\n"
            "end objective\n"
            "begin loads\n"
            "    traction sideset name ss_1 value 0.0 1.0e5 0.0 load id 1\n"
            "    heat flux sideset name ss_1 value 0.0 load id 2\n"
            "end loads\n"
            "begin material 1\n"
            "    poissons ratio 0.3\n"
            "    youngs modulus 1e11\n"
            "    thermal expansion coefficient 1e-5\n"
            "    thermal conductivity 910.0\n"
            "    reference temperature 1e-2\n"
            "end material\n"
            "begin boundary conditions\n"
            "    fixed displacement nodeset name 1 y bc id 1\n"
            "    fixed displacement nodeset name 1 z bc id 2\n"
            "    fixed temperature nodeset name 1 bc id 3\n"
            "    fixed displacement nodeset name 11 x bc id 4\n"
            "    fixed displacement nodeset name 2 y bc id 5\n"
            "    fixed displacement nodeset name 2 z bc id 6\n"
            "    fixed temperature nodeset name 2 bc id 7\n"
            "    fixed displacement nodeset name 21 x bc id 8\n"
            "end boundary conditions\n";



    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseMaterials(iss), true);

    const XMLGen::InputData& tInputData = tester.getInputData();
    DefaultInputGenerator_UnitTester tGenerator(tInputData);
    std::ostringstream tOStringStream;
    EXPECT_EQ(tGenerator.publicGeneratePlatoAnalyzeInputDecks(&tOStringStream), true);
    EXPECT_EQ(tOStringStream.str(), gThermomechanicalGoldString);

}

} // end PlatoTestXMLGenerator namespace
