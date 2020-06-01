/*
 * XMLGeneratorInterfaceFile_UnitTester.cpp
 *
 *  Created on: May 28, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorInterfaceFileUtilities.hpp"
#include "XMLGeneratorRandomInterfaceFileUtilities.hpp"

namespace PlatoTestXMLGenerator
{

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

TEST(PlatoTestXMLGenerator, AppendObjectiveSharedData_ErrorEmptyObjectiveList)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    ASSERT_THROW(XMLGen::append_objective_shared_data(tInputData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendObjectiveSharedData)
{
    pugi::xml_document tDocument;
    XMLGen::Objective tObjective;
    tObjective.performer_name = "plato analyze";
    XMLGen::InputData tInputData;
    tInputData.objectives.push_back(tObjective);

    ASSERT_NO_THROW(XMLGen::append_objective_shared_data(tInputData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tTemp = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataKeys;
    tGoldSharedDataKeys.push_back(std::make_pair("Objective Value", tTemp));
    tTemp = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tGoldSharedDataKeys.push_back(std::make_pair("Objective Gradient", tTemp));

    tTemp = {"Objective Value 0", "Scalar", "Global", "1", "plato analyze", "PlatoMain"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataValues;
    tGoldSharedDataValues.push_back(std::make_pair("Objective Value", tTemp));
    tTemp = {"Objective Gradient 0", "Scalar", "Nodal Field", "plato analyze", "PlatoMain"};
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

TEST(PlatoTestXMLGenerator, AppendConstraintSharedData)
{
    pugi::xml_document tDocument;
    XMLGen::Constraint tConstraint;
    tConstraint.mPerformerName = "plato analyze";
    XMLGen::InputData tInputData;
    tInputData.constraints.push_back(tConstraint);

    ASSERT_NO_THROW(XMLGen::append_constraint_shared_data(tInputData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tTemp = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataKeys;
    tGoldSharedDataKeys.push_back(std::make_pair("Constraint Value", tTemp));
    tTemp = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tGoldSharedDataKeys.push_back(std::make_pair("Constraint Gradient", tTemp));

    tTemp = {"Constraint Value 0", "Scalar", "Global", "1", "plato analyze", "PlatoMain"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataValues;
    tGoldSharedDataValues.push_back(std::make_pair("Constraint Value", tTemp));
    tTemp = {"Constraint Gradient 0", "Scalar", "Nodal Field", "plato analyze", "PlatoMain"};
    tGoldSharedDataValues.push_back(std::make_pair("Constraint Gradient", tTemp));

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

    auto tInput = tOperation.child("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);

    tInput = tInput.next_sibling("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Gradient", "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);
}

TEST(PlatoTestXMLGenerator, AppendFilterCriterionGradientOperation)
{
    pugi::xml_document tDocument;
    XMLGen::append_filter_criterion_gradient_operation("Objective Gradient", tDocument);
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

    auto tInput = tOperation.child("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);

    tInput = tInput.next_sibling("Input");
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
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Initialized Field", "Control"};
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
    XMLGen::append_nondeterministic_operation(tKeys, tValues, tDocument);
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
    ASSERT_FALSE(tStage.empty());
    std::vector<std::string> tGoldKeys = {"Name", "Input", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Set Upper Bounds", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOuterInput = tStage.child("Input");
    ASSERT_FALSE(tOuterInput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Upper Bound Value"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterInput);

    auto tOperation = tStage.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Calculate Upper Bounds", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    auto tInnerInput = tOperation.child("Input");
    ASSERT_FALSE(tInnerInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Upper Bound Vector", "Upper Bound Vector"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInnerInput);
    auto tInnerOutput = tOperation.child("Output");
    ASSERT_FALSE(tInnerOutput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Upper Bound Vector", "Upper Bound Vector"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInnerOutput);

    auto tOuterOutput = tStage.child("Output");
    ASSERT_FALSE(tOuterOutput.empty());
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
    ASSERT_FALSE(tStage.empty());
    std::vector<std::string> tGoldKeys = {"Name", "Output"};
    std::vector<std::string> tGoldValues = {"Set Upper Bounds", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOuterOutput = tStage.child("Output");
    ASSERT_FALSE(tOuterOutput.empty());
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
    ASSERT_FALSE(tStage.empty());
    std::vector<std::string> tGoldKeys = {"Name", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Calculate Design Domain Volume", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOperation = tStage.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Output"};
    tGoldValues = {"Calculate Design Domain Volume", "PlatoMain", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tInnerOutput = tOperation.child("Output");
    ASSERT_FALSE(tInnerOutput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Reference Value", "Reference Value"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInnerOutput);

    auto tOuterOutput = tStage.child("Output");
    ASSERT_FALSE(tOuterOutput.empty());
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
    ASSERT_FALSE(tStage.empty());
    std::vector<std::string> tGoldKeys = {"Name", "Type", "Input", "Operation", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Calculate Constraint Value 0", "volume", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOuterInput = tStage.child("Input");
    ASSERT_FALSE(tOuterInput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterInput);

    auto tOuterOutput = tStage.child("Output");
    ASSERT_FALSE(tOuterOutput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Constraint Value 0"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterOutput);

    auto tOperation = tStage.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Filter Control", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    auto tFilterInput = tOperation.child("Input");
    ASSERT_FALSE(tFilterInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterInput);
    auto tFilterOutput = tOperation.child("Output");
    ASSERT_FALSE(tFilterOutput.empty());
    tGoldValues = {"Filtered Field", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterOutput);

    tOperation = tOperation.next_sibling("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Calculate Constraint Value", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    auto tConstraintInput = tOperation.child("Input");
    ASSERT_FALSE(tConstraintInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Topology", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConstraintInput);
    auto tConstraintOutput = tOperation.child("Output");
    ASSERT_FALSE(tConstraintOutput.empty());
    tGoldValues = {"Constraint Value", "Constraint Value 0"};
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
    ASSERT_FALSE(tStage.empty());
    std::vector<std::string> tGoldKeys = {"Name", "Type", "Input", "Operation", "Operation", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Calculate Constraint Gradient 0", "volume", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOuterInput = tStage.child("Input");
    ASSERT_FALSE(tOuterInput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterInput);

    auto tOuterOutput = tStage.child("Output");
    ASSERT_FALSE(tOuterOutput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Constraint Gradient 0"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterOutput);

    auto tOperation = tStage.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Filter Control", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    auto tFilterInput = tOperation.child("Input");
    ASSERT_FALSE(tFilterInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterInput);
    auto tFilterOutput = tOperation.child("Output");
    ASSERT_FALSE(tFilterOutput.empty());
    tGoldValues = {"Filtered Field", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterOutput);

    tOperation = tOperation.next_sibling("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Calculate Constraint Gradient", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    auto tConstraintInput = tOperation.child("Input");
    ASSERT_FALSE(tConstraintInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Topology", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConstraintInput);
    auto tConstraintOutput = tOperation.child("Output");
    ASSERT_FALSE(tConstraintOutput.empty());
    tGoldValues = {"Constraint Gradient", "Constraint Gradient 0"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConstraintOutput);

    tOperation = tOperation.next_sibling("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Input", "Output"};
    tGoldValues = {"Filter Gradient", "PlatoMain", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    tFilterInput = tOperation.child("Input");
    ASSERT_FALSE(tFilterInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterInput);
    tFilterInput = tFilterInput.next_sibling("Input");
    ASSERT_FALSE(tFilterInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Gradient", "Constraint Gradient 0"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterInput);
    tFilterOutput = tOperation.child("Output");
    ASSERT_FALSE(tFilterOutput.empty());
    tGoldValues = {"Filtered Gradient", "Constraint Gradient 0"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterOutput);
}

TEST(PlatoTestXMLGenerator, AppendNondeterministicParameters)
{
    pugi::xml_document tDocument;
    auto tOperation = tDocument.append_child("Operation");
    std::unordered_map<std::string, std::vector<std::string>> tTags =
        { { "0", {"traction load-id-0 x-axis", "traction load-id-0 y-axis", "traction load-id-0 z-axis"} } };
    XMLGen::append_nondeterministic_parameters(tTags, tOperation);
    ASSERT_FALSE(tOperation.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tGoldKeys = {"Parameter", "Parameter", "Parameter"};
    std::vector<std::string> tGoldValues = {"", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tParameter = tOperation.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction load-id-0 x-axis",
                   "{traction load-id-0 x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction load-id-0 y-axis",
                   "{traction load-id-0 y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction load-id-0 z-axis",
                   "{traction load-id-0 z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);
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
    XMLGen::append_sample_objective_value_operation(tObjective.performer_name, tXMLMetaData, tStage);
    ASSERT_FALSE(tStage.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tGoldKeys = {"For"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);
    auto tFor = tStage.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tOperation = tFor.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tFor = tOperation.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tOperation = tFor.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Parameter", "Parameter", "Parameter",
        "Parameter", "Parameter", "Parameter", "Input", "Output"};
    tGoldValues = {"Compute Objective Value", "plato analyze {PerformerIndex}",
        "", "", "", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    // TEST PARAMETERS AGAINST GOLD VALUES
    auto tParameter = tOperation.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction load-id-1 x-axis",
                   "{traction load-id-1 x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction load-id-1 y-axis",
                   "{traction load-id-1 y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction load-id-1 z-axis",
                   "{traction load-id-1 z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction load-id-0 x-axis",
                   "{traction load-id-0 x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction load-id-0 y-axis",
                   "{traction load-id-0 y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction load-id-0 z-axis",
                   "{traction load-id-0 z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    // TEST OPERATION INPUTS AND OUTPUTS AGAINST GOLD VALUES
    auto tOperationInput = tOperation.child("Input");
    ASSERT_FALSE(tOperationInput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Topology", "Topology" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperationInput);

    auto tOperationOutput = tOperation.child("Output");
    ASSERT_FALSE(tOperationOutput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Objective Value", "Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperationOutput);
}

TEST(PlatoTestXMLGenerator, AppendEvaluateNondeterministicCriterionValueOperation)
{
    // CALL FUNCTION
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.objective_number_standard_deviations = "1";
    pugi::xml_document tDocument;
    auto tStage = tDocument.append_child("Stage");
    XMLGen::append_evaluate_nondeterministic_objective_value_operation("Objective Value", tXMLMetaData, tStage);
    ASSERT_FALSE(tStage.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tGoldKeys = {"Operation"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);
    auto tOperation = tStage.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "For", "Output"};
    tGoldValues = {"Calculate Non-Deterministic Objective Value", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tFor = tOperation.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tFor = tFor.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tInput = tFor.child("Input");
    ASSERT_FALSE(tInput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Mean Plus 1 StdDev", "Objective Value"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);
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
    tXMLMetaData.objective_number_standard_deviations = "1";

    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::append_objective_value_stage_for_nondeterministic_usecase(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // ****** 1) TEST RESULTS AGAINST STAGE GOLD VALUES ******
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    std::vector<std::string> tGoldKeys = {"Name", "Input", "Operation", "For", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Calculate Objective Value 0", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tStageInput= tStage.child("Input");
    ASSERT_FALSE(tStageInput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageInput);

    auto tStageOutput= tStage.child("Output");
    ASSERT_FALSE(tStageOutput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Objective Value 0"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageOutput);

    // ****** 2) TEST RESULTS AGAINST FILTER OPERATION GOLD VALUES ******
    auto tStageOperation = tStage.child("Operation");
    ASSERT_FALSE(tStageOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Filter Control", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageOperation);

    auto tFilterInput = tStageOperation.child("Input");
    ASSERT_FALSE(tFilterInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterInput);

    auto tFilterOutput = tStageOperation.child("Output");
    ASSERT_FALSE(tFilterOutput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Filtered Field", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterOutput);

    // ****** 4) TEST RESULTS AGAINST SAMPLE OBJECTIVE GOLD VALUES ******
    auto tFor = tStage.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tOperation = tFor.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tFor = tOperation.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tOperation = tFor.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Parameter", "Parameter", "Parameter",
        "Parameter", "Parameter", "Parameter", "Input", "Output"};
    tGoldValues = {"Compute Objective Value", "plato analyze {PerformerIndex}",
        "", "", "", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    // ****** 4.1) TEST PARAMETERS AGAINST GOLD VALUES ******
    auto tParameter = tOperation.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction load-id-1 x-axis",
                   "{traction load-id-1 x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction load-id-1 y-axis",
                   "{traction load-id-1 y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction load-id-1 z-axis",
                   "{traction load-id-1 z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction load-id-0 x-axis",
                   "{traction load-id-0 x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction load-id-0 y-axis",
                   "{traction load-id-0 y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction load-id-0 z-axis",
                   "{traction load-id-0 z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    // ****** 4.2) TEST SAMPLE OBJECTIVE OPERATION INPUTS AND OUTPUTS AGAINST GOLD VALUES ******
    auto tOperationInput = tOperation.child("Input");
    ASSERT_FALSE(tOperationInput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Topology", "Topology" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperationInput);

    auto tOperationOutput = tOperation.child("Output");
    ASSERT_FALSE(tOperationOutput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Objective Value", "Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperationOutput);

    // ****** 3) TEST RESULTS AGAINST EVALUATE RANDOM OBJECTIVE OPERATION GOLD VALUES ******
    tStageOperation = tStageOperation.next_sibling("Operation");
    ASSERT_FALSE(tStageOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "For", "Output"};
    tGoldValues = {"Calculate Non-Deterministic Objective Value", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageOperation);

    tFor = tStageOperation.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tFor = tFor.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tRandomObjectiveInput = tFor.child("Input");
    ASSERT_FALSE(tRandomObjectiveInput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tRandomObjectiveInput);

    auto tRandomObjectiveOutput = tStageOperation.child("Output");
    ASSERT_FALSE(tRandomObjectiveOutput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Mean Plus 1 StdDev", "Objective Value 0"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tRandomObjectiveOutput);
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
    ASSERT_FALSE(tFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tFor = tFor.child("Operation").child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tOperation = tFor.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Parameter", "Parameter", "Parameter",
        "Parameter", "Parameter", "Parameter", "Input", "Output"};
    tGoldValues = {"Compute Objective Gradient", "plato analyze {PerformerIndex}",
        "", "", "", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    // TEST PARAMETERS AGAINST GOLD VALUES
    auto tParameter = tOperation.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction load-id-1 x-axis",
                   "{traction load-id-1 x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction load-id-1 y-axis",
                   "{traction load-id-1 y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction load-id-1 z-axis",
                   "{traction load-id-1 z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction load-id-0 x-axis",
                   "{traction load-id-0 x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction load-id-0 y-axis",
                   "{traction load-id-0 y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction load-id-0 z-axis",
                   "{traction load-id-0 z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    // TEST OPERATION INPUTS AND OUTPUTS AGAINST GOLD VALUES
    auto tOperationInput = tOperation.child("Input");
    ASSERT_FALSE(tOperationInput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Topology", "Topology" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperationInput);

    auto tOperationOutput = tOperation.child("Output");
    ASSERT_FALSE(tOperationOutput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Objective Gradient", "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperationOutput);
}

TEST(PlatoTestXMLGenerator, AppendEvaluateNondeterministicCriterionGradientOperation)
{
    // CALL FUNCTION
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.objective_number_standard_deviations = "2";
    pugi::xml_document tDocument;
    auto tStage = tDocument.append_child("Stage");
    XMLGen::append_evaluate_nondeterministic_objective_gradient_operation("Objective Gradient 0", tXMLMetaData, tStage);
    ASSERT_FALSE(tStage.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tGoldKeys = {"Operation"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);
    auto tOperation = tStage.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "For", "Output"};
    tGoldValues = {"Calculate Non-Deterministic Objective Gradient", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tFor = tOperation.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tFor = tFor.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tInput = tFor.child("Input");
    ASSERT_FALSE(tInput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    tGoldValues = {"Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Mean Plus 2 StdDev Gradient", "Objective Gradient 0"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);
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
    tXMLMetaData.objective_number_standard_deviations = "3";

    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::append_objective_gradient_stage_for_nondeterministic_usecase(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // ****** 1) TEST RESULTS AGAINST STAGE GOLD VALUES ******
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    std::vector<std::string> tGoldKeys = {"Name", "Input", "Operation", "For", "For", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Calculate Objective Gradient 0", "", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tStageInput= tStage.child("Input");
    ASSERT_FALSE(tStageInput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageInput);

    auto tStageOutput= tStage.child("Output");
    ASSERT_FALSE(tStageOutput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Objective Gradient 0"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageOutput);

    // ****** 2) TEST RESULTS AGAINST FILTER OPERATION GOLD VALUES ******
    auto tStageOperation = tStage.child("Operation");
    ASSERT_FALSE(tStageOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Filter Control", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageOperation);

    auto tFilterInput = tStageOperation.child("Input");
    ASSERT_FALSE(tFilterInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterInput);

    auto tFilterOutput = tStageOperation.child("Output");
    ASSERT_FALSE(tFilterOutput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Filtered Field", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterOutput);

    // ****** 3) TEST RESULTS AGAINST EVALUATE RANDOM OBJECTIVE GRADIENT OPERATION GOLD VALUES ******
    tStageOperation = tStageOperation.next_sibling("Operation");
    ASSERT_FALSE(tStageOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "For", "Output"};
    tGoldValues = {"Calculate Non-Deterministic Objective Gradient", "PlatoMain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageOperation);

    auto tFor = tStageOperation.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tFor = tFor.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tForInput = tFor.child("Input");
    ASSERT_FALSE(tForInput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tForInput);

    tForInput = tForInput.next_sibling("Input");
    ASSERT_FALSE(tForInput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tForInput);

    auto tOperationOutput = tStageOperation.child("Output");
    ASSERT_FALSE(tOperationOutput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Mean Plus 3 StdDev Gradient", "Objective Gradient 0"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperationOutput);

    // ****** 4) TEST RESULTS AGAINST SAMPLE OBJECTIVE GRADIENT GOLD VALUES ******
    auto tStageOuterFor = tStage.child("For");
    ASSERT_FALSE(tStageOuterFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tStageOuterFor);

    auto tStageInnerFor = tStageOuterFor.child("Operation").child("For");
    ASSERT_FALSE(tStageInnerFor.empty());
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tStageInnerFor);

    auto tOperation = tStageInnerFor.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Parameter", "Parameter", "Parameter",
        "Parameter", "Parameter", "Parameter", "Input", "Output"};
    tGoldValues = {"Compute Objective Gradient", "plato analyze {PerformerIndex}",
        "", "", "", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    // ****** 4.1) TEST PARAMETERS AGAINST GOLD VALUES ******
    auto tParameter = tOperation.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction load-id-1 x-axis",
                   "{traction load-id-1 x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction load-id-1 y-axis",
                   "{traction load-id-1 y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction load-id-1 z-axis",
                   "{traction load-id-1 z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction load-id-0 x-axis",
                   "{traction load-id-0 x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction load-id-0 y-axis",
                   "{traction load-id-0 y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction load-id-0 z-axis",
                   "{traction load-id-0 z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    // ****** 4.2) TEST OPERATION INPUTS AND OUTPUTS AGAINST GOLD VALUES ******
    auto tOperationInput = tOperation.child("Input");
    ASSERT_FALSE(tOperationInput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Topology", "Topology" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperationInput);

    tOperationOutput = tOperation.child("Output");
    ASSERT_FALSE(tOperationOutput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Objective Gradient", "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperationOutput);

    // ****** 5) TEST RESULTS AGAINST FILTER GRADIENT OPERATION GOLD VALUES ******
    tStageOuterFor = tStageOuterFor.next_sibling("For");
    ASSERT_FALSE(tStageOuterFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tStageOuterFor);

    tStageInnerFor = tStageOuterFor.child("Operation").child("For");
    ASSERT_FALSE(tStageInnerFor.empty());
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tStageInnerFor);

    auto tFilterGradOperation = tStageInnerFor.child("Operation");
    ASSERT_FALSE(tFilterGradOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Input", "Output"};
    tGoldValues = {"Filter Gradient", "PlatoMain", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterGradOperation);

    auto tFilterGradOutput = tFilterGradOperation.child("Output");
    ASSERT_FALSE(tFilterGradOutput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Filtered Gradient", "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterGradOutput);

    auto tFilterGradInput = tFilterGradOperation.child("Input");
    ASSERT_FALSE(tFilterGradInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterGradInput);

    tFilterGradInput = tFilterGradInput.next_sibling("Input");
    ASSERT_FALSE(tFilterGradInput.empty());
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
    XMLGen::Objective tObjective;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.objectives.push_back(tObjective);
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    XMLGen::append_optimization_objective_options(tXMLMetaData, tOptimizerNode);

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"Objective"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tObjectiveNode = tOptimizerNode.child("Objective");
    tGoldKeys = {"ValueName", "ValueStageName", "GradientName", "GradientStageName"};
    tGoldValues = {"Objective Value 0", "Calculate Objective Value 0", "Objective Gradient 0",
        "Calculate Objective Gradient 0"};
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
    tGoldValues = {"Constraint Value 0", "Calculate Constraint Value 0", "Constraint Gradient 0",
        "Calculate Constraint Gradient 0", "Reference Value"};
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
    tGoldValues = {"Constraint Value 0", "Calculate Constraint Value 0", "Constraint Gradient 0",
        "Calculate Constraint Gradient 0", "Reference Value", "1.0"};
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

    // 4. TEST OUTPUT FILE
    auto tReadData = XMLGen::read_data_from_file("interface.xml");
    auto tGold = std::string("<?xmlversion=\"1.0\"?><includefilename=\"defines.xml\"/><Console><Verbose>true</Verbose></Console><Performer><Name>PlatoMain</Name><Code>PlatoMain</Code>")
    +"<PerformerID>0</PerformerID></Performer><Performer><PerformerID>1</PerformerID><Forvar=\"PerformerIndex\"in=\"Performers\"><Name>platoanalyze{PerformerIndex}</Name><Code>analyze</Code>"
    +"</For></Performer><SharedData><Name>Control</Name><Type>Scalar</Type><Layout>NodalField</Layout><OwnerName>PlatoMain</OwnerName><UserName>PlatoMain</UserName></SharedData>"
    +"<SharedData><Name>LowerBoundValue</Name><Type>Scalar</Type><Layout>Global</Layout><Size>1</Size><OwnerName>PlatoMain</OwnerName><UserName>PlatoMain</UserName></SharedData>"
    +"<SharedData><Name>LowerBoundVector</Name><Type>Scalar</Type><Layout>NodalField</Layout><OwnerName>PlatoMain</OwnerName><UserName>PlatoMain</UserName></SharedData><SharedData>"
    +"<Name>UpperBoundValue</Name><Type>Scalar</Type><Layout>Global</Layout><Size>1</Size><OwnerName>PlatoMain</OwnerName><UserName>PlatoMain</UserName></SharedData><SharedData><Name>"
    +"UpperBoundVector</Name><Type>Scalar</Type><Layout>NodalField</Layout><OwnerName>PlatoMain</OwnerName><UserName>PlatoMain</UserName></SharedData><SharedData><Name>ReferenceValue</Name>"
    +"<Type>Scalar</Type><Layout>Global</Layout><Size>1</Size><OwnerName>PlatoMain</OwnerName><UserName>PlatoMain</UserName></SharedData><SharedData><Name>ObjectiveValue0</Name>"
    +"<Type>Scalar</Type><Layout>Global</Layout><Size>1</Size><OwnerName>PlatoMain</OwnerName><UserName>PlatoMain</UserName></SharedData><SharedData><Name>ObjectiveGradient0</Name>"
    +"<Type>Scalar</Type><Layout>NodalField</Layout><OwnerName>PlatoMain</OwnerName><UserName>PlatoMain</UserName></SharedData><SharedData><Name>ConstraintValue0</Name><Type>Scalar</Type>"
    +"<Layout>Global</Layout><Size>1</Size><OwnerName>PlatoMain</OwnerName><UserName>PlatoMain</UserName></SharedData><SharedData><Name>ConstraintGradient0</Name><Type>Scalar</Type>"
    +"<Layout>NodalField</Layout><OwnerName>PlatoMain</OwnerName><UserName>PlatoMain</UserName></SharedData><Forvar=\"PerformerIndex\"in=\"Performers\"><Forvar=\"PerformerSampleIndex\"in=\"PerformerSamples\">"
    +"<SharedData><Name>VonMises{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}</Name><Type>Scalar</Type><Layout>ElementField</Layout><OwnerName>platoanalyze{PerformerIndex}</OwnerName>"
    +"<UserName>PlatoMain</UserName></SharedData></For></For><SharedData><Name>Topology</Name><Type>Scalar</Type><Layout>NodalField</Layout><OwnerName>PlatoMain</OwnerName><UserName>PlatoMain"
    +"</UserName><Forvar=\"PerformerIndex\"in=\"Performers\"><UserName>platoanalyze{PerformerIndex}</UserName></For></SharedData><Forvar=\"PerformerIndex\"in=\"Performers\"><Forvar=\"PerformerSampleIndex\"in=\"PerformerSamples\">"
    +"<SharedData><Name>ObjectiveValue{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}</Name><Type>Scalar</Type><Layout>Global</Layout><Size>1</Size><OwnerName>platoanalyze{PerformerIndex}</OwnerName>"
    +"<UserName>PlatoMain</UserName></SharedData></For></For><Forvar=\"PerformerIndex\"in=\"Performers\"><Forvar=\"PerformerSampleIndex\"in=\"PerformerSamples\"><SharedData><Name>"
    +"ObjectiveGradient{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}</Name><Type>Scalar</Type><Layout>NodalField</Layout><OwnerName>platoanalyze{PerformerIndex}</OwnerName>"
    +"<UserName>PlatoMain</UserName></SharedData></For></For><Stage><Name>CalculateDesignDomainVolume</Name><Operation><Name>CalculateDesignDomainVolume</Name><PerformerName>PlatoMain</PerformerName>"
    +"<Output><ArgumentName>ReferenceValue</ArgumentName><SharedDataName>ReferenceValue</SharedDataName></Output></Operation><Output><SharedDataName>ReferenceValue</SharedDataName></Output>"
    +"</Stage><Stage><Name>InitialGuess</Name><Operation><Name>InitializeField</Name><PerformerName>PlatoMain</PerformerName><Output><ArgumentName>InitializedField</ArgumentName>"
    +"<SharedDataName>Control</SharedDataName></Output></Operation><Output><SharedDataName>Control</SharedDataName></Output></Stage><Stage><Name>SetLowerBounds</Name><Input><SharedDataName>"
    +"LowerBoundValue</SharedDataName></Input><Operation><Name>CalculateLowerBounds</Name><PerformerName>PlatoMain</PerformerName><Input><ArgumentName>LowerBoundVector</ArgumentName>"
    +"<SharedDataName>LowerBoundVector</SharedDataName></Input><Output><ArgumentName>LowerBoundVector</ArgumentName><SharedDataName>LowerBoundVector</SharedDataName></Output></Operation>"
    +"<Output><SharedDataName>LowerBoundVector</SharedDataName></Output></Stage><Stage><Name>SetUpperBounds</Name><Input><SharedDataName>UpperBoundValue</SharedDataName></Input><Operation>"
    +"<Name>CalculateUpperBounds</Name><PerformerName>PlatoMain</PerformerName><Input><ArgumentName>UpperBoundVector</ArgumentName><SharedDataName>UpperBoundVector</SharedDataName></Input>"
    +"<Output><ArgumentName>UpperBoundVector</ArgumentName><SharedDataName>UpperBoundVector</SharedDataName></Output></Operation><Output><SharedDataName>UpperBoundVector</SharedDataName></Output>"
    +"</Stage><Stage><Name>CacheState:platoanalyze0</Name><Forvar=\"PerformerIndex\"in=\"Performers\"><Forvar=\"PerformerSampleIndex\"in=\"PerformerSamples\"><Operation><Name>CacheState</Name>"
    +"<PerformerName>platoanalyze{PerformerIndex}</PerformerName></Operation></For></For></Stage><Stage><Name>UpdateProblem:platoanalyze0</Name><Forvar=\"PerformerIndex\"in=\"Performers\"><Forvar=\"PerformerSampleIndex\"in=\"PerformerSamples\">"
    +"<Operation><Name>UpdateProblem</Name><PerformerName>platoanalyze{PerformerIndex}</PerformerName></Operation></For></For></Stage><Stage><Name>CalculateConstraintValue0</Name><Type>volume</Type>"
    +"<Input><SharedDataName>Control</SharedDataName></Input><Operation><Name>FilterControl</Name><PerformerName>PlatoMain</PerformerName><Input><ArgumentName>Field</ArgumentName><SharedDataName>"
    +"Control</SharedDataName></Input><Output><ArgumentName>FilteredField</ArgumentName><SharedDataName>Topology</SharedDataName></Output></Operation><Operation><Name>CalculateConstraintValue</Name>"
    +"<PerformerName>PlatoMain</PerformerName><Input><ArgumentName>Topology</ArgumentName><SharedDataName>Topology</SharedDataName></Input><Output><ArgumentName>ConstraintValue</ArgumentName>"
    +"<SharedDataName>ConstraintValue0</SharedDataName></Output></Operation><Output><SharedDataName>ConstraintValue0</SharedDataName></Output></Stage><Stage><Name>CalculateConstraintGradient0</Name>"
    +"<Type>volume</Type><Input><SharedDataName>Control</SharedDataName></Input><Operation><Name>FilterControl</Name><PerformerName>PlatoMain</PerformerName><Input><ArgumentName>Field</ArgumentName>"
    +"<SharedDataName>Control</SharedDataName></Input><Output><ArgumentName>FilteredField</ArgumentName><SharedDataName>Topology</SharedDataName></Output></Operation><Operation><Name>CalculateConstraintGradient</Name>"
    +"<PerformerName>PlatoMain</PerformerName><Input><ArgumentName>Topology</ArgumentName><SharedDataName>Topology</SharedDataName></Input><Output><ArgumentName>ConstraintGradient</ArgumentName>"
    +"<SharedDataName>ConstraintGradient0</SharedDataName></Output></Operation><Operation><Name>FilterGradient</Name><PerformerName>PlatoMain</PerformerName><Input><ArgumentName>Field</ArgumentName>"
    +"<SharedDataName>Control</SharedDataName></Input><Input><ArgumentName>Gradient</ArgumentName><SharedDataName>ConstraintGradient0</SharedDataName></Input><Output><ArgumentName>FilteredGradient</ArgumentName>"
    +"<SharedDataName>ConstraintGradient0</SharedDataName></Output></Operation><Output><SharedDataName>ConstraintGradient0</SharedDataName></Output></Stage><Stage><Name>CalculateObjectiveValue0</Name>"
    +"<Input><SharedDataName>Control</SharedDataName></Input><Operation><Name>FilterControl</Name><PerformerName>PlatoMain</PerformerName><Input><ArgumentName>Field</ArgumentName><SharedDataName>Control</SharedDataName>"
    +"</Input><Output><ArgumentName>FilteredField</ArgumentName><SharedDataName>Topology</SharedDataName></Output></Operation><Forvar=\"PerformerSampleIndex\"in=\"PerformerSamples\"><Operation>"
    +"<Forvar=\"PerformerIndex\"in=\"Performers\"><Operation><Name>ComputeObjectiveValue</Name><PerformerName>platoanalyze{PerformerIndex}</PerformerName><Parameter><ArgumentName>tractionload-id-1x-axis</ArgumentName>"
    +"<ArgumentValue>{tractionload-id-1x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue></Parameter><Parameter><ArgumentName>tractionload-id-1y-axis</ArgumentName>"
    +"<ArgumentValue>{tractionload-id-1y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue></Parameter><Parameter><ArgumentName>tractionload-id-1z-axis</ArgumentName>"
    +"<ArgumentValue>{tractionload-id-1z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue></Parameter><Parameter><ArgumentName>tractionload-id-0x-axis</ArgumentName>"
    +"<ArgumentValue>{tractionload-id-0x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue></Parameter><Parameter><ArgumentName>tractionload-id-0y-axis</ArgumentName>"
    +"<ArgumentValue>{tractionload-id-0y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue></Parameter><Parameter><ArgumentName>tractionload-id-0z-axis</ArgumentName>"
    +"<ArgumentValue>{tractionload-id-0z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue></Parameter><Parameter><ArgumentName>elasticmodulusblock-id-2</ArgumentName>"
    +"<ArgumentValue>{elasticmodulusblock-id-2[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue></Parameter><Parameter><ArgumentName>poissonsratioblock-id-2</ArgumentName>"
    +"<ArgumentValue>{poissonsratioblock-id-2[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue></Parameter><Parameter><ArgumentName>elasticmodulusblock-id-1</ArgumentName>"
    +"<ArgumentValue>{elasticmodulusblock-id-1[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue></Parameter><Parameter><ArgumentName>poissonsratioblock-id-1</ArgumentName>"
    +"<ArgumentValue>{poissonsratioblock-id-1[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue></Parameter><Input><ArgumentName>Topology</ArgumentName><SharedDataName>Topology</SharedDataName>"
    +"</Input><Output><ArgumentName>ObjectiveValue</ArgumentName><SharedDataName>ObjectiveValue{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}</SharedDataName></Output></Operation>"
    +"</For></Operation></For><Operation><Name>CalculateNon-DeterministicObjectiveValue</Name><PerformerName>PlatoMain</PerformerName>"
    +"<Forvar=\"PerformerIndex\"in=\"Performers\"><Forvar=\"PerformerSampleIndex\"in=\"PerformerSamples\"><Input><ArgumentName>ObjectiveValue{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}</ArgumentName>"
    +"<SharedDataName>ObjectiveValue{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}</SharedDataName></Input></For></For><Output><ArgumentName>ObjectiveMeanPlus1StdDev</ArgumentName><SharedDataName>"
    +"ObjectiveValue0</SharedDataName></Output></Operation><Output><SharedDataName>ObjectiveValue0</SharedDataName></Output></Stage><Stage><Name>CalculateObjectiveGradient0</Name><Input><SharedDataName>Control</SharedDataName>"
    +"</Input><Operation><Name>FilterControl</Name><PerformerName>PlatoMain</PerformerName><Input><ArgumentName>Field</ArgumentName><SharedDataName>Control</SharedDataName></Input><Output><ArgumentName>FilteredField</ArgumentName>"
    +"<SharedDataName>Topology</SharedDataName></Output></Operation><Forvar=\"PerformerSampleIndex\"in=\"PerformerSamples\"><Operation><Forvar=\"PerformerIndex\"in=\"Performers\"><Operation><Name>ComputeObjectiveGradient</Name>"
    +"<PerformerName>platoanalyze{PerformerIndex}</PerformerName><Parameter><ArgumentName>tractionload-id-1x-axis</ArgumentName><ArgumentValue>{tractionload-id-1x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue>"
    +"</Parameter><Parameter><ArgumentName>tractionload-id-1y-axis</ArgumentName><ArgumentValue>{tractionload-id-1y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue>"
    +"</Parameter><Parameter><ArgumentName>tractionload-id-1z-axis</ArgumentName><ArgumentValue>{tractionload-id-1z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue>"
    +"</Parameter><Parameter><ArgumentName>tractionload-id-0x-axis</ArgumentName><ArgumentValue>{tractionload-id-0x-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue>"
    +"</Parameter><Parameter><ArgumentName>tractionload-id-0y-axis</ArgumentName><ArgumentValue>{tractionload-id-0y-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue>"
    +"</Parameter><Parameter><ArgumentName>tractionload-id-0z-axis</ArgumentName><ArgumentValue>{tractionload-id-0z-axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue>"
    +"</Parameter><Parameter><ArgumentName>elasticmodulusblock-id-2</ArgumentName><ArgumentValue>{elasticmodulusblock-id-2[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue>"
    +"</Parameter><Parameter><ArgumentName>poissonsratioblock-id-2</ArgumentName><ArgumentValue>{poissonsratioblock-id-2[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue>"
    +"</Parameter><Parameter><ArgumentName>elasticmodulusblock-id-1</ArgumentName><ArgumentValue>{elasticmodulusblock-id-1[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue>"
    +"</Parameter><Parameter><ArgumentName>poissonsratioblock-id-1</ArgumentName><ArgumentValue>{poissonsratioblock-id-1[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</ArgumentValue>"
    +"</Parameter><Input><ArgumentName>Topology</ArgumentName><SharedDataName>Topology</SharedDataName></Input><Output><ArgumentName>ObjectiveGradient</ArgumentName><SharedDataName>"
    +"ObjectiveGradient{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}</SharedDataName></Output></Operation></For></Operation></For>"
    +"<Forvar=\"PerformerSampleIndex\"in=\"PerformerSamples\"><Operation><Forvar=\"PerformerIndex\"in=\"Performers\"><Operation><Name>FilterGradient</Name><PerformerName>PlatoMain</PerformerName>"
    +"<Input><ArgumentName>Field</ArgumentName><SharedDataName>Control</SharedDataName></Input><Input><ArgumentName>Gradient</ArgumentName><SharedDataName>"
    +"ObjectiveGradient{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}</SharedDataName></Input><Output><ArgumentName>FilteredGradient</ArgumentName><SharedDataName>"
    +"ObjectiveGradient{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}</SharedDataName></Output></Operation></For></Operation></For><Operation><Name>CalculateNon-DeterministicObjectiveGradient"
    +"</Name><PerformerName>PlatoMain</PerformerName><Forvar=\"PerformerIndex\"in=\"Performers\"><Forvar=\"PerformerSampleIndex\"in=\"PerformerSamples\"><Input><ArgumentName>"
    +"ObjectiveValue{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}</ArgumentName><SharedDataName>ObjectiveValue{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}</SharedDataName>"
    +"</Input><Input><ArgumentName>ObjectiveGradient{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}</ArgumentName><SharedDataName>ObjectiveGradient{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}</SharedDataName>"
    +"</Input></For></For><Output><ArgumentName>ObjectiveMeanPlus1StdDevGradient</ArgumentName><SharedDataName>ObjectiveGradient0</SharedDataName></Output></Operation><Output><SharedDataName>ObjectiveGradient0</SharedDataName>"
    +"</Output></Stage><Optimizer><Package>MMA</Package><Options><MaxNumOuterIterations>100</MaxNumOuterIterations></Options><UpdateProblemStage><Name>UpdateProblem</Name></UpdateProblemStage><CacheStage><Name>CacheState</Name>"
    +"</CacheStage><Output><OutputStage>OutputToFile</OutputStage></Output><OptimizationVariables><ValueName>Control</ValueName><InitializationStage>InitialGuess</InitializationStage><FilteredName>"
    +"Topology</FilteredName><LowerBoundValueName>LowerBoundValue</LowerBoundValueName><LowerBoundVectorName>LowerBoundVector</LowerBoundVectorName><UpperBoundValueName>UpperBoundValue</UpperBoundValueName>"
    +"<UpperBoundVectorName>UpperBoundVector</UpperBoundVectorName><SetLowerBoundsStage>SetLowerBounds</SetLowerBoundsStage><SetUpperBoundsStage>SetUpperBounds</SetUpperBoundsStage></OptimizationVariables>"
    +"<Objective><GradientStageName>CalculateObjectiveGradient0</GradientStageName><GradientName>ObjectiveGradient0</GradientName><ValueStageName>CalculateObjectiveValue0</ValueStageName><ValueName>ObjectiveValue0</ValueName></Objective>"
    +"<Constraint><ReferenceValueName>ReferenceValue</ReferenceValueName><NormalizedTargetValue>1.0</NormalizedTargetValue><GradientStageName>CalculateConstraintGradient0</GradientStageName>"
    +"<GradientName>ConstraintGradient0</GradientName><ValueStageName>CalculateConstraintValue0</ValueStageName><ValueName>ConstraintValue0</ValueName></Constraint><BoundConstraint>"
    +"<Upper>1.0</Upper><Lower>0.0</Lower></BoundConstraint></Optimizer>";
    ASSERT_STREQ(tGold.c_str(), tReadData.str().c_str());

    //std::system("rm -f interface.xml");
}

}
// namespace XMLGeneratorInterfaceFileUnitTester