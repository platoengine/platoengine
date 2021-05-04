/*
 * XMLGeneratorDefineFile_UnitTester.cpp
 *
 *  Created on: May 28, 2020
 */

#include <gtest/gtest.h>

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorDefinesFileUtilities.hpp"

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, Test)
{
    // 1.1 APPEND LOADS
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::NaturalBoundaryCondition tLoad1;
    tLoad1.type("traction");
    tLoad1.id("1");
    tLoad1.is_random("true");
    tLoad1.location_name("sideset");
    std::vector<std::string> tValues;
    tValues.push_back("1");
    tValues.push_back("2");
    tValues.push_back("3");
    tLoad1.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::NaturalBoundaryCondition tLoad2;
    tLoad2.id("2");
    tLoad2.type("traction");
    tLoad2.is_random("true");
    tLoad2.location_name("sideset");
    tValues.clear();
    tValues.push_back("4");
    tValues.push_back("5");
    tValues.push_back("6");
    tLoad2.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::NaturalBoundaryCondition tLoad3;
    tLoad3.id("3");
    tLoad3.type("traction");
    tLoad3.is_random("false");
    tLoad3.location_name("sideset");
    tValues.clear();
    tValues.push_back("7");
    tValues.push_back("8");
    tValues.push_back("9");
    tLoad3.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::NaturalBoundaryCondition tLoad4;
    tLoad4.id("1");
    tLoad4.type("traction");
    tLoad4.is_random("true");
    tLoad4.location_name("sideset");
    tValues.clear();
    tValues.push_back("11");
    tValues.push_back("12");
    tValues.push_back("13");
    tLoad4.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::NaturalBoundaryCondition tLoad5;
    tLoad5.id("2");
    tLoad5.type("traction");
    tLoad5.is_random("true");
    tLoad5.location_name("sideset");
    tValues.clear();
    tValues.push_back("14");
    tValues.push_back("15");
    tValues.push_back("16");
    tLoad5.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // 1.2 CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // 2. CALL FUNCTION
    auto tMapFromIdentifierToLoadValues =
        XMLGen::prepare_tractions_for_define_xml_file(tRandomMetaData);
    ASSERT_FALSE(tMapFromIdentifierToLoadValues.empty());

    // 3. POSE GOLD LOAD VALUES AND TEST
    std::map<std::string, std::vector<std::vector<std::string>>> tGold =
        {
          { "Random Traction Vector Boundary Condition with ID 1", { {"1", "11"}, {"2", "12"}, {"3", "13"} } },
          { "Random Traction Vector Boundary Condition with ID 2", { {"4", "14"}, {"5", "15"}, {"6", "16"} } }
        };

    for(auto& tPair : tMapFromIdentifierToLoadValues)
    {
        auto tGoldItr = tGold.find(tPair.first);
        ASSERT_TRUE(tGoldItr != tGold.end());
        for(auto& tComponent : tPair.second)
        {
            auto tDimIndex = &tComponent - &tPair.second[0];
            for(auto& tSample : tComponent)
            {
                auto tSampleIndex = &tSample - &tComponent[0];
                ASSERT_STREQ(tGoldItr->second[tDimIndex][tSampleIndex].c_str(), tSample.c_str());
            }
        }
    }
}

TEST(PlatoTestXMLGenerator, WriteDefineXmlFile_Loads)
{
    // 1.1 APPEND LOADS
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::NaturalBoundaryCondition tLoad1;
    tLoad1.is_random("true");
    tLoad1.id("1");
    tLoad1.type("traction");
    tLoad1.location_name("sideset");
    std::vector<std::string> tValues;
    tValues.push_back("1");
    tValues.push_back("2");
    tValues.push_back("3");
    tLoad1.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::NaturalBoundaryCondition tLoad2;
    tLoad2.id("2");
    tLoad2.is_random("true");
    tLoad2.type("traction");
    tLoad2.location_name("sideset");
    tValues.clear();
    tValues.push_back("4");
    tValues.push_back("5");
    tValues.push_back("6");
    tLoad2.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::NaturalBoundaryCondition tLoad3;
    tLoad3.id("3");
    tLoad3.is_random("false");
    tLoad3.type("traction");
    tLoad3.location_name("sideset");
    tValues.clear();
    tValues.push_back("7");
    tValues.push_back("8");
    tValues.push_back("9");
    tLoad3.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::NaturalBoundaryCondition tLoad4;
    tLoad4.id("1");
    tLoad4.is_random("true");
    tLoad4.type("traction");
    tLoad4.location_name("sideset");
    tValues.clear();
    tValues.push_back("11");
    tValues.push_back("12");
    tValues.push_back("13");
    tLoad4.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::NaturalBoundaryCondition tLoad5;
    tLoad5.id("2");
    tLoad5.is_random("true");
    tLoad5.type("traction");
    tLoad5.location_name("sideset");
    tValues.clear();
    tValues.push_back("14");
    tValues.push_back("15");
    tValues.push_back("16");
    tLoad5.load_values(tValues);
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
    tUncertaintyMetaData.numPerformers = 2;

    // 3. CALL FUNCTION
    pugi::xml_document tDoc;
    XMLGen::InputData tInputData;
    tInputData.mRandomMetaData = tRandomMetaData;
    tInputData.m_UncertaintyMetaData = tUncertaintyMetaData;
    XMLGen::add_robust_optimization_data_to_define_xml_file(tInputData, tDoc);
    tDoc.save_file("defines.xml", "  ");

    // 4. TEST OUTPUT FILE
    auto tReadData = XMLGen::read_data_from_file("defines.xml");

    auto tGold = std::string("<?xmlversion=\"1.0\"?><Definename=\"NumSamples\"type=\"int\"value=\"2\"/>")
            + "<Definename=\"NumPerformers\"type=\"int\"value=\"2\"/>"
            + "<Definename=\"NumSamplesPerPerformer\"type=\"int\"value=\"{NumSamples/NumPerformers}\"/>"
            + "<Arrayname=\"Samples\"type=\"int\"from=\"0\"to=\"{NumSamples-1}\"/>"
            + "<Arrayname=\"Performers\"type=\"int\"from=\"0\"to=\"{NumPerformers-1}\"/>"
            + "<Arrayname=\"PerformerSamples\"type=\"int\"from=\"0\"to=\"{NumSamplesPerPerformer-1}\"/>"
            + "<Arrayname=\"traction_load_id_1_x_axis\"type=\"real\"values=\"1,11\"/>"
            + "<Arrayname=\"traction_load_id_1_y_axis\"type=\"real\"values=\"2,12\"/>"
            + "<Arrayname=\"traction_load_id_1_z_axis\"type=\"real\"values=\"3,13\"/>"
            + "<Arrayname=\"traction_load_id_2_x_axis\"type=\"real\"values=\"4,14\"/>"
            + "<Arrayname=\"traction_load_id_2_y_axis\"type=\"real\"values=\"5,15\"/>"
            + "<Arrayname=\"traction_load_id_2_z_axis\"type=\"real\"values=\"6,16\"/>"
            + "<Arrayname=\"Probabilities\"type=\"real\"values=\"5.000000000000000000000e-01,5.000000000000000000000e-01\"/>";
    ASSERT_STREQ(tGold.c_str(), tReadData.str().c_str());

    Plato::system("rm -f defines.xml");
}

TEST(PlatoTestXMLGenerator, PrepareProbabilitiesForDefineXmlFile)
{
    // 1.1 APPEND LOADS
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::NaturalBoundaryCondition tLoad1;
    tLoad1.is_random("true");
    std::vector<std::string> tValues;
    tValues.push_back("1");
    tValues.push_back("2");
    tValues.push_back("3");
    tLoad1.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad1);
    tLoadCase1.loads[0].location_name("sideset");
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::NaturalBoundaryCondition tLoad2;
    tLoad2.is_random("true");
    tValues.clear();
    tValues.push_back("11");
    tValues.push_back("12");
    tValues.push_back("13");
    tLoad2.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad2);
    tLoadCase2.loads[0].location_name("sideset");
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
    XMLGen::NaturalBoundaryCondition tLoad1;
    tLoad1.id("1");
    tLoad1.type("traction");
    tLoad1.is_random("true");
    std::vector<std::string> tValues;
    tValues.push_back("1");
    tValues.push_back("2");
    tValues.push_back("3");
    tLoad1.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad1);
    tLoadCase1.loads[0].location_name("sideset");
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::NaturalBoundaryCondition tLoad2;
    tLoad2.id("1");
    tLoad2.type("traction");
    tLoad2.is_random("true");
    tValues.clear();
    tValues.push_back("11");
    tValues.push_back("12");
    tValues.push_back("13");
    tLoad2.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad2);
    tLoadCase2.loads[0].location_name("sideset");
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // 1.2 CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // 2. CALL FUNCTION
    auto tMapFromIDtoTractionValues = XMLGen::prepare_tractions_for_define_xml_file(tRandomMetaData);
    ASSERT_FALSE(tMapFromIDtoTractionValues.empty());

    // 3. POSE GOLD LOAD VALUES AND TEST
    std::map<std::string, std::vector<std::vector<std::string>>> tGold =
        {
            {"Random Traction Vector Boundary Condition with ID 1", { {"1", "11"}, {"2", "12"}, {"3", "13"} } }
        };

    for(auto& tPair : tMapFromIDtoTractionValues)
    {
        auto tGoldItr = tGold.find(tPair.first);
        ASSERT_TRUE(tGoldItr != tGold.end());
        for(auto& tLoadComponent : tPair.second)
        {
            auto tDimIndex = &tLoadComponent - &tPair.second[0];
            for(auto& tSample : tLoadComponent)
            {
                auto tSampleIndex = &tSample - &tLoadComponent[0];
                ASSERT_STREQ(tGoldItr->second[tDimIndex][tSampleIndex].c_str(), tSample.c_str());
            }
        }
    }
}

TEST(PlatoTestXMLGenerator, PrepareRandomTractionsForDefineXmlFile_AllRandomLoads_2LoadPerLoadCase_3Dim)
{
    // 1.1 APPEND LOADS
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::NaturalBoundaryCondition tLoad1;
    tLoad1.id("1");
    tLoad1.type("traction");
    tLoad1.is_random("true");
    tLoad1.location_name("sideset");
    std::vector<std::string> tValues;
    tValues.push_back("1");
    tValues.push_back("2");
    tValues.push_back("3");
    tLoad1.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::NaturalBoundaryCondition tLoad2;
    tLoad2.id("2");
    tLoad2.type("traction");
    tLoad2.is_random("true");
    tLoad2.location_name("sideset");
    tValues.clear();
    tValues.push_back("4");
    tValues.push_back("5");
    tValues.push_back("6");
    tLoad2.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad2);
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::NaturalBoundaryCondition tLoad3;
    tLoad3.id("1");
    tLoad3.type("traction");
    tLoad3.is_random("true");
    tLoad3.location_name("sideset");
    tValues.clear();
    tValues.push_back("11");
    tValues.push_back("12");
    tValues.push_back("13");
    tLoad3.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad3);
    XMLGen::NaturalBoundaryCondition tLoad4;
    tLoad4.id("2");
    tLoad4.type("traction");
    tLoad4.is_random("true");
    tLoad4.location_name("sideset");
    tValues.clear();
    tValues.push_back("14");
    tValues.push_back("15");
    tValues.push_back("16");
    tLoad4.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad4);
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // 1.2 CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // 2. CALL FUNCTION
    auto tMapFromIDtoTractionValues = XMLGen::prepare_tractions_for_define_xml_file(tRandomMetaData);
    ASSERT_FALSE(tMapFromIDtoTractionValues.empty());

    // 3. POSE GOLD LOAD VALUES AND TEST
    std::map<std::string, std::vector<std::vector<std::string>>> tGold =
        {
          { "Random Traction Vector Boundary Condition with ID 1", { {"1", "11"}, {"2", "12"}, {"3", "13"} } }, // Load One
          { "Random Traction Vector Boundary Condition with ID 2", { {"4", "14"}, {"5", "15"}, {"6", "16"} } } // Load Two
        };

    for(auto& tPair : tMapFromIDtoTractionValues)
    {
        auto tGoldItr = tGold.find(tPair.first);
        ASSERT_TRUE(tGoldItr != tGold.end());
        for(auto& tLoadComponent : tPair.second)
        {
            auto tDimIndex = &tLoadComponent - &tPair.second[0];
            for(auto& tSample : tLoadComponent)
            {
                auto tSampleIndex = &tSample - &tLoadComponent[0];
                ASSERT_STREQ(tGoldItr->second[tDimIndex][tSampleIndex].c_str(), tSample.c_str());
            }
        }
    }
}

TEST(PlatoTestXMLGenerator, PrepareRandomTractionsForDefineXmlFile_2RandomLoadsAnd1DeterministicLoadPerLoadCase_3Dim)
{
    // 1.1 APPEND LOADS
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::NaturalBoundaryCondition tLoad1;
    tLoad1.id("1");
    tLoad1.type("traction");
    tLoad1.is_random("true");
    tLoad1.location_name("sideset");
    std::vector<std::string> tValues;
    tValues.push_back("1");
    tValues.push_back("2");
    tValues.push_back("3");
    tLoad1.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::NaturalBoundaryCondition tLoad2;
    tLoad2.id("2");
    tLoad2.type("traction");
    tLoad2.is_random("true");
    tLoad2.location_name("sideset");
    tValues.clear();
    tValues.push_back("4");
    tValues.push_back("5");
    tValues.push_back("6");
    tLoad2.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::NaturalBoundaryCondition tLoad3;
    tLoad3.id("3");
    tLoad3.type("traction");
    tLoad3.is_random("false");
    tLoad3.location_name("sideset");
    tValues.clear();
    tValues.push_back("7");
    tValues.push_back("8");
    tValues.push_back("9");
    tLoad3.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::NaturalBoundaryCondition tLoad4;
    tLoad4.id("1");
    tLoad4.type("traction");
    tLoad4.is_random("true");
    tLoad4.location_name("sideset");
    tValues.clear();
    tValues.push_back("11");
    tValues.push_back("12");
    tValues.push_back("13");
    tLoad3.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::NaturalBoundaryCondition tLoad5;
    tLoad5.id("2");
    tLoad5.type("traction");
    tLoad5.is_random("true");
    tLoad5.location_name("sideset");
    tValues.clear();
    tValues.push_back("14");
    tValues.push_back("15");
    tValues.push_back("16");
    tLoad5.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // 1.2 CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // 2. CALL FUNCTION
    auto tMapFromIDtoTractionValues = XMLGen::prepare_tractions_for_define_xml_file(tRandomMetaData);
    ASSERT_FALSE(tMapFromIDtoTractionValues.empty());

    // 3. POSE GOLD LOAD VALUES AND TEST
    std::map<std::string, std::vector<std::vector<std::string>>> tGold =
        {
          { "Random Traction Vector Boundary Condition with ID 1", { {"1", "11"}, {"2", "12"}, {"3", "13"} } },
          { "Random Traction Vector Boundary Condition with ID 2", { {"4", "14"}, {"5", "15"}, {"6", "16"} } }
        };

    for(auto& tPair : tMapFromIDtoTractionValues)
    {
        auto tGoldItr = tGold.find(tPair.first);
        ASSERT_TRUE(tGoldItr != tGold.end());
        for(auto& tLoadComponent : tPair.second)
        {
            auto tDimIndex = &tLoadComponent - &tPair.second[0];
            for(auto& tSample : tLoadComponent)
            {
                auto tSampleIndex = &tSample - &tLoadComponent[0];
                EXPECT_STREQ(tGoldItr->second[tDimIndex][tSampleIndex].c_str(), tSample.c_str());
            }
        }
    }
}

TEST(PlatoTestXMLGenerator, PrepareRandomTractionsForDefineXmlFile_AllRandomLoads_2LoadPerLoadCase_2Dim)
{
    // 1.1 APPEND LOADS
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::NaturalBoundaryCondition tLoad1;
    tLoad1.id("1");
    tLoad1.type("traction");
    tLoad1.is_random("true");
    tLoad1.location_name("sideset");
    std::vector<std::string> tValues;
    tValues.push_back("1");
    tValues.push_back("2");
    tLoad1.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::NaturalBoundaryCondition tLoad2;
    tLoad2.id("2");
    tLoad2.type("traction");
    tLoad2.is_random("true");
    tLoad2.location_name("sideset");
    tValues.clear();
    tValues.push_back("4");
    tValues.push_back("5");
    tLoad2.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad2);
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::NaturalBoundaryCondition tLoad3;
    tLoad3.id("1");
    tLoad3.type("traction");
    tLoad3.is_random("true");
    tLoad3.location_name("sideset");
    tValues.clear();
    tValues.push_back("11");
    tValues.push_back("12");
    tLoad3.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad3);
    XMLGen::NaturalBoundaryCondition tLoad4;
    tLoad4.id("2");
    tLoad4.type("traction");
    tLoad4.is_random("true");
    tLoad4.location_name("sideset");
    tValues.clear();
    tValues.push_back("14");
    tValues.push_back("15");
    tLoad4.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad4);
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // 1.2 CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // 2. CALL FUNCTION
    auto tMapFromIDtoTractionValues = XMLGen::prepare_tractions_for_define_xml_file(tRandomMetaData);
    ASSERT_FALSE(tMapFromIDtoTractionValues.empty());

    // 3. POSE GOLD LOAD VALUES AND TEST
    std::map<std::string, std::vector<std::vector<std::string>>> tGold =
        {
          { "Random Traction Vector Boundary Condition with ID 1", { {"1", "11"}, {"2", "12"} } },
          { "Random Traction Vector Boundary Condition with ID 2", { {"4", "14"}, {"5", "15"} } }
        };

    for(auto& tPair : tMapFromIDtoTractionValues)
    {
        auto tGoldItr = tGold.find(tPair.first);
        ASSERT_TRUE(tGoldItr != tGold.end());
        for(auto& tLoadComponent : tPair.second)
        {
            auto tDimIndex = &tLoadComponent - &tPair.second[0];
            for(auto& tSample : tLoadComponent)
            {
                auto tSampleIndex = &tSample - &tLoadComponent[0];
                ASSERT_STREQ(tGoldItr->second[tDimIndex][tSampleIndex].c_str(), tSample.c_str());
            }
        }
    }
}

TEST(PlatoTestXMLGenerator, PrepareRandomTractionsForDefineXmlFile_2RandomLoadsAnd1DeterministicLoadPerLoadCase_2Dim)
{
    // 1.1 APPEND LOADS
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::NaturalBoundaryCondition tLoad1;
    tLoad1.id("1");
    tLoad1.type("traction");
    tLoad1.is_random("true");
    tLoad1.location_name("sideset");
    std::vector<std::string> tValues;
    tValues.push_back("1");
    tValues.push_back("2");
    tLoad1.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::NaturalBoundaryCondition tLoad2;
    tLoad2.id("2");
    tLoad2.type("traction");
    tLoad2.is_random("true");
    tLoad2.location_name("sideset");
    tValues.clear();
    tValues.push_back("4");
    tValues.push_back("5");
    tLoad2.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::NaturalBoundaryCondition tLoad3;
    tLoad3.id("3");
    tLoad3.type("traction");
    tLoad3.is_random("false");
    tLoad3.location_name("sideset");
    tValues.clear();
    tValues.push_back("7");
    tValues.push_back("8");
    tLoad3.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::NaturalBoundaryCondition tLoad4;
    tLoad4.id("1");
    tLoad4.type("traction");
    tLoad4.is_random("true");
    tLoad4.location_name("sideset");
    tValues.clear();
    tValues.push_back("11");
    tValues.push_back("12");
    tLoad4.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::NaturalBoundaryCondition tLoad5;
    tLoad5.id("2");
    tLoad5.type("traction");
    tLoad5.is_random("true");
    tLoad5.location_name("sideset");
    tValues.clear();
    tValues.push_back("14");
    tValues.push_back("15");
    tLoad5.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // 1.2 CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // 2. CALL FUNCTION
    auto tMapFromIDtoTractionValues = XMLGen::prepare_tractions_for_define_xml_file(tRandomMetaData);
    ASSERT_FALSE(tMapFromIDtoTractionValues.empty());

    // 3. POSE GOLD LOAD VALUES AND TEST
    std::map<std::string, std::vector<std::vector<std::string>>> tGold =
        {
          { "Random Traction Vector Boundary Condition with ID 1", { {"1", "11"}, {"2", "12"} } },
          { "Random Traction Vector Boundary Condition with ID 2", { {"4", "14"}, {"5", "15"} } }
        };

    for(auto& tPair : tMapFromIDtoTractionValues)
    {
        auto tGoldItr = tGold.find(tPair.first);
        ASSERT_TRUE(tGoldItr != tGold.end());
        for(auto& tLoadComponent : tPair.second)
        {
            auto tDimIndex = &tLoadComponent - &tPair.second[0];
            for(auto& tSample : tLoadComponent)
            {
                auto tSampleIndex = &tSample - &tLoadComponent[0];
                ASSERT_STREQ(tGoldItr->second[tDimIndex][tSampleIndex].c_str(), tSample.c_str());
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
        ASSERT_STREQ(tValuesIterator.operator*().c_str(), tNode.attribute("values").value());
        std::advance(tValuesIterator, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendMaterialPropertiesToDefineXmlFile)
{
    // CALL FUNCTION
    std::unordered_map<std::string, std::vector<std::string>> tData =
            { {"youngs_modulus_block_id_1", {"1", "1.1"} },
              {"youngs_modulus_block_id_2", {"1", "1"} },
              {"poissons_ratio_block_id_1", {"0.3", "0.33"} },
              {"poissons_ratio_block_id_2", {"0.3", "0.3"} } };

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_material_properties_to_define_xml_file(tData, tDocument));

    // POSE GOLD VALUES
    std::vector<std::string> tGoldNames =
        {"youngs_modulus_block_id_1", "youngs_modulus_block_id_2", "poissons_ratio_block_id_1","poissons_ratio_block_id_2"};
    std::vector<std::string> tGoldValues = {"1, 1.1", "1, 1", "0.3, 0.33", "0.3, 0.3"};

    for(pugi::xml_node tArray : tDocument.children("Array"))
    {
        ASSERT_FALSE(tArray.empty());
        auto tAttribute = tArray.attribute("name");
        ASSERT_FALSE(tAttribute.empty());
        auto tGoldNameItr = std::find(tGoldNames.begin(), tGoldNames.end(), tAttribute.value());
        EXPECT_TRUE(tGoldNameItr != tGoldNames.end());
        ASSERT_STREQ(tGoldNameItr->c_str(), tAttribute.value());

        tAttribute = tArray.attribute("values");
        ASSERT_FALSE(tAttribute.empty());
        auto tGoldValueItr = std::find(tGoldValues.begin(), tGoldValues.end(), tAttribute.value());
        EXPECT_TRUE(tGoldValueItr != tGoldValues.end());
        ASSERT_STREQ(tGoldValueItr->c_str(), tAttribute.value());

        ASSERT_STREQ("real", tArray.attribute("type").value());
    }
}

TEST(PlatoTestXMLGenerator, ReturnMaterialPropertiesTagsForDefineXmlFile)
{
    // POSE MATERIAL SET 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("2");
    tMaterial1.materialModel("isotropic");
    tMaterial1.property("youngs_modulus", "1");
    tMaterial1.property("poissons_ratio", "0.3");
    XMLGen::Material tMaterial2;
    tMaterial2.id("2");
    tMaterial2.materialModel("isotropic");
    tMaterial2.property("youngs_modulus", "1");
    tMaterial2.property("poissons_ratio", "0.3");

    XMLGen::MaterialSet tMaterialSetOne;
    tMaterialSetOne.insert({"1", tMaterial1});
    tMaterialSetOne.insert({"2", tMaterial2});
    auto tRandomMaterialCaseOne = std::make_pair(0.5, tMaterialSetOne);

    // POSE MATERIAL SET 2
    XMLGen::Material tMaterial3;
    tMaterial3.id("2");
    tMaterial3.materialModel("isotropic");
    tMaterial3.property("youngs_modulus", "1.1");
    tMaterial3.property("poissons_ratio", "0.33");
    XMLGen::Material tMaterial4;
    tMaterial4.id("2");
    tMaterial4.materialModel("isotropic");
    tMaterial4.property("youngs_modulus", "1");
    tMaterial4.property("poissons_ratio", "0.3");

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
    auto tMap = XMLGen::return_material_property_tags_for_define_xml_file(tRandomMetaData);
    ASSERT_FALSE(tMap.empty());

    // 4. TEST RESULTS
    std::unordered_map<std::string, std::vector<std::string>> tGold =
        {
          {"1", {"poissons_ratio_block_id_1", "youngs_modulus_block_id_1"} },
          {"2", {"poissons_ratio_block_id_2", "youngs_modulus_block_id_2"} }
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
    XMLGen::NaturalBoundaryCondition tLoad1;
    tLoad1.id("1");
    tLoad1.is_random("true");
    tLoad1.type("traction");
    tLoad1.location_name("sideset");
    std::vector<std::string> tValues;
    tValues.push_back("1");
    tValues.push_back("2");
    tValues.push_back("3");
    tLoad1.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::NaturalBoundaryCondition tLoad2;
    tLoad2.id("2");
    tLoad2.is_random("true");
    tLoad2.type("traction");
    tLoad2.location_name("sideset");
    tValues.clear();
    tValues.push_back("4");
    tValues.push_back("5");
    tValues.push_back("6");
    tLoad2.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::NaturalBoundaryCondition tLoad3;
    tLoad3.id("3");
    tLoad3.is_random("false");
    tLoad3.type("traction");
    tLoad3.location_name("sideset");
    tValues.clear();
    tValues.push_back("7");
    tValues.push_back("8");
    tValues.push_back("9");
    tLoad3.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::NaturalBoundaryCondition tLoad4;
    tLoad4.id("1");
    tLoad4.is_random("true");
    tLoad4.type("traction");
    tLoad4.location_name("sideset");
    tValues.clear();
    tValues.push_back("11");
    tValues.push_back("12");
    tValues.push_back("13");
    tLoad4.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::NaturalBoundaryCondition tLoad5;
    tLoad5.id("2");
    tLoad5.is_random("true");
    tLoad5.type("traction");
    tLoad5.location_name("sideset");
    tValues.clear();
    tValues.push_back("14");
    tValues.push_back("15");
    tValues.push_back("16");
    tLoad5.load_values(tValues);
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
          {"Random Traction Vector Boundary Condition with ID 2", {"traction_load_id_2_x_axis", "traction_load_id_2_y_axis", "traction_load_id_2_z_axis"} },
          {"Random Traction Vector Boundary Condition with ID 1", {"traction_load_id_1_x_axis", "traction_load_id_1_y_axis", "traction_load_id_1_z_axis"} }
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
          {"0", {"traction_load_id_0_x_axis","traction_load_id_0_y_axis", "traction_load_id_0_z_axis"} },
          {"1", {"traction_load_id_1_x_axis","traction_load_id_1_y_axis", "traction_load_id_1_z_axis"} }
        };
    std::map<std::string, std::vector<std::vector<std::string>>> tTractionValues =
        {
          { "0", { {"1", "11"}, {"2", "12"}, {"3", "13"} } },
          { "1", { {"4", "14"}, {"5", "15"}, {"6", "16"} } }
        };

    pugi::xml_document tDocument;

    ASSERT_NO_THROW(XMLGen::append_tractions_to_define_xml_file(tTractionTags, tTractionValues, tDocument));

    // 4. POSE GOLD VALUES
    std::vector<std::string> tGoldTypes =
        {"real", "real", "real", "real", "real", "real"};
    std::vector<std::string> tGoldNames =
        {"traction_load_id_0_x_axis", "traction_load_id_0_y_axis", "traction_load_id_0_z_axis",
         "traction_load_id_1_x_axis", "traction_load_id_1_y_axis", "traction_load_id_1_z_axis"};
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
        ASSERT_STREQ(tValuesIterator.operator*().c_str(), tNode.attribute("values").value());
        std::advance(tValuesIterator, 1);
    }
}

TEST(PlatoTestXMLGenerator, PrepareRandomMaterialPropertiesForDefineXmlFile)
{
    // POSE MATERIAL SET 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("2");
    tMaterial1.materialModel("isotropic");
    tMaterial1.property("youngs_modulus", "1");
    tMaterial1.property("poissons_ratio", "0.3");
    XMLGen::Material tMaterial2;
    tMaterial2.id("2");
    tMaterial2.materialModel("isotropic");
    tMaterial2.property("youngs_modulus", "1");
    tMaterial2.property("poissons_ratio", "0.3");

    XMLGen::MaterialSet tMaterialSetOne;
    tMaterialSetOne.insert({"1", tMaterial1});
    tMaterialSetOne.insert({"2", tMaterial2});
    auto tRandomMaterialCaseOne = std::make_pair(0.5, tMaterialSetOne);

    // POSE MATERIAL SET 2
    XMLGen::Material tMaterial3;
    tMaterial3.id("2");
    tMaterial3.materialModel("isotropic");
    tMaterial3.property("youngs_modulus", "1.1");
    tMaterial3.property("poissons_ratio", "0.33");
    XMLGen::Material tMaterial4;
    tMaterial4.id("2");
    tMaterial4.materialModel("isotropic");
    tMaterial4.property("youngs_modulus", "1");
    tMaterial4.property("poissons_ratio", "0.3");

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
            { {"youngs_modulus_block_id_1", {"1", "1.1"} },
              {"youngs_modulus_block_id_2", {"1", "1"} },
              {"poissons_ratio_block_id_1", {"0.3", "0.33"} },
              {"poissons_ratio_block_id_2", {"0.3", "0.3"} } };
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
    tLoadCase1.loads.push_back(XMLGen::NaturalBoundaryCondition());
    tLoadCase1.loads[0].location_name("sideset");
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    tLoadCase2.loads.push_back(XMLGen::NaturalBoundaryCondition());
    tLoadCase2.loads[0].location_name("sideset");
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
    tLoadCase1.loads.push_back(XMLGen::NaturalBoundaryCondition());
    tLoadCase1.loads[0].location_name("sideset");
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    tLoadCase2.loads.push_back(XMLGen::NaturalBoundaryCondition());
    tLoadCase2.loads[0].location_name("sideset");
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // 1.2 CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // 2 SET NUM PERFORMERS
    XMLGen::UncertaintyMetaData tUncertaintyMetaData;
    tUncertaintyMetaData.numPerformers = 2;

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

}
// namespace PlatoTestXMLGenerator
