/*
 * XMLGeneratorPlatoAnalyzeInputFile_UnitTester.cpp
 *
 *  Created on: Jun 8, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeInputFileUtilities.hpp"
#include "XMLGeneratorAnalyzePhysicsFunctionInterface.hpp"
#include "XMLGeneratorAnalyzeNaturalBCFunctionInterface.hpp"
#include "XMLGeneratorAnalyzeEssentialBCFunctionInterface.hpp"
#include "XMLGeneratorAnalyzeNaturalBCTagFunctionInterface.hpp"
#include "XMLGeneratorAnalyzeMaterialModelFunctionInterface.hpp"
#include "XMLGeneratorAnalyzeEssentialBCTagFunctionInterface.hpp"

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, AppendPhysicsMechanical)
{
    XMLGen::Output tOutput;
    tOutput.appendDeterminsiticQoI("stress", "element field");
    tOutput.appendDeterminsiticQoI("vonmises", "element field");
    XMLGen::Service tService;
    tService.code("plato_analyze");
    tService.physics("mechanical");
    tService.additiveContinuation("");
    tService.materialPenaltyExponent("3.0");
    tService.minErsatzMaterialConstant("1e-9");
    pugi::xml_document tDocument;
    XMLGen::AnalyzePhysicsFunctionInterface tPhysics;
    tPhysics.call(tService, tOutput, tDocument);

    // TEST RESULTS
    auto tElliptic = tDocument.child("ParameterList");
    ASSERT_FALSE(tElliptic.empty());
    ASSERT_STREQ("ParameterList", tElliptic.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Elliptic"}, tElliptic);

    auto tParameter = tElliptic.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Plottable", "Array(string)", "{stress, Vonmises}"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());

    auto tPenaltyFunction = tElliptic.child("ParameterList");
    ASSERT_FALSE(tPenaltyFunction.empty());
    ASSERT_STREQ("ParameterList", tPenaltyFunction.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Penalty Function"}, tPenaltyFunction);
    tParameter = tPenaltyFunction.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Type", "string", "SIMP"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Exponent", "double", "3.0"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Minimum Value", "double", "1e-9"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());
}

TEST(PlatoTestXMLGenerator, AppendPhysicsThermal)
{
    XMLGen::Output tOutput;
    tOutput.appendDeterminsiticQoI("vonmises", "element field");
    tOutput.appendDeterminsiticQoI("temperature", "nodal field");
    XMLGen::Service tService;
    tService.code("plato_analyze");
    tService.physics("thermal");
    tService.additiveContinuation("");
    tService.materialPenaltyExponent("3.0");
    tService.minErsatzMaterialConstant("1e-9");
    pugi::xml_document tDocument;
    XMLGen::AnalyzePhysicsFunctionInterface tPhysics;
    tPhysics.call(tService, tOutput, tDocument);

    // TEST RESULTS
    auto tElliptic = tDocument.child("ParameterList");
    ASSERT_FALSE(tElliptic.empty());
    ASSERT_STREQ("ParameterList", tElliptic.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Elliptic"}, tElliptic);

    auto tParameter = tElliptic.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Plottable", "Array(string)", "{Vonmises}"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());

    auto tPenaltyFunction = tElliptic.child("ParameterList");
    ASSERT_FALSE(tPenaltyFunction.empty());
    ASSERT_STREQ("ParameterList", tPenaltyFunction.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Penalty Function"}, tPenaltyFunction);
    tParameter = tPenaltyFunction.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Type", "string", "SIMP"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Exponent", "double", "3.0"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Minimum Value", "double", "1e-9"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());
}

TEST(PlatoTestXMLGenerator, AppendPhysicsThermoMechanical)
{
    XMLGen::Output tOutput;
    XMLGen::Service tService;
    tService.code("plato_analyze");
    tService.physics("thermomechanical");
    tService.additiveContinuation("");
    tService.materialPenaltyExponent("3.0");
    tService.minErsatzMaterialConstant("1e-9");
    pugi::xml_document tDocument;
    XMLGen::AnalyzePhysicsFunctionInterface tPhysics;
    tPhysics.call(tService, tOutput, tDocument);

    // TEST RESULTS
    auto tElliptic = tDocument.child("ParameterList");
    ASSERT_FALSE(tElliptic.empty());
    ASSERT_STREQ("ParameterList", tElliptic.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Elliptic"}, tElliptic);
    auto tParameter = tElliptic.child("Parameter");
    ASSERT_TRUE(tParameter.empty());

    auto tPenaltyFunction = tElliptic.child("ParameterList");
    ASSERT_FALSE(tPenaltyFunction.empty());
    ASSERT_STREQ("ParameterList", tPenaltyFunction.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Penalty Function"}, tPenaltyFunction);
    tParameter = tPenaltyFunction.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Type", "string", "SIMP"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Exponent", "double", "3.0"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Minimum Value", "double", "1e-9"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());
}

TEST(PlatoTestXMLGenerator, AppendPhysicsElectroMechanical)
{
    XMLGen::Output tOutput;
    XMLGen::Service tService;
    tService.code("plato_analyze");
    tService.physics("electromechanical");
    tService.additiveContinuation("");
    tService.materialPenaltyExponent("3.0");
    tService.minErsatzMaterialConstant("1e-9");
    pugi::xml_document tDocument;
    XMLGen::AnalyzePhysicsFunctionInterface tPhysics;
    tPhysics.call(tService, tOutput, tDocument);

    // TEST RESULTS
    auto tElliptic = tDocument.child("ParameterList");
    ASSERT_FALSE(tElliptic.empty());
    ASSERT_STREQ("ParameterList", tElliptic.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Elliptic"}, tElliptic);
    auto tParameter = tElliptic.child("Parameter");
    ASSERT_TRUE(tParameter.empty());

    auto tPenaltyFunction = tElliptic.child("ParameterList");
    ASSERT_FALSE(tPenaltyFunction.empty());
    ASSERT_STREQ("ParameterList", tPenaltyFunction.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Penalty Function"}, tPenaltyFunction);
    tParameter = tPenaltyFunction.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Type", "string", "SIMP"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Exponent", "double", "3.0"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Minimum Value", "double", "1e-9"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());
}

TEST(PlatoTestXMLGenerator, AppendPhysicsHeatConduction)
{
    XMLGen::Output tOutput;
    XMLGen::Service tService;
    tService.code("plato_analyze");
    tService.physics("heat conduction");
    tService.timeStep("1.0");
    tService.numTimeSteps("10");
    tService.materialPenaltyExponent("3.0");
    tService.minErsatzMaterialConstant("1e-9");
    pugi::xml_document tDocument;
    XMLGen::AnalyzePhysicsFunctionInterface tPhysics;
    tPhysics.call(tService, tOutput, tDocument);

    // TEST RESULTS
    auto tParabolic = tDocument.child("ParameterList");
    ASSERT_FALSE(tParabolic.empty());
    ASSERT_STREQ("ParameterList", tParabolic.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Parabolic"}, tParabolic);
    auto tParameter = tParabolic.child("Parameter");
    ASSERT_TRUE(tParameter.empty());

    auto tPenaltyFunction = tParabolic.child("ParameterList");
    ASSERT_FALSE(tPenaltyFunction.empty());
    ASSERT_STREQ("ParameterList", tPenaltyFunction.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Penalty Function"}, tPenaltyFunction);
    tParameter = tPenaltyFunction.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Type", "string", "SIMP"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Exponent", "double", "3.0"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Minimum Value", "double", "1e-9"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());

    auto tTimeStep = tParabolic.next_sibling("ParameterList");
    ASSERT_FALSE(tTimeStep.empty());
    ASSERT_STREQ("ParameterList", tTimeStep.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Time Integration"}, tTimeStep);
    tParameter = tTimeStep.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Number Time Steps", "int", "10"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Time Step", "double", "1.0"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());
}

TEST(PlatoTestXMLGenerator, AppendPhysicsTransientThermoMechanics)
{
    XMLGen::Output tOutput;
    XMLGen::Service tService;
    tService.code("plato_analyze");
    tService.physics("transient thermomechanics");
    tService.timeStep("2.0");
    tService.numTimeSteps("20");
    tService.materialPenaltyExponent("3.0");
    tService.minErsatzMaterialConstant("1e-9");
    pugi::xml_document tDocument;
    XMLGen::AnalyzePhysicsFunctionInterface tPhysics;
    tPhysics.call(tService, tOutput, tDocument);

    // TEST RESULTS
    auto tParabolic = tDocument.child("ParameterList");
    ASSERT_FALSE(tParabolic.empty());
    ASSERT_STREQ("ParameterList", tParabolic.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Parabolic"}, tParabolic);
    auto tParameter = tParabolic.child("Parameter");
    ASSERT_TRUE(tParameter.empty());

    auto tPenaltyFunction = tParabolic.child("ParameterList");
    ASSERT_FALSE(tPenaltyFunction.empty());
    ASSERT_STREQ("ParameterList", tPenaltyFunction.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Penalty Function"}, tPenaltyFunction);
    tParameter = tPenaltyFunction.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Type", "string", "SIMP"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Exponent", "double", "3.0"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Minimum Value", "double", "1e-9"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());

    auto tTimeStep = tParabolic.next_sibling("ParameterList");
    ASSERT_FALSE(tTimeStep.empty());
    ASSERT_STREQ("ParameterList", tTimeStep.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Time Integration"}, tTimeStep);
    tParameter = tTimeStep.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Number Time Steps", "int", "20"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Time Step", "double", "2.0"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());
}

TEST(PlatoTestXMLGenerator, AppendPhysicsTransientMechanics)
{
    XMLGen::Output tOutput;
    XMLGen::Service tService;
    tService.code("plato_analyze");
    tService.physics("transient mechanics");
    tService.timeStep("2.0");
    tService.numTimeSteps("20");
    tService.newmarkBeta("0.25");
    tService.newmarkGamma("0.5");
    tService.materialPenaltyExponent("3.0");
    tService.minErsatzMaterialConstant("1e-9");
    pugi::xml_document tDocument;
    XMLGen::AnalyzePhysicsFunctionInterface tPhysics;
    tPhysics.call(tService, tOutput, tDocument);

    // TEST RESULTS
    auto tParabolic = tDocument.child("ParameterList");
    ASSERT_FALSE(tParabolic.empty());
    ASSERT_STREQ("ParameterList", tParabolic.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Hyperbolic"}, tParabolic);
    auto tParameter = tParabolic.child("Parameter");
    ASSERT_TRUE(tParameter.empty());

    auto tPenaltyFunction = tParabolic.child("ParameterList");
    ASSERT_FALSE(tPenaltyFunction.empty());
    ASSERT_STREQ("ParameterList", tPenaltyFunction.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Penalty Function"}, tPenaltyFunction);
    tParameter = tPenaltyFunction.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Type", "string", "SIMP"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Exponent", "double", "3.0"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Minimum Value", "double", "1e-9"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());

    auto tTimeStep = tParabolic.next_sibling("ParameterList");
    ASSERT_FALSE(tTimeStep.empty());
    ASSERT_STREQ("ParameterList", tTimeStep.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Time Integration"}, tTimeStep);
    tParameter = tTimeStep.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Number Time Steps", "int", "20"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Time Step", "double", "2.0"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Newmark Beta", "double", "0.25"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Newmark Gamma", "double", "0.5"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());
}

TEST(PlatoTestXMLGenerator, AppendPhysicsStabilizedMechanics)
{
    XMLGen::Output tOutput;
    XMLGen::Service tService;
    tService.code("plato_analyze");
    tService.physics("stabilized mechanics");
    tService.timeStep("2.0");
    tService.numTimeSteps("20");
    tService.materialPenaltyExponent("3.0");
    tService.minErsatzMaterialConstant("1e-9");
    pugi::xml_document tDocument;
    XMLGen::AnalyzePhysicsFunctionInterface tPhysics;
    tPhysics.call(tService, tOutput, tDocument);

    // TEST RESULTS
    auto tElliptic = tDocument.child("ParameterList");
    ASSERT_FALSE(tElliptic.empty());
    ASSERT_STREQ("ParameterList", tElliptic.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Elliptic"}, tElliptic);
    auto tParameter = tElliptic.child("Parameter");
    ASSERT_TRUE(tParameter.empty());

    auto tPenaltyFunction = tElliptic.child("ParameterList");
    ASSERT_FALSE(tPenaltyFunction.empty());
    ASSERT_STREQ("ParameterList", tPenaltyFunction.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Penalty Function"}, tPenaltyFunction);
    tParameter = tPenaltyFunction.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Type", "string", "SIMP"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Exponent", "double", "3.0"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Minimum Value", "double", "1e-9"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());

    auto tTimeStep = tElliptic.next_sibling("ParameterList");
    ASSERT_FALSE(tTimeStep.empty());
    ASSERT_STREQ("ParameterList", tTimeStep.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Time Stepping"}, tTimeStep);
    tParameter = tTimeStep.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Number Time Steps", "int", "20"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Time Step", "double", "2.0"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());
}

TEST(PlatoTestXMLGenerator, AppendPhysicsPlasticity)
{
    XMLGen::Output tOutput;
    XMLGen::Service tService;
    tService.code("plato_analyze");
    tService.physics("plasticity");
    tService.timeStep("2.0");
    tService.numTimeSteps("20");
    tService.maxNumTimeSteps("40");
    tService.timeStepExpansion("1.1");
    tService.solverTolerance("1e-5");
    tService.solverMaxNumIterations("10");
    tService.materialPenaltyExponent("3.0");
    tService.minErsatzMaterialConstant("1e-9");
    pugi::xml_document tDocument;
    XMLGen::AnalyzePhysicsFunctionInterface tPhysics;
    tPhysics.call(tService, tOutput, tDocument);

    // TEST RESULTS
    // global residual
    auto tElliptic = tDocument.child("ParameterList");
    ASSERT_FALSE(tElliptic.empty());
    ASSERT_STREQ("ParameterList", tElliptic.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Elliptic"}, tElliptic);
    auto tParameter = tElliptic.child("Parameter");
    ASSERT_TRUE(tParameter.empty());

    auto tPenaltyFunction = tElliptic.child("ParameterList");
    ASSERT_FALSE(tPenaltyFunction.empty());
    ASSERT_STREQ("ParameterList", tPenaltyFunction.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Penalty Function"}, tPenaltyFunction);
    tParameter = tPenaltyFunction.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Type", "string", "SIMP"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Exponent", "double", "3.0"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Minimum Value", "double", "1e-9"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());

    // projected gradient residual
    auto tStateGradProjection = tElliptic.next_sibling("ParameterList");
    ASSERT_FALSE(tStateGradProjection.empty());
    ASSERT_STREQ("ParameterList", tStateGradProjection.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"State Gradient Projection"}, tStateGradProjection);
    tPenaltyFunction = tStateGradProjection.child("ParameterList");
    ASSERT_FALSE(tPenaltyFunction.empty());
    ASSERT_STREQ("ParameterList", tPenaltyFunction.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Penalty Function"}, tPenaltyFunction);
    tParameter = tPenaltyFunction.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Type", "string", "SIMP"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Exponent", "double", "3.0"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Minimum Value", "double", "1e-9"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());

    auto tTimeStep = tStateGradProjection.next_sibling("ParameterList");
    ASSERT_FALSE(tTimeStep.empty());
    ASSERT_STREQ("ParameterList", tTimeStep.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Time Stepping"}, tTimeStep);
    tParameter = tTimeStep.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Expansion Multiplier", "double", "1.1"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Initial Num. Pseudo Time Steps", "int", "20"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Maximum Num. Pseudo Time Steps", "int", "40"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());

    auto tSolver = tTimeStep.next_sibling("ParameterList");
    ASSERT_FALSE(tSolver.empty());
    ASSERT_STREQ("ParameterList", tSolver.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Newton-Raphson"}, tSolver);
    tParameter = tSolver.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Stopping Tolerance", "double", "1e-5"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tValues = {"Maximum Number Iterations", "int", "10"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());
}

TEST(PlatoTestXMLGenerator, ReturnObjectivesComputedByPlatoAnalyze)
{
    XMLGen::Objective tObjective1;
    tObjective1.code_name = "plato_analyze";
    tObjective1.type = "internal elastic energy";
    XMLGen::Objective tObjective2;
    tObjective2.code_name = "aria";
    tObjective2.type = "internal thermal energy";

    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.objectives.push_back(tObjective1);
    tXMLMetaData.objectives.push_back(tObjective2);

    auto tCategories = XMLGen::return_objectives_computed_by_plato_analyze(tXMLMetaData);
    ASSERT_EQ(1u, tCategories.size());
    ASSERT_STREQ("internal elastic energy", tCategories[0].c_str());
}

TEST(PlatoTestXMLGenerator, ReturnConstraintsComputedByPlatoAnalyze)
{
    XMLGen::Constraint tConstraint1;
    tConstraint1.code("plato_analyze");
    tConstraint1.category("internal elastic energy");
    XMLGen::Constraint tConstraint2;
    tConstraint2.code("aria");
    tConstraint2.category("internal thermal energy");

    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.constraints.push_back(tConstraint1);
    tXMLMetaData.constraints.push_back(tConstraint2);

    auto tCategories = XMLGen::return_constraints_computed_by_plato_analyze(tXMLMetaData);
    ASSERT_EQ(1u, tCategories.size());
    ASSERT_STREQ("internal elastic energy", tCategories[0].c_str());
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_ErrorEmptyAppName)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.mPhysics = "mechanical";
    tBC.mCode = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Displacement Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_ErrorInvalidPhysics)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.mPhysics = "cfd";
    tBC.mCode = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Displacement Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_ErrorInvalidCategory)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "mechanical";
    tBC.mCategory = "pin";
    tBC.mCode = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Displacement Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryZeroValue_ErrorInvalidPhysics)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "cfd";
    tBC.mCategory = "zero value";
    tBC.mCode = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryZeroValue_ErrorEmptyDof)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "zero value";
    tBC.mCode = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryZeroValue_ErrorInvalidDof)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.dof = "dispx";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "zero value";
    tBC.mCode = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryRigid)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "mechanical";
    tBC.mCategory = "rigid";
    tBC.mCode = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_NO_THROW(tInterface.call("Displacement Boundary Condition with ID 1", tBC, tDocument));

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Zero Value"}, {"Index", "int", "2"}, {"Sides", "string", "ss_1"},
          {"Type", "string", "Zero Value"}, {"Index", "int", "1"}, {"Sides", "string", "ss_1"},
          {"Type", "string", "Zero Value"}, {"Index", "int", "0"}, {"Sides", "string", "ss_1"} };
    std::vector<std::string> tGoldParameterListNames =
        {"Displacement Boundary Condition with ID 1 applied to Dof with tag DISPZ",
         "Displacement Boundary Condition with ID 1 applied to Dof with tag DISPY",
         "Displacement Boundary Condition with ID 1 applied to Dof with tag DISPX"};

    auto tParamList = tDocument.child("ParameterList");
    auto tGoldValuesItr = tGoldValues.begin();
    auto tGoldParameterListNamesItr = tGoldParameterListNames.begin();
    while(!tParamList.empty())
    {
        ASSERT_FALSE(tParamList.empty());
        ASSERT_STREQ("ParameterList", tParamList.name());
        PlatoTestXMLGenerator::test_attributes({"name"}, {tGoldParameterListNamesItr->c_str()}, tParamList);

        auto tParameter = tParamList.child("Parameter");
        while(!tParameter.empty())
        {
            ASSERT_FALSE(tParameter.empty());
            ASSERT_STREQ("Parameter", tParameter.name());
            PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
            tParameter = tParameter.next_sibling();
            std::advance(tGoldValuesItr, 1);
        }
        tParamList = tParamList.next_sibling();
        std::advance(tGoldParameterListNamesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryFixed)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.app_id = "1";
    tBC.mPhysics = "mechanical";
    tBC.mCategory = "fixed";
    tBC.mCode = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_NO_THROW(tInterface.call("Displacement Boundary Condition with ID 1", tBC, tDocument));

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Zero Value"}, {"Index", "int", "2"}, {"Sides", "string", "1"},
          {"Type", "string", "Zero Value"}, {"Index", "int", "1"}, {"Sides", "string", "1"},
          {"Type", "string", "Zero Value"}, {"Index", "int", "0"}, {"Sides", "string", "1"} };
    std::vector<std::string> tGoldParameterListNames =
        {"Displacement Boundary Condition with ID 1 applied to Dof with tag DISPZ",
         "Displacement Boundary Condition with ID 1 applied to Dof with tag DISPY",
         "Displacement Boundary Condition with ID 1 applied to Dof with tag DISPX"};

    auto tParamList = tDocument.child("ParameterList");
    auto tGoldValuesItr = tGoldValues.begin();
    auto tGoldParameterListNamesItr = tGoldParameterListNames.begin();
    while(!tParamList.empty())
    {
        ASSERT_FALSE(tParamList.empty());
        ASSERT_STREQ("ParameterList", tParamList.name());
        PlatoTestXMLGenerator::test_attributes({"name"}, {tGoldParameterListNamesItr->c_str()}, tParamList);

        auto tParameter = tParamList.child("Parameter");
        while(!tParameter.empty())
        {
            ASSERT_FALSE(tParameter.empty());
            ASSERT_STREQ("Parameter", tParameter.name());
            PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
            tParameter = tParameter.next_sibling();
            std::advance(tGoldValuesItr, 1);
        }
        tParamList = tParamList.next_sibling();
        std::advance(tGoldParameterListNamesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryZeroValue)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.dof = "temp";
    tBC.app_name = "ss_2";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "zero value";
    tBC.mCode = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_NO_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument));

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Zero Value"}, {"Index", "int", "0"}, {"Sides", "string", "ss_2"} };

    auto tParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tParamList.empty());
    ASSERT_STREQ("ParameterList", tParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Thermal Boundary Condition with ID 1 applied to Dof with tag TEMP"}, tParamList);

    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tParamList.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryFixedValue_ErrorInvalidPhysics)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "cfd";
    tBC.mCategory = "fixed value";
    tBC.mCode = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryFixedValue_ErrorEmptyDof)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "fixed value";
    tBC.mCode = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryFixedValue_ErrorInvalidDof)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.dof = "dispx";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "fixed value";
    tBC.mCode = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryFixedValue_ErrorEmptyValue)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.dof = "temp";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "fixed value";
    tBC.mCode = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryFixedValue)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.dof = "temp";
    tBC.value = "10.0";
    tBC.app_name = "ss_2";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "fixed value";
    tBC.mCode = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_NO_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument));

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Fixed Value"}, {"Index", "int", "0"}, {"Sides", "string", "ss_2"}, {"Value", "double", "10.0"} };

    auto tParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tParamList.empty());
    ASSERT_STREQ("ParameterList", tParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Thermal Boundary Condition with ID 1 applied to Dof with tag TEMP"}, tParamList);

    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tParamList.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryInsulated_ErrorInvalidPhysics)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.dof = "temp";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "cfd";
    tBC.mCategory = "insulated";
    tBC.mCode = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryInsulated_ErrorEmptyDof)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "insulated";
    tBC.mCode = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryInsulated_ErrorInvalidDof)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.dof = "dispx";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "insulated";
    tBC.mCode = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryInsulated)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.dof = "temp";
    tBC.app_name = "ss_11";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "insulated";
    tBC.mCode = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_NO_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument));

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Zero Value"}, {"Index", "int", "0"}, {"Sides", "string", "ss_11"} };

    auto tParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tParamList.empty());
    ASSERT_STREQ("ParameterList", tParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Thermal Boundary Condition with ID 1 applied to Dof with tag TEMP"}, tParamList);

    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tParamList.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_PerformerNotAnalyze_DoNothing)
{
    XMLGen::BC tBC;
    tBC.type = "displacement";
    tBC.bc_id = "1";
    tBC.mCode = "sierra";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    tInterface.call("Displacement Boundary Condition with ID 1", tBC, tDocument);
    auto tEssentialBCParamList = tDocument.child("ParameterList");
    ASSERT_TRUE(tEssentialBCParamList.empty());
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryConditionsToPlatoAnalyzeInputDeck)
{
    // POSE PROBLEM
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.app_name = "ss_1";
    tBC.type = "displacement";
    tBC.mPhysics = "mechanical";
    tBC.mCategory = "rigid";
    tBC.mCode = "plato_analyze";
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.bcs.push_back(tBC);

    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::append_essential_boundary_conditions_to_plato_analyze_input_deck(tXMLMetaData, tDocument);

    // TEST
    auto tEssentialBC = tDocument.child("ParameterList");
    ASSERT_FALSE(tEssentialBC.empty());
    ASSERT_STREQ("ParameterList", tEssentialBC.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Essential Boundary Conditions"}, tEssentialBC);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Zero Value"}, {"Index", "int", "2"}, {"Sides", "string", "ss_1"},
          {"Type", "string", "Zero Value"}, {"Index", "int", "1"}, {"Sides", "string", "ss_1"},
          {"Type", "string", "Zero Value"}, {"Index", "int", "0"}, {"Sides", "string", "ss_1"} };
    std::vector<std::string> tGoldParameterListNames =
        {"Displacement Boundary Condition with ID 1 applied to Dof with tag DISPZ",
         "Displacement Boundary Condition with ID 1 applied to Dof with tag DISPY",
         "Displacement Boundary Condition with ID 1 applied to Dof with tag DISPX"};

    auto tParamList = tEssentialBC.child("ParameterList");
    auto tGoldValuesItr = tGoldValues.begin();
    auto tGoldParameterListNamesItr = tGoldParameterListNames.begin();
    while(!tParamList.empty())
    {
        ASSERT_FALSE(tParamList.empty());
        ASSERT_STREQ("ParameterList", tParamList.name());
        PlatoTestXMLGenerator::test_attributes({"name"}, {tGoldParameterListNamesItr->c_str()}, tParamList);

        auto tParameter = tParamList.child("Parameter");
        while(!tParameter.empty())
        {
            ASSERT_FALSE(tParameter.empty());
            ASSERT_STREQ("Parameter", tParameter.name());
            PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
            tParameter = tParameter.next_sibling();
            std::advance(tGoldValuesItr, 1);
        }
        tParamList = tParamList.next_sibling();
        std::advance(tGoldParameterListNamesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, EssentialBoundaryConditionTag_InvalidTag)
{
    XMLGen::EssentialBoundaryConditionTag tInterface;
    XMLGen::BC tBC;
    tBC.type = "fluid velocity";
    ASSERT_THROW(tInterface.call(tBC), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, EssentialBoundaryConditionTag)
{
    XMLGen::EssentialBoundaryConditionTag tInterface;
    XMLGen::BC tBC;

    // TEST 1
    tBC.type = "displacement";
    tBC.bc_id = "1";
    auto tName = tInterface.call(tBC);
    ASSERT_STREQ("Displacement Boundary Condition with ID 1", tName.c_str());

    // TEST 2
    tBC.type = "temperature";
    tBC.bc_id = "1";
    tName = tInterface.call(tBC);
    ASSERT_STREQ("Temperature Boundary Condition with ID 1", tName.c_str());

    // TEST 3
    tBC.type = "potential";
    tBC.bc_id = "1";
    tName = tInterface.call(tBC);
    ASSERT_STREQ("Potential Boundary Condition with ID 1", tName.c_str());

    // TEST 4
    tBC.type = "velocity";
    tBC.bc_id = "1";
    tName = tInterface.call(tBC);
    ASSERT_STREQ("Velocity Boundary Condition with ID 1", tName.c_str());
}

TEST(PlatoTestXMLGenerator, AppendNaturalBoundaryConditionsToPlatoAnalyzeInputDeck_DoNotAppend_PerformerIsNotAnalyze)
{
    XMLGen::Load tLoad;
    tLoad.type = "traction";
    tLoad.load_id = "1";
    tLoad.app_name = "ss_1";
    tLoad.values = {"1.0", "2.0", "3.0"};
    XMLGen::LoadCase tLoadCase;
    tLoadCase.mCode = "sierra";
    tLoadCase.loads.push_back(tLoad);
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.load_cases.push_back(tLoadCase);

    pugi::xml_document tDocument;
    XMLGen::append_natural_boundary_conditions_to_plato_analyze_input_deck(tXMLMetaData, tDocument);
    auto tLoadParamList = tDocument.child("ParameterList");
    ASSERT_TRUE(tLoadParamList.empty());
}

TEST(PlatoTestXMLGenerator, AppendNaturalBoundaryConditionsToPlatoAnalyzeInputDeck)
{
    XMLGen::Load tLoad;
    tLoad.type = "traction";
    tLoad.load_id = "1";
    tLoad.app_name = "ss_1";
    tLoad.values = {"1.0", "2.0", "3.0"};
    XMLGen::LoadCase tLoadCase;
    tLoadCase.mCode = "plato_analyze";
    tLoadCase.loads.push_back(tLoad);
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.load_cases.push_back(tLoadCase);

    pugi::xml_document tDocument;
    XMLGen::append_natural_boundary_conditions_to_plato_analyze_input_deck(tXMLMetaData, tDocument);

    auto tLoadParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tLoadParamList.empty());
    ASSERT_STREQ("ParameterList", tLoadParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Natural Boundary Conditions"}, tLoadParamList);

    auto tTraction = tLoadParamList.child("ParameterList");
    ASSERT_FALSE(tTraction.empty());
    ASSERT_STREQ("ParameterList", tTraction.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Traction Vector Boundary Condition with ID 1"}, tTraction);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Uniform"}, {"Values", "Array(double)", "{1.0, 2.0, 3.0}"}, {"Sides", "string", "ss_1"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tLoadParamList.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendNaturalBoundaryConditionsToPlatoAnalyzeInputDeck_RandomUseCase)
{
    // POSE LOAD SET 1
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::Load tLoad1;
    tLoad1.mIsRandom = true;
    tLoad1.type = "traction";
    tLoad1.load_id = "1";
    tLoad1.app_name = "sideset";
    tLoad1.values.push_back("1.0");
    tLoad1.values.push_back("2.0");
    tLoad1.values.push_back("3.0");
    tLoadCase1.loads.push_back(tLoad1);
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    // POSE LOAD SET 2
    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::Load tLoad2;
    tLoad2.mIsRandom = true;
    tLoad2.type = "traction";
    tLoad1.load_id = "1";
    tLoad2.app_name = "sideset";
    tLoad2.values.push_back("11");
    tLoad2.values.push_back("12");
    tLoad2.values.push_back("13");
    tLoadCase2.loads.push_back(tLoad2);
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // CONSTRUCT SAMPLES SET
    XMLGen::InputData tXMLMetaData;
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.finalize());

    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::append_natural_boundary_conditions_to_plato_analyze_input_deck(tXMLMetaData, tDocument);

    // TEST
    auto tLoadParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tLoadParamList.empty());
    ASSERT_STREQ("ParameterList", tLoadParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Natural Boundary Conditions"}, tLoadParamList);

    auto tTraction = tLoadParamList.child("ParameterList");
    ASSERT_FALSE(tTraction.empty());
    ASSERT_STREQ("ParameterList", tTraction.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Random Traction Vector Boundary Condition with ID 1"}, tTraction);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Uniform"}, {"Values", "Array(double)", "{1.0, 2.0, 3.0}"}, {"Sides", "string", "ss_1"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tLoadParamList.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendNaturalBoundaryCondition_ErrorInvalidType)
{
    XMLGen::Load tLoad;
    tLoad.type = "nonsense";
    pugi::xml_document tDocument;
    XMLGen::AppendNaturalBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Traction Vector Boundary Condition 1", tLoad, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendNaturalBoundaryCondition_Traction)
{
    XMLGen::Load tLoad;
    tLoad.type = "traction";
    tLoad.load_id = "1";
    tLoad.app_name = "ss_1";
    tLoad.values = {"1.0", "2.0", "3.0"};
    pugi::xml_document tDocument;
    XMLGen::AppendNaturalBoundaryCondition tInterface;
    ASSERT_NO_THROW(tInterface.call("Traction Vector Boundary Condition 1", tLoad, tDocument));

    auto tLoadParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tLoadParamList.empty());
    ASSERT_STREQ("ParameterList", tLoadParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Traction Vector Boundary Condition 1"}, tLoadParamList);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Uniform"}, {"Values", "Array(double)", "{1.0, 2.0, 3.0}"}, {"Sides", "string", "ss_1"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tLoadParamList.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendNaturalBoundaryCondition_UniformPressure)
{
    XMLGen::Load tLoad;
    tLoad.type = "uniform pressure";
    tLoad.load_id = "1";
    tLoad.app_name = "ss_1";
    tLoad.values = {"1.0"};
    pugi::xml_document tDocument;
    XMLGen::AppendNaturalBoundaryCondition tInterface;
    ASSERT_NO_THROW(tInterface.call("Uniform Pressure Boundary Condition 1", tLoad, tDocument));

    auto tLoadParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tLoadParamList.empty());
    ASSERT_STREQ("ParameterList", tLoadParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Uniform Pressure Boundary Condition 1"}, tLoadParamList);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Uniform"}, {"Value", "double", "1.0"}, {"Sides", "string", "ss_1"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tLoadParamList.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendNaturalBoundaryCondition_UniformSurfacePotential)
{
    XMLGen::Load tLoad;
    tLoad.type = "uniform surface potential";
    tLoad.load_id = "1";
    tLoad.app_name = "ss_1";
    tLoad.values = {"1.0"};
    pugi::xml_document tDocument;
    XMLGen::AppendNaturalBoundaryCondition tInterface;
    ASSERT_NO_THROW(tInterface.call("Uniform Surface Potential Boundary Condition 1", tLoad, tDocument));

    auto tLoadParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tLoadParamList.empty());
    ASSERT_STREQ("ParameterList", tLoadParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Uniform Surface Potential Boundary Condition 1"}, tLoadParamList);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Uniform"}, {"Value", "double", "1.0"}, {"Sides", "string", "ss_1"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tLoadParamList.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendNaturalBoundaryCondition_UniformSurfaceFlux)
{
    XMLGen::Load tLoad;
    tLoad.type = "uniform surface flux";
    tLoad.load_id = "1";
    tLoad.app_name = "ss_1";
    tLoad.values = {"1.0"};
    pugi::xml_document tDocument;
    XMLGen::AppendNaturalBoundaryCondition tInterface;
    ASSERT_NO_THROW(tInterface.call("Uniform Surface Flux Boundary Condition 1", tLoad, tDocument));

    auto tLoadParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tLoadParamList.empty());
    ASSERT_STREQ("ParameterList", tLoadParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Uniform Surface Flux Boundary Condition 1"}, tLoadParamList);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Uniform"}, {"Value", "double", "1.0"}, {"Sides", "string", "ss_1"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tLoadParamList.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, NaturalBoundaryConditionTag_ErrorInvalidType)
{
    XMLGen::NaturalBoundaryConditionTag tInterface;
    XMLGen::Load tLoad;
    tLoad.type = "nonsense";
    ASSERT_THROW(tInterface.call(tLoad), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, NaturalBoundaryConditionTag)
{
    XMLGen::NaturalBoundaryConditionTag tInterface;
    XMLGen::Load tLoad;

    // TRACTION TEST
    tLoad.type = "traction";
    tLoad.load_id = "1";
    auto tName = tInterface.call(tLoad);
    ASSERT_STREQ("Traction Vector Boundary Condition with ID 1", tName.c_str());

    tLoad.mIsRandom = true;
    tName = tInterface.call(tLoad);
    ASSERT_STREQ("Random Traction Vector Boundary Condition with ID 1", tName.c_str());

    // PRESSURE TEST
    tLoad.mIsRandom = false;
    tLoad.type = "uniform pressure";
    tName = tInterface.call(tLoad);
    ASSERT_STREQ("Uniform Pressure Boundary Condition with ID 1", tName.c_str());

    tLoad.mIsRandom = true;
    tName = tInterface.call(tLoad);
    ASSERT_STREQ("Random Uniform Pressure Boundary Condition with ID 1", tName.c_str());

    // SURFACE POTENTIAL TEST
    tLoad.mIsRandom = false;
    tLoad.type = "uniform surface potential";
    tName = tInterface.call(tLoad);
    ASSERT_STREQ("Uniform Surface Potential Boundary Condition with ID 1", tName.c_str());

    tLoad.mIsRandom = true;
    tName = tInterface.call(tLoad);
    ASSERT_STREQ("Random Uniform Surface Potential Boundary Condition with ID 1", tName.c_str());

    // SURFACE POTENTIAL TEST
    tLoad.mIsRandom = false;
    tLoad.type = "uniform surface flux";
    tName = tInterface.call(tLoad);
    ASSERT_STREQ("Uniform Surface Flux Boundary Condition with ID 1", tName.c_str());

    tLoad.mIsRandom = true;
    tName = tInterface.call(tLoad);
    ASSERT_STREQ("Random Uniform Surface Flux Boundary Condition with ID 1", tName.c_str());
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_ErrorEmptyMaterialContainer)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    ASSERT_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_Empty_MaterialIsNotFromAnalyzePerformer)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Material tMaterial;
    tMaterial.code("sierra");
    tXMLMetaData.materials.push_back(tMaterial);
    ASSERT_NO_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument));
    auto tParamList = tDocument.child("ParameterList");
    ASSERT_TRUE(tParamList.empty());
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_ErrorInvalidMatModel)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Material tMaterial;
    tMaterial.code("plato_analyze");
    tMaterial.category("isotropic linear viscoelastic");
    tXMLMetaData.materials.push_back(tMaterial);
    ASSERT_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_ErrorMatPropAreNotDefined)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Material tMaterial;
    tMaterial.id("1");
    tMaterial.code("plato_analyze");
    tXMLMetaData.materials.push_back(tMaterial);
    ASSERT_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_IsotropicLinearElasticMatModel)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Material tMaterial;
    tMaterial.code("plato_analyze");
    tMaterial.category("isotropic linear elastic");
    tMaterial.property("youngs_modulus", "1e9");
    tMaterial.property("poissons_ratio", "0.3");
    tXMLMetaData.materials.push_back(tMaterial);
    ASSERT_NO_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument));

    auto tMaterialModelParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tMaterialModelParamList.empty());
    ASSERT_STREQ("ParameterList", tMaterialModelParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Material Model"}, tMaterialModelParamList);
    auto tMyMaterialModel = tMaterialModelParamList.child("Isotropic Linear Elastic");
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Isotropic Linear Elastic"}, tMyMaterialModel);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Poissons Ratio", "double", "0.3"}, {"Youngs Modulus", "double", "1e9"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tMyMaterialModel.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_IsotropicLinearThermalMatModel)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Material tMaterial;
    tMaterial.code("plato_analyze");
    tMaterial.category("isotropic linear thermal");
    tMaterial.property("thermal_conductivity", "10");
    tMaterial.property("mass_density", "200");
    tMaterial.property("specific_heat", "20");
    tXMLMetaData.materials.push_back(tMaterial);
    ASSERT_NO_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument));

    auto tMaterialModelParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tMaterialModelParamList.empty());
    ASSERT_STREQ("ParameterList", tMaterialModelParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Material Model"}, tMaterialModelParamList);

    auto tThermalConduction = tMaterialModelParamList.child("ParameterList");
    ASSERT_FALSE(tThermalConduction.empty());
    ASSERT_STREQ("ParameterList", tThermalConduction.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Thermal Conduction"}, tThermalConduction);
    auto tParameter = tThermalConduction.child("Parameter");
    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, {"Thermal Conductivity", "double", "10"}, tParameter);

    auto tThermalMass = tThermalConduction.next_sibling("ParameterList");
    ASSERT_FALSE(tThermalMass.empty());
    ASSERT_STREQ("ParameterList", tThermalMass.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Thermal Mass"}, tThermalMass);
    tParameter = tThermalMass.child("Parameter");
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, {"Mass Density", "double", "200"}, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, {"Specific Heat", "double", "20"}, tParameter);
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_OrthotropicLinearElasticMatModel)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Material tMaterial;
    tMaterial.code("plato_analyze");
    tMaterial.category("orthotropic linear elastic");
    tMaterial.property("youngs_modulus_x", "1.0");
    tMaterial.property("youngs_modulus_y", "2.0");
    tMaterial.property("youngs_modulus_z", "3.0");
    tMaterial.property("poissons_ratio_xy", "0.3");
    tMaterial.property("poissons_ratio_xz", "0.4");
    tMaterial.property("poissons_ratio_yz", "0.25");
    tMaterial.property("shear_modulus_xy", "1.3");
    tMaterial.property("shear_modulus_xz", "1.4");
    tMaterial.property("shear_modulus_yz", "1.25");
    tXMLMetaData.materials.push_back(tMaterial);
    ASSERT_NO_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument));

    auto tMaterialModelParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tMaterialModelParamList.empty());
    ASSERT_STREQ("ParameterList", tMaterialModelParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Material Model"}, tMaterialModelParamList);
    auto tMyMaterialModel = tMaterialModelParamList.child("Orthotropic Linear Elastic");
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Orthotropic Linear Elastic"}, tMyMaterialModel);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Poissons Ratio XY", "double", "0.3"}, {"Poissons Ratio XZ", "double", "0.4"}, {"Poissons Ratio YZ", "double", "0.25"},
          {"Shear Modulus XY", "double", "1.3"}, {"Shear Modulus XZ", "double", "1.4"}, {"Shear Modulus YZ", "double", "1.25"},
          {"Youngs Modulus X", "double", "1.0"}, {"Youngs Modulus Y", "double", "2.0"}, {"Youngs Modulus Z", "double", "3.0"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tMyMaterialModel.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_IsotropicLinearThermoelasticMatModel)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Material tMaterial;
    tMaterial.code("plato_analyze");
    tMaterial.category("isotropic linear thermoelastic");
    tMaterial.property("thermal_conductivity", "1.0");
    tMaterial.property("youngs_modulus", "2.3");
    tMaterial.property("poissons_ratio", "0.3");
    tMaterial.property("thermal_expansivity", "0.4");
    tMaterial.property("reference_temperature", "1.25");
    tXMLMetaData.materials.push_back(tMaterial);
    ASSERT_NO_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument));

    auto tMaterialModelParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tMaterialModelParamList.empty());
    ASSERT_STREQ("ParameterList", tMaterialModelParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Material Model"}, tMaterialModelParamList);

    auto tThermalModel = tMaterialModelParamList.child("ParameterList");
    ASSERT_FALSE(tThermalModel.empty());
    ASSERT_STREQ("ParameterList", tThermalModel.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Thermoelastic"}, tThermalModel);
    auto tParameter = tThermalModel.child("Parameter");
    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, {"Thermal Expansivity", "double", "0.4"}, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, {"Thermal Conductivity", "double", "1.0"}, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, {"Reference Temperature", "double", "1.25"}, tParameter);

    auto tElasticModel = tThermalModel.child("ParameterList");
    ASSERT_FALSE(tElasticModel.empty());
    ASSERT_STREQ("ParameterList", tElasticModel.name());
    tParameter = tElasticModel.child("Parameter");
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, {"Youngs Modulus", "double", "2.3"}, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, {"Poissons Ratio", "double", "0.3"}, tParameter);
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_J2PlasticityMatModel)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Material tMaterial;
    tMaterial.code("plato_analyze");
    tMaterial.category("j2 plasticity");
    tMaterial.property("youngs_modulus", "2.3");
    tMaterial.property("poissons_ratio", "0.3");
    tMaterial.property("pressure_scaling", "1.0");
    tMaterial.property("initial_yield_stress", "2.0");
    tMaterial.property("hardening_modulus_isotropic", "0.4");
    tMaterial.property("hardening_modulus_kinematic", "1.25");
    tXMLMetaData.materials.push_back(tMaterial);
    XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument);

    // elasticity model
    auto tElasticityParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tElasticityParamList.empty());
    ASSERT_STREQ("ParameterList", tElasticityParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Material Model"}, tElasticityParamList);

    auto tElasticModel = tElasticityParamList.child("ParameterList");
    ASSERT_FALSE(tElasticModel.empty());
    ASSERT_STREQ("ParameterList", tElasticModel.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Isotropic Linear Elastic"}, tElasticModel);
    auto tParameter = tElasticModel.child("Parameter");
    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, {"Poissons Ratio", "double", "0.3"}, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, {"Youngs Modulus", "double", "2.3"}, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());

    tParameter = tElasticityParamList.child("Parameter");
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, {"Pressure Scaling", "double", "1.0"}, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());

    // plasticity model
    auto tPlasticityParamList = tElasticityParamList.next_sibling("ParameterList");
    ASSERT_FALSE(tPlasticityParamList.empty());
    ASSERT_STREQ("ParameterList", tPlasticityParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Plasticity Model"}, tPlasticityParamList);

    auto tPlasticModel = tPlasticityParamList.child("ParameterList");
    ASSERT_FALSE(tPlasticModel.empty());
    ASSERT_STREQ("ParameterList", tPlasticModel.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"J2 Plasticity"}, tPlasticModel);
    tParameter = tPlasticModel.child("Parameter");
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, {"Initial Yield Stress", "double", "2.0"}, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, {"Hardening Modulus Isotropic", "double", "0.4"}, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, {"Hardening Modulus Kinematic", "double", "1.25"}, tParameter);
    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_IsotropicLinearElectroelasticMatModel)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Material tMaterial;
    tMaterial.code("plato_analyze");
    tMaterial.category("isotropic linear electroelastic");
    tMaterial.property("youngs_modulus", "2.3");
    tMaterial.property("poissons_ratio", "0.3");
    tMaterial.property("dielectric_permittivity_11", "1.0");
    tMaterial.property("dielectric_permittivity_33", "0.4");
    tMaterial.property("piezoelectric_coupling_15", "1.25");
    tMaterial.property("piezoelectric_coupling_33", "2.25");
    tMaterial.property("piezoelectric_coupling_31", "3.25");
    tMaterial.property("thermal_expansivity", "0.25");
    tXMLMetaData.materials.push_back(tMaterial);
    ASSERT_NO_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument));

    auto tMaterialModelParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tMaterialModelParamList.empty());
    ASSERT_STREQ("ParameterList", tMaterialModelParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Material Model"}, tMaterialModelParamList);
    auto tMyMaterialModel = tMaterialModelParamList.child("Isotropic Linear Electroelastic");
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Isotropic Linear Electroelastic"}, tMyMaterialModel);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"p11", "double", "1.0"}, {"p33", "double", "0.4"}, {"e15", "double", "1.25"},
          {"e31", "double", "3.25"}, {"e33", "double", "2.25"}, {"Poissons Ratio", "double", "0.3"},
          {"Alpha", "double", "0.25"}, {"Youngs Modulus", "double", "2.3"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tMyMaterialModel.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_RandomIsotropicLinearElasticMatModel)
{
    // POSE INPUTS
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.optimization_type = "topology";
    XMLGen::Objective tObjective;
    tObjective.code_name = "plato_analyze";
    tXMLMetaData.objectives.push_back(tObjective);

    // POSE MATERIAL SET 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("1");
    tMaterial1.category("isotropic linear elastic");
    tMaterial1.property("youngs_modulus", "1");
    tMaterial1.property("poissons_ratio", "0.3");
    XMLGen::MaterialSet tMaterialSetOne;
    tMaterialSetOne.insert({"1", tMaterial1});
    auto tRandomMaterialCaseOne = std::make_pair(0.5, tMaterialSetOne);

    // POSE MATERIAL SET 2
    XMLGen::Material tMaterial2;
    tMaterial2.id("1");
    tMaterial2.category("isotropic linear elastic");
    tMaterial2.property("youngs_modulus", "1.1");
    tMaterial2.property("poissons_ratio", "0.33");
    XMLGen::MaterialSet tMaterialSetTwo;
    tMaterialSetTwo.insert({"1", tMaterial2});
    auto tRandomMaterialCaseTwo = std::make_pair(0.5, tMaterialSetTwo);

    // CONSTRUCT SAMPLES SET
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tRandomMaterialCaseOne));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tRandomMaterialCaseTwo));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.finalize());

    // CALL FUNCTION WITH RANDOM MATERIAL
    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument));

    auto tMaterialModelParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tMaterialModelParamList.empty());
    ASSERT_STREQ("ParameterList", tMaterialModelParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Material Model"}, tMaterialModelParamList);
    auto tMyMaterialModel = tMaterialModelParamList.child("Isotropic Linear Elastic");
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Isotropic Linear Elastic"}, tMyMaterialModel);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Poissons Ratio", "double", "0.3"}, {"Youngs Modulus", "double", "1.0"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tMyMaterialModel.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendPhysicsToPlatoAnalyzeInputDeck)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Service tService;
    tService.physics("mechanical");
    tService.code("plato_analyze");
    tXMLMetaData.append(tService);
    ASSERT_NO_THROW(XMLGen::append_physics_to_plato_analyze_input_deck(tXMLMetaData, tDocument));

    auto tPDE = tDocument.child("ParameterList");
    ASSERT_FALSE(tPDE.empty());
    ASSERT_STREQ("ParameterList", tPDE.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Elliptic"}, tPDE);

    auto tPenaltyFunc = tPDE.child("Penalty Function");
    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "SIMP"}, {"Exponent", "double", "3.0"}, {"Minimum Value", "double", "1e-9"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tPenaltyFunc.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendPhysicsToPlatoAnalyzeInputDeck_ErrorInvalidPhysics)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Service tService;
    tService.code("plato_analyze");
    tService.physics("computational fluid dynamics");
    tXMLMetaData.append(tService);
    ASSERT_THROW(XMLGen::append_physics_to_plato_analyze_input_deck(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendObjectiveParameterToPlatoProblem_DoNotAppendParameter)
{
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.run_mesh_name = "lbracket.exo";
    XMLGen::Service tService;
    tService.physics("mechanical");
    tService.dimensions("2");
    tXMLMetaData.append(tService);
    XMLGen::Objective tObjective;
    tObjective.type = "maximize stiffness";
    tObjective.code_name = "sierra";
    tXMLMetaData.objectives.push_back(tObjective);

    pugi::xml_document tDocument;
    XMLGen::append_objective_parameter_to_plato_problem(tXMLMetaData, tDocument);
    auto tParameter = tDocument.child("Parameter");
    ASSERT_TRUE(tParameter.empty());
}

TEST(PlatoTestXMLGenerator, AppendConstraintParameterToPlatoProblem_DoNotAppendParameter)
{
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.run_mesh_name = "lbracket.exo";
    XMLGen::Service tService;
    tService.physics("mechanical");
    tService.dimensions("2");
    tXMLMetaData.append(tService);
    XMLGen::Constraint tConstraint;
    tConstraint.category("maximize stiffness");
    tConstraint.code("sierra_sd");
    tXMLMetaData.constraints.push_back(tConstraint);

    pugi::xml_document tDocument;
    XMLGen::append_constraint_parameter_to_plato_problem(tXMLMetaData, tDocument);
    auto tParameter = tDocument.child("Parameter");
    ASSERT_TRUE(tParameter.empty());
}

TEST(PlatoTestXMLGenerator, AppendSelfAdjointParameterToPlatoProblem_ErrorEmptyObjective)
{
    XMLGen::InputData tXMLMetaData;
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_self_adjoint_parameter_to_plato_problem(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendSelfAdjointParameterToPlatoProblem_ErrorInvalidCriterion)
{
    XMLGen::InputData tXMLMetaData;
    XMLGen::Objective tObjective;
    tObjective.type = "maximize thrust";
    tXMLMetaData.objectives.push_back(tObjective);
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_self_adjoint_parameter_to_plato_problem(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendPDEConstraintParameterToPlatoProblem_ErrorInvalidPhysics)
{
    XMLGen::InputData tXMLMetaData;
    XMLGen::Service tService;
    tService.physics("computational fluid dynamics");
    tXMLMetaData.append(tService);
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_pde_constraint_parameter_to_plato_problem(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendPhysicsParameterToPlatoProblem_ErrorInvalidPhysics)
{
    XMLGen::InputData tXMLMetaData;
    XMLGen::Service tService;
    tService.physics("computational fluid dynamics");
    tXMLMetaData.append(tService);
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_physics_parameter_to_plato_problem(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendProblemDescriptionToPlatoAnalyzeInputDeck_ErrorInvalidSpatialDim)
{
    XMLGen::InputData tXMLMetaData;
    XMLGen::Service tService;
    tService.dimensions("1");
    tXMLMetaData.append(tService);
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_problem_description_to_plato_analyze_input_deck(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendProblemDescriptionToPlatoAnalyzeInputDeck_ErrorEmptyMeshFile)
{
    XMLGen::InputData tXMLMetaData;
    XMLGen::Service tService;
    tService.dimensions("2");
    tXMLMetaData.append(tService);
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_problem_description_to_plato_analyze_input_deck(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendObjectiveCriteriaToPlatoProblem_StressConstraintGeneral)
{
    XMLGen::Objective tObjective;
    tObjective.type = "stress constrained mass minimization";
    tObjective.stress_limit = "2.0";
    tObjective.code_name = "plato_analyze";
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.objectives.push_back(tObjective);

    pugi::xml_document tDocument;
    XMLGen::append_objective_criteria_to_plato_problem(tXMLMetaData, tDocument);

    // TEST MY OBJECTIVE
    auto tParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tParamList.empty());
    ASSERT_STREQ("ParameterList", tParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"my stress constrained mass minimization"}, tParamList);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        {
          {"Type", "string", "Scalar Function"},
          {"Scalar Function Type", "string", "Stress Constraint General"},
          {"Stress Limit", "double", "2.0"},
          {"Mass Criterion Weight", "double", "1.0"},
          {"Stress Criterion Weight", "double", "1.0"}
        };

    auto tParam = tParamList.child("Parameter");
    auto tValuesItr = tGoldValues.begin();
    while(!tParam.empty())
    {
        ASSERT_FALSE(tParam.empty());
        ASSERT_STREQ("Parameter", tParam.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tValuesItr.operator*(), tParam);
        tParam = tParam.next_sibling();
        std::advance(tValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendObjectiveCriteriaToPlatoAnalyzeInputDeck)
{
    XMLGen::InputData tXMLMetaData;
    XMLGen::Objective tObjective1;
    tObjective1.type = "maximize stiffness";
    tObjective1.code_name = "plato_analyze";
    tXMLMetaData.objectives.push_back(tObjective1);
    XMLGen::Objective tObjective2;
    tObjective2.type = "volume";
    tObjective2.mPenaltyParam = "1.0";
    tObjective2.mMinimumErsatzValue = "0.0";
    tObjective2.code_name = "plato_analyze";
    tXMLMetaData.objectives.push_back(tObjective2);

    pugi::xml_document tDocument;
    XMLGen::append_objective_criteria_to_plato_analyze_input_deck(tXMLMetaData, tDocument);

    // TEST MY OBJECTIVE
    auto tParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tParamList.empty());
    ASSERT_STREQ("ParameterList", tParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"My Objective"}, tParamList);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Weighted Sum"}, {"Functions", "Array(string)", "{my maximize stiffness, my volume}"}, {"Weights", "Array(double)", "{1.0, 1.0}"} };
    auto tGoldValuesItr = tGoldValues.begin();

    auto tChild = tParamList.child("Parameter");
    while(!tChild.empty())
    {
        ASSERT_FALSE(tChild.empty());
        ASSERT_STREQ("Parameter", tChild.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tChild);
        tChild = tChild.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }

    // TEST MY OBJECTIVE 1 - 'my maximize stiffness'
    tParamList = tParamList.next_sibling("ParameterList");
    ASSERT_FALSE(tParamList.empty());
    ASSERT_STREQ("ParameterList", tParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"my maximize stiffness"}, tParamList);
    tGoldValues = { {"Type", "string", "Scalar Function"}, {"Scalar Function Type", "string", "Internal Elastic Energy"}, {} };
    tGoldValuesItr = tGoldValues.begin();

    tChild = tParamList.child("Parameter");
    std::vector<std::string> tGoldChildName = {"Parameter", "Parameter", "ParameterList"};
    auto tGoldChildItr = tGoldChildName.begin();
    while(!tChild.empty())
    {
        ASSERT_FALSE(tChild.empty());
        ASSERT_STREQ(tGoldChildItr->c_str(), tChild.name());
        if (tGoldChildItr->compare("Parameter") == 0)
        {
            // TEST PARAMETER CHILDREN, SKIP PENALTY FUNCTION CHILDREN (TEST BELOW)
            PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tChild);
        }
        tChild = tChild.next_sibling();
        std::advance(tGoldValuesItr, 1);
        std::advance(tGoldChildItr, 1);
    }

    auto tPenaltyModel = tParamList.child("Penalty Function");
    tGoldValues = { {"Type", "string", "SIMP"}, {"Exponent", "double", "3.0"}, {"Minimum Value", "double", "1e-9"} };
    tGoldValuesItr = tGoldValues.begin();
    tChild = tPenaltyModel.child("Parameter");
    while(!tChild.empty())
    {
        ASSERT_FALSE(tChild.empty());
        ASSERT_STREQ("Parameter", tChild.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tChild);
        tChild = tChild.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }

    // TEST MY OBJECTIVE 2 - 'my volume'
    tParamList = tParamList.next_sibling("ParameterList");
    ASSERT_FALSE(tParamList.empty());
    ASSERT_STREQ("ParameterList", tParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"my volume"}, tParamList);
    tGoldValues = { {"Type", "string", "Scalar Function"}, {"Scalar Function Type", "string", "Volume"}, {} };
    tGoldValuesItr = tGoldValues.begin();

    tChild = tParamList.child("Parameter");
    tGoldChildItr = tGoldChildName.begin();
    while(!tChild.empty())
    {
        ASSERT_FALSE(tChild.empty());
        ASSERT_STREQ(tGoldChildItr->c_str(), tChild.name());
        if (tGoldChildItr->compare("Parameter") == 0)
        {
            // TEST PARAMETER CHILDREN, SKIP PENALTY FUNCTION CHILDREN (TEST BELOW)
            PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tChild);
        }
        tChild = tChild.next_sibling();
        std::advance(tGoldValuesItr, 1);
        std::advance(tGoldChildItr, 1);
    }

    tPenaltyModel = tParamList.child("Penalty Function");
    tGoldValues = { {"Type", "string", "SIMP"}, {"Exponent", "double", "1.0"}, {"Minimum Value", "double", "0.0"} };
    tGoldValuesItr = tGoldValues.begin();
    tChild = tPenaltyModel.child("Parameter");
    while(!tChild.empty())
    {
        ASSERT_FALSE(tChild.empty());
        ASSERT_STREQ("Parameter", tChild.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tChild);
        tChild = tChild.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendConstraintCriteriaToPlatoAnalyzeInputDeck)
{
    XMLGen::InputData tXMLMetaData;
    XMLGen::Constraint tConstraint;
    tConstraint.category("stress p-norm");
    tConstraint.weight("0.5");
    tConstraint.code("plato_analyze");
    tConstraint.pnormExponent("6");
    tConstraint.materialPenaltyExponent("3.0");
    tConstraint.minErsatzMaterialConstant("1e-9");
    tXMLMetaData.constraints.push_back(tConstraint);

    pugi::xml_document tDocument;
    XMLGen::append_constraint_criteria_to_plato_analyze_input_deck(tXMLMetaData, tDocument);

    // TEST MY CONSTRAINT
    auto tParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tParamList.empty());
    ASSERT_STREQ("ParameterList", tParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"My Constraint"}, tParamList);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Weighted Sum"}, {"Functions", "Array(string)", "{my stress p-norm}"},
          {"Weights", "Array(double)", "{0.5}"}, {"Exponent", "double", "6"} };
    auto tGoldValuesItr = tGoldValues.begin();

    auto tChild = tParamList.child("Parameter");
    while(!tChild.empty())
    {
        ASSERT_FALSE(tChild.empty());
        ASSERT_STREQ("Parameter", tChild.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tChild);
        tChild = tChild.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }

    // TEST MY OBJECTIVE 1 - 'my stress p-norm'
    tParamList = tParamList.next_sibling("ParameterList");
    ASSERT_FALSE(tParamList.empty());
    ASSERT_STREQ("ParameterList", tParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"my stress p-norm"}, tParamList);
    tGoldValues = { {"Type", "string", "Scalar Function"}, {"Scalar Function Type", "string", "Stress P-Norm"}, {} };
    tGoldValuesItr = tGoldValues.begin();

    tChild = tParamList.child("Parameter");
    std::vector<std::string> tGoldChildName = {"Parameter", "Parameter", "ParameterList"};
    auto tGoldChildItr = tGoldChildName.begin();
    while(!tChild.empty())
    {
        ASSERT_FALSE(tChild.empty());
        ASSERT_STREQ(tGoldChildItr->c_str(), tChild.name());
        if (tGoldChildItr->compare("Parameter") == 0)
        {
            // TEST PARAMETER CHILDREN, SKIP PENALTY FUNCTION CHILDREN (TEST BELOW)
            PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tChild);
        }
        tChild = tChild.next_sibling();
        std::advance(tGoldValuesItr, 1);
        std::advance(tGoldChildItr, 1);
    }

    auto tPenaltyModel = tParamList.child("Penalty Function");
    tGoldValues = { {"Type", "string", "SIMP"}, {"Exponent", "double", "3.0"}, {"Minimum Value", "double", "1e-9"} };
    tGoldValuesItr = tGoldValues.begin();
    tChild = tPenaltyModel.child("Parameter");
    while(!tChild.empty())
    {
        ASSERT_FALSE(tChild.empty());
        ASSERT_STREQ("Parameter", tChild.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tChild);
        tChild = tChild.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendProblemDescriptionToPlatoAnalyzeInputDeck)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.run_mesh_name = "lbracket.exo";
    XMLGen::Service tService;
    tService.dimensions("2");
    tXMLMetaData.append(tService);
    ASSERT_NO_THROW(XMLGen::append_problem_description_to_plato_analyze_input_deck(tXMLMetaData, tDocument));

    auto tProblem = tDocument.child("ParameterList");
    ASSERT_FALSE(tProblem.empty());
    ASSERT_STREQ("ParameterList", tProblem.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Problem"}, tProblem);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Physics", "string", "Plato Driver"}, {"Spatial Dimension", "int", "2"}, {"Input Mesh", "string", "lbracket.exo"} };
    auto tGoldValuesItr = tGoldValues.begin();

    auto tParameter = tProblem.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendPlatoProblemToPlatoAnalyzeInputDeck)
{
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.run_mesh_name = "lbracket.exo";
    XMLGen::Service tService;
    tService.physics("mechanical");
    tService.dimensions("2");
    tXMLMetaData.append(tService);
    XMLGen::Objective tObjective;
    tObjective.type = "maximize stiffness";
    tObjective.code_name = "plato_analyze";
    tXMLMetaData.objectives.push_back(tObjective);
    XMLGen::Constraint tConstraint;
    tConstraint.category("volume");
    tConstraint.code("plato_analyze");
    tXMLMetaData.constraints.push_back(tConstraint);

    pugi::xml_document tDocument;
    auto tProblem = tDocument.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Problem"}, tProblem);
    XMLGen::append_plato_problem_description_to_plato_analyze_input_deck(tXMLMetaData, tDocument);

    tProblem = tDocument.child("ParameterList");
    ASSERT_FALSE(tProblem.empty());
    ASSERT_STREQ("ParameterList", tProblem.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Problem"}, tProblem);

    auto tPlatoProblem = tProblem.child("ParameterList");
    ASSERT_FALSE(tPlatoProblem.empty());
    ASSERT_STREQ("ParameterList", tPlatoProblem.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Plato Problem"}, tPlatoProblem);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Physics", "string", "Mechanical"}, {"PDE Constraint", "string", "Elliptic"}, {"Constraint", "string", "My Constraint"},
          {"Objective", "string", "My Objective"}, {"Self-Adjoint", "bool", "true"} };
    auto tGoldValuesItr = tGoldValues.begin();

    auto tParameter = tPlatoProblem.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

}
// namespace PlatoTestXMLGenerator
