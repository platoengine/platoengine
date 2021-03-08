###############################################################################
# Sources:
###############################################################################
SET(PlatoXMLGenerator_UnitTester_SRCS XMLGenerator_UnitMain.cpp
                                      ../XMLGenerator.cpp
                                      XMLGenerator_UnitTester.cpp
                                      ../XMLGeneratorUtilities.cpp
                                      ../XMLGeneratorParserUtilities.cpp
                                      Plato_Test_XMLGenerator.cpp
                                      XMLGeneratorSrom_UnitTester.cpp
                                      XMLGeneratorDefineFile_UnitTester.cpp
                                      XMLGeneratorLaunchScript_UnitTester.cpp
                                      XMLGeneratorAnalyzeUncertaintyLaunchScript_UnitTester.cpp
                                      XMLGeneratorParseMetaData_UnitTester.cpp
                                      XMLGeneratorInterfaceFile_UnitTester.cpp
                                      XMLGeneratorRandomInterfaceFile_UnitTester.cpp
                                      XMLGeneratorPlatoMainInputDeckFile_UnitTester.cpp
                                      XMLGeneratorPlatoMainOperationFile_UnitTester.cpp
                                      XMLGeneratorPlatoAnalyzeInputFile_UnitTester.cpp
                                      XMLGeneratorPlatoAnalyzeOperationsFile_UnitTester.cpp
                                      ../XMLGeneratorParseOptimizationParameters.cpp
                                      ../XMLGeneratorOptimizationParametersMetadata.cpp
                                      ../XMLGeneratorParseOutput.cpp
                                      ../XMLGeneratorParseScenario.cpp
                                      ../XMLGeneratorParseServices.cpp
                                      ../XMLGeneratorParseMaterial.cpp
                                      ../XMLGeneratorParseCriteria.cpp
                                      ../XMLGeneratorOutputMetadata.cpp
                                      ../XMLGeneratorParseObjective.cpp
                                      ../XMLGeneratorParseEssentialBoundaryCondition.cpp
                                      ../XMLGeneratorParseNaturalBoundaryCondition.cpp
                                      ../XMLGeneratorParseConstraint.cpp
                                      ../XMLGeneratorParseUncertainty.cpp
                                      ../XMLGeneratorScenarioMetadata.cpp
                                      ../XMLGeneratorServiceMetadata.cpp
                                      ../XMLGeneratorConstraintMetadata.cpp
                                      ../XMLGeneratorUncertaintyMetadata.cpp
                                      ../XMLGeneratorDefinesFileUtilities.cpp
                                      ../XMLGeneratorPlatoAnalyzeUtilities.cpp
                                      ../XMLGeneratorLaunchScriptUtilities.cpp
                                       ../XMLGeneratorInterfaceFileUtilities.cpp
                                      ../XMLGeneratorMaterialFunctionInterface.cpp
                                      ../XMLGeneratorPlatoMainInputFileUtilities.cpp
                                      ../XMLGeneratorPlatoESPInputFileUtilities.cpp
                                      ../XMLGeneratorPlatoESPOperationsFileUtilities.cpp
                                      ../XMLGeneratorRandomInterfaceFileUtilities.cpp
                                      ../XMLGeneratorPlatoAnalyzeInputFileUtilities.cpp
                                      ../XMLGeneratorAnalyzePhysicsFunctionInterface.cpp
                                      ../XMLGeneratorPlatoMainOperationFileUtilities.cpp
                                      ../XMLGeneratorPlatoAnalyzeOperationsFileUtilities.cpp
                                      ../XMLGeneratorSierraSDOperationsFileUtilities.cpp
                                      ../XMLGeneratorSierraSDInputDeckUtilities.cpp
                                      ../XMLGeneratorAnalyzeNaturalBCFunctionInterface.cpp
                                      ../XMLGeneratorAnalyzeNaturalBCTagFunctionInterface.cpp
                                      ../XMLGeneratorAnalyzeEssentialBCFunctionInterface.cpp
                                      ../XMLGeneratorAnalyzeEssentialBCTagFunctionInterface.cpp
                                      ../XMLGeneratorAnalyzeMaterialModelFunctionInterface.cpp
                                      ../XMLGeneratorPlatoMainConstraintValueOperationInterface.cpp
                                      ../XMLGeneratorPlatoMainConstraintGradientOperationInterface.cpp
                                      ../XMLGeneratorAnalyzeUncertaintyLaunchScriptUtilities.cpp
                                      # ../PlatoAnalyzeInputDeckWriter.cpp
                                      # ../SalinasInputDeckWriter.cpp)
                                      )

SET(PlatoXMLGenerator_UnitTester_HDRS XMLGenerator_UnitTester.hpp
                                      XMLGenerator_UnitTester_Tools.hpp)

if(CMAKE_COMPILER_IS_GNUCC)
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   -Wno-unused-variable -Wno-write-strings")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wno-unused-variable -Wno-write-strings")
endif()

set(PLATOUNIT_INCLUDES ${CMAKE_SOURCE_DIR}/base/src/tools ..)
INCLUDE_DIRECTORIES(${PLATOUNIT_INCLUDES})

set(PlatoXMLGenerator_UnitTester_LIBS ${GTEST_LIBRARY} ${PLATO_LIBRARIES} ${Trilinos_LIBRARIES} ${Trilinos_TPL_LIBRARIES} PlatoParser )
#set(PlatoXMLGenerator_UnitTester_LIBS ${GTEST_LIBRARY} PlatoParser PlatoExceptions ${Trilinos_LIBRARIES} ${Trilinos_TPL_LIBRARIES} )
add_executable(PlatoXMLGenerator_UnitTester ${PlatoXMLGenerator_UnitTester_SRCS})
target_link_libraries(PlatoXMLGenerator_UnitTester ${PlatoXMLGenerator_UnitTester_LIBS})
add_test(NAME PlatoXMLGenerator_UnitTester COMMAND ${CMAKE_BINARY_DIR}/base/src/input_generator/unittest/PlatoXMLGenerator_UnitTester)


if( CMAKE_INSTALL_PREFIX )
  install( TARGETS PlatoXMLGenerator_UnitTester DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
endif()
