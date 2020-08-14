###############################################################################
# Sources:
###############################################################################
SET(XMLGenerator_SRCS XMLGenerator_Main.cpp 
                      XMLGenerator.cpp
                      # XMLGeneratorParseOutput.cpp
                      # XMLGeneratorParseService.cpp
                      # XMLGeneratorOutputMetadata.cpp 
                      # XMLGeneratorParseObjective.cpp
                      XMLGeneratorParseMaterial.cpp
                      # XMLGeneratorParseCriteria.cpp
                      # XMLGeneratorParseConstraint.cpp
                      XMLGeneratorParserUtilities.cpp
                      # XMLGeneratorServiceMetadata.cpp
                      # XMLGeneratorParseUncertainty.cpp
                      # XMLGeneratorConstraintMetadata.cpp
                      # XMLGeneratorUncertaintyMetadata.cpp
                      # XMLGeneratorDefinesFileUtilities.cpp
                      # XMLGeneratorPlatoAnalyzeUtilities.cpp
                      # XMLGeneratorInterfaceFileUtilities.cpp
                      # XMLGeneratorAnalyzePhysicsFunctionInterface.cpp
                      # XMLGeneratorMaterialFunctionInterface.cpp
                      # XMLGeneratorRandomInterfaceFileUtilities.cpp
                      # XMLGeneratorPlatoMainInputFileUtilities.cpp
                      # XMLGeneratorPlatoAnalyzeInputFileUtilities.cpp
                      # XMLGeneratorPlatoMainOperationFileUtilities.cpp
                      # XMLGeneratorPlatoAnalyzeOperationsFileUtilities.cpp
                      # XMLGeneratorLaunchScriptUtilities.cpp
                      # XMLGeneratorAnalyzeNaturalBCFunctionInterface.cpp
                      # XMLGeneratorAnalyzeNaturalBCTagFunctionInterface.cpp
                      # XMLGeneratorAnalyzeEssentialBCFunctionInterface.cpp
                      # XMLGeneratorAnalyzeEssentialBCTagFunctionInterface.cpp
                      # XMLGeneratorAnalyzeMaterialModelFunctionInterface.cpp
                      # XMLGeneratorPlatoMainConstraintValueOperationInterface.cpp
                      # XMLGeneratorPlatoMainConstraintGradientOperationInterface.cpp
                      # XMLGeneratorAnalyzeUncertaintyLaunchScriptUtilities.cpp
                      # PlatoAnalyzeInputDeckWriter.cpp
                      # XMLGeneratorUtilities.cpp
                      # SalinasInputDeckWriter.cpp
                      )
                      
SET(XMLGenerator_HDRS XMLGenerator.hpp 
                      XMLGeneratorDataStruct.hpp
                      # XMLGeneratorValidInputKeys.hpp
                      # XMLGeneratorRandomMetadata.hpp
                      # XMLGeneratorOutputMetadata.hpp 
                      XMLGeneratorParserUtilities.hpp 
                      # XMLGeneratorServiceMetadata.hpp
                      XMLGeneratorMaterialMetadata.hpp 
                      # XMLGeneratorCriterionMetadata.hpp 
                      # XMLGeneratorCriterionMetadata.hpp 
                      # XMLGeneratorConstraintMetadata.hpp
                      # XMLGeneratorUncertaintyMetadata.hpp
                      # XMLGeneratorPlatoAnalyzeProblem.hpp 
                      # XMLGeneratorDefinesFileUtilities.hpp 
                      # XMLGeneratorPlatoAnalyzeUtilities.hpp 
                      # XMLGeneratorInterfaceFileUtilities.hpp 
                      # XMLGeneratorAnalyzeFunctionMapTypes.hpp
                      # XMLGeneratorAnalyzePhysicsFunctionInterface.hpp
                      # XMLGeneratorMaterialFunctionInterface.hpp
                      # XMLGeneratorPlatoMainInputFileUtilities.hpp
                      # XMLGeneratorPlatoAnalyzeInputFileUtilities.hpp
                      # XMLGeneratorPlatoMainOperationFileUtilities.hpp
                      # XMLGeneratorPlatoAnalyzeOperationsFileUtilities.hpp
                      # XMLGeneratorLaunchScriptUtilities.hpp
                      # XMLGeneratorAnalyzeCriterionUtilities.hpp
                      # XMLGeneratorAnalyzeNaturalBCFunctionInterface.hpp
                      # XMLGeneratorAnalyzeNaturalBCTagFunctionInterface.hpp
                      # XMLGeneratorAnalyzeEssentialBCFunctionInterface.hpp
                      # XMLGeneratorAnalyzeEssentiallBCTagFunctionInterface.hpp
                      # XMLGeneratorAnalyzeMaterialModelFunctionInterface.hpp
                      # XMLGeneratorAnalyzeAppendCriterionFunctionInterface.hpp
                      # XMLGeneratorPlatoMainConstraintValueOperationInterface.hpp
                      # XMLGeneratorPlatoMainConstraintGradientOperationInterface.hpp
                      # XMLGeneratorAnalyzeUncertaintyLaunchScriptUtilities.hpp
                      # XMLGeneratorRandomInterfaceFileUtilities.hpp 
                      # XMLGeneratorPlatoMainFunctionMapTypes.hpp
                      # XMLGeneratorDataStruct.hpp
                      # PlatoAnalyzeInputDeckWriter.hpp
                      # XMLGeneratorUtilities.hpp
                      # XMLGeneratorParseOutput.hpp 
                      XMLGeneratorParseMaterial.hpp 
                      # XMLGeneratorParseCriteria.hpp 
                      # XMLGeneratorParseObjective.hpp 
                      # XMLGeneratorParseMetadata.hpp 
                      # XMLGeneratorParseService.hpp 
                      # XMLGeneratorParseConstraint.hpp 
                      # XMLGeneratorParseUncertainty.hpp
                      # SalinasInputDeckWriter.hpp
                      )

find_package(MPI)

# try to determine if Open MPI is being used
STRING(FIND ${MPI_C_COMPILER} "openmpi" FIND_POS) 

if( FIND_POS EQUAL -1 )
  execute_process(COMMAND mpirun --version OUTPUT_VARIABLE OUT)
  STRING(FIND ${OUT} "Open MPI" FIND_POS) 
  if( FIND_POS EQUAL -1 )
  else()
    add_definitions(-DUSING_OPEN_MPI)
  endif()
 else()
   add_definitions(-DUSING_OPEN_MPI)
endif()

INCLUDE_DIRECTORIES(${PLATO_INCLUDES})

# actual target:
set(PLATOMAIN_LIBS ${PLATO_LIBRARIES} ${Plato_EXTRA_LINK_FLAGS})
add_executable(XMLGenerator ${XMLGenerator_SRCS})
target_link_libraries(XMLGenerator ${PLATOMAIN_LIBS} ${Trilinos_LIBRARIES} ${Trilinos_TPL_LIBRARIES})
#set(INTERFACEXMLCHECKER_BINARY ${CMAKE_BINARY_DIR}/base/src/input_checkers/XMLGenerator PARENT_SCOPE)

if( CMAKE_INSTALL_PREFIX )
  install( TARGETS XMLGenerator DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
endif()
###############################################################################
###############################################################################

if( UNIT_TESTING )
  add_subdirectory(./unittest)
endif()

