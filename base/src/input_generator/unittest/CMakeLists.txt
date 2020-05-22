###############################################################################
# Sources:
###############################################################################
SET(PlatoXMLGenerator_UnitTester_SRCS XMLGenerator_UnitMain.cpp
                                      XMLGenerator_UnitTester.cpp
                                      DefaultInputGenerator_UnitTester.cpp
                                      ComplianceMinTOPlatoAnalyzeInputGenerator_UnitTester.cpp
                                      ComplianceMinTOPlatoAnalyzeUncertInputGenerator_UnitTester.cpp
                                      Plato_Test_XMLGenerator.cpp
                                      ../XMLGenerator.cpp
                                      ../XMLGeneratorUtilities.cpp
                                      ../XMLGeneratorAnalyzeDefinesFileUtilities.cpp
                                      ../ComplianceMinTOPlatoAnalyzeInputGenerator.cpp
                                      ../ComplianceMinTOPlatoAnalyzeUncertInputGenerator.cpp
                                      ../DefaultInputGenerator.cpp
                                      ../PlatoAnalyzeInputDeckWriter.cpp
                                      ../SalinasInputDeckWriter.cpp)

SET(PlatoXMLGenerator_UnitTester_HDRS XMLGenerator_UnitTester.hpp)

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
