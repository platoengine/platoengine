###############################################################################
# Sources:
###############################################################################
SET(PlatoInputData_UnitTester_SRCS UnitMain.cpp
                                   Plato_Test_InputData.cpp)

SET(PlatoInputData_UnitTester_HDRS )

if(CMAKE_COMPILER_IS_GNUCC)
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   -Wno-unused-variable -Wno-write-strings")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wno-unused-variable -Wno-write-strings")
endif()

set(PLATOUNIT_INCLUDES ${CMAKE_SOURCE_DIR}/base/src/tools)
INCLUDE_DIRECTORIES(${PLATOUNIT_INCLUDES})

set(PlatoInputData_UnitTester_LIBS ${GTEST_BOTH_LIBRARIES} PlatoParser PlatoExceptions ${Trilinos_LIBRARIES} ${Trilinos_TPL_LIBRARIES} )
add_executable(PlatoInputData_UnitTester ${PlatoInputData_UnitTester_SRCS})
target_link_libraries(PlatoInputData_UnitTester ${PlatoInputData_UnitTester_LIBS})
add_test(NAME PlatoInputData_UnitTester COMMAND ${CMAKE_BINARY_DIR}/base/src/tools/unittest/PlatoInputData_UnitTester)

if( CMAKE_INSTALL_PREFIX )
  install( TARGETS PlatoInputData_UnitTester DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
endif()
