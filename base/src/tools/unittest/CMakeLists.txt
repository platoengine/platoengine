###############################################################################
# Sources:
###############################################################################
SET(PlatoInputData_UnitTester_SRCS UnitMain.cpp
                                   Plato_Test_InputData.cpp)

SET(PlatoInputData_UnitTester_HDRS )

set(CMAKE_CXX_FLAGS_DEBUG "-Wall -g -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -g -O3 -std=c++11")

if(CMAKE_COMPILER_IS_GNUCC)
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   -Wno-unused-variable -Wno-write-strings")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wno-unused-variable -Wno-write-strings")
endif()

set(PLATOUNIT_INCLUDES ${CMAKE_SOURCE_DIR}/base/src/tools)
INCLUDE_DIRECTORIES(${PLATOUNIT_INCLUDES})

set(PlatoInputData_UnitTester_LIBS ${GTEST_LIBRARY} PlatoParser PlatoExceptions ${Trilinos_LIBRARIES} ${Trilinos_TPL_LIBRARIES} )
add_executable(PlatoInputData_UnitTester ${PlatoInputData_UnitTester_SRCS})
target_link_libraries(PlatoInputData_UnitTester ${PlatoInputData_UnitTester_LIBS})

if( CMAKE_INSTALL_PREFIX )
  install( TARGETS PlatoInputData_UnitTester DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
endif()
