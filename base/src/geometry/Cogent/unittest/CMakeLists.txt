###############################################################################
# Sources:
###############################################################################
SET(PlatoGeometryCogent_UnitTester_SRCS 
  UnitTest.cpp
  Test_ParameterFunction.cpp
  Test_BodyReduced.cpp
  Test_BodyConformal.cpp
  Test_ParameterizedModel.cpp
  Test_Dicer.cpp
)

SET(PlatoGeometryCogent_UnitTester_HDRS )

set(CMAKE_CXX_FLAGS_DEBUG "-Wall -g -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -g -O3 -std=c++11")

if(CMAKE_COMPILER_IS_GNUCC)
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   -Wno-unused-variable -Wno-write-strings")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wno-unused-variable -Wno-write-strings")
endif()

set(PLATOUNIT_INCLUDES ${CMAKE_SOURCE_DIR}/base/src/geometry/Cogent/core
                       ${CMAKE_SOURCE_DIR}/base/src/geometry/Cogent/geometry)
INCLUDE_DIRECTORIES(${PLATOUNIT_INCLUDES})

set(PlatoGeometryCogent_UnitTester_LIBS ${GTEST_LIBRARY} PlatoGeometryCogent ${Trilinos_LIBRARIES} ${Trilinos_TPL_LIBRARIES} )
add_executable(PlatoGeometryCogent_UnitTester ${PlatoGeometryCogent_UnitTester_SRCS})
target_link_libraries(PlatoGeometryCogent_UnitTester ${PlatoGeometryCogent_UnitTester_LIBS})
add_test(NAME PlatoGeometryCogent_UnitTester COMMAND ${CMAKE_BINARY_DIR}/base/src/geometry/MovingLeastSquares/unittest/PlatoGeometryCogent_UnitTester)

if( CMAKE_INSTALL_PREFIX )
  install( TARGETS PlatoGeometryCogent_UnitTester DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
endif()
