###############################################################################
# Sources:
###############################################################################
SET(PlatoSu2ToExodus_UnitTester_SRCS Su2ToExodus_UnitMain.cpp
                                      Plato_Test_Su2ToExodus.cpp
                                      Su2ToExodus_UnitTester.cpp
                                      ../Su2ToExodus.cpp)

#SET(PlatoXMLGenerator_UnitTester_HDRS XMLGenerator_UnitTester.hpp)

if(CMAKE_COMPILER_IS_GNUCC)
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   -Wno-unused-variable -Wno-write-strings")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wno-unused-variable -Wno-write-strings")
endif()

set(PLATOUNIT_INCLUDES ${CMAKE_SOURCE_DIR}/base/src/tools ..)
INCLUDE_DIRECTORIES(${PLATOUNIT_INCLUDES})


set(PlatoSu2ToExodus_UnitTester_LIBS ${GTEST_BOTH_LIBRARIES})
add_executable(PlatoSu2ToExodus_UnitTester ${PlatoSu2ToExodus_UnitTester_SRCS})
target_link_libraries(PlatoSu2ToExodus_UnitTester ${PlatoSu2ToExodus_UnitTester_LIBS} ${Trilinos_LIBRARIES} ${Trilinos_TPL_LIBRARIES})
add_test(NAME PlatoSu2ToExodus_UnitTester COMMAND ${CMAKE_BINARY_DIR}/base/src/Su2ToExodus/unittest/PlatoSu2ToExodus_UnitTester)


if( CMAKE_INSTALL_PREFIX )
  install( TARGETS PlatoSu2ToExodus_UnitTester DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
endif()
