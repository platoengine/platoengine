###############################################################################
# Sources:
###############################################################################
SET(Plato_Operations_UnitTester_SRCS ../Plato_InitializeValues.cpp
                                        Plato_Test_Operations.cpp
                                        Operations_UnitMain.cpp)

#SET(Plato_Operations_UnitTester_HDRS XMLGenerator_UnitTester.hpp)

if(CMAKE_COMPILER_IS_GNUCC)
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   -Wno-unused-variable -Wno-write-strings")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wno-unused-variable -Wno-write-strings")
endif()

set(PLATOUNIT_INCLUDES ${CMAKE_SOURCE_DIR}/base/src/tools ..)
INCLUDE_DIRECTORIES(${PLATOUNIT_INCLUDES})

# The order of these libraries and the duplicates are necessary
set(Plato_Operations_UnitTester_LIBS ${GTEST_LIBRARY} 
                                     PlatoApp
                                     ${PLATO_LIBRARIES}
                                     ${Trilinos_LIBRARIES})
 
add_executable(Plato_Operations_UnitTester ${Plato_Operations_UnitTester_SRCS})
target_link_libraries(Plato_Operations_UnitTester ${Plato_Operations_UnitTester_LIBS} ${Plato_Operations_UnitTester_LIBS})
add_test(NAME Plato_Operations_UnitTester COMMAND ${CMAKE_BINARY_DIR}/base/src/operations/unittest/Plato_Operations_UnitTester)


if( CMAKE_INSTALL_PREFIX )
  install( TARGETS Plato_Operations_UnitTester DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
endif()
