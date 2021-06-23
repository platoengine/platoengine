###############################################################################
# Sources:
###############################################################################
SET(PlatoXTK_UnitTester_SRCS UnitMain.cpp
                             Plato_Test_XTK_Application.cpp)

SET(PlatoXTK_UnitTester_HDRS )

if(CMAKE_COMPILER_IS_GNUCC)
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   -Wno-unused-variable -Wno-write-strings")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wno-unused-variable -Wno-write-strings")
endif()

set(PLATOUNIT_INCLUDES ${CMAKE_SOURCE_DIR}/base/src/xtk
                       ${CMAKE_SOURCE_DIR}/base/src/tools)
                       
INCLUDE_DIRECTORIES(${PLATOUNIT_INCLUDES})


set(PlatoXTK_UnitTester_LIBS 
    ${GTEST_BOTH_LIBRARIES} 
    PlatoXTK
    ${XTK_Libs} 
    PlatoParser 
    PlatoExceptions
    ${Trilinos_LIBRARIES}
    ${Trilinos_TPL_LIBRARIES} )
         
add_executable(PlatoXTK_UnitTester ${PlatoXTK_UnitTester_SRCS})
target_link_libraries(PlatoXTK_UnitTester ${PlatoXTK_UnitTester_LIBS})
add_test(NAME PlatoXTK_UnitTester COMMAND ${CMAKE_BINARY_DIR}/base/src/xtk/unittest/PlatoXTK_UnitTester)

if( CMAKE_INSTALL_PREFIX )
  install( TARGETS PlatoXTK_UnitTester DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
endif()
