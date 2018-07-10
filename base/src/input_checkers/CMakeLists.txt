###############################################################################
# Sources:
###############################################################################
SET(InputXMLChecker_SRCS InputXMLChecker_Main.cpp InputXMLChecker.cpp)
SET(InputXMLChecker_HDRS InputXMLChecker.hpp)

INCLUDE_DIRECTORIES(${PLATO_INCLUDES})

# actual target:
set(PLATOMAIN_LIBS ${PLATO_LIBRARIES} ${PLATO_LIBRARIES} ${PLATO_LIBRARIES} ${NLOPT_LIBRARY} 
    ${Trilinos_LIBRARIES} ${Trilinos_TPL_LIBRARIES} ${PYTHON_LIBRARY} 
    ${PYTHON_LINK_LIBS} ${Plato_EXTRA_LINK_FLAGS})
add_executable(InputXMLChecker ${InputXMLChecker_SRCS})
target_link_libraries(InputXMLChecker ${PLATOMAIN_LIBS})

if( CMAKE_INSTALL_PREFIX )
  install( TARGETS InputXMLChecker DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
endif()
###############################################################################
###############################################################################

