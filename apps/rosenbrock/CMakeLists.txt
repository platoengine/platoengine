###############################################################################
# Sources:
###############################################################################
SET(ExeName RosenbrockProxy)

SET(${ExeName}_SRCS RosenbrockMain.cpp)
SET(${ExeName}_HDRS Plato_RosenbrockApp.hpp)

INCLUDE_DIRECTORIES(${PLATO_INCLUDES})

# actual target:
set(${ExeName}_LIBS ${PLATO_LIBRARIES} ${PLATO_LIBRARIES} ${NLOPT_LIBRARY} 
    ${Trilinos_LIBRARIES} ${Trilinos_TPL_LIBRARIES} ${PYTHON_LIBRARY} 
    ${PYTHON_LINK_LIBS} ${Plato_EXTRA_LINK_FLAGS})
add_executable(${ExeName} ${${ExeName}_SRCS})
target_link_libraries(${ExeName} ${${ExeName}_LIBS})
set(ROSENBROCKPROXY_BINARY ${CMAKE_BINARY_DIR}/apps/rosenbrock/${ExeName} PARENT_SCOPE)

if( CMAKE_INSTALL_PREFIX )
  install( TARGETS ${ExeName} DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
endif()
###############################################################################
###############################################################################
            