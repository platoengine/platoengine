###############################################################################
# Sources:
###############################################################################
SET(ExeName TopoProxy)

SET(${ExeName}_SRCS StructuralTopologyOptimizationProxyMain.cpp)
SET(${ExeName}_HDRS )

INCLUDE_DIRECTORIES(${PLATO_INCLUDES})

# actual target:
set(${ExeName}_LIBS ${PLATO_LIBRARIES} ${PLATO_LIBRARIES} ${Trilinos_LIBRARIES} 
    ${Trilinos_TPL_LIBRARIES} ${Plato_EXTRA_LINK_FLAGS})
add_executable(${ExeName} ${${ExeName}_SRCS})
target_link_libraries(${ExeName} ${${ExeName}_LIBS})
set(TOPOPROXY_BINARY ${CMAKE_BINARY_DIR}/apps/proxy/${ExeName} PARENT_SCOPE)

if( CMAKE_INSTALL_PREFIX )
  install( TARGETS ${ExeName} DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
endif()
###############################################################################
###############################################################################
            