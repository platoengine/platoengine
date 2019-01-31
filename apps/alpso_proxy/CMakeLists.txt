###############################################################################
# Sources:
###############################################################################
SET(ExeName ProxyALPSO)

SET(${ExeName}_SRCS ProxyMainALPSO.cpp)
SET(${ExeName}_HDRS Plato_ProxyAppALPSO.hpp)

INCLUDE_DIRECTORIES(${PLATO_INCLUDES})

# actual target:
set(${ExeName}_LIBS ${PLATO_LIBRARIES} ${PLATO_LIBRARIES}
    ${Trilinos_LIBRARIES} ${Trilinos_TPL_LIBRARIES}
    ${Plato_EXTRA_LINK_FLAGS})
add_executable(${ExeName} ${${ExeName}_SRCS})
target_link_libraries(${ExeName} ${${ExeName}_LIBS})
set(PROXYALPSO_BINARY ${CMAKE_BINARY_DIR}/apps/alpso_proxy/${ExeName} PARENT_SCOPE)

if( CMAKE_INSTALL_PREFIX )
  install( TARGETS ${ExeName} DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
endif()
###############################################################################
###############################################################################
            