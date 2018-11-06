###############################################################################
# Sources:
###############################################################################
SET(ExeName KsalProxy)

SET(${ExeName}_SRCS KsalProxyMain.cpp)
SET(${ExeName}_HDRS Plato_KsalProxyApp.hpp)

INCLUDE_DIRECTORIES(${PLATO_INCLUDES})

# actual target:
set(${ExeName}_LIBS ${PLATO_LIBRARIES} ${PLATO_LIBRARIES}
    ${Trilinos_LIBRARIES} ${Trilinos_TPL_LIBRARIES}
    ${Plato_EXTRA_LINK_FLAGS})
add_executable(${ExeName} ${${ExeName}_SRCS})
target_link_libraries(${ExeName} ${${ExeName}_LIBS})
set(KSALPROXY_BINARY ${CMAKE_BINARY_DIR}/apps/ksal_proxy/${ExeName} PARENT_SCOPE)

if( CMAKE_INSTALL_PREFIX )
  install( TARGETS ${ExeName} DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
endif()
###############################################################################
###############################################################################
            