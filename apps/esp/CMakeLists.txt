###############################################################################
# Sources:
###############################################################################
SET(PlatoESP_SRCS Plato_ESP.cpp PlatoESPApp.cpp)

SET(PlatoESP_HDRS PlatoESPApp.hpp)

include_directories(${PLATO_INCLUDES})

# actual target:
set(PLATOESP_LIBS  ${PLATO_LIBRARIES} ${PLATO_LIBRARIES}  
     ${Trilinos_LIBRARIES} ${Trilinos_TPL_LIBRARIES}
     ${Plato_EXTRA_LINK_FLAGS})
add_executable(PlatoESP ${PlatoESP_SRCS})
target_link_libraries(PlatoESP ${PLATOESP_LIBS})
set(PLATOESP_BINARY ${CMAKE_BINARY_DIR}/apps/esp/PlatoESP PARENT_SCOPE)

if( CMAKE_INSTALL_PREFIX )
  install( TARGETS PlatoESP DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
endif()

if (PLATO_ENABLE_SERVICES_PYTHON)
    find_package(Python2 COMPONENTS Development Interpreter)
    add_library( PythonPlatoESP SHARED PlatoESPApp.cpp PlatoESPPythonModule.cpp )
    target_include_directories(PythonPlatoESP SYSTEM PRIVATE ${Python2_INCLUDE_DIRS} )
    target_link_libraries( PythonPlatoESP ${PLATO_LIBRARIES} ${PLATO_LIBRARIES} ${Trilinos_LIBRARIES}
                           ${Trilinos_TPL_LIBRARIES} ${Python2_LIBRARIES} )
    set_target_properties( PythonPlatoESP PROPERTIES PREFIX "" )
    target_compile_options( PythonPlatoESP PRIVATE "-lmpi" )

    if( CMAKE_INSTALL_PREFIX )
      install( TARGETS PythonPlatoESP DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
    endif()

endif()
###############################################################################
###############################################################################
