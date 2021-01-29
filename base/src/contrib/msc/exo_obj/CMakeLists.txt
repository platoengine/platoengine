include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_definitions(
  -fno-math-errno
)

###################################################################
## Exodus setup                                                  ##
###################################################################
Plato_find_lib( EXODUS_LIB ON exodus ${SEACAS_PATH}/lib )

SET(CMAKE_CXX_COMPILER ${MPI_CXX_COMPILER})

ADD_EXECUTABLE(exo2obj src/exo-to-objs-tets-only.cpp)

TARGET_LINK_LIBRARIES(exo2obj ${EXODUS_LIB})

if( CMAKE_INSTALL_PREFIX )
  install( TARGETS exo2obj DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
endif()

