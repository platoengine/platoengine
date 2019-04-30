set(LIB_NAMES)

set(LIB_NAME PlatoKernelLibraryInterface)
set(LIB_NAMES ${LIB_NAMES} ${LIB_NAME})
set(${LIB_NAME}_SOURCES PSL_InterfaceToEngine_ParameterDataBuilder.cpp
	PSL_InterfaceToEngine_OptimizationMesh.cpp)
set(${LIB_NAME}_HEADERS PSL_InterfaceToEngine_ParameterDataBuilder.hpp
	PSL_InterfaceToEngine_OptimizationMesh.hpp)

INCLUDE_DIRECTORIES(${PLATO_INCLUDES})
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/AbstractInterface)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/DiscreteGlobalOptimization)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Filter)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Geometry)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Helper)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/ParameterData)

FOREACH(PLATO_LIB ${LIB_NAMES})
  add_library(Plato${PLATO_LIB}     ${${PLATO_LIB}_SOURCES}     ${${PLATO_LIB}_HEADERS}     )
  set(ADD_PLATO_LIBRARIES ${ADD_PLATO_LIBRARIES} Plato${PLATO_LIB})
ENDFOREACH()
set(PLATO_LIBRARIES ${PLATO_LIBRARIES} ${ADD_PLATO_LIBRARIES} PARENT_SCOPE)

if( CMAKE_INSTALL_PREFIX )
 FOREACH(PLATO_LIB ${LIB_NAMES})
  install( TARGETS Plato${PLATO_LIB} EXPORT Plato${PLATO_LIB}
           LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
           ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
           PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
  install( FILES ${${PLATO_LIB}_HEADERS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
 ENDFOREACH()
endif()
