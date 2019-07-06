set(LIB_NAMES)

set(LIB_NAME Operations)
set(LIB_NAMES ${LIB_NAMES} ${LIB_NAME})
set(${LIB_NAME}_SOURCES Plato_Filter.cpp
                        Plato_Roughness.cpp
                        Plato_Aggregator.cpp
                        Plato_DesignVolume.cpp
                        Plato_EnforceBounds.cpp
                        Plato_UpdateProblem.cpp
                        Plato_ComputeVolume.cpp
                        Plato_SetUpperBounds.cpp
                        Plato_SetLowerBounds.cpp
                        Plato_PlatoMainOutput.cpp
                        Plato_InitializeField.cpp
                        Plato_InitializeValues.cpp
                        Plato_WriteGlobalValue.cpp
                        Plato_OperationsUtilities.cpp
                        Plato_MeanPlusVarianceMeasure.cpp
                        Plato_NormalizeObjectiveValue.cpp
                        Plato_ReciprocateObjectiveValue.cpp
                        Plato_NormalizeObjectiveGradient.cpp
                        Plato_ReciprocateObjectiveGradient.cpp
                        )
                        
set(${LIB_NAME}_HEADERS Plato_Filter.hpp
                        Plato_Roughness.hpp
                        Plato_Aggregator.hpp
                        Plato_DesignVolume.hpp
                        Plato_EnforceBounds.hpp
                        Plato_UpdateProblem.hpp
                        Plato_ComputeVolume.hpp
                        Plato_SetUpperBounds.hpp
                        Plato_SetLowerBounds.hpp
                        Plato_LocalOperation.hpp
                        Plato_PlatoMainOutput.hpp
                        Plato_InitializeField.hpp
                        Plato_InitializeValues.hpp
                        Plato_WriteGlobalValue.hpp
                        Plato_OperationsUtilities.hpp
                        Plato_MeanPlusVarianceMeasure.hpp
                        Plato_NormalizeObjectiveValue.hpp
                        Plato_NormalizeObjectiveGradient.hpp
                        Plato_ReciprocateObjectiveGradient.hpp
                        )
                        
if( GEOMETRY )
                        
set(${LIB_NAME}_HEADERS ${${LIB_NAME}_HEADERS}
                        Plato_MapMLSField.hpp
                        Plato_MetaDataMLS.hpp
                        Plato_ComputeMLSField.hpp
                        Plato_InitializeMLSPoints.hpp
                        )
                        
endif()

INCLUDE_DIRECTORIES(${PLATO_INCLUDES} ${CMAKE_SOURCE_DIR}/apps/services)

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
