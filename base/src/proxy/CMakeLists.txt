set(LIB_NAMES)

set(LIB_NAME Proxy)
set(LIB_NAMES ${LIB_NAMES} ${LIB_NAME})
set(${LIB_NAME}_SOURCES Plato_RocketDesignApp.cpp
                        Plato_GradFreeSimpleRocketObjective.cpp
                        Plato_StructuralTopologyOptimization.cpp
                        Plato_StructuralTopologyOptimizationProxyApp.cpp
                        )

set(${LIB_NAME}_HEADERS Plato_ProxyVolume.hpp
						Plato_Cylinder.hpp
						Plato_GeometryModel.hpp
						Plato_ProxyCompliance.hpp
						Plato_RocketDesignApp.hpp
						Plato_AlgebraicRocketModel.hpp
						Plato_SimpleRocketObjective.hpp
						Plato_GradFreeSimpleRocketObjective.hpp
						Plato_StructuralTopologyOptimization.hpp
						Plato_StructuralTopologyOptimizationProxyApp.hpp
						)

INCLUDE_DIRECTORIES(${PLATO_INCLUDES})

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
